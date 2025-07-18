/**
 * @file GpioManager.cpp
 * @brief Advanced GPIO management system implementation for the HardFOC platform.
 * 
 * @details This file implements the comprehensive GPIO management system that integrates
 *          with the platform mapping system to automatically manage GPIOs from multiple
 *          hardware sources (ESP32-C6, PCAL95555, TMC9660) based on functional pin
 *          identifiers and hardware chip mappings.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 */

#include "GpioManager.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/McuDigitalGpio.h"

#include <algorithm>
#include <sstream>
#include <iomanip>

static const char* TAG = "GpioManager";

//==============================================================================
// SINGLETON IMPLEMENTATION
//==============================================================================

GpioManager& GpioManager::GetInstance() noexcept {
    static GpioManager instance;
    return instance;
}

//==============================================================================
// LIFECYCLE METHODS
//==============================================================================

Result<void> GpioManager::Initialize(SfI2cBus& i2cBus, Tmc9660MotorController& tmc9660Controller) noexcept {
    console_info(TAG, "Initializing advanced GPIO management system with platform mapping integration");
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (is_initialized_.load()) {
        console_info(TAG, "GPIO manager already initialized");
        return HARDFOC_SUCCESS();
    }
    
    // Store hardware interface references
    i2c_bus_ = &i2cBus;
    tmc9660_controller_ = &tmc9660Controller;
    
    // Initialize system start time
    system_start_time_.store(os_get_time_msec());
    
    // Initialize hardware subsystems
    auto esp32_result = InitializeEsp32Gpio();
    if (esp32_result.IsError()) {
        console_error(TAG, "Failed to initialize ESP32 GPIO: %s", esp32_result.GetDescription().data());
        return esp32_result;
    }
    
    auto pcal_result = InitializePcal95555Handler();
    if (pcal_result.IsError()) {
        console_error(TAG, "Failed to initialize PCAL95555 GPIO wrapper: %s", pcal_result.GetDescription().data());
        return pcal_result;
    }
    
    auto tmc_result = InitializeTmc9660Gpio();
    if (tmc_result.IsError()) {
        console_error(TAG, "Failed to initialize TMC9660 GPIO: %s", tmc_result.GetDescription().data());
        return tmc_result;
    }
    
    // Register all available pins from platform mapping
    auto register_result = RegisterAllPlatformPins();
    if (register_result.IsError()) {
        console_error(TAG, "Failed to register platform pins: %s", register_result.GetDescription().data());
        return register_result;
    }
    
    is_initialized_.store(true);
    
    console_info(TAG, "Advanced GPIO management system initialized successfully");
    console_info(TAG, "Registered %zu pins from platform mapping", pin_registry_.size());
    
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::Shutdown() noexcept {
    console_info(TAG, "Shutting down GPIO management system");
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_info(TAG, "GPIO manager not initialized");
        return HARDFOC_SUCCESS();
    }
    
    // Deinitialize all pins
    for (auto& [pin, info] : pin_registry_) {
        if (info && info->gpio_driver) {
            info->gpio_driver->Deinitialize();
        }
    }
    
    // Clear pin registry
    pin_registry_.clear();
    
    // Reset hardware interfaces
    pcal95555_handler_.reset();
    tmc9660_controller_ = nullptr;
    i2c_bus_ = nullptr;
    
    // Reset statistics
    total_operations_.store(0);
    successful_operations_.store(0);
    failed_operations_.store(0);
    communication_errors_.store(0);
    hardware_errors_.store(0);
    system_start_time_.store(0);
    
    // Clear error messages
    {
        std::lock_guard<std::mutex> error_lock(error_mutex_);
        recent_errors_.clear();
    }
    
    is_initialized_.store(false);
    
    console_info(TAG, "GPIO management system shutdown completed");
    return HARDFOC_SUCCESS();
}

bool GpioManager::IsInitialized() const noexcept {
    return is_initialized_.load();
}

//==============================================================================
// PLATFORM MAPPING INTEGRATION
//==============================================================================

bool GpioManager::IsPinAvailable(HardFOC::FunctionalGpioPin pin) const noexcept {
    return HardFOC::GpioPlatformMapping::isPinAvailable(pin);
}

Result<const HardFOC::GpioHardwareResource*> GpioManager::GetPinHardwareResource(
    HardFOC::FunctionalGpioPin pin) const noexcept {
    
    const auto* resource = HardFOC::GpioPlatformMapping::getHardwareResource(pin);
    if (!resource) {
        return Result<const HardFOC::GpioHardwareResource*>::Error(
            ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
            "Functional pin not available on this platform");
    }
    
    return Result<const HardFOC::GpioHardwareResource*>(resource);
}

Result<void> GpioManager::RegisterAllPlatformPins() noexcept {
    console_info(TAG, "Registering all available pins from platform mapping");
    
    size_t registered_count = 0;
    size_t failed_count = 0;
    
    // Iterate through all functional pins
    for (uint8_t i = 0; i < static_cast<uint8_t>(HardFOC::FunctionalGpioPin::FUNCTIONAL_GPIO_PIN_COUNT); ++i) {
        auto functional_pin = static_cast<HardFOC::FunctionalGpioPin>(i);
        
        // Check if pin is available on this platform
        if (!IsPinAvailable(functional_pin)) {
            console_debug(TAG, "Functional pin %d not available on this platform", i);
            continue;
        }
        
        // Get hardware resource information
        auto resource_result = GetPinHardwareResource(functional_pin);
        if (resource_result.IsError()) {
            console_error(TAG, "Failed to get hardware resource for pin %d: %s", 
                         i, resource_result.GetDescription().data());
            failed_count++;
            continue;
        }
        
        const auto* resource = resource_result.GetValue();
        
        // Validate hardware resource
        auto validate_result = ValidateHardwareResource(resource);
        if (validate_result.IsError()) {
            console_error(TAG, "Hardware resource validation failed for pin %d: %s", 
                         i, validate_result.GetDescription().data());
            failed_count++;
            continue;
        }
        
        // Check for conflicts
        auto conflict_result = CheckHardwareConflicts(resource);
        if (conflict_result.IsError()) {
            console_error(TAG, "Hardware conflict detected for pin %d: %s", 
                         i, conflict_result.GetDescription().data());
            failed_count++;
            continue;
        }
        
        // Determine initial direction based on hardware resource
        bool is_input = (resource->has_pullup || resource->has_pulldown);
        
        // Register the pin
        auto register_result = RegisterPin(functional_pin, is_input, false);
        if (register_result.IsError()) {
            console_error(TAG, "Failed to register pin %d: %s", 
                         i, register_result.GetDescription().data());
            failed_count++;
            continue;
        }
        
        registered_count++;
        console_debug(TAG, "Registered functional pin %d (chip: %d, pin: %d)", 
                     i, static_cast<int>(resource->chip_id), resource->pin_id);
    }
    
    console_info(TAG, "Platform pin registration completed: %zu registered, %zu failed", 
                registered_count, failed_count);
    
    if (failed_count > 0) {
        return Result<void>::Error(ResultCode::ERROR_OPERATION_FAILED,
                                  "Some pins failed to register");
    }
    
    return HARDFOC_SUCCESS();
}

std::vector<HardFOC::FunctionalGpioPin> GpioManager::GetAvailablePins() const noexcept {
    std::vector<HardFOC::FunctionalGpioPin> available_pins;
    
    for (uint8_t i = 0; i < static_cast<uint8_t>(HardFOC::FunctionalGpioPin::FUNCTIONAL_GPIO_PIN_COUNT); ++i) {
        auto functional_pin = static_cast<HardFOC::FunctionalGpioPin>(i);
        if (IsPinAvailable(functional_pin)) {
            available_pins.push_back(functional_pin);
        }
    }
    
    return available_pins;
}

//==============================================================================
// PIN REGISTRATION AND MANAGEMENT
//==============================================================================

Result<void> GpioManager::RegisterPin(HardFOC::FunctionalGpioPin pin, bool direction, 
                                     bool initial_state) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    // Check if pin is already registered
    if (pin_registry_.find(pin) != pin_registry_.end()) {
        return Result<void>::Error(ResultCode::ERROR_ALREADY_EXISTS,
                                  "Pin already registered");
    }
    
    // Get hardware resource information
    auto resource_result = GetPinHardwareResource(pin);
    if (resource_result.IsError()) {
        return resource_result.ConvertError<void>();
    }
    
    const auto* resource = resource_result.GetValue();
    
    // Create GPIO driver
    auto driver_result = CreateGpioDriver(pin, direction);
    if (driver_result.IsError()) {
        return driver_result.ConvertError<void>();
    }
    
    auto driver = driver_result.GetValue();
    
    // Initialize the driver
    if (!driver->EnsureInitialized()) {
        return Result<void>::Error(ResultCode::ERROR_INIT_FAILED,
                                  "Failed to initialize GPIO driver");
    }
    
    // Set initial state for output pins
    if (!direction && initial_state) {
        auto set_result = driver->SetActive();
        if (set_result != hf_gpio_err_t::GPIO_SUCCESS) {
            return Result<void>::Error(ResultCode::ERROR_OPERATION_FAILED,
                                      "Failed to set initial pin state");
        }
    }
    
    // Create pin info
    auto pin_name = GetPinName(pin);
    auto pin_info = std::make_unique<GpioInfo>(pin, std::move(driver), pin_name, 
                                              resource->chip_id, resource->pin_id);
    
    // Store in registry
    pin_registry_[pin] = std::move(pin_info);
    
    console_debug(TAG, "Registered pin %s (chip: %d, pin: %d, direction: %s)", 
                 pin_name.c_str(), static_cast<int>(resource->chip_id), 
                 resource->pin_id, direction ? "input" : "output");
    
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::UnregisterPin(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto it = pin_registry_.find(pin);
    if (it == pin_registry_.end()) {
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    // Deinitialize the driver
    if (it->second && it->second->gpio_driver) {
        it->second->gpio_driver->Deinitialize();
    }
    
    // Remove from registry
    pin_registry_.erase(it);
    
    console_debug(TAG, "Unregistered pin %s", GetPinName(pin).c_str());
    
    return HARDFOC_SUCCESS();
}

bool GpioManager::IsPinRegistered(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return pin_registry_.find(pin) != pin_registry_.end();
}

Result<const GpioInfo*> GpioManager::GetPinInfo(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<const GpioInfo*>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                             "GPIO manager not initialized");
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return Result<const GpioInfo*>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                             "Pin not registered");
    }
    
    return Result<const GpioInfo*>(info);
}

size_t GpioManager::GetRegisteredPinCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return pin_registry_.size();
}

std::vector<HardFOC::FunctionalGpioPin> GpioManager::GetRegisteredPins() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<HardFOC::FunctionalGpioPin> pins;
    pins.reserve(pin_registry_.size());
    
    for (const auto& [pin, info] : pin_registry_) {
        pins.push_back(pin);
    }
    
    return pins;
}

//==============================================================================
// BASIC PIN OPERATIONS
//==============================================================================

Result<void> GpioManager::SetActive(HardFOC::FunctionalGpioPin pin) noexcept {
    return SetState(pin, true);
}

Result<void> GpioManager::SetInactive(HardFOC::FunctionalGpioPin pin) noexcept {
    return SetState(pin, false);
}

Result<void> GpioManager::SetState(HardFOC::FunctionalGpioPin pin, bool state) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    if (info->is_input) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_INVALID_OPERATION,
                                  "Cannot set state of input pin");
    }
    
    // Perform the GPIO operation
    hf_gpio_err_t result;
    if (state) {
        result = info->gpio_driver->SetActive();
    } else {
        result = info->gpio_driver->SetInactive();
    }
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        info->current_state = state;
        info->access_count++;
        info->last_access_time = os_get_time_msec();
        UpdateStatistics(true);
        
        console_debug(TAG, "Set pin %s to %s", info->name.data(), state ? "active" : "inactive");
        return HARDFOC_SUCCESS();
    } else {
        info->error_count++;
        UpdateStatistics(false);
        
        std::string error_msg = "Failed to set pin state: " + std::string(HfGpioErrToString(result));
        AddErrorMessage(error_msg);
        
        return Result<void>::Error(ResultCode::ERROR_GPIO_WRITE_FAILED, error_msg);
    }
}

Result<bool> GpioManager::Toggle(HardFOC::FunctionalGpioPin pin) noexcept {
    // First read current state
    auto current_result = ReadState(pin);
    if (current_result.IsError()) {
        return Result<bool>(current_result.GetResult());
    }
    
    // Toggle the state
    bool new_state = !current_result.GetValue();
    auto set_result = SetState(pin, new_state);
    if (set_result.IsError()) {
        return Result<bool>(set_result.GetResult());
    }
    
    return Result<bool>(new_state);
}

Result<bool> GpioManager::ReadState(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<bool>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<bool>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    // Perform the GPIO read operation
    bool state = false;
    auto result = info->gpio_driver->IsActive(state);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        info->current_state = state;
        info->access_count++;
        info->last_access_time = os_get_time_msec();
        UpdateStatistics(true);
        
        console_debug(TAG, "Read pin %s state: %s", info->name.data(), 
                     state ? "active" : "inactive");
        return Result<bool>(state);
    } else {
        info->error_count++;
        UpdateStatistics(false);
        
        std::string error_msg = "Failed to read pin state: " + std::string(HfGpioErrToString(result));
        AddErrorMessage(error_msg);
        
        return Result<bool>::Error(ResultCode::ERROR_GPIO_READ_FAILED, error_msg);
    }
}

Result<bool> GpioManager::IsActive(HardFOC::FunctionalGpioPin pin) noexcept {
    return ReadState(pin);
}

//==============================================================================
// BATCH OPERATIONS
//==============================================================================

Result<GpioBatchResult> GpioManager::BatchWrite(const GpioBatchOperation& operation) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<GpioBatchResult>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                             "GPIO manager not initialized");
    }
    
    if (!operation.is_write_operation) {
        return Result<GpioBatchResult>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                             "Operation is not a write operation");
    }
    
    if (operation.pins.size() != operation.states.size()) {
        return Result<GpioBatchResult>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                             "Pin count does not match state count");
    }
    
    GpioBatchResult result;
    result.pins = operation.pins;
    result.states = operation.states;
    result.results.reserve(operation.pins.size());
    
    bool all_successful = true;
    
    // Perform operations for each pin
    for (size_t i = 0; i < operation.pins.size(); ++i) {
        auto pin_result = SetState(operation.pins[i], operation.states[i]);
        result.results.push_back(pin_result.GetResult());
        
        if (pin_result.IsError()) {
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? ResultCode::SUCCESS : ResultCode::ERROR_OPERATION_FAILED;
    
    UpdateStatistics(all_successful);
    
    return Result<GpioBatchResult>(result);
}

Result<GpioBatchResult> GpioManager::BatchRead(const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<GpioBatchResult>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                             "GPIO manager not initialized");
    }
    
    GpioBatchResult result;
    result.pins = pins;
    result.states.reserve(pins.size());
    result.results.reserve(pins.size());
    
    bool all_successful = true;
    
    // Perform read operations for each pin
    for (const auto& pin : pins) {
        auto pin_result = ReadState(pin);
        result.results.push_back(pin_result.GetResult());
        
        if (pin_result.IsSuccess()) {
            result.states.push_back(pin_result.GetValue());
        } else {
            result.states.push_back(false); // Default value on error
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? ResultCode::SUCCESS : ResultCode::ERROR_OPERATION_FAILED;
    
    UpdateStatistics(all_successful);
    
    return Result<GpioBatchResult>(result);
}

Result<GpioBatchResult> GpioManager::SetMultipleActive(const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept {
    std::vector<bool> states(pins.size(), true);
    GpioBatchOperation operation(pins, states);
    return BatchWrite(operation);
}

Result<GpioBatchResult> GpioManager::SetMultipleInactive(const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept {
    std::vector<bool> states(pins.size(), false);
    GpioBatchOperation operation(pins, states);
    return BatchWrite(operation);
}

//==============================================================================
// PIN CONFIGURATION
//==============================================================================

Result<void> GpioManager::SetPinDirection(HardFOC::FunctionalGpioPin pin, bool direction) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    auto result = info->gpio_driver->SetDirection(
        direction ? hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT : hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT
    );
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        info->is_input = direction;
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_CONFIG_FAILED,
                                  "Failed to set pin direction");
    }
}

Result<void> GpioManager::SetPinPullMode(HardFOC::FunctionalGpioPin pin, hf_gpio_pull_mode_t pull_mode) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    auto result = info->gpio_driver->SetPullMode(pull_mode);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_CONFIG_FAILED,
                                  "Failed to set pull mode");
    }
}

Result<void> GpioManager::SetPinOutputMode(HardFOC::FunctionalGpioPin pin, hf_gpio_output_mode_t output_mode) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    auto result = info->gpio_driver->SetOutputMode(output_mode);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_CONFIG_FAILED,
                                  "Failed to set output mode");
    }
}

Result<bool> GpioManager::GetPinDirection(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<bool>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return Result<bool>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    return Result<bool>(info->is_input);
}

Result<hf_gpio_pull_mode_t> GpioManager::GetPinPullMode(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<hf_gpio_pull_mode_t>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                                 "GPIO manager not initialized");
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return Result<hf_gpio_pull_mode_t>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                                 "Pin not registered");
    }
    
    return Result<hf_gpio_pull_mode_t>(info->gpio_driver->GetPullMode());
}

Result<hf_gpio_output_mode_t> GpioManager::GetPinOutputMode(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<hf_gpio_output_mode_t>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                                   "GPIO manager not initialized");
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return Result<hf_gpio_output_mode_t>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                                   "Pin not registered");
    }
    
    return Result<hf_gpio_output_mode_t>(info->gpio_driver->GetOutputMode());
}

//==============================================================================
// INTERRUPT SUPPORT
//==============================================================================

Result<void> GpioManager::ConfigureInterrupt(HardFOC::FunctionalGpioPin pin,
                                           BaseGpio::InterruptTrigger trigger,
                                           BaseGpio::InterruptCallback callback,
                                           void* user_data) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    if (!info->gpio_driver->SupportsInterrupts()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_UNSUPPORTED_OPERATION,
                                  "Pin does not support interrupts");
    }
    
    auto result = info->gpio_driver->ConfigureInterrupt(trigger, callback, user_data);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_CONFIG_FAILED,
                                  "Failed to configure interrupt");
    }
}

Result<void> GpioManager::EnableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    auto result = info->gpio_driver->EnableInterrupt();
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_CONFIG_FAILED,
                                  "Failed to enable interrupt");
    }
}

Result<void> GpioManager::DisableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    auto result = info->gpio_driver->DisableInterrupt();
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_CONFIG_FAILED,
                                  "Failed to disable interrupt");
    }
}

bool GpioManager::SupportsInterrupts(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return false;
    }
    
    return info->gpio_driver->SupportsInterrupts();
}

//==============================================================================
// SYSTEM INFORMATION
//==============================================================================

Result<std::string> GpioManager::GetSystemHealth() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<std::string>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                         "GPIO manager not initialized");
    }
    
    std::stringstream health;
    health << "HardFOC GPIO Manager Health Report\n";
    health << "==================================\n";
    health << "Status: " << (is_initialized_.load() ? "Initialized" : "Not Initialized") << "\n";
    health << "Registered Pins: " << pin_registry_.size() << "\n";
    health << "Total Operations: " << total_operations_.load() << "\n";
    health << "Successful Operations: " << successful_operations_.load() << "\n";
    health << "Failed Operations: " << failed_operations_.load() << "\n";
    health << "Communication Errors: " << communication_errors_.load() << "\n";
    health << "Hardware Errors: " << hardware_errors_.load() << "\n";
    
    if (total_operations_.load() > 0) {
        float success_rate = (static_cast<float>(successful_operations_.load()) / 
                             static_cast<float>(total_operations_.load())) * 100.0f;
        health << "Success Rate: " << std::fixed << std::setprecision(2) << success_rate << "%\n";
    }
    
    // Pin breakdown by chip
    health << "\nPin Distribution by Chip:\n";
    std::array<uint32_t, static_cast<uint8_t>(HardFOC::HardwareChip::HARDWARE_CHIP_COUNT)> chip_counts = {};
    
    for (const auto& [pin, info] : pin_registry_) {
        if (info) {
            chip_counts[static_cast<uint8_t>(info->hardware_chip)]++;
        }
    }
    
    for (uint8_t i = 0; i < static_cast<uint8_t>(HardFOC::HardwareChip::HARDWARE_CHIP_COUNT); ++i) {
        auto chip = static_cast<HardFOC::HardwareChip>(i);
        health << "  " << GetChipName(chip) << ": " << chip_counts[i] << " pins\n";
    }
    
    return Result<std::string>(health.str());
}

Result<void> GpioManager::ResetAllPins() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    size_t reset_count = 0;
    size_t fail_count = 0;
    
    for (auto& [pin, info] : pin_registry_) {
        if (info && !info->is_input && info->gpio_driver) {
            auto result = info->gpio_driver->SetInactive();
            if (result == hf_gpio_err_t::GPIO_SUCCESS) {
                info->current_state = false;
                reset_count++;
            } else {
                fail_count++;
            }
        }
    }
    
    console_info(TAG, "Reset %zu output pins (%zu failed)", reset_count, fail_count);
    
    return fail_count == 0 ? HARDFOC_SUCCESS() : 
           Result<void>::Error(ResultCode::ERROR_OPERATION_FAILED,
                              "Some pins failed to reset");
}

Result<BaseGpio::PinStatistics> GpioManager::GetPinStatistics(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return Result<BaseGpio::PinStatistics>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                                     "GPIO manager not initialized");
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return Result<BaseGpio::PinStatistics>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                                     "Pin not registered");
    }
    
    return Result<BaseGpio::PinStatistics>(info->gpio_driver->GetStatistics());
}

Result<void> GpioManager::ClearPinStatistics(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_NOT_INITIALIZED,
                                  "GPIO manager not initialized");
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_GPIO_PIN_NOT_FOUND,
                                  "Pin not registered");
    }
    
    auto result = info->gpio_driver->ResetStatistics();
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        return Result<void>::Error(ResultCode::ERROR_OPERATION_FAILED,
                                  "Failed to clear pin statistics");
    }
}

Result<GpioSystemDiagnostics> GpioManager::GetSystemDiagnostics() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioSystemDiagnostics diagnostics;
    
    diagnostics.system_healthy = is_initialized_.load();
    diagnostics.total_pins_registered = static_cast<uint32_t>(pin_registry_.size());
    diagnostics.total_operations = total_operations_.load();
    diagnostics.successful_operations = successful_operations_.load();
    diagnostics.failed_operations = failed_operations_.load();
    diagnostics.communication_errors = communication_errors_.load();
    diagnostics.hardware_errors = hardware_errors_.load();
    diagnostics.system_uptime_ms = os_get_time_msec() - system_start_time_.load();
    
    // Calculate pins by chip
    std::fill(std::begin(diagnostics.pins_by_chip), std::end(diagnostics.pins_by_chip), 0);
    for (const auto& [pin, info] : pin_registry_) {
        if (info) {
            diagnostics.pins_by_chip[static_cast<uint8_t>(info->hardware_chip)]++;
        }
    }
    
    // Get recent error messages
    {
        std::lock_guard<std::mutex> error_lock(error_mutex_);
        diagnostics.error_messages = recent_errors_;
    }
    
    return Result<GpioSystemDiagnostics>(diagnostics);
}

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

std::string GetChipName(HardFOC::HardwareChip chip) {
    switch (chip) {
        case HardFOC::HardwareChip::ESP32_INTERNAL_GPIO:
            return "ESP32_GPIO";
        case HardFOC::HardwareChip::ESP32_INTERNAL_ADC:
            return "ESP32_ADC";
        case HardFOC::HardwareChip::PCAL95555_GPIO:
            return "PCAL95555_GPIO";
        case HardFOC::HardwareChip::TMC9660_ADC:
            return "TMC9660_ADC";
        case HardFOC::HardwareChip::TMC9660_GPIO:
            return "TMC9660_GPIO";
        default:
            return "UNKNOWN_CHIP";
    }
}

//==============================================================================
// PRIVATE IMPLEMENTATION METHODS
//==============================================================================

Result<void> GpioManager::InitializeEsp32Gpio() noexcept {
    console_info(TAG, "Initializing ESP32-C6 native GPIO system");
    // ESP32 GPIO is initialized automatically when drivers are created
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::InitializePcal95555Handler() noexcept {
    console_info(TAG, "Initializing PCAL95555 GPIO wrapper");
    
    if (!i2c_bus_) {
        return Result<void>::Error(ResultCode::ERROR_DEPENDENCY_MISSING,
                                  "I2C bus not available for PCAL95555");
    }
    
    try {
        // Create PCAL95555 wrapper using the BaseI2c interface
        pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_bus_, GetDefaultPcal95555I2cAddress());
        if (!pcal95555_handler_->IsHealthy()) {
            return Result<void>::Error(ResultCode::ERROR_INIT_FAILED,
                                      "Failed to create or initialize PCAL95555 GPIO wrapper");
        }
        
        console_info(TAG, "PCAL95555 GPIO wrapper initialized successfully");
        return HARDFOC_SUCCESS();
    } catch (const std::exception& e) {
        return Result<void>::Error(ResultCode::ERROR_INIT_FAILED,
                                  std::string("Exception during PCAL95555 initialization: ") + e.what());
    }
}

Result<void> GpioManager::InitializeTmc9660Gpio() noexcept {
    console_info(TAG, "Initializing TMC9660 GPIO system");
    
    if (!tmc9660_controller_) {
        return Result<void>::Error(ResultCode::ERROR_DEPENDENCY_MISSING,
                                  "TMC9660 controller not available");
    }
    
    // TMC9660 GPIO is managed through the motor controller
    return HARDFOC_SUCCESS();
}

Result<std::unique_ptr<BaseGpio>> GpioManager::CreateGpioDriver(
    HardFOC::FunctionalGpioPin pin, bool direction) noexcept {
    
    // Get hardware resource information
    auto resource_result = GetPinHardwareResource(pin);
    if (resource_result.IsError()) {
        return resource_result.ConvertError<std::unique_ptr<BaseGpio>>();
    }
    
    const auto* resource = resource_result.GetValue();
    
    // Create driver based on hardware chip type
    switch (resource->chip_id) {
        case HardFOC::HardwareChip::ESP32_INTERNAL_GPIO:
            return CreateEsp32GpioDriver(resource->pin_id, direction);
            
        case HardFOC::HardwareChip::PCAL95555_GPIO:
            return CreatePcal95555GpioDriver(resource->pin_id, direction);
            
        case HardFOC::HardwareChip::TMC9660_GPIO:
            return CreateTmc9660GpioDriver(resource->pin_id, direction);
            
        default:
            return Result<std::unique_ptr<BaseGpio>>::Error(ResultCode::ERROR_UNSUPPORTED_OPERATION,
                                                           "Unsupported hardware chip type");
    }
}

Result<std::unique_ptr<BaseGpio>> GpioManager::CreateEsp32GpioDriver(uint8_t pin_id, bool direction) noexcept {
    try {
        auto gpio = std::make_unique<McuDigitalGpio>(
            static_cast<gpio_num_t>(pin_id),
            direction ? hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT : hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT,
            hf_gpio_active_state_t::HF_GPIO_ACTIVE_STATE_HIGH,
            hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
            hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_NONE
        );
        
        return Result<std::unique_ptr<BaseGpio>>(std::move(gpio));
    } catch (const std::exception& e) {
        return Result<std::unique_ptr<BaseGpio>>::Error(ResultCode::ERROR_INIT_FAILED,
                                                       std::string("Failed to create ESP32 GPIO: ") + e.what());
    }
}

Result<std::unique_ptr<BaseGpio>> GpioManager::CreatePcal95555GpioDriver(uint8_t pin_id, bool direction) noexcept {
    if (!pcal95555_handler_) {
        return Result<std::unique_ptr<BaseGpio>>::Error(ResultCode::ERROR_DEPENDENCY_MISSING,
                                                       "PCAL95555 wrapper not initialized");
    }
    
    try {
        // Create pin using the wrapper
        auto chip_pin = static_cast<Pcal95555Chip1Pin>(pin_id);
        auto gpio = pcal95555_handler_->CreateGpioPin(chip_pin);
        
        if (!gpio) {
            return Result<std::unique_ptr<BaseGpio>>::Error(ResultCode::ERROR_INIT_FAILED,
                                                           "Failed to create PCAL95555 GPIO pin");
        }
        
        return Result<std::unique_ptr<BaseGpio>>(std::move(gpio));
    } catch (const std::exception& e) {
        return Result<std::unique_ptr<BaseGpio>>::Error(ResultCode::ERROR_INIT_FAILED,
                                                       std::string("Failed to create PCAL95555 GPIO: ") + e.what());
    }
}

Result<std::unique_ptr<BaseGpio>> GpioManager::CreateTmc9660GpioDriver(uint8_t pin_id, bool direction) noexcept {
    // TMC9660 GPIO implementation would go here
    // For now, return unsupported
    return Result<std::unique_ptr<BaseGpio>>::Error(ResultCode::ERROR_UNSUPPORTED_OPERATION,
                                                   "TMC9660 GPIO not yet implemented");
}

GpioInfo* GpioManager::FindGpioInfo(HardFOC::FunctionalGpioPin pin) noexcept {
    auto it = pin_registry_.find(pin);
    return (it != pin_registry_.end()) ? it->second.get() : nullptr;
}

const GpioInfo* GpioManager::FindGpioInfo(HardFOC::FunctionalGpioPin pin) const noexcept {
    auto it = pin_registry_.find(pin);
    return (it != pin_registry_.end()) ? it->second.get() : nullptr;
}

void GpioManager::UpdateStatistics(bool success) noexcept {
    total_operations_.fetch_add(1);
    if (success) {
        successful_operations_.fetch_add(1);
    } else {
        failed_operations_.fetch_add(1);
    }
}

void GpioManager::AddErrorMessage(const std::string& error_message) noexcept {
    std::lock_guard<std::mutex> lock(error_mutex_);
    
    recent_errors_.push_back(error_message);
    
    // Keep only the most recent error messages
    if (recent_errors_.size() > MAX_ERROR_MESSAGES) {
        recent_errors_.erase(recent_errors_.begin());
    }
}

std::string GpioManager::GetPinName(HardFOC::FunctionalGpioPin pin) const noexcept {
    // Convert functional pin to string representation
    switch (pin) {
        case HardFOC::FunctionalGpioPin::MOTOR_ENABLE:
            return "MOTOR_ENABLE";
        case HardFOC::FunctionalGpioPin::MOTOR_BRAKE:
            return "MOTOR_BRAKE";
        case HardFOC::FunctionalGpioPin::MOTOR_FAULT_STATUS:
            return "MOTOR_FAULT_STATUS";
        case HardFOC::FunctionalGpioPin::LED_STATUS_OK:
            return "LED_STATUS_OK";
        case HardFOC::FunctionalGpioPin::LED_STATUS_ERROR:
            return "LED_STATUS_ERROR";
        case HardFOC::FunctionalGpioPin::LED_STATUS_COMM:
            return "LED_STATUS_COMM";
        case HardFOC::FunctionalGpioPin::COMM_CAN_TX:
            return "COMM_CAN_TX";
        case HardFOC::FunctionalGpioPin::COMM_CAN_RX:
            return "COMM_CAN_RX";
        case HardFOC::FunctionalGpioPin::COMM_I2C_SDA:
            return "COMM_I2C_SDA";
        case HardFOC::FunctionalGpioPin::COMM_I2C_SCL:
            return "COMM_I2C_SCL";
        case HardFOC::FunctionalGpioPin::SPI_MOTOR_CONTROLLER_CS:
            return "SPI_MOTOR_CONTROLLER_CS";
        case HardFOC::FunctionalGpioPin::SPI_ENCODER_CS:
            return "SPI_ENCODER_CS";
        case HardFOC::FunctionalGpioPin::SPI_MISO:
            return "SPI_MISO";
        case HardFOC::FunctionalGpioPin::SPI_MOSI:
            return "SPI_MOSI";
        case HardFOC::FunctionalGpioPin::SPI_CLK:
            return "SPI_CLK";
        case HardFOC::FunctionalGpioPin::USER_OUTPUT_1:
            return "USER_OUTPUT_1";
        case HardFOC::FunctionalGpioPin::USER_OUTPUT_2:
            return "USER_OUTPUT_2";
        case HardFOC::FunctionalGpioPin::USER_INPUT_1:
            return "USER_INPUT_1";
        case HardFOC::FunctionalGpioPin::USER_INPUT_2:
            return "USER_INPUT_2";
        case HardFOC::FunctionalGpioPin::EXTERNAL_RELAY_1:
            return "EXTERNAL_RELAY_1";
        case HardFOC::FunctionalGpioPin::EXTERNAL_RELAY_2:
            return "EXTERNAL_RELAY_2";
        default:
            return "UNKNOWN_PIN";
    }
}

Result<void> GpioManager::ValidateHardwareResource(const HardFOC::GpioHardwareResource* resource) const noexcept {
    if (!resource) {
        return Result<void>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                  "Hardware resource is null");
    }
    
    // Validate chip ID
    if (static_cast<uint8_t>(resource->chip_id) >= static_cast<uint8_t>(HardFOC::HardwareChip::HARDWARE_CHIP_COUNT)) {
        return Result<void>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                  "Invalid hardware chip ID");
    }
    
    // Validate pin ID based on chip type
    switch (resource->chip_id) {
        case HardFOC::HardwareChip::ESP32_INTERNAL_GPIO:
            if (resource->pin_id >= GPIO_NUM_MAX) {
                return Result<void>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                          "Invalid ESP32 GPIO pin ID");
            }
            break;
            
        case HardFOC::HardwareChip::PCAL95555_GPIO:
            if (resource->pin_id >= 16) { // PCAL95555 has 16 pins
                return Result<void>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                          "Invalid PCAL95555 pin ID");
            }
            break;
            
        case HardFOC::HardwareChip::TMC9660_GPIO:
            if (resource->pin_id >= 2) { // TMC9660 has 2 GPIO pins
                return Result<void>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                          "Invalid TMC9660 pin ID");
            }
            break;
            
        default:
            return Result<void>::Error(ResultCode::ERROR_UNSUPPORTED_OPERATION,
                                      "Unsupported hardware chip type");
    }
    
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::CheckHardwareConflicts(const HardFOC::GpioHardwareResource* resource) const noexcept {
    if (!resource) {
        return Result<void>::Error(ResultCode::ERROR_INVALID_PARAMETER,
                                  "Hardware resource is null");
    }
    
    // Check for conflicts with already registered pins
    for (const auto& [pin, info] : pin_registry_) {
        if (info && info->hardware_chip == resource->chip_id && 
            info->hardware_pin_id == resource->pin_id) {
            return Result<void>::Error(ResultCode::ERROR_RESOURCE_CONFLICT,
                                      "Hardware resource already in use");
        }
    }
    
    return HARDFOC_SUCCESS();
}
