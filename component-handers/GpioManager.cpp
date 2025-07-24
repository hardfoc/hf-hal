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
#include "CommChannelsManager.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspGpio.h"

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

bool GpioManager::EnsureInitialized() noexcept {
    // Fast check without lock for already initialized case
    if (is_initialized_.load()) {
        return true;
    }
    
    // Need to initialize - acquire lock and check again
    RtosMutex::LockGuard lock(mutex_);
    if (is_initialized_.load()) {
        return true;
    }
    
    // Perform actual initialization
    if (Initialize()) {
        return true;
    } else {
        console_error(TAG, "GPIO Manager initialization failed");
        return false;
    }
}

bool GpioManager::Initialize() noexcept {
    console_info(TAG, "Initializing GPIO management system with platform pin registration");
    
    // Initialize system start time
    system_start_time_.store(os_get_time_msec());
    
    // Verify CommChannelsManager is available
    auto& comm_manager = CommChannelsManager::GetInstance();
    if (!comm_manager.EnsureInitialized()) {
        console_error(TAG, "CommChannelsManager not initialized - required for GPIO hardware access");
        return false;
    }
    
    // Register all platform pins for availability checking
    if (!RegisterAllPlatformPins()) {
        console_error(TAG, "Failed to register platform pins");
        return false;
    }
    
    console_info(TAG, "Registered %zu platform pins for GPIO management", GetRegisteredPinCount());
    
    // Mark as initialized - handlers will be lazy initialized when first accessed
    is_initialized_.store(true);
    console_info(TAG, "GPIO Manager initialized successfully with lazy handler loading");
    return true;
}

bool GpioManager::Shutdown() noexcept {
    console_info(TAG, "Shutting down GPIO management system");
    
    RtosMutex::LockGuard lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_info(TAG, "GPIO manager not initialized");
        return true;
    }
    
    // Clear shared pin registry with proper mutex protection
    {
        RtosMutex::LockGuard shared_lock(shared_pin_mutex_);
        shared_pin_registry_.clear();
    }
    
    // Deinitialize all pins
    for (auto& [pin, info] : pin_registry_) {
        if (info && info->gpio_driver) {
            info->gpio_driver->Deinitialize();
        }
    }
    
    // Clear pin registry
    pin_registry_.clear();
    
    // Clear shared pin registry
    {
        std::lock_guard<std::mutex> shared_lock(shared_pin_mutex_);
        shared_pin_registry_.clear();
    };
    
    // Reset hardware handlers (lazy initialized)
    pcal95555_handler_.reset();
    pcal_interrupt_pin_.reset();
    tmc9660_handler_.reset();
    
    // Reset statistics
    total_operations_.store(0);
    successful_operations_.store(0);
    failed_operations_.store(0);
    communication_errors_.store(0);
    hardware_errors_.store(0);
    system_start_time_.store(0);
    
    // Clear error messages
    {
        RtosMutex::LockGuard error_lock(error_mutex_);
        recent_errors_.clear();
    }
    
    is_initialized_.store(false);
    
    console_info(TAG, "GPIO management system shutdown completed");
    return true;
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

bool GpioManager::GetPinHardwareResource(HardFOC::FunctionalGpioPin pin, 
                                        const HardFOC::GpioHardwareResource*& resource) const noexcept {
    
    resource = HardFOC::GpioPlatformMapping::getHardwareResource(pin);
    if (!resource) {
        console_error(TAG, "Functional pin not available on this platform");
        return false;
    }
    
    return true;
}

bool GpioManager::RegisterAllPlatformPins() noexcept {
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
        const HardFOC::GpioHardwareResource* resource = nullptr;
        if (!GetPinHardwareResource(functional_pin, resource)) {
            console_error(TAG, "Failed to get hardware resource for pin %d", i);
            failed_count++;
            continue;
        }
        
        // Register the pin with simplified logic
        bool is_input = (resource->has_pullup || resource->has_pulldown);
        
        if (!RegisterPin(functional_pin, is_input, false)) {
            console_error(TAG, "Failed to register pin %d", i);
            failed_count++;
            continue;
        }
        
        registered_count++;
        console_debug(TAG, "Registered functional pin %d (chip: %d, pin: %d)", 
                     i, static_cast<int>(resource->chip_id), resource->pin_id);
    }
    
    console_info(TAG, "Platform pin registration completed: %zu registered, %zu failed", 
                registered_count, failed_count);
    
    return failed_count == 0;
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

bool GpioManager::RegisterPin(HardFOC::FunctionalGpioPin pin, bool direction, 
                              bool initial_state) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_error(TAG, "GPIO manager not initialized");
        return false;
    }
    
    // Check if pin is already registered
    if (pin_registry_.find(pin) != pin_registry_.end()) {
        console_error(TAG, "Pin already registered");
        return false;
    }
    
    // Get hardware resource information
    const HardFOC::GpioHardwareResource* resource = nullptr;
    if (!GetPinHardwareResource(pin, resource)) {
        return false;
    
    // Create GPIO pin
    std::unique_ptr<BaseGpio> driver;
    if (!CreateGpioPin(pin, direction, driver)) {
        console_error(TAG, "Failed to create GPIO pin");
        return false;
    }
    
    // Initialize the driver
    if (!driver->EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO driver");
        return false;
    }
    
    // Set initial state for output pins
    if (!direction && initial_state) {
        auto set_result = driver->SetActive();
        if (set_result != hf_gpio_err_t::GPIO_SUCCESS) {
            console_error(TAG, "Failed to set initial pin state: %s", HfGpioErrToString(set_result));
            return false;
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
    
    return true;
}

bool GpioManager::UnregisterPin(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_error(TAG, "GPIO manager not initialized");
        return false;
    }
    
    auto it = pin_registry_.find(pin);
    if (it == pin_registry_.end()) {
        console_error(TAG, "Pin not registered");
        return false;
    }
    
    // Deinitialize the driver
    if (it->second && it->second->gpio_driver) {
        it->second->gpio_driver->Deinitialize();
    }
    
    // Remove from registry
    pin_registry_.erase(it);
    
    console_debug(TAG, "Unregistered pin %s", GetPinName(pin).c_str());
    
    return true;
}

bool GpioManager::IsPinRegistered(HardFOC::FunctionalGpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return pin_registry_.find(pin) != pin_registry_.end();
}

bool GpioManager::GetPinInfo(HardFOC::FunctionalGpioPin pin, const GpioInfo*& info) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_error(TAG, "GPIO manager not initialized");
        return false;
    }
    
    info = FindGpioInfo(pin);
    if (!info) {
        console_error(TAG, "Pin not registered");
        return false;
    }
    
    return true;
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
// MODERN SHARED_PTR PIN ACCESS
//==============================================================================

std::shared_ptr<BaseGpio> GpioManager::GetPin(HfFunctionalGpioPin pin) noexcept {
    RtosMutex::LockGuard lock(shared_pin_mutex_);
    
    // Check if pin already exists in shared registry
    auto it = shared_pin_registry_.find(pin);
    if (it != shared_pin_registry_.end()) {
        return it->second;  // Return existing shared pin
    }
    
    // Return nullptr if pin not found - do NOT create new pin
    return nullptr;
}

std::shared_ptr<BaseGpio> GpioManager::CreateSharedPin(
    HfFunctionalGpioPin pin,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_pull_mode_t pull_mode) noexcept {
    
    // Ensure system is initialized
    auto init_result = EnsureInitialized();
    if (!init_result.IsSuccess()) {
        console_error(TAG, "Failed to initialize GPIO system: %s", init_result.GetErrorMessage().c_str());
        return nullptr;
    }
    
    RtosMutex::LockGuard lock(shared_pin_mutex_);
    
    // Check if pin already exists
    auto it = shared_pin_registry_.find(pin);
    if (it != shared_pin_registry_.end()) {
        return it->second;  // Return existing pin
    }
    
    // Get GPIO mapping for this functional pin (no exceptions)
    const auto* gpio_mapping = GetGpioMapping(pin);
    if (!gpio_mapping) {
        console_error(TAG, "Functional pin %d not found in GPIO mapping", static_cast<int>(pin));
        return nullptr;
    }
    
    std::unique_ptr<BaseGpio> gpio_pin;
    
    // Create pin based on chip type with lazy handler initialization
    switch (gpio_mapping->chip_type) {
        case HfGpioChipType::ESP32_INTERNAL: {
            if (CreateEsp32GpioPin(gpio_mapping->physical_pin, 
                                   direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT, 
                                   gpio_pin)) {
                // Pin created successfully
            }
            break;
        }
        
        case HfGpioChipType::PCAL95555_EXPANDER: {
            if (CreatePcal95555GpioPin(gpio_mapping->physical_pin, 
                                       direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT, 
                                       gpio_pin)) {
                // Pin created successfully
            }
            break;
        }
        
        default:
            console_error(TAG, "Unsupported chip type for pin %d", static_cast<int>(pin));
            return nullptr;
    }
    
    if (!gpio_pin) {
        console_error(TAG, "Failed to create GPIO pin %d", static_cast<int>(pin));
        return nullptr;
    }
    
    // Convert to shared_ptr and store in registry
    auto shared_pin = std::shared_ptr<BaseGpio>(std::move(gpio_pin));
    shared_pin_registry_[pin] = shared_pin;
    
    console_info(TAG, "Created shared GPIO pin %d (%s)", static_cast<int>(pin), gpio_mapping->description);
    return shared_pin;
}

std::vector<std::shared_ptr<BaseGpio>> GpioManager::GetPins(
    const std::vector<HfFunctionalGpioPin>& pins) noexcept {
    
    std::vector<std::shared_ptr<BaseGpio>> result;
    result.reserve(pins.size());
    
    for (const auto& pin : pins) {
        result.push_back(GetPin(pin));
    }
    
    return result;
}

//==============================================================================
// BASIC PIN OPERATIONS
//==============================================================================

bool GpioManager::SetActive(HardFOC::FunctionalGpioPin pin) noexcept {
    return SetState(pin, true);
}

bool GpioManager::SetInactive(HardFOC::FunctionalGpioPin pin) noexcept {
    return SetState(pin, false);
}

bool GpioManager::SetState(HardFOC::FunctionalGpioPin pin, bool state) noexcept {
    // Ensure system is initialized
    if (!EnsureInitialized().IsSuccess()) {
        UpdateStatistics(false);
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return false;
    }
    
    if (info->is_input) {
        UpdateStatistics(false);
        return false;
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
        return true;
    } else {
        info->error_count++;
        UpdateStatistics(false);
        
        std::string error_msg = "Failed to set pin state: " + std::string(HfGpioErrToString(result));
        AddErrorMessage(error_msg);
        console_error(TAG, "Failed to set pin %s: %s", info->name.data(), error_msg.c_str());
        return false;
    }
}

bool GpioManager::Toggle(HardFOC::FunctionalGpioPin pin) noexcept {
    // First read current state
    bool current_state = false;
    if (!ReadState(pin, current_state)) {
        return false;
    }
    
    // Toggle the state
    bool new_state = !current_state;
    if (!SetState(pin, new_state)) {
        return false;
    }
    
    return true;
}

bool GpioManager::ReadState(HardFOC::FunctionalGpioPin pin, bool& state) noexcept {
    // Ensure system is initialized
    if (!EnsureInitialized()) {
        UpdateStatistics(false);
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        console_error(TAG, "Pin not registered");
        return false;
    }
    
    // Perform the GPIO read operation
    auto result = info->gpio_driver->IsActive(state);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        info->current_state = state;
        info->access_count++;
        info->last_access_time = os_get_time_msec();
        UpdateStatistics(true);
        
        console_debug(TAG, "Read pin %s state: %s", info->name.data(), 
                     state ? "active" : "inactive");
        return true;
    } else {
        info->error_count++;
        UpdateStatistics(false);
        
        std::string error_msg = "Failed to read pin state: " + std::string(HfGpioErrToString(result));
        AddErrorMessage(error_msg);
        console_error(TAG, "Failed to read pin %s: %s", info->name.data(), error_msg.c_str());
        return false;
    }
}

bool GpioManager::IsActive(HardFOC::FunctionalGpioPin pin, bool& active) noexcept {
    return ReadState(pin, active);
}

//==============================================================================
// BATCH OPERATIONS
//==============================================================================

GpioBatchResult GpioManager::BatchWrite(const GpioBatchOperation& operation) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioBatchResult result;
    result.pins = operation.pins;
    result.states = operation.states;
    result.results.reserve(operation.pins.size());
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        // Fill all results with error and return
        for (size_t i = 0; i < operation.pins.size(); ++i) {
            result.results.push_back(ResultCode::ERROR_NOT_INITIALIZED);
        }
        result.overall_result = ResultCode::ERROR_NOT_INITIALIZED;
        return result;
    }
    
    if (!operation.is_write_operation) {
        for (size_t i = 0; i < operation.pins.size(); ++i) {
            result.results.push_back(ResultCode::ERROR_INVALID_PARAMETER);
        }
        result.overall_result = ResultCode::ERROR_INVALID_PARAMETER;
        return result;
    }
    
    if (operation.pins.size() != operation.states.size()) {
        for (size_t i = 0; i < operation.pins.size(); ++i) {
            result.results.push_back(ResultCode::ERROR_INVALID_PARAMETER);
        }
        result.overall_result = ResultCode::ERROR_INVALID_PARAMETER;
        return result;
    }
    
    bool all_successful = true;
    
    // Perform operations for each pin
    for (size_t i = 0; i < operation.pins.size(); ++i) {
        bool pin_success = SetState(operation.pins[i], operation.states[i]);
        result.results.push_back(pin_success ? ResultCode::SUCCESS : ResultCode::ERROR_OPERATION_FAILED);
        
        if (!pin_success) {
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? ResultCode::SUCCESS : ResultCode::ERROR_OPERATION_FAILED;
    
    UpdateStatistics(all_successful);
    
    return result;
}

GpioBatchResult GpioManager::BatchRead(const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioBatchResult result;
    result.pins = pins;
    result.states.reserve(pins.size());
    result.results.reserve(pins.size());
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        // Fill all results with error and return
        for (size_t i = 0; i < pins.size(); ++i) {
            result.results.push_back(ResultCode::ERROR_NOT_INITIALIZED);
            result.states.push_back(false);
        }
        result.overall_result = ResultCode::ERROR_NOT_INITIALIZED;
        return result;
    }
    
    bool all_successful = true;
    
    // Perform read operations for each pin
    for (const auto& pin : pins) {
        bool state = false;
        bool pin_success = ReadState(pin, state);
        result.results.push_back(pin_success ? ResultCode::SUCCESS : ResultCode::ERROR_OPERATION_FAILED);
        result.states.push_back(state);
        
        if (!pin_success) {
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? ResultCode::SUCCESS : ResultCode::ERROR_OPERATION_FAILED;
    
    UpdateStatistics(all_successful);
    
    return result;
}

GpioBatchResult GpioManager::SetMultipleActive(const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept {
    std::vector<bool> states(pins.size(), true);
    GpioBatchOperation operation(pins, states);
    return BatchWrite(operation);
}

GpioBatchResult GpioManager::SetMultipleInactive(const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept {
    std::vector<bool> states(pins.size(), false);
    GpioBatchOperation operation(pins, states);
    return BatchWrite(operation);
}

//==============================================================================
// PIN CONFIGURATION
//==============================================================================

hf_gpio_err_t GpioManager::SetPinDirection(HardFOC::FunctionalGpioPin pin, bool direction) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        console_error(TAG, "GPIO manager not initialized");
        return hf_gpio_err_t::GPIO_ERROR_NOT_INITIALIZED;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        console_error(TAG, "Pin not registered");
        return hf_gpio_err_t::GPIO_ERROR_PIN_NOT_FOUND;
    }
    
    auto result = info->gpio_driver->SetDirection(
        direction ? hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT : hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT
    );
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        info->is_input = direction;
        UpdateStatistics(true);
        console_debug(TAG, "Set pin %s direction to %s", info->name.data(), 
                     direction ? "input" : "output");
        return hf_gpio_err_t::GPIO_SUCCESS;
    } else {
        UpdateStatistics(false);
        console_error(TAG, "Failed to set pin %s direction: %s", info->name.data(),
                     HfGpioErrToString(result));
        return result;
    }
}

hf_gpio_err_t GpioManager::SetPinPullMode(HardFOC::FunctionalGpioPin pin, hf_gpio_pull_mode_t pull_mode) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        console_error(TAG, "GPIO manager not initialized");
        return hf_gpio_err_t::GPIO_ERROR_NOT_INITIALIZED;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        console_error(TAG, "Pin not registered");
        return hf_gpio_err_t::GPIO_ERROR_PIN_NOT_FOUND;
    }
    
    auto result = info->gpio_driver->SetPullMode(pull_mode);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        console_debug(TAG, "Set pin %s pull mode to %d", info->name.data(), (int)pull_mode);
        return hf_gpio_err_t::GPIO_SUCCESS;
    } else {
        UpdateStatistics(false);
        console_error(TAG, "Failed to set pin %s pull mode: %s", info->name.data(),
                     HfGpioErrToString(result));
        return result;
    }
}

hf_gpio_err_t GpioManager::SetPinOutputMode(HardFOC::FunctionalGpioPin pin, hf_gpio_output_mode_t output_mode) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        console_error(TAG, "GPIO manager not initialized");
        return hf_gpio_err_t::GPIO_ERROR_NOT_INITIALIZED;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        console_error(TAG, "Pin not registered");
        return hf_gpio_err_t::GPIO_ERROR_PIN_NOT_FOUND;
    }
    
    auto result = info->gpio_driver->SetOutputMode(output_mode);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        console_debug(TAG, "Set pin %s output mode to %d", info->name.data(), (int)output_mode);
        return hf_gpio_err_t::GPIO_SUCCESS;
    } else {
        UpdateStatistics(false);
        console_error(TAG, "Failed to set pin %s output mode: %s", info->name.data(),
                     HfGpioErrToString(result));
        return result;
    }
}

bool GpioManager::GetPinDirection(HardFOC::FunctionalGpioPin pin, bool& direction) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_error(TAG, "GPIO manager not initialized");
        return false;
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        console_error(TAG, "Pin not registered");
        return false;
    }
    
    direction = info->is_input;
    return true;
}

bool GpioManager::GetPinPullMode(HardFOC::FunctionalGpioPin pin, hf_gpio_pull_mode_t& pull_mode) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_error(TAG, "GPIO manager not initialized");
        return false;
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        console_error(TAG, "Pin not registered");
        return false;
    }
    
    pull_mode = info->gpio_driver->GetPullMode();
    return true;
}

bool GpioManager::GetPinOutputMode(HardFOC::FunctionalGpioPin pin, hf_gpio_output_mode_t& output_mode) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        console_error(TAG, "GPIO manager not initialized");
        return false;
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        console_error(TAG, "Pin not registered");
        return false;
    }
    
    output_mode = info->gpio_driver->GetOutputMode();
    return true;
}

//==============================================================================
// INTERRUPT SUPPORT
//==============================================================================

hf_gpio_err_t GpioManager::ConfigureInterrupt(HardFOC::FunctionalGpioPin pin,
                                             BaseGpio::InterruptTrigger trigger,
                                             BaseGpio::InterruptCallback callback,
                                             void* user_data) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        console_error(TAG, "GPIO manager not initialized");
        return hf_gpio_err_t::GPIO_ERROR_NOT_INITIALIZED;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        console_error(TAG, "Pin not registered");
        return hf_gpio_err_t::GPIO_ERROR_PIN_NOT_FOUND;
    }
    
    if (!info->gpio_driver->SupportsInterrupts()) {
        UpdateStatistics(false);
        console_error(TAG, "Pin does not support interrupts");
        return hf_gpio_err_t::GPIO_ERR_INTERRUPT_NOT_SUPPORTED;
    }
    
    auto result = info->gpio_driver->ConfigureInterrupt(trigger, callback, user_data);
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return hf_gpio_err_t::GPIO_SUCCESS;
    } else {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_INTERRUPT_HANDLER_FAILED;
    }
}

hf_gpio_err_t GpioManager::EnableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    auto result = info->gpio_driver->EnableInterrupt();
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return hf_gpio_err_t::GPIO_SUCCESS;
    } else {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_INTERRUPT_HANDLER_FAILED;
    }
}

hf_gpio_err_t GpioManager::DisableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    auto result = info->gpio_driver->DisableInterrupt();
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return hf_gpio_err_t::GPIO_SUCCESS;
    } else {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_INTERRUPT_HANDLER_FAILED;
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

bool GpioManager::GetSystemHealth(std::string& health_info) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return false;
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
    
    health_info = health.str();
    return true;
}

bool GpioManager::ResetAllPins() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return false;
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
    
    return fail_count == 0;
}

bool GpioManager::GetPinStatistics(HardFOC::FunctionalGpioPin pin, BaseGpio::PinStatistics& statistics) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        return false;
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return false;
    }
    
    statistics = info->gpio_driver->GetStatistics();
    return true;
}

bool GpioManager::ClearPinStatistics(HardFOC::FunctionalGpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!is_initialized_.load()) {
        UpdateStatistics(false);
        return false;
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return false;
    }
    
    auto result = info->gpio_driver->ResetStatistics();
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateStatistics(true);
        return true;
    } else {
        UpdateStatistics(false);
        return false;
    }
}

bool GpioManager::GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
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
    
    return true;
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

bool GpioManager::InitializeEsp32Gpio() noexcept {
    console_info(TAG, "Initializing ESP32-C6 native GPIO system");
    // ESP32 GPIO is initialized automatically when drivers are created
    return true;
}



bool GpioManager::InitializeTmc9660Gpio() noexcept {
    console_info(TAG, "TMC9660 GPIO support available via lazy initialization");
    
    // TMC9660 GPIO will be initialized lazily when first accessed
    // Uses CommChannelsManager for SPI/UART access to TMC9660
    return true;
}

bool GpioManager::CreateGpioPin(
    HardFOC::FunctionalGpioPin pin, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept {
    
    // Get hardware resource information
    const HardFOC::GpioHardwareResource* resource = nullptr;
    if (!GetPinHardwareResource(pin, resource)) {
        return false;
    }
    
    // Create pin based on hardware chip type
    switch (resource->chip_id) {
        case HardFOC::HardwareChip::ESP32_INTERNAL_GPIO:
            return CreateEsp32GpioPin(resource->pin_id, direction, driver);
            
        case HardFOC::HardwareChip::PCAL95555_GPIO:
            return CreatePcal95555GpioPin(resource->pin_id, direction, driver);
            
        case HardFOC::HardwareChip::TMC9660_GPIO:
            return CreateTmc9660GpioPin(resource->pin_id, direction, driver);
            
        default:
            return false;
    }
}
bool GpioManager::CreateEsp32GpioPin(uint8_t pin_id, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept {
    // Create ESP32 GPIO with noexcept constructor (no exceptions)
    auto gpio = std::make_unique<EspGpio>(
        static_cast<hf_pin_num_t>(pin_id),
        direction ? hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT : hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT,
        hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING
    );
    
    driver = std::move(gpio);
    return true;
}

//==============================================================================
// LAZY HANDLER INITIALIZATION
//==============================================================================

bool GpioManager::EnsurePcal95555Handler() noexcept {
    RtosMutex::LockGuard lock(pcal_handler_mutex_);
    
    // Return if already initialized
    if (pcal95555_handler_) {
        return true;
    }
    
    console_info(TAG, "Lazy initializing PCAL95555 GPIO handler");
    
    // Get I2C device for PCAL95555 from CommChannelsManager
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto* i2c_device = comm_manager.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c_device) {
        return false;
    }
    
    // Create hardware interrupt pin for PCAL95555 INT line (ESP32 GPIO)
    // For now, we'll skip interrupt support and just create the handler
    console_info(TAG, "Creating PCAL95555 GPIO handler without interrupt support");
    
    // Create PCAL95555 handler with I2C device (no exceptions)
    pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_device, nullptr);
    if (!pcal95555_handler_->IsHealthy()) {
        pcal95555_handler_.reset();
        return false;
    }
    
    console_info(TAG, "PCAL95555 GPIO handler initialized successfully");
    return true;
}

bool GpioManager::CreatePcal95555GpioPin(uint8_t pin_id, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept {
    // Ensure PCAL95555 handler is initialized (lazy initialization)
    if (!EnsurePcal95555Handler()) {
        return false;
    }
    
    // Create pin using the wrapper (no exceptions)
    auto chip_pin = static_cast<Pcal95555Chip1Pin>(pin_id);
    auto gpio = pcal95555_handler_->CreateGpioPin(chip_pin);
    
    if (!gpio) {
        return false;
    }
    
    driver = std::move(gpio);
    return true;
}

bool GpioManager::CreateTmc9660GpioPin(uint8_t pin_id, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept {
    // TMC9660 GPIO implementation would go here
    // For now, return unsupported
    return false;
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

bool GpioManager::ValidateHardwareResource(const HardFOC::GpioHardwareResource* resource) const noexcept {
    if (!resource) {
        return false;
    }
    
    // Validate chip ID
    if (static_cast<uint8_t>(resource->chip_id) >= static_cast<uint8_t>(HardFOC::HardwareChip::HARDWARE_CHIP_COUNT)) {
        return false;
    }
    
    // Validate pin ID based on chip type
    switch (resource->chip_id) {
        case HardFOC::HardwareChip::ESP32_INTERNAL_GPIO:
            if (resource->pin_id >= GPIO_NUM_MAX) {
                return false;
            }
            break;
            
        case HardFOC::HardwareChip::PCAL95555_GPIO:
            if (resource->pin_id >= 16) { // PCAL95555 has 16 pins
                return false;
            }
            break;
            
        case HardFOC::HardwareChip::TMC9660_GPIO:
            if (resource->pin_id >= 2) { // TMC9660 has 2 GPIO pins
                return false;
            }
            break;
            
        default:
            return false;
    }
    
    return true;
}

bool GpioManager::CheckHardwareConflicts(const HardFOC::GpioHardwareResource* resource) const noexcept {
    if (!resource) {
        return false;
    }
    
    // Check for conflicts with already registered pins
    for (const auto& [pin, info] : pin_registry_) {
        if (info && info->hardware_chip == resource->chip_id && 
            info->hardware_pin_id == resource->pin_id) {
            return false;
        }
    }
    
    return true;
}
