#include "GpioManager.h"
#include "Pcal95555Gpio.h"
#include "mcu/McuDigitalGpio.h"
#include "esp_log.h"

/**
 * @file GpioManager.cpp
 * @brief Implementation of the consolidated GPIO manager for HardFOC.
 * 
 * This file implements the unified GPIO management system that replaces
 * the legacy GpioData and GpioHandler classes.
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

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

Result<void> GpioManager::Initialize(SfI2cBus& i2cBus, 
                                           Tmc9660MotorController& tmc9660Controller) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (isInitialized_.load()) {
        ESP_LOGW(TAG, "GPIO manager already initialized");
        return Result<void>(ResultCode::ERROR_ALREADY_INITIALIZED);
    }
    
    ESP_LOGI(TAG, "Initializing HardFOC GPIO Manager v2.0");
    
    // Store hardware interface references
    i2cBus_ = &i2cBus;
    tmc9660Controller_ = &tmc9660Controller;
    
    // Initialize hardware subsystems
    auto esp32Result = InitializeEsp32Gpio();
    if (esp32Result.IsError()) {
        ESP_LOGE(TAG, "Failed to initialize ESP32 GPIO: %s", 
                 GetResultDescription(esp32Result.GetResult()).data());
        return esp32Result;
    }
    
    auto pcalResult = InitializePcal95555Gpio();
    if (pcalResult.IsError()) {
        ESP_LOGE(TAG, "Failed to initialize PCAL95555 GPIO: %s", 
                 GetResultDescription(pcalResult.GetResult()).data());
        return pcalResult;
    }
    
    auto tmcResult = InitializeTmc9660Gpio();
    if (tmcResult.IsError()) {
        ESP_LOGE(TAG, "Failed to initialize TMC9660 GPIO: %s", 
                 GetResultDescription(tmcResult.GetResult()).data());
        return tmcResult;
    }
    
    // Register default pin mappings
    auto regResult = RegisterDefaultPins();
    if (regResult.IsError()) {
        ESP_LOGE(TAG, "Failed to register default pins: %s", 
                 GetResultDescription(regResult.GetResult()).data());
        return regResult;
    }
    
    isInitialized_.store(true);
    ESP_LOGI(TAG, "GPIO manager initialized successfully with %zu pins", 
             gpioRegistry_.size());
    
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::Shutdown() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    ESP_LOGI(TAG, "Shutting down GPIO manager");
    
    // Reset all pins to safe states
    auto resetResult = ResetAllPins();
    if (resetResult.IsError()) {
        ESP_LOGW(TAG, "Warning: Failed to reset all pins during shutdown");
    }
    
    // Clear registrations
    gpioRegistry_.clear();
    nameToPin_.clear();
    
    // Reset hardware interfaces
    pcal95555Gpio_.reset();
    tmc9660Controller_ = nullptr;
    i2cBus_ = nullptr;
    
    isInitialized_.store(false);
    ESP_LOGI(TAG, "GPIO manager shutdown complete");
    
    return HARDFOC_SUCCESS();
}

bool GpioManager::IsInitialized() const noexcept {
    return isInitialized_.load();
}

//==============================================================================
// PIN REGISTRATION METHODS
//==============================================================================

Result<void> GpioManager::RegisterPin(GpioPin pin, bool direction, 
                                            bool initialState) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto it = gpioRegistry_.find(pin);
    if (it != gpioRegistry_.end()) {
        ESP_LOGW(TAG, "Pin already registered: %d", static_cast<int>(pin));
        return Result<void>(ResultCode::ERROR_GPIO_ALREADY_REGISTERED);
    }
    
    // TODO: Implement pin registration logic based on pin mapping
    // This would involve:
    // 1. Looking up the pin in the platform mapping
    // 2. Creating the appropriate GPIO driver instance
    // 3. Configuring the pin direction and initial state
    // 4. Adding to the registry
    
    ESP_LOGD(TAG, "Registered pin %d (direction: %s, initial: %s)", 
             static_cast<int>(pin), 
             direction ? "input" : "output",
             initialState ? "active" : "inactive");
    
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::UnregisterPin(GpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto it = gpioRegistry_.find(pin);
    if (it == gpioRegistry_.end()) {
        return Result<void>(ResultCode::ERROR_GPIO_PIN_NOT_FOUND);
    }
    
    gpioRegistry_.erase(it);
    ESP_LOGD(TAG, "Unregistered pin %d", static_cast<int>(pin));
    
    return HARDFOC_SUCCESS();
}

bool GpioManager::IsPinRegistered(GpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return gpioRegistry_.find(pin) != gpioRegistry_.end();
}

Result<const GpioInfo*> GpioManager::GetPinInfo(GpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<const GpioInfo*>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    const auto* info = FindGpioInfo(pin);
    if (!info) {
        return Result<const GpioInfo*>(ResultCode::ERROR_GPIO_PIN_NOT_FOUND);
    }
    
    return Result<const GpioInfo*>(info);
}

//==============================================================================
// BASIC PIN OPERATIONS
//==============================================================================

Result<void> GpioManager::SetActive(GpioPin pin) noexcept {
    return SetState(pin, true);
}

Result<void> GpioManager::SetInactive(GpioPin pin) noexcept {
    return SetState(pin, false);
}

Result<void> GpioManager::SetState(GpioPin pin, bool state) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        UpdateStatistics(false);
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<void>(ResultCode::ERROR_GPIO_PIN_NOT_FOUND);
    }
    
    if (info->isInput) {
        UpdateStatistics(false);
        return Result<void>(ResultCode::ERROR_GPIO_DIRECTION_INVALID);
    }
    
    // Perform the GPIO write operation
    bool success = false;
    if (info->gpio) {
        if (state) {
            success = info->gpio->SetActive();
        } else {
            success = info->gpio->SetInactive();
        }
    }
    
    if (success) {
        info->currentState = state;
        info->accessCount++;
        UpdateStatistics(true);
        ESP_LOGV(TAG, "Set pin %d to %s", static_cast<int>(pin), state ? "active" : "inactive");
        return HARDFOC_SUCCESS();
    } else {
        UpdateStatistics(false);
        ESP_LOGE(TAG, "Failed to set pin %d state", static_cast<int>(pin));
        return Result<void>(ResultCode::ERROR_GPIO_WRITE_FAILED);
    }
}

Result<bool> GpioManager::Toggle(GpioPin pin) noexcept {
    // First read current state
    auto currentResult = ReadState(pin);
    if (currentResult.IsError()) {
        return Result<bool>(currentResult.GetResult());
    }
    
    // Toggle the state
    bool newState = !currentResult.GetValue();
    auto setResult = SetState(pin, newState);
    if (setResult.IsError()) {
        return Result<bool>(setResult.GetResult());
    }
    
    return Result<bool>(newState);
}

Result<bool> GpioManager::ReadState(GpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        UpdateStatistics(false);
        return Result<bool>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto* info = FindGpioInfo(pin);
    if (!info) {
        UpdateStatistics(false);
        return Result<bool>(ResultCode::ERROR_GPIO_PIN_NOT_FOUND);
    }
    
    // Perform the GPIO read operation
    bool state = false;
    bool success = false;
    if (info->gpio) {
        success = info->gpio->IsActive();
        state = success;  // For this simplified implementation
    }
    
    if (success) {
        info->currentState = state;
        info->accessCount++;
        UpdateStatistics(true);
        ESP_LOGV(TAG, "Read pin %d state: %s", static_cast<int>(pin), 
                 state ? "active" : "inactive");
        return Result<bool>(state);
    } else {
        UpdateStatistics(false);
        ESP_LOGE(TAG, "Failed to read pin %d state", static_cast<int>(pin));
        return Result<bool>(ResultCode::ERROR_GPIO_READ_FAILED);
    }
}

Result<bool> GpioManager::IsActive(GpioPin pin) noexcept {
    return ReadState(pin);
}

//==============================================================================
// SYSTEM INFORMATION METHODS
//==============================================================================

size_t GpioManager::GetRegisteredPinCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return gpioRegistry_.size();
}

std::vector<GpioPin> GpioManager::GetRegisteredPins() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<GpioPin> pins;
    pins.reserve(gpioRegistry_.size());
    
    for (const auto& pair : gpioRegistry_) {
        pins.push_back(pair.first);
    }
    
    return pins;
}

Result<std::string> GpioManager::GetSystemHealth() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<std::string>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    std::string health;
    health += "HardFOC GPIO Manager Health Report\n";
    health += "==================================\n";
    health += "Status: " + std::string(isInitialized_.load() ? "Initialized" : "Not Initialized") + "\n";
    health += "Registered Pins: " + std::to_string(gpioRegistry_.size()) + "\n";
    health += "Total Operations: " + std::to_string(totalOperations_.load()) + "\n";
    health += "Successful Operations: " + std::to_string(successfulOperations_.load()) + "\n";
    health += "Failed Operations: " + std::to_string(failedOperations_.load()) + "\n";
    
    if (totalOperations_.load() > 0) {
        float successRate = (static_cast<float>(successfulOperations_.load()) / 
                            static_cast<float>(totalOperations_.load())) * 100.0f;
        health += "Success Rate: " + std::to_string(successRate) + "%\n";
    }
    
    return Result<std::string>(std::move(health));
}

Result<void> GpioManager::ResetAllPins() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    size_t resetCount = 0;
    size_t failCount = 0;
    
    for (auto& pair : gpioRegistry_) {
        auto& info = pair.second;
        if (!info->isInput && info->gpio) {
            if (info->gpio->SetInactive()) {
                info->currentState = false;
                resetCount++;
            } else {
                failCount++;
            }
        }
    }
    
    ESP_LOGI(TAG, "Reset %zu output pins (%zu failed)", resetCount, failCount);
    
    return failCount == 0 ? HARDFOC_SUCCESS() : 
           Result<void>(ResultCode::ERROR_OPERATION_FAILED);
}

//==============================================================================
// PRIVATE IMPLEMENTATION METHODS
//==============================================================================

Result<void> GpioManager::InitializeEsp32Gpio() noexcept {
    ESP_LOGD(TAG, "Initializing ESP32-C6 native GPIO");
    // TODO: Initialize ESP32-C6 GPIO pins
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::InitializePcal95555Gpio() noexcept {
    ESP_LOGD(TAG, "Initializing PCAL95555 GPIO expander");
    
    if (!i2cBus_) {
        return Result<void>(ResultCode::ERROR_DEPENDENCY_MISSING);
    }
    
    try {
        pcal95555Gpio_ = std::make_unique<Pcal95555Gpio>(*i2cBus_);
        // TODO: Configure PCAL95555
        return HARDFOC_SUCCESS();
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Failed to create PCAL95555 GPIO: %s", e.what());
        return Result<void>(ResultCode::ERROR_INIT_FAILED);
    }
}

Result<void> GpioManager::InitializeTmc9660Gpio() noexcept {
    ESP_LOGD(TAG, "Initializing TMC9660 GPIO");
    
    if (!tmc9660Controller_) {
        return Result<void>(ResultCode::ERROR_DEPENDENCY_MISSING);
    }
    
    // TODO: Initialize TMC9660 GPIO pins
    return HARDFOC_SUCCESS();
}

Result<void> GpioManager::RegisterDefaultPins() noexcept {
    ESP_LOGD(TAG, "Registering default pin mappings");
    
    // TODO: Register all default pins based on platform mapping
    // This would involve iterating through the platform mapping table
    // and registering each functional pin with its corresponding hardware
    
    return HARDFOC_SUCCESS();
}

GpioInfo* GpioManager::FindGpioInfo(GpioPin pin) noexcept {
    auto it = gpioRegistry_.find(pin);
    return (it != gpioRegistry_.end()) ? it->second.get() : nullptr;
}

const GpioInfo* GpioManager::FindGpioInfo(GpioPin pin) const noexcept {
    auto it = gpioRegistry_.find(pin);
    return (it != gpioRegistry_.end()) ? it->second.get() : nullptr;
}

void GpioManager::UpdateStatistics(bool success) noexcept {
    totalOperations_.fetch_add(1);
    if (success) {
        successfulOperations_.fetch_add(1);
    } else {
        failedOperations_.fetch_add(1);
    }
}
