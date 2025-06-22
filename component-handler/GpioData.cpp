/**
 * @file GpioData.cpp
 * @brief Implementation of the comprehensive multi-source GPIO data management system.
 *
 * This file implements the unified GPIO data handler that supports multiple GPIO sources
 * including ESP32-C6 native pins, PCAL95555 I2C expander, and TMC9660 controller pins.
 */

#include "GpioData.h"
#include "OsUtility.h"
#include "ConsolePort.h"

#include <algorithm>
#include <cstring>
#include <mutex>

static const char* TAG = "GpioData";

//==============================================================================
// Singleton Implementation
//==============================================================================

GpioData& GpioData::GetInstance() noexcept {
    static GpioData instance;
    return instance;
}

GpioData::GpioData() noexcept
    : gpioTable_{},
      registeredPinCount_(0),
      systemHealth_{},
      lastHealthUpdate_(0),
      communicationErrorCount_(0)
{
    console_info(TAG, "GpioData constructor - GPIO data storage system ready");
    
    // Initialize system health
    systemHealth_.totalRegisteredPins = 0;
    systemHealth_.totalCommunicationErrors = 0;
    systemHealth_.lastHealthCheckTime = 0;
}

GpioData::~GpioData() noexcept {
    console_info(TAG, "GpioData destructor called - cleaning up GPIO resources");
}

//==============================================================================
// System Health Management (Generic)
//==============================================================================

void GpioData::IncrementCommunicationErrors() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    communicationErrorCount_++;
    systemHealth_.totalCommunicationErrors = communicationErrorCount_;
}

//==============================================================================
// GPIO Registration and Management
//==============================================================================

bool GpioData::RegisterGpioPin(GpioPin pin, DigitalGpio& gpio, std::string_view name, 
                               GpioChip source) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (registeredPinCount_ >= static_cast<uint8_t>(GpioPin::GPIO_PIN_COUNT)) {
        console_error(TAG, "GPIO table full - cannot register pin %s", name.data());
        return false;
    }
    
    // Check if pin is already registered
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].pin == pin) {
            console_warning(TAG, "Pin %s already registered", GpioPinToString(pin).data());
            return false;
        }
    }
  
    
    // Add the new pin
    gpioTable_[registeredPinCount_] = GpioInfo(pin, gpio, name, source);
    registeredPinCount_++;
    
    console_info(TAG, "Registered GPIO pin: %s (%s) from %s", 
                 GpioPinToString(pin).data(), name.data(), GpioChipToString(source).data());
    
    return true;
}

uint8_t GpioData::GetRegisteredPinCount() const noexcept {
    return registeredPinCount_;
}

uint8_t GpioData::GetRegisteredPinCount(GpioChip source) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].source == source) {
            count++;
        }
    }
    return count;
}

const GpioInfo* GpioData::GetPinInfo(GpioPin pin) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].pin == pin) {
            return &gpioTable_[i];
        }
    }
    
    return nullptr;
}

const GpioInfo* GpioData::GetPinByName(std::string_view name) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].name == name) {
            return &gpioTable_[i];
        }
    }
    
    return nullptr;
}

uint8_t GpioData::GetPinsBySource(GpioChip source, std::vector<const GpioInfo*>& pins) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    pins.clear();
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].source == source) {
            pins.push_back(&gpioTable_[i]);
        }
    }
    
    return static_cast<uint8_t>(pins.size());
}

GpioInfo* GpioData::FindGpioInfo(GpioPin pin) noexcept {
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].pin == pin) {
            return &gpioTable_[i];
        }
    }
    return nullptr;
}

GpioInfo* GpioData::FindGpioInfoByName(std::string_view name) noexcept {
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].name == name) {
            return &gpioTable_[i];
        }
    }
    return nullptr;
}

//==============================================================================
// Single Pin Operations  
//==============================================================================

bool GpioData::SetActive(GpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "Pin %s not found", GpioPinToString(pin).data());
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Pin %s not initialized", GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.SetActive();
}

bool GpioData::SetInactive(GpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "Pin %s not found", GpioPinToString(pin).data());
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Pin %s not initialized", GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.SetInactive();
}

bool GpioData::Toggle(GpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "Pin %s not found", GpioPinToString(pin).data());
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Pin %s not initialized", GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.Toggle();
}

bool GpioData::IsActive(GpioPin pin) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        return false;
    }
    
    return gpioInfo->gpio.IsActive();
}

//==============================================================================
// Name-Based Operations
//==============================================================================

bool GpioData::SetPinByName(std::string_view name, bool active) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfoByName(name);
    if (!gpioInfo) {
        console_error(TAG, "Pin named '%s' not found", name.data());
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Pin '%s' not initialized", name.data());
        return false;
    }
    
    return active ? gpioInfo->gpio.SetActive() : gpioInfo->gpio.SetInactive();
}

bool GpioData::GetPinByName(std::string_view name) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfoByName(name);
    if (!gpioInfo) {
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        return false;
    }
    
    return gpioInfo->gpio.IsActive();
}

bool GpioData::TogglePinByName(std::string_view name) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfoByName(name);
    if (!gpioInfo) {
        console_error(TAG, "Pin named '%s' not found", name.data());
        return false;
    }
    
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Pin '%s' not initialized", name.data());
        return false;
    }
    
    return gpioInfo->gpio.Toggle();
}

//==============================================================================
// System Health and Diagnostics
//==============================================================================

GpioSystemHealth GpioData::GetSystemHealth() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    UpdateSystemHealth();
    return systemHealth_;
}

bool GpioData::UpdateSystemHealth() noexcept {
    uint32_t currentTime = OsUtility::GetCurrentTimeMs();
    
    // Update basic health statistics
    systemHealth_.totalRegisteredPins = registeredPinCount_;
    systemHealth_.totalCommunicationErrors = communicationErrorCount_;
    systemHealth_.lastHealthCheckTime = currentTime;
    lastHealthUpdate_ = currentTime;
    
    return systemHealth_.IsBasicallyHealthy();
}

bool GpioData::IsCommunicating(GpioChip gpioChip) noexcept {
    // GpioData doesn't track communication status - this is managed by GpioHandler
    // Return true if we have registered pins from this source
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].source == gpioChip) {
            return true;  // We have at least one pin from this source
        }
    }
    
    return false;  // No pins from this source
}

bool GpioData::RunGpioTest() noexcept {
    console_info(TAG, "Running GPIO data system test...");
    
    bool testResult = true;
    uint32_t testedPins = 0;
    uint32_t passedPins = 0;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Test each registered pin
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        GpioInfo& pinInfo = gpioTable_[i];
        testedPins++;
        
        console_info(TAG, "Testing pin %s (%s)", 
                     pinInfo.name.data(), GpioPinToString(pinInfo.pin).data());
        
        // Test pin initialization
        if (!pinInfo.gpio.EnsureInitialized()) {
            console_error(TAG, "Pin %s failed initialization test", pinInfo.name.data());
            testResult = false;
            continue;
        }
        
        // Test pin operations (for output pins)
        if (pinInfo.gpio.GetDirection() == BaseGpio::Direction::Output) {
            // Test set active
            if (!pinInfo.gpio.SetActive()) {
                console_error(TAG, "Pin %s failed SetActive test", pinInfo.name.data());
                testResult = false;
                continue;
            }
            
            // Brief delay
            OsUtility::DelayMs(1);
            
            // Test set inactive
            if (!pinInfo.gpio.SetInactive()) {
                console_error(TAG, "Pin %s failed SetInactive test", pinInfo.name.data());
                testResult = false;
                continue;
            }
        }
        
        passedPins++;
        console_info(TAG, "Pin %s passed all tests", pinInfo.name.data());
    }
    
    console_info(TAG, "GPIO test complete: %u/%u pins passed", passedPins, testedPins);
    
    if (testResult) {
        console_info(TAG, "GPIO data system test PASSED");
    } else {
        console_error(TAG, "GPIO data system test FAILED");
    }
    
    return testResult;
}

//==============================================================================
// Multi-Pin and Advanced Operations (Simplified implementations)
//==============================================================================

bool GpioData::SetMultipleActive(const std::vector<GpioPin>& pins) noexcept {
    bool success = true;
    for (GpioPin pin : pins) {
        success &= SetActive(pin);
    }
    return success;
}

bool GpioData::SetMultipleInactive(const std::vector<GpioPin>& pins) noexcept {
    bool success = true;
    for (GpioPin pin : pins) {
        success &= SetInactive(pin);
    }
    return success;
}

uint8_t GpioData::SetMultiplePins(const GpioPin* pins, uint8_t count, bool active) noexcept {
    if (!pins || count == 0) return 0;
    
    uint8_t successful = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (active ? SetActive(pins[i]) : SetInactive(pins[i])) {
            successful++;
        }
    }
    return successful;
}

uint8_t GpioData::ReadMultiplePins(GpioReadSpec* specs, uint8_t count) noexcept {
    if (!specs || count == 0) return 0;
    
    uint32_t timestamp = OsUtility::GetCurrentTimeMs();
    uint8_t successful = 0;
    
    for (uint8_t i = 0; i < count; ++i) {
        bool newState = IsActive(specs[i].pin);
        specs[i].UpdateState(newState, timestamp);
        successful++;
    }
    
    return successful;
}

bool GpioData::PulsePin(GpioPin pin, uint32_t duration_ms) noexcept {
    if (!SetActive(pin)) return false;
    OsUtility::DelayMs(duration_ms);
    return SetInactive(pin);
}

bool GpioData::SetLedStatus(GpioPin status_led, GpioPin error_led, GpioPin comm_led,
                           bool status_on, bool error_on, bool comm_on) noexcept {
    bool success = true;
    success &= SetPinByName("led_status", status_on);
    success &= SetPinByName("led_error", error_on);
    success &= SetPinByName("led_comm", comm_on);
    return success;
}

bool GpioData::BlinkPin(GpioPin pin, uint32_t onTimeMs, uint32_t offTimeMs, uint8_t cycles) noexcept {
    // Simplified implementation - just pulse the pin once
    if (!SetActive(pin)) return false;
    OsUtility::DelayMs(onTimeMs);
    return SetInactive(pin);
}

bool GpioData::StopBlink(GpioPin pin) noexcept {
    // Simplified implementation - just set pin inactive
    return SetInactive(pin);
}

bool GpioData::RunDiagnostics() noexcept {
    return RunGpioTest();
}

bool GpioData::ResetAllPins() noexcept {
    console_info(TAG, "Resetting all GPIO pins to inactive state...");
    
    std::lock_guard<std::mutex> lock(mutex_);
    bool success = true;
    
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        GpioInfo& pinInfo = gpioTable_[i];
        if (pinInfo.gpio.GetDirection() == BaseGpio::Direction::Output) {
            if (!pinInfo.gpio.SetInactive()) {
                console_warning(TAG, "Failed to reset pin %s", pinInfo.name.data());
                success = false;
            }
        }
    }
    
    return success;
}

// Additional methods would be implemented similarly...
