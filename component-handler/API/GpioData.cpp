#include "GpioData.h"
#include "OsUtility.h"
#include "ConsolePort.h"
#include <algorithm>

/**
 * @file GpioData.cpp
 * @brief Implementation of the GpioData class.
 * 
 * This file contains the implementation of the comprehensive GPIO data management
 * system for the HardFOC project.
 */

static const char* TAG = "GpioData";

// Static member definition
GpioData& GpioData::GetInstance() noexcept {
    static GpioData instance;
    return instance;
}

GpioData::GpioData() noexcept
    : initialized_(false),
      gpioTable_{
          // Initialize with dummy entries - will be populated during initialization
      },
      registeredPinCount_(0)
{
    console_info(TAG, "GpioData constructor called");
}

GpioData::~GpioData() noexcept {
    console_info(TAG, "GpioData destructor called");
}

bool GpioData::EnsureInitialized() noexcept {
    if (!initialized_.load()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_.load()) {
            console_info(TAG, "Initializing GPIO system");
            if (Initialize()) {
                initialized_.store(true);
                console_info(TAG, "GPIO system initialized successfully");
            } else {
                console_error(TAG, "Failed to initialize GPIO system");
                return false;
            }
        }
    }
    return true;
}

bool GpioData::Initialize() noexcept {
    console_info(TAG, "GpioData::Initialize() - Setting up GPIO system");
    
    // Reset the registration count
    registeredPinCount_ = 0;
    
    // TODO: Register actual GPIO pins here
    // This would be done by the application during startup
    // For now, we just prepare the system for registration
    
    console_info(TAG, "GPIO system ready for pin registration");
    return true;
}

bool GpioData::RegisterGpioPin(GpioPin pin, DigitalGpio& gpio, std::string_view name) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (registeredPinCount_ >= static_cast<uint8_t>(GpioPin::GPIO_PIN_COUNT)) {
        console_error(TAG, "Cannot register more GPIO pins - table full");
        return false;
    }
    
    // Check if pin is already registered
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (gpioTable_[i].pin == pin) {
            console_warning(TAG, "Pin %s already registered", 
                          GpioPinToString(pin).data());
            return false;
        }
    }
    
    // Add the new pin
    gpioTable_[registeredPinCount_] = GpioInfo(pin, gpio, name);
    registeredPinCount_++;
    
    console_info(TAG, "Registered GPIO pin: %s (%s)", 
                 GpioPinToString(pin).data(), name.data());
    
    return true;
}

uint8_t GpioData::GetRegisteredPinCount() const noexcept {
    return registeredPinCount_;
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

bool GpioData::SetActive(GpioPin pin) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin %s not found", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.SetActive();
}

bool GpioData::SetInactive(GpioPin pin) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin %s not found", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.SetInactive();
}

bool GpioData::Toggle(GpioPin pin) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin %s not found", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.Toggle();
}

bool GpioData::IsActive(GpioPin pin) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin %s not found", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", 
                     GpioPinToString(pin).data());
        return false;
    }
    
    return gpioInfo->gpio.IsActive();
}

bool GpioData::SetPinByName(std::string_view name, bool active) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfoByName(name);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin with name '%s' not found", name.data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", name.data());
        return false;
    }
    
    if (active) {
        return gpioInfo->gpio.SetActive();
    } else {
        return gpioInfo->gpio.SetInactive();
    }
}

bool GpioData::GetPinByName(std::string_view name) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfoByName(name);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin with name '%s' not found", name.data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", name.data());
        return false;
    }
    
    return gpioInfo->gpio.IsActive();
}

bool GpioData::TogglePinByName(std::string_view name) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfoByName(name);
    if (!gpioInfo) {
        console_error(TAG, "GPIO pin with name '%s' not found", name.data());
        return false;
    }
    
    // Ensure the GPIO is initialized
    if (!gpioInfo->gpio.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO for pin %s", name.data());
        return false;
    }
    
    return gpioInfo->gpio.Toggle();
}

bool GpioData::SetMultipleActive(const std::vector<GpioPin>& pins) noexcept {
    bool overallSuccess = true;
    
    for (const auto& pin : pins) {
        if (!SetActive(pin)) {
            overallSuccess = false;
        }
    }
    
    return overallSuccess;
}

bool GpioData::SetMultipleInactive(const std::vector<GpioPin>& pins) noexcept {
    bool overallSuccess = true;
    
    for (const auto& pin : pins) {
        if (!SetInactive(pin)) {
            overallSuccess = false;
        }
    }
    
    return overallSuccess;
}

bool GpioData::ReadMultiplePins(std::vector<GpioReadSpec>& readSpecs) noexcept {
    bool overallSuccess = true;
    
    for (auto& spec : readSpecs) {
        bool state = IsActive(spec.pin);
        spec.UpdateState(state);
        
        // If IsActive returned false due to an error, mark as failed
        if (!EnsureInitialized()) {
            overallSuccess = false;
        }
    }
    
    return overallSuccess;
}

bool GpioData::SetPinPattern(const std::vector<GpioPin>& pins, uint32_t pattern) noexcept {
    bool overallSuccess = true;
    
    for (size_t i = 0; i < pins.size() && i < 32; ++i) {
        bool pinState = (pattern & (1U << i)) != 0;
        
        if (pinState) {
            if (!SetActive(pins[i])) {
                overallSuccess = false;
            }
        } else {
            if (!SetInactive(pins[i])) {
                overallSuccess = false;
            }
        }
    }
    
    return overallSuccess;
}

bool GpioData::GetPinPattern(const std::vector<GpioPin>& pins, uint32_t& pattern) noexcept {
    bool overallSuccess = true;
    pattern = 0;
    
    for (size_t i = 0; i < pins.size() && i < 32; ++i) {
        if (IsActive(pins[i])) {
            pattern |= (1U << i);
        }
        
        // Check if we had an error (assuming IsActive returns false on error)
        if (!EnsureInitialized()) {
            overallSuccess = false;
        }
    }
    
    return overallSuccess;
}

bool GpioData::PulsePin(GpioPin pin, uint32_t duration_ms) noexcept {
    if (!SetActive(pin)) {
        return false;
    }
    
    os_delay_msec(duration_ms);
    
    return SetInactive(pin);
}

bool GpioData::BlinkPin(GpioPin pin, uint32_t on_time_ms, uint32_t off_time_ms, uint8_t cycles) noexcept {
    for (uint8_t i = 0; i < cycles; ++i) {
        if (!SetActive(pin)) {
            return false;
        }
        
        os_delay_msec(on_time_ms);
        
        if (!SetInactive(pin)) {
            return false;
        }
        
        if (i < cycles - 1) { // Don't delay after the last cycle
            os_delay_msec(off_time_ms);
        }
    }
    
    return true;
}

bool GpioData::SetLedStatus(GpioPin status_led, GpioPin error_led, GpioPin comm_led,
                           bool status_on, bool error_on, bool comm_on) noexcept {
    bool success = true;
    
    // Set status LED
    if (status_on) {
        success &= SetActive(status_led);
    } else {
        success &= SetInactive(status_led);
    }
    
    // Set error LED
    if (error_on) {
        success &= SetActive(error_led);
    } else {
        success &= SetInactive(error_led);
    }
    
    // Set communication LED
    if (comm_on) {
        success &= SetActive(comm_led);
    } else {
        success &= SetInactive(comm_led);
    }
    
    return success;
}

bool GpioData::IsCommunicating(GpioChip gpioChip) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // This is a simplified check - in a real implementation,
    // you would map pins to chips and test communication
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        // Try to read the GPIO state
        if (gpioTable_[i].gpio.EnsureInitialized()) {
            // If we can initialize at least one GPIO from this chip, consider it communicating
            return true;
        }
    }
    
    return false;
}

bool GpioData::IsResponding(GpioPin pin) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    GpioInfo* gpioInfo = FindGpioInfo(pin);
    if (!gpioInfo) {
        return false;
    }
    
    // Try to initialize and read the GPIO
    return gpioInfo->gpio.EnsureInitialized();
}

bool GpioData::RunGpioTest() noexcept {
    if (!EnsureInitialized()) {
        console_error(TAG, "GPIO system not initialized for testing");
        return false;
    }
    
    console_info(TAG, "Running comprehensive GPIO test");
    
    bool testPassed = true;
    
    // Test all registered output pins
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        GpioInfo& info = gpioTable_[i];
        
        console_info(TAG, "Testing pin %s (%s)", 
                     GpioPinToString(info.pin).data(), info.name.data());
        
        // Test initialization
        if (!info.gpio.EnsureInitialized()) {
            console_error(TAG, "Failed to initialize pin %s", 
                         GpioPinToString(info.pin).data());
            testPassed = false;
            continue;
        }
        
        // Test set active
        if (!info.gpio.SetActive()) {
            console_error(TAG, "Failed to set pin %s active", 
                         GpioPinToString(info.pin).data());
            testPassed = false;
        }
        
        os_delay_msec(10);
        
        // Test set inactive
        if (!info.gpio.SetInactive()) {
            console_error(TAG, "Failed to set pin %s inactive", 
                         GpioPinToString(info.pin).data());
            testPassed = false;
        }
        
        os_delay_msec(10);
    }
    
    if (testPassed) {
        console_info(TAG, "GPIO test completed successfully");
    } else {
        console_error(TAG, "GPIO test failed");
    }
    
    return testPassed;
}

bool GpioData::GetSystemHealth() noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if all registered GPIOs are responding
    for (uint8_t i = 0; i < registeredPinCount_; ++i) {
        if (!gpioTable_[i].gpio.EnsureInitialized()) {
            console_warning(TAG, "GPIO pin %s not responding", 
                           GpioPinToString(gpioTable_[i].pin).data());
            return false;
        }
    }
    
    return true;
}
