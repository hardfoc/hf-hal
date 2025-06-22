/**
 * @file GpioHandler.cpp
 * @brief Implementation of the main external GPIO interface for HardFOC.
 * 
 * This file implements the high-level GPIO interface that applications use
 * to interact with GPIO pins. It handles all initialization logic and
 * delegates data operations to the GpioData helper class.
 */

#include "GpioHandler.h"
#include "Pcal95555Gpio.h"
#include "Tmc9660Gpio.h"
#include "Esp32C6Gpio.h"
#include "hf_gpio_config.hpp"
#include "ConsolePort.h"
#include "OsUtility.h"

static const char* TAG = "GpioHandler";

//==============================================================================
// Singleton Implementation
//==============================================================================

GpioHandler& GpioHandler::GetInstance() noexcept {
    static GpioHandler instance;
    return instance;
}

//==============================================================================
// Initialization
//==============================================================================

bool GpioHandler::Initialize(SfI2cBus& i2cBus, Tmc9660MotorController& tmc9660Controller) noexcept {
    if (initialized_) {
        console_info(TAG, "GPIO Handler already initialized");
        return true;
    }

    console_info(TAG, "Initializing comprehensive GPIO system via GpioHandler...");

    // Store references to external systems
    i2cBus_ = &i2cBus;
    tmc9660Controller_ = &tmc9660Controller;    // Initialize all GPIO sources
    bool success = true;
    
    // 1. Initialize ESP32-C6 GPIO (safe pins only)
    console_info(TAG, "Initializing ESP32-C6 GPIO...");
    esp32GpioHealthy_ = InitializeEsp32Gpio();
    success &= esp32GpioHealthy_;
    
    // 2. Initialize PCAL95555 GPIO expander
    console_info(TAG, "Initializing PCAL95555 GPIO expander...");
    pcal95555Healthy_ = InitializePcal95555Gpio(i2cBus);
    success &= pcal95555Healthy_;
    
    // 3. Initialize TMC9660 GPIO
    console_info(TAG, "Initializing TMC9660 GPIO...");
    tmc9660GpioHealthy_ = InitializeTmc9660Gpio(tmc9660Controller);
    success &= tmc9660GpioHealthy_;

    if (success) {
        initialized_ = true;
        console_info(TAG, "GPIO system initialization complete - %u pins registered", 
                     GetGpioData().GetRegisteredPinCount());
    } else {
        console_error(TAG, "GPIO system initialization failed");
    }

    return success;
}

bool GpioHandler::IsInitialized() const noexcept {
    return initialized_;
}

//==============================================================================
// GPIO Initialization Methods
//==============================================================================

bool GpioHandler::InitializeEsp32Gpio() noexcept {
    console_info(TAG, "Creating ESP32-C6 GPIO instances for safe pins");
    
    if (!CreateEsp32GpioInstances()) {
        console_error(TAG, "Failed to create ESP32-C6 GPIO instances");
        return false;
    }
    
    if (!RegisterSafeEsp32Pins()) {
        console_error(TAG, "Failed to register ESP32-C6 GPIO pins");
        return false;
    }
    
    console_info(TAG, "ESP32-C6 GPIO initialization complete");
    return true;
}

bool GpioHandler::InitializePcal95555Gpio(SfI2cBus& i2cBus) noexcept {
    try {
        // Create PCAL95555 chip instance
        pcal95555Chip_ = std::make_unique<Pcal95555Chip>(i2cBus, 0x20);
        
        // Initialize the device
        if (!pcal95555Chip_->Initialize()) {
            console_error(TAG, "Failed to initialize PCAL95555 chip at address 0x20");
            return false;
        }
        
        // Register all pins with the GPIO system
        if (!pcal95555Chip_->RegisterAllPins()) {
            console_error(TAG, "Failed to register PCAL95555 pins");
            return false;
        }
        
        console_info(TAG, "PCAL95555 GPIO initialization complete");
        return true;
        
    } catch (const std::exception& e) {
        console_error(TAG, "Exception during PCAL95555 initialization: %s", e.what());
        return false;
    } catch (...) {
        console_error(TAG, "Unknown exception during PCAL95555 initialization");
        return false;
    }
}

bool GpioHandler::InitializeTmc9660Gpio(Tmc9660MotorController& tmc9660Controller) noexcept {
    try {
        // Create TMC9660 GPIO manager
        tmc9660Manager_ = std::make_unique<Tmc9660GpioManager>(tmc9660Controller);
        
        // Initialize the manager
        if (!tmc9660Manager_->Initialize()) {
            console_error(TAG, "Failed to initialize TMC9660 GPIO manager");
            return false;
        }
        
        // Register all TMC9660 GPIO pins
        if (!tmc9660Manager_->RegisterAllPins()) {
            console_error(TAG, "Failed to register TMC9660 GPIO pins");
            return false;
        }
        
        console_info(TAG, "TMC9660 GPIO initialization complete - %u pins registered", 
                     tmc9660Manager_->GetRegisteredPinCount());
        return true;
        
    } catch (const std::exception& e) {
        console_error(TAG, "Exception during TMC9660 GPIO initialization: %s", e.what());
        return false;
    } catch (...) {
        console_error(TAG, "Unknown exception during TMC9660 GPIO initialization");
        return false;
    }
}

bool GpioHandler::CreateEsp32GpioInstances() noexcept {
    // Create GPIO instances for safe ESP32-C6 pins only
    const gpio_num_t safePins[] = {
        GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_3,
        GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_20,
        GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23
    };
    
    const size_t numSafePins = sizeof(safePins) / sizeof(safePins[0]);
    
    try {
        esp32GpioInstances_.reserve(numSafePins);
        
        for (size_t i = 0; i < numSafePins; ++i) {
            gpio_num_t pin = safePins[i];
              // Validate pin safety using pin configuration header
            if (!IsPinSafeForGpio(pin)) {
                console_warning(TAG, "Pin GPIO_%d is not marked as safe - skipping", pin);
                continue;
            }
            
            // Create appropriate GPIO instance based on intended use
            // For now, create general output pins
            auto gpioInstance = std::make_unique<Esp32C6Output>(
                pin, DigitalGpio::ActiveState::High, DigitalGpio::State::Inactive
            );
            
            esp32GpioInstances_.push_back(std::move(gpioInstance));
        }
        
        console_info(TAG, "Created %zu ESP32-C6 GPIO instances", esp32GpioInstances_.size());
        return true;
        
    } catch (const std::exception& e) {
        console_error(TAG, "Exception creating ESP32-C6 GPIO instances: %s", e.what());
        return false;
    } catch (...) {
        console_error(TAG, "Unknown exception creating ESP32-C6 GPIO instances");
        return false;
    }
}

bool GpioHandler::RegisterSafeEsp32Pins() noexcept {
    // Map ESP32 pins to GpioPin enum values
    const struct {
        gpio_num_t esp32Pin;
        GpioPin gpioPin;
        const char* name;
    } esp32PinMap[] = {
        {GPIO_NUM_0, GpioPin::GPIO_ESP32_PIN_0, "esp32_gpio0"},
        {GPIO_NUM_1, GpioPin::GPIO_ESP32_PIN_1, "esp32_gpio1"},
        {GPIO_NUM_3, GpioPin::GPIO_ESP32_PIN_3, "esp32_gpio3"},
        {GPIO_NUM_10, GpioPin::GPIO_ESP32_PIN_10, "esp32_gpio10"},
        {GPIO_NUM_11, GpioPin::GPIO_ESP32_PIN_11, "esp32_gpio11"},
        {GPIO_NUM_20, GpioPin::GPIO_ESP32_PIN_20, "esp32_gpio20"},
        {GPIO_NUM_21, GpioPin::GPIO_ESP32_PIN_21, "esp32_gpio21"},
        {GPIO_NUM_22, GpioPin::GPIO_ESP32_PIN_22, "esp32_gpio22"},
        {GPIO_NUM_23, GpioPin::GPIO_ESP32_PIN_23, "esp32_gpio23"}
    };
    
    const size_t numMappings = sizeof(esp32PinMap) / sizeof(esp32PinMap[0]);
    bool success = true;
    
    for (size_t i = 0; i < esp32GpioInstances_.size() && i < numMappings; ++i) {
        DigitalGpio* gpio = esp32GpioInstances_[i].get();
        if (gpio && gpio->GetPin() == esp32PinMap[i].esp32Pin) {
            if (!GetGpioData().RegisterGpioPin(esp32PinMap[i].gpioPin, *gpio, esp32PinMap[i].name, 
                                              GpioChip::GPIO_ESP32_CHIP)) {
                console_error(TAG, "Failed to register ESP32 GPIO pin %s", esp32PinMap[i].name);
                success = false;
            }
        }
    }
    
    return success;
}

//==============================================================================
// Pin Control Methods
//==============================================================================

bool GpioHandler::SetActive(GpioPin pin) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().SetActive(pin);
}

bool GpioHandler::SetInactive(GpioPin pin) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().SetInactive(pin);
}

bool GpioHandler::Toggle(GpioPin pin) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().Toggle(pin);
}

bool GpioHandler::IsActive(GpioPin pin) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().IsActive(pin);
}

DigitalGpio::State GpioHandler::GetState(GpioPin pin) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return DigitalGpio::State::Inactive;
    }

    return IsActive(pin) ? DigitalGpio::State::Active : DigitalGpio::State::Inactive;
}

//==============================================================================
// Name-based Pin Control Methods
//==============================================================================

bool GpioHandler::SetActiveByName(std::string_view name) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().SetPinByName(name, true);
}

bool GpioHandler::SetInactiveByName(std::string_view name) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().SetPinByName(name, false);
}

bool GpioHandler::ToggleByName(std::string_view name) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().TogglePinByName(name);
}

bool GpioHandler::IsActiveByName(std::string_view name) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().GetPinByName(name);
}

//==============================================================================
// Batch Operations
//==============================================================================

uint8_t GpioHandler::SetMultipleActive(const GpioPin* pins, uint8_t count) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return 0;
    }

    return GetGpioData().SetMultiplePins(pins, count, true);
}

uint8_t GpioHandler::SetMultipleInactive(const GpioPin* pins, uint8_t count) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return 0;
    }

    return GetGpioData().SetMultiplePins(pins, count, false);
}

uint8_t GpioHandler::ReadMultiplePins(GpioReadSpec* specs, uint8_t count) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return 0;
    }

    return GetGpioData().ReadMultiplePins(specs, count);
}

//==============================================================================
// System Information Methods
//==============================================================================

const GpioInfo* GpioHandler::GetPinInfo(GpioPin pin) const noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return nullptr;
    }

    return GetGpioData().GetPinInfo(pin);
}

const GpioInfo* GpioHandler::GetPinInfoByName(std::string_view name) const noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return nullptr;
    }

    return GetGpioData().GetPinByName(name);
}

uint8_t GpioHandler::GetRegisteredPinCount() const noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return 0;
    }

    return GetGpioData().GetRegisteredPinCount();
}

bool GpioHandler::IsSourceCommunicating(GpioChip source) const noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().IsCommunicating(source);
}

GpioSystemHealth GpioHandler::GetSystemHealth() const noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return GpioSystemHealth{};
    }

    return GetGpioData().GetSystemHealth();
}

//==============================================================================
// Advanced Operations
//==============================================================================

bool GpioHandler::BlinkPin(GpioPin pin, uint32_t onTimeMs, uint32_t offTimeMs, uint8_t cycles) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().BlinkPin(pin, onTimeMs, offTimeMs, cycles);
}

bool GpioHandler::StopBlink(GpioPin pin) noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    return GetGpioData().StopBlink(pin);
}

bool GpioHandler::RunDiagnostics() noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    console_info(TAG, "Running GPIO system diagnostics...");
    
    // Run comprehensive diagnostics
    bool result = GetGpioData().RunDiagnostics();
    
    if (result) {
        console_info(TAG, "GPIO system diagnostics passed");
    } else {
        console_error(TAG, "GPIO system diagnostics failed");
    }
    
    return result;
}

bool GpioHandler::ResetAllPins() noexcept {
    if (!initialized_) {
        console_warn(TAG, "GPIO Handler not initialized");
        return false;
    }

    console_info(TAG, "Resetting all GPIO pins to default states...");
    return GetGpioData().ResetAllPins();
}

//==============================================================================
// Health Monitoring Methods
//==============================================================================

bool GpioHandler::IsEsp32GpioHealthy() const noexcept {
    return esp32GpioHealthy_;
}

bool GpioHandler::IsPcal95555Healthy() const noexcept {
    return pcal95555Healthy_;
}

bool GpioHandler::IsTmc9660GpioHealthy() const noexcept {
    return tmc9660GpioHealthy_;
}

bool GpioHandler::IsSystemHealthy() const noexcept {
    return esp32GpioHealthy_ && pcal95555Healthy_ && tmc9660GpioHealthy_;
}

//==============================================================================
// Private Methods
//==============================================================================

GpioData& GpioHandler::GetGpioData() noexcept {
    return GpioData::GetInstance();
}
