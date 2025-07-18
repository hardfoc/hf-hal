/**
 * @file Pcal95555Example.cpp
 * @brief Example usage of the PCAL95555 GPIO wrapper with BaseGpio interface.
 *
 * This file demonstrates how to properly use the new PCAL95555 GPIO wrapper
 * that implements the BaseGpio interface and integrates with the ESP32 I2C system
 * and platform mapping configuration.
 *
 * @note THIS EXAMPLE NEEDS TO BE UPDATED to use the new CommChannelsManager
 *       and EspI2cBus/EspI2cDevice architecture instead of the old EspI2c class.
 *       The example currently uses the deprecated EspI2c class directly.
 * 
 * @todo Update to use CommChannelsManager::GetGpioExpander() API
 *
 * @author HardFOC Team
 * @version 2.0
 * @date 2025
 * @copyright HardFOC
 */

#include "component-handler/Pcal95555GpioWrapper.h"
#include "mcu/esp32/EspI2c.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-general/include/ConsolePort.h"
#include <memory>
#include <thread>
#include <chrono>

static const char* TAG = "Pcal95555Example";

/**
 * @brief Example function demonstrating PCAL95555 GPIO wrapper usage.
 * 
 * This function shows how to:
 * 1. Initialize the ESP32 I2C bus
 * 2. Create and initialize the PCAL95555 wrapper
 * 3. Create GPIO pin instances using functional pin identifiers
 * 4. Perform GPIO operations using the BaseGpio interface
 * 5. Handle errors and diagnostics
 */
void DemonstratePcal95555Usage() {
    console_info(TAG, "=== PCAL95555 GPIO Wrapper Example ===");
    
    //==============================================================================
    // STEP 1: Initialize ESP32 I2C Bus
    //==============================================================================
    
    console_info(TAG, "Step 1: Initializing ESP32 I2C bus...");
    
    // Create ESP32 I2C instance (using default I2C port 0)
    auto esp32_i2c = std::make_shared<EspI2c>();
    if (!esp32_i2c) {
        console_error(TAG, "Failed to create ESP32 I2C bus");
        return;
    }
    
    // Initialize I2C bus with standard configuration
    if (!esp32_i2c->Initialize()) {
        console_error(TAG, "Failed to initialize ESP32 I2C bus");
        return;
    }
    
    console_info(TAG, "ESP32 I2C bus initialized successfully");
    
    //==============================================================================
    // STEP 2: Create and Initialize PCAL95555 Wrapper
    //==============================================================================
    
    console_info(TAG, "Step 2: Creating PCAL95555 GPIO wrapper...");
    
    // Create PCAL95555 wrapper with BaseI2c interface (platform-agnostic)
    // The wrapper only knows about BaseI2c, not EspI2c specifically
    auto pcal95555_wrapper = CreatePcal95555GpioWrapper(*esp32_i2c, GetDefaultPcal95555I2cAddress());
    if (!pcal95555_wrapper) {
        console_error(TAG, "Failed to create PCAL95555 GPIO wrapper");
        return;
    }
    
    // Check if wrapper is healthy
    if (!pcal95555_wrapper->IsHealthy()) {
        console_error(TAG, "PCAL95555 chip is not responding");
        return;
    }
    
    console_info(TAG, "PCAL95555 GPIO wrapper created and healthy");
    
    //==============================================================================
    // STEP 3: Create GPIO Pin Instances Using Functional Pin Identifiers
    //==============================================================================
    
    console_info(TAG, "Step 3: Creating GPIO pin instances...");
    
    // Create GPIO pins using functional pin identifiers (platform-agnostic)
    auto motor_enable_pin = pcal95555_wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
    auto motor_brake_pin = pcal95555_wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_BRAKE);
    auto led_status_pin = pcal95555_wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::LED_STATUS_OK);
    auto fault_status_pin = pcal95555_wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_FAULT_STATUS);
    
    if (!motor_enable_pin || !motor_brake_pin || !led_status_pin || !fault_status_pin) {
        console_error(TAG, "Failed to create one or more GPIO pin instances");
        return;
    }
    
    console_info(TAG, "GPIO pin instances created successfully");
    
    //==============================================================================
    // STEP 4: Perform GPIO Operations Using BaseGpio Interface
    //==============================================================================
    
    console_info(TAG, "Step 4: Performing GPIO operations...");
    
    // Initialize all pins (lazy initialization)
    if (!motor_enable_pin->EnsureInitialized() ||
        !motor_brake_pin->EnsureInitialized() ||
        !led_status_pin->EnsureInitialized() ||
        !fault_status_pin->EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO pins");
        return;
    }
    
    // Set output pins to inactive state initially
    hf_gpio_err_t result;
    
    result = motor_enable_pin->SetInactive();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        console_error(TAG, "Failed to set motor enable inactive: %s", HfGpioErrToString(result));
    }
    
    result = motor_brake_pin->SetInactive();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        console_error(TAG, "Failed to set motor brake inactive: %s", HfGpioErrToString(result));
    }
    
    result = led_status_pin->SetInactive();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        console_error(TAG, "Failed to set LED status inactive: %s", HfGpioErrToString(result));
    }
    
    console_info(TAG, "All output pins set to inactive state");
    
    //==============================================================================
    // STEP 5: Demonstrate GPIO Operations
    //==============================================================================
    
    console_info(TAG, "Step 5: Demonstrating GPIO operations...");
    
    // Read fault status (input pin)
    bool fault_active = false;
    result = fault_status_pin->IsActive(fault_active);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        console_info(TAG, "Motor fault status: %s", fault_active ? "FAULT" : "OK");
    } else {
        console_error(TAG, "Failed to read fault status: %s", HfGpioErrToString(result));
    }
    
    // Toggle LED status
    console_info(TAG, "Toggling LED status pin...");
    result = led_status_pin->Toggle();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        console_error(TAG, "Failed to toggle LED: %s", HfGpioErrToString(result));
    } else {
        console_info(TAG, "LED status toggled successfully");
    }
    
    // Enable motor
    console_info(TAG, "Enabling motor...");
    result = motor_enable_pin->SetActive();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        console_error(TAG, "Failed to enable motor: %s", HfGpioErrToString(result));
    } else {
        console_info(TAG, "Motor enabled successfully");
    }
    
    // Wait a moment
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Disable motor
    console_info(TAG, "Disabling motor...");
    result = motor_enable_pin->SetInactive();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        console_error(TAG, "Failed to disable motor: %s", HfGpioErrToString(result));
    } else {
        console_info(TAG, "Motor disabled successfully");
    }
    
    // Toggle LED back
    result = led_status_pin->Toggle();
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        console_info(TAG, "LED status toggled back");
    }
    
    //==============================================================================
    // STEP 6: Demonstrate Configuration Changes
    //==============================================================================
    
    console_info(TAG, "Step 6: Demonstrating configuration changes...");
    
    // Change LED pin to open-drain mode
    result = led_status_pin->SetOutputMode(hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        console_info(TAG, "LED pin configured for open-drain mode");
    } else {
        console_error(TAG, "Failed to set LED to open-drain: %s", HfGpioErrToString(result));
    }
    
    // Change fault pin pull mode
    result = fault_status_pin->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        console_info(TAG, "Fault pin pull-up enabled");
    } else {
        console_error(TAG, "Failed to set fault pin pull-up: %s", HfGpioErrToString(result));
    }
    
    //==============================================================================
    // STEP 7: Demonstrate Diagnostics
    //==============================================================================
    
    console_info(TAG, "Step 7: Demonstrating diagnostics...");
    
    Pcal95555GpioWrapper::Diagnostics diagnostics;
    if (pcal95555_wrapper->GetDiagnostics(diagnostics)) {
        console_info(TAG, "PCAL95555 Diagnostics:");
        console_info(TAG, "  Chip initialized: %s", diagnostics.chip_initialized ? "Yes" : "No");
        console_info(TAG, "  Chip responsive: %s", diagnostics.chip_responsive ? "Yes" : "No");
        console_info(TAG, "  Error flags: 0x%04X", diagnostics.error_flags);
        console_info(TAG, "  Interrupt status: 0x%04X", diagnostics.interrupt_status);
        console_info(TAG, "  Input Port 0: 0x%02X", diagnostics.input_port_0);
        console_info(TAG, "  Input Port 1: 0x%02X", diagnostics.input_port_1);
        console_info(TAG, "  Output Port 0: 0x%02X", diagnostics.output_port_0);
        console_info(TAG, "  Output Port 1: 0x%02X", diagnostics.output_port_1);
        console_info(TAG, "  Config Port 0: 0x%02X", diagnostics.config_port_0);
        console_info(TAG, "  Config Port 1: 0x%02X", diagnostics.config_port_1);
    } else {
        console_error(TAG, "Failed to get diagnostics");
    }
    
    //==============================================================================
    // STEP 8: Demonstrate Pin Information
    //==============================================================================
    
    console_info(TAG, "Step 8: Pin information:");
    
    console_info(TAG, "Motor Enable Pin:");
    console_info(TAG, "  Functional Pin: %d", static_cast<int>(motor_enable_pin->GetFunctionalPin()));
    console_info(TAG, "  Chip Pin: %d", static_cast<int>(motor_enable_pin->GetChipPin()));
    console_info(TAG, "  Direction: %s", BaseGpio::ToString(motor_enable_pin->GetDirection()));
    console_info(TAG, "  Active State: %s", BaseGpio::ToString(motor_enable_pin->GetActiveState()));
    console_info(TAG, "  Output Mode: %s", BaseGpio::ToString(motor_enable_pin->GetOutputMode()));
    console_info(TAG, "  Pull Mode: %s", BaseGpio::ToString(motor_enable_pin->GetPullMode()));
    console_info(TAG, "  Description: %s", motor_enable_pin->GetDescription());
    
    console_info(TAG, "Fault Status Pin:");
    console_info(TAG, "  Functional Pin: %d", static_cast<int>(fault_status_pin->GetFunctionalPin()));
    console_info(TAG, "  Chip Pin: %d", static_cast<int>(fault_status_pin->GetChipPin()));
    console_info(TAG, "  Direction: %s", BaseGpio::ToString(fault_status_pin->GetDirection()));
    console_info(TAG, "  Active State: %s", BaseGpio::ToString(fault_status_pin->GetActiveState()));
    console_info(TAG, "  Supports Interrupts: %s", fault_status_pin->SupportsInterrupts() ? "Yes" : "No");
    
    //==============================================================================
    // STEP 9: Demonstrate Error Handling
    //==============================================================================
    
    console_info(TAG, "Step 9: Demonstrating error handling...");
    
    // Try to create a pin with invalid functional pin
    auto invalid_pin = pcal95555_wrapper->CreateGpioPin(static_cast<HardFOC::FunctionalGpioPin>(255));
    if (!invalid_pin) {
        console_info(TAG, "Correctly rejected invalid functional pin (expected behavior)");
    } else {
        console_warning(TAG, "Unexpectedly accepted invalid functional pin");
    }
    
    // Try to create a pin with invalid chip pin
    auto invalid_chip_pin = pcal95555_wrapper->CreateGpioPin(static_cast<Pcal95555Chip1Pin>(255));
    if (!invalid_chip_pin) {
        console_info(TAG, "Correctly rejected invalid chip pin (expected behavior)");
    } else {
        console_warning(TAG, "Unexpectedly accepted invalid chip pin");
    }
    
    //==============================================================================
    // STEP 10: Cleanup
    //==============================================================================
    
    console_info(TAG, "Step 10: Cleaning up...");
    
    // Deinitialize pins (optional, will be done automatically when objects are destroyed)
    motor_enable_pin->Deinitialize();
    motor_brake_pin->Deinitialize();
    led_status_pin->Deinitialize();
    fault_status_pin->Deinitialize();
    
    // Deinitialize wrapper
    pcal95555_wrapper->Deinitialize();
    
    // Deinitialize I2C bus
    esp32_i2c->Deinitialize();
    
    console_info(TAG, "=== PCAL95555 GPIO Wrapper Example Completed ===");
}

/**
 * @brief Example function demonstrating direct chip pin access.
 * 
 * This function shows how to create GPIO pins using direct chip pin numbers
 * instead of functional pin identifiers.
 */
void DemonstrateDirectChipPinAccess() {
    console_info(TAG, "=== Direct Chip Pin Access Example ===");
    
    // Create I2C bus and wrapper (simplified)
    auto esp32_i2c = std::make_shared<EspI2c>();
    if (!esp32_i2c || !esp32_i2c->Initialize()) {
        console_error(TAG, "Failed to initialize I2C bus");
        return;
    }
    
    // Create wrapper using BaseI2c interface (platform-agnostic)
    auto pcal95555_wrapper = CreatePcal95555GpioWrapper(*esp32_i2c);
    if (!pcal95555_wrapper || !pcal95555_wrapper->IsHealthy()) {
        console_error(TAG, "Failed to create PCAL95555 wrapper");
        return;
    }
    
    // Create pins using direct chip pin numbers
    auto pin_0 = pcal95555_wrapper->CreateGpioPin(Pcal95555Chip1Pin::MOTOR_ENABLE_1);
    auto pin_1 = pcal95555_wrapper->CreateGpioPin(Pcal95555Chip1Pin::MOTOR_BRAKE_1);
    auto pin_6 = pcal95555_wrapper->CreateGpioPin(Pcal95555Chip1Pin::LED_STATUS_GREEN);
    
    if (!pin_0 || !pin_1 || !pin_6) {
        console_error(TAG, "Failed to create chip pins");
        return;
    }
    
    // Configure pins
    pin_0->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    pin_1->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    pin_6->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    
    // Perform operations
    pin_0->SetActive();   // Enable motor
    pin_1->SetInactive(); // Release brake
    pin_6->Toggle();      // Toggle LED
    
    console_info(TAG, "Direct chip pin access completed");
    
    // Cleanup
    pcal95555_wrapper->Deinitialize();
    esp32_i2c->Deinitialize();
}

/**
 * @brief Example function demonstrating platform-agnostic usage.
 * 
 * This function shows how the wrapper can be used with any BaseI2c implementation,
 * making it platform-agnostic.
 */
void DemonstratePlatformAgnosticUsage(BaseI2c& i2c_bus) {
    console_info(TAG, "=== Platform-Agnostic Usage Example ===");
    
    // Create wrapper using any BaseI2c implementation
    auto pcal95555_wrapper = CreatePcal95555GpioWrapper(i2c_bus);
    if (!pcal95555_wrapper) {
        console_error(TAG, "Failed to create PCAL95555 wrapper");
        return;
    }
    
    // Use the wrapper exactly the same way regardless of the underlying I2C implementation
    auto motor_enable_pin = pcal95555_wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
    if (motor_enable_pin) {
        motor_enable_pin->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        motor_enable_pin->SetActive();
        console_info(TAG, "Motor enabled using platform-agnostic interface");
        motor_enable_pin->SetInactive();
    }
    
    console_info(TAG, "Platform-agnostic usage completed");
    
    // Cleanup
    pcal95555_wrapper->Deinitialize();
}

/**
 * @brief Main example function that runs all demonstrations.
 */
void RunPcal95555Examples() {
    console_info(TAG, "Starting PCAL95555 GPIO wrapper examples...");
    
    // Run the main demonstration
    DemonstratePcal95555Usage();
    
    // Run the direct chip pin access demonstration
    DemonstrateDirectChipPinAccess();
    
    // Run the platform-agnostic demonstration
    // Note: This would be called with any BaseI2c implementation
    // For this example, we'll create an ESP32 I2C instance
    auto esp32_i2c = std::make_shared<EspI2c>();
    if (esp32_i2c && esp32_i2c->Initialize()) {
        DemonstratePlatformAgnosticUsage(*esp32_i2c);
        esp32_i2c->Deinitialize();
    }
    
    console_info(TAG, "All PCAL95555 examples completed");
} 