/**
 * @file Pcal95555Gpio.cpp
 * @brief Implementation of PCAL95555 GPIO expander integration.
 * 
 * This file implements the PCAL95555 GPIO expander wrapper classes that
 * integrate with the unified GPIO data handler system.
 */

#include "Pcal95555Gpio.h"
#include "GpioData.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-general/include/ConsolePort.h"

static const char* TAG = "Pcal95555Gpio";

//==============================================================================
// Pcal95555Chip Implementation
//==============================================================================

bool Pcal95555Chip::RegisterAllPins() noexcept {
    console_info(TAG, "Registering PCAL95555 pins with GpioData system...");

    // Pin mapping for single PCAL95555 chip
    const std::array<std::pair<GpioPin, std::string_view>, kPinCount> pinMapping = {{
        {GpioPin::GPIO_PCAL95555_PIN_0, "motor_enable_1"},
        {GpioPin::GPIO_PCAL95555_PIN_1, "motor_enable_2"},
        {GpioPin::GPIO_PCAL95555_PIN_2, "motor_brake_1"},
        {GpioPin::GPIO_PCAL95555_PIN_3, "motor_brake_2"},
        {GpioPin::GPIO_PCAL95555_PIN_4, "motor_fault_1"},
        {GpioPin::GPIO_PCAL95555_PIN_5, "motor_fault_2"},
        {GpioPin::GPIO_PCAL95555_PIN_6, "led_status_green"},
        {GpioPin::GPIO_PCAL95555_PIN_7, "led_status_red"},
        {GpioPin::GPIO_PCAL95555_PIN_8, "led_error"},
        {GpioPin::GPIO_PCAL95555_PIN_9, "led_comm"},
        {GpioPin::GPIO_PCAL95555_PIN_10, "external_relay_1"},
        {GpioPin::GPIO_PCAL95555_PIN_11, "external_relay_2"},
        {GpioPin::GPIO_PCAL95555_PIN_12, "external_output_1"},
        {GpioPin::GPIO_PCAL95555_PIN_13, "external_output_2"},
        {GpioPin::GPIO_PCAL95555_PIN_14, "external_input_1"},
        {GpioPin::GPIO_PCAL95555_PIN_15, "external_input_2"}
    }};

    GpioData& gpioData = GpioData::GetInstance();
    bool success = true;
    uint8_t registeredCount = 0;

    for (size_t i = 0; i < kPinCount; ++i) {
        if (gpioData.RegisterGpioPin(pinMapping[i].first, pins_[i], pinMapping[i].second)) {
            registeredCount++;
        } else {
            console_error(TAG, "Failed to register PCAL95555 pin %d (%s)", 
                         static_cast<int>(i), std::string(pinMapping[i].second).c_str());
            success = false;
        }
    }

    console_info(TAG, "PCAL95555 pin registration complete: %d/%d pins registered", 
                 registeredCount, kPinCount);
                 
    return success;
}

bool Pcal95555Chip::Initialize() noexcept {
    console_info(TAG, "Initializing PCAL95555 device...");

    // Initialize the PCAL95555 device
    if (!device_.initialize()) {
        console_error(TAG, "Failed to initialize PCAL95555 device");
        return false;
    }

    // Set up default pin configurations
    // Configure pins 0-3 as outputs (motor control)
    for (uint8_t i = 0; i <= 3; ++i) {
        if (!device_.setPinDirection(i, PCAL95555::GPIODir::Output)) {
            console_error(TAG, "Failed to set pin %d as output", i);
            return false;
        }
    }

    // Configure pins 4-5 as inputs (motor faults)
    for (uint8_t i = 4; i <= 5; ++i) {
        if (!device_.setPinDirection(i, PCAL95555::GPIODir::Input)) {
            console_error(TAG, "Failed to set pin %d as input", i);
            return false;
        }
    }

    // Configure pins 6-13 as outputs (LEDs, relays, external outputs)
    for (uint8_t i = 6; i <= 13; ++i) {
        if (!device_.setPinDirection(i, PCAL95555::GPIODir::Output)) {
            console_error(TAG, "Failed to set pin %d as output", i);
            return false;
        }
    }

    // Configure pins 14-15 as inputs (external inputs)
    for (uint8_t i = 14; i <= 15; ++i) {
        if (!device_.setPinDirection(i, PCAL95555::GPIODir::Input)) {
            console_error(TAG, "Failed to set pin %d as input", i);
            return false;
        }
    }

    // Set all outputs to inactive (low) state initially
    for (uint8_t i = 0; i <= 3; ++i) {
        device_.writePin(i, false);
    }
    for (uint8_t i = 6; i <= 13; ++i) {
        device_.writePin(i, false);
    }

    console_info(TAG, "PCAL95555 device initialized successfully");
    return true;
}

bool Pcal95555Chip::IsHealthy() noexcept {
    // Try to read a register to check communication
    // Read the Input Port register (0x00) as a health check
    uint8_t testData = 0;
    return device_.readRegister(0x00, &testData, 1);
}
