#ifndef HF_EXT_PINS_ENUM_HPP
#define HF_EXT_PINS_ENUM_HPP

/**
 * @file hf_ext_pins_enum.hpp
 * @brief External GPIO pin definitions for PCAL95555 and other GPIO expanders.
 *
 * This header defines all external GPIO pins used by the HardFOC controller
 * through GPIO expander chips like PCAL95555.
 */

#include <cstdint>

/**
 * @name PCAL95555 GPIO Expander Pin Definitions
 * These pins are accessed through the PCAL95555 16-bit I/O expander.
 */
///@{

/**
 * @brief PCAL95555 Chip 1 pin assignments
 */
enum class Pcal95555Chip1Pin : uint8_t {
    // Motor control related pins
    MOTOR_ENABLE_1      = 0,  ///< Motor 1 enable signal
    MOTOR_ENABLE_2      = 1,  ///< Motor 2 enable signal
    MOTOR_BRAKE_1       = 2,  ///< Motor 1 brake signal
    MOTOR_BRAKE_2       = 3,  ///< Motor 2 brake signal
    MOTOR_FAULT_1       = 4,  ///< Motor 1 fault input
    MOTOR_FAULT_2       = 5,  ///< Motor 2 fault input
    
    // Status LEDs
    LED_STATUS_GREEN    = 6,  ///< Green status LED
    LED_STATUS_RED      = 7,  ///< Red status LED
    LED_ERROR           = 8,  ///< Error indicator LED
    LED_COMM            = 9,  ///< Communication activity LED
    
    // External control signals
    EXT_RELAY_1         = 10, ///< External relay 1
    EXT_RELAY_2         = 11, ///< External relay 2
    EXT_OUTPUT_1        = 12, ///< General purpose output 1
    EXT_OUTPUT_2        = 13, ///< General purpose output 2
    
    // Input signals
    EXT_INPUT_1         = 14, ///< General purpose input 1
    EXT_INPUT_2         = 15, ///< General purpose input 2
};

/**
 * @brief PCAL95555 Chip 2 pin assignments (if used)
 */
enum class Pcal95555Chip2Pin : uint8_t {
    // Additional motor control
    MOTOR_DIR_1         = 0,  ///< Motor 1 direction
    MOTOR_DIR_2         = 1,  ///< Motor 2 direction
    MOTOR_STEP_1        = 2,  ///< Motor 1 step signal
    MOTOR_STEP_2        = 3,  ///< Motor 2 step signal
    
    // Fan control
    FAN_ENABLE_1        = 4,  ///< Fan 1 enable
    FAN_ENABLE_2        = 5,  ///< Fan 2 enable
    FAN_PWM_1           = 6,  ///< Fan 1 PWM control
    FAN_PWM_2           = 7,  ///< Fan 2 PWM control
    
    // Heater control
    HEATER_ENABLE       = 8,  ///< Heater enable signal
    HEATER_PWM          = 9,  ///< Heater PWM control
    
    // Valve control
    VALVE_1_OPEN        = 10, ///< Valve 1 open signal
    VALVE_1_CLOSE       = 11, ///< Valve 1 close signal
    VALVE_2_OPEN        = 12, ///< Valve 2 open signal
    VALVE_2_CLOSE       = 13, ///< Valve 2 close signal
    
    // Additional I/O
    AUX_OUTPUT_1        = 14, ///< Auxiliary output 1
    AUX_OUTPUT_2        = 15, ///< Auxiliary output 2
};

/**
 * @brief Functional mapping of external pins to logical names
 */
namespace ExtPinMap {
    // Motor control pin mappings
    constexpr auto MOTOR_ENABLE = Pcal95555Chip1Pin::MOTOR_ENABLE_1;
    constexpr auto MOTOR_FAULT = Pcal95555Chip1Pin::MOTOR_FAULT_1;
    constexpr auto MOTOR_BRAKE = Pcal95555Chip1Pin::MOTOR_BRAKE_1;
    
    // LED pin mappings
    constexpr auto LED_STATUS = Pcal95555Chip1Pin::LED_STATUS_GREEN;
    constexpr auto LED_ERROR = Pcal95555Chip1Pin::LED_ERROR;
    constexpr auto LED_COMM = Pcal95555Chip1Pin::LED_COMM;
    
    // External I/O mappings
    constexpr auto USER_OUTPUT_1 = Pcal95555Chip1Pin::EXT_OUTPUT_1;
    constexpr auto USER_OUTPUT_2 = Pcal95555Chip1Pin::EXT_OUTPUT_2;
    constexpr auto USER_INPUT_1 = Pcal95555Chip1Pin::EXT_INPUT_1;
    constexpr auto USER_INPUT_2 = Pcal95555Chip1Pin::EXT_INPUT_2;
}

/**
 * @brief I2C addresses for PCAL95555 chips
 */
namespace Pcal95555Addresses {
    constexpr uint8_t CHIP_1_ADDR = 0x20; ///< I2C address for first PCAL95555
    constexpr uint8_t CHIP_2_ADDR = 0x21; ///< I2C address for second PCAL95555
}

/**
 * @brief Pin direction configurations for PCAL95555 pins
 */
namespace Pcal95555Config {
    // Chip 1 configuration - bit mask for input pins (1 = input, 0 = output)
    constexpr uint16_t CHIP_1_INPUT_MASK = 
        (1 << static_cast<uint8_t>(Pcal95555Chip1Pin::MOTOR_FAULT_1)) |
        (1 << static_cast<uint8_t>(Pcal95555Chip1Pin::MOTOR_FAULT_2)) |
        (1 << static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_INPUT_1)) |
        (1 << static_cast<uint8_t>(Pcal95555Chip1Pin::EXT_INPUT_2));
    
    // Chip 2 configuration - all pins are outputs by default
    constexpr uint16_t CHIP_2_INPUT_MASK = 0x0000;
}

/**
 * @brief String representations for external pins
 */
constexpr const char* Pcal95555Chip1PinToString(Pcal95555Chip1Pin pin) {
    switch (pin) {
        case Pcal95555Chip1Pin::MOTOR_ENABLE_1: return "MOTOR_ENABLE_1";
        case Pcal95555Chip1Pin::MOTOR_ENABLE_2: return "MOTOR_ENABLE_2";
        case Pcal95555Chip1Pin::MOTOR_BRAKE_1: return "MOTOR_BRAKE_1";
        case Pcal95555Chip1Pin::MOTOR_BRAKE_2: return "MOTOR_BRAKE_2";
        case Pcal95555Chip1Pin::MOTOR_FAULT_1: return "MOTOR_FAULT_1";
        case Pcal95555Chip1Pin::MOTOR_FAULT_2: return "MOTOR_FAULT_2";
        case Pcal95555Chip1Pin::LED_STATUS_GREEN: return "LED_STATUS_GREEN";
        case Pcal95555Chip1Pin::LED_STATUS_RED: return "LED_STATUS_RED";
        case Pcal95555Chip1Pin::LED_ERROR: return "LED_ERROR";
        case Pcal95555Chip1Pin::LED_COMM: return "LED_COMM";
        case Pcal95555Chip1Pin::EXT_RELAY_1: return "EXT_RELAY_1";
        case Pcal95555Chip1Pin::EXT_RELAY_2: return "EXT_RELAY_2";
        case Pcal95555Chip1Pin::EXT_OUTPUT_1: return "EXT_OUTPUT_1";
        case Pcal95555Chip1Pin::EXT_OUTPUT_2: return "EXT_OUTPUT_2";
        case Pcal95555Chip1Pin::EXT_INPUT_1: return "EXT_INPUT_1";
        case Pcal95555Chip1Pin::EXT_INPUT_2: return "EXT_INPUT_2";
        default: return "UNKNOWN_CHIP1_PIN";
    }
}

constexpr const char* Pcal95555Chip2PinToString(Pcal95555Chip2Pin pin) {
    switch (pin) {
        case Pcal95555Chip2Pin::MOTOR_DIR_1: return "MOTOR_DIR_1";
        case Pcal95555Chip2Pin::MOTOR_DIR_2: return "MOTOR_DIR_2";
        case Pcal95555Chip2Pin::MOTOR_STEP_1: return "MOTOR_STEP_1";
        case Pcal95555Chip2Pin::MOTOR_STEP_2: return "MOTOR_STEP_2";
        case Pcal95555Chip2Pin::FAN_ENABLE_1: return "FAN_ENABLE_1";
        case Pcal95555Chip2Pin::FAN_ENABLE_2: return "FAN_ENABLE_2";
        case Pcal95555Chip2Pin::FAN_PWM_1: return "FAN_PWM_1";
        case Pcal95555Chip2Pin::FAN_PWM_2: return "FAN_PWM_2";
        case Pcal95555Chip2Pin::HEATER_ENABLE: return "HEATER_ENABLE";
        case Pcal95555Chip2Pin::HEATER_PWM: return "HEATER_PWM";
        case Pcal95555Chip2Pin::VALVE_1_OPEN: return "VALVE_1_OPEN";
        case Pcal95555Chip2Pin::VALVE_1_CLOSE: return "VALVE_1_CLOSE";
        case Pcal95555Chip2Pin::VALVE_2_OPEN: return "VALVE_2_OPEN";
        case Pcal95555Chip2Pin::VALVE_2_CLOSE: return "VALVE_2_CLOSE";
        case Pcal95555Chip2Pin::AUX_OUTPUT_1: return "AUX_OUTPUT_1";
        case Pcal95555Chip2Pin::AUX_OUTPUT_2: return "AUX_OUTPUT_2";
        default: return "UNKNOWN_CHIP2_PIN";
    }
}

///@}

#endif // HF_EXT_PINS_ENUM_HPP