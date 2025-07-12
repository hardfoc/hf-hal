#ifndef COMPONENT_HANDLER_COMMON_IDS_H_
#define COMPONENT_HANDLER_COMMON_IDS_H_

#include "../utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include <cstdint>
#include <string_view>

/**
 * @file CommonIDs.h
 * @brief Modernized hardware-agnostic identifiers for the HardFOC system.
 * 
 * This file provides the core type definitions and error handling using X-macros
 * for the HardFOC HAL system. It bridges between the low-level platform mapping
 * and the high-level HAL interfaces.
 * 
 * Key Features:
 * - X-macro based error code definitions
 * - Hardware-agnostic functional identifiers
 * - Legacy compatibility aliases
 * - Clean, maintainable enumeration patterns
 * 
 * To port to a new microcontroller platform:
 * 1. Update the hardware mapping in utils-and-drivers/hf-pincfg/hf_platform_mapping.hpp
 * 2. Ensure appropriate drivers exist in utils-and-drivers/hf-core-drivers/
 * 3. Application code using these identifiers remains unchanged
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

//==============================================================================
// CORE TYPE DEFINITIONS
//==============================================================================

/**
 * @brief ADC input sensor/channel identifier.
 * 
 * Modern functional identifier for ADC channels. Use this instead of
 * hardware-specific channel numbers.
 */
using AdcInputSensor = HardFOC::FunctionalAdcChannel;

/**
 * @brief GPIO pin functional identifier.
 * 
 * Modern functional identifier for GPIO pins. Use this instead of
 * hardware-specific pin numbers.
 */
using GpioPin = HardFOC::FunctionalGpioPin;

//==============================================================================
// LEGACY ERROR ENUMS WITH X-MACRO PATTERN
//==============================================================================

/**
 * @brief X-macro definition for legacy GPIO error codes.
 * 
 * @deprecated Use Result from Result.h instead.
 * 
 * Format: X(ENUM_NAME, numeric_value, "description_string")
 */
#define GPIO_ERROR_CODES(X) \
    X(GPIO_SUCCESS,           0,   "GPIO operation successful") \
    X(GPIO_ERROR_GENERIC,     1,   "Generic GPIO error") \
    X(GPIO_ERROR_INVALID_PIN, 2,   "Invalid GPIO pin specified") \
    X(GPIO_ERROR_NOT_INIT,    3,   "GPIO system not initialized") \
    X(GPIO_ERROR_READ_FAIL,   4,   "GPIO read operation failed") \
    X(GPIO_ERROR_WRITE_FAIL,  5,   "GPIO write operation failed") \
    X(GPIO_ERROR_CONFIG_FAIL, 6,   "GPIO configuration failed") \
    X(GPIO_ERROR_TIMEOUT,     7,   "GPIO operation timeout") \
    X(GPIO_ERROR_BUSY,        8,   "GPIO resource busy") \
    X(GPIO_ERROR_NO_MEMORY,   9,   "GPIO memory allocation failed")

/**
 * @brief X-macro definition for legacy ADC error codes.
 * 
 * @deprecated Use Result from Result.h instead.
 * 
 * Format: X(ENUM_NAME, numeric_value, "description_string")
 */
#define ADC_ERROR_CODES(X) \
    X(ADC_SUCCESS,            0,   "ADC operation successful") \
    X(ADC_ERROR_GENERIC,      1,   "Generic ADC error") \
    X(ADC_ERROR_INVALID_CH,   2,   "Invalid ADC channel specified") \
    X(ADC_ERROR_NOT_INIT,     3,   "ADC system not initialized") \
    X(ADC_ERROR_READ_FAIL,    4,   "ADC read operation failed") \
    X(ADC_ERROR_CONVERSION,   5,   "ADC conversion failed") \
    X(ADC_ERROR_CALIBRATION,  6,   "ADC calibration failed") \
    X(ADC_ERROR_OUT_OF_RANGE, 7,   "ADC reading out of range") \
    X(ADC_ERROR_TIMEOUT,      8,   "ADC operation timeout") \
    X(ADC_ERROR_NO_MEMORY,    9,   "ADC memory allocation failed")

/**
 * @brief Legacy GPIO error enumeration.
 * @deprecated Use HardFocResult from HardFocResult.h instead.
 */
enum class GpioError : uint8_t {
#define X(name, value, desc) name = value,
    GPIO_ERROR_CODES(X)
#undef X
};

/**
 * @brief Legacy ADC error enumeration.
 * @deprecated Use Result from Result.h instead.
 */
enum class AdcError : uint8_t {
#define X(name, value, desc) name = value,
    ADC_ERROR_CODES(X)
#undef X
};

/**
 * @brief Get description for GPIO error code.
 * @deprecated Use GetResultDescription() from Result.h instead.
 */
[[nodiscard]] constexpr std::string_view GetGpioErrorDescription(GpioError error) noexcept {
    switch (error) {
#define X(name, value, desc) case GpioError::name: return desc;
        GPIO_ERROR_CODES(X)
#undef X
        default: return "Unknown GPIO error";
    }
}

/**
 * @brief Get description for ADC error code.
 * @deprecated Use GetResultDescription() from Result.h instead.
 */
[[nodiscard]] constexpr std::string_view GetAdcErrorDescription(AdcError error) noexcept {
    switch (error) {
#define X(name, value, desc) case AdcError::name: return desc;
        ADC_ERROR_CODES(X)
#undef X
        default: return "Unknown ADC error";
    }
}

//==============================================================================
// LEGACY COMPATIBILITY ALIASES
//==============================================================================

/**
 * @brief Legacy ADC channel aliases for backward compatibility.
 * @deprecated Use HardFOC::FunctionalAdcChannel values directly.
 */
namespace LegacyAdcChannels {
    constexpr auto ADC_MOTOR_CURRENT_PHASE_A = HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_A;
    constexpr auto ADC_MOTOR_CURRENT_PHASE_B = HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_B;
    constexpr auto ADC_MOTOR_CURRENT_PHASE_C = HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_C;
    constexpr auto ADC_SYSTEM_VOLTAGE_3V3 = HardFOC::FunctionalAdcChannel::SYSTEM_VOLTAGE_3V3;
    constexpr auto ADC_SYSTEM_VOLTAGE_5V = HardFOC::FunctionalAdcChannel::SYSTEM_VOLTAGE_5V;
    constexpr auto ADC_SYSTEM_VOLTAGE_12V = HardFOC::FunctionalAdcChannel::SYSTEM_VOLTAGE_12V;
    constexpr auto ADC_SYSTEM_TEMPERATURE_AMBIENT = HardFOC::FunctionalAdcChannel::SYSTEM_TEMPERATURE_AMBIENT;
    constexpr auto ADC_MOTOR_TEMPERATURE = HardFOC::FunctionalAdcChannel::MOTOR_TEMPERATURE;
    constexpr auto ADC_USER_ANALOG_INPUT_1 = HardFOC::FunctionalAdcChannel::USER_ANALOG_INPUT_1;
    constexpr auto ADC_USER_ANALOG_INPUT_2 = HardFOC::FunctionalAdcChannel::USER_ANALOG_INPUT_2;
    constexpr auto ADC_SYSTEM_VREF_INTERNAL = HardFOC::FunctionalAdcChannel::SYSTEM_VREF_INTERNAL;
}

/**
 * @brief Legacy GPIO pin aliases for backward compatibility.
 * @deprecated Use HardFOC::FunctionalGpioPin values directly.
 */
namespace LegacyGpioPins {
    constexpr auto GPIO_MOTOR_ENABLE = HardFOC::FunctionalGpioPin::MOTOR_ENABLE;
    constexpr auto GPIO_MOTOR_BRAKE = HardFOC::FunctionalGpioPin::MOTOR_BRAKE;
    constexpr auto GPIO_MOTOR_FAULT_STATUS = HardFOC::FunctionalGpioPin::MOTOR_FAULT_STATUS;
    constexpr auto GPIO_LED_STATUS_OK = HardFOC::FunctionalGpioPin::LED_STATUS_OK;
    constexpr auto GPIO_LED_STATUS_ERROR = HardFOC::FunctionalGpioPin::LED_STATUS_ERROR;
    constexpr auto GPIO_LED_STATUS_COMM = HardFOC::FunctionalGpioPin::LED_STATUS_COMM;
    constexpr auto GPIO_USER_OUTPUT_1 = HardFOC::FunctionalGpioPin::USER_OUTPUT_1;
    constexpr auto GPIO_USER_OUTPUT_2 = HardFOC::FunctionalGpioPin::USER_OUTPUT_2;
    constexpr auto GPIO_USER_INPUT_1 = HardFOC::FunctionalGpioPin::USER_INPUT_1;
    constexpr auto GPIO_USER_INPUT_2 = HardFOC::FunctionalGpioPin::USER_INPUT_2;
}

//==============================================================================
// CHIP TYPE ENUMERATIONS
//==============================================================================

/**
 * @brief Enumeration for ADC chip types.
 */
enum class AdcChip : uint8_t {
    ADC_ESP32_INTERNAL = 0,  ///< ESP32-C6 internal ADC units
    ADC_TMC9660,             ///< TMC9660 motor controller ADC
    ADC_CHIP_COUNT           ///< Total number of ADC chip types
};

/**
 * @brief Enumeration for GPIO chip types.
 */
enum class GpioChip : uint8_t {
    GPIO_ESP32_INTERNAL = 0, ///< ESP32-C6 native GPIO
    GPIO_PCAL95555,          ///< PCAL95555 I2C GPIO expander
    GPIO_TMC9660,            ///< TMC9660 integrated GPIO
    GPIO_CHIP_COUNT          ///< Total number of GPIO chip types
};

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

/**
 * @brief Get the string name for an ADC chip type.
 * @param chip The ADC chip type
 * @return String name of the chip
 */
[[nodiscard]] constexpr std::string_view GetAdcChipName(AdcChip chip) noexcept {
    switch (chip) {
        case AdcChip::ADC_ESP32_INTERNAL: return "ESP32_INTERNAL";
        case AdcChip::ADC_TMC9660: return "TMC9660";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Get the string name for a GPIO chip type.
 * @param chip The GPIO chip type
 * @return String name of the chip
 */
[[nodiscard]] constexpr std::string_view GetGpioChipName(GpioChip chip) noexcept {
    switch (chip) {
        case GpioChip::GPIO_ESP32_INTERNAL: return "ESP32_INTERNAL";
        case GpioChip::GPIO_PCAL95555: return "PCAL95555";
        case GpioChip::GPIO_TMC9660: return "TMC9660";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Check if an ADC chip type is valid.
 * @param chip The ADC chip type to check
 * @return true if valid, false otherwise
 */
[[nodiscard]] constexpr bool IsValidAdcChip(AdcChip chip) noexcept {
    return chip < AdcChip::ADC_CHIP_COUNT;
}

/**
 * @brief Check if a GPIO chip type is valid.
 * @param chip The GPIO chip type to check
 * @return true if valid, false otherwise
 */
[[nodiscard]] constexpr bool IsValidGpioChip(GpioChip chip) noexcept {
    return chip < GpioChip::GPIO_CHIP_COUNT;
}

#endif // COMPONENT_HANDLER_COMMON_IDS_H_

//==============================================================================
// HARDWARE ABSTRACTION ENUMS (For backward compatibility)
//==============================================================================

/**
 * @brief GPIO chip source identifiers.
 */
enum class GpioChip : uint8_t {
    ESP32_NATIVE = 0,        ///< ESP32-C6 native GPIO pins
    PCAL95555_EXPANDER,      ///< PCAL95555 I2C GPIO expander
    TMC9660_CONTROLLER,      ///< TMC9660 motor controller GPIO pins
    
    GPIO_CHIP_COUNT
};

/**
 * @brief TMC9660 chip instance identifiers.
 */
enum class Tmc9660ChipId : uint8_t {
    TMC9660_CHIP_1 = 0,      ///< Primary TMC9660 on SPI0/UART0
    TMC9660_CHIP_2,          ///< Secondary TMC9660 (future expansion)
    TMC9660_CHIP_3,          ///< Third TMC9660 (future expansion)
    TMC9660_CHIP_4,          ///< Fourth TMC9660 (future expansion)
    TMC9660_CHIP_COUNT
};

/**
 * @brief Communication interface types for TMC9660.
 */
enum class Tmc9660CommInterface : uint8_t {
    TMC_COMM_SPI = 0,        ///< SPI communication
    TMC_COMM_UART,           ///< UART communication
    TMC_COMM_COUNT
};

/**
 * @brief Time unit enumeration for delay operations.
 */
enum class TimeUnit : uint8_t {
    TIME_UNIT_US = 0,        ///< Microseconds
    TIME_UNIT_MS,            ///< Milliseconds
    TIME_UNIT_S,             ///< Seconds
    
    TIME_UNIT_COUNT
};

#endif // COMPONENT_HANDLER_COMMON_IDS_H_
