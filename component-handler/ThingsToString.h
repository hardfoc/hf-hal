#ifndef COMPONENT_HANDLER_THINGS_TO_STRING_H_
#define COMPONENT_HANDLER_THINGS_TO_STRING_H_

#include "CommonIDs.h"
#include <string_view>

/**
 * @file ThingsToString.h
 * @brief String conversion utilities for enums and identifiers.
 */

/**
 * @brief Convert AdcInputSensor enum to string.
 * @param sensor The ADC input sensor enum value.
 * @return String representation of the sensor.
 */
constexpr std::string_view AdcInputSensorToString(AdcInputSensor sensor) noexcept {
    switch (sensor) {
        case AdcInputSensor::ADC_INTERNAL_CH0: return "ADC_INTERNAL_CH0";
        case AdcInputSensor::ADC_INTERNAL_CH1: return "ADC_INTERNAL_CH1";
        case AdcInputSensor::ADC_INTERNAL_CH2: return "ADC_INTERNAL_CH2";
        case AdcInputSensor::ADC_INTERNAL_CH3: return "ADC_INTERNAL_CH3";
        case AdcInputSensor::ADC_INTERNAL_CH4: return "ADC_INTERNAL_CH4";
        case AdcInputSensor::ADC_INTERNAL_CH5: return "ADC_INTERNAL_CH5";
        case AdcInputSensor::ADC_INTERNAL_CH6: return "ADC_INTERNAL_CH6";
        
        case AdcInputSensor::ADC_EXTERNAL_CH0: return "ADC_EXTERNAL_CH0";
        case AdcInputSensor::ADC_EXTERNAL_CH1: return "ADC_EXTERNAL_CH1";
        case AdcInputSensor::ADC_EXTERNAL_CH2: return "ADC_EXTERNAL_CH2";
        case AdcInputSensor::ADC_EXTERNAL_CH3: return "ADC_EXTERNAL_CH3";
        case AdcInputSensor::ADC_EXTERNAL_CH4: return "ADC_EXTERNAL_CH4";
        case AdcInputSensor::ADC_EXTERNAL_CH5: return "ADC_EXTERNAL_CH5";
        case AdcInputSensor::ADC_EXTERNAL_CH6: return "ADC_EXTERNAL_CH6";
        case AdcInputSensor::ADC_EXTERNAL_CH7: return "ADC_EXTERNAL_CH7";
        case AdcInputSensor::ADC_EXTERNAL_CH8: return "ADC_EXTERNAL_CH8";
        case AdcInputSensor::ADC_EXTERNAL_CH9: return "ADC_EXTERNAL_CH9";
        case AdcInputSensor::ADC_EXTERNAL_CH10: return "ADC_EXTERNAL_CH10";
        case AdcInputSensor::ADC_EXTERNAL_CH11: return "ADC_EXTERNAL_CH11";
        
        case AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A: return "ADC_MOTOR_CURRENT_PHASE_A";
        case AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_B: return "ADC_MOTOR_CURRENT_PHASE_B";
        case AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_C: return "ADC_MOTOR_CURRENT_PHASE_C";
        case AdcInputSensor::ADC_MOTOR_VOLTAGE_BUS: return "ADC_MOTOR_VOLTAGE_BUS";
        case AdcInputSensor::ADC_MOTOR_TEMPERATURE: return "ADC_MOTOR_TEMPERATURE";
        
        case AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3: return "ADC_SYSTEM_VOLTAGE_3V3";
        case AdcInputSensor::ADC_SYSTEM_VOLTAGE_5V: return "ADC_SYSTEM_VOLTAGE_5V";
        case AdcInputSensor::ADC_SYSTEM_VOLTAGE_12V: return "ADC_SYSTEM_VOLTAGE_12V";
        case AdcInputSensor::ADC_SYSTEM_TEMPERATURE: return "ADC_SYSTEM_TEMPERATURE";
        case AdcInputSensor::ADC_SYSTEM_VREF: return "ADC_SYSTEM_VREF";
        
        case AdcInputSensor::ADC_USER_INPUT_1: return "ADC_USER_INPUT_1";
        case AdcInputSensor::ADC_USER_INPUT_2: return "ADC_USER_INPUT_2";
        case AdcInputSensor::ADC_USER_INPUT_3: return "ADC_USER_INPUT_3";
        case AdcInputSensor::ADC_USER_INPUT_4: return "ADC_USER_INPUT_4";
        
        case AdcInputSensor::ADC_INPUT_COUNT: return "ADC_INPUT_COUNT";
        default: return "UNKNOWN_ADC_SENSOR";
    }
}

/**
 * @brief Convert GpioPin enum to string.
 * @param pin The GPIO pin enum value.
 * @return String representation of the pin.
 */
constexpr std::string_view GpioPinToString(GpioPin pin) noexcept {
    switch (pin) {
        case GpioPin::GPIO_ESP32_PIN_0: return "GPIO_ESP32_PIN_0";
        case GpioPin::GPIO_ESP32_PIN_1: return "GPIO_ESP32_PIN_1";
        case GpioPin::GPIO_ESP32_PIN_2: return "GPIO_ESP32_PIN_2";
        case GpioPin::GPIO_ESP32_PIN_3: return "GPIO_ESP32_PIN_3";
        case GpioPin::GPIO_ESP32_PIN_4: return "GPIO_ESP32_PIN_4";
        case GpioPin::GPIO_ESP32_PIN_5: return "GPIO_ESP32_PIN_5";
        case GpioPin::GPIO_ESP32_PIN_6: return "GPIO_ESP32_PIN_6";
        case GpioPin::GPIO_ESP32_PIN_7: return "GPIO_ESP32_PIN_7";
        case GpioPin::GPIO_ESP32_PIN_8: return "GPIO_ESP32_PIN_8";
        case GpioPin::GPIO_ESP32_PIN_9: return "GPIO_ESP32_PIN_9";
        case GpioPin::GPIO_ESP32_PIN_10: return "GPIO_ESP32_PIN_10";
        case GpioPin::GPIO_ESP32_PIN_11: return "GPIO_ESP32_PIN_11";
        case GpioPin::GPIO_ESP32_PIN_12: return "GPIO_ESP32_PIN_12";
        case GpioPin::GPIO_ESP32_PIN_13: return "GPIO_ESP32_PIN_13";
        case GpioPin::GPIO_ESP32_PIN_14: return "GPIO_ESP32_PIN_14";
        case GpioPin::GPIO_ESP32_PIN_15: return "GPIO_ESP32_PIN_15";
        case GpioPin::GPIO_ESP32_PIN_16: return "GPIO_ESP32_PIN_16";
        case GpioPin::GPIO_ESP32_PIN_17: return "GPIO_ESP32_PIN_17";
        case GpioPin::GPIO_ESP32_PIN_18: return "GPIO_ESP32_PIN_18";
        case GpioPin::GPIO_ESP32_PIN_19: return "GPIO_ESP32_PIN_19";
        case GpioPin::GPIO_ESP32_PIN_20: return "GPIO_ESP32_PIN_20";
        case GpioPin::GPIO_ESP32_PIN_21: return "GPIO_ESP32_PIN_21";
        case GpioPin::GPIO_ESP32_PIN_22: return "GPIO_ESP32_PIN_22";
        case GpioPin::GPIO_ESP32_PIN_23: return "GPIO_ESP32_PIN_23";
        
        case GpioPin::GPIO_EXT_PIN_0: return "GPIO_EXT_PIN_0";
        case GpioPin::GPIO_EXT_PIN_1: return "GPIO_EXT_PIN_1";
        case GpioPin::GPIO_EXT_PIN_2: return "GPIO_EXT_PIN_2";
        case GpioPin::GPIO_EXT_PIN_3: return "GPIO_EXT_PIN_3";
        case GpioPin::GPIO_EXT_PIN_4: return "GPIO_EXT_PIN_4";
        case GpioPin::GPIO_EXT_PIN_5: return "GPIO_EXT_PIN_5";
        case GpioPin::GPIO_EXT_PIN_6: return "GPIO_EXT_PIN_6";
        case GpioPin::GPIO_EXT_PIN_7: return "GPIO_EXT_PIN_7";
        case GpioPin::GPIO_EXT_PIN_8: return "GPIO_EXT_PIN_8";
        case GpioPin::GPIO_EXT_PIN_9: return "GPIO_EXT_PIN_9";
        case GpioPin::GPIO_EXT_PIN_10: return "GPIO_EXT_PIN_10";
        case GpioPin::GPIO_EXT_PIN_11: return "GPIO_EXT_PIN_11";
        case GpioPin::GPIO_EXT_PIN_12: return "GPIO_EXT_PIN_12";
        case GpioPin::GPIO_EXT_PIN_13: return "GPIO_EXT_PIN_13";
        case GpioPin::GPIO_EXT_PIN_14: return "GPIO_EXT_PIN_14";
        case GpioPin::GPIO_EXT_PIN_15: return "GPIO_EXT_PIN_15";
        
        case GpioPin::GPIO_SPI_MISO: return "GPIO_SPI_MISO";
        case GpioPin::GPIO_SPI_MOSI: return "GPIO_SPI_MOSI";
        case GpioPin::GPIO_SPI_CLK: return "GPIO_SPI_CLK";
        case GpioPin::GPIO_SPI_CS_TMC: return "GPIO_SPI_CS_TMC";
        case GpioPin::GPIO_SPI_CS_AS5047: return "GPIO_SPI_CS_AS5047";
        case GpioPin::GPIO_SPI_CS_EXT1: return "GPIO_SPI_CS_EXT1";
        case GpioPin::GPIO_SPI_CS_EXT2: return "GPIO_SPI_CS_EXT2";
        
        case GpioPin::GPIO_UART_RX: return "GPIO_UART_RX";
        case GpioPin::GPIO_UART_TX: return "GPIO_UART_TX";
        case GpioPin::GPIO_DEBUG_UART_RX: return "GPIO_DEBUG_UART_RX";
        case GpioPin::GPIO_DEBUG_UART_TX: return "GPIO_DEBUG_UART_TX";
        
        case GpioPin::GPIO_I2C_SDA: return "GPIO_I2C_SDA";
        case GpioPin::GPIO_I2C_SCL: return "GPIO_I2C_SCL";
        
        case GpioPin::GPIO_CAN_TX: return "GPIO_CAN_TX";
        case GpioPin::GPIO_CAN_RX: return "GPIO_CAN_RX";
        
        case GpioPin::GPIO_WS2812_DATA: return "GPIO_WS2812_DATA";
        
        case GpioPin::GPIO_MOTOR_ENABLE: return "GPIO_MOTOR_ENABLE";
        case GpioPin::GPIO_MOTOR_FAULT: return "GPIO_MOTOR_FAULT";
        case GpioPin::GPIO_MOTOR_BRAKE: return "GPIO_MOTOR_BRAKE";
        
        case GpioPin::GPIO_LED_STATUS: return "GPIO_LED_STATUS";
        case GpioPin::GPIO_LED_ERROR: return "GPIO_LED_ERROR";
        case GpioPin::GPIO_LED_COMM: return "GPIO_LED_COMM";
        
        case GpioPin::GPIO_USER_OUTPUT_1: return "GPIO_USER_OUTPUT_1";
        case GpioPin::GPIO_USER_OUTPUT_2: return "GPIO_USER_OUTPUT_2";
        case GpioPin::GPIO_USER_INPUT_1: return "GPIO_USER_INPUT_1";
        case GpioPin::GPIO_USER_INPUT_2: return "GPIO_USER_INPUT_2";
        
        case GpioPin::GPIO_PIN_COUNT: return "GPIO_PIN_COUNT";
        default: return "UNKNOWN_GPIO_PIN";
    }
}

/**
 * @brief Convert AdcChip enum to string.
 * @param chip The ADC chip enum value.
 * @return String representation of the chip.
 */
constexpr std::string_view AdcChipToString(AdcChip chip) noexcept {
    switch (chip) {
        case AdcChip::ADC_ESP32_INTERNAL: return "ADC_ESP32_INTERNAL";
        case AdcChip::ADC_EXTERNAL_SPI_CHIP_1: return "ADC_EXTERNAL_SPI_CHIP_1";
        case AdcChip::ADC_EXTERNAL_SPI_CHIP_2: return "ADC_EXTERNAL_SPI_CHIP_2";
        case AdcChip::ADC_EXTERNAL_I2C_CHIP_1: return "ADC_EXTERNAL_I2C_CHIP_1";
        case AdcChip::ADC_CHIP_COUNT: return "ADC_CHIP_COUNT";
        default: return "UNKNOWN_ADC_CHIP";
    }
}

/**
 * @brief Convert GpioChip enum to string.
 * @param chip The GPIO chip enum value.
 * @return String representation of the chip.
 */
constexpr std::string_view GpioChipToString(GpioChip chip) noexcept {
    switch (chip) {
        case GpioChip::GPIO_ESP32_INTERNAL: return "GPIO_ESP32_INTERNAL";
        case GpioChip::GPIO_PCAL95555_CHIP_1: return "GPIO_PCAL95555_CHIP_1";
        case GpioChip::GPIO_PCAL95555_CHIP_2: return "GPIO_PCAL95555_CHIP_2";
        case GpioChip::GPIO_CHIP_COUNT: return "GPIO_CHIP_COUNT";
        default: return "UNKNOWN_GPIO_CHIP";
    }
}

#endif // COMPONENT_HANDLER_THINGS_TO_STRING_H_
