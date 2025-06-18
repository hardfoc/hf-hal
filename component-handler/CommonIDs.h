#ifndef COMPONENT_HANDLER_COMMON_IDS_H_
#define COMPONENT_HANDLER_COMMON_IDS_H_

#include <cstdint>

/**
 * @file CommonIDs.h
 * @brief Common enumerations for ADC and GPIO identifiers in the HardFOC system.
 */

/**
 * @brief Enumeration for ADC input sensors/channels.
 */
enum class AdcInputSensor : uint8_t {
    // ESP32-C6 Internal ADC channels
    ADC_INTERNAL_CH0 = 0,
    ADC_INTERNAL_CH1,
    ADC_INTERNAL_CH2,
    ADC_INTERNAL_CH3,
    ADC_INTERNAL_CH4,
    ADC_INTERNAL_CH5,
    ADC_INTERNAL_CH6,
    
    // External ADC channels (e.g., via SPI ADC chips)
    ADC_EXTERNAL_CH0,
    ADC_EXTERNAL_CH1,
    ADC_EXTERNAL_CH2,
    ADC_EXTERNAL_CH3,
    ADC_EXTERNAL_CH4,
    ADC_EXTERNAL_CH5,
    ADC_EXTERNAL_CH6,
    ADC_EXTERNAL_CH7,
    ADC_EXTERNAL_CH8,
    ADC_EXTERNAL_CH9,
    ADC_EXTERNAL_CH10,
    ADC_EXTERNAL_CH11,
    
    // Motor control specific sensors
    ADC_MOTOR_CURRENT_PHASE_A,
    ADC_MOTOR_CURRENT_PHASE_B,
    ADC_MOTOR_CURRENT_PHASE_C,
    ADC_MOTOR_VOLTAGE_BUS,
    ADC_MOTOR_TEMPERATURE,
    
    // System monitoring
    ADC_SYSTEM_VOLTAGE_3V3,
    ADC_SYSTEM_VOLTAGE_5V,
    ADC_SYSTEM_VOLTAGE_12V,
    ADC_SYSTEM_TEMPERATURE,
    ADC_SYSTEM_VREF,
    
    // External inputs for user expansion
    ADC_USER_INPUT_1,
    ADC_USER_INPUT_2,
    ADC_USER_INPUT_3,
    ADC_USER_INPUT_4,
    
    ADC_INPUT_COUNT ///< Total number of ADC inputs
};

/**
 * @brief Enumeration for GPIO pins.
 */
enum class GpioPin : uint8_t {
    // ESP32-C6 Native GPIO pins
    GPIO_ESP32_PIN_0 = 0,
    GPIO_ESP32_PIN_1,
    GPIO_ESP32_PIN_2,
    GPIO_ESP32_PIN_3,
    GPIO_ESP32_PIN_4,
    GPIO_ESP32_PIN_5,
    GPIO_ESP32_PIN_6,
    GPIO_ESP32_PIN_7,
    GPIO_ESP32_PIN_8,
    GPIO_ESP32_PIN_9,
    GPIO_ESP32_PIN_10,
    GPIO_ESP32_PIN_11,
    GPIO_ESP32_PIN_12,
    GPIO_ESP32_PIN_13,
    GPIO_ESP32_PIN_14,
    GPIO_ESP32_PIN_15,
    GPIO_ESP32_PIN_16,
    GPIO_ESP32_PIN_17,
    GPIO_ESP32_PIN_18,
    GPIO_ESP32_PIN_19,
    GPIO_ESP32_PIN_20,
    GPIO_ESP32_PIN_21,
    GPIO_ESP32_PIN_22,
    GPIO_ESP32_PIN_23,
    
    // PCAL95555 GPIO Expander pins
    GPIO_EXT_PIN_0,
    GPIO_EXT_PIN_1,
    GPIO_EXT_PIN_2,
    GPIO_EXT_PIN_3,
    GPIO_EXT_PIN_4,
    GPIO_EXT_PIN_5,
    GPIO_EXT_PIN_6,
    GPIO_EXT_PIN_7,
    GPIO_EXT_PIN_8,
    GPIO_EXT_PIN_9,
    GPIO_EXT_PIN_10,
    GPIO_EXT_PIN_11,
    GPIO_EXT_PIN_12,
    GPIO_EXT_PIN_13,
    GPIO_EXT_PIN_14,
    GPIO_EXT_PIN_15,
    
    // Functional pin mappings
    GPIO_SPI_MISO,
    GPIO_SPI_MOSI,
    GPIO_SPI_CLK,
    GPIO_SPI_CS_TMC,
    GPIO_SPI_CS_AS5047,
    GPIO_SPI_CS_EXT1,
    GPIO_SPI_CS_EXT2,
    
    GPIO_UART_RX,
    GPIO_UART_TX,
    GPIO_DEBUG_UART_RX,
    GPIO_DEBUG_UART_TX,
    
    GPIO_I2C_SDA,
    GPIO_I2C_SCL,
    
    GPIO_CAN_TX,
    GPIO_CAN_RX,
    
    GPIO_WS2812_DATA,
    
    // Motor control pins
    GPIO_MOTOR_ENABLE,
    GPIO_MOTOR_FAULT,
    GPIO_MOTOR_BRAKE,
    
    // System status LEDs
    GPIO_LED_STATUS,
    GPIO_LED_ERROR,
    GPIO_LED_COMM,
    
    // User expansion pins
    GPIO_USER_OUTPUT_1,
    GPIO_USER_OUTPUT_2,
    GPIO_USER_INPUT_1,
    GPIO_USER_INPUT_2,
    
    GPIO_PIN_COUNT ///< Total number of GPIO pins
};

/**
 * @brief Enumeration for ADC chip types.
 */
enum class AdcChip : uint8_t {
    ADC_ESP32_INTERNAL = 0,
    ADC_EXTERNAL_SPI_CHIP_1,
    ADC_EXTERNAL_SPI_CHIP_2,
    ADC_EXTERNAL_I2C_CHIP_1,
    ADC_CHIP_COUNT
};

/**
 * @brief Enumeration for GPIO chip types.
 */
enum class GpioChip : uint8_t {
    GPIO_ESP32_INTERNAL = 0,
    GPIO_PCAL95555_CHIP_1,
    GPIO_PCAL95555_CHIP_2,
    GPIO_CHIP_COUNT
};

/**
 * @brief Time units for delay operations.
 */
enum class TimeUnit : uint8_t {
    TIME_UNIT_MS = 0,
    TIME_UNIT_US,
    TIME_UNIT_S
};

#endif // COMPONENT_HANDLER_COMMON_IDS_H_
