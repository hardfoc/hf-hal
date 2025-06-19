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
    // ESP32-C6 Internal ADC channels (ADC1: 0-6, ADC2: 0-5 mapped to 7-12)
    ADC_INTERNAL_CH0 = 0,
    ADC_INTERNAL_CH1,
    ADC_INTERNAL_CH2,
    ADC_INTERNAL_CH3,
    ADC_INTERNAL_CH4,
    ADC_INTERNAL_CH5,
    ADC_INTERNAL_CH6,
    ADC_INTERNAL_CH7,    // ADC2_CHANNEL_0
    ADC_INTERNAL_CH8,    // ADC2_CHANNEL_1
    ADC_INTERNAL_CH9,    // ADC2_CHANNEL_2
    ADC_INTERNAL_CH10,   // ADC2_CHANNEL_3
    ADC_INTERNAL_CH11,   // ADC2_CHANNEL_4
    ADC_INTERNAL_CH12,   // ADC2_CHANNEL_5
    
    // TMC9660 Motor Controller ADC channels (3 channels for current sensing)
    ADC_TMC9660_CURRENT_A,
    ADC_TMC9660_CURRENT_B,
    ADC_TMC9660_CURRENT_C,
    
    // System monitoring channels (mapped to internal ADC channels)
    ADC_SYSTEM_VOLTAGE_3V3,
    ADC_SYSTEM_VOLTAGE_5V,
    ADC_SYSTEM_TEMPERATURE,
    ADC_SYSTEM_VREF,
    
    // User expansion (if needed)
    ADC_USER_INPUT_1,
    ADC_USER_INPUT_2,
    
    ADC_INPUT_COUNT ///< Total number of ADC inputs
};

/**
 * @brief Enumeration for GPIO pins.
 */
enum class GpioPin : uint8_t {
    // ESP32-C6 Native GPIO pins (as defined in hf_gpio_config.hpp)
    GPIO_ESP32_PIN_0 = 0,    // Debug UART TX  
    GPIO_ESP32_PIN_1,        // Debug UART RX
    GPIO_ESP32_PIN_2,        // SPI0_MISO / WS2812_LED
    GPIO_ESP32_PIN_4,        // UART_RXD
    GPIO_ESP32_PIN_5,        // UART_TXD
    GPIO_ESP32_PIN_6,        // SPI0_CLK
    GPIO_ESP32_PIN_7,        // SPI0_MOSI
    GPIO_ESP32_PIN_8,        // SPI0_CS_EXT2
    GPIO_ESP32_PIN_12,       // USB_JTAG_D_MINUS
    GPIO_ESP32_PIN_13,       // USB_JTAG_D_PLUS
    GPIO_ESP32_PIN_15,       // TWAI_RX
    GPIO_ESP32_PIN_18,       // SPI0_CS_TMC
    GPIO_ESP32_PIN_19,       // SPI0_CS_EXT1 / TWAI_TX
    GPIO_ESP32_PIN_20,       // SPI0_CS_AS5047
    GPIO_ESP32_PIN_22,       // I2C_SDA
    GPIO_ESP32_PIN_23,       // I2C_SCL
    
    // PCAL95555 GPIO Expander pins (single chip at 0x20)
    GPIO_PCAL95555_PIN_0,    // Motor Enable 1
    GPIO_PCAL95555_PIN_1,    // Motor Enable 2
    GPIO_PCAL95555_PIN_2,    // Motor Brake 1
    GPIO_PCAL95555_PIN_3,    // Motor Brake 2
    GPIO_PCAL95555_PIN_4,    // Motor Fault 1
    GPIO_PCAL95555_PIN_5,    // Motor Fault 2
    GPIO_PCAL95555_PIN_6,    // LED Status Green
    GPIO_PCAL95555_PIN_7,    // LED Status Red
    GPIO_PCAL95555_PIN_8,    // LED Error
    GPIO_PCAL95555_PIN_9,    // LED Comm
    GPIO_PCAL95555_PIN_10,   // External Relay 1
    GPIO_PCAL95555_PIN_11,   // External Relay 2
    GPIO_PCAL95555_PIN_12,   // External Output 1
    GPIO_PCAL95555_PIN_13,   // External Output 2
    GPIO_PCAL95555_PIN_14,   // External Input 1
    GPIO_PCAL95555_PIN_15,   // External Input 2
    
    // Functional pin mappings (mapped to physical pins)
    GPIO_SPI_MISO,           // GPIO_2
    GPIO_SPI_MOSI,           // GPIO_7
    GPIO_SPI_CLK,            // GPIO_6
    GPIO_SPI_CS_TMC,         // GPIO_18
    GPIO_SPI_CS_AS5047,      // GPIO_20
    GPIO_SPI_CS_EXT1,        // GPIO_19
    GPIO_SPI_CS_EXT2,        // GPIO_8
    
    GPIO_UART_RX,            // GPIO_4
    GPIO_UART_TX,            // GPIO_5
    GPIO_DEBUG_UART_RX,      // GPIO_1
    GPIO_DEBUG_UART_TX,      // GPIO_0
    
    GPIO_I2C_SDA,            // GPIO_22
    GPIO_I2C_SCL,            // GPIO_23
    
    GPIO_CAN_TX,             // GPIO_19 (shared with SPI_CS_EXT1)
    GPIO_CAN_RX,             // GPIO_15
    
    GPIO_WS2812_DATA,        // GPIO_2 (shared with SPI_MISO)
    
    // Motor control pins (mapped to PCAL95555)
    GPIO_MOTOR_ENABLE,       // PCAL95555_PIN_0
    GPIO_MOTOR_FAULT,        // PCAL95555_PIN_4
    GPIO_MOTOR_BRAKE,        // PCAL95555_PIN_2
    
    // System status LEDs (mapped to PCAL95555)
    GPIO_LED_STATUS,         // PCAL95555_PIN_6 (Green)
    GPIO_LED_ERROR,          // PCAL95555_PIN_8
    GPIO_LED_COMM,           // PCAL95555_PIN_9
    
    // User expansion pins (mapped to PCAL95555)
    GPIO_USER_OUTPUT_1,      // PCAL95555_PIN_12
    GPIO_USER_OUTPUT_2,      // PCAL95555_PIN_13
    GPIO_USER_INPUT_1,       // PCAL95555_PIN_14
    GPIO_USER_INPUT_2,       // PCAL95555_PIN_15
    
    GPIO_PIN_COUNT ///< Total number of GPIO pins
};

/**
 * @brief Enumeration for ADC chip types.
 */
enum class AdcChip : uint8_t {
    ADC_ESP32_INTERNAL = 0,  // ESP32-C6 internal ADC units
    ADC_TMC9660,             // TMC9660 motor controller ADC
    ADC_CHIP_COUNT
};

/**
 * @brief Enumeration for GPIO chip types.
 */
enum class GpioChip : uint8_t {
    GPIO_ESP32_INTERNAL = 0, // ESP32-C6 native GPIO
    GPIO_PCAL95555,          // Single PCAL95555 GPIO expander
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
