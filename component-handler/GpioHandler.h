#ifndef COMPONENT_HANDLER_GPIO_HANDLER_H_
#define COMPONENT_HANDLER_GPIO_HANDLER_H_

#include "GpioData.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "Pcal95555Gpio.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/DigitalGpio.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/SfI2cBus.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_ext_pins_enum.hpp"
#include <array>
#include <string_view>
#include <memory>

/**
 * @file GpioHandler.h
 * @brief Legacy compatibility wrapper for the new GpioData system.
 *
 * This wrapper provides backward compatibility with the old GpioHandler interface
 * while using the new comprehensive GpioData system under the hood.
 * 
 * @deprecated This interface is deprecated. Use GpioData directly for new code.
 */
template <std::size_t N> class GpioHandler {
public:
  struct PinInfo {
    std::string_view name; ///< Logical pin name
    GpioPin pin;           ///< Mapped pin ID
    DigitalGpio &gpio;     ///< Reference to the GPIO object
  };

  /**
   * @brief Access the singleton instance.
   * @deprecated Use GpioData::GetInstance() instead.
   */
  static GpioHandler &Instance() noexcept {
    static GpioHandler inst;
    return inst;
  }

  GpioHandler(const GpioHandler &) = delete;
  GpioHandler &operator=(const GpioHandler &) = delete;

  /**
   * @brief Initialize the GPIO handler and all associated hardware.
   * @param i2cBus Reference to the I2C bus for PCAL95555 communication.
   * @return true if initialization successful, false otherwise.
   */
  bool Initialize(SfI2cBus &i2cBus) noexcept {
    if (initialized_) {
      return true;  // Already initialized
    }

    // Initialize PCAL95555 chips
    if (!InitializePcal95555Chips(i2cBus)) {
      return false;
    }

    // Register all PCAL95555 pins with the GPIO system
    if (!RegisterPcal95555Pins()) {
      return false;
    }

    initialized_ = true;
    return true;
  }

  /**
   * @brief Register a GPIO reference with a name.
   * @param name Logical pin name.
   * @param gpio Reference to a DigitalGpio derived object.
   * @return true if successfully registered, false if full.
   * @deprecated Use GpioData::RegisterGpioPin() instead.
   */
  bool RegisterPin(std::string_view name, DigitalGpio &gpio) noexcept {
    if (count_ >= N)
      return false;
      
    // Map the name to a pin ID
    GpioPin pin = MapNameToPin(name);
    
    // Register with the new GpioData system
    GpioData& gpioData = GpioData::GetInstance();
    if (!gpioData.RegisterGpioPin(pin, gpio, name)) {
      return false;
    }
    
    // Store in our local table for backward compatibility
    pins_[count_++] = {name, pin, gpio};
    return true;
  }

  /**
   * @brief Retrieve a pointer to a registered pin by name.
   * @return Pointer to DigitalGpio or nullptr if not found.
   * @deprecated Use GpioData::GetPinByName() instead.
   */
  DigitalGpio *GetPin(std::string_view name) noexcept {
    // Try to find in our local table first
    for (std::size_t i = 0; i < count_; ++i) {
      if (pins_[i].name == name)
        return &pins_[i].gpio;
    }
    
    // Fall back to the new system
    GpioData& gpioData = GpioData::GetInstance();
    const GpioInfo* info = gpioData.GetPinByName(name);
    if (info) {
      return &info->gpio;
    }
    
    return nullptr;
  }

  /**
   * @brief Set a pin active by name.
   * @param name The pin name.
   * @return true if successful, false otherwise.
   * @deprecated Use GpioData::SetPinByName() instead.
   */
  bool SetPinActive(std::string_view name) noexcept {
    GpioData& gpioData = GpioData::GetInstance();
    return gpioData.SetPinByName(name, true);
  }

  /**
   * @brief Set a pin inactive by name.
   * @param name The pin name.
   * @return true if successful, false otherwise.
   * @deprecated Use GpioData::SetPinByName() instead.
   */
  bool SetPinInactive(std::string_view name) noexcept {
    GpioData& gpioData = GpioData::GetInstance();
    return gpioData.SetPinByName(name, false);
  }

  /**
   * @brief Toggle a pin by name.
   * @param name The pin name.
   * @return true if successful, false otherwise.
   * @deprecated Use GpioData::TogglePinByName() instead.
   */
  bool TogglePin(std::string_view name) noexcept {
    GpioData& gpioData = GpioData::GetInstance();
    return gpioData.TogglePinByName(name);
  }

  /**
   * @brief Check if a pin is active by name.
   * @param name The pin name.
   * @return true if active, false if inactive or not found.
   * @deprecated Use GpioData::GetPinByName() instead.
   */
  bool IsPinActive(std::string_view name) noexcept {
    GpioData& gpioData = GpioData::GetInstance();
    return gpioData.GetPinByName(name);
  }

private:
  GpioHandler() = default;

  /**
   * @brief Initialize PCAL95555 GPIO expander chips.
   * @param i2cBus Reference to the I2C bus.
   * @return true if successful, false otherwise.
   */
  bool InitializePcal95555Chips(SfI2cBus &i2cBus) noexcept {
    try {
      // Create PCAL95555 chip instances
      // PCAL95555 Chip 1 at address 0x20
      chip1_ = std::make_unique<Pcal95555Chip>(i2cBus, 0x20);
      
      // PCAL95555 Chip 2 at address 0x21  
      chip2_ = std::make_unique<Pcal95555Chip>(i2cBus, 0x21);

      // Initialize the devices
      if (!chip1_->Device().initialize()) {
        return false;
      }
      
      if (!chip2_->Device().initialize()) {
        return false;
      }

      return true;
    } catch (...) {
      return false;
    }
  }

  /**
   * @brief Register all PCAL95555 pins with the GPIO system.
   * @return true if successful, false otherwise.
   */
  bool RegisterPcal95555Pins() noexcept {
    GpioData& gpioData = GpioData::GetInstance();
    bool success = true;

    // Register Chip 1 pins
    if (chip1_) {
      success &= RegisterChip1Pins(gpioData);
    }

    // Register Chip 2 pins  
    if (chip2_) {
      success &= RegisterChip2Pins(gpioData);
    }

    return success;
  }

  /**
   * @brief Register PCAL95555 Chip 1 pins.
   * @param gpioData Reference to GPIO data system.
   * @return true if successful, false otherwise.
   */
  bool RegisterChip1Pins(GpioData& gpioData) noexcept {
    // Map from hf_ext_pins_enum.hpp Chip1 pins to GpioPins
    const std::array<std::pair<GpioPin, std::string_view>, 16> chip1Mapping = {{
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_0, "pcal1_pin0"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_1, "pcal1_pin1"}, 
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_2, "pcal1_pin2"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_3, "pcal1_pin3"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_4, "pcal1_pin4"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_5, "pcal1_pin5"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_6, "pcal1_pin6"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_7, "pcal1_pin7"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_8, "pcal1_pin8"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_9, "pcal1_pin9"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_10, "pcal1_pin10"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_11, "pcal1_pin11"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_12, "pcal1_pin12"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_13, "pcal1_pin13"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_14, "pcal1_pin14"},
      {GpioPin::GPIO_PCAL95555_CHIP1_PIN_15, "pcal1_pin15"}
    }};

    bool success = true;
    for (size_t i = 0; i < chip1Mapping.size(); ++i) {
      success &= gpioData.RegisterGpioPin(
        chip1Mapping[i].first,
        chip1_->Pin(i),
        chip1Mapping[i].second
      );
    }

    return success;
  }

  /**
   * @brief Register PCAL95555 Chip 2 pins.
   * @param gpioData Reference to GPIO data system.
   * @return true if successful, false otherwise.
   */
  bool RegisterChip2Pins(GpioData& gpioData) noexcept {
    // Map from hf_ext_pins_enum.hpp Chip2 pins to GpioPins
    const std::array<std::pair<GpioPin, std::string_view>, 16> chip2Mapping = {{
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_0, "pcal2_pin0"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_1, "pcal2_pin1"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_2, "pcal2_pin2"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_3, "pcal2_pin3"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_4, "pcal2_pin4"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_5, "pcal2_pin5"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_6, "pcal2_pin6"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_7, "pcal2_pin7"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_8, "pcal2_pin8"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_9, "pcal2_pin9"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_10, "pcal2_pin10"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_11, "pcal2_pin11"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_12, "pcal2_pin12"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_13, "pcal2_pin13"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_14, "pcal2_pin14"},
      {GpioPin::GPIO_PCAL95555_CHIP2_PIN_15, "pcal2_pin15"}
    }};

    bool success = true;
    for (size_t i = 0; i < chip2Mapping.size(); ++i) {
      success &= gpioData.RegisterGpioPin(
        chip2Mapping[i].first,
        chip2_->Pin(i),
        chip2Mapping[i].second
      );
    }

    return success;
  }

  /**
   * @brief Map a pin name to a GpioPin ID.
   * @param name The pin name.
   * @return Mapped pin ID.
   */
  GpioPin MapNameToPin(std::string_view name) noexcept {
    // Simple name-to-pin mapping
    // In a real implementation, this would be more sophisticated
    if (name == "motor_enable") return GpioPin::GPIO_MOTOR_ENABLE;
    if (name == "motor_fault") return GpioPin::GPIO_MOTOR_FAULT;
    if (name == "motor_brake") return GpioPin::GPIO_MOTOR_BRAKE;
    if (name == "led_status") return GpioPin::GPIO_LED_STATUS;
    if (name == "led_error") return GpioPin::GPIO_LED_ERROR;
    if (name == "led_comm") return GpioPin::GPIO_LED_COMM;
    if (name == "user_output_1") return GpioPin::GPIO_USER_OUTPUT_1;
    if (name == "user_output_2") return GpioPin::GPIO_USER_OUTPUT_2;
    if (name == "user_input_1") return GpioPin::GPIO_USER_INPUT_1;
    if (name == "user_input_2") return GpioPin::GPIO_USER_INPUT_2;
    if (name == "spi_cs_tmc") return GpioPin::GPIO_SPI_CS_TMC;
    if (name == "spi_cs_as5047") return GpioPin::GPIO_SPI_CS_AS5047;
    if (name == "ws2812_data") return GpioPin::GPIO_WS2812_DATA;
    
    // Default to first ESP32 pin if no match
    return GpioPin::GPIO_ESP32_PIN_0;
  }

  std::array<PinInfo, N> pins_{}; ///< Compile-time array of pin info
  std::size_t count_ = 0;         ///< Number of registered pins
  
  // PCAL95555 GPIO expander management
  bool initialized_ = false;                    ///< Initialization status
  std::unique_ptr<Pcal95555Chip> chip1_;       ///< PCAL95555 Chip 1 instance
  std::unique_ptr<Pcal95555Chip> chip2_;       ///< PCAL95555 Chip 2 instance
};

#endif // COMPONENT_HANDLER_GPIO_HANDLER_H_
