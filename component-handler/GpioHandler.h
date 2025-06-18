#ifndef COMPONENT_HANDLER_GPIO_HANDLER_H_
#define COMPONENT_HANDLER_GPIO_HANDLER_H_

#include "GpioData.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/DigitalGpio.h"
#include <array>
#include <string_view>

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
};

#endif // COMPONENT_HANDLER_GPIO_HANDLER_H_
