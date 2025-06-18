#ifndef COMPONENT_HANDLER_GPIO_HANDLER_H_
#define COMPONENT_HANDLER_GPIO_HANDLER_H_

#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/DigitalGpio.h"
#include <array>
#include <string_view>

/**
 * @file GpioHandler.h
 * @brief Aggregates multiple DigitalGpio instances by name.
 *
 * The handler stores references to GPIO drivers known at compile time.  Use
 * RegisterPin() during initialization to map a human readable name to each
 * DigitalGpio reference.  Pins can later be retrieved by name for reading or
 * writing without caring which chip provides the actual GPIO.
 */
template <std::size_t N> class GpioHandler {
public:
  struct PinInfo {
    std::string_view name; ///< Logical pin name
    DigitalGpio &gpio;     ///< Reference to the GPIO object
  };

  /**
   * @brief Access the singleton instance.
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
   */
  bool RegisterPin(std::string_view name, DigitalGpio &gpio) noexcept {
    if (count_ >= N)
      return false;
    pins_[count_++] = {name, gpio};
    return true;
  }

  /**
   * @brief Retrieve a pointer to a registered pin by name.
   * @return Pointer to DigitalGpio or nullptr if not found.
   */
  DigitalGpio *GetPin(std::string_view name) noexcept {
    for (std::size_t i = 0; i < count_; ++i) {
      if (pins_[i].name == name)
        return &pins_[i].gpio;
    }
    return nullptr;
  }

private:
  GpioHandler() = default;

  std::array<PinInfo, N> pins_{}; ///< Compile-time array of pin info
  std::size_t count_ = 0;         ///< Number of registered pins
};

#endif // COMPONENT_HANDLER_GPIO_HANDLER_H_
