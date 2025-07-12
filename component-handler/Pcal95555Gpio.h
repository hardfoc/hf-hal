#ifndef COMPONENT_HANDLER_PCAL95555GPIO_H_
#define COMPONENT_HANDLER_PCAL95555GPIO_H_

#include "CommonIDs.h"
#include "PCAL95555.hpp"
#include "BaseGpio.h"
#include "SfI2cBus.h"
#include <array>
#include <string_view>

/**
 * @file Pcal95555Gpio.h
 * @brief Wrapper classes to integrate the PCAL95555 GPIO expander with the
 *        internal SfI2cBus driver and GpioData system.
 * 
 * This file provides comprehensive integration for a single PCAL95555 GPIO 
 * expander chip, providing 16 additional GPIO pins via I2C communication.
 * 
 * Features:
 * - Single PCAL95555 chip support (address 0x20)
 * - Thread-safe I2C communication via SfI2cBus
 * - Full DigitalGpio interface compatibility
 * - Automatic error handling and recovery
 * - Pin direction and state management
 */

/**
 * @class SfPcal95555Bus
 * @brief Adapter implementing PCAL95555::i2cBus using SfI2cBus for
 *        thread-safe transfers.
 */
class SfPcal95555Bus : public PCAL95555::i2cBus {
public:
  explicit SfPcal95555Bus(SfI2cBus &bus) noexcept : bus_(bus) {}

  bool write(uint8_t addr, uint8_t reg, const uint8_t *data,
             size_t len) override {
    std::array<uint8_t, 9> buf{}; // reg + up to 8 bytes
    if (len > buf.size() - 1)
      return false;
    buf[0] = reg;
    for (size_t i = 0; i < len; ++i)
      buf[i + 1] = data[i];
    return bus_.Write(addr, buf.data(), len + 1, 1000);
  }

  bool read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) override {
    return bus_.WriteRead(addr, &reg, 1, data, len, 1000);
  }

private:
  SfI2cBus &bus_;
};

/**
 * @class Pcal95555Pin
 * @brief Represents a single PCAL95555 GPIO as a BaseGpio instance.
 */
class Pcal95555Pin : public BaseGpio {
public:  Pcal95555Pin(PCAL95555 &chip, uint8_t index, ActiveState act,
               PCAL95555::GPIODir dir = PCAL95555::GPIODir::Input) noexcept
      : BaseGpio(GPIO_NUM_NC, 
                 (dir == PCAL95555::GPIODir::Input) ? Direction::Input : Direction::Output,
                 act), 
        chip_(chip), index_(index), dir_(dir) {}

  bool Initialize() noexcept override {
    return chip_.setPinDirection(index_, dir_);
  }

  bool SetActive() noexcept {
    return EnsureInitialized() && chip_.writePin(index_, true ^ IsActiveLow());
  }

  bool SetInactive() noexcept {
    return EnsureInitialized() && chip_.writePin(index_, false ^ IsActiveLow());
  }

  bool Toggle() noexcept {
    return EnsureInitialized() && chip_.togglePin(index_);
  }

  bool IsActive() noexcept {
    if (!EnsureInitialized())
      return false;
    bool level = chip_.readPin(index_);
    return IsActiveHigh() ? level : !level;
  }

private:
  PCAL95555 &chip_;
  uint8_t index_;
  PCAL95555::GPIODir dir_;
};

/**
 * @class Pcal95555Chip
 * @brief Helper owning a PCAL95555 device and exposing its pins.
 */
class Pcal95555Chip {
public:
  static constexpr std::size_t kPinCount = 16;

  Pcal95555Chip(SfI2cBus &bus, uint8_t addr)
      : adapter_(bus), device_(&adapter_, addr),
        pins_{
            Pcal95555Pin(device_, 0, ActiveState::High),
            Pcal95555Pin(device_, 1, ActiveState::High),
            Pcal95555Pin(device_, 2, ActiveState::High),
            Pcal95555Pin(device_, 3, ActiveState::High),
            Pcal95555Pin(device_, 4, ActiveState::High),
            Pcal95555Pin(device_, 5, ActiveState::High),
            Pcal95555Pin(device_, 6, ActiveState::High),
            Pcal95555Pin(device_, 7, ActiveState::High),
            Pcal95555Pin(device_, 8, ActiveState::High),
            Pcal95555Pin(device_, 9, ActiveState::High),
            Pcal95555Pin(device_, 10, ActiveState::High),
            Pcal95555Pin(device_, 11, ActiveState::High),
            Pcal95555Pin(device_, 12, ActiveState::High),
            Pcal95555Pin(device_, 13, ActiveState::High),
            Pcal95555Pin(device_, 14, ActiveState::High),
            Pcal95555Pin(device_, 15, ActiveState::High),
        } {}
  /**
   * @brief Register all PCAL95555 pins with the GpioData system.
   * @return true if all pins registered successfully, false otherwise
   */
  bool RegisterAllPins() noexcept;

  /**
   * @brief Initialize the PCAL95555 device.
   * @return true if initialization successful, false otherwise
   */
  bool Initialize() noexcept;

  /**
   * @brief Check if the PCAL95555 device is communicating.
   * @return true if device responds, false otherwise
   */
  bool IsHealthy() noexcept;

  PCAL95555 &Device() noexcept { return device_; }
  Pcal95555Pin &Pin(std::size_t i) noexcept { return pins_[i]; }

private:
  SfPcal95555Bus adapter_;
  PCAL95555 device_;
  std::array<Pcal95555Pin, kPinCount> pins_;
};

#endif // COMPONENT_HANDLER_PCAL95555GPIO_H_
