/**
 * @file Pcal95555DigitalGpio.h
 * @brief PCAL95555 GPIO expander implementation of unified BaseGpio interface.
 *
 * This file provides a concrete implementation of the unified BaseGpio class
 * for PCAL95555 I2C GPIO expander pins. It supports dynamic mode switching,
 * pull resistor configuration, and integrates seamlessly with MCU GPIOs.
 */
#ifndef PCAL95555DIGITALGPIO_H
#define PCAL95555DIGITALGPIO_H

#include "base/BaseGpio.h"
#include "PCAL95555.hpp"
#include "SfI2cBus.h"
#include <memory>

/**
 * @class Pcal95555DigitalGpio * @brief PCAL95555 implementation of unified BaseGpio with dynamic mode switching.
 * @details This class provides a concrete implementation of BaseGpio for PCAL95555
 *          I2C GPIO expander pins. It supports all the features of the unified BaseGpio:
 *          - Dynamic switching between input and output modes
 *          - Active-high/active-low polarity configuration
 *          - Pull resistor configuration (floating, pull-up, pull-down)
 *          - Output drive modes (push-pull, open-drain)
 *          - Thread-safe I2C communication
 *          
 * @note This class requires a shared PCAL95555 driver instance and I2C bus.
 *       Multiple pins can share the same PCAL95555 chip.
 */
class Pcal95555DigitalGpio : public BaseGpio {
public:
  //==============================================================//
  // CONSTRUCTORS
  //==============================================================//
  
  /**
   * @brief Constructor for PCAL95555 GPIO pin with full configuration.
   * @param chip_pin PCAL95555 pin number (0-15)
   * @param pcal95555_driver Shared pointer to PCAL95555 driver instance
   * @param i2c_address I2C address of the PCAL95555 chip (typically 0x20)
   * @param direction Initial pin direction (Input or Output)
   * @param active_state Polarity configuration (High or Low active)
   * @param output_mode Output drive mode (PushPull or OpenDrain)
   * @param pull_mode Pull resistor configuration (Floating, PullUp, or PullDown)
   * @details Creates a PCAL95555 GPIO instance with the specified configuration.
   *          The pin is not physically configured until Initialize() is called.
   */
  explicit Pcal95555DigitalGpio(uint8_t chip_pin,
                                std::shared_ptr<PCAL95555> pcal95555_driver,
                                uint8_t i2c_address = 0x20,
                                Direction direction = Direction::Input,
                                ActiveState active_state = ActiveState::High,
                                OutputMode output_mode = OutputMode::PushPull,
                                PullMode pull_mode = PullMode::Floating) noexcept;

  /**
   * @brief Destructor - ensures proper cleanup.
   */
  ~Pcal95555DigitalGpio() override = default;

  //==============================================================//
  // BASEGPIO IMPLEMENTATION
  //==============================================================//

  /**
   * @brief Initialize the PCAL95555 GPIO pin with current configuration.
   * @return true if initialization successful, false otherwise
   * @details Configures the PCAL95555 pin according to the current
   *          direction, pull mode, and output mode settings.
   */
  bool Initialize() noexcept override;

  /**
   * @brief Deinitialize the PCAL95555 GPIO pin.
   * @return true if deinitialization successful, false otherwise
   * @details Resets the pin to a safe default state and marks it as uninitialized.
   */
  bool Deinitialize() noexcept override;

  /**
   * @brief Check if the pin is available for GPIO operations.
   * @return true if pin is available, false if invalid pin number
   * @details Validates that the pin number is within valid range (0-15).
   */
  bool IsPinAvailable() const noexcept override;

  /**
   * @brief Get the maximum number of pins supported by PCAL95555.
   * @return 16 (PCAL95555 has 16 GPIO pins)
   */
  uint8_t GetMaxPins() const noexcept override;
  /**
   * @brief Get human-readable description of this GPIO instance.
   * @return Pointer to description string
   */
  const char* GetDescription() const noexcept override;

  //==============================================================//
  // INTERRUPT FUNCTIONALITY OVERRIDES
  //==============================================================//

  /**
   * @brief PCAL95555 has built-in interrupt-on-change capability.
   * @return false - Complex interrupt setup handled at chip level
   * @details PCAL95555 automatically triggers interrupts on pin state changes:
   *          - Built-in interrupt-on-change for all pins
   *          - Chip-level INT pin signals MCU when any monitored pin changes  
   *          - No additional configuration needed for basic change detection
   *          - Advanced interrupt configuration available at chip level
   * @note For most applications, polling pin state is sufficient.
   *       Complex interrupt handling should be implemented at the 
   *       PCAL95555 chip level, not individual pin level.
   */
  [[nodiscard]] bool SupportsInterrupts() const noexcept override;

  /**
   * @brief Configure GPIO interrupt settings.
   * @details Not supported at pin level. PCAL95555 interrupt features:
   *          - Automatic interrupt-on-change is always active
   *          - Chip-level interrupt mask can be configured via I2C
   *          - MCU INT pin setup handled by chip-level driver
   *          - I2C status reading managed by chip-level interrupt handler
   * @note Use chip-level interrupt configuration instead of pin-level
   */
  HfGpioErr ConfigureInterrupt(InterruptTrigger trigger, 
                               InterruptCallback callback = nullptr, 
                               void* user_data = nullptr) noexcept override;

  //==============================================================//
  // PCAL95555-SPECIFIC METHODS
  //==============================================================//

  /**
   * @brief Get the PCAL95555 chip pin number.
   * @return Chip pin number (0-15)
   */
  uint8_t GetChipPin() const noexcept { return chip_pin_; }

  /**
   * @brief Get the I2C address of this PCAL95555 chip.
   * @return I2C address (typically 0x20)
   */
  uint8_t GetI2cAddress() const noexcept { return i2c_address_; }

  /**
   * @brief Check if the PCAL95555 driver is available and responsive.
   * @return true if chip is responsive, false otherwise
   */
  bool IsChipResponsive() const noexcept;

protected:
  //==============================================================//
  // DIGITALGPIO PURE VIRTUAL IMPLEMENTATIONS
  //==============================================================//

  /**
   * @brief Platform-specific implementation for setting pin direction.
   * @param direction Desired pin direction (Input or Output)
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Configures the PCAL95555 pin direction register.
   */
  HfGpioErr SetDirectionImpl(Direction direction) noexcept override;

  /**
   * @brief Platform-specific implementation for setting output mode.
   * @param mode Desired output mode (PushPull or OpenDrain)
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Configures the PCAL95555 output drive strength and mode registers.
   */
  HfGpioErr SetOutputModeImpl(OutputMode mode) noexcept override;

  /**
   * @brief Platform-specific implementation for setting active state.
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Sets the PCAL95555 output register bit corresponding to logical active.
   */
  HfGpioErr SetActiveImpl() noexcept override;

  /**
   * @brief Platform-specific implementation for setting inactive state.
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Sets the PCAL95555 output register bit corresponding to logical inactive.
   */
  HfGpioErr SetInactiveImpl() noexcept override;

  /**
   * @brief Platform-specific implementation for reading pin active state.
   * @param is_active Output parameter: true if active, false if inactive
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Reads the PCAL95555 input register and converts to logical state.
   */
  HfGpioErr IsActiveImpl(bool& is_active) noexcept override;

  /**
   * @brief Platform-specific implementation for toggling pin state.
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Toggles the PCAL95555 output register bit.
   */
  HfGpioErr ToggleImpl() noexcept override;

  /**
   * @brief Platform-specific implementation for setting pull resistor mode.
   * @param mode Desired PullMode configuration (Floating, PullUp, or PullDown)
   * @return HfGpioErr::GPIO_SUCCESS if successful, error code otherwise
   * @details Configures the PCAL95555 pull-up/pull-down enable registers.
   */
  HfGpioErr SetPullModeImpl(PullMode mode) noexcept override;

  /**
   * @brief Platform-specific implementation for reading pull resistor mode.
   * @return Current PullMode configuration
   * @details Returns the cached pull resistor configuration.
   */
  PullMode GetPullModeImpl() const noexcept override;

private:
  //==============================================================//
  // PRIVATE MEMBERS
  //==============================================================//

  uint8_t chip_pin_;                            ///< PCAL95555 pin number (0-15)
  uint8_t i2c_address_;                         ///< I2C address of PCAL95555 chip
  std::shared_ptr<PCAL95555> pcal95555_driver_; ///< Shared PCAL95555 driver instance

  //==============================================================//
  // PRIVATE HELPERS
  //==============================================================//

  /**
   * @brief Validate chip pin number.
   * @return true if pin number is valid (0-15), false otherwise
   */
  bool ValidateChipPin() const noexcept;

  /**
   * @brief Get the bit mask for this pin.
   * @return Bit mask with only this pin's bit set
   */
  uint16_t GetPinMask() const noexcept;

  /**
   * @brief Convert I2C/driver errors to HfGpioErr codes.
   * @param success True if operation succeeded, false if failed
   * @return Appropriate HfGpioErr code
   */
  HfGpioErr ConvertDriverError(bool success) const noexcept;

  /**
   * @brief Apply the current configuration to the PCAL95555 chip.
   * @return HfGpioErr error code
   * @details Internal helper that configures direction, pull, and output mode.
   */
  HfGpioErr ApplyConfiguration() noexcept;
};

//==============================================================================
// FACTORY FUNCTIONS
//==============================================================================

/**
 * @brief Create a shared PCAL95555 driver instance.
 * @param i2c_bus Reference to the I2C bus to use
 * @param i2c_address I2C address of the PCAL95555 chip (default 0x20)
 * @return Shared pointer to PCAL95555 driver instance
 * @details Creates a shared PCAL95555 driver that can be used by multiple
 *          Pcal95555DigitalGpio instances for the same chip.
 */
std::shared_ptr<PCAL95555> CreatePcal95555Driver(SfI2cBus& i2c_bus, 
                                                 uint8_t i2c_address = 0x20);

/**
 * @brief Create a PCAL95555 GPIO pin instance.
 * @param chip_pin PCAL95555 pin number (0-15)
 * @param pcal95555_driver Shared PCAL95555 driver instance
 * @param i2c_address I2C address of the chip
 * @param direction Initial pin direction
 * @param active_state Polarity configuration
 * @param output_mode Output drive mode
 * @param pull_mode Pull resistor configuration
 * @return Unique pointer to Pcal95555DigitalGpio instance
 * @details Convenience factory function for creating PCAL95555 GPIO instances.
 */
std::unique_ptr<Pcal95555DigitalGpio> CreatePcal95555Pin(
    uint8_t chip_pin,
    std::shared_ptr<PCAL95555> pcal95555_driver,
    uint8_t i2c_address = 0x20,
    BaseGpio::Direction direction = BaseGpio::Direction::Input,
    BaseGpio::ActiveState active_state = BaseGpio::ActiveState::High,
    BaseGpio::OutputMode output_mode = BaseGpio::OutputMode::PushPull,
    BaseGpio::PullMode pull_mode = BaseGpio::PullMode::Floating);

#endif // PCAL95555DIGITALGPIO_H
