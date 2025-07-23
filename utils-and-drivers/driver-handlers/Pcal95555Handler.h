/**
 * @file Pcal95555Handler.h
 * @brief Minimal PCAL95555 I2C GPIO expander handler for direct pin control.
 *
 * This class provides a focused, platform-agnostic handler for the PCAL95555 16-bit I2C GPIO expander.
 * It exposes all pin operations by pin number (0-15), with no platform mapping or diagnostics.
 *
 * Features:
 * - Direct pin control (input/output, direction, pull, toggle, etc.)
 * - Thread-safe I2C communication
 * - Minimal, modern C++ interface
 * - No platform mapping, diagnostics, or functional pin abstractions
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#ifndef COMPONENT_HANDLER_PCAL95555_HANDLER_H_
#define COMPONENT_HANDLER_PCAL95555_HANDLER_H_

#include "base/BaseGpio.h"
#include "base/BaseI2c.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-pcal95555-driver/src/pcal95555.hpp"
#include "utils/RtosMutex.h"
#include <memory>
#include <cstdint>

/**
 * @brief Adapter to connect BaseI2c device to PCAL95555::i2cBus interface.
 * 
 * This adapter bridges the device-centric BaseI2c interface (where device address 
 * is pre-configured) with the PCAL95555 driver's i2cBus interface (which expects 
 * address parameters). The adapter validates that operations target the correct 
 * device and strips the address parameter before calling BaseI2c methods.
 */
class Pcal95555I2cAdapter : public PCAL95555::i2cBus {
public:
    /**
     * @brief Construct adapter with BaseI2c device reference.
     * @param i2c_device Reference to BaseI2c device (not bus)
     * @note The device address is already configured in the BaseI2c device
     */
    explicit Pcal95555I2cAdapter(BaseI2c& i2c_device) noexcept 
        : i2c_device_(i2c_device) {}

    /**
     * @brief Write data to device register via BaseI2c device.
     * @param addr I2C device address (validated against BaseI2c device address)
     * @param reg Register address
     * @param data Data buffer to write
     * @param len Number of bytes to write
     * @return true if successful, false on error
     * @note Address validation ensures type safety - prevents accidental cross-device communication
     */
    bool write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) override;

    /**
     * @brief Read data from device register via BaseI2c device.
     * @param addr I2C device address (validated against BaseI2c device address)
     * @param reg Register address
     * @param data Buffer to store read data
     * @param len Number of bytes to read
     * @return true if successful, false on error
     * @note Address validation ensures type safety - prevents accidental cross-device communication
     */
    bool read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) override;

private:
    BaseI2c& i2c_device_;           ///< Reference to I2C device (not bus)
    mutable RtosMutex i2c_mutex_;   ///< Thread safety for I2C operations
};

// ===================== Per-Pin BaseGpio Wrapper ===================== //
class Pcal95555GpioPin : public BaseGpio {
public:
    Pcal95555GpioPin(hf_pin_num_t pin,
                     std::shared_ptr<PCAL95555> driver,
                     hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
                     hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
                     hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
                     hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;
    ~Pcal95555GpioPin() override = default;

    hf_bool_t Initialize() noexcept override;
    hf_bool_t Deinitialize() noexcept override;
    hf_bool_t IsPinAvailable() const noexcept override;
    hf_u8_t GetMaxPins() const noexcept override { return 16; }
    const char* GetDescription() const noexcept override;
    hf_bool_t SupportsInterrupts() const noexcept override { return true; }
    hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                     InterruptCallback callback = nullptr,
                                     void* user_data = nullptr) noexcept override;
    // Advanced PCAL95555A features
    hf_gpio_err_t SetPolarityInversion(hf_bool_t invert) noexcept;
    hf_gpio_err_t GetPolarityInversion(hf_bool_t& invert) noexcept;
    hf_gpio_err_t SetInterruptMask(hf_bool_t mask) noexcept;
    hf_gpio_err_t GetInterruptMask(hf_bool_t& mask) noexcept;
    hf_gpio_err_t GetInterruptStatus(hf_bool_t& status) noexcept;

protected:
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;
    hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;
    hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;
    hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;
    hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;
    hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;

private:
    hf_pin_num_t pin_;
    std::shared_ptr<PCAL95555> driver_;
    mutable RtosMutex pin_mutex_;
    char description_[32] = {};
};

// ===================== Handler Factory & Advanced Features ===================== //
class Pcal95555Handler {
public:
    /**
     * @brief Construct a new handler for a PCAL95555 chip.
     * @param i2c_device Reference to BaseI2c device (not bus)
     * @note The device address should already be configured in the BaseI2c device
     */
    explicit Pcal95555Handler(BaseI2c& i2c_device) noexcept;
    ~Pcal95555Handler() = default;

    // No copy
    Pcal95555Handler(const Pcal95555Handler&) = delete;
    Pcal95555Handler& operator=(const Pcal95555Handler&) = delete;
    // Allow move
    Pcal95555Handler(Pcal95555Handler&&) = default;
    Pcal95555Handler& operator=(Pcal95555Handler&&) = default;

    // Batch operations
    hf_gpio_err_t SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept;
    hf_gpio_err_t SetOutputs(uint16_t pin_mask, hf_bool_t active) noexcept;
    hf_gpio_err_t SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept;

    // Get all interrupt masks and status
    hf_gpio_err_t GetAllInterruptMasks(uint16_t& mask) noexcept;
    hf_gpio_err_t GetAllInterruptStatus(uint16_t& status) noexcept;

    // Update all single-pin methods to return hf_gpio_err_t
    hf_gpio_err_t Initialize() noexcept;
    hf_gpio_err_t Deinitialize() noexcept;
    hf_gpio_err_t SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept;
    hf_gpio_err_t SetOutput(hf_u8_t pin, hf_bool_t active) noexcept;
    hf_gpio_err_t ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept;
    hf_gpio_err_t Toggle(hf_u8_t pin) noexcept;
    hf_gpio_err_t SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept;
    hf_gpio_err_t GetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept;

    /**
     * @brief Get the number of pins (always 16).
     */
    static constexpr hf_u8_t PinCount() noexcept { return 16; }

    /**
     * @brief Get the I2C address.
     */
    hf_u8_t GetI2cAddress() const noexcept { return i2c_address_; }

    /**
     * @brief Create a BaseGpio-compatible pin wrapper for a PCAL95555 pin.
     * @param pin Pin number (0-15)
     * @param direction Initial direction
     * @param active_state Active polarity
     * @param output_mode Output mode
     * @param pull_mode Pull resistor mode
     * @return std::unique_ptr<BaseGpio> or nullptr if invalid
     */
    std::unique_ptr<BaseGpio> CreateGpioPin(
        hf_pin_num_t pin,
        hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;

    // Advanced PCAL95555A features
    hf_bool_t SetPolarityInversion(hf_pin_num_t pin, hf_bool_t invert) noexcept;
    hf_bool_t GetPolarityInversion(hf_pin_num_t pin, hf_bool_t& invert) noexcept;
    hf_bool_t SetInterruptMask(hf_pin_num_t pin, hf_bool_t mask) noexcept;
    hf_bool_t GetInterruptMask(hf_pin_num_t pin, hf_bool_t& mask) noexcept;
    hf_bool_t GetInterruptStatus(hf_pin_num_t pin, hf_bool_t& status) noexcept;
    hf_bool_t SoftwareReset() noexcept;
    hf_bool_t PowerDown() noexcept;

private:
    std::unique_ptr<Pcal95555I2cAdapter> i2c_adapter_;      ///< I2C adapter bridging BaseI2c to PCAL95555
    std::shared_ptr<PCAL95555> pcal95555_driver_;           ///< PCAL95555 driver instance
    hf_bool_t initialized_ = false;                         ///< Initialization status
    mutable RtosMutex handler_mutex_;                       ///< Thread safety mutex

    hf_bool_t ValidatePin(hf_u8_t pin) const noexcept { return pin < 16; }
    
    /**
     * @brief Architecture Note: Address Handling
     * 
     * This design implements device-centric I2C communication:
     * 
     * 1. **BaseI2c Device**: Pre-configured with device address, no address params needed
     * 2. **PCAL95555 Driver**: Stores address internally for validation and state management  
     * 3. **Pcal95555I2cAdapter**: Validates address consistency between the two layers
     * 
     * The adapter serves as a bridge between:
     * - PCAL95555 driver API (expects address parameters)  
     * - BaseI2c device API (address pre-configured, no parameters)
     * 
     * This provides both type safety (can't use wrong device) and compatibility
     * with existing driver interfaces without requiring modification.
     */
};

#endif // COMPONENT_HANDLER_PCAL95555_HANDLER_H_ 