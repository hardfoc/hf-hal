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
#include <memory>
#include <mutex>
#include <cstdint>

/**
 * @brief Adapter to connect BaseI2c to PCAL95555::i2cBus interface.
 */
class Pcal95555I2cAdapter : public PCAL95555::i2cBus {
public:
    explicit Pcal95555I2cAdapter(BaseI2c& i2c_bus, uint8_t i2c_address) noexcept
        : i2c_bus_(i2c_bus), i2c_address_(i2c_address) {}

    bool write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) override;
    bool read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) override;

private:
    BaseI2c& i2c_bus_;
    uint8_t i2c_address_;
    mutable std::mutex i2c_mutex_;
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
    hf_gpio_err_t SetActiveImpl() noexcept override;
    hf_gpio_err_t SetInactiveImpl() noexcept override;
    hf_gpio_err_t IsActiveImpl(hf_bool_t& is_active) noexcept override;
    hf_gpio_err_t ToggleImpl() noexcept override;

private:
    hf_pin_num_t pin_;
    std::shared_ptr<PCAL95555> driver_;
    mutable std::mutex pin_mutex_;
    char description_[32] = {};
};

// ===================== Handler Factory & Advanced Features ===================== //
class Pcal95555Handler {
public:
    /**
     * @brief Construct a new handler for a PCAL95555 chip.
     * @param i2c_bus Reference to BaseI2c bus
     * @param i2c_address 7-bit I2C address (default: 0x20)
     */
    explicit Pcal95555Handler(BaseI2c& i2c_bus, hf_u8_t i2c_address = 0x20) noexcept;
    ~Pcal95555Handler() = default;

    // No copy
    Pcal95555Handler(const Pcal95555Handler&) = delete;
    Pcal95555Handler& operator=(const Pcal95555Handler&) = delete;
    // Allow move
    Pcal95555Handler(Pcal95555Handler&&) = default;
    Pcal95555Handler& operator=(Pcal95555Handler&&) = default;

    /**
     * @brief Initialize the chip (idempotent).
     * @return true if successful
     */
    hf_bool_t Initialize() noexcept;

    /**
     * @brief Deinitialize the chip (idempotent).
     * @return true if successful
     */
    hf_bool_t Deinitialize() noexcept;

    /**
     * @brief Set pin direction.
     * @param pin Pin number (0-15)
     * @param direction Input or output
     * @return true if successful
     */
    hf_bool_t SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept;

    /**
     * @brief Set pin output value.
     * @param pin Pin number (0-15)
     * @param active true for active, false for inactive
     * @return true if successful
     */
    hf_bool_t SetOutput(hf_u8_t pin, hf_bool_t active) noexcept;

    /**
     * @brief Read pin input value.
     * @param pin Pin number (0-15)
     * @param active Output: true if active
     * @return true if successful
     */
    hf_bool_t ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept;

    /**
     * @brief Toggle pin output value.
     * @param pin Pin number (0-15)
     * @return true if successful
     */
    hf_bool_t Toggle(hf_u8_t pin) noexcept;

    /**
     * @brief Set pin pull resistor mode.
     * @param pin Pin number (0-15)
     * @param pull_mode Pull mode
     * @return true if successful
     */
    hf_bool_t SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept;

    /**
     * @brief Get pin pull resistor mode.
     * @param pin Pin number (0-15)
     * @param pull_mode Output: current pull mode
     * @return true if successful
     */
    hf_bool_t GetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept;

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
    std::unique_ptr<Pcal95555I2cAdapter> i2c_adapter_;
    std::shared_ptr<PCAL95555> pcal95555_driver_;
    hf_u8_t i2c_address_;
    hf_bool_t initialized_ = false;
    mutable std::mutex handler_mutex_;

    hf_bool_t ValidatePin(hf_u8_t pin) const noexcept { return pin < 16; }
};

#endif // COMPONENT_HANDLER_PCAL95555_HANDLER_H_ 