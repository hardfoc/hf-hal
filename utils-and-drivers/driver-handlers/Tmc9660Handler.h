#ifndef COMPONENT_HANDLER_TMC9660_HANDLER_H_
#define COMPONENT_HANDLER_TMC9660_HANDLER_H_

#include <cstdint>
#include <memory>
#include <array>
#include "utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/TMC9660.hpp"
#include "base/BaseGpio.h"
#include "base/BaseAdc.h"

/**
 * @file Tmc9660Handler.h
 * @brief Unified handler for TMC9660 motor controller, GPIO, and ADC integration.
 *
 * This class provides a modern, unified interface for a single TMC9660 device.
 * - Owns a TMC9660 driver instance
 * - Exposes BaseGpio and BaseAdc compatible wrappers for internal GPIO/ADC
 * - Exposes the TMC9660 instance for advanced parameter access
 * - Accepts a bootloader config (defaulted, but user-overridable)
 * - Handles initialization, error reporting, and safe access patterns
 * - Designed for single-device use (multi-device can be handled by user if needed)
 */
class Tmc9660Handler {
public:
    /**
     * @brief Default bootloader configuration (can be overridden at construction).
     *        These values should be tuned for your hardware.
     */
    static constexpr tmc9660::BootloaderConfig kDefaultBootConfig = {
        .spiFrequency = 10000000, // 10 MHz
        .uartBaudRate = 115200,   // 115200 baud
        // Add other fields as needed for your hardware
    };

    /**
     * @brief Construct a Tmc9660Handler with optional custom bootloader config.
     * @param comm Reference to a TMC9660CommInterface (SPI/UART implementation)
     * @param address 7-bit device address
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     */
    Tmc9660Handler(TMC9660CommInterface& comm, uint8_t address,
                  const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

    ~Tmc9660Handler();

    /**
     * @brief Initialize the TMC9660 device (bootloader, parameter mode, etc).
     * @return true if successful, false otherwise
     */
    bool Initialize();

    /**
     * @brief Get direct access to the underlying TMC9660 driver instance.
     *        Use this for advanced parameter access and tweaking.
     */
    TMC9660& driver() noexcept { return tmc9660_; }

    /**
     * @brief GPIO wrapper for TMC9660 internal GPIO channels.
     *        Implements BaseGpio interface.
     */
    class Gpio : public BaseGpio {
    public:
        Gpio(Tmc9660Handler& parent, uint8_t gpioNumber);
        ~Gpio() override = default;
        bool Initialize() noexcept override;
        bool Deinitialize() noexcept override;
        hf_gpio_err_t SetActive() noexcept override;
        hf_gpio_err_t SetInactive() noexcept override;
        hf_gpio_err_t Toggle() noexcept override;
        hf_gpio_err_t IsActive(bool& is_active) noexcept override;
        hf_gpio_err_t GetState(hf_gpio_state_t& state) noexcept override;
        hf_gpio_err_t SetState(hf_gpio_state_t state) noexcept override;
        hf_gpio_err_t ValidatePin() noexcept override;
        hf_gpio_direction_t GetDirection() const noexcept override;
        bool IsPinAvailable() const noexcept override;
        hf_u8_t GetMaxPins() const noexcept override;
        const char* GetDescription() const noexcept override;
    private:
        Tmc9660Handler& parent_;
        uint8_t gpioNumber_;
        mutable char description_[64];
    };

    /**
     * @brief ADC wrapper for TMC9660 internal ADC channels.
     *        Implements BaseAdc interface.
     */
    class Adc : public BaseAdc {
    public:
        Adc(Tmc9660Handler& parent);
        ~Adc() override = default;
        bool Initialize() noexcept override;
        bool Deinitialize() noexcept override;
        hf_u8_t GetMaxChannels() const noexcept override;
        bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;
        hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                 hf_u8_t numOfSamplesToAvg = 1,
                                 hf_time_t timeBetweenSamples = 0) noexcept override;
        hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                     hf_u8_t numOfSamplesToAvg = 1,
                                     hf_time_t timeBetweenSamples = 0) noexcept override;
        hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                               float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                               hf_time_t timeBetweenSamples = 0) noexcept override;
    private:
        Tmc9660Handler& parent_;
    };

    /**
     * @brief Get a reference to the GPIO wrapper for a given internal GPIO channel.
     * @param gpioNumber The TMC9660 internal GPIO number (e.g., 17, 18)
     * @return Reference to the GPIO wrapper
     */
    Gpio& gpio(uint8_t gpioNumber);

    /**
     * @brief Get a reference to the ADC wrapper.
     */
    Adc& adc();

    /**
     * @brief Get the current bootloader config in use.
     */
    const tmc9660::BootloaderConfig& bootConfig() const noexcept { return *bootCfg_; }

private:
    TMC9660 tmc9660_;
    std::array<std::unique_ptr<Gpio>, 2> gpioWrappers_; // e.g., for GPIO17, GPIO18
    std::unique_ptr<Adc> adcWrapper_;
    const tmc9660::BootloaderConfig* bootCfg_;
};

#endif // COMPONENT_HANDLER_TMC9660_HANDLER_H_ 