#ifndef COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_
#define COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_

#include <memory>
#include <vector>
#include <cstdint>

// Forward declarations for ESP32 comm interface classes
class EspSpiBus;
class EspSpiDevice;
class EspI2c;
class EspUart;
class EspCan;

// Base interface includes
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseSpi.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseI2c.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseUart.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseCan.h"

// Board mapping includes (for pin/bus config)
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config.hpp"

/**
 * @class CommChannelsManager
 * @brief Singleton for managing all board comm channels (SPI, I2C, UART, CAN).
 *
 * - Instantiates and owns all available comm buses for the board.
 * - Provides indexed accessors to the base interface for each bus.
 * - Board-agnostic: does not know about device purposes, only hardware buses.
 * - Uses board mapping for pin/bus configuration.
 * - Follows the singleton pattern (see GpioManager/AdcManager).
 */
class CommChannelsManager {
public:
    /**
     * @brief Get the singleton instance.
     */
    static CommChannelsManager& GetInstance() noexcept;

    // Non-copyable, non-movable
    CommChannelsManager(const CommChannelsManager&) = delete;
    CommChannelsManager& operator=(const CommChannelsManager&) = delete;
    CommChannelsManager(CommChannelsManager&&) = delete;
    CommChannelsManager& operator=(CommChannelsManager&&) = delete;

    //==================== Accessors ====================//
    /**
     * @brief Get reference to a SPI bus by index (host/device enum).
     */
    BaseSpi& GetSpi(std::size_t which = 0) noexcept;
    std::size_t GetSpiCount() const noexcept;

    /**
     * @brief Get reference to an I2C bus by index.
     */
    BaseI2c& GetI2c(std::size_t which = 0) noexcept;
    std::size_t GetI2cCount() const noexcept;

    /**
     * @brief Get reference to a UART bus by index.
     */
    BaseUart& GetUart(std::size_t which = 0) noexcept;
    std::size_t GetUartCount() const noexcept;

    /**
     * @brief Get reference to a CAN bus by index.
     */
    BaseCan& GetCan(std::size_t which = 0) noexcept;
    std::size_t GetCanCount() const noexcept;

private:
    CommChannelsManager();
    ~CommChannelsManager();

    // Internal storage for comm buses
    std::unique_ptr<EspSpiBus> spi_bus_;
    std::vector<std::unique_ptr<BaseSpi>> spi_devices_;
    std::vector<std::unique_ptr<EspI2c>> i2c_buses_;
    std::vector<std::unique_ptr<EspUart>> uart_buses_;
    std::vector<std::unique_ptr<EspCan>> can_buses_;
};

#endif // COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_ 