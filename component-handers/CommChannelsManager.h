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
 * @enum SpiDeviceId
 * @brief Enumeration for SPI device identification on the board.
 * 
 * This enum provides a type-safe way to access specific SPI devices by their
 * functional purpose rather than numeric indices. The order matches the CS pin
 * configuration in kSpiCsPins array.
 */
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0,  ///< TMC9660 motor controller (SPI Mode 3)
    AS5047U_POSITION_ENCODER = 1,  ///< AS5047U position encoder (SPI Mode 1)
    EXTERNAL_DEVICE_1 = 2,         ///< External device 1 (SPI Mode 0)
    EXTERNAL_DEVICE_2 = 3,         ///< External device 2 (SPI Mode 0)
    
    // Aliases for common usage
    MOTOR_CONTROLLER = TMC9660_MOTOR_CONTROLLER,
    POSITION_ENCODER = AS5047U_POSITION_ENCODER,
    
    SPI_DEVICE_COUNT  ///< Total number of SPI devices
};

/**
 * @class CommChannelsManager
 * @brief Singleton for managing all board comm channels (SPI, I2C, UART, CAN).
 *
 * - Instantiates and owns all available comm buses for the board.
 * - Provides indexed accessors to the base interface for each bus.
 * - Board-agnostic: does not know about device purposes, only hardware buses.
 * - Uses board mapping for pin/bus configuration.
 * - Follows the singleton pattern (see GpioManager/AdcManager).
 * 
 * ## UART Configuration for TMC9660
 * - UART0 configured for TMC9660 TMCL protocol communication
 * - Baud rate: 115200, Format: 8N1 (8 data bits, no parity, 1 stop bit)
 * - Pins: TX=GPIO5, RX=GPIO4 (per vortex-v1 board pin mapping)
 * - Buffers: TX=256 bytes, RX=512 bytes (optimized for 9-byte TMCL frames)
 * - Mode: Interrupt-driven with event queue for reliable communication
 * - Compatible with ESP-IDF v5.5+ UART driver features
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

    //==================== INITIALZATION ====================//
    
    // Lazy Initialization
    bool EnsureInitialized() noexcept {
        if (!initialized_) {
            initialized_ = Initialize();
        }
        return initialized_;
    }
    bool EnsureDeinitialized() noexcept {
        if (initialized_) {
            initialized_ = !Deinitialize();
        }
        return !initialized_;
    }

    [[nodiscard]] bool IsInitialized() const noexcept {
        return initialized_;
    }

    //==================== SPI Accessors ====================//
    /**
     * @brief Get reference to the SPI bus.
     * @return Reference to the SPI bus
     */
    EspSpiBus& GetSpiBus() noexcept;
    
    /**
     * @brief Get reference to a SPI device by index.
     * @param device_index Index of the device on the bus
     * @return Pointer to BaseSpi device, or nullptr if invalid
     */
    BaseSpi* GetSpiDevice(int device_index) noexcept;

    /**
     * @brief Get reference to a SPI device by device ID (enumeration-based access).
     * @param device_id Device identifier from SpiDeviceId enum
     * @return Pointer to BaseSpi device, or nullptr if invalid
     */
    BaseSpi* GetSpiDevice(SpiDeviceId device_id) noexcept;
    
    /**
     * @brief Get ESP-specific SPI device by index.
     * @param device_index Index of the device on the bus
     * @return Pointer to EspSpiDevice, or nullptr if invalid
     */
    EspSpiDevice* GetEspSpiDevice(int device_index) noexcept;

    /**
     * @brief Get ESP-specific SPI device by device ID (enumeration-based access).
     * @param device_id Device identifier from SpiDeviceId enum
     * @return Pointer to EspSpiDevice, or nullptr if invalid
     */
    EspSpiDevice* GetEspSpiDevice(SpiDeviceId device_id) noexcept;

    /**
     * @brief Get number of SPI devices on the bus.
     * @return Number of SPI devices
     */
    std::size_t GetSpiDeviceCount() const noexcept;

    //==================== Convenience SPI Device Accessors ====================//
    
    /**
     * @brief Get the TMC9660 motor controller device.
     * @return Pointer to BaseSpi device for TMC9660, or nullptr if not available
     */
    BaseSpi* GetMotorController() noexcept {
        return GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    }
    
    /**
     * @brief Get the AS5047U position encoder device.
     * @return Pointer to BaseSpi device for AS5047U, or nullptr if not available
     */
    BaseSpi* GetPositionEncoder() noexcept {
        return GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    }
    
    /**
     * @brief Get external SPI device 1.
     * @return Pointer to BaseSpi device for external device 1, or nullptr if not available
     */
    BaseSpi* GetExternalDevice1() noexcept {
        return GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_1);
    }
    
    /**
     * @brief Get external SPI device 2.
     * @return Pointer to BaseSpi device for external device 2, or nullptr if not available
     */
    BaseSpi* GetExternalDevice2() noexcept {
        return GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_2);
    }

    /**
     * @brief Get the TMC9660 motor controller device (ESP-specific interface).
     * @return Pointer to EspSpiDevice for TMC9660, or nullptr if not available
     */
    EspSpiDevice* GetEspMotorController() noexcept {
        return GetEspSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    }
    
    /**
     * @brief Get the AS5047U position encoder device (ESP-specific interface).
     * @return Pointer to EspSpiDevice for AS5047U, or nullptr if not available
     */
    EspSpiDevice* GetEspPositionEncoder() noexcept {
        return GetEspSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    }

    //==================== I2C Accessors ====================//

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
     * @brief Get reference to TMC9660 UART communication interface.
     * @return Reference to the UART configured for TMC9660 TMCL protocol
     * @note This is a convenience method that returns GetUart(0) with proper documentation
     */
    BaseUart& GetTmc9660Uart() noexcept { return GetUart(0); }

    /**
     * @brief Get reference to a CAN bus by index.
     */
    BaseCan& GetCan(std::size_t which = 0) noexcept;
    std::size_t GetCanCount() const noexcept;

private:
    CommChannelsManager();
    ~CommChannelsManager();

    /**
     * @brief Initialize all comm channels.
     * @return true if successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Deinitialize all comm channels.
     * @return true if successful, false otherwise
     */
    bool Deinitialize() noexcept;

    // Initialization state
    bool initialized_ = false;

    // Static configuration for SPI CS pins
    static constexpr HfFunctionalGpioPin kSpiCsPins[] = {
        HfFunctionalGpioPin::SPI2_CS_TMC9660,
        HfFunctionalGpioPin::SPI2_CS_AS5047,
        HfFunctionalGpioPin::EXT_GPIO_CS_1,
        HfFunctionalGpioPin::EXT_GPIO_CS_2,
    };
    static constexpr std::size_t kSpiCsPinCount = sizeof(kSpiCsPins) / sizeof(kSpiCsPins[0]);

    // Internal storage for comm buses
    std::unique_ptr<EspSpiBus> spi_bus_;
    std::vector<int> spi_device_indices_; ///< Track device indices for easy access

    // Internal vectors for managing multiple buses
    std::vector<std::unique_ptr<EspI2c>> i2c_buses_;
    std::vector<std::unique_ptr<EspUart>> uart_buses_;
    std::vector<std::unique_ptr<EspCan>> can_buses_;
};

#endif // COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_ 