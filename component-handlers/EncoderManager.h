#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <array>
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"

// Forward declarations
class As5047uHandler;
class CommChannelsManager;
class GpioManager;
class BaseGpio;

/**
 * @class EncoderManager
 * @brief Singleton for managing multiple AS5047U encoder devices with indexed access and flexible device management.
 *
 * This manager provides a global singleton for managing AS5047U encoder devices, following the same pattern as ImuManager:
 * - Always creates and manages the onboard AS5047U encoder device (device index 0)
 * - Allows dynamic creation/deletion of external AS5047U devices (indices 1-3)
 * - Array-based access to AS5047U handler instances by device index
 * - Thread-safe device registration and access
 * - Board-aware device management with predefined SPI assignments
 * - Currently supports AS5047U encoder family (extensible for other encoder types)
 *
 * **Key Features:**
 * - Multiple AS5047U device support with indexed access
 * - Lazy initialization of onboard AS5047U device
 * - Dynamic external device creation/deletion
 * - Thread-safe singleton pattern with RtosMutex
 * - Exception-free operation with pointer-based returns
 * - Proper device initialization and lifecycle management
 * - ESP-IDF v5.5+ SPI integration
 * - Interrupt support for encoder data ready signals
 *
 * **Device Indexing:**
 * - Index 0: Onboard AS5047U encoder (always available, auto-created)
 * - Index 1: External encoder device 1 (optional, user-created)
 * - Index 2: External encoder device 2 (optional, user-created)
 * - Index 3: External encoder device 3 (optional, user-created)
 *
 * **Usage Example:**
 * @code
 * auto& encoder_mgr = EncoderManager::GetInstance();
 * if (encoder_mgr.EnsureInitialized()) {
 *     // Access onboard AS5047U (index 0)
 *     As5047uHandler* onboard_handler = encoder_mgr.GetAs5047uHandler(0);
 *     if (onboard_handler) {
 *         // Read encoder angle
 *         uint16_t angle;
 *         if (onboard_handler->ReadAngle(angle) == As5047uError::SUCCESS) {
 *             printf("Angle: %u LSB\n", angle);
 *         }
 *     }
 *     
 *     // Create external AS5047U device (index 1) with SPI interface
 *     if (encoder_mgr.CreateExternalAs5047uDevice(1, SpiDeviceId::EXTERNAL_DEVICE_1)) {
 *         As5047uHandler* external_handler = encoder_mgr.GetAs5047uHandler(1);
 *         if (external_handler) {
 *             // Configure and use external encoder
 *             external_handler->SetZeroPosition(0);
 *         }
 *     }
 * }
 * @endcode
 *
 * @note This manager follows the same architectural excellence as ImuManager
 *       for consistent device management across the HardFOC system.
 */
class EncoderManager {
public:
    static constexpr uint8_t MAX_ENCODER_DEVICES = 4;           ///< Maximum supported AS5047U devices
    static constexpr uint8_t ONBOARD_ENCODER_INDEX = 0;         ///< Onboard AS5047U device index
    static constexpr uint8_t EXTERNAL_ENCODER_1_INDEX = 1;      ///< External encoder device 1 index
    static constexpr uint8_t EXTERNAL_ENCODER_2_INDEX = 2;      ///< External encoder device 2 index
    static constexpr uint8_t EXTERNAL_ENCODER_3_INDEX = 3;      ///< External encoder device 3 index

public:
    /**
     * @brief Get the singleton instance of EncoderManager.
     * @return Reference to the singleton EncoderManager.
     */
    static EncoderManager& GetInstance() noexcept;

    //**************************************************************************//
    //**                  DEVICE MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Ensure the encoder manager system is initialized.
     * @note This automatically creates the onboard AS5047U encoder device using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Check if the encoder manager is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Deinitialize all encoders and release resources.
     * @return true if deinitialized successfully, false otherwise
     */
    bool Deinitialize() noexcept;

    //**************************************************************************//
    //**                  HANDLER AND DRIVER MANAGEMENT                       **//
    //**************************************************************************//

    /**
     * @brief Get access to AS5047U encoder handler by device index.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Pointer to As5047uHandler if valid and active, nullptr otherwise
     * @note Returns nullptr if deviceIndex is invalid, device not active, or not initialized
     */
    As5047uHandler* GetAs5047uHandler(uint8_t deviceIndex = ONBOARD_ENCODER_INDEX) noexcept;

    /**
     * @brief Get access to the underlying AS5047U driver by device index.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Shared pointer to AS5047U driver, nullptr if invalid/not ready
     * @note Returns nullptr if deviceIndex is invalid, device not active/initialized, or driver unavailable
     */
    std::shared_ptr<AS5047U> GetAs5047uDriver(uint8_t deviceIndex = ONBOARD_ENCODER_INDEX) noexcept;

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create an external AS5047U encoder device on SPI interface.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @param spiDeviceId SPI device ID for communication
     * @param config Optional AS5047U configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     */
    bool CreateExternalAs5047uDevice(uint8_t deviceIndex, 
                                   SpiDeviceId spiDeviceId,
                                   const As5047uConfig& config = As5047uHandler::GetDefaultConfig());

    /**
     * @brief Create an external AS5047U encoder device using direct BaseSpi interface.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @param spi_interface Direct reference to BaseSpi interface
     * @param config Optional AS5047U configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     * @note This is the most flexible method - allows any external SPI interface
     */
    bool CreateExternalAs5047uDevice(uint8_t deviceIndex, 
                                   BaseSpi& spi_interface,
                                   const As5047uConfig& config = As5047uHandler::GetDefaultConfig());

    /**
     * @brief Delete an external encoder device.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @return true if device deleted successfully, false otherwise
     * @note Cannot delete onboard device (index 0). Only external devices can be deleted.
     */
    bool DeleteExternalDevice(uint8_t deviceIndex);

    /**
     * @brief Get the number of active encoder devices.
     * @return Number of active devices (1 to MAX_ENCODER_DEVICES)
     * @note Always includes onboard device, plus any active external devices
     */
    uint8_t GetDeviceCount() const noexcept;

    /**
     * @brief Check if a device index is valid and has an active device.
     * @param deviceIndex Device index to check
     * @return true if device exists and is active, false otherwise
     */
    bool IsDeviceValid(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Check if an external device slot is available for creation.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @return true if slot is available, false if occupied or invalid index
     */
    bool IsExternalSlotAvailable(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Get list of active device indices.
     * @return Vector of active device indices
     */
    std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;

    /**
     * @brief Initialize all devices and report status.
     * @return Vector of initialization results (true/false) for each active device
     */
    std::vector<bool> InitializeAllDevices();

    /**
     * @brief Get initialization status for all devices.
     * @return Vector of initialization status for each active device
     */
    std::vector<bool> GetInitializationStatus() const;

    //**************************************************************************//
    //**                  DEVICE INFORMATION METHODS                          **//
    //**************************************************************************//

    /**
     * @brief Get information about available encoder devices.
     * @return Vector of device names/identifiers
     */
    std::vector<std::string> GetAvailableDevices() const noexcept;

    /**
     * @brief Get device type by index.
     * @param deviceIndex Device index
     * @return Device type string or "Unknown" if invalid
     */
    std::string GetDeviceType(uint8_t deviceIndex) const noexcept;

    //**************************************************************************//
    //**                  HIGH-LEVEL ENCODER OPERATIONS                       **//
    //**************************************************************************//

    /**
     * @brief Read angle from specific encoder device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param angle Output angle value (0-16383 LSB, 14-bit)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadAngle(uint8_t deviceIndex, uint16_t& angle) noexcept;

    /**
     * @brief Read angle in degrees from specific encoder device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param angle_degrees Output angle in degrees (0.0-359.978°)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadAngleDegrees(uint8_t deviceIndex, double& angle_degrees) noexcept;

    /**
     * @brief Read velocity from specific encoder device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param velocity_rpm Output velocity in RPM
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadVelocityRPM(uint8_t deviceIndex, double& velocity_rpm) noexcept;

    /**
     * @brief Read diagnostics from specific encoder device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param diagnostics Output diagnostic information
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadDiagnostics(uint8_t deviceIndex, As5047uDiagnostics& diagnostics) noexcept;

    /**
     * @brief Set zero position for specific encoder device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param zero_position Zero position in LSB (0-16383)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetZeroPosition(uint8_t deviceIndex, uint16_t zero_position) noexcept;

    /**
     * @brief Read angles from all active encoder devices.
     * @param angles Output vector of angle values (one per active device)
     * @param device_indices Output vector of corresponding device indices
     * @return Vector of error codes (one per device)
     */
    std::vector<As5047uError> ReadAllAngles(std::vector<uint16_t>& angles, 
                                          std::vector<uint8_t>& device_indices) noexcept;

    /**
     * @brief Read velocities from all active encoder devices.
     * @param velocities_rpm Output vector of velocity values in RPM
     * @param device_indices Output vector of corresponding device indices
     * @return Vector of error codes (one per device)
     */
    std::vector<As5047uError> ReadAllVelocities(std::vector<double>& velocities_rpm, 
                                              std::vector<uint8_t>& device_indices) noexcept;

    /**
     * @brief Check health status of all active encoder devices.
     * @return true if all active devices are healthy, false if any have errors
     */
    bool CheckAllDevicesHealth() noexcept;

    //**************************************************************************//
    //**                  INTERRUPT SUPPORT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Configure GPIO interrupt for AS5047U data ready/error pin on specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param callback Optional callback function executed in interrupt context (keep minimal)
     * @return true if interrupt configuration successful, false otherwise
     * @note Uses appropriate functional pin through GpioManager for onboard device
     * @note External devices may use different interrupt pins
     */
    bool ConfigureInterrupt(uint8_t deviceIndex, std::function<void()> callback = nullptr) noexcept;

    /**
     * @brief Enable AS5047U GPIO interrupt for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return true if interrupt enabled successfully, false otherwise
     * @note Must call ConfigureInterrupt() first
     */
    bool EnableInterrupt(uint8_t deviceIndex) noexcept;

    /**
     * @brief Disable AS5047U GPIO interrupt for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return true if interrupt disabled successfully, false otherwise
     */
    bool DisableInterrupt(uint8_t deviceIndex) noexcept;

    /**
     * @brief Check if interrupt is configured and enabled for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return true if interrupt is active, false otherwise
     */
    bool IsInterruptEnabled(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Wait for interrupt signal with timeout for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param timeout_ms Timeout in milliseconds (0 = wait indefinitely)
     * @return true if interrupt occurred, false on timeout
     * @note Useful for interrupt-driven processing in tasks
     */
    bool WaitForInterrupt(uint8_t deviceIndex, uint32_t timeout_ms = 0) noexcept;

    /**
     * @brief Get interrupt statistics for monitoring for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Number of interrupts processed since initialization
     */
    uint32_t GetInterruptCount(uint8_t deviceIndex) const noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all device statistics, interrupt counts, and system health information.
     */
    void DumpStatistics() const noexcept;

    // Delete copy/move constructors and assignment operators
    EncoderManager(const EncoderManager&) = delete;
    EncoderManager& operator=(const EncoderManager&) = delete;
    EncoderManager(EncoderManager&&) = delete;
    EncoderManager& operator=(EncoderManager&&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    EncoderManager() noexcept;

    /**
     * @brief Private destructor.
     */
    ~EncoderManager() noexcept;

    /**
     * @brief Initialize the encoder manager system.
     * @note This automatically creates the onboard AS5047U encoder device using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Initialize the onboard AS5047U encoder device with SPI transport.
     * @return true if initialization successful, false otherwise
     */
    bool InitializeOnboardAs5047uDevice() noexcept;

    /**
     * @brief Initialize GPIO interrupt for AS5047U data ready pin.
     * @return true if GPIO setup successful, false otherwise
     */
    bool InitializeInterruptGpio() noexcept;

    /**
     * @brief Validate if device index is for external device.
     * @param deviceIndex Device index to validate
     * @return true if index is for external device (1, 2, or 3), false otherwise
     */
    bool IsExternalDeviceIndex(uint8_t deviceIndex) const noexcept;

    /**
     * @brief GPIO interrupt handler for AS5047U data ready pin.
     * @param gpio Pointer to the GPIO that triggered interrupt
     * @param trigger Interrupt trigger type
     * @param user_data Pointer to user data (EncoderManager instance)
     */
    static void GpioInterruptHandler(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept;

    // ===============================
    // SYSTEM STATE
    // ===============================

    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> initialized_{false};

    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex manager_mutex_;

    // ===============================
    // DEVICE STORAGE
    // ===============================

    /**
     * @brief Array of AS5047U encoder handlers (one per device slot).
     * Uses unique_ptr for exclusive ownership of each handler.
     * Protected by RtosMutex for thread-safe access.
     */
    std::array<std::unique_ptr<As5047uHandler>, MAX_ENCODER_DEVICES> as5047u_handlers_;

    /**
     * @brief Device initialization status tracking.
     * Tracks which devices have been successfully initialized.
     */
    std::array<bool, MAX_ENCODER_DEVICES> device_initialized_;

    /**
     * @brief Device active status tracking.
     * Tracks which device slots have active devices.
     */
    std::array<bool, MAX_ENCODER_DEVICES> device_active_;

    /**
     * @brief Onboard device creation tracking.
     * Tracks if onboard device has been created.
     */
    bool onboard_device_created_ = false;

    // ===============================
    // DEPENDENCIES
    // ===============================

    /**
     * @brief Reference to communication manager for SPI access.
     * Used for creating encoder devices with proper transport interfaces.
     */
    CommChannelsManager* comm_manager_ = nullptr;

    /**
     * @brief Reference to GPIO manager for interrupt pin access.
     * Used for configuring interrupt pins for encoder devices.
     */
    GpioManager* gpio_manager_ = nullptr;

    // ===============================
    // INTERRUPT SUPPORT
    // ===============================

    /**
     * @brief GPIO interrupt support for onboard device.
     * External devices may have their own interrupt configurations.
     */
    BaseGpio* interrupt_gpio_ = nullptr;                    ///< Raw pointer for compatibility
    std::shared_ptr<BaseGpio> interrupt_gpio_shared_;       ///< Shared pointer for safe ownership
    std::function<void()> interrupt_callback_;
    bool interrupt_configured_ = false;
    bool interrupt_enabled_ = false;
    std::atomic<uint32_t> interrupt_count_{0};
    void* interrupt_semaphore_ = nullptr;  ///< FreeRTOS semaphore for WaitForInterrupt()
};