#ifndef COMPONENT_HANDLER_ENCODER_CONTROLLER_H_
#define COMPONENT_HANDLER_ENCODER_CONTROLLER_H_

#include "utils-and-drivers/driver-handlers/As5047uHandler.h"
#include "CommChannelsManager.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include <memory>
#include <array>
#include <vector>

/**
 * @file EncoderController.h
 * @brief Singleton class for managing multiple AS5047U encoder handlers and their interfaces.
 *
 * This class provides a global singleton for managing AS5047U position encoders, including:
 * - Always creates and manages the onboard AS5047U device (device index 0)
 * - Allows dynamic creation/deletion of external AS5047U devices (indices 1-3)
 * - Array-based access to As5047uHandler instances by device index
 * - Access to individual AS5047U sensors by device index
 * - Thread-safe device registration and access
 * - Board-aware device management with predefined CS pin assignments
 */
class EncoderController {
public:
    static constexpr uint8_t MAX_ENCODER_DEVICES = 4;     ///< Maximum supported AS5047U devices
    static constexpr uint8_t ONBOARD_ENCODER_INDEX = 0;   ///< Onboard AS5047U device index (SPI2_CS_AS5047U)
    static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 1; ///< External device 1 index (EXT_GPIO_CS_1)  
    static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 2; ///< External device 2 index (EXT_GPIO_CS_2)
    static constexpr uint8_t EXTERNAL_DEVICE_3_INDEX = 3; ///< External device 3 index (EXT_GPIO_CS_3)

public:
    /**
     * @brief Get the singleton instance.
     */
    static EncoderController& GetInstance();

    //**************************************************************************//
    //**                  DEVICE MANAGEMENT METHODS                           **//
    //**************************************************************************//

    bool EnsureInitialized() noexcept {
        if (!initialized_) {
            initialized_ = Initialize();
        }
        return initialized_;
    }

    inline bool IsInitialized() const noexcept { return initialized_; }

    //**************************************************************************//
    //**                  HANDLER AND DRIVER MANAGEMENT                       **//
    //**************************************************************************//

    /**
     * @brief Get access to As5047uHandler by device index.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Pointer to As5047uHandler if valid and active, nullptr otherwise
     * @note Returns nullptr if deviceIndex is invalid, device not active, or not initialized
     */
    As5047uHandler* handler(uint8_t deviceIndex = ONBOARD_ENCODER_INDEX) noexcept;

    /**
     * @brief Get access to the underlying AS5047U sensor by device index.
     * @param deviceIndex Device index (0=onboard, 1-3=external) 
     * @return Shared pointer to AS5047U sensor, nullptr if invalid/not ready
     * @note Returns nullptr if deviceIndex is invalid, device not active/initialized, or sensor unavailable
     */
    std::shared_ptr<AS5047U> sensor(uint8_t deviceIndex = ONBOARD_ENCODER_INDEX) noexcept;

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create an external AS5047U device on specified CS line.
     * @param csDeviceIndex External device CS index (1, 2, or 3 only)
     * @param spiDeviceId SPI device ID for communication
     * @param config AS5047U configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     * @note Only EXTERNAL_DEVICE_*_INDEX (1-3) are allowed
     */
    bool CreateExternalDevice(uint8_t csDeviceIndex, 
                            SpiDeviceId spiDeviceId, 
                            const As5047uConfig& config = As5047uHandler::GetDefaultConfig());

    /**
     * @brief Delete an external AS5047U device.
     * @param csDeviceIndex External device CS index (1, 2, or 3 only)
     * @return true if device deleted successfully, false otherwise
     * @note Cannot delete onboard device (index 0). Only external devices can be deleted.
     */
    bool DeleteExternalDevice(uint8_t csDeviceIndex);

    /**
     * @brief Get the number of active AS5047U devices.
     * @return Number of active devices (1 to MAX_ENCODER_DEVICES)
     * @note Always includes onboard device, plus any active external devices
     */
    uint8_t GetDeviceCount() const noexcept;

    /**
     * @brief Get list of all active device indices.
     * @return Vector containing active device indices
     */
    std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;

    /**
     * @brief Check if a specific device index is active and ready.
     * @param deviceIndex Device index to check
     * @return true if device is active and initialized, false otherwise
     */
    bool IsDeviceActive(uint8_t deviceIndex) const noexcept;

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

    //**************************************************************************//
    //**                  MULTI-ENCODER OPERATIONS                            **//
    //**************************************************************************//

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
    //**                  UTILITY AND DIAGNOSTICS                             **//
    //**************************************************************************//

    /**
     * @brief Get detailed status information for all devices.
     * @return String containing comprehensive status report
     */
    std::string GetStatusReport() const noexcept;

    /**
     * @brief Dump comprehensive diagnostics for all active devices.
     */
    void DumpAllDiagnostics() const noexcept;

    /**
     * @brief Reset statistics for all active devices.
     */
    void ResetAllStatistics() noexcept;

    //**************************************************************************//
    //**                  LIFECYCLE MANAGEMENT                                **//
    //**************************************************************************//

    /**
     * @brief Reset all encoder devices to default state.
     * @return true if all devices reset successfully
     */
    bool ResetAllDevices() noexcept;

    /**
     * @brief Shutdown and cleanup all encoder devices.
     */
    void Shutdown() noexcept;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    EncoderController() = default;

    /**
     * @brief Private destructor.
     */
    ~EncoderController() = default;

    // Disable copy and move operations
    EncoderController(const EncoderController&) = delete;
    EncoderController& operator=(const EncoderController&) = delete;
    EncoderController(EncoderController&&) = delete;
    EncoderController& operator=(EncoderController&&) = delete;

    /**
     * @brief Initialize the EncoderController.
     * @note This automatically creates the onboard AS5047U device using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Validate device index range.
     * @param deviceIndex Device index to validate
     * @return true if valid, false otherwise
     */
    bool IsValidDeviceIndex(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Check if device index is for external device.
     * @param deviceIndex Device index to check
     * @return true if external device index, false if onboard
     */
    bool IsExternalDeviceIndex(uint8_t deviceIndex) const noexcept;

    // Member variables
    mutable RtosMutex controller_mutex_;  ///< Thread safety for controller operations
    std::array<std::unique_ptr<As5047uHandler>, MAX_ENCODER_DEVICES> encoderHandlers_;
    std::array<bool, MAX_ENCODER_DEVICES> deviceInitialized_;
    std::array<bool, MAX_ENCODER_DEVICES> deviceActive_;    ///< Track which devices are active
    bool initialized_{false};             ///< Controller initialization state
    uint8_t activeDeviceCount_{0};        ///< Number of currently active devices
};

#endif // COMPONENT_HANDLER_ENCODER_CONTROLLER_H_