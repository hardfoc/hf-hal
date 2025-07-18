#pragma once

#include <memory>
#include <mutex>
#include <vector>

// Forward declarations
class BNO085;
class IBNO085Transport;
class CommChannelsManager;
class EspGpio;

// FreeRTOS queue handle
typedef void* QueueHandle_t;

/**
 * @class ImuManager
 * @brief Singleton for managing IMUs on the board with direct device object access.
 *
 * This manager provides direct access to BNO085 IMU objects rather than using
 * base class abstractions. It handles initialization of I2C transport and 
 * provides type-safe access to specific IMU devices on the board.
 *
 * **Key Features:**
 * - Direct BNO085 object access (no BaseImuDriver abstraction)
 * - Automatic I2C transport setup using CommChannelsManager
 * - Thread-safe singleton pattern
 * - Proper device initialization and lifecycle management
 * - ESP-IDF v5.5+ I2C integration
 *
 * **BNO08x Callback Operation:**
 * - **Polling Mode**: Call bno08x.update() regularly (50-100Hz) to trigger callbacks
 * - **Interrupt Mode**: Connect INT pin to GPIO, call update() when interrupt occurs
 * - **Callback Context**: SensorCallback executes in the task that calls update()
 * - **No INT Pin Required**: Polling mode works perfectly without interrupt connection
 *
 * **Usage Example:**
 * @code
 * auto& imu_mgr = ImuManager::GetInstance();
 * if (imu_mgr.EnsureInitialized()) {
 *     BNO085& bno08x = imu_mgr.GetBno08x();
 *     
 *     // Set callback for sensor events
 *     bno08x.setCallback([](const SensorEvent& event) {
 *         ESP_LOGI("IMU", "Sensor: %d, Timestamp: %llu", (int)event.sensor, event.timestamp);
 *     });
 *     
 *     // Enable sensors
 *     if (bno08x.enableSensor(BNO085Sensor::RotationVector, 50)) {
 *         // In your main loop or task:
 *         while (true) {
 *             bno08x.update();  // This triggers callbacks when new data arrives
 *             vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz polling rate
 *         }
 *     }
 * }
 * @endcode
 *
 * @note This manager works directly with BNO085 objects as requested by the user.
 *       No BaseImuDriver abstraction is used.
 */
class ImuManager {
public:
    /**
     * @brief Get the singleton instance of ImuManager.
     * @return Reference to the singleton ImuManager.
     */
    static ImuManager& GetInstance() noexcept;

    /**
     * @brief Ensure all IMUs are initialized (lazy initialization).
     * @return true if all IMUs are initialized successfully, false otherwise.
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Check if the IMU manager is initialized.
     * @return true if initialized, false otherwise.
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Deinitialize all IMUs and release resources.
     * @return true if deinitialized successfully, false otherwise.
     */
    bool Deinitialize() noexcept;

    //==================== Direct Device Access ====================//

    /**
     * @brief Get direct reference to the BNO08x IMU device.
     * @return Reference to the BNO085 device object
     * @note Returns a dummy object if not initialized or device not available
     * @note This provides direct access to the BNO085 object for full control
     */
    BNO085& GetBno08x();

    /**
     * @brief Get direct pointer to the BNO08x IMU device (safe version).
     * @return Pointer to the BNO085 device object, or nullptr if not available
     * @note This is the safe version that returns nullptr if not available
     */
    BNO085* GetBno08xPtr() noexcept;

    /**
     * @brief Check if the BNO08x IMU is available and initialized.
     * @return true if BNO08x is available, false otherwise
     */
    bool IsBno08xAvailable() const noexcept;

    //==================== Device Information ====================//

    /**
     * @brief Get the number of initialized IMUs.
     * @return Number of successfully initialized IMU devices
     */
    size_t GetImuCount() const noexcept;

    /**
     * @brief Get information about available IMU devices.
     * @return Vector of device names/identifiers
     */
    std::vector<std::string> GetAvailableDevices() const noexcept;

    //==================== Interrupt Support ====================//

    /**
     * @brief Configure BNO08x interrupt pin for efficient data processing.
     * 
     * This method sets up the BNO08x interrupt pin using the EspGpio abstraction.
     * When configured, the interrupt pin will trigger when new sensor data is available,
     * allowing for more efficient processing compared to pure polling mode.
     * 
     * @param callback Optional callback to invoke when interrupt occurs
     * @param user_data Optional user data passed to callback
     * @return true if interrupt successfully configured, false otherwise
     * 
     * @note The interrupt pin mapping depends on board configuration
     * @note If interrupt pin is on PCAL95555 expander, additional setup may be required
     * @note You still need to call bno08x.update() when interrupt occurs
     */
    bool ConfigureInterrupt(std::function<void()> callback = nullptr, void* user_data = nullptr) noexcept;

    /**
     * @brief Enable BNO08x interrupt.
     * @return true if interrupt enabled successfully, false otherwise
     */
    bool EnableInterrupt() noexcept;

    /**
     * @brief Disable BNO08x interrupt.
     * @return true if interrupt disabled successfully, false otherwise
     */
    bool DisableInterrupt() noexcept;

    /**
     * @brief Check if BNO08x interrupt is configured and enabled.
     * @return true if interrupt is active, false otherwise
     */
    bool IsInterruptEnabled() const noexcept;

    // Delete copy/move constructors and assignment operators
    ImuManager(const ImuManager&) = delete;
    ImuManager& operator=(const ImuManager&) = delete;
    ImuManager(ImuManager&&) = delete;
    ImuManager& operator=(ImuManager&&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    ImuManager() noexcept;

    /**
     * @brief Private destructor.
     */
    ~ImuManager() noexcept;

    /**
     * @brief Initialize the BNO08x IMU with I2C transport.
     * @return true if initialization successful, false otherwise
     */
    bool InitializeBno08x() noexcept;

    /**
     * @brief Create I2C transport for BNO08x communication.
     * @return Unique pointer to transport, or nullptr on failure
     */
    std::unique_ptr<IBNO085Transport> CreateBno08xTransport() noexcept;

    /**
     * @brief Configure BNO08x interrupt GPIO pin.
     * @return true if successful, false otherwise
     */
    bool SetupInterruptGpio() noexcept;

    // State tracking
    bool initialized_ = false;
    mutable std::mutex mutex_;

    // Device objects (direct access, no base class abstraction)
    std::unique_ptr<BNO085> bno08x_device_;
    std::unique_ptr<IBNO085Transport> bno08x_transport_;

    // Interrupt handling
    std::unique_ptr<EspGpio> bno08x_int_gpio_;
    std::function<void()> interrupt_callback_;
    QueueHandle_t interrupt_queue_;
    bool interrupt_configured_ = false;

    // Reference to communication manager for I2C access
    CommChannelsManager* comm_manager_ = nullptr;
}; 