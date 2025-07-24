#pragma once

#include <memory>
#include <vector>
#include <functional>
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"

// Forward declarations
class Bno08xHandler;
class CommChannelsManager;
class GpioManager;
class BaseGpio;

/**
 * @class ImuManager
 * @brief Singleton for managing IMUs on the board using Bno08xHandler abstraction.
 *
 * This manager provides access to BNO08x IMU devices through the Bno08xHandler
 * which offers a unified, exception-free interface with proper RTOS synchronization.
 * It handles initialization of I2C transport through CommChannelsManager and 
 * provides type-safe access to IMU devices on the board.
 *
 * **Key Features:**
 * - Bno08xHandler-based abstraction (consistent with MotorController/Tmc9660Handler pattern)
 * - Automatic I2C transport setup using CommChannelsManager
 * - Thread-safe singleton pattern with RtosMutex
 * - Exception-free operation with pointer-based returns
 * - Proper device initialization and lifecycle management
 * - ESP-IDF v5.5+ I2C integration
 *
 * **BNO08x Callback Operation:**
 * - **Polling Mode**: Call handler->update() regularly (50-100Hz) to trigger callbacks
 * - **Interrupt Mode**: Connect INT pin to GPIO, call update() when interrupt occurs
 * - **Callback Context**: SensorCallback executes in the task that calls update()
 * - **No INT Pin Required**: Polling mode works perfectly without interrupt connection
 *
 * **Usage Example:**
 * @code
 * auto& imu_mgr = ImuManager::GetInstance();
 * if (imu_mgr.Initialize()) {
 *     Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
 *     if (handler) {
 *         // Configure sensors through handler
 *         if (handler->EnableSensor(Bno08xSensorType::ROTATION_VECTOR, 50)) {
 *             // In your main loop or task:
 *             while (true) {
 *                 handler->Update();  // This triggers callbacks when new data arrives
 *                 vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz polling rate
 *             }
 *         }
 *     }
 * }
 * @endcode
 *
 * @note This manager now uses Bno08xHandler for consistent architecture across
 *       all device handlers in the HardFOC system.
 */
class ImuManager {
public:
    /**
     * @brief Get the singleton instance of ImuManager.
     * @return Reference to the singleton ImuManager.
     */
    static ImuManager& GetInstance() noexcept;

    /**
     * @brief Initialize the IMU manager system.
     * @note This automatically creates the BNO08x IMU handler using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Check if the IMU manager is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Deinitialize all IMUs and release resources.
     * @return true if deinitialized successfully, false otherwise
     */
    bool Deinitialize() noexcept;

    //==================== Handler Access ====================//

    /**
     * @brief Get access to the BNO08x IMU handler.
     * @return Pointer to Bno08xHandler if available and initialized, nullptr otherwise
     * @note Returns nullptr if not initialized or device not available
     * @note This provides access to the unified BNO08x handler interface
     */
    Bno08xHandler* GetBno08xHandler() noexcept;

    /**
     * @brief Check if the BNO08x IMU handler is available and initialized.
     * @return true if BNO08x handler is available, false otherwise
     */
    bool IsBno08xAvailable() const noexcept;

    //==================== Device Information ====================//

    /**
     * @brief Get the number of initialized IMUs.
     * @return Number of successfully initialized IMU devices
     */
    uint8_t GetImuCount() const noexcept;

    /**
     * @brief Get information about available IMU devices.
     * @return Vector of device names/identifiers
     */
    std::vector<std::string> GetAvailableDevices() const noexcept;

    //==================== Interrupt Support ====================//

    /**
     * @brief Configure GPIO interrupt for BNO08x INT pin.
     * @param callback Optional callback function executed in interrupt context (keep minimal)
     * @return true if interrupt configuration successful, false otherwise
     * @note Uses PCAL_IMU_INT functional pin through GpioManager
     * @note Callback executes in ISR context - keep operations minimal
     */
    bool ConfigureInterrupt(std::function<void()> callback = nullptr) noexcept;

    /**
     * @brief Enable BNO08x GPIO interrupt.
     * @return true if interrupt enabled successfully, false otherwise
     * @note Must call ConfigureInterrupt() first
     */
    bool EnableInterrupt() noexcept;

    /**
     * @brief Disable BNO08x GPIO interrupt.
     * @return true if interrupt disabled successfully, false otherwise
     */
    bool DisableInterrupt() noexcept;

    /**
     * @brief Check if interrupt is configured and enabled.
     * @return true if interrupt is active, false otherwise
     */
    bool IsInterruptEnabled() const noexcept;

    /**
     * @brief Wait for interrupt signal with timeout.
     * @param timeout_ms Timeout in milliseconds (0 = wait indefinitely)
     * @return true if interrupt occurred, false on timeout
     * @note Useful for interrupt-driven processing in tasks
     */
    bool WaitForInterrupt(uint32_t timeout_ms = 0) noexcept;

    /**
     * @brief Get interrupt statistics for monitoring.
     * @return Number of interrupts processed since initialization
     */
    uint32_t GetInterruptCount() const noexcept;

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
     * @brief Initialize the BNO08x IMU handler with I2C transport.
     * @return true if initialization successful, false otherwise
     */
    bool InitializeBno08xHandler() noexcept;

    /**
     * @brief Initialize GPIO interrupt for BNO08x INT pin.
     * @return true if GPIO setup successful, false otherwise
     */
    bool InitializeInterruptGpio() noexcept;

    /**
     * @brief GPIO interrupt handler for BNO08x INT pin.
     * @param gpio Pointer to the GPIO that triggered interrupt
     * @param trigger Interrupt trigger type
     * @param user_data Pointer to user data (ImuManager instance)
     */
    static void GpioInterruptHandler(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept;

    // State tracking
    bool initialized_ = false;
    mutable RtosMutex manager_mutex_;

    // Device handler (unified interface, consistent with MotorController pattern)
    std::unique_ptr<Bno08xHandler> bno08x_handler_;

    // Reference to communication manager for I2C access
    CommChannelsManager* comm_manager_ = nullptr;

    // GPIO interrupt support
    GpioManager* gpio_manager_ = nullptr;
    BaseGpio* interrupt_gpio_ = nullptr;
    std::function<void()> interrupt_callback_;
    bool interrupt_configured_ = false;
    bool interrupt_enabled_ = false;
    std::atomic<uint32_t> interrupt_count_{0};
    void* interrupt_semaphore_ = nullptr;  // FreeRTOS semaphore for WaitForInterrupt()
}; 