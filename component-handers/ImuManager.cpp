#include "ImuManager.h"

// BNO08x driver includes
#include "utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085_Transport.hpp"

// Communication manager for I2C access
#include "CommChannelsManager.h"

// ESP32 I2C wrapper for integration
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h"

// ESP32 GPIO abstraction for interrupt handling
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspGpio.h"

// Board pin mapping
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config.hpp"

// Standard library
#include <iostream>

// ESP-IDF logging and timing
extern "C" {
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
}
static const char* TAG = "ImuManager";
#define IMU_LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define IMU_LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define IMU_LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

/**
 * @class EspI2cBno085Transport
 * @brief I2C transport implementation for BNO085 using EspI2c wrapper.
 * 
 * This transport bridges the BNO085 driver's IBNO085Transport interface
 * with the HardFOC ESP-IDF v5.5+ I2C implementation (EspI2c).
 * 
 * **BNO08x Operation Modes:**
 * - **Polling Mode**: Call bno08x.update() in your main loop to check for new data
 * - **Interrupt Mode**: Connect BNO08x INT pin to ESP32 GPIO and use GPIO interrupts
 * 
 * **Callback Mechanism:**
 * - SensorCallback is invoked by the BNO085 driver when bno08x.update() finds new sensor data
 * - The callback executes in the same task context that calls update() (not interrupt context)
 * - For interrupt-driven operation, connect PCAL_IMU_INT pin and use GPIO interrupt handler
 */
class EspI2cBno085Transport : public IBNO085Transport {
public:
    /**
     * @brief Constructor with I2C bus reference and device address.
     * @param i2c_bus Reference to the EspI2c bus instance
     * @param device_addr I2C device address (7-bit, typically 0x4A for BNO08x)
     */
    explicit EspI2cBno085Transport(EspI2c& i2c_bus, uint8_t device_addr = 0x4A) noexcept
        : i2c_bus_(i2c_bus), device_address_(device_addr) {}

    /**
     * @brief Opens the I2C transport (no-op since bus is managed by CommChannelsManager).
     * @return true always (bus is already initialized)
     */
    bool open() override {
        IMU_LOGI("Opening BNO085 I2C transport (address=0x%02X)", device_address_);
        return true; // Bus is already initialized by CommChannelsManager
    }

    /**
     * @brief Closes the I2C transport (no-op since bus is managed by CommChannelsManager).
     */
    void close() override {
        IMU_LOGI("Closing BNO085 I2C transport");
        // Bus remains open for other devices (PCAL95555, etc.)
    }

    /**
     * @brief Writes data to the BNO085 via I2C.
     * @param data Pointer to data buffer to write
     * @param length Number of bytes to write
     * @return Number of bytes written, or negative on error
     */
    int write(const uint8_t* data, uint32_t length) override {
        if (!data || length == 0) {
            return 0;
        }

        hf_i2c_err_t result = i2c_bus_.Write(device_address_, data, static_cast<uint16_t>(length));
        if (result == hf_i2c_err_t::I2C_SUCCESS) {
            return static_cast<int>(length);
        } else {
            IMU_LOGW("I2C write failed: error=%d, length=%lu", static_cast<int>(result), length);
            return -1;
        }
    }

    /**
     * @brief Reads data from the BNO085 via I2C.
     * @param data Pointer to buffer to receive data
     * @param length Number of bytes to read
     * @return Number of bytes read, or negative on error
     */
    int read(uint8_t* data, uint32_t length) override {
        if (!data || length == 0) {
            return 0;
        }

        hf_i2c_err_t result = i2c_bus_.Read(device_address_, data, static_cast<uint16_t>(length));
        if (result == hf_i2c_err_t::I2C_SUCCESS) {
            return static_cast<int>(length);
        } else {
            // For BNO08x, a failed read often means no data available (not an error)
            return 0; // Return 0 to indicate no data available
        }
    }

    /**
     * @brief Checks if data is available (always true for polling-based operation).
     * @return true always (we use polling mode)
     */
    bool dataAvailable() override {
        return true; // Use polling mode - always attempt to read
    }

    /**
     * @brief Delays execution for specified milliseconds.
     * @param ms Delay duration in milliseconds
     */
    void delay(uint32_t ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    /**
     * @brief Gets current time in microseconds (ESP32 high-resolution timer).
     * @return Current time in microseconds
     */
    uint32_t getTimeUs() override {
        return static_cast<uint32_t>(esp_timer_get_time());
    }

private:
    EspI2c& i2c_bus_;           ///< Reference to the I2C bus
    uint8_t device_address_;    ///< I2C device address
};

//==============================================================================
// IMUMAMAGER IMPLEMENTATION
//==============================================================================

ImuManager& ImuManager::GetInstance() noexcept {
    static ImuManager instance;
    return instance;
}

ImuManager::ImuManager() noexcept : interrupt_queue_(nullptr) {}

ImuManager::~ImuManager() noexcept {
    Deinitialize();
    
    // Clean up interrupt resources
    if (interrupt_queue_) {
        vQueueDelete(interrupt_queue_);
        interrupt_queue_ = nullptr;
    }
}

bool ImuManager::EnsureInitialized() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return true;
    }

    IMU_LOGI("Initializing IMU Manager with direct BNO085 device access");

    // Get reference to communication manager
    comm_manager_ = &CommChannelsManager::GetInstance();
    if (!comm_manager_->EnsureInitialized()) {
        IMU_LOGE("Failed to initialize CommChannelsManager");
        return false;
    }

    // Initialize BNO08x IMU
    bool bno08x_ok = InitializeBno08x();
    if (bno08x_ok) {
        IMU_LOGI("BNO08x IMU successfully initialized");
    } else {
        IMU_LOGW("BNO08x IMU initialization failed");
    }

    // Consider initialized if at least one device is available
    initialized_ = bno08x_ok;

    if (initialized_) {
        IMU_LOGI("IMU Manager initialized successfully (%zu devices available)", GetImuCount());
    } else {
        IMU_LOGE("IMU Manager initialization failed - no devices available");
    }

    return initialized_;
}

bool ImuManager::IsInitialized() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

bool ImuManager::Deinitialize() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return true;
    }

    IMU_LOGI("Deinitializing IMU Manager");

    bool all_ok = true;

    // Deinitialize BNO08x
    if (bno08x_device_) {
        // BNO085 doesn't have explicit deinitialize method
        bno08x_device_.reset();
        bno08x_transport_.reset();
        IMU_LOGI("BNO08x device deinitialized");
    }

    initialized_ = false;
    comm_manager_ = nullptr;

    IMU_LOGI("IMU Manager deinitialized");
    return all_ok;
}

BNO085& ImuManager::GetBno08x() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || !bno08x_device_) {
        // Return a static dummy object instead of throwing
        static BNO085 dummy_device(nullptr);
        IMU_LOGE("BNO08x device not available - returning dummy object");
        return dummy_device;
    }
    return *bno08x_device_;
}

BNO085* ImuManager::GetBno08xPtr() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return bno08x_device_.get();
}

bool ImuManager::IsBno08xAvailable() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ && bno08x_device_ != nullptr;
}

size_t ImuManager::GetImuCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    if (bno08x_device_) count++;
    return count;
}

std::vector<std::string> ImuManager::GetAvailableDevices() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> devices;
    if (bno08x_device_) {
        devices.emplace_back("BNO08x (I2C address 0x4A)");
    }
    return devices;
}

bool ImuManager::InitializeBno08x() noexcept {
    IMU_LOGI("Initializing BNO08x IMU with I2C transport");

    // Create I2C transport for BNO08x
    bno08x_transport_ = CreateBno08xTransport();
    if (!bno08x_transport_) {
        IMU_LOGE("Failed to create BNO08x I2C transport");
        return false;
    }

    // Create BNO085 device instance
    bno08x_device_ = std::make_unique<BNO085>();

    // Initialize the device with the transport
    if (!bno08x_device_->begin(bno08x_transport_.get())) {
        IMU_LOGE("Failed to initialize BNO085 device (SH-2 initialization failed)");
        int error_code = bno08x_device_->getLastError();
        IMU_LOGE("BNO085 last error code: %d", error_code);
        bno08x_device_.reset();
        bno08x_transport_.reset();
        return false;
    }

    IMU_LOGI("BNO08x IMU initialized successfully");
    IMU_LOGI("BNO085 device ready for sensor configuration and data collection");
    return true;
}

std::unique_ptr<IBNO085Transport> ImuManager::CreateBno08xTransport() noexcept {
    // Get I2C bus from communication manager
    if (comm_manager_->GetI2cCount() == 0) {
        IMU_LOGE("No I2C buses available in CommChannelsManager");
        return nullptr;
    }

    BaseI2c& base_i2c = comm_manager_->GetI2c(0);
    EspI2c* esp_i2c = dynamic_cast<EspI2c*>(&base_i2c);
    if (!esp_i2c) {
        IMU_LOGE("I2C bus is not an EspI2c instance");
        return nullptr;
    }

    // Create transport with standard BNO08x I2C address
    auto transport = std::make_unique<EspI2cBno085Transport>(*esp_i2c, 0x4A);

    IMU_LOGI("Created BNO08x I2C transport (address=0x4A)");
    return transport;
}

//==============================================================================
// INTERRUPT SUPPORT IMPLEMENTATION
//==============================================================================

/**
 * @brief Internal interrupt callback for BNO08x GPIO.
 */
void Bno08xGpioInterruptCallback(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
    ImuManager* imu_manager = static_cast<ImuManager*>(user_data);
    if (!imu_manager || !imu_manager->interrupt_queue_) {
        return;
    }
    
    // Send notification to main task
    uint32_t pin_num = gpio->GetPinNumber();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(imu_manager->interrupt_queue_, &pin_num, &xHigherPriorityTaskWoken);
    
    // Call user callback if provided
    if (imu_manager->interrupt_callback_) {
        // Note: This executes in interrupt context - user callback should be fast
        imu_manager->interrupt_callback_();
    }
    
    // Yield to higher priority task if needed
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

bool ImuManager::ConfigureInterrupt(std::function<void()> callback, void* user_data) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (interrupt_configured_) {
        IMU_LOGW("BNO08x interrupt already configured");
        return true;
    }
    
    // Store user callback
    interrupt_callback_ = callback;
    
    // Get GPIO mapping for BNO08x interrupt pin
    auto* imu_int_mapping = GetGpioMapping(HfFunctionalGpioPin::PCAL_IMU_INT);
    if (!imu_int_mapping) {
        IMU_LOGE("PCAL_IMU_INT pin mapping not found in board configuration");
        return false;
    }
    
    // Check if this is a direct ESP32 GPIO
    if (imu_int_mapping->chip_type != HfGpioChipType::ESP32_INTERNAL) {
        IMU_LOGW("BNO08x INT pin is on PCAL95555 expander - advanced configuration needed");
        IMU_LOGW("Direct GPIO interrupt not available, use polling mode");
        return false;
    }

    // Create EspGpio instance for interrupt pin
    bno08x_int_gpio_ = std::make_unique<EspGpio>(
        static_cast<hf_pin_num_t>(imu_int_mapping->physical_pin),
        hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW,  // BNO08x INT is active low
        hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_UP  // Enable pull-up
    );
    
    // Initialize the GPIO
    if (!bno08x_int_gpio_->EnsureInitialized()) {
        IMU_LOGE("Failed to initialize BNO08x interrupt GPIO");
        return false;
    }
    
    // Create queue for interrupt events
    interrupt_queue_ = xQueueCreate(10, sizeof(uint32_t));
    if (!interrupt_queue_) {
        IMU_LOGE("Failed to create interrupt event queue");
        return false;
    }
    
    // Configure interrupt: falling edge (BNO08x INT is active low)
    auto result = bno08x_int_gpio_->ConfigureInterrupt(
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTR_NEGEDGE,
        Bno08xGpioInterruptCallback,
        this  // Pass ImuManager instance as user data
    );
    
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        IMU_LOGE("Failed to configure BNO08x interrupt: %d", static_cast<int>(result));
        return false;
    }
    
    interrupt_configured_ = true;
    IMU_LOGI("BNO08x interrupt configured on GPIO %d", imu_int_mapping->physical_pin);
    return true;
}

bool ImuManager::EnableInterrupt() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!interrupt_configured_ || !bno08x_int_gpio_) {
        IMU_LOGE("BNO08x interrupt not configured");
        return false;
    }
    
    auto result = bno08x_int_gpio_->EnableInterrupt();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        IMU_LOGE("Failed to enable BNO08x interrupt: %d", static_cast<int>(result));
        return false;
    }
    
    IMU_LOGI("BNO08x interrupt enabled");
    return true;
}

bool ImuManager::DisableInterrupt() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!interrupt_configured_ || !bno08x_int_gpio_) {
        IMU_LOGW("BNO08x interrupt not configured");
        return true;  // Not an error if already disabled
    }
    
    auto result = bno08x_int_gpio_->DisableInterrupt();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        IMU_LOGE("Failed to disable BNO08x interrupt: %d", static_cast<int>(result));
        return false;
    }
    
    IMU_LOGI("BNO08x interrupt disabled");
    return true;
}

bool ImuManager::IsInterruptEnabled() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!interrupt_configured_ || !bno08x_int_gpio_) {
        return false;
    }
    
    // Check interrupt status through GPIO
    InterruptStatus status;
    auto result = bno08x_int_gpio_->GetInterruptStatus(status);
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        return false;
    }
    
    return status.is_enabled;
} 