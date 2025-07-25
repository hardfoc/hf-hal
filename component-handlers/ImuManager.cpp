#include "ImuManager.h"

// Bno08x handler for unified IMU interface
#include "utils-and-drivers/driver-handlers/Bno08xHandler.h"

// Communication manager for I2C access
#include "CommChannelsManager.h"

// GPIO manager for interrupt pin access
#include "GpioManager.h"

// Base interfaces
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseI2c.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseGpio.h"

// Platform mapping for functional pin definitions
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"

// RtosMutex for thread safety
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"

// Standard library
#include <iostream>

// ESP-IDF for semaphores and logging
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}
static const char* TAG = "ImuManager";
#define IMU_LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define IMU_LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define IMU_LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)

//==============================================================================
// IMUMANAGER IMPLEMENTATION
//==============================================================================

ImuManager& ImuManager::GetInstance() noexcept {
    static ImuManager instance;
    return instance;
}

ImuManager::ImuManager() noexcept {}

ImuManager::~ImuManager() noexcept {
    Deinitialize();
}

bool ImuManager::Initialize() noexcept {
    MutexLockGuard lock(manager_mutex_);
    if (initialized_) {
        return true;
    }

    IMU_LOGI("Initializing IMU Manager with Bno08xHandler");

    // Get reference to communication manager
    comm_manager_ = &CommChannelsManager::GetInstance();
    if (!comm_manager_->EnsureInitialized()) {
        IMU_LOGE("Failed to initialize CommChannelsManager");
        return false;
    }

    // Get reference to GPIO manager for interrupt support
    gpio_manager_ = &GpioManager::GetInstance();
    if (!gpio_manager_->IsInitialized()) {
        // Try to initialize GPIO manager
        auto init_result = gpio_manager_->EnsureInitialized();
        if (!init_result.IsSuccess()) {
            IMU_LOGW("GPIO Manager initialization failed - interrupt support will be limited");
        }
    }

    // Initialize BNO08x IMU handler
    bool bno08x_ok = InitializeBno08xHandler();
    if (bno08x_ok) {
        IMU_LOGI("BNO08x IMU handler successfully initialized");
    } else {
        IMU_LOGW("BNO08x IMU handler initialization failed");
    }

    // Initialize interrupt GPIO (optional - don't fail if not available)
    if (gpio_manager_ && gpio_manager_->IsInitialized()) {
        bool gpio_ok = InitializeInterruptGpio();
        if (gpio_ok) {
            IMU_LOGI("BNO08x interrupt GPIO initialized successfully");
        } else {
            IMU_LOGW("BNO08x interrupt GPIO not available - using polling mode only");
        }
    }

    // Consider initialized if at least one device is available
    initialized_ = bno08x_ok;

    if (initialized_) {
        IMU_LOGI("IMU Manager initialized successfully (%u devices available)", GetImuCount());
    } else {
        IMU_LOGE("IMU Manager initialization failed - no devices available");
    }

    return initialized_;
}

bool ImuManager::IsInitialized() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    return initialized_;
}

bool ImuManager::Deinitialize() noexcept {
    MutexLockGuard lock(manager_mutex_);
    if (!initialized_) {
        return true;
    }

    IMU_LOGI("Deinitializing IMU Manager");

    // Disable interrupt first
    if (interrupt_enabled_) {
        DisableInterrupt();
    }

    // Clean up interrupt semaphore
    if (interrupt_semaphore_) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(interrupt_semaphore_));
        interrupt_semaphore_ = nullptr;
    }

    // Reset interrupt state
    interrupt_gpio_ = nullptr;
    interrupt_gpio_shared_.reset();  // Release shared ownership
    interrupt_configured_ = false;
    interrupt_enabled_ = false;
    interrupt_count_.store(0);
    interrupt_callback_ = nullptr;

    // Deinitialize BNO08x handler
    if (bno08x_handler_) {
        bno08x_handler_.reset();
        IMU_LOGI("BNO08x handler deinitialized");
    }

    initialized_ = false;
    comm_manager_ = nullptr;

    IMU_LOGI("IMU Manager deinitialized");
    return true;
}

Bno08xHandler* ImuManager::GetBno08xHandler() noexcept {
    MutexLockGuard lock(manager_mutex_);
    if (!initialized_ || !bno08x_handler_) {
        return nullptr;
    }
    return bno08x_handler_.get();
}
}

bool ImuManager::IsBno08xAvailable() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    return initialized_ && bno08x_handler_ != nullptr;
}

uint8_t ImuManager::GetImuCount() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    uint8_t count = 0;
    if (bno08x_handler_) count++;
    return count;
}

std::vector<std::string> ImuManager::GetAvailableDevices() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    std::vector<std::string> devices;
    if (bno08x_handler_) {
        devices.emplace_back("BNO08x (I2C Handler)");
    }
    return devices;
}

bool ImuManager::InitializeBno08xHandler() noexcept {
    IMU_LOGI("Initializing BNO08x IMU handler with I2C transport");

    // Get I2C device from communication manager for BNO08x
    BaseI2c* imu_device = comm_manager_->GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!imu_device) {
        IMU_LOGE("BNO08x IMU device not available in CommChannelsManager");
        return false;
    }

    // Create Bno08x handler with I2C interface (no exceptions)
    // Create default configuration for BNO08x
    Bno08xConfig config = {};  // Use default configuration
    
    // Create the handler with I2C interface (no GPIO pins for now)
    bno08x_handler_ = std::make_unique<Bno08xHandler>(*imu_device, config);
    
    // Initialize the handler
    Bno08xError init_result = bno08x_handler_->Initialize();
    if (init_result != Bno08xError::SUCCESS) {
        IMU_LOGE("Failed to initialize Bno08xHandler: %s", Bno08xErrorToString(init_result));
        bno08x_handler_.reset();
        return false;
    }
    
    IMU_LOGI("BNO08x IMU handler initialized successfully");
    return true;
}

//==============================================================================
// GPIO INTERRUPT SUPPORT IMPLEMENTATION
//==============================================================================

bool ImuManager::InitializeInterruptGpio() noexcept {
    if (!gpio_manager_) {
        IMU_LOGW("GPIO Manager not available for interrupt setup");
        return false;
    }

    // Check if PCAL_IMU_INT functional pin is available
    if (!gpio_manager_->IsPinAvailable(static_cast<HardFOC::FunctionalGpioPin>(HfFunctionalGpioPin::PCAL_IMU_INT))) {
        IMU_LOGW("PCAL_IMU_INT functional pin not available on this platform");
        return false;
    }

    // Get shared GPIO pin for PCAL_IMU_INT (creates if needed)
    // Configure as input with pull-up since BNO08x INT is active low
    interrupt_gpio_shared_ = gpio_manager_->CreateSharedPin(
        HfFunctionalGpioPin::PCAL_IMU_INT,
        hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW,  // BNO08x INT is active low
        hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP    // Pull-up for active low signal
    );
    
    if (!interrupt_gpio_shared_) {
        IMU_LOGE("Failed to create shared PCAL_IMU_INT pin");
        return false;
    }

    // Store raw pointer for compatibility with existing callback code
    interrupt_gpio_ = interrupt_gpio_shared_.get();

    // Create semaphore for WaitForInterrupt()
    interrupt_semaphore_ = xSemaphoreCreateBinary();
    if (!interrupt_semaphore_) {
        IMU_LOGE("Failed to create interrupt semaphore");
        interrupt_gpio_ = nullptr;
        interrupt_gpio_shared_.reset();
        return false;
    }

    IMU_LOGI("BNO08x interrupt GPIO (PCAL_IMU_INT) initialized successfully with shared pin access");
    return true;
}

bool ImuManager::ConfigureInterrupt(std::function<void()> callback) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!initialized_) {
        IMU_LOGE("ImuManager not initialized");
        return false;
    }

    if (!interrupt_gpio_) {
        IMU_LOGE("Interrupt GPIO not available");
        return false;
    }

    if (interrupt_configured_) {
        IMU_LOGW("Interrupt already configured - reconfiguring");
        // Disable first if enabled
        if (interrupt_enabled_) {
            DisableInterrupt();
        }
    }

    // Store user callback
    interrupt_callback_ = callback;

    // Configure GPIO interrupt (falling edge for active-low BNO08x INT)
    auto configure_result = interrupt_gpio_->ConfigureInterrupt(
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTR_FALLING_EDGE,
        GpioInterruptHandler,
        this  // Pass ImuManager instance as user data
    );

    if (configure_result != hf_gpio_err_t::GPIO_SUCCESS) {
        IMU_LOGE("Failed to configure GPIO interrupt: %d", static_cast<int>(configure_result));
        interrupt_callback_ = nullptr;
        return false;
    }

    interrupt_configured_ = true;
    IMU_LOGI("BNO08x interrupt configured successfully");
    return true;
}

bool ImuManager::EnableInterrupt() noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!interrupt_configured_) {
        IMU_LOGE("Interrupt not configured - call ConfigureInterrupt() first");
        return false;
    }

    if (interrupt_enabled_) {
        IMU_LOGW("Interrupt already enabled");
        return true;
    }

    auto enable_result = interrupt_gpio_->EnableInterrupt();
    if (enable_result != hf_gpio_err_t::GPIO_SUCCESS) {
        IMU_LOGE("Failed to enable GPIO interrupt: %d", static_cast<int>(enable_result));
        return false;
    }

    interrupt_enabled_ = true;
    IMU_LOGI("BNO08x interrupt enabled successfully");
    return true;
}

bool ImuManager::DisableInterrupt() noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!interrupt_enabled_) {
        return true;  // Already disabled
    }

    if (!interrupt_gpio_) {
        IMU_LOGE("Interrupt GPIO not available");
        return false;
    }

    auto disable_result = interrupt_gpio_->DisableInterrupt();
    if (disable_result != hf_gpio_err_t::GPIO_SUCCESS) {
        IMU_LOGE("Failed to disable GPIO interrupt: %d", static_cast<int>(disable_result));
        return false;
    }

    interrupt_enabled_ = false;
    IMU_LOGI("BNO08x interrupt disabled successfully");
    return true;
}

bool ImuManager::IsInterruptEnabled() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    return interrupt_enabled_;
}

bool ImuManager::WaitForInterrupt(uint32_t timeout_ms) noexcept {
    if (!interrupt_semaphore_) {
        IMU_LOGE("Interrupt semaphore not available");
        return false;
    }

    if (!interrupt_enabled_) {
        IMU_LOGE("Interrupt not enabled");
        return false;
    }

    TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    BaseType_t result = xSemaphoreTake(static_cast<SemaphoreHandle_t>(interrupt_semaphore_), timeout_ticks);
    
    return (result == pdTRUE);
}

uint32_t ImuManager::GetInterruptCount() const noexcept {
    return interrupt_count_.load();
}

void ImuManager::GpioInterruptHandler(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept {
    // This function executes in interrupt context - keep it minimal!
    auto* imu_manager = static_cast<ImuManager*>(user_data);
    if (!imu_manager) {
        return;
    }

    // Increment interrupt counter
    imu_manager->interrupt_count_.fetch_add(1);

    // Signal semaphore for WaitForInterrupt()
    if (imu_manager->interrupt_semaphore_) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(
            static_cast<SemaphoreHandle_t>(imu_manager->interrupt_semaphore_), 
            &higher_priority_task_woken
        );
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    // Call user callback if provided
    if (imu_manager->interrupt_callback_) {
        imu_manager->interrupt_callback_();
    }
} 