/**
 * @file I2cBno08xCallbackExample.cpp
 * @brief Comprehensive example showing BNO08x callback mechanisms with EspGpio interrupt handling.
 *
 * This example demonstrates:
 * 1. Polling mode with manual update() calls
 * 2. Interrupt-driven mode using EspGpio for BNO08x INT pin
 * 3. Different callback patterns for sensor data processing
 * 4. Proper error handling and device initialization
 *
 * Hardware Setup:
 * - BNO08x connected to I2C bus (SDA: GPIO18, SCL: GPIO19)
 * - BNO08x INT pin connected via PCAL95555 (PCAL_IMU_INT) for interrupt mode
 * - BNO08x RST pin connected via PCAL95555 (PCAL_IMU_RST) for hardware reset
 *
 * @author HardFOC Team
 * @date 2025
 */

#include "CommChannelsManager.h"
#include "ImuManager.h"
#include "GpioManager.h"

// BNO08x driver
#include "utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085.hpp"

// ESP32 GPIO abstraction
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspGpio.h"

// Board pin mapping
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config.hpp"

// ESP-IDF
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
}

static const char* TAG = "BNO08xCallbackExample";

//==============================================================================
// GLOBAL VARIABLES FOR INTERRUPT HANDLING
//==============================================================================

static QueueHandle_t gpio_evt_queue = nullptr;
static volatile bool interrupt_mode = false;
static volatile uint32_t interrupt_count = 0;

// EspGpio instance for BNO08x interrupt pin
static std::unique_ptr<EspGpio> bno08x_int_gpio = nullptr;

//==============================================================================
// SENSOR DATA PROCESSING CALLBACKS
//==============================================================================

/**
 * @brief Simple sensor callback that logs all sensor events.
 * 
 * This callback is invoked by the BNO085 driver when new sensor data is available.
 * It executes in the same task context that calls bno08x.update().
 */
void SimpleSensorCallback(const SensorEvent& event) {
    ESP_LOGI(TAG, "Sensor Event: %d, Timestamp: %llu", 
             static_cast<int>(event.sensor), event.timestamp);
    
    switch (event.sensor) {
        case BNO085Sensor::RotationVector:
            ESP_LOGI(TAG, "Rotation: w=%.3f, x=%.3f, y=%.3f, z=%.3f, acc=%d",
                     event.rotation.w, event.rotation.x, event.rotation.y, 
                     event.rotation.z, event.rotation.accuracy);
            break;
            
        case BNO085Sensor::Accelerometer:
            ESP_LOGI(TAG, "Accel: x=%.3f, y=%.3f, z=%.3f m/s², acc=%d",
                     event.vector.x, event.vector.y, event.vector.z, event.vector.accuracy);
            break;
            
        case BNO085Sensor::Gyroscope:
            ESP_LOGI(TAG, "Gyro: x=%.3f, y=%.3f, z=%.3f rad/s, acc=%d",
                     event.vector.x, event.vector.y, event.vector.z, event.vector.accuracy);
            break;
            
        case BNO085Sensor::Magnetometer:
            ESP_LOGI(TAG, "Mag: x=%.3f, y=%.3f, z=%.3f µT, acc=%d",
                     event.vector.x, event.vector.y, event.vector.z, event.vector.accuracy);
            break;
            
        case BNO085Sensor::StepCounter:
            ESP_LOGI(TAG, "Steps: %lu", event.stepCount);
            break;
            
        case BNO085Sensor::TapDetector:
            ESP_LOGI(TAG, "Tap: %s tap, direction=%d", 
                     event.tap.doubleTap ? "Double" : "Single", event.tap.direction);
            break;
            
        default:
            ESP_LOGI(TAG, "Other sensor data received");
            break;
    }
}

/**
 * @brief Advanced sensor callback with data filtering and processing.
 */
void AdvancedSensorCallback(const SensorEvent& event) {
    static uint32_t rotation_count = 0;
    static uint32_t accel_count = 0;
    
    // Filter high-frequency data to reduce log spam
    switch (event.sensor) {
        case BNO085Sensor::RotationVector:
            rotation_count++;
            if (rotation_count % 10 == 0) { // Log every 10th sample
                ESP_LOGI(TAG, "[%lu] Rotation Vector: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                         rotation_count, event.rotation.w, event.rotation.x, 
                         event.rotation.y, event.rotation.z);
            }
            break;
            
        case BNO085Sensor::Accelerometer:
            accel_count++;
            if (accel_count % 20 == 0) { // Log every 20th sample
                float magnitude = sqrt(event.vector.x * event.vector.x + 
                                     event.vector.y * event.vector.y + 
                                     event.vector.z * event.vector.z);
                ESP_LOGI(TAG, "[%lu] Accel magnitude: %.3f m/s²", accel_count, magnitude);
            }
            break;
            
        case BNO085Sensor::TapDetector:
        case BNO085Sensor::StepCounter:
            // Always log discrete events
            SimpleSensorCallback(event);
            break;
            
        default:
            // Log other sensors normally
            break;
    }
}

//==============================================================================
// GPIO INTERRUPT HANDLING WITH ESPGPIO
//==============================================================================

/**
 * @brief BNO08x interrupt callback using EspGpio framework.
 * 
 * This callback is invoked by the EspGpio interrupt system when the BNO08x
 * asserts its interrupt pin, indicating new data is available.
 */
void Bno08xInterruptCallback(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
    // This executes in interrupt context - keep it minimal
    interrupt_count++;
    
    // Send notification to main task
    uint32_t pin_num = gpio->GetPinNumber();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &pin_num, &xHigherPriorityTaskWoken);
    
    // Yield to higher priority task if needed
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Configure BNO08x interrupt using EspGpio abstraction.
 */
esp_err_t ConfigureBno08xInterrupt() {
    ESP_LOGI(TAG, "Configuring BNO08x interrupt using EspGpio");
    
    // First, get the physical pin mapping for PCAL_IMU_INT
    // Note: The BNO08x INT signal is routed through the PCAL95555 GPIO expander
    // For this example, we'll assume it's mapped to a direct ESP32 GPIO
    // In practice, you may need to configure the PCAL95555 to route the signal
    
    auto* imu_int_mapping = GetGpioMapping(HfFunctionalGpioPin::PCAL_IMU_INT);
    if (!imu_int_mapping) {
        ESP_LOGE(TAG, "PCAL_IMU_INT pin mapping not found");
        return ESP_FAIL;
    }
    
    // Check if this is a direct ESP32 GPIO or PCAL95555 expander pin
    if (imu_int_mapping->chip_type != HfGpioChipType::ESP32_INTERNAL) {
        ESP_LOGW(TAG, "BNO08x INT pin is on PCAL95555 expander - advanced configuration needed");
        ESP_LOGW(TAG, "For this example, interrupt mode is disabled");
        return ESP_FAIL;
    }
    
    // Create EspGpio instance for interrupt pin
    bno08x_int_gpio = std::make_unique<EspGpio>(
        static_cast<hf_pin_num_t>(imu_int_mapping->physical_pin),
        hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW,  // BNO08x INT is active low
        hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_UP  // Enable pull-up
    );
    
    // Initialize the GPIO
    if (!bno08x_int_gpio->EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize BNO08x interrupt GPIO");
        return ESP_FAIL;
    }
    
    // Create queue for GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (!gpio_evt_queue) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return ESP_FAIL;
    }
    
    // Configure interrupt: falling edge (BNO08x INT is active low)
    auto result = bno08x_int_gpio->ConfigureInterrupt(
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTR_NEGEDGE,
        Bno08xInterruptCallback,
        nullptr  // No user data needed
    );
    
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGE(TAG, "Failed to configure BNO08x interrupt: %d", static_cast<int>(result));
        return ESP_FAIL;
    }
    
    // Enable the interrupt
    result = bno08x_int_gpio->EnableInterrupt();
    if (result != hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGE(TAG, "Failed to enable BNO08x interrupt: %d", static_cast<int>(result));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "BNO08x interrupt configured on GPIO %d using EspGpio", 
             imu_int_mapping->physical_pin);
    return ESP_OK;
}

//==============================================================================
// POLLING MODE EXAMPLE
//==============================================================================

/**
 * @brief Polling mode task - manually calls bno08x.update() to check for data.
 * 
 * In polling mode, you must call bno08x.update() regularly to:
 * 1. Check for new sensor data from the BNO08x
 * 2. Trigger any registered SensorCallback functions
 * 3. Service the internal SH-2 library state machine
 */
void PollingModeTask(void* pvParameters) {
    ESP_LOGI(TAG, "Starting BNO08x polling mode task");
    
    // Get IMU manager instance
    auto& imu_mgr = ImuManager::GetInstance();
    if (!imu_mgr.EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize IMU manager");
        vTaskDelete(NULL);
        return;
    }
    
    // Get BNO08x device
    BNO085* bno08x = imu_mgr.GetBno08xPtr();
    if (!bno08x) {
        ESP_LOGE(TAG, "BNO08x device not available");
        vTaskDelete(NULL);
        return;
    }
    
    // Set callback for sensor events
    bno08x->setCallback(SimpleSensorCallback);
    
    // Enable sensors
    if (!bno08x->enableSensor(BNO085Sensor::RotationVector, 50)) { // 20 Hz
        ESP_LOGE(TAG, "Failed to enable rotation vector sensor");
    }
    
    if (!bno08x->enableSensor(BNO085Sensor::Accelerometer, 100)) { // 10 Hz
        ESP_LOGE(TAG, "Failed to enable accelerometer sensor");
    }
    
    if (!bno08x->enableSensor(BNO085Sensor::TapDetector)) {
        ESP_LOGE(TAG, "Failed to enable tap detector");
    }
    
    ESP_LOGI(TAG, "BNO08x sensors enabled, starting polling loop");
    
    // Main polling loop
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t polling_interval = pdMS_TO_TICKS(10); // 100 Hz polling rate
    
    while (1) {
        // This is the key call - it checks for new data and triggers callbacks
        bno08x->update();
        
        // Wait for next polling interval
        vTaskDelayUntil(&last_wake_time, polling_interval);
    }
}

//==============================================================================
// INTERRUPT MODE EXAMPLE
//==============================================================================

/**
 * @brief Interrupt-driven task - waits for GPIO interrupts to process data.
 * 
 * In interrupt mode, the BNO08x INT pin triggers a GPIO interrupt when
 * new data is available, making the system more efficient.
 */
void InterruptModeTask(void* pvParameters) {
    ESP_LOGI(TAG, "Starting BNO08x interrupt mode task");
    
    // Configure GPIO interrupt
    if (ConfigureBno08xInterrupt() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure BNO08x interrupt");
        vTaskDelete(NULL);
        return;
    }
    
    // Get IMU manager instance
    auto& imu_mgr = ImuManager::GetInstance();
    if (!imu_mgr.EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize IMU manager");
        vTaskDelete(NULL);
        return;
    }
    
    // Get BNO08x device
    BNO085* bno08x = imu_mgr.GetBno08xPtr();
    if (!bno08x) {
        ESP_LOGE(TAG, "BNO08x device not available");
        vTaskDelete(NULL);
        return;
    }
    
    // Set callback for sensor events
    bno08x->setCallback(AdvancedSensorCallback);
    
    // Enable sensors
    bno08x->enableSensor(BNO085Sensor::RotationVector, 20);  // 50 Hz
    bno08x->enableSensor(BNO085Sensor::Accelerometer, 50);   // 20 Hz
    bno08x->enableSensor(BNO085Sensor::Gyroscope, 50);       // 20 Hz
    bno08x->enableSensor(BNO085Sensor::TapDetector);
    bno08x->enableSensor(BNO085Sensor::StepCounter);
    
    ESP_LOGI(TAG, "BNO08x sensors enabled, waiting for interrupts");
    interrupt_mode = true;
    
    uint32_t io_num;
    uint32_t processed_interrupts = 0;
    
    // Main interrupt handling loop
    while (1) {
        // Wait for GPIO interrupt event
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            processed_interrupts++;
            
            // Process sensor data when interrupt occurs
            bno08x->update();
            
            // Log interrupt statistics periodically
            if (processed_interrupts % 100 == 0) {
                ESP_LOGI(TAG, "Processed %lu interrupts (total: %lu)", 
                         processed_interrupts, interrupt_count);
            }
        }
    }
}

//==============================================================================
// MAIN EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Run polling mode example.
 */
void RunPollingExample() {
    ESP_LOGI(TAG, "=== BNO08x Polling Mode Example ===");
    
    xTaskCreatePinnedToCore(
        PollingModeTask,
        "bno08x_polling",
        4096,
        NULL,
        5,
        NULL,
        1  // Pin to core 1
    );
}

/**
 * @brief Run interrupt mode example.
 */
void RunInterruptExample() {
    ESP_LOGI(TAG, "=== BNO08x Interrupt Mode Example ===");
    
    xTaskCreatePinnedToCore(
        InterruptModeTask,
        "bno08x_interrupt",
        4096,
        NULL,
        6,  // Higher priority than polling
        NULL,
        1   // Pin to core 1
    );
}

/**
 * @brief Main example entry point.
 */
extern "C" void app_main() {
    ESP_LOGI(TAG, "BNO08x Callback Example Starting");
    
    // Initialize communication channels
    auto& comm_mgr = CommChannelsManager::GetInstance();
    if (!comm_mgr.Initialize()) {
        ESP_LOGE(TAG, "Failed to initialize communication channels");
        return;
    }
    
    // Choose operation mode
    bool use_interrupt_mode = false; // Set to true to test interrupt mode
    
    if (use_interrupt_mode) {
        RunInterruptExample();
    } else {
        RunPollingExample();
    }
    
    ESP_LOGI(TAG, "Example tasks started, main function exiting");
}

//==============================================================================
// CALLBACK MECHANISM SUMMARY
//==============================================================================

/*
 * BNO08x Callback Mechanism Explained:
 * 
 * 1. **Polling Mode (Default)**:
 *    - Call bno08x.update() in your main loop (typically 50-100 Hz)
 *    - update() checks for new data via I2C
 *    - If new data is found, your SensorCallback is invoked immediately
 *    - Callback executes in the same task context as update()
 *    - Simple and reliable, but uses more CPU cycles
 * 
 * 2. **Interrupt Mode (Optional)**:
 *    - Connect BNO08x INT pin to ESP32 GPIO
 *    - Configure GPIO interrupt (falling edge for BNO08x)
 *    - In GPIO ISR, send message to main task
 *    - Main task calls bno08x.update() when interrupt occurs
 *    - More efficient, only processes data when available
 *    - Requires additional hardware connection
 * 
 * 3. **Callback Execution Context**:
 *    - SensorCallback is NOT an interrupt handler
 *    - It's a regular function called by update()
 *    - Safe to use ESP_LOG, allocate memory, etc.
 *    - Avoid blocking operations in callbacks
 * 
 * 4. **No Interrupt Pin? No Problem!**:
 *    - BNO08x works perfectly in polling mode
 *    - Just call update() regularly (10-50ms intervals)
 *    - The driver handles all I2C communication internally
 */
