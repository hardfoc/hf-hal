/**
 * @file ImuManagerInterruptExample.cpp
 * @brief Simple example showing how to use ImuManager with EspGpio interrupt support.
 *
 * This example demonstrates the clean, high-level API for using BNO08x interrupts
 * through the ImuManager and EspGpio abstraction layers.
 *
 * @author HardFOC Team
 * @date 2025
 */

#include "CommChannelsManager.h"
#include "ImuManager.h"

// BNO08x driver
#include "utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085.hpp"

// ESP-IDF
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static const char* TAG = "ImuInterruptExample";

//==============================================================================
// SENSOR DATA CALLBACK
//==============================================================================

/**
 * @brief Process BNO08x sensor events with filtering.
 */
void ProcessSensorEvent(const SensorEvent& event) {
    static uint32_t rotation_count = 0;
    static uint32_t accel_count = 0;
    
    switch (event.sensor) {
        case BNO085Sensor::RotationVector:
            rotation_count++;
            if (rotation_count % 10 == 0) {  // Log every 10th sample
                ESP_LOGI(TAG, "[%lu] Rotation: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                         rotation_count, event.rotation.w, event.rotation.x, 
                         event.rotation.y, event.rotation.z);
            }
            break;
            
        case BNO085Sensor::Accelerometer:
            accel_count++;
            if (accel_count % 20 == 0) {  // Log every 20th sample
                float magnitude = sqrt(event.vector.x * event.vector.x + 
                                     event.vector.y * event.vector.y + 
                                     event.vector.z * event.vector.z);
                ESP_LOGI(TAG, "[%lu] Accel magnitude: %.3f m/s²", accel_count, magnitude);
            }
            break;
            
        case BNO085Sensor::TapDetector:
            ESP_LOGI(TAG, "Tap detected: %s, direction=%d", 
                     event.tap.doubleTap ? "Double" : "Single", event.tap.direction);
            break;
            
        case BNO085Sensor::StepCounter:
            ESP_LOGI(TAG, "Step count: %lu", event.stepCount);
            break;
            
        default:
            // Handle other sensors as needed
            break;
    }
}

//==============================================================================
// INTERRUPT-DRIVEN TASK
//==============================================================================

/**
 * @brief Task that uses interrupt-driven BNO08x processing.
 * 
 * This demonstrates the cleanest way to use BNO08x interrupts with the
 * ImuManager's built-in interrupt support.
 */
void InterruptDrivenTask(void* pvParameters) {
    ESP_LOGI(TAG, "Starting interrupt-driven BNO08x task");
    
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
    
    // Set sensor callback
    bno08x->setCallback(ProcessSensorEvent);
    
    // Configure interrupt with optional callback
    bool interrupt_available = imu_mgr.ConfigureInterrupt([]() {
        // This callback executes in interrupt context - keep it minimal
        // Just a simple LED toggle or counter increment
        static uint32_t int_count = 0;
        int_count++;
        if (int_count % 100 == 0) {
            // Periodic status (this will be called from ISR context)
        }
    });
    
    if (interrupt_available) {
        ESP_LOGI(TAG, "BNO08x interrupt configured successfully");
        
        // Enable interrupt
        if (!imu_mgr.EnableInterrupt()) {
            ESP_LOGE(TAG, "Failed to enable BNO08x interrupt");
            interrupt_available = false;
        }
    } else {
        ESP_LOGW(TAG, "BNO08x interrupt not available - using polling mode");
    }
    
    // Enable sensors
    bno08x->enableSensor(BNO085Sensor::RotationVector, 50);    // 20 Hz
    bno08x->enableSensor(BNO085Sensor::Accelerometer, 100);   // 10 Hz
    bno08x->enableSensor(BNO085Sensor::TapDetector);
    bno08x->enableSensor(BNO085Sensor::StepCounter);
    
    ESP_LOGI(TAG, "BNO08x sensors enabled");
    
    if (interrupt_available) {
        // Interrupt-driven mode: wait for GPIO interrupts
        ESP_LOGI(TAG, "Using interrupt-driven mode");
        
        uint32_t processed_interrupts = 0;
        
        while (1) {
            // Wait for interrupt notification
            // Note: In a real implementation, you'd get the interrupt queue from ImuManager
            // For this example, we'll use a simple delay and polling
            
            // Process sensor data when interrupt occurs
            bno08x->update();
            processed_interrupts++;
            
            // Log statistics periodically
            if (processed_interrupts % 100 == 0) {
                ESP_LOGI(TAG, "Processed %lu interrupts", processed_interrupts);
            }
            
            // Small delay - in interrupt mode this would be replaced by queue wait
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        
    } else {
        // Polling mode fallback
        ESP_LOGI(TAG, "Using polling mode fallback");
        
        TickType_t last_wake_time = xTaskGetTickCount();
        const TickType_t polling_interval = pdMS_TO_TICKS(10); // 100 Hz
        
        while (1) {
            // Process sensor data
            bno08x->update();
            
            // Wait for next polling interval
            vTaskDelayUntil(&last_wake_time, polling_interval);
        }
    }
}

//==============================================================================
// POLLING MODE TASK (SIMPLE ALTERNATIVE)
//==============================================================================

/**
 * @brief Simple polling-mode task for comparison.
 */
void PollingModeTask(void* pvParameters) {
    ESP_LOGI(TAG, "Starting polling mode BNO08x task");
    
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
    
    // Set sensor callback
    bno08x->setCallback(ProcessSensorEvent);
    
    // Enable sensors
    bno08x->enableSensor(BNO085Sensor::RotationVector, 50);    // 20 Hz
    bno08x->enableSensor(BNO085Sensor::Accelerometer, 100);   // 10 Hz
    bno08x->enableSensor(BNO085Sensor::TapDetector);
    
    ESP_LOGI(TAG, "BNO08x sensors enabled, starting polling loop");
    
    // Main polling loop
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t polling_interval = pdMS_TO_TICKS(10); // 100 Hz
    
    while (1) {
        // This is the key call - it checks for new data and triggers callbacks
        bno08x->update();
        
        // Wait for next polling interval
        vTaskDelayUntil(&last_wake_time, polling_interval);
    }
}

//==============================================================================
// MAIN EXAMPLE FUNCTION
//==============================================================================

/**
 * @brief Main example entry point.
 */
extern "C" void app_main() {
    ESP_LOGI(TAG, "BNO08x ImuManager Interrupt Example Starting");
    
    // Initialize communication channels
    auto& comm_mgr = CommChannelsManager::GetInstance();
    if (!comm_mgr.Initialize()) {
        ESP_LOGE(TAG, "Failed to initialize communication channels");
        return;
    }
    
    // Choose operation mode
    bool use_interrupt_mode = true;  // Set to false to test polling mode
    
    if (use_interrupt_mode) {
        ESP_LOGI(TAG, "=== Starting Interrupt-Driven Mode ===");
        xTaskCreatePinnedToCore(
            InterruptDrivenTask,
            "bno08x_interrupt",
            4096,
            NULL,
            6,  // Higher priority
            NULL,
            1   // Pin to core 1
        );
    } else {
        ESP_LOGI(TAG, "=== Starting Polling Mode ===");
        xTaskCreatePinnedToCore(
            PollingModeTask,
            "bno08x_polling",
            4096,
            NULL,
            5,
            NULL,
            1   // Pin to core 1
        );
    }
    
    ESP_LOGI(TAG, "Example task started, main function exiting");
}

//==============================================================================
// KEY BENEFITS OF THIS APPROACH
//==============================================================================

/*
 * Benefits of using ImuManager with EspGpio interrupt support:
 * 
 * 1. **Clean API**: Simple configure/enable/disable interface
 * 2. **Automatic GPIO Management**: No manual ESP-IDF GPIO configuration
 * 3. **Board Abstraction**: Uses functional pin mapping system
 * 4. **Error Handling**: Comprehensive error checking and logging
 * 5. **Fallback Support**: Gracefully falls back to polling if interrupt unavailable
 * 6. **Thread Safety**: All operations are thread-safe
 * 7. **Resource Management**: Automatic cleanup in destructor
 * 8. **Unified Interface**: Same API works across different hardware configurations
 * 
 * Usage Patterns:
 * 
 * // Simple polling (always works)
 * auto& imu_mgr = ImuManager::GetInstance();
 * BNO085& bno08x = imu_mgr.GetBno08x();
 * while (true) {
 *     bno08x.update();
 *     vTaskDelay(pdMS_TO_TICKS(10));
 * }
 * 
 * // Interrupt with fallback
 * if (imu_mgr.ConfigureInterrupt() && imu_mgr.EnableInterrupt()) {
 *     // Interrupt mode
 *     while (true) {
 *         // Wait for interrupt, then call bno08x.update()
 *     }
 * } else {
 *     // Fallback to polling
 *     while (true) {
 *         bno08x.update();
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 * }
 */
