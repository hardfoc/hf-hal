/**
 * @file ImuInterruptTaskExample.cpp
 * @brief Example showing interrupt-driven BNO08x processing in FreeRTOS tasks.
 * 
 * This example demonstrates:
 * 1. Setting up ImuManager with GPIO interrupt support
 * 2. Creating FreeRTOS tasks for interrupt-driven sensor processing
 * 3. Using WaitForInterrupt() for efficient task synchronization
 * 4. Proper task creation and cleanup patterns
 * 
 * Hardware Requirements:
 * - BNO08x IMU connected to I2C bus
 * - BNO08x INT pin connected to PCAL_IMU_INT (PCAL95555 IO4)
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "ImuManager.h"
#include "GpioManager.h"
#include "utils-and-drivers/driver-handlers/Bno08xHandler.h"

// ESP-IDF for tasks and logging
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static const char* TAG = "ImuInterruptTask";

// Task handle for interrupt-driven IMU processing
static TaskHandle_t imu_task_handle = nullptr;
static bool task_running = false;

/**
 * @brief Task that processes BNO08x data when interrupt occurs
 */
void imu_interrupt_task(void* pvParameters) {
    ESP_LOGI(TAG, "IMU interrupt task started");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Get BNO08x handler
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    if (!handler) {
        ESP_LOGE(TAG, "BNO08x handler not available - terminating task");
        task_running = false;
        vTaskDelete(nullptr);
        return;
    }
    
    ESP_LOGI(TAG, "BNO08x handler ready, waiting for interrupts...");
    
    uint32_t last_interrupt_count = 0;
    uint32_t processed_count = 0;
    
    while (task_running) {
        // Wait for interrupt with 1 second timeout
        if (imu_mgr.WaitForInterrupt(1000)) {
            uint32_t current_count = imu_mgr.GetInterruptCount();
            
            if (current_count > last_interrupt_count) {
                ESP_LOGI(TAG, "Interrupt %lu: Processing BNO08x data", current_count);
                
                // Process sensor data (this would trigger callbacks)
                // In real application, the handler->Update() would read new data
                // and trigger any configured sensor callbacks
                
                // Example: handler->Update();
                
                processed_count++;
                last_interrupt_count = current_count;
                
                // Add some processing delay to simulate real work
                vTaskDelay(pdMS_TO_TICKS(10));
                
                ESP_LOGI(TAG, "Data processing complete (processed: %lu)", processed_count);
            }
        } else {
            // Timeout - no interrupt received
            ESP_LOGD(TAG, "No interrupt received in last second (total: %lu)", 
                     imu_mgr.GetInterruptCount());
        }
        
        // Small delay to prevent task from consuming too much CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGI(TAG, "IMU interrupt task terminated (processed %lu interrupts)", processed_count);
    vTaskDelete(nullptr);
}

/**
 * @brief Initialize and start interrupt-driven IMU processing
 */
bool startInterruptDrivenImu() {
    ESP_LOGI(TAG, "Starting interrupt-driven IMU processing");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Initialize ImuManager (this sets up GPIO interrupt automatically)
    if (!imu_mgr.Initialize()) {
        ESP_LOGE(TAG, "Failed to initialize ImuManager");
        return false;
    }
    
    // Configure interrupt with minimal callback
    bool interrupt_configured = imu_mgr.ConfigureInterrupt([]() {
        // Minimal ISR callback - just increment a counter or set a flag
        // The actual processing happens in the task via WaitForInterrupt()
    });
    
    if (!interrupt_configured) {
        ESP_LOGW(TAG, "GPIO interrupt not available - this example requires interrupt support");
        return false;
    }
    
    // Enable the interrupt
    if (!imu_mgr.EnableInterrupt()) {
        ESP_LOGE(TAG, "Failed to enable interrupt");
        return false;
    }
    
    ESP_LOGI(TAG, "GPIO interrupt configured and enabled");
    
    // Start the interrupt processing task
    task_running = true;
    BaseType_t result = xTaskCreate(
        imu_interrupt_task,     // Task function
        "imu_interrupt_task",   // Task name
        4096,                   // Stack size (bytes)
        nullptr,                // Task parameters
        5,                      // Task priority
        &imu_task_handle        // Task handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU interrupt task");
        imu_mgr.DisableInterrupt();
        task_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "IMU interrupt task created successfully");
    return true;
}

/**
 * @brief Stop interrupt-driven IMU processing
 */
void stopInterruptDrivenImu() {
    ESP_LOGI(TAG, "Stopping interrupt-driven IMU processing");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Signal task to stop
    task_running = false;
    
    // Wait for task to finish (with timeout)
    if (imu_task_handle) {
        uint32_t wait_count = 0;
        while (eTaskGetState(imu_task_handle) != eDeleted && wait_count < 50) {
            vTaskDelay(pdMS_TO_TICKS(100));
            wait_count++;
        }
        imu_task_handle = nullptr;
    }
    
    // Disable interrupt
    imu_mgr.DisableInterrupt();
    
    ESP_LOGI(TAG, "Interrupt-driven IMU processing stopped");
}

/**
 * @brief Main example function
 */
int main() {
    ESP_LOGI(TAG, "=== IMU Interrupt Task Example ===");
    
    // Start interrupt-driven processing
    if (startInterruptDrivenImu()) {
        ESP_LOGI(TAG, "✓ Interrupt-driven IMU started successfully");
        
        // Run for 30 seconds to demonstrate
        ESP_LOGI(TAG, "Running for 30 seconds...");
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        // Stop processing
        stopInterruptDrivenImu();
        ESP_LOGI(TAG, "✓ Interrupt-driven IMU stopped");
    } else {
        ESP_LOGE(TAG, "✗ Failed to start interrupt-driven IMU");
    }
    
    ESP_LOGI(TAG, "=== Example completed ===");
    return 0;
}

/**
 * @brief FreeRTOS app_main entry point for ESP32
 */
extern "C" void app_main() {
    main();
}
