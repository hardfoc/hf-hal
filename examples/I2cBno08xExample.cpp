/**
 * @file I2cBno08xExample.cpp
 * @brief Example demonstrating I2C communication with BNO08x IMU using the new architecture.
 *
 * This example shows how to:
 * - Initialize the I2C bus for BNO08x and PCAL95555 devices
 * - Access the BNO08x IMU directly (no BaseImuDriver abstraction)
 * - Configure and read sensor data from the BNO08x
 * - Use the ESP-IDF v5.5+ I2C bus-device model
 *
 * Hardware Requirements:
 * - ESP32C6 development board
 * - BNO08x IMU connected via I2C (address 0x4A)
 * - Optional: PCAL95555 GPIO expander (address 0x20)
 * - I2C pull-up resistors (4.7kΩ recommended)
 *
 * Pin Configuration (see hf_functional_pin_config.hpp):
 * - I2C_SDA: GPIO pin as defined in board mapping
 * - I2C_SCL: GPIO pin as defined in board mapping
 *
 * @author HardFOC Team
 * @date 2025
 */

#include <iostream>
#include <chrono>
#include <thread>

// Component managers
#include "../component-handers/CommChannelsManager.h"
#include "../component-handers/ImuManager.h"

// BNO08x driver
#include "../utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085.hpp"

// ESP-IDF includes for logging
#ifdef ESP_PLATFORM
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}
static const char* TAG = "I2cBno08xExample";
#define EXAMPLE_LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define EXAMPLE_LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define EXAMPLE_LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#else
#define EXAMPLE_LOGI(format, ...) printf("[INFO] I2cBno08xExample: " format "\n", ##__VA_ARGS__)
#define EXAMPLE_LOGW(format, ...) printf("[WARN] I2cBno08xExample: " format "\n", ##__VA_ARGS__)
#define EXAMPLE_LOGE(format, ...) printf("[ERROR] I2cBno08xExample: " format "\n", ##__VA_ARGS__)
#define DELAY_MS(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))
#endif

/**
 * @brief Sensor event callback for BNO08x data processing.
 * @param event Sensor event containing new data
 */
void sensorEventCallback(const BNO085::SensorEvent& event) {
    switch (event.sensor) {
        case BNO085Sensor::RotationVector:
            EXAMPLE_LOGI("Rotation Vector: i=%.3f, j=%.3f, k=%.3f, real=%.3f (accuracy=%d)",
                         event.data.rotation.i, event.data.rotation.j, 
                         event.data.rotation.k, event.data.rotation.real,
                         event.data.rotation.accuracy);
            break;
            
        case BNO085Sensor::LinearAcceleration:
            EXAMPLE_LOGI("Linear Acceleration: x=%.3f, y=%.3f, z=%.3f m/s² (accuracy=%d)",
                         event.data.acceleration.x, event.data.acceleration.y, 
                         event.data.acceleration.z, event.data.acceleration.accuracy);
            break;
            
        case BNO085Sensor::Gyroscope:
            EXAMPLE_LOGI("Gyroscope: x=%.3f, y=%.3f, z=%.3f rad/s (accuracy=%d)",
                         event.data.gyroscope.x, event.data.gyroscope.y, 
                         event.data.gyroscope.z, event.data.gyroscope.accuracy);
            break;
            
        case BNO085Sensor::Accelerometer:
            EXAMPLE_LOGI("Accelerometer: x=%.3f, y=%.3f, z=%.3f m/s² (accuracy=%d)",
                         event.data.acceleration.x, event.data.acceleration.y, 
                         event.data.acceleration.z, event.data.acceleration.accuracy);
            break;
            
        default:
            EXAMPLE_LOGI("Other sensor data received (sensor=%d)", static_cast<int>(event.sensor));
            break;
    }
}

/**
 * @brief Initialize and configure the BNO08x IMU.
 * @param bno08x Reference to the BNO085 device
 * @return true if configuration successful, false otherwise
 */
bool configureBno08x(BNO085& bno08x) {
    EXAMPLE_LOGI("Configuring BNO08x sensors...");
    
    // Set up callback for sensor events
    bno08x.setCallback(sensorEventCallback);
    
    // Enable rotation vector at 50Hz (20ms intervals)
    if (!bno08x.enableSensor(BNO085Sensor::RotationVector, 20)) {
        EXAMPLE_LOGE("Failed to enable rotation vector sensor");
        return false;
    }
    EXAMPLE_LOGI("Rotation vector sensor enabled (50Hz)");
    
    // Enable linear acceleration at 100Hz (10ms intervals)
    if (!bno08x.enableSensor(BNO085Sensor::LinearAcceleration, 10)) {
        EXAMPLE_LOGE("Failed to enable linear acceleration sensor");
        return false;
    }
    EXAMPLE_LOGI("Linear acceleration sensor enabled (100Hz)");
    
    // Enable gyroscope at 100Hz (10ms intervals)
    if (!bno08x.enableSensor(BNO085Sensor::Gyroscope, 10)) {
        EXAMPLE_LOGE("Failed to enable gyroscope sensor");
        return false;
    }
    EXAMPLE_LOGI("Gyroscope sensor enabled (100Hz)");
    
    // Enable accelerometer at 100Hz (10ms intervals)
    if (!bno08x.enableSensor(BNO085Sensor::Accelerometer, 10)) {
        EXAMPLE_LOGE("Failed to enable accelerometer sensor");
        return false;
    }
    EXAMPLE_LOGI("Accelerometer sensor enabled (100Hz)");
    
    EXAMPLE_LOGI("BNO08x sensor configuration complete");
    return true;
}

/**
 * @brief Demonstrate polling-based sensor data reading.
 * @param bno08x Reference to the BNO085 device
 */
void demonstratePollingMode(BNO085& bno08x) {
    EXAMPLE_LOGI("=== Polling Mode Demonstration ===");
    
    for (int i = 0; i < 20; ++i) {
        // Update the sensor (processes any incoming data)
        bno08x.update();
        
        // Check for rotation vector data
        if (bno08x.hasNewData(BNO085Sensor::RotationVector)) {
            auto event = bno08x.getLatest(BNO085Sensor::RotationVector);
            EXAMPLE_LOGI("Polled Rotation Vector: i=%.3f, j=%.3f, k=%.3f, real=%.3f",
                         event.data.rotation.i, event.data.rotation.j, 
                         event.data.rotation.k, event.data.rotation.real);
        }
        
        // Check for linear acceleration data
        if (bno08x.hasNewData(BNO085Sensor::LinearAcceleration)) {
            auto event = bno08x.getLatest(BNO085Sensor::LinearAcceleration);
            EXAMPLE_LOGI("Polled Linear Acceleration: x=%.3f, y=%.3f, z=%.3f m/s²",
                         event.data.acceleration.x, event.data.acceleration.y, 
                         event.data.acceleration.z);
        }
        
        DELAY_MS(100); // 100ms delay between polls
    }
}

/**
 * @brief Main example function.
 */
int main() {
    EXAMPLE_LOGI("=== I2C BNO08x IMU Example ===");
    EXAMPLE_LOGI("This example demonstrates ESP-IDF v5.5+ I2C communication with BNO08x IMU");
    
    try {
        // Initialize communication channels
        EXAMPLE_LOGI("Initializing communication channels...");
        auto& comm_mgr = CommChannelsManager::GetInstance();
        if (!comm_mgr.EnsureInitialized()) {
            EXAMPLE_LOGE("Failed to initialize CommChannelsManager");
            return -1;
        }
        EXAMPLE_LOGI("Communication channels initialized successfully");
        
        // Initialize IMU manager
        EXAMPLE_LOGI("Initializing IMU manager...");
        auto& imu_mgr = ImuManager::GetInstance();
        if (!imu_mgr.EnsureInitialized()) {
            EXAMPLE_LOGE("Failed to initialize ImuManager");
            return -1;
        }
        EXAMPLE_LOGI("IMU manager initialized successfully");
        
        // Check if BNO08x is available
        if (!imu_mgr.IsBno08xAvailable()) {
            EXAMPLE_LOGE("BNO08x IMU is not available");
            EXAMPLE_LOGE("Please check I2C connections and device address");
            return -1;
        }
        
        // Get direct access to BNO08x device
        EXAMPLE_LOGI("Getting BNO08x device reference...");
        BNO085& bno08x = imu_mgr.GetBno08x();
        EXAMPLE_LOGI("BNO08x device accessed successfully");
        
        // Configure sensors
        if (!configureBno08x(bno08x)) {
            EXAMPLE_LOGE("Failed to configure BNO08x sensors");
            return -1;
        }
        
        // Wait for sensor stabilization
        EXAMPLE_LOGI("Waiting for sensor stabilization...");
        DELAY_MS(1000);
        
        // Demonstrate callback-based operation
        EXAMPLE_LOGI("=== Callback Mode Demonstration ===");
        EXAMPLE_LOGI("Collecting sensor data for 10 seconds using callbacks...");
        
        for (int i = 0; i < 100; ++i) {
            bno08x.update(); // Process any incoming sensor data
            DELAY_MS(100);   // 100ms update interval
        }
        
        // Demonstrate polling-based operation
        demonstratePollingMode(bno08x);
        
        // Display device information
        EXAMPLE_LOGI("=== Device Information ===");
        auto available_devices = imu_mgr.GetAvailableDevices();
        EXAMPLE_LOGI("Available IMU devices: %zu", available_devices.size());
        for (const auto& device : available_devices) {
            EXAMPLE_LOGI("  - %s", device.c_str());
        }
        
        // I2C bus information
        EXAMPLE_LOGI("I2C buses available: %zu", comm_mgr.GetI2cCount());
        
        EXAMPLE_LOGI("=== Example completed successfully ===");
        
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("Exception in main: %s", e.what());
        return -1;
    }
    
    // Clean shutdown
    EXAMPLE_LOGI("Cleaning up...");
    ImuManager::GetInstance().Deinitialize();
    CommChannelsManager::GetInstance().EnsureDeinitialized();
    
    return 0;
}

#ifdef ESP_PLATFORM
/**
 * @brief ESP-IDF application entry point.
 */
extern "C" void app_main() {
    main();
}
#endif
