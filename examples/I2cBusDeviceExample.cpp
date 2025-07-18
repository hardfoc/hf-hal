/**
 * @file I2cBusDeviceExample.cpp
 * @brief Example demonstrating the new I2C bus-device architecture with CommChannelsManager.
 *
 * This example shows how to:
 * - Use the new ESP-IDF v5.5+ I2C bus-device model
 * - Access I2C devices through CommChannelsManager using the new APIs
 * - Use both enumerated device access and direct index access
 * - Demonstrate the clean separation between bus and device
 *
 * Hardware Requirements:
 * - ESP32C6 development board
 * - BNO08x IMU connected via I2C (address 0x4A)
 * - PCAL95555 GPIO expander connected via I2C (address 0x20)
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

// ESP-IDF includes for logging
#ifdef ESP_PLATFORM
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}
static const char* TAG = "I2cBusDeviceExample";
#define EXAMPLE_LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define EXAMPLE_LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define EXAMPLE_LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#else
#define EXAMPLE_LOGI(format, ...) printf("[INFO] I2cBusDeviceExample: " format "\n", ##__VA_ARGS__)
#define EXAMPLE_LOGW(format, ...) printf("[WARN] I2cBusDeviceExample: " format "\n", ##__VA_ARGS__)
#define EXAMPLE_LOGE(format, ...) printf("[ERROR] I2cBusDeviceExample: " format "\n", ##__VA_ARGS__)
#define DELAY_MS(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))
#endif

/**
 * @brief Demonstrate I2C bus and device access patterns.
 */
void demonstrateI2cAccess() {
    EXAMPLE_LOGI("=== I2C Bus-Device Access Demonstration ===");
    
    // Get CommChannelsManager instance
    auto& comm_mgr = CommChannelsManager::GetInstance();
    if (!comm_mgr.EnsureInitialized()) {
        EXAMPLE_LOGE("Failed to initialize CommChannelsManager");
        return;
    }
    
    // Method 1: Direct bus access (for advanced users)
    EXAMPLE_LOGI("Method 1: Direct bus access");
    try {
        auto& i2c_bus = comm_mgr.GetI2cBus();
        EXAMPLE_LOGI("I2C bus reference obtained successfully");
        EXAMPLE_LOGI("Bus has %zu devices configured", i2c_bus.GetDeviceCount());
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("Failed to get I2C bus: %s", e.what());
    }
    
    // Method 2: Device access by index (basic usage)
    EXAMPLE_LOGI("Method 2: Device access by index");
    std::size_t device_count = comm_mgr.GetI2cDeviceCount();
    EXAMPLE_LOGI("Total I2C devices: %zu", device_count);
    
    for (std::size_t i = 0; i < device_count; ++i) {
        BaseI2c* device = comm_mgr.GetI2cDevice(i);
        if (device) {
            EXAMPLE_LOGI("Device %zu: Available", i);
            // Here you could perform I2C operations using the BaseI2c interface
        } else {
            EXAMPLE_LOGW("Device %zu: Not available", i);
        }
    }
    
    // Method 3: Device access by enumeration (recommended for type safety)
    EXAMPLE_LOGI("Method 3: Device access by enumeration");
    
    // Access BNO08x IMU device
    BaseI2c* imu_device = comm_mgr.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (imu_device) {
        EXAMPLE_LOGI("BNO08x IMU device: Available");
        // Demo: Try to detect the device (read WHO_AM_I or similar)
        // This would be device-specific code
    } else {
        EXAMPLE_LOGW("BNO08x IMU device: Not available");
    }
    
    // Access PCAL95555 GPIO expander device
    BaseI2c* gpio_expander = comm_mgr.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (gpio_expander) {
        EXAMPLE_LOGI("PCAL95555 GPIO expander device: Available");
        // Demo: Try to read device ID or configuration
        // This would be device-specific code
    } else {
        EXAMPLE_LOGW("PCAL95555 GPIO expander device: Not available");
    }
    
    // Method 4: Convenience accessors (most user-friendly)
    EXAMPLE_LOGI("Method 4: Convenience accessors");
    
    BaseI2c* imu_convenience = comm_mgr.GetImu();
    if (imu_convenience) {
        EXAMPLE_LOGI("IMU (convenience): Available");
    } else {
        EXAMPLE_LOGW("IMU (convenience): Not available");
    }
    
    BaseI2c* gpio_convenience = comm_mgr.GetGpioExpander();
    if (gpio_convenience) {
        EXAMPLE_LOGI("GPIO expander (convenience): Available");
    } else {
        EXAMPLE_LOGW("GPIO expander (convenience): Not available");
    }
    
    // Method 5: ESP-specific device access (for advanced ESP-IDF features)
    EXAMPLE_LOGI("Method 5: ESP-specific device access");
    
    EspI2cDevice* esp_imu = comm_mgr.GetEspImu();
    if (esp_imu) {
        EXAMPLE_LOGI("ESP IMU device: Available (ESP-specific interface)");
        // Here you could use ESP-specific features like:
        // - Advanced timeout settings
        // - ESP-specific error handling
        // - Direct ESP-IDF driver access
    } else {
        EXAMPLE_LOGW("ESP IMU device: Not available");
    }
    
    EspI2cDevice* esp_gpio = comm_mgr.GetEspGpioExpander();
    if (esp_gpio) {
        EXAMPLE_LOGI("ESP GPIO expander device: Available (ESP-specific interface)");
    } else {
        EXAMPLE_LOGW("ESP GPIO expander device: Not available");
    }
}

/**
 * @brief Demonstrate basic I2C communication patterns.
 */
void demonstrateI2cCommunication() {
    EXAMPLE_LOGI("=== I2C Communication Demonstration ===");
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Get the IMU device for demonstration
    BaseI2c* imu_device = comm_mgr.GetImu();
    if (!imu_device) {
        EXAMPLE_LOGE("IMU device not available for communication demo");
        return;
    }
    
    // Demonstrate basic I2C read/write operations
    EXAMPLE_LOGI("Demonstrating basic I2C operations with IMU device");
    
    // Example 1: Simple register read
    try {
        uint8_t reg_addr = 0x00; // Example register address
        uint8_t data;
        
        // Note: This would be the actual I2C read operation
        // auto result = imu_device->ReadRegister(reg_addr, data);
        // if (result == I2cResult::Success) {
        //     EXAMPLE_LOGI("Register 0x%02X: 0x%02X", reg_addr, data);
        // }
        
        EXAMPLE_LOGI("I2C read operation template demonstrated");
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("I2C read error: %s", e.what());
    }
    
    // Example 2: Simple register write
    try {
        uint8_t reg_addr = 0x01; // Example register address
        uint8_t data = 0x42;     // Example data
        
        // Note: This would be the actual I2C write operation
        // auto result = imu_device->WriteRegister(reg_addr, data);
        // if (result == I2cResult::Success) {
        //     EXAMPLE_LOGI("Successfully wrote 0x%02X to register 0x%02X", data, reg_addr);
        // }
        
        EXAMPLE_LOGI("I2C write operation template demonstrated");
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("I2C write error: %s", e.what());
    }
    
    // Example 3: Multi-byte transfer
    try {
        uint8_t reg_addr = 0x02; // Example register address
        uint8_t buffer[4];       // Example buffer
        
        // Note: This would be the actual I2C multi-byte read operation
        // auto result = imu_device->ReadMultiple(reg_addr, buffer, sizeof(buffer));
        // if (result == I2cResult::Success) {
        //     EXAMPLE_LOGI("Multi-byte read: %02X %02X %02X %02X", 
        //                  buffer[0], buffer[1], buffer[2], buffer[3]);
        // }
        
        EXAMPLE_LOGI("I2C multi-byte operation template demonstrated");
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("I2C multi-byte error: %s", e.what());
    }
}

/**
 * @brief Demonstrate comparison with legacy I2C access.
 */
void demonstrateLegacyComparison() {
    EXAMPLE_LOGI("=== Legacy vs New I2C API Comparison ===");
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Legacy approach (deprecated but still supported)
    EXAMPLE_LOGI("Legacy approach (deprecated):");
    try {
        BaseI2c& legacy_i2c = comm_mgr.GetI2c(0); // Get first I2C device
        EXAMPLE_LOGI("Legacy I2C device obtained (returns first device)");
        // Note: Legacy approach doesn't distinguish between different devices
        // and assumes all devices are on the same interface
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("Legacy I2C access error: %s", e.what());
    }
    
    // New approach (recommended)
    EXAMPLE_LOGI("New approach (recommended):");
    BaseI2c* new_imu = comm_mgr.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    BaseI2c* new_gpio = comm_mgr.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    
    if (new_imu && new_gpio) {
        EXAMPLE_LOGI("New I2C devices obtained successfully");
        EXAMPLE_LOGI("- IMU device: Specific BNO08x configuration");
        EXAMPLE_LOGI("- GPIO expander: Specific PCAL95555 configuration");
        EXAMPLE_LOGI("- Each device has its own configuration and handle");
        EXAMPLE_LOGI("- Type-safe access using enumerations");
    } else {
        EXAMPLE_LOGW("Some new I2C devices not available");
    }
}

/**
 * @brief Main example function.
 */
int main() {
    EXAMPLE_LOGI("=== I2C Bus-Device Architecture Example ===");
    EXAMPLE_LOGI("This example demonstrates the new ESP-IDF v5.5+ I2C bus-device model");
    EXAMPLE_LOGI("New features:");
    EXAMPLE_LOGI("- Clean separation between bus and device");
    EXAMPLE_LOGI("- Per-device configuration and handles");
    EXAMPLE_LOGI("- Type-safe enumerated device access");
    EXAMPLE_LOGI("- Consistent API across all communication channels");
    
    try {
        // Initialize communication channels
        EXAMPLE_LOGI("Initializing communication channels...");
        auto& comm_mgr = CommChannelsManager::GetInstance();
        if (!comm_mgr.EnsureInitialized()) {
            EXAMPLE_LOGE("Failed to initialize CommChannelsManager");
            return -1;
        }
        EXAMPLE_LOGI("Communication channels initialized successfully");
        
        // Demonstrate different access patterns
        demonstrateI2cAccess();
        
        DELAY_MS(1000);
        
        // Demonstrate communication patterns
        demonstrateI2cCommunication();
        
        DELAY_MS(1000);
        
        // Demonstrate legacy vs new API comparison
        demonstrateLegacyComparison();
        
        EXAMPLE_LOGI("=== Example completed successfully ===");
        
    } catch (const std::exception& e) {
        EXAMPLE_LOGE("Example failed with exception: %s", e.what());
        return -1;
    }
    
    return 0;
}
