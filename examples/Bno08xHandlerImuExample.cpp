/**
 * @file Bno08xHandlerImuExample.cpp
 * @brief Example demonstrating Bno08xHandler usage through ImuManager with GPIO interrupt support.
 * 
 * This example shows how to:
 * 1. Initialize ImuManager (automatically creates Bno08xHandler and GPIO interrupt support)
 * 2. Access BNO08x IMU through unified Bno08xHandler interface
 * 3. Configure GPIO interrupts using PCAL_IMU_INT pin through GpioManager
 * 4. Use both polling and interrupt-driven modes
 * 5. Handle errors gracefully with pointer-based returns
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "ImuManager.h"
#include "GpioManager.h"
#include "utils-and-drivers/driver-handlers/Bno08xHandler.h"
#include <iostream>
#include <memory>

// ESP-IDF for tasks and logging
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

/**
 * @brief Example of setting up BNO08x IMU using Bno08xHandler
 */
void setupBno08xHandler() {
    auto& imu_mgr = ImuManager::GetInstance();
    
    std::cout << "=== BNO08x Handler Setup ===" << std::endl;
    
    // Initialize with automatic Bno08xHandler creation
    bool initSuccess = imu_mgr.Initialize();
    if (initSuccess) {
        std::cout << "ImuManager initialized with Bno08xHandler" << std::endl;
    } else {
        std::cout << "ImuManager initialization failed!" << std::endl;
        return;
    }
    
    std::cout << "Total IMU devices: " << static_cast<int>(imu_mgr.GetImuCount()) << std::endl;
    
    // List available devices
    auto devices = imu_mgr.GetAvailableDevices();
    std::cout << "Available IMU devices:" << std::endl;
    for (const auto& device : devices) {
        std::cout << "  - " << device << std::endl;
    }
}

/**
 * @brief Example of using Bno08xHandler for sensor operations (no exceptions)
 */
void useBno08xHandler() {
    std::cout << "\n=== BNO08x Handler Usage ===" << std::endl;
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Get the BNO08x handler (safe pointer-based access)
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    if (!handler) {
        std::cout << "✗ BNO08x handler not available" << std::endl;
        return;
    }
    
    std::cout << "✓ BNO08x handler available" << std::endl;
    
    // Configure sensor callback (if supported by handler)
    // handler->SetSensorCallback([](const SensorEvent& event) {
    //     std::cout << "Sensor data: type=" << static_cast<int>(event.sensor) 
    //               << ", timestamp=" << event.timestamp << std::endl;
    // });
    
    // Enable rotation vector sensor (if supported)
    // Bno08xError result = handler->EnableSensor(Bno08xSensorType::ROTATION_VECTOR, 50);
    // if (result == Bno08xError::SUCCESS) {
    //     std::cout << "✓ Rotation vector sensor enabled (50Hz)" << std::endl;
    // } else {
    //     std::cout << "✗ Failed to enable rotation vector sensor: " 
    //               << Bno08xErrorToString(result) << std::endl;
    // }
    
    // Example of reading sensor status
    // bool isEnabled = handler->IsSensorEnabled(Bno08xSensorType::ROTATION_VECTOR);
    // std::cout << "Rotation vector enabled: " << (isEnabled ? "Yes" : "No") << std::endl;
    
    std::cout << "✓ BNO08x handler operations completed" << std::endl;
}

/**
 * @brief Example of configuring and using GPIO interrupts for BNO08x
 */
void demonstrateInterruptMode() {
    std::cout << "\n=== BNO08x Interrupt Mode Example ===" << std::endl;
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Check if interrupt is available
    if (!imu_mgr.IsInterruptEnabled()) {
        std::cout << "Configuring BNO08x interrupt..." << std::endl;
        
        // Configure interrupt with callback
        bool interrupt_configured = imu_mgr.ConfigureInterrupt([]() {
            // This callback executes in interrupt context - keep it minimal!
            static uint32_t callback_count = 0;
            callback_count++;
            // In real applications, you might set a flag or post to a queue
        });
        
        if (interrupt_configured) {
            std::cout << "✓ Interrupt configured successfully" << std::endl;
            
            // Enable the interrupt
            if (imu_mgr.EnableInterrupt()) {
                std::cout << "✓ Interrupt enabled successfully" << std::endl;
                
                // Demonstrate interrupt-driven operation
                std::cout << "Waiting for interrupts (10 seconds)..." << std::endl;
                for (int i = 0; i < 10; ++i) {
                    if (imu_mgr.WaitForInterrupt(1000)) {  // 1 second timeout
                        std::cout << "Interrupt received! Count: " << imu_mgr.GetInterruptCount() << std::endl;
                        
                        // In real application, you would call handler->Update() here
                        // to process the sensor data that triggered the interrupt
                    } else {
                        std::cout << "." << std::flush;  // Timeout indicator
                    }
                }
                std::cout << std::endl;
                
                // Disable interrupt when done
                imu_mgr.DisableInterrupt();
                std::cout << "✓ Interrupt disabled" << std::endl;
            } else {
                std::cout << "✗ Failed to enable interrupt" << std::endl;
            }
        } else {
            std::cout << "✗ Failed to configure interrupt - falling back to polling mode" << std::endl;
        }
    } else {
        std::cout << "✓ Interrupt already enabled" << std::endl;
    }
    
    std::cout << "Total interrupts received: " << imu_mgr.GetInterruptCount() << std::endl;
}

/**
 * @brief Example of safe error handling with handler interface
 */
void demonstrateErrorHandling() {
    std::cout << "\n=== Error Handling Example ===" << std::endl;
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Check if IMU is available before using
    if (!imu_mgr.IsBno08xAvailable()) {
        std::cout << "✗ BNO08x not available" << std::endl;
        return;
    }
    
    // Safe handler access
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    if (handler) {
        std::cout << "✓ Handler access successful" << std::endl;
        
        // All operations through handler are exception-free
        // Error checking through return values and error codes
        
        // Example error handling pattern:
        // Bno08xError result = handler->SomeOperation();
        // if (result != Bno08xError::SUCCESS) {
        //     std::cout << "Operation failed: " << Bno08xErrorToString(result) << std::endl;
        //     return;
        // }
        
    } else {
        std::cout << "✗ Handler access failed" << std::endl;
    }
}

/**
 * @brief Example of data polling loop (typical usage pattern)
 */
void dataPollingExample() {
    std::cout << "\n=== Data Polling Example ===" << std::endl;
    
    auto& imu_mgr = ImuManager::GetInstance();
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    
    if (!handler) {
        std::cout << "✗ Handler not available for polling" << std::endl;
        return;
    }
    
    std::cout << "✓ Starting data polling simulation..." << std::endl;
    
    // Simulate polling loop (in real application, this would be continuous)
    for (int i = 0; i < 5; ++i) {
        // Update handler to process new data
        // handler->Update();  // This would trigger callbacks if new data available
        
        // Or read specific sensor data directly
        // RotationVector rotation;
        // if (handler->GetRotationVector(rotation)) {
        //     std::cout << "Rotation: qx=" << rotation.x << ", qy=" << rotation.y 
        //               << ", qz=" << rotation.z << ", qw=" << rotation.w << std::endl;
        // }
        
        std::cout << "Polling iteration " << (i + 1) << "/5" << std::endl;
        
        // In real application: vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz polling
    }
    
    std::cout << "✓ Polling simulation completed" << std::endl;
}

/**
 * @brief Main example function
 */
int main() {
    std::cout << "=== Bno08xHandler IMU Example with GPIO Interrupt Support ===" << std::endl;
    
    // Setup BNO08x handler through ImuManager
    setupBno08xHandler();
    
    // Demonstrate handler usage
    useBno08xHandler();
    
    // Demonstrate interrupt functionality
    demonstrateInterruptMode();
    
    // Show error handling patterns
    demonstrateErrorHandling();
    
    // Demonstrate typical polling usage
    dataPollingExample();
    
    std::cout << "\n=== Example completed ===" << std::endl;
    return 0;
}
