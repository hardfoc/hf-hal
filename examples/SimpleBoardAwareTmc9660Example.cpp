/**
 * @file SimpleBoardAwareTmc9660Example.cpp
 * @brief Simple example demonstrating automatic onboard device creation and external device management.
 * 
 * This example shows how to:
 * 1. Initialize MotorController (automatically creates onboard TMC9660)
 * 2. Create external TMC9660 devices using SPI device IDs
 * 3. Access GPIO and ADC through handlers (no exceptions)
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "MotorController.h"
#include "CommChannelsManager.h"
#include <iostream>

/**
 * @brief Example of automatic onboard device initialization
 */
void automaticOnboardDeviceExample() {
    std::cout << "=== Automatic Onboard Device Creation ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Simply call Initialize() - onboard device is created automatically
    bool initSuccess = motorController.Initialize();
    if (initSuccess) {
        std::cout << "MotorController initialized successfully!" << std::endl;
        std::cout << "Onboard TMC9660 device created automatically at index 0" << std::endl;
    } else {
        std::cout << "MotorController initialization failed!" << std::endl;
        return;
    }
    
    // Verify onboard device is available (no exceptions)
    if (motorController.IsDeviceValid(MotorController::ONBOARD_TMC9660_INDEX)) {
        std::cout << "Onboard TMC9660 device is ready for use" << std::endl;
        
        // Access handler safely (returns pointer, not reference)
        auto* handler = motorController.handler(MotorController::ONBOARD_TMC9660_INDEX);
        if (handler) {
            // Access GPIO through handler
            auto& gpio17 = handler->gpio(17);
            gpio17.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
            std::cout << "Set GPIO17 on onboard device to HIGH" << std::endl;
            
            // Access ADC through handler
            auto& adc = handler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc.ReadChannelV(0, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                std::cout << "ADC Channel 0 on onboard device: " << voltage << "V" << std::endl;
            }
        } else {
            std::cout << "Failed to get onboard device handler" << std::endl;
        }
    }
}

/**
 * @brief Example of creating external devices using SPI device IDs
 */
void externalDeviceCreationExample() {
    std::cout << "\n=== External Device Creation ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Create external device 1 using SpiDeviceId
    bool device1Created = motorController.CreateExternalDevice(
        MotorController::EXTERNAL_DEVICE_1_INDEX, 
        SpiDeviceId::EXTERNAL_DEVICE_1, 
        0x02  // TMC9660 address 2
    );
    
    if (device1Created) {
        std::cout << "External device 1 created successfully at index " 
                  << static_cast<int>(MotorController::EXTERNAL_DEVICE_1_INDEX) << std::endl;
        
        // Access the external device through handler (no exceptions)
        auto* handler1 = motorController.handler(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (handler1) {
            auto& gpio18 = handler1->gpio(18);
            gpio18.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_LOW);
            std::cout << "Set GPIO18 on external device 1 to LOW" << std::endl;
        } else {
            std::cout << "Failed to get external device 1 handler" << std::endl;
        }
    } else {
        std::cout << "Failed to create external device 1" << std::endl;
    }
    
    // Create external device 2 using SpiDeviceId
    bool device2Created = motorController.CreateExternalDevice(
        MotorController::EXTERNAL_DEVICE_2_INDEX, 
        SpiDeviceId::EXTERNAL_DEVICE_2, 
        0x03  // TMC9660 address 3
    );
    
    if (device2Created) {
        std::cout << "External device 2 created successfully at index " 
                  << static_cast<int>(MotorController::EXTERNAL_DEVICE_2_INDEX) << std::endl;
    } else {
        std::cout << "Failed to create external device 2" << std::endl;
    }
}

/**
 * @brief Example of device status reporting
 */
void deviceStatusExample() {
    std::cout << "\n=== Device Status Report ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    uint8_t deviceCount = motorController.GetDeviceCount();
    std::cout << "Total active devices: " << static_cast<int>(deviceCount) << std::endl;
    
    auto activeIndices = motorController.GetActiveDeviceIndices();
    std::cout << "Active device indices: ";
    for (uint8_t index : activeIndices) {
        std::cout << static_cast<int>(index) << " ";
    }
    std::cout << std::endl;
    
    auto initStatus = motorController.GetInitializationStatus();
    for (size_t i = 0; i < initStatus.size(); ++i) {
        if (motorController.IsDeviceValid(static_cast<uint8_t>(i))) {
            std::cout << "Device " << i << ": " 
                      << (initStatus[i] ? "Initialized" : "Not Initialized") << std::endl;
        }
    }
}

/**
 * @brief Example of safe driver access (no exceptions)
 */
void driverAccessExample() {
    std::cout << "\n=== Driver Access Example ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Access TMC9660 driver safely (returns nullptr if not available)
    auto driver = motorController.driver(MotorController::ONBOARD_TMC9660_INDEX);
    if (driver) {
        std::cout << "Successfully accessed TMC9660 driver for onboard device" << std::endl;
        // Use driver for advanced TMC9660 operations...
    } else {
        std::cout << "TMC9660 driver not available for onboard device" << std::endl;
    }
    
    // Check external device drivers
    for (auto index : motorController.GetActiveDeviceIndices()) {
        if (index != MotorController::ONBOARD_TMC9660_INDEX) {
            auto extDriver = motorController.driver(index);
            if (extDriver) {
                std::cout << "External device " << static_cast<int>(index) 
                          << " driver is available" << std::endl;
            } else {
                std::cout << "External device " << static_cast<int>(index) 
                          << " driver is not available" << std::endl;
            }
        }
    }
}

/**
 * @brief Example of external device cleanup
 */
void deviceCleanupExample() {
    std::cout << "\n=== External Device Cleanup ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Delete external device 1 if it exists
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        bool deleted = motorController.DeleteExternalDevice(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (deleted) {
            std::cout << "External device 1 deleted successfully" << std::endl;
        }
    }
    
    // Try to delete onboard device (should fail)
    bool onboardDeleted = motorController.DeleteExternalDevice(MotorController::ONBOARD_TMC9660_INDEX);
    if (!onboardDeleted) {
        std::cout << "Onboard device cannot be deleted (expected behavior)" << std::endl;
    }
}

/**
 * @brief Main example function
 */
int main() {
    std::cout << "=== Simple Board-Aware TMC9660 Example ===" << std::endl;
    
    // 1. Automatic onboard device creation
    automaticOnboardDeviceExample();
    
    // 2. External device creation using SPI device IDs
    externalDeviceCreationExample();
    
    // 3. Device status reporting
    deviceStatusExample();
    
    // 4. Safe driver access (no exceptions)
    driverAccessExample();
    
    // 5. External device cleanup
    deviceCleanupExample();
    
    std::cout << "\n=== Example completed ===" << std::endl;
    return 0;
}