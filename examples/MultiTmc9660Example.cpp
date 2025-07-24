/**
 * @file MultiTmc9660Example.cpp
 * @brief Example demonstrating multi-device TMC9660 usage with MotorController.
 * 
 * This example shows how to:
 * 1. Initialize MotorController (automatically creates onboard TMC9660)
 * 2. Create external TMC9660 devices using SPI device IDs
 * 3. Access individual devices by index (no exceptions)
 * 4. Control GPIO and ADC on specific devices through handlers
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "MotorController.h"
#include "CommChannelsManager.h"
#include <iostream>
#include <memory>

/**
 * @brief Example of setting up multiple TMC9660 devices
 */
void setupMultipleTmc9660Devices() {
    auto& motorController = MotorController::GetInstance();
    
    std::cout << "=== Automatic Onboard + External Device Setup ===" << std::endl;
    
    // Initialize with automatic onboard device creation
    bool initSuccess = motorController.Initialize();
    if (initSuccess) {
        std::cout << "MotorController initialized with onboard device" << std::endl;
    } else {
        std::cout << "MotorController initialization failed!" << std::endl;
        return;
    }
    
    // Create external devices using SPI device IDs
    bool device1Created = motorController.CreateExternalDevice(
        MotorController::EXTERNAL_DEVICE_1_INDEX, 
        SpiDeviceId::EXTERNAL_DEVICE_1, 
        0x02  // TMC9660 address 2
    );
    
    bool device2Created = motorController.CreateExternalDevice(
        MotorController::EXTERNAL_DEVICE_2_INDEX, 
        SpiDeviceId::EXTERNAL_DEVICE_2, 
        0x03  // TMC9660 address 3
    );
    
    std::cout << "Device creation results:" << std::endl;
    std::cout << "  Onboard (index 0): Always created automatically" << std::endl;
    std::cout << "  External 1 (index 2): " << (device1Created ? "Success" : "Failed") << std::endl;
    std::cout << "  External 2 (index 3): " << (device2Created ? "Success" : "Failed") << std::endl;
    
    std::cout << "Total active devices: " << static_cast<int>(motorController.GetDeviceCount()) << std::endl;
}

/**
 * @brief Example of controlling individual devices (no exceptions)
 */
void controlIndividualDevices() {
    std::cout << "\n=== Individual Device Control ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    if (motorController.GetDeviceCount() == 0) {
        std::cout << "No devices available!" << std::endl;
        return;
    }
    
    // Control onboard device (index 0)
    auto* handler0 = motorController.handler(MotorController::ONBOARD_TMC9660_INDEX);
    if (handler0) {
        auto& gpio17_dev0 = handler0->gpio(17);
        gpio17_dev0.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
        std::cout << "✓ Set GPIO17 on onboard device to HIGH" << std::endl;
        
        // Read ADC from onboard device
        auto& adc_dev0 = handler0->adc();
        float voltage = 0.0f;
        hf_adc_err_t result = adc_dev0.ReadChannelV(0, voltage);
        if (result == hf_adc_err_t::ADC_SUCCESS) {
            std::cout << "✓ ADC Channel 0 on onboard device: " << voltage << "V" << std::endl;
        }
    } else {
        std::cout << "✗ Failed to get onboard device handler" << std::endl;
    }
    
    // Control external device 1 (if available)
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        auto* handler1 = motorController.handler(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (handler1) {
            auto& gpio18_dev1 = handler1->gpio(18);
            gpio18_dev1.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_LOW);
            std::cout << "✓ Set GPIO18 on external device 1 to LOW" << std::endl;
        } else {
            std::cout << "✗ Failed to get external device 1 handler" << std::endl;
        }
    }
    
    // Access TMC9660 driver directly for advanced control
    auto driver0 = motorController.driver(MotorController::ONBOARD_TMC9660_INDEX);
    if (driver0) {
        std::cout << "✓ Direct access to onboard TMC9660 driver successful" << std::endl;
        // Use driver0 for advanced TMC9660 operations...
    } else {
        std::cout << "✗ Onboard TMC9660 driver not available" << std::endl;
    }
}

/**
 * @brief Example of safe device enumeration (no exceptions)
 */
void enumerateDevices() {
    std::cout << "\n=== Device Enumeration ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    std::cout << "Total devices: " << static_cast<int>(motorController.GetDeviceCount()) << std::endl;
    
    auto activeIndices = motorController.GetActiveDeviceIndices();
    std::cout << "Active device indices: ";
    for (uint8_t index : activeIndices) {
        std::cout << static_cast<int>(index) << " ";
    }
    std::cout << std::endl;
    
    // Check each possible device slot
    for (uint8_t i = 0; i < MotorController::MAX_TMC9660_DEVICES; ++i) {
        std::cout << "Device " << static_cast<int>(i) << ": ";
        
        if (motorController.IsDeviceValid(i)) {
            auto* handler = motorController.handler(i);
            if (handler) {
                std::cout << "Active and handler ready";
                
                auto driver = motorController.driver(i);
                if (driver) {
                    std::cout << " (driver available)";
                } else {
                    std::cout << " (driver not ready)";
                }
            } else {
                std::cout << "Active but handler not available";
            }
        } else {
            std::cout << "Not active";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief Main example function
 */
int main() {
    std::cout << "=== Multi-TMC9660 Device Example (No Exceptions) ===" << std::endl;
    
    // Setup automatic onboard + external devices
    setupMultipleTmc9660Devices();
    
    // Enumerate all devices safely
    enumerateDevices();
    
    // Control individual devices
    controlIndividualDevices();
    
    std::cout << "\n=== Example completed ===" << std::endl;
    return 0;
}
