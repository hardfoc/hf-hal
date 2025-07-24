/**
 * @file BoardAwareTmc9660Example.cpp
 * @brief Example demonstrating board-aware TMC9660 device management with MotorController.
 * 
 * This example shows how to:
 * 1. Initialize the MotorController (automatically creates onboard TMC9660)
 * 2. Create external TMC9660 devices using SPI device IDs
 * 3. Access devices by their board-defined indices
 * 4. Safely delete external devices when needed
 * 5. Handle device enumeration and validation
 * 
 * Board Configuration:
 * - Index 0: Onboard TMC9660 (SPI2_CS_TMC9660) - Auto-created, cannot be deleted
 * - Index 1: AS5047U Position Encoder (SPI2_CS_AS5047) - Not used for TMC9660
 * - Index 2: External Device 1 (EXT_GPIO_CS_1) - Dynamic TMC9660 creation/deletion
 * - Index 3: External Device 2 (EXT_GPIO_CS_2) - Dynamic TMC9660 creation/deletion
 * 
 * New Features:
 * - Automatic onboard device creation via CommChannelsManager integration
 * - SPI device ID specification instead of manual interface management
 * - Simplified initialization with one-line setup
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "MotorController.h"
#include "CommChannelsManager.h"
#include <iostream>
#include <memory>

/**
 * @brief Initialize the motor controller system with onboard device
 */
void initializeMotorControllerSystem() {
    std::cout << "=== Initializing Motor Controller System ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Initialize the system - this automatically creates the onboard TMC9660
    bool initSuccess = motorController.Initialize();
    if (initSuccess) {
        std::cout << "✓ Motor controller system initialized successfully!" << std::endl;
        std::cout << "✓ Onboard TMC9660 device created at index " 
                  << static_cast<int>(MotorController::ONBOARD_TMC9660_INDEX) << std::endl;
    } else {
        std::cout << "✗ Motor controller system initialization failed!" << std::endl;
        return;
    }
    
    // Show active devices
    auto activeDevices = motorController.GetActiveDeviceIndices();
    std::cout << "Active devices: ";
    for (auto idx : activeDevices) {
        std::cout << static_cast<int>(idx) << " ";
    }
    std::cout << "(Total: " << static_cast<int>(motorController.GetDeviceCount()) << ")" << std::endl;
}

/**
 * @brief Create external TMC9660 devices on available CS lines
 */
void createExternalDevices() {
    std::cout << "\n=== Creating External TMC9660 Devices ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Check available external slots
    bool slot2Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
    bool slot3Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_2_INDEX);
    
    std::cout << "External slot 2 (EXT_GPIO_CS_1): " << (slot2Available ? "Available" : "Occupied") << std::endl;
    std::cout << "External slot 3 (EXT_GPIO_CS_2): " << (slot3Available ? "Available" : "Occupied") << std::endl;
    
    // Create external device 1 using SPI device ID
    if (slot2Available) {
        bool created = motorController.CreateExternalDevice(
            MotorController::EXTERNAL_DEVICE_1_INDEX, 
            SpiDeviceId::EXTERNAL_DEVICE_1, 
            0x02  // Device address 2
        );
        if (created) {
            std::cout << "✓ External TMC9660 device 1 created at index " 
                      << static_cast<int>(MotorController::EXTERNAL_DEVICE_1_INDEX) << std::endl;
        } else {
            std::cout << "✗ Failed to create external TMC9660 device 1" << std::endl;
        }
    }
    
    // Create external device 2 using SPI device ID
    if (slot3Available) {
        bool created = motorController.CreateExternalDevice(
            MotorController::EXTERNAL_DEVICE_2_INDEX,
            SpiDeviceId::EXTERNAL_DEVICE_2,
            0x03  // Device address 3
        );
        if (created) {
            std::cout << "✓ External TMC9660 device 2 created at index " 
                      << static_cast<int>(MotorController::EXTERNAL_DEVICE_2_INDEX) << std::endl;
        } else {
            std::cout << "✗ Failed to create external TMC9660 device 2" << std::endl;
        }
        } else {
            std::cout << "✗ SPI interface for external device 2 not available" << std::endl;
        }
    }
    
    // Alternative: Create external device using UART interface
    if (commManager.GetUartCount() > 0) {
        std::cout << "Note: External devices can also be created with UART interface using CreateExternalDevice()" << std::endl;
    }
    
    // Show updated device count
    std::cout << "Total active devices: " << static_cast<int>(motorController.GetDeviceCount()) << std::endl;
}

/**
 * @brief Control specific devices by their board indices
 */
void controlSpecificDevices() {
    std::cout << "\n=== Controlling Specific Devices ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Control onboard TMC9660 (always available) - no exceptions
    if (motorController.IsDeviceValid(MotorController::ONBOARD_TMC9660_INDEX)) {
        auto* onboardHandler = motorController.handler(MotorController::ONBOARD_TMC9660_INDEX);
        if (onboardHandler) {
            // Access GPIO and ADC through the handler
            auto& gpio17_onboard = onboardHandler->gpio(17);
            gpio17_onboard.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
            std::cout << "✓ Set GPIO17 on onboard TMC9660 (index 0) to HIGH" << std::endl;
            
            auto& adc_onboard = onboardHandler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc_onboard.ReadChannelV(0, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                std::cout << "✓ ADC Channel 0 on onboard TMC9660: " << voltage << "V" << std::endl;
            }
        } else {
            std::cout << "✗ Failed to get onboard TMC9660 handler" << std::endl;
        }
    }
    
    // Control external device 1 (if available) - no exceptions
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        auto* ext1Handler = motorController.handler(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (ext1Handler) {
            // Access GPIO and driver through the handler
            auto& gpio18_ext1 = ext1Handler->gpio(18);
            gpio18_ext1.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_LOW);
            std::cout << "✓ Set GPIO18 on external device 1 (index 2) to LOW" << std::endl;
            
            // Access TMC9660 driver directly for advanced control - no exceptions
            auto driver_ext1 = motorController.driver(MotorController::EXTERNAL_DEVICE_1_INDEX);
            if (driver_ext1) {
                std::cout << "✓ Direct access to external TMC9660 driver 1 successful" << std::endl;
                // Use driver_ext1 for advanced TMC9660 operations...
            } else {
                std::cout << "✗ External TMC9660 driver 1 not available" << std::endl;
            }
        } else {
            std::cout << "✗ Failed to get external device 1 handler" << std::endl;
        }
    }
    
    // Control external device 2 (if available) - no exceptions
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_2_INDEX)) {
        auto* ext2Handler = motorController.handler(MotorController::EXTERNAL_DEVICE_2_INDEX);
        if (ext2Handler) {
            // Access ADC through the handler
            auto& adc_ext2 = ext2Handler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc_ext2.ReadChannelV(1, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                std::cout << "✓ ADC Channel 1 on external device 2: " << voltage << "V" << std::endl;
            }
        } else {
            std::cout << "✗ Failed to get external device 2 handler" << std::endl;
        }
    }
}

/**
 * @brief Demonstrate device enumeration and validation
 */
void enumerateAndValidateDevices() {
    std::cout << "\n=== Device Enumeration and Validation ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Get all active device indices
    auto activeDevices = motorController.GetActiveDeviceIndices();
    std::cout << "Active device indices: ";
    for (auto idx : activeDevices) {
        std::cout << static_cast<int>(idx) << " ";
    }
    std::cout << std::endl;
    
    // Check each possible device slot
    for (uint8_t i = 0; i < MotorController::MAX_TMC9660_DEVICES; ++i) {
        std::cout << "Device slot " << static_cast<int>(i) << ": ";
        
        if (i == 0) {
            std::cout << "Onboard TMC9660 - ";
        } else if (i == 1) {
            std::cout << "AS5047U Encoder - ";
        } else if (i == 2) {
            std::cout << "External Device 1 - ";
        } else if (i == 3) {
            std::cout << "External Device 2 - ";
        }
        
        if (motorController.IsDeviceValid(i)) {
            std::cout << "ACTIVE";
            auto* handler = motorController.handler(i);
            if (handler) {
                std::cout << " (Handler ready)";
            } else {
                std::cout << " (Handler not available)";
            }
            }
        } else {
            std::cout << "INACTIVE";
        }
        std::cout << std::endl;
    }
    
    // Check initialization status
    auto initStatus = motorController.GetInitializationStatus();
    std::cout << "Initialization status: ";
    for (size_t i = 0; i < initStatus.size(); ++i) {
        std::cout << (initStatus[i] ? "OK" : "FAILED") << " ";
    }
    std::cout << std::endl;
}

/**
 * @brief Demonstrate external device deletion
 */
void demonstrateDeviceDeletion() {
    std::cout << "\n=== External Device Deletion ===" << std::endl;
    
    auto& motorController = MotorController::GetInstance();
    
    // Try to delete onboard device (should fail)
    bool onboardDeleted = motorController.DeleteExternalDevice(MotorController::ONBOARD_TMC9660_INDEX);
    std::cout << "Attempt to delete onboard device (index 0): " 
              << (onboardDeleted ? "✓ SUCCESS" : "✗ FAILED (as expected)") << std::endl;
    
    // Delete external device 1 if it exists
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        bool ext1Deleted = motorController.DeleteExternalDevice(MotorController::EXTERNAL_DEVICE_1_INDEX);
        std::cout << "Delete external device 1 (index 2): " 
                  << (ext1Deleted ? "✓ SUCCESS" : "✗ FAILED") << std::endl;
    } else {
        std::cout << "External device 1 (index 2): Not present, cannot delete" << std::endl;
    }
    
    // Show updated device count
    auto activeDevices = motorController.GetActiveDeviceIndices();
    std::cout << "Remaining active devices: ";
    for (auto idx : activeDevices) {
        std::cout << static_cast<int>(idx) << " ";
    }
    std::cout << "(Total: " << static_cast<int>(motorController.GetDeviceCount()) << ")" << std::endl;
    
    // Verify external device 1 slot is now available
    bool slot2Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
    std::cout << "External slot 2 (EXT_GPIO_CS_1) availability: " 
              << (slot2Available ? "✓ Available for new device" : "✗ Still occupied") << std::endl;
}

/**
 * @brief Main example function
 */
int main() {
    std::cout << "=== Board-Aware TMC9660 Device Management Example ===" << std::endl;
    
    // Initialize the motor controller system (creates onboard device)
    initializeMotorControllerSystem();
    
    // Create external devices on available CS lines
    createExternalDevices();
    
    // Enumerate and validate all devices
    enumerateAndValidateDevices();
    
    // Control specific devices by their board indices
    controlSpecificDevices();
    
    // Demonstrate external device deletion
    demonstrateDeviceDeletion();
    
    std::cout << "\n=== Example completed ===" << std::endl;
    return 0;
}
