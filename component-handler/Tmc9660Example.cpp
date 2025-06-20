/**
 * @file Tmc9660Example.cpp
 * @brief Example usage of the TMC9660 Motor Controller system.
 * 
 * This file demonstrates how to use the TMC9660 motor controller singleton
 * for motor control operations in the HardFOC system.
 */

#include "component-handler/All.h"
#include "ConsolePort.h"
#include "OsUtility.h"

static const char* TAG = "Tmc9660Example";

/**
 * @brief Basic TMC9660 initialization and setup example.
 */
void BasicTmc9660Setup() {
    console_info(TAG, "=== Basic TMC9660 Setup Example ===");
    
    using namespace HardFocComponentHandler;
    
    // Initialize the entire HardFOC system (includes TMC9660)
    if (!Initialize()) {
        console_error(TAG, "Failed to initialize HardFOC system");
        return;
    }
    
    // Get the motor controller instance
    MotorController& motorController = GetMotorController();
    
    // Check system health
    if (!IsHealthy()) {
        console_error(TAG, "System health check failed");
        return;
    }
    
    console_info(TAG, "TMC9660 system initialized successfully");
    console_info(TAG, "Registered chips: %d", motorController.GetRegisteredChipCount());
}

/**
 * @brief Example of basic motor control operations.
 */
void BasicMotorControl() {
    console_info(TAG, "=== Basic Motor Control Example ===");
    
    using namespace HardFocComponentHandler;
    
    MotorController& motorController = GetMotorController();
    Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;
    
    // Check if the primary chip is registered
    if (!motorController.IsChipRegistered(primaryChip)) {
        console_error(TAG, "Primary TMC9660 chip not registered");
        return;
    }
    
    // Get chip status
    Tmc9660Status status;
    if (motorController.GetChipStatus(primaryChip, status)) {
        console_info(TAG, "Chip Status:");
        console_info(TAG, "  Initialized: %s", status.isInitialized ? "Yes" : "No");
        console_info(TAG, "  Driver Enabled: %s", status.isDriverEnabled ? "Yes" : "No");
        console_info(TAG, "  Has Fault: %s", status.hasFault ? "Yes" : "No");
        console_info(TAG, "  Communicating: %s", status.isCommunicating ? "Yes" : "No");
    }
    
    // Enable the motor driver
    console_info(TAG, "Enabling motor driver...");
    if (motorController.EnableDriver(primaryChip)) {
        console_info(TAG, "Motor driver enabled successfully");
    } else {
        console_error(TAG, "Failed to enable motor driver");
        return;
    }
    
    // Wait a bit
    os_delay_msec(1000);
    
    // Check for faults
    if (motorController.HasFault(primaryChip)) {
        console_warning(TAG, "Motor fault detected!");
        if (motorController.ClearFault(primaryChip)) {
            console_info(TAG, "Fault cleared successfully");
        } else {
            console_error(TAG, "Failed to clear fault");
        }
    }
    
    // Disable the motor driver
    console_info(TAG, "Disabling motor driver...");
    if (motorController.DisableDriver(primaryChip)) {
        console_info(TAG, "Motor driver disabled successfully");
    } else {
        console_error(TAG, "Failed to disable motor driver");
    }
}

/**
 * @brief Example of communication interface switching.
 */
void CommunicationInterfaceExample() {
    console_info(TAG, "=== Communication Interface Example ===");
    
    using namespace HardFocComponentHandler;
    
    MotorController& motorController = GetMotorController();
    Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;
    
    // Switch to SPI communication
    console_info(TAG, "Switching to SPI communication...");
    if (motorController.SwitchCommunicationInterface(primaryChip, Tmc9660CommInterface::TMC_COMM_SPI)) {
        console_info(TAG, "Switched to SPI successfully");
        
        // Test communication
        if (motorController.TestCommunication()) {
            console_info(TAG, "SPI communication test passed");
        } else {
            console_warning(TAG, "SPI communication test failed");
        }
        
        // Wait a bit
        os_delay_msec(2000);
        
        // Switch back to UART
        console_info(TAG, "Switching back to UART communication...");
        if (motorController.SwitchCommunicationInterface(primaryChip, Tmc9660CommInterface::TMC_COMM_UART)) {
            console_info(TAG, "Switched to UART successfully");
        } else {
            console_error(TAG, "Failed to switch to UART");
        }
    } else {
        console_error(TAG, "Failed to switch to SPI");
    }
}

/**
 * @brief Example of ADC reading from TMC9660.
 */
void AdcReadingExample() {
    console_info(TAG, "=== ADC Reading Example ===");
    
    using namespace HardFocComponentHandler;
    
    MotorController& motorController = GetMotorController();
    Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;
    
    // Read all three ADC channels
    for (uint8_t channel = 1; channel <= 3; channel++) {
        uint32_t adcCount;
        float adcVoltage;
        
        // Read ADC count
        if (motorController.ReadAdcValue(primaryChip, channel, adcCount)) {
            console_info(TAG, "AIN%d count: %lu", channel, adcCount);
        } else {
            console_error(TAG, "Failed to read AIN%d count", channel);
            continue;
        }
        
        // Read ADC voltage
        if (motorController.ReadAdcVoltage(primaryChip, channel, adcVoltage)) {
            console_info(TAG, "AIN%d voltage: %.3f V", channel, adcVoltage);
        } else {
            console_error(TAG, "Failed to read AIN%d voltage", channel);
        }
        
        os_delay_msec(100);
    }
}

/**
 * @brief Example of GPIO control through TMC9660.
 */
void GpioControlExample() {
    console_info(TAG, "=== GPIO Control Example ===");
    
    using namespace HardFocComponentHandler;
    
    MotorController& motorController = GetMotorController();
    Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;
    
    // Control GPIO pins 17 and 18
    for (uint8_t gpio = 17; gpio <= 18; gpio++) {
        console_info(TAG, "Testing GPIO %d...", gpio);
        
        // Set GPIO high
        if (motorController.SetGpioState(primaryChip, gpio, true)) {
            console_info(TAG, "GPIO %d set high", gpio);
        } else {
            console_error(TAG, "Failed to set GPIO %d high", gpio);
            continue;
        }
        
        os_delay_msec(500);
        
        // Read GPIO state
        bool state = motorController.GetGpioState(primaryChip, gpio);
        console_info(TAG, "GPIO %d state: %s", gpio, state ? "High" : "Low");
        
        // Set GPIO low
        if (motorController.SetGpioState(primaryChip, gpio, false)) {
            console_info(TAG, "GPIO %d set low", gpio);
        } else {
            console_error(TAG, "Failed to set GPIO %d low", gpio);
        }
        
        os_delay_msec(500);
    }
}

/**
 * @brief Example of system diagnostics.
 */
void SystemDiagnosticsExample() {
    console_info(TAG, "=== System Diagnostics Example ===");
    
    using namespace HardFocComponentHandler;
    
    MotorController& motorController = GetMotorController();
    
    // Run diagnostics
    console_info(TAG, "Running system diagnostics...");
    if (motorController.RunDiagnostics()) {
        console_info(TAG, "All diagnostics passed");
    } else {
        console_warning(TAG, "Some diagnostics failed");
    }
    
    // Print system status
    motorController.PrintSystemStatus();
    
    // Check overall system health
    if (IsHealthy()) {
        console_info(TAG, "Overall system health: GOOD");
    } else {
        console_warning(TAG, "Overall system health: DEGRADED");
    }
}

/**
 * @brief Complete TMC9660 usage example.
 */
void CompleteTmc9660Example() {
    console_info(TAG, "=== Complete TMC9660 Usage Example ===");
    
    // Run all examples
    BasicTmc9660Setup();
    os_delay_msec(1000);
    
    BasicMotorControl();
    os_delay_msec(1000);
    
    CommunicationInterfaceExample();
    os_delay_msec(1000);
    
    AdcReadingExample();
    os_delay_msec(1000);
    
    GpioControlExample();
    os_delay_msec(1000);
    
    SystemDiagnosticsExample();
    
    console_info(TAG, "Example completed successfully");
}

/**
 * @brief Motor control loop example.
 */
void MotorControlLoopExample() {
    console_info(TAG, "=== Motor Control Loop Example ===");
    
    using namespace HardFocComponentHandler;
    
    MotorController& motorController = GetMotorController();
    Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;
    
    // Enable the motor
    if (!motorController.EnableDriver(primaryChip)) {
        console_error(TAG, "Failed to enable motor driver");
        return;
    }
    
    console_info(TAG, "Starting motor control loop...");
    
    // Run for 10 seconds
    for (int i = 0; i < 100; i++) {
        // Check for faults
        if (motorController.HasFault(primaryChip)) {
            console_warning(TAG, "Fault detected, attempting to clear...");
            if (!motorController.ClearFault(primaryChip)) {
                console_error(TAG, "Failed to clear fault, stopping");
                break;
            }
        }
        
        // Read motor current (assuming AIN1 is connected to current sensor)
        float current;
        if (motorController.ReadAdcVoltage(primaryChip, 1, current)) {
            console_info(TAG, "Motor current: %.3f A", current / 0.1f); // Assuming 100mV/A scale
        }
        
        // Perform maintenance
        Maintain();
        
        os_delay_msec(100);
    }
    
    // Disable the motor
    motorController.DisableDriver(primaryChip);
    console_info(TAG, "Motor control loop completed");
}
