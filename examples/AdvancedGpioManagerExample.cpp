/**
 * @file AdvancedGpioManagerExample.cpp
 * @brief Advanced example demonstrating the new GpioManager with TMC9660 integration
 * 
 * This example shows how to use the new GpioManager system with:
 * - Multi-chip GPIO management (ESP32, PCAL95555, TMC9660)
 * - Platform mapping and functional GPIO pin usage
 * - TMC9660 motor controller integration
 * - Advanced GPIO features (interrupts, batch operations, diagnostics)
 * - Thread-safe operations and error handling
 */

#include "component-handler/All.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_config.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_ext_pins_enum.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_functional_pin_config.hpp"
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/BaseThread.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/ConsolePort.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-general/include/OsUtility.h"

using namespace hf::hal;
using namespace hf::utils;

static const char* TAG = "AdvancedGpioExample";

// Global instances
GpioManager& gpioManager = GpioManager::GetInstance();
AdcManager& adcManager = AdcManager::GetInstance();
Tmc9660MotorController& tmcController = Tmc9660MotorController::GetInstance();

// GPIO interrupt callback for demonstration
class GpioInterruptHandler : public IGpioInterruptCallback {
public:
    void OnGpioInterrupt(GpioPin pin, bool active, uint64_t timestamp) override {
        console_info(TAG, "GPIO Interrupt: Pin %s is now %s at %llu", 
                     GpioPinToString(pin).data(), 
                     active ? "ACTIVE" : "INACTIVE", 
                     timestamp);
        
        // Handle specific pin interrupts
        switch (pin) {
            case GpioPin::GPIO_TMC_nFAULT_STATUS:
                if (!active) { // Fault is active low
                    console_error(TAG, "TMC9660 FAULT DETECTED!");
                    HandleTmcFault();
                }
                break;
                
            case GpioPin::GPIO_PCAL95555_INT:
                console_info(TAG, "PCAL95555 interrupt detected");
                HandlePcalInterrupt();
                break;
                
            case GpioPin::GPIO_ESP32_USER_BUTTON:
                console_info(TAG, "User button pressed");
                HandleUserButton();
                break;
                
            default:
                console_info(TAG, "Unhandled GPIO interrupt on pin %s", GpioPinToString(pin).data());
                break;
        }
    }

private:
    void HandleTmcFault() {
        console_error(TAG, "Handling TMC9660 fault...");
        
        // Try to clear the fault
        if (tmcController.ClearFault(Tmc9660ChipId::TMC9660_CHIP_1)) {
            console_info(TAG, "TMC9660 fault cleared successfully");
        } else {
            console_error(TAG, "Failed to clear TMC9660 fault");
        }
    }
    
    void HandlePcalInterrupt() {
        console_info(TAG, "Handling PCAL95555 interrupt...");
        
        // Read all PCAL95555 GPIO states
        GpioStateMap pcalStates;
        if (gpioManager.GetChipGpioStates(HardwareChipId::CHIP_PCAL95555, pcalStates)) {
            console_info(TAG, "PCAL95555 GPIO states:");
            for (const auto& [pin, state] : pcalStates) {
                console_info(TAG, "  %s: %s", GpioPinToString(pin).data(), state ? "ACTIVE" : "INACTIVE");
            }
        }
    }
    
    void HandleUserButton() {
        console_info(TAG, "Handling user button press...");
        
        // Toggle LED
        static bool ledState = false;
        ledState = !ledState;
        
        if (gpioManager.SetState(GpioPin::GPIO_ESP32_USER_LED, ledState)) {
            console_info(TAG, "LED toggled to %s", ledState ? "ON" : "OFF");
        }
    }
};

// Thread for GPIO monitoring
class GpioMonitorThread : public BaseThread {
public:
    GpioMonitorThread() : BaseThread("GpioMonitor", 4096, 5) {}
    
protected:
    void Run() override {
        console_info(TAG, "GPIO Monitor thread started");
        
        while (!ShouldStop()) {
            // Monitor GPIO states every 100ms
            MonitorGpioStates();
            os_delay_msec(100);
        }
        
        console_info(TAG, "GPIO Monitor thread stopped");
    }
    
private:
    void MonitorGpioStates() {
        // Monitor TMC9660 fault status
        if (gpioManager.IsActive(GpioPin::GPIO_TMC_nFAULT_STATUS)) {
            static uint64_t lastFaultTime = 0;
            uint64_t currentTime = os_get_time_msec();
            
            if (currentTime - lastFaultTime > 5000) { // Log every 5 seconds
                console_warning(TAG, "TMC9660 fault status is active");
                lastFaultTime = currentTime;
            }
        }
        
        // Monitor user button state
        static bool lastButtonState = false;
        bool currentButtonState = gpioManager.IsActive(GpioPin::GPIO_ESP32_USER_BUTTON);
        
        if (currentButtonState != lastButtonState) {
            console_info(TAG, "User button state changed to %s", currentButtonState ? "PRESSED" : "RELEASED");
            lastButtonState = currentButtonState;
        }
    }
};

// Main example class
class AdvancedGpioExample {
private:
    GpioInterruptHandler interruptHandler_;
    GpioMonitorThread monitorThread_;
    bool initialized_ = false;

public:
    bool Initialize() {
        console_info(TAG, "=== Advanced GPIO Manager Example ===");
        
        // Initialize all systems
        if (!InitializeSystems()) {
            console_error(TAG, "Failed to initialize systems");
            return false;
        }
        
        // Set up GPIO configurations
        if (!SetupGpioConfigurations()) {
            console_error(TAG, "Failed to setup GPIO configurations");
            return false;
        }
        
        // Set up interrupts
        if (!SetupInterrupts()) {
            console_error(TAG, "Failed to setup interrupts");
            return false;
        }
        
        // Start monitoring thread
        if (!monitorThread_.Start()) {
            console_error(TAG, "Failed to start GPIO monitor thread");
            return false;
        }
        
        initialized_ = true;
        console_info(TAG, "Advanced GPIO example initialized successfully");
        return true;
    }
    
    void Run() {
        if (!initialized_) {
            console_error(TAG, "Example not initialized");
            return;
        }
        
        console_info(TAG, "Starting advanced GPIO example...");
        
        // Run the main example sequence
        RunExampleSequence();
        
        // Keep running for a while to demonstrate features
        for (int i = 0; i < 30; i++) {
            os_delay_msec(1000);
            
            // Print system status every 10 seconds
            if (i % 10 == 0) {
                PrintSystemStatus();
            }
            
            // Run diagnostics every 15 seconds
            if (i % 15 == 0) {
                RunDiagnostics();
            }
        }
        
        console_info(TAG, "Advanced GPIO example completed");
    }
    
    void Cleanup() {
        console_info(TAG, "Cleaning up advanced GPIO example...");
        
        // Stop monitoring thread
        monitorThread_.Stop();
        monitorThread_.Join();
        
        // Disable interrupts
        gpioManager.DisableInterrupt(GpioPin::GPIO_TMC_nFAULT_STATUS);
        gpioManager.DisableInterrupt(GpioPin::GPIO_PCAL95555_INT);
        gpioManager.DisableInterrupt(GpioPin::GPIO_ESP32_USER_BUTTON);
        
        // Turn off all outputs
        gpioManager.SetInactive(GpioPin::GPIO_ESP32_USER_LED);
        gpioManager.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
        gpioManager.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_2);
        
        console_info(TAG, "Cleanup completed");
    }

private:
    bool InitializeSystems() {
        console_info(TAG, "Initializing systems...");
        
        // Initialize GPIO manager
        if (!gpioManager.EnsureInitialized()) {
            console_error(TAG, "Failed to initialize GPIO manager");
            return false;
        }
        
        // Initialize ADC manager
        if (!adcManager.IsInitialized()) {
            console_error(TAG, "ADC manager not initialized");
            return false;
        }
        
        // Initialize TMC9660 controller
        if (!tmcController.EnsureInitialized()) {
            console_error(TAG, "Failed to initialize TMC9660 controller");
            return false;
        }
        
        console_info(TAG, "All systems initialized successfully");
        return true;
    }
    
    bool SetupGpioConfigurations() {
        console_info(TAG, "Setting up GPIO configurations...");
        
        // Configure ESP32 GPIO pins
        GpioConfig esp32Config = {
            .direction = GpioDirection::GPIO_OUTPUT,
            .pullUpDown = GpioPullUpDown::GPIO_PULL_NONE,
            .driveStrength = GpioDriveStrength::GPIO_DRIVE_12MA
        };
        
        if (!gpioManager.ConfigurePin(GpioPin::GPIO_ESP32_USER_LED, esp32Config)) {
            console_error(TAG, "Failed to configure ESP32 LED pin");
            return false;
        }
        
        // Configure PCAL95555 GPIO pins
        GpioConfig pcalOutputConfig = {
            .direction = GpioDirection::GPIO_OUTPUT,
            .pullUpDown = GpioPullUpDown::GPIO_PULL_NONE,
            .driveStrength = GpioDriveStrength::GPIO_DRIVE_8MA
        };
        
        if (!gpioManager.ConfigurePin(GpioPin::GPIO_PCAL95555_OUTPUT_1, pcalOutputConfig)) {
            console_error(TAG, "Failed to configure PCAL95555 output 1");
            return false;
        }
        
        if (!gpioManager.ConfigurePin(GpioPin::GPIO_PCAL95555_OUTPUT_2, pcalOutputConfig)) {
            console_error(TAG, "Failed to configure PCAL95555 output 2");
            return false;
        }
        
        // Configure input pins
        GpioConfig inputConfig = {
            .direction = GpioDirection::GPIO_INPUT,
            .pullUpDown = GpioPullUpDown::GPIO_PULL_UP,
            .driveStrength = GpioDriveStrength::GPIO_DRIVE_4MA
        };
        
        if (!gpioManager.ConfigurePin(GpioPin::GPIO_ESP32_USER_BUTTON, inputConfig)) {
            console_error(TAG, "Failed to configure user button pin");
            return false;
        }
        
        if (!gpioManager.ConfigurePin(GpioPin::GPIO_TMC_nFAULT_STATUS, inputConfig)) {
            console_error(TAG, "Failed to configure TMC fault pin");
            return false;
        }
        
        console_info(TAG, "GPIO configurations completed");
        return true;
    }
    
    bool SetupInterrupts() {
        console_info(TAG, "Setting up GPIO interrupts...");
        
        // Set up interrupt for TMC9660 fault status
        if (!gpioManager.EnableInterrupt(GpioPin::GPIO_TMC_nFAULT_STATUS, 
                                       GpioInterruptTrigger::GPIO_INTR_NEGEDGE,
                                       &interruptHandler_)) {
            console_error(TAG, "Failed to enable TMC fault interrupt");
            return false;
        }
        
        // Set up interrupt for PCAL95555
        if (!gpioManager.EnableInterrupt(GpioPin::GPIO_PCAL95555_INT,
                                       GpioInterruptTrigger::GPIO_INTR_ANYEDGE,
                                       &interruptHandler_)) {
            console_error(TAG, "Failed to enable PCAL95555 interrupt");
            return false;
        }
        
        // Set up interrupt for user button
        if (!gpioManager.EnableInterrupt(GpioPin::GPIO_ESP32_USER_BUTTON,
                                       GpioInterruptTrigger::GPIO_INTR_NEGEDGE,
                                       &interruptHandler_)) {
            console_error(TAG, "Failed to enable user button interrupt");
            return false;
        }
        
        console_info(TAG, "GPIO interrupts configured successfully");
        return true;
    }
    
    void RunExampleSequence() {
        console_info(TAG, "Running example sequence...");
        
        // 1. Demonstrate basic GPIO operations
        DemonstrateBasicGpioOperations();
        
        // 2. Demonstrate batch operations
        DemonstrateBatchOperations();
        
        // 3. Demonstrate TMC9660 integration
        DemonstrateTmc9660Integration();
        
        // 4. Demonstrate PCAL95555 operations
        DemonstratePcal95555Operations();
        
        // 5. Demonstrate platform mapping
        DemonstratePlatformMapping();
        
        console_info(TAG, "Example sequence completed");
    }
    
    void DemonstrateBasicGpioOperations() {
        console_info(TAG, "--- Basic GPIO Operations ---");
        
        // Set and read ESP32 GPIO
        console_info(TAG, "Testing ESP32 GPIO operations...");
        
        if (gpioManager.SetActive(GpioPin::GPIO_ESP32_USER_LED)) {
            console_info(TAG, "ESP32 LED turned ON");
            
            if (gpioManager.IsActive(GpioPin::GPIO_ESP32_USER_LED)) {
                console_info(TAG, "ESP32 LED state confirmed as ACTIVE");
            }
        }
        
        os_delay_msec(1000);
        
        if (gpioManager.SetInactive(GpioPin::GPIO_ESP32_USER_LED)) {
            console_info(TAG, "ESP32 LED turned OFF");
        }
        
        // Read input pin
        bool buttonState = gpioManager.IsActive(GpioPin::GPIO_ESP32_USER_BUTTON);
        console_info(TAG, "User button state: %s", buttonState ? "PRESSED" : "RELEASED");
    }
    
    void DemonstrateBatchOperations() {
        console_info(TAG, "--- Batch GPIO Operations ---");
        
        // Create batch operation for multiple outputs
        GpioBatchOperation batch;
        
        // Add multiple operations to the batch
        batch.AddSetActive(GpioPin::GPIO_ESP32_USER_LED);
        batch.AddSetActive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
        batch.AddSetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_2);
        
        console_info(TAG, "Executing batch operation with %zu commands", batch.GetOperationCount());
        
        if (gpioManager.ExecuteBatch(batch)) {
            console_info(TAG, "Batch operation executed successfully");
        } else {
            console_error(TAG, "Batch operation failed");
        }
        
        os_delay_msec(2000);
        
        // Create another batch to reverse the states
        GpioBatchOperation reverseBatch;
        reverseBatch.AddSetInactive(GpioPin::GPIO_ESP32_USER_LED);
        reverseBatch.AddSetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
        reverseBatch.AddSetActive(GpioPin::GPIO_PCAL95555_OUTPUT_2);
        
        if (gpioManager.ExecuteBatch(reverseBatch)) {
            console_info(TAG, "Reverse batch operation executed successfully");
        }
    }
    
    void DemonstrateTmc9660Integration() {
        console_info(TAG, "--- TMC9660 Integration ---");
        
        // Initialize TMC9660 chip
        if (!InitializePrimaryTmc9660()) {
            console_error(TAG, "Failed to initialize TMC9660");
            return;
        }
        
        // Test TMC9660 communication
        if (tmcController.TestCommunication()) {
            console_info(TAG, "TMC9660 communication test passed");
        } else {
            console_error(TAG, "TMC9660 communication test failed");
        }
        
        // Enable TMC9660 driver
        if (tmcController.EnableDriver(Tmc9660ChipId::TMC9660_CHIP_1)) {
            console_info(TAG, "TMC9660 driver enabled");
        }
        
        // Read TMC9660 ADC values
        uint32_t adcValue;
        if (tmcController.ReadAdcValue(Tmc9660ChipId::TMC9660_CHIP_1, 1, adcValue)) {
            console_info(TAG, "TMC9660 AIN1 ADC value: %lu", adcValue);
        }
        
        float voltage;
        if (tmcController.ReadAdcVoltage(Tmc9660ChipId::TMC9660_CHIP_1, 1, voltage)) {
            console_info(TAG, "TMC9660 AIN1 voltage: %.3fV", voltage);
        }
        
        // Test TMC9660 GPIO
        if (tmcController.SetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 17, true)) {
            console_info(TAG, "TMC9660 GPIO17 set to ACTIVE");
        }
        
        if (tmcController.GetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 17)) {
            console_info(TAG, "TMC9660 GPIO17 state confirmed as ACTIVE");
        }
    }
    
    void DemonstratePcal95555Operations() {
        console_info(TAG, "--- PCAL95555 Operations ---");
        
        // Get all PCAL95555 GPIO states
        GpioStateMap pcalStates;
        if (gpioManager.GetChipGpioStates(HardwareChipId::CHIP_PCAL95555, pcalStates)) {
            console_info(TAG, "PCAL95555 GPIO states:");
            for (const auto& [pin, state] : pcalStates) {
                console_info(TAG, "  %s: %s", GpioPinToString(pin).data(), state ? "ACTIVE" : "INACTIVE");
            }
        }
        
        // Test PCAL95555 outputs
        console_info(TAG, "Testing PCAL95555 outputs...");
        
        // Blink PCAL95555 outputs
        for (int i = 0; i < 3; i++) {
            gpioManager.SetActive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
            gpioManager.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_2);
            os_delay_msec(500);
            
            gpioManager.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
            gpioManager.SetActive(GpioPin::GPIO_PCAL95555_OUTPUT_2);
            os_delay_msec(500);
        }
        
        // Set both outputs to inactive
        gpioManager.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
        gpioManager.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_2);
    }
    
    void DemonstratePlatformMapping() {
        console_info(TAG, "--- Platform Mapping ---");
        
        // Demonstrate functional pin usage
        console_info(TAG, "Testing functional pin mapping...");
        
        // Get hardware resource for a functional pin
        HardwareResource hwResource;
        if (gpioManager.GetHardwareResource(GpioPin::GPIO_ESP32_USER_LED, hwResource)) {
            console_info(TAG, "ESP32 LED hardware resource:");
            console_info(TAG, "  Chip: %s", HardwareChipIdToString(hwResource.chipId).data());
            console_info(TAG, "  Pin: %d", hwResource.pinNumber);
            console_info(TAG, "  Direction: %s", GpioDirectionToString(hwResource.direction).data());
        }
        
        // Test pin registration conflict detection
        console_info(TAG, "Testing pin registration conflict detection...");
        
        GpioConfig testConfig = {
            .direction = GpioDirection::GPIO_OUTPUT,
            .pullUpDown = GpioPullUpDown::GPIO_PULL_NONE,
            .driveStrength = GpioDriveStrength::GPIO_DRIVE_8MA
        };
        
        // Try to register a pin that's already registered
        if (!gpioManager.RegisterPin(GpioPin::GPIO_ESP32_USER_LED, testConfig)) {
            console_info(TAG, "Pin registration conflict detected (expected)");
        }
        
        // Test invalid pin registration
        if (!gpioManager.RegisterPin(static_cast<GpioPin>(999), testConfig)) {
            console_info(TAG, "Invalid pin registration rejected (expected)");
        }
    }
    
    void PrintSystemStatus() {
        console_info(TAG, "--- System Status ---");
        
        // Print GPIO manager status
        GpioManagerStatus gpioStatus;
        if (gpioManager.GetStatus(gpioStatus)) {
            console_info(TAG, "GPIO Manager Status:");
            console_info(TAG, "  Initialized: %s", gpioStatus.isInitialized ? "Yes" : "No");
            console_info(TAG, "  Registered pins: %d", gpioStatus.registeredPinCount);
            console_info(TAG, "  Active interrupts: %d", gpioStatus.activeInterruptCount);
            console_info(TAG, "  Total operations: %lu", gpioStatus.totalOperations);
            console_info(TAG, "  Error count: %lu", gpioStatus.errorCount);
        }
        
        // Print TMC9660 status
        tmcController.PrintSystemStatus();
        
        // Print chip-specific status
        for (auto chipId : {HardwareChipId::CHIP_ESP32, HardwareChipId::CHIP_PCAL95555, HardwareChipId::CHIP_TMC9660}) {
            ChipGpioStatus chipStatus;
            if (gpioManager.GetChipStatus(chipId, chipStatus)) {
                console_info(TAG, "%s Status:", HardwareChipIdToString(chipId).data());
                console_info(TAG, "  Registered pins: %d", chipStatus.registeredPinCount);
                console_info(TAG, "  Active outputs: %d", chipStatus.activeOutputCount);
                console_info(TAG, "  Active inputs: %d", chipStatus.activeInputCount);
                console_info(TAG, "  Interrupt count: %lu", chipStatus.interruptCount);
            }
        }
    }
    
    void RunDiagnostics() {
        console_info(TAG, "--- Running Diagnostics ---");
        
        // Run GPIO manager diagnostics
        GpioDiagnostics diagnostics;
        if (gpioManager.RunDiagnostics(diagnostics)) {
            console_info(TAG, "GPIO Diagnostics:");
            console_info(TAG, "  System healthy: %s", diagnostics.isSystemHealthy ? "Yes" : "No");
            console_info(TAG, "  All chips responding: %s", diagnostics.allChipsResponding ? "Yes" : "No");
            console_info(TAG, "  Interrupt system working: %s", diagnostics.interruptSystemWorking ? "Yes" : "No");
            
            if (!diagnostics.isSystemHealthy) {
                console_error(TAG, "GPIO system has issues!");
                for (const auto& issue : diagnostics.issues) {
                    console_error(TAG, "  Issue: %s", issue.c_str());
                }
            }
        }
        
        // Run TMC9660 diagnostics
        if (tmcController.RunDiagnostics()) {
            console_info(TAG, "TMC9660 diagnostics passed");
        } else {
            console_error(TAG, "TMC9660 diagnostics failed");
        }
        
        // Check system health
        if (gpioManager.GetSystemHealth() && tmcController.GetSystemHealth()) {
            console_info(TAG, "Overall system health: GOOD");
        } else {
            console_error(TAG, "Overall system health: POOR");
        }
    }
};

// Main function
extern "C" void app_main() {
    console_info(TAG, "Starting Advanced GPIO Manager Example");
    
    AdvancedGpioExample example;
    
    if (example.Initialize()) {
        example.Run();
        example.Cleanup();
    } else {
        console_error(TAG, "Failed to initialize example");
    }
    
    console_info(TAG, "Advanced GPIO Manager Example completed");
} 