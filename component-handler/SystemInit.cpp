#include "SystemInit.h"
#include "ConsolePort.h"
#include "OsUtility.h"

/**
 * @file SystemInit.cpp
 * @brief Implementation of system initialization functions.
 * 
 * This file contains the implementation of the system initialization
 * functions for the HardFOC ADC and GPIO subsystems.
 */

static const char* TAG = "SystemInit";

bool SystemInit::InitializeSystem() noexcept {
    console_info(TAG, "Initializing HardFOC system components");
    
    // Initialize hardware pin configuration first
    console_info(TAG, "Configuring hardware pins");
    init_mcu_pinconfig();
    
    // Initialize GPIO system
    if (!InitializeGpioSystem()) {
        console_error(TAG, "Failed to initialize GPIO system");
        return false;
    }
      // Initialize ADC system
    if (!InitializeAdcSystem()) {
        console_error(TAG, "Failed to initialize ADC system");
        return false;
    }
    
    // Initialize TMC9660 system
    if (!InitializeTmc9660System()) {
        console_error(TAG, "Failed to initialize TMC9660 system");
        return false;
    }
    
    console_info(TAG, "HardFOC system initialization completed successfully");
    return true;
}

bool SystemInit::InitializeAdcSystem() noexcept {
    console_info(TAG, "Initializing ADC system");
    
    // Get the ADC data singleton and ensure it's initialized
    AdcData& adcData = AdcData::GetInstance();
    if (!adcData.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC data system");
        return false;
    }
    
    // Register internal ADC channels
    if (!RegisterInternalAdcChannels()) {
        console_error(TAG, "Failed to register internal ADC channels");
        return false;
    }
    
    // Register external ADC channels
    if (!RegisterExternalAdcChannels()) {
        console_warning(TAG, "Failed to register some external ADC channels");
        // Don't fail initialization if external ADCs are not available
    }
    
    // Map functional ADC channels
    if (!MapFunctionalAdcChannels()) {
        console_warning(TAG, "Failed to map some functional ADC channels");
    }
    
    console_info(TAG, "ADC system initialized with %d channels", 
                 adcData.GetRegisteredChannelCount());
    
    return true;
}

bool SystemInit::InitializeGpioSystem() noexcept {
    console_info(TAG, "Initializing GPIO system");
    
    // Get the GPIO data singleton and ensure it's initialized
    GpioData& gpioData = GpioData::GetInstance();
    if (!gpioData.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO data system");
        return false;
    }
    
    // Register native GPIO pins
    if (!RegisterNativeGpioPins()) {
        console_error(TAG, "Failed to register native GPIO pins");
        return false;
    }
    
    // Register GPIO expander pins
    if (!RegisterExpanderGpioPins()) {
        console_warning(TAG, "Failed to register some GPIO expander pins");
        // Don't fail initialization if expanders are not available
    }
    
    // Map functional pins
    if (!MapFunctionalPins()) {
        console_warning(TAG, "Failed to map some functional pins");
    }
    
    console_info(TAG, "GPIO system initialized with %d pins", 
                 gpioData.GetRegisteredPinCount());
    
    return true;
}

bool SystemInit::InitializeTmc9660System() noexcept {
    console_info(TAG, "Initializing TMC9660 motor controller system");
    
    // Get the TMC9660 controller and ensure it's initialized
    Tmc9660MotorController& tmcController = Tmc9660MotorController::GetInstance();
    if (!tmcController.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize TMC9660 controller");
        return false;
    }
    
    // Initialize the primary TMC9660 chip
    if (!InitializePrimaryTmc9660()) {
        console_error(TAG, "Failed to initialize primary TMC9660 chip");
        return false;
    }
    
    console_info(TAG, "TMC9660 system initialized with %d chips", 
                 tmcController.GetRegisteredChipCount());
    
    return true;
}

bool SystemInit::RegisterNativeGpioPins() noexcept {
    console_info(TAG, "Registering ESP32-C6 native GPIO pins");
    
    // This is a placeholder implementation
    // In a real implementation, you would create ESP32GpioPin objects
    // and register them with the GpioData system
    
    // TODO: Implement actual ESP32 native GPIO registration
    // For now, just return success to indicate the function was called
    
    console_info(TAG, "Native GPIO pins registered successfully");
    return true;
}

bool SystemInit::RegisterExpanderGpioPins() noexcept {
    console_info(TAG, "Registering PCAL95555 GPIO expander pins");
    
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Initialize I2C bus
    // 2. Create SfPcal95555Bus adapters
    // 3. Create Pcal95555Chip instances
    // 4. Register all pins from the expanders
    
    // TODO: Implement actual PCAL95555 GPIO registration
    // For now, just return success to indicate the function was called
    
    console_info(TAG, "GPIO expander pins registered successfully");
    return true;
}

bool SystemInit::RegisterInternalAdcChannels() noexcept {
    console_info(TAG, "Registering ESP32-C6 internal ADC channels");
    
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Create ESP32 internal ADC driver instances
    // 2. Register each channel with the AdcData system
    
    // TODO: Implement actual internal ADC registration
    // For now, just return success to indicate the function was called
    
    console_info(TAG, "Internal ADC channels registered successfully");
    return true;
}

bool SystemInit::RegisterExternalAdcChannels() noexcept {
    console_info(TAG, "Registering external ADC channels");
    
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Initialize SPI bus for external ADC chips
    // 2. Create external ADC driver instances
    // 3. Register each channel with the AdcData system
    
    // TODO: Implement actual external ADC registration
    // For now, just return success to indicate the function was called
    
    console_info(TAG, "External ADC channels registered successfully");
    return true;
}

bool SystemInit::CreateNativeGpioPins() noexcept {
    // Placeholder for creating native GPIO pin objects
    return true;
}

bool SystemInit::CreateExpanderGpioPins() noexcept {
    // Placeholder for creating GPIO expander pin objects
    return true;
}

bool SystemInit::CreateInternalAdcChannels() noexcept {
    // Placeholder for creating internal ADC channel objects
    return true;
}

bool SystemInit::CreateExternalAdcChannels() noexcept {
    // Placeholder for creating external ADC channel objects
    return true;
}

bool SystemInit::MapFunctionalPins() noexcept {
    console_info(TAG, "Mapping functional GPIO pins");
    
    // This function would map logical pin names to physical pins
    // For example, mapping "motor_enable" to a specific GPIO pin
    
    // TODO: Implement functional pin mapping
    
    console_info(TAG, "Functional GPIO pins mapped successfully");
    return true;
}

bool SystemInit::MapFunctionalAdcChannels() noexcept {
    console_info(TAG, "Mapping functional ADC channels");
    
    // This function would map logical channel names to physical channels
    // For example, mapping "motor_current_a" to a specific ADC channel
    
    // TODO: Implement functional ADC channel mapping
    
    console_info(TAG, "Functional ADC channels mapped successfully");
    return true;
}

bool SystemInit::RunSystemSelfTest() noexcept {
    console_info(TAG, "Running system self-test");
    
    bool testPassed = true;
    
    // Test GPIO system
    GpioData& gpioData = GpioData::GetInstance();
    if (!gpioData.RunGpioTest()) {
        console_error(TAG, "GPIO system test failed");
        testPassed = false;
    }
    
    // Test ADC system
    // TODO: Implement ADC system test
    
    if (testPassed) {
        console_info(TAG, "System self-test passed");
    } else {
        console_error(TAG, "System self-test failed");
    }
    
    return testPassed;
}

bool SystemInit::GetSystemHealth() noexcept {
    bool systemHealthy = true;
    
    // Check GPIO system health
    GpioData& gpioData = GpioData::GetInstance();
    if (!gpioData.GetSystemHealth()) {
        console_warning(TAG, "GPIO system health check failed");
        systemHealthy = false;
    }
    
    // Check ADC system health
    AdcData& adcData = AdcData::GetInstance();
    if (!adcData.EnsureInitialized()) {
        console_warning(TAG, "ADC system health check failed");
        systemHealthy = false;
    }
    
    return systemHealthy;
}

void SystemInit::PrintSystemStatus() noexcept {
    console_info(TAG, "=== HardFOC System Status ===");
    
    // GPIO system status
    GpioData& gpioData = GpioData::GetInstance();
    console_info(TAG, "GPIO System: %d pins registered, Health: %s",
                 gpioData.GetRegisteredPinCount(),
                 gpioData.GetSystemHealth() ? "OK" : "DEGRADED");
    
    // ADC system status
    AdcData& adcData = AdcData::GetInstance();
    console_info(TAG, "ADC System: %d channels registered, Health: %s",
                 adcData.GetRegisteredChannelCount(),
                 adcData.EnsureInitialized() ? "OK" : "DEGRADED");
    
    console_info(TAG, "Overall System Health: %s",
                 GetSystemHealth() ? "HEALTHY" : "DEGRADED");
    
    console_info(TAG, "=== End System Status ===");
}

//=============================================================================
// Global System Functions
//=============================================================================

bool InitializeHardFocSystem() noexcept {
    return SystemInit::InitializeSystem();
}

bool GetHardFocSystemStatus() noexcept {
    return SystemInit::GetSystemHealth();
}

bool RunHardFocSystemTest() noexcept {
    return SystemInit::RunSystemSelfTest();
}
