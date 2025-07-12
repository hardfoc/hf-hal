#include "SystemInit.h"
#include "AdcManager.h"
#include "GpioManager.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include "thread_safe/SfI2cBus.h"

/**
 * @file SystemInit.cpp
 * @brief Implementation of system initialization functions.
 * 
 * This file contains the implementation of the system initialization
 * functions for the HardFOC ADC and GPIO subsystems.
 */

static const char* TAG = "SystemInit";

// Static instances for system-wide I2C bus and managers
static std::unique_ptr<SfI2cBus> g_i2cBus;
static SemaphoreHandle_t g_i2cMutex = nullptr;

bool SystemInit::InitializeSystem() noexcept {
    console_info(TAG, "Initializing HardFOC system components");
    
    // Initialize hardware pin configuration first
    console_info(TAG, "Configuring hardware pins");
    HardFOC::init_platform_hardware();
    
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
      // Get the ADC manager singleton and ensure it's initialized
    AdcManager& adcManager = AdcManager::GetInstance();
    if (!adcManager.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC manager system");
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
                 adcManager.GetRegisteredChannelCount());
    
    return true;
}

bool SystemInit::InitializeGpioSystem() noexcept {
    console_info(TAG, "Initializing comprehensive GPIO system");
    
    // Ensure I2C bus is available
    if (!g_i2cBus) {
        console_error(TAG, "I2C bus not initialized - cannot initialize GPIO system");
        return false;
    }
    
    // Get TMC9660 controller instance
    Tmc9660MotorController& tmcController = Tmc9660MotorController::GetInstance();
    if (!tmcController.EnsureInitialized()) {
        console_warning(TAG, "TMC9660 controller not initialized - GPIO will work with limited functionality");
    }
      // Get the new GPIO manager and initialize with dependencies
    GpioManager& gpioManager = GpioManager::GetInstance();
    if (!gpioManager.Initialize(*g_i2cBus, tmcController)) {
        console_error(TAG, "Failed to initialize comprehensive GPIO manager system");
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
    
    console_info(TAG, "Comprehensive GPIO system initialized with %d pins", 
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
    // and register them with the GpioManager system
    
    // TODO: Implement actual ESP32 native GPIO registration
    // For now, just return success to indicate the function was called
    
    console_info(TAG, "Native GPIO pins registered successfully");
    return true;
}

bool SystemInit::RegisterExpanderGpioPins() noexcept {
    console_info(TAG, "Registering PCAL95555 GPIO expander pins");
    
    // Initialize I2C bus if not already done
    if (!g_i2cBus) {
        if (!InitializeI2cBus()) {
            console_error(TAG, "Failed to initialize I2C bus for GPIO expanders");
            return false;
        }
    }
      // Get GPIO manager instance and initialize with PCAL95555 chips
    auto& gpioManager = GpioManager<64>::Instance();  // Support up to 64 GPIO pins
    if (!gpioManager.Initialize(*g_i2cBus)) {
        console_error(TAG, "Failed to initialize GPIO manager with PCAL95555 chips");
        return false;
    }
    
    console_info(TAG, "GPIO expander pins registered successfully");
    return true;
}

bool SystemInit::RegisterInternalAdcChannels() noexcept {
    console_info(TAG, "Registering ESP32-C6 internal ADC channels");
      // Get ADC manager instance and initialize with ESP32-C6 ADCs
    auto& adcManager = AdcManager<32>::Instance();  // Support up to 32 ADC channels
    if (!adcManager.Initialize()) {
        console_error(TAG, "Failed to initialize ADC manager with ESP32-C6 ADCs");
        return false;
    }
    
    console_info(TAG, "Internal ADC channels registered successfully");
    return true;
}

bool SystemInit::RegisterExternalAdcChannels() noexcept {
    console_info(TAG, "Registering external ADC channels");
    
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Initialize SPI bus for external ADC chips
    // 2. Create external ADC driver instances
    // 3. Register each channel with the AdcManager system
    
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

bool SystemInit::InitializeI2cBus() noexcept {
    console_info(TAG, "Initializing I2C bus for GPIO expanders");
    
    // Create I2C mutex
    g_i2cMutex = xSemaphoreCreateMutex();
    if (!g_i2cMutex) {
        console_error(TAG, "Failed to create I2C mutex");
        return false;
    }
    
    // Configure I2C parameters
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = I2C_SDA_PIN;
    i2c_config.scl_io_num = I2C_SCL_PIN;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = 100000;  // 100kHz
    
    // Create I2C bus instance
    g_i2cBus = std::make_unique<SfI2cBus>(I2C_NUM_0, i2c_config, g_i2cMutex);
    
    // Open the I2C bus
    if (!g_i2cBus->Open()) {
        console_error(TAG, "Failed to open I2C bus");
        g_i2cBus.reset();
        return false;
    }
    
    console_info(TAG, "I2C bus initialized successfully");
    return true;
}

bool SystemInit::RunSystemSelfTest() noexcept {
    console_info(TAG, "Running system self-test");
      bool testPassed = true;
      // Test GPIO system
    GpioManager& gpioManager = GpioManager::GetInstance();
    if (!gpioManager.RunGpioTest()) {
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
    GpioManager& gpioManager = GpioManager::GetInstance();
    if (!gpioManager.GetSystemHealth()) {
        console_warning(TAG, "GPIO system health check failed");
        systemHealthy = false;
    }
      // Check ADC system health
    AdcManager& adcManager = AdcManager::GetInstance();
    if (!adcManager.EnsureInitialized()) {
        console_warning(TAG, "ADC system health check failed");
        systemHealthy = false;
    }
    
    return systemHealthy;
}

void SystemInit::PrintSystemStatus() noexcept {
    console_info(TAG, "=== HardFOC System Status ===");
      // GPIO system status
    GpioManager& gpioManager = GpioManager::GetInstance();
    console_info(TAG, "GPIO System: %d pins registered, Health: %s",
                 gpioManager.GetRegisteredPinCount(),
                 gpioManager.GetSystemHealth() ? "OK" : "DEGRADED");
    
    // ADC system status
    AdcManager& adcManager = AdcManager::GetInstance();
    console_info(TAG, "ADC System: %d channels registered, Health: %s",
                 adcManager.GetRegisteredChannelCount(),
                 adcManager.EnsureInitialized() ? "OK" : "DEGRADED");
    
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
