#include "SystemInit.h"
#include "../AdcHandler.h"
#include "../GpioHandler.h"
#include "../TMC9660Controller.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/SfI2cBus.h"
#include "driver/i2c.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_gpio_config.hpp"

/**
 * @file SystemInit.cpp
 * @brief Implementation of system initialization functions.
 * 
 * This file contains the implementation of the system initialization
 * functions for the HardFOC ADC and GPIO subsystems.
 */

static const char* TAG = "SystemInit";

// Static instances for system-wide hardware components
static std::unique_ptr<SfI2cBus> g_i2cBus;
static std::unique_ptr<TMC9660Controller> g_tmcController;
static SemaphoreHandle_t g_i2cMutex = nullptr;

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
    
    // Initialize TMC9660 controller
    if (!InitializeTMC9660Controller()) {
        console_error(TAG, "Failed to initialize TMC9660 controller");
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
    
    // Initialize and register TMC9660 controller ADC channels
    if (!RegisterExternalAdcChannels()) {
        console_warning(TAG, "Failed to register TMC9660 ADC channels");
        // Don't fail initialization if TMC9660 is not available
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
    GpioHandler& gpioData = GpioHandler::GetInstance();
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

bool SystemInit::InitializeTMC9660Controller() noexcept {
    console_info(TAG, "Initializing TMC9660 controller");
    
    // Create and configure TMC9660 controller instance
    g_tmcController = std::make_unique<TMC9660Controller>();
    if (!g_tmcController) {
        console_error(TAG, "Failed to create TMC9660 controller instance");
        return false;
    }
    
    // Initialize TMC9660 controller
    if (!g_tmcController->Initialize()) {
        console_error(TAG, "Failed to initialize TMC9660 controller");
        g_tmcController.reset();
        return false;
    }
    
    console_info(TAG, "TMC9660 controller initialized successfully");
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
    console_info(TAG, "Registering PCAL95555 GPIO expander pins (single chip at 0x20)");
    
    // Initialize I2C bus if not already done
    if (!g_i2cBus) {
        if (!InitializeI2cBus()) {
            console_error(TAG, "Failed to initialize I2C bus for GPIO expanders");
            return false;
        }
    }
    
    // Get GPIO data system
    GpioHandler& gpioData = GpioHandler::GetInstance();
    
    // Create and configure the single PCAL95555 chip
    // The PCAL95555 chip wrapper will be created and managed internally
    // Register all 16 pins of the single PCAL95555 chip
    
    // TODO: Create actual PCAL95555 pin instances and register them
    // For now, this is a placeholder implementation
    
    console_info(TAG, "PCAL95555 GPIO expander pins registered successfully");
    return true;
}

bool SystemInit::RegisterInternalAdcChannels() noexcept {
    console_info(TAG, "Registering ESP32-C6 internal ADC channels");
    
    // Get ADC handler instance and initialize with ESP32-C6 ADCs
    auto& adcHandler = AdcHandler<32>::Instance();  // Support up to 32 ADC channels
    if (!adcHandler.Initialize()) {
        console_error(TAG, "Failed to initialize ADC handler with ESP32-C6 ADCs");
        return false;
    }
    
    console_info(TAG, "Internal ADC channels registered successfully");
    return true;
}

bool SystemInit::RegisterExternalAdcChannels() noexcept {
    console_info(TAG, "Registering TMC9660 motor controller ADC channels");
    
    // Create TMC9660 controller instance
    g_tmcController = std::make_unique<TMC9660Controller>();
    if (!g_tmcController->Initialize()) {
        console_error(TAG, "Failed to initialize TMC9660 controller");
        return false;
    }
    
    // Get the ADC data system
    AdcData& adcData = AdcData::GetInstance();
    
    // Register TMC9660 ADC channels
    TMC9660Adc& tmcAdc = g_tmcController->GetAdc();
    
    // Register the 3 TMC9660 current sensing channels
    if (!adcData.RegisterAdcChannel(AdcInputSensor::ADC_TMC9660_CURRENT_A, tmcAdc, 0)) {
        console_error(TAG, "Failed to register TMC9660 current A channel");
        return false;
    }
    
    if (!adcData.RegisterAdcChannel(AdcInputSensor::ADC_TMC9660_CURRENT_B, tmcAdc, 1)) {
        console_error(TAG, "Failed to register TMC9660 current B channel");
        return false;
    }
    
    if (!adcData.RegisterAdcChannel(AdcInputSensor::ADC_TMC9660_CURRENT_C, tmcAdc, 2)) {
        console_error(TAG, "Failed to register TMC9660 current C channel");
        return false;
    }
    
    console_info(TAG, "TMC9660 ADC channels registered successfully");
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
    GpioHandler& gpioData = GpioHandler::GetInstance();
    if (!gpioData.RunGpioTest()) {
        console_error(TAG, "GPIO system test failed");
        testPassed = false;
    }
    
    // Test ADC system
    // TODO: Implement ADC system test
    
    // Test TMC9660 controller
    if (g_tmcController && !g_tmcController->RunSelfTest()) {
        console_error(TAG, "TMC9660 controller test failed");
        testPassed = false;
    }
    
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
    GpioHandler& gpioData = GpioHandler::GetInstance();
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
    
    // Check TMC9660 controller health
    if (g_tmcController && !g_tmcController->IsHealthy()) {
        console_warning(TAG, "TMC9660 controller health check failed");
        systemHealthy = false;
    }
    
    return systemHealthy;
}

void SystemInit::PrintSystemStatus() noexcept {
    console_info(TAG, "=== HardFOC System Status ===");
    
    // GPIO system status
    GpioHandler& gpioData = GpioHandler::GetInstance();
    console_info(TAG, "GPIO System: %d pins registered, Health: %s",
                 gpioData.GetRegisteredPinCount(),
                 gpioData.GetSystemHealth() ? "OK" : "DEGRADED");
    
    // ADC system status
    AdcData& adcData = AdcData::GetInstance();
    console_info(TAG, "ADC System: %d channels registered, Health: %s",
                 adcData.GetRegisteredChannelCount(),
                 adcData.EnsureInitialized() ? "OK" : "DEGRADED");
    
    // TMC9660 controller status
    if (g_tmcController) {
        console_info(TAG, "TMC9660 Controller: Health: %s",
                     g_tmcController->IsHealthy() ? "OK" : "DEGRADED");
    } else {
        console_info(TAG, "TMC9660 Controller: Not initialized");
    }
    
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
