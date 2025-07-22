/**
 * @file Tmc9660HandlerUsageExample.cpp
 * @brief Example showing how to use the updated TMC9660Handler with BaseSpi/BaseUart interfaces.
 * 
 * This example demonstrates the new architecture where TMC9660Handler accepts BaseSpi and/or BaseUart
 * interfaces and creates the appropriate TMC9660CommInterface internally.
 *
 * @author HardFOC Team
 * @date 2025
 */

#include "utils-and-drivers/driver-handlers/Tmc9660Handler.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspUart.h"
#include "CommChannelsManager.h"
#include <iostream>

static const char* TAG = "Tmc9660HandlerExample";

/**
 * @brief Example 1: TMC9660Handler with SPI interface only
 */
void ExampleSpiOnly() {
    std::cout << "\n=== TMC9660Handler SPI-Only Example ===" << std::endl;
    
    // Get SPI interface from CommChannelsManager
    CommChannelsManager& commManager = CommChannelsManager::GetInstance();
    if (!commManager.Initialize()) {
        std::cout << "Failed to initialize CommChannelsManager" << std::endl;
        return;
    }
    
    // Get SPI device for TMC9660 (assuming device index 0)
    BaseSpi* spiDevice = commManager.GetSpiDevice(0);
    if (!spiDevice) {
        std::cout << "Failed to get SPI device" << std::endl;
        return;
    }
    
    // Create TMC9660Handler with SPI interface
    constexpr uint8_t TMC9660_ADDRESS = 0x01;
    Tmc9660Handler tmc9660Handler(*spiDevice, TMC9660_ADDRESS);
    
    // Initialize the handler
    if (!tmc9660Handler.Initialize()) {
        std::cout << "Failed to initialize TMC9660Handler" << std::endl;
        return;
    }
    
    std::cout << "TMC9660Handler initialized successfully!" << std::endl;
    std::cout << "Communication mode: " << (tmc9660Handler.GetCommMode() == CommMode::SPI ? "SPI" : "UART") << std::endl;
    std::cout << "Has SPI interface: " << (tmc9660Handler.HasSpiInterface() ? "Yes" : "No") << std::endl;
    std::cout << "Has UART interface: " << (tmc9660Handler.HasUartInterface() ? "Yes" : "No") << std::endl;
    
    // Access GPIO 17
    try {
        auto& gpio17 = tmc9660Handler.gpio(17);
        if (gpio17.Initialize()) {
            std::cout << "GPIO17 initialized: " << gpio17.GetDescription() << std::endl;
            
            // Set GPIO17 active
            if (gpio17.SetActive() == hf_gpio_err_t::GPIO_SUCCESS) {
                std::cout << "GPIO17 set to active" << std::endl;
            }
            
            // Toggle GPIO17
            if (gpio17.Toggle() == hf_gpio_err_t::GPIO_SUCCESS) {
                std::cout << "GPIO17 toggled" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cout << "GPIO error: " << e.what() << std::endl;
    }
    
    // Access ADC
    auto& adc = tmc9660Handler.adc();
    if (adc.Initialize()) {
        std::cout << "ADC initialized, max channels: " << static_cast<int>(adc.GetMaxChannels()) << std::endl;
        
        // Read ADC channel 0
        float voltage = 0.0f;
        if (adc.ReadChannelV(0, voltage) == hf_adc_err_t::ADC_SUCCESS) {
            std::cout << "ADC Channel 0 voltage: " << voltage << "V" << std::endl;
        }
    }
}

/**
 * @brief Example 2: TMC9660Handler with UART interface only
 */
void ExampleUartOnly() {
    std::cout << "\n=== TMC9660Handler UART-Only Example ===" << std::endl;
    
    // Get UART interface from CommChannelsManager
    CommChannelsManager& commManager = CommChannelsManager::GetInstance();
    
    // Get UART device (assuming index 0)
    BaseUart& uartDevice = commManager.GetUart(0);
    
    // Create TMC9660Handler with UART interface
    constexpr uint8_t TMC9660_ADDRESS = 0x01;
    Tmc9660Handler tmc9660Handler(uartDevice, TMC9660_ADDRESS);
    
    // Initialize the handler
    if (!tmc9660Handler.Initialize()) {
        std::cout << "Failed to initialize TMC9660Handler" << std::endl;
        return;
    }
    
    std::cout << "TMC9660Handler initialized successfully!" << std::endl;
    std::cout << "Communication mode: " << (tmc9660Handler.GetCommMode() == CommMode::SPI ? "SPI" : "UART") << std::endl;
    std::cout << "Has SPI interface: " << (tmc9660Handler.HasSpiInterface() ? "Yes" : "No") << std::endl;
    std::cout << "Has UART interface: " << (tmc9660Handler.HasUartInterface() ? "Yes" : "No") << std::endl;
}

/**
 * @brief Example 3: TMC9660Handler with both SPI and UART interfaces
 */
void ExampleDualInterface() {
    std::cout << "\n=== TMC9660Handler Dual Interface Example ===" << std::endl;
    
    // Get communication interfaces from CommChannelsManager
    CommChannelsManager& commManager = CommChannelsManager::GetInstance();
    
    BaseSpi* spiDevice = commManager.GetSpiDevice(0);
    BaseUart& uartDevice = commManager.GetUart(0);
    
    if (!spiDevice) {
        std::cout << "Failed to get SPI device" << std::endl;
        return;
    }
    
    // Create TMC9660Handler with both interfaces
    constexpr uint8_t TMC9660_ADDRESS = 0x01;
    Tmc9660Handler tmc9660Handler(*spiDevice, uartDevice, TMC9660_ADDRESS);
    
    // Initialize the handler
    if (!tmc9660Handler.Initialize()) {
        std::cout << "Failed to initialize TMC9660Handler" << std::endl;
        return;
    }
    
    std::cout << "TMC9660Handler initialized successfully!" << std::endl;
    std::cout << "Communication mode: " << (tmc9660Handler.GetCommMode() == CommMode::SPI ? "SPI" : "UART") << std::endl;
    std::cout << "Has SPI interface: " << (tmc9660Handler.HasSpiInterface() ? "Yes" : "No") << std::endl;
    std::cout << "Has UART interface: " << (tmc9660Handler.HasUartInterface() ? "Yes" : "No") << std::endl;
    
    // Demonstrate interface switching
    std::cout << "\nSwitching to UART interface..." << std::endl;
    if (tmc9660Handler.SwitchCommInterface(CommMode::UART)) {
        std::cout << "Successfully switched to UART" << std::endl;
        std::cout << "Current communication mode: " << (tmc9660Handler.GetCommMode() == CommMode::SPI ? "SPI" : "UART") << std::endl;
    } else {
        std::cout << "Failed to switch to UART" << std::endl;
    }
    
    std::cout << "\nSwitching back to SPI interface..." << std::endl;
    if (tmc9660Handler.SwitchCommInterface(CommMode::SPI)) {
        std::cout << "Successfully switched to SPI" << std::endl;
        std::cout << "Current communication mode: " << (tmc9660Handler.GetCommMode() == CommMode::SPI ? "SPI" : "UART") << std::endl;
    } else {
        std::cout << "Failed to switch to SPI" << std::endl;
    }
}

/**
 * @brief Example 4: Custom bootloader configuration
 */
void ExampleCustomBootConfig() {
    std::cout << "\n=== TMC9660Handler Custom Boot Config Example ===" << std::endl;
    
    // Define custom bootloader configuration
    static const tmc9660::BootloaderConfig customBootConfig = {
        .spiFrequency = 5000000,  // 5 MHz instead of default 10 MHz
        .uartBaudRate = 230400,   // 230400 baud instead of default 115200
        // Add other custom settings as needed
    };
    
    // Get SPI interface
    CommChannelsManager& commManager = CommChannelsManager::GetInstance();
    BaseSpi* spiDevice = commManager.GetSpiDevice(0);
    if (!spiDevice) {
        std::cout << "Failed to get SPI device" << std::endl;
        return;
    }
    
    // Create TMC9660Handler with custom configuration
    constexpr uint8_t TMC9660_ADDRESS = 0x01;
    Tmc9660Handler tmc9660Handler(*spiDevice, TMC9660_ADDRESS, &customBootConfig);
    
    // Initialize the handler
    if (!tmc9660Handler.Initialize()) {
        std::cout << "Failed to initialize TMC9660Handler" << std::endl;
        return;
    }
    
    std::cout << "TMC9660Handler with custom config initialized successfully!" << std::endl;
    
    // Access bootloader configuration
    const auto& bootConfig = tmc9660Handler.bootConfig();
    std::cout << "SPI Frequency: " << bootConfig.spiFrequency << " Hz" << std::endl;
    std::cout << "UART Baud Rate: " << bootConfig.uartBaudRate << " baud" << std::endl;
}

/**
 * @brief Main example function
 */
void RunTmc9660HandlerExamples() {
    std::cout << "TMC9660Handler Usage Examples" << std::endl;
    std::cout << "=============================" << std::endl;
    
    try {
        ExampleSpiOnly();
        ExampleUartOnly();
        ExampleDualInterface();
        ExampleCustomBootConfig();
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    
    std::cout << "\nAll examples completed!" << std::endl;
}
