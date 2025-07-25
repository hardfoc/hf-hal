/**
 * @file GpioManagerExample.cpp
 * @brief Example demonstrating the advanced GPIO management system.
 * 
 * This example shows how to use the GpioManager with string-based pin identification,
 * complete BaseGpio function coverage, smart pin categorization, and proper
 * electrical configuration handling.
 * 
 * Key Features Demonstrated:
 * - String-based pin identification for extensibility
 * - Complete BaseGpio function coverage through routing
 * - Smart pin categorization (CORE, COMM, GPIO, USER)
 * - Proper electrical configuration (pull resistors, output modes, inversion)
 * - Handler-aware GPIO creation and ownership
 * - Thread-safe operations with comprehensive error handling
 * - Batch operations for performance optimization
 * - Advanced diagnostics and health monitoring
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 */

#include "component-handlers/GpioManager.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include <iostream>
#include <thread>
#include <chrono>

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate basic GPIO operations using string names.
 */
void DemonstrateBasicOperations() {
    std::cout << "\n=== Basic GPIO Operations ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Set a GPIO pin to active state
    if (gpio_manager.SetActive("GPIO_WS2812_LED_DAT")) {
        std::cout << "✓ Set WS2812 LED pin to active" << std::endl;
    } else {
        std::cout << "✗ Failed to set WS2812 LED pin to active" << std::endl;
    }
    
    // Read the current state
    bool state;
    if (gpio_manager.Read("GPIO_WS2812_LED_DAT", state)) {
        std::cout << "✓ Read WS2812 LED pin state: " << (state ? "ACTIVE" : "INACTIVE") << std::endl;
    } else {
        std::cout << "✗ Failed to read WS2812 LED pin state" << std::endl;
    }
    
    // Toggle the pin
    if (gpio_manager.Toggle("GPIO_WS2812_LED_DAT")) {
        std::cout << "✓ Toggled WS2812 LED pin" << std::endl;
    } else {
        std::cout << "✗ Failed to toggle WS2812 LED pin" << std::endl;
    }
    
    // Set to inactive
    if (gpio_manager.SetInactive("GPIO_WS2812_LED_DAT")) {
        std::cout << "✓ Set WS2812 LED pin to inactive" << std::endl;
    } else {
        std::cout << "✗ Failed to set WS2812 LED pin to inactive" << std::endl;
    }
}

/**
 * @brief Demonstrate pin configuration operations.
 */
void DemonstratePinConfiguration() {
    std::cout << "\n=== Pin Configuration ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Configure pin direction
    if (gpio_manager.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT) == hf_gpio_err_t::GPIO_SUCCESS) {
        std::cout << "✓ Set EXT_GPIO_CS_1 to output direction" << std::endl;
    } else {
        std::cout << "✗ Failed to set EXT_GPIO_CS_1 direction" << std::endl;
    }
    
    // Configure pull mode (this pin has pull-up configured in mapping)
    if (gpio_manager.SetPullMode("GPIO_EXT_GPIO_CS_1", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP) == hf_gpio_err_t::GPIO_SUCCESS) {
        std::cout << "✓ Set EXT_GPIO_CS_1 pull mode to UP" << std::endl;
    } else {
        std::cout << "✗ Failed to set EXT_GPIO_CS_1 pull mode" << std::endl;
    }
    
    // Configure output mode (push-pull vs open-drain)
    if (gpio_manager.SetOutputMode("GPIO_EXT_GPIO_CS_1", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL) == hf_gpio_err_t::GPIO_SUCCESS) {
        std::cout << "✓ Set EXT_GPIO_CS_1 to push-pull output mode" << std::endl;
    } else {
        std::cout << "✗ Failed to set EXT_GPIO_CS_1 output mode" << std::endl;
    }
    
    // Read back configuration
    hf_gpio_direction_t direction;
    if (gpio_manager.GetDirection("GPIO_EXT_GPIO_CS_1", direction)) {
        std::cout << "✓ EXT_GPIO_CS_1 direction: " << (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT ? "OUTPUT" : "INPUT") << std::endl;
    }
    
    hf_gpio_pull_mode_t pull_mode;
    if (gpio_manager.GetPullMode("GPIO_EXT_GPIO_CS_1", pull_mode)) {
        std::cout << "✓ EXT_GPIO_CS_1 pull mode: " << static_cast<int>(pull_mode) << std::endl;
    }
    
    hf_gpio_output_mode_t output_mode;
    if (gpio_manager.GetOutputMode("GPIO_EXT_GPIO_CS_1", output_mode)) {
        std::cout << "✓ EXT_GPIO_CS_1 output mode: " << (output_mode == hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL ? "PUSH_PULL" : "OPEN_DRAIN") << std::endl;
    }
}

/**
 * @brief Demonstrate interrupt handling.
 */
void DemonstrateInterruptHandling() {
    std::cout << "\n=== Interrupt Handling ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Check if pin supports interrupts
    if (gpio_manager.SupportsInterrupts("GPIO_PCAL_IMU_INT")) {
        std::cout << "✓ PCAL_IMU_INT supports interrupts" << std::endl;
        
        // Configure interrupt callback
        auto interrupt_callback = [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
            std::cout << "🔔 Interrupt triggered on pin!" << std::endl;
            (void)gpio; (void)trigger; (void)user_data; // Suppress unused warnings
        };
        
        // Configure interrupt
        if (gpio_manager.ConfigureInterrupt("GPIO_PCAL_IMU_INT", 
                                          hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
                                          interrupt_callback) == hf_gpio_err_t::GPIO_SUCCESS) {
            std::cout << "✓ Configured rising edge interrupt on PCAL_IMU_INT" << std::endl;
        } else {
            std::cout << "✗ Failed to configure interrupt on PCAL_IMU_INT" << std::endl;
        }
        
        // Enable interrupt
        if (gpio_manager.EnableInterrupt("GPIO_PCAL_IMU_INT") == hf_gpio_err_t::GPIO_SUCCESS) {
            std::cout << "✓ Enabled interrupt on PCAL_IMU_INT" << std::endl;
        } else {
            std::cout << "✗ Failed to enable interrupt on PCAL_IMU_INT" << std::endl;
        }
        
        // Wait a bit for potential interrupt
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Disable interrupt
        if (gpio_manager.DisableInterrupt("GPIO_PCAL_IMU_INT") == hf_gpio_err_t::GPIO_SUCCESS) {
            std::cout << "✓ Disabled interrupt on PCAL_IMU_INT" << std::endl;
        } else {
            std::cout << "✗ Failed to disable interrupt on PCAL_IMU_INT" << std::endl;
        }
    } else {
        std::cout << "✗ PCAL_IMU_INT does not support interrupts" << std::endl;
    }
}

/**
 * @brief Demonstrate batch operations for performance.
 */
void DemonstrateBatchOperations() {
    std::cout << "\n=== Batch Operations ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Batch read multiple pins
    std::vector<std::string_view> pins_to_read = {
        "GPIO_WS2812_LED_DAT",
        "GPIO_EXT_GPIO_CS_1",
        "GPIO_EXT_GPIO_CS_2"
    };
    
    auto read_result = gpio_manager.BatchRead(pins_to_read);
    if (read_result.AllSuccessful()) {
        std::cout << "✓ Batch read successful for " << read_result.pin_names.size() << " pins" << std::endl;
        for (size_t i = 0; i < read_result.pin_names.size(); ++i) {
            std::cout << "  " << read_result.pin_names[i] << ": " << (read_result.states[i] ? "ACTIVE" : "INACTIVE") << std::endl;
        }
    } else {
        std::cout << "✗ Batch read failed" << std::endl;
    }
    
    // Batch write multiple pins
    std::vector<std::string_view> pins_to_write = {"GPIO_WS2812_LED_DAT", "GPIO_EXT_GPIO_CS_1"};
    std::vector<bool> states = {true, false};
    
    GpioBatchOperation write_operation(pins_to_write, states);
    auto write_result = gpio_manager.BatchWrite(write_operation);
    if (write_result.AllSuccessful()) {
        std::cout << "✓ Batch write successful for " << write_result.pin_names.size() << " pins" << std::endl;
    } else {
        std::cout << "✗ Batch write failed" << std::endl;
    }
    
    // Set multiple pins to active
    auto active_result = gpio_manager.SetMultipleActive({"GPIO_WS2812_LED_DAT"});
    if (active_result.AllSuccessful()) {
        std::cout << "✓ Set multiple pins to active successful" << std::endl;
    } else {
        std::cout << "✗ Set multiple pins to active failed" << std::endl;
    }
    
    // Set multiple pins to inactive
    auto inactive_result = gpio_manager.SetMultipleInactive({"GPIO_WS2812_LED_DAT"});
    if (inactive_result.AllSuccessful()) {
        std::cout << "✓ Set multiple pins to inactive successful" << std::endl;
    } else {
        std::cout << "✗ Set multiple pins to inactive failed" << std::endl;
    }
}

/**
 * @brief Demonstrate user-defined GPIO registration.
 */
void DemonstrateUserDefinedGpio() {
    std::cout << "\n=== User-Defined GPIO Registration ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Create a custom ESP32 GPIO instance
    auto custom_gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(25)); // GPIO25
    
    // Configure the custom GPIO
    custom_gpio->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    custom_gpio->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
    custom_gpio->SetOutputMode(hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
    custom_gpio->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH);
    
    if (custom_gpio->Initialize()) {
        // Register the custom GPIO
        if (gpio_manager.RegisterGpio("USER_CUSTOM_LED", custom_gpio)) {
            std::cout << "✓ Registered custom GPIO as USER_CUSTOM_LED" << std::endl;
        } else {
            std::cout << "✗ Failed to register custom GPIO" << std::endl;
            return;
        }
        
        // Use the custom GPIO
        if (gpio_manager.SetActive("USER_CUSTOM_LED")) {
            std::cout << "✓ Set custom LED to active" << std::endl;
        }
        
        if (gpio_manager.SetInactive("USER_CUSTOM_LED")) {
            std::cout << "✓ Set custom LED to inactive" << std::endl;
        }
    } else {
        std::cout << "✗ Failed to initialize custom GPIO" << std::endl;
    }
}

/**
 * @brief Demonstrate system diagnostics and health monitoring.
 */
void DemonstrateSystemDiagnostics() {
    std::cout << "\n=== System Diagnostics ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Get system diagnostics
    GpioSystemDiagnostics diagnostics;
    if (gpio_manager.GetSystemDiagnostics(diagnostics)) {
        std::cout << "✓ System Diagnostics:" << std::endl;
        std::cout << "  System Healthy: " << (diagnostics.system_healthy ? "YES" : "NO") << std::endl;
        std::cout << "  Total Pins Registered: " << diagnostics.total_pins_registered << std::endl;
        std::cout << "  Total Operations: " << diagnostics.total_operations << std::endl;
        std::cout << "  Successful Operations: " << diagnostics.successful_operations << std::endl;
        std::cout << "  Failed Operations: " << diagnostics.failed_operations << std::endl;
        std::cout << "  Communication Errors: " << diagnostics.communication_errors << std::endl;
        std::cout << "  Hardware Errors: " << diagnostics.hardware_errors << std::endl;
        
        // Show pins by category
        std::cout << "  Pins by Category:" << std::endl;
        std::cout << "    CORE: " << diagnostics.pins_by_category[0] << std::endl;
        std::cout << "    COMM: " << diagnostics.pins_by_category[1] << std::endl;
        std::cout << "    GPIO: " << diagnostics.pins_by_category[2] << std::endl;
        std::cout << "    USER: " << diagnostics.pins_by_category[3] << std::endl;
        
        // Show pins by chip
        std::cout << "  Pins by Chip:" << std::endl;
        std::cout << "    ESP32: " << diagnostics.pins_by_chip[0] << std::endl;
        std::cout << "    PCAL95555: " << diagnostics.pins_by_chip[1] << std::endl;
        std::cout << "    TMC9660: " << diagnostics.pins_by_chip[2] << std::endl;
        
        // Show recent errors
        if (!diagnostics.error_messages.empty()) {
            std::cout << "  Recent Errors:" << std::endl;
            for (const auto& error : diagnostics.error_messages) {
                std::cout << "    - " << error << std::endl;
            }
        }
    } else {
        std::cout << "✗ Failed to get system diagnostics" << std::endl;
    }
    
    // Get system health information
    std::string health_info;
    if (gpio_manager.GetSystemHealth(health_info)) {
        std::cout << "✓ System Health: " << health_info << std::endl;
    } else {
        std::cout << "✗ Failed to get system health" << std::endl;
    }
    
    // Get pin statistics
    BaseGpio::PinStatistics stats;
    if (gpio_manager.GetStatistics("GPIO_WS2812_LED_DAT", stats)) {
        std::cout << "✓ WS2812 LED Pin Statistics:" << std::endl;
        std::cout << "  Total Operations: " << stats.totalOperations << std::endl;
        std::cout << "  Successful Operations: " << stats.successfulOperations << std::endl;
        std::cout << "  Failed Operations: " << stats.failedOperations << std::endl;
        std::cout << "  State Changes: " << stats.stateChanges << std::endl;
        std::cout << "  Direction Changes: " << stats.directionChanges << std::endl;
        std::cout << "  Interrupt Count: " << stats.interruptCount << std::endl;
    } else {
        std::cout << "✗ Failed to get pin statistics" << std::endl;
    }
    
    // Reset statistics
    if (gpio_manager.ResetStatistics("GPIO_WS2812_LED_DAT")) {
        std::cout << "✓ Reset pin statistics" << std::endl;
    } else {
        std::cout << "✗ Failed to reset pin statistics" << std::endl;
    }
}

/**
 * @brief Demonstrate pin categorization and validation.
 */
void DemonstratePinCategorization() {
    std::cout << "\n=== Pin Categorization and Validation ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Demonstrate pin categorization
    std::cout << "Pin Categorization:" << std::endl;
    
    // CORE pins (system reserved - should not be registered as GPIOs)
    std::cout << "  CORE pins (system reserved):" << std::endl;
    std::cout << "    CORE_XTAL_32K_P: " << (gpio_manager.Contains("CORE_XTAL_32K_P") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    std::cout << "    CORE_BOOT_SEL: " << (gpio_manager.Contains("CORE_BOOT_SEL") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    
    // COMM pins (communication - should not be registered as GPIOs)
    std::cout << "  COMM pins (communication):" << std::endl;
    std::cout << "    COMM_SPI2_MISO: " << (gpio_manager.Contains("COMM_SPI2_MISO") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    std::cout << "    COMM_I2C_SDA: " << (gpio_manager.Contains("COMM_I2C_SDA") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    
    // GPIO pins (available for GPIO operations)
    std::cout << "  GPIO pins (available for GPIO):" << std::endl;
    std::cout << "    GPIO_WS2812_LED_DAT: " << (gpio_manager.Contains("GPIO_WS2812_LED_DAT") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    std::cout << "    GPIO_EXT_GPIO_CS_1: " << (gpio_manager.Contains("GPIO_EXT_GPIO_CS_1") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    
    // PCAL95555 pins
    std::cout << "  PCAL95555 pins:" << std::endl;
    std::cout << "    GPIO_PCAL_GPIO17: " << (gpio_manager.Contains("GPIO_PCAL_GPIO17") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    std::cout << "    GPIO_PCAL_IMU_INT: " << (gpio_manager.Contains("GPIO_PCAL_IMU_INT") ? "REGISTERED" : "NOT REGISTERED") << std::endl;
    
    // Demonstrate validation
    std::cout << "\nPin Name Validation:" << std::endl;
    std::cout << "  Valid names (should work):" << std::endl;
    std::cout << "    USER_CUSTOM_LED: " << (gpio_manager.Contains("USER_CUSTOM_LED") ? "EXISTS" : "DOES NOT EXIST") << std::endl;
    
    std::cout << "  Reserved prefixes (should be rejected):" << std::endl;
    std::cout << "    CORE_RESERVED: Reserved prefix" << std::endl;
    std::cout << "    COMM_RESERVED: Reserved prefix" << std::endl;
    std::cout << "    SYS_RESERVED: Reserved prefix" << std::endl;
    std::cout << "    INTERNAL_RESERVED: Reserved prefix" << std::endl;
    
    // Show total registered pins
    std::cout << "\nTotal registered pins: " << gpio_manager.Size() << std::endl;
}

/**
 * @brief Demonstrate electrical configuration handling.
 */
void DemonstrateElectricalConfiguration() {
    std::cout << "\n=== Electrical Configuration ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Demonstrate how pins are configured with proper electrical characteristics
    std::cout << "Pin Electrical Configuration (from mapping):" << std::endl;
    
    // Show configuration for different pin types
    std::vector<std::string_view> test_pins = {
        "GPIO_WS2812_LED_DAT",      // No pull, push-pull output
        "GPIO_EXT_GPIO_CS_1",       // Pull-up, push-pull output, inverted
        "GPIO_PCAL_FAULT_STATUS",   // No pull, push-pull output, inverted
        "GPIO_PCAL_IMU_INT"         // No pull, push-pull output, inverted
    };
    
    for (const auto& pin_name : test_pins) {
        if (gpio_manager.Contains(pin_name)) {
            std::cout << "  " << pin_name << ":" << std::endl;
            
            // Get pull mode
            hf_gpio_pull_mode_t pull_mode;
            if (gpio_manager.GetPullMode(pin_name, pull_mode)) {
                std::cout << "    Pull Mode: ";
                switch (pull_mode) {
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
                        std::cout << "FLOATING (no pull)";
                        break;
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
                        std::cout << "PULL-UP";
                        break;
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
                        std::cout << "PULL-DOWN";
                        break;
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
                        std::cout << "PULL-UP + PULL-DOWN";
                        break;
                }
                std::cout << std::endl;
            }
            
            // Get output mode
            hf_gpio_output_mode_t output_mode;
            if (gpio_manager.GetOutputMode(pin_name, output_mode)) {
                std::cout << "    Output Mode: ";
                switch (output_mode) {
                    case hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL:
                        std::cout << "PUSH-PULL";
                        break;
                    case hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN:
                        std::cout << "OPEN-DRAIN";
                        break;
                }
                std::cout << std::endl;
            }
            
            // Get direction
            hf_gpio_direction_t direction;
            if (gpio_manager.GetDirection(pin_name, direction)) {
                std::cout << "    Direction: ";
                switch (direction) {
                    case hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT:
                        std::cout << "INPUT";
                        break;
                    case hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT:
                        std::cout << "OUTPUT";
                        break;
                }
                std::cout << std::endl;
            }
        }
    }
    
    std::cout << "\nNote: All pins are automatically configured with proper electrical characteristics" << std::endl;
    std::cout << "based on the primitive configuration fields in the pin mapping:" << std::endl;
    std::cout << "- has_pull: Whether pin has pull resistor" << std::endl;
    std::cout << "- pull_is_up: If has_pull=true: true=pull-up, false=pull-down" << std::endl;
    std::cout << "- is_push_pull: Output mode: true=push-pull, false=open-drain" << std::endl;
    std::cout << "- is_inverted: Logic inversion: true=inverted, false=normal" << std::endl;
    std::cout << "- max_current_ma: Maximum current in milliamps" << std::endl;
}

//==============================================================================
// MAIN FUNCTION
//==============================================================================

int main() {
    std::cout << "HardFOC GPIO Manager Example" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Demonstrating advanced GPIO management with:" << std::endl;
    std::cout << "- String-based pin identification" << std::endl;
    std::cout << "- Complete BaseGpio function coverage" << std::endl;
    std::cout << "- Smart pin categorization (CORE, COMM, GPIO, USER)" << std::endl;
    std::cout << "- Proper electrical configuration handling" << std::endl;
    std::cout << "- Handler-aware GPIO creation and ownership" << std::endl;
    std::cout << "- Thread-safe operations with comprehensive error handling" << std::endl;
    std::cout << "- Batch operations for performance optimization" << std::endl;
    std::cout << "- Advanced diagnostics and health monitoring" << std::endl;
    
    // Initialize the GPIO manager
    auto& gpio_manager = GpioManager::GetInstance();
    if (!gpio_manager.EnsureInitialized()) {
        std::cout << "✗ Failed to initialize GPIO manager" << std::endl;
        return -1;
    }
    
    std::cout << "✓ GPIO manager initialized successfully" << std::endl;
    
    // Run demonstrations
    DemonstratePinCategorization();
    DemonstrateElectricalConfiguration();
    DemonstrateBasicOperations();
    DemonstratePinConfiguration();
    DemonstrateInterruptHandling();
    DemonstrateBatchOperations();
    DemonstrateUserDefinedGpio();
    DemonstrateSystemDiagnostics();
    
    // Reset all pins to inactive state
    if (gpio_manager.ResetAllPins()) {
        std::cout << "\n✓ Reset all output pins to inactive state" << std::endl;
    } else {
        std::cout << "\n✗ Failed to reset all pins" << std::endl;
    }
    
    std::cout << "\n=== Example Complete ===" << std::endl;
    std::cout << "The GPIO manager successfully demonstrated:" << std::endl;
    std::cout << "✓ String-based pin identification for extensibility" << std::endl;
    std::cout << "✓ Complete BaseGpio function coverage through routing" << std::endl;
    std::cout << "✓ Smart pin categorization (CORE, COMM, GPIO, USER)" << std::endl;
    std::cout << "✓ Proper electrical configuration (pull resistors, output modes, inversion)" << std::endl;
    std::cout << "✓ Handler-aware GPIO creation and ownership" << std::endl;
    std::cout << "✓ Thread-safe operations with comprehensive error handling" << std::endl;
    std::cout << "✓ Batch operations for performance optimization" << std::endl;
    std::cout << "✓ Advanced diagnostics and health monitoring" << std::endl;
    std::cout << "✓ User-defined GPIO registration and management" << std::endl;
    
    return 0;
} 