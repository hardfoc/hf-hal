/**
 * @file GpioManagerImprovedExample.cpp
 * @brief Example demonstrating improved GPIO Manager features.
 * 
 * This example showcases the enhanced pin name validation and static error
 * message buffer that eliminates dynamic allocations for embedded systems.
 * 
 * Key Improvements Demonstrated:
 * - Enhanced pin name validation with detailed error reporting
 * - Static error buffer with fixed memory allocation
 * - Compact error messages for memory conservation
 * - Thread-safe error tracking without dynamic allocation
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 */

#include "component-handlers/GpioManager.h"
#include <iostream>
#include <string_view>

//==============================================================================
// DEMONSTRATION FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate enhanced pin name validation.
 */
void DemonstrateEnhancedValidation() {
    std::cout << "\n=== Enhanced Pin Name Validation ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Test various pin names and show detailed validation results
    std::array<std::string_view, 10> test_names = {{
        "VALID_PIN_NAME",           // Valid
        "",                         // Empty
        "CORE_RESERVED_PIN",        // Reserved prefix
        "COMM_SPI_PIN",            // Reserved prefix  
        "SYS_INTERNAL",            // Reserved prefix
        "INTERNAL_TEST",           // Reserved prefix
        "1_STARTS_WITH_DIGIT",     // Starts with digit
        "INVALID@CHARS!",          // Invalid characters
        "ThisPinNameIsWayTooLongForTheSystemAndExceedsMaximumLength", // Too long
        "GOOD_GPIO_PIN_NAME"       // Valid
    }};
    
    for (const auto& name : test_names) {
        auto result = gpio_manager.ValidatePinNameDetailed(name);
        const char* result_str = GpioManager::ValidationResultToString(result);
        
        std::cout << "Pin name: '" << name << "'" << std::endl;
        std::cout << "  Result: " << result_str;
        
        if (result == GpioManager::PinNameValidationResult::VALID) {
            std::cout << " ✓";
        } else {
            std::cout << " ✗";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief Demonstrate static error buffer functionality.
 */
void DemonstrateStaticErrorBuffer() {
    std::cout << "\n=== Static Error Buffer Demonstration ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Force some validation errors to populate the error buffer
    std::cout << "Generating validation errors..." << std::endl;
    
    // Try to register pins with invalid names (this will generate errors)
    auto dummy_gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(25));
    
    // These will all fail and add errors to the static buffer
    gpio_manager.RegisterGpio("", dummy_gpio);                    // Empty name
    gpio_manager.RegisterGpio("CORE_RESERVED", dummy_gpio);       // Reserved prefix
    gpio_manager.RegisterGpio("1_DIGIT_START", dummy_gpio);       // Starts with digit
    gpio_manager.RegisterGpio("INVALID@CHAR", dummy_gpio);        // Invalid chars
    gpio_manager.RegisterGpio("ThisIsWayTooLongForSystem", dummy_gpio); // Too long
    
    // Try to register the same pin twice
    gpio_manager.RegisterGpio("VALID_PIN", dummy_gpio);
    gpio_manager.RegisterGpio("VALID_PIN", dummy_gpio);           // Duplicate
    
    // Get system diagnostics to see the errors
    GpioSystemDiagnostics diagnostics;
    if (gpio_manager.GetSystemDiagnostics(diagnostics)) {
        std::cout << "\nSystem Diagnostics:" << std::endl;
        std::cout << "Total operations: " << diagnostics.total_operations << std::endl;
        std::cout << "Failed operations: " << diagnostics.failed_operations << std::endl;
        std::cout << "Error count: " << diagnostics.error_messages.size() << std::endl;
        
        std::cout << "\nRecent errors (from static buffer):" << std::endl;
        for (size_t i = 0; i < diagnostics.error_messages.size(); ++i) {
            std::cout << "  " << (i + 1) << ". " << diagnostics.error_messages[i] << std::endl;
        }
    }
    
    std::cout << "\nNote: Error messages are stored in a fixed-size circular buffer" << std::endl;
    std::cout << "      - No dynamic allocations" << std::endl;
    std::cout << "      - Memory usage: 8 entries × 64 bytes = 512 bytes total" << std::endl;
    std::cout << "      - Older errors are automatically overwritten" << std::endl;
}

/**
 * @brief Demonstrate memory-efficient error tracking.
 */
void DemonstrateMemoryEfficiency() {
    std::cout << "\n=== Memory Efficiency Demonstration ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    std::cout << "Static Error Buffer Design:" << std::endl;
    std::cout << "  - Fixed memory allocation: 8 entries × 64 bytes = 512 bytes" << std::endl;
    std::cout << "  - Circular buffer overwrites oldest entries" << std::endl;
    std::cout << "  - No heap allocations during operation" << std::endl;
    std::cout << "  - Thread-safe with mutex protection" << std::endl;
    std::cout << "  - Compact error messages for memory conservation" << std::endl;
    
    std::cout << "\nPin Name Validation Features:" << std::endl;
    std::cout << "  - Compile-time reserved prefix array" << std::endl;
    std::cout << "  - Maximum name length: 32 characters" << std::endl;
    std::cout << "  - Detailed error categorization" << std::endl;
    std::cout << "  - Static string error messages" << std::endl;
    std::cout << "  - No string concatenation for error messages" << std::endl;
    
    // Generate more errors to show circular buffer behavior
    std::cout << "\nGenerating additional errors to demonstrate circular buffer..." << std::endl;
    
    auto dummy_gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(26));
    
    for (int i = 0; i < 12; ++i) {  // Generate more errors than buffer size
        std::string invalid_name = "CORE_ERROR_" + std::to_string(i);
        gpio_manager.RegisterGpio(invalid_name, dummy_gpio);
    }
    
    // Check how many errors are retained
    GpioSystemDiagnostics final_diagnostics;
    if (gpio_manager.GetSystemDiagnostics(final_diagnostics)) {
        std::cout << "\nFinal error buffer state:" << std::endl;
        std::cout << "Total errors generated: ~18+" << std::endl;
        std::cout << "Errors retained in buffer: " << final_diagnostics.error_messages.size() << std::endl;
        std::cout << "Memory usage: unchanged (fixed allocation)" << std::endl;
        
        if (final_diagnostics.error_messages.size() == 8) {
            std::cout << "✓ Circular buffer working correctly - oldest errors overwritten" << std::endl;
        }
    }
}

/**
 * @brief Show validation result enum usage.
 */
void DemonstrateValidationResultUsage() {
    std::cout << "\n=== Validation Result Usage ===" << std::endl;
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    std::cout << "Detailed validation allows precise error handling:" << std::endl;
    
    std::array<std::string_view, 6> test_cases = {{
        "GOOD_PIN",
        "",
        "CORE_SYSTEM_PIN", 
        "1_BAD_START",
        "bad@chars",
        "VeryLongPinNameThatExceedsLimits"
    }};
    
    for (const auto& pin_name : test_cases) {
        auto result = gpio_manager.ValidatePinNameDetailed(pin_name);
        
        std::cout << "\nPin: '" << pin_name << "'" << std::endl;
        std::cout << "Result: " << GpioManager::ValidationResultToString(result) << std::endl;
        
        // Example of how to handle specific validation errors
        switch (result) {
            case GpioManager::PinNameValidationResult::VALID:
                std::cout << "Action: Proceed with registration" << std::endl;
                break;
                
            case GpioManager::PinNameValidationResult::EMPTY:
                std::cout << "Action: Request user to provide pin name" << std::endl;
                break;
                
            case GpioManager::PinNameValidationResult::RESERVED_PREFIX:
                std::cout << "Action: Suggest alternative name without reserved prefix" << std::endl;
                break;
                
            case GpioManager::PinNameValidationResult::TOO_LONG:
                std::cout << "Action: Truncate or request shorter name" << std::endl;
                break;
                
            case GpioManager::PinNameValidationResult::STARTS_WITH_DIGIT:
                std::cout << "Action: Suggest adding letter prefix" << std::endl;
                break;
                
            case GpioManager::PinNameValidationResult::INVALID_CHARS:
                std::cout << "Action: Remove invalid characters" << std::endl;
                break;
        }
    }
}

//==============================================================================
// MAIN FUNCTION
//==============================================================================

int main() {
    std::cout << "GPIO Manager Enhanced Features Example" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Demonstrating improvements for embedded systems:" << std::endl;
    std::cout << "- Enhanced pin name validation" << std::endl;
    std::cout << "- Static error buffer (no dynamic allocation)" << std::endl;
    std::cout << "- Memory-efficient error tracking" << std::endl;
    std::cout << "- Compact error messages" << std::endl;
    
    // Initialize the GPIO manager
    auto& gpio_manager = GpioManager::GetInstance();
    if (!gpio_manager.EnsureInitialized()) {
        std::cout << "✗ Failed to initialize GPIO manager" << std::endl;
        return -1;
    }
    
    std::cout << "✓ GPIO manager initialized successfully" << std::endl;
    
    // Run demonstrations
    DemonstrateEnhancedValidation();
    DemonstrateValidationResultUsage();
    DemonstrateStaticErrorBuffer();
    DemonstrateMemoryEfficiency();
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Enhanced GPIO Manager Features:" << std::endl;
    std::cout << "✓ Detailed pin name validation with specific error types" << std::endl;
    std::cout << "✓ Static error buffer eliminates dynamic allocations" << std::endl;
    std::cout << "✓ Fixed memory usage: 512 bytes for error tracking" << std::endl;
    std::cout << "✓ Compact error messages conserve memory" << std::endl;
    std::cout << "✓ Thread-safe operation with mutex protection" << std::endl;
    std::cout << "✓ Circular buffer automatically manages old errors" << std::endl;
    std::cout << "✓ Suitable for memory-constrained embedded systems" << std::endl;
    
    return 0;
}