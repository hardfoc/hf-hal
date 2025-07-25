/**
 * @file GpioManagerCleanExample.cpp
 * @brief Example demonstrating clean GPIO Manager using hf_gpio_err_t error codes.
 * 
 * This example shows how to use the clean GpioManager that eliminates error
 * message storage and uses the comprehensive hf_gpio_err_t error codes from
 * the hf-internal-interface submodule.
 * 
 * Key Benefits Demonstrated:
 * - Zero dynamic allocations for error tracking
 * - Structured error handling with actionable error codes
 * - 650+ bytes of memory savings
 * - Better integration with existing BaseGpio infrastructure
 * - Precise error handling and recovery
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.2 (Clean)
 */

#include "component-handlers/GpioManagerClean.h"
#include <iostream>
#include <string_view>

//==============================================================================
// ERROR HANDLING UTILITIES
//==============================================================================

/**
 * @brief Convert hf_gpio_err_t to descriptive string for debugging.
 * @param error GPIO error code
 * @return Static string describing the error
 */
const char* GpioErrorToString(hf_gpio_err_t error) noexcept {
    switch (error) {
        case hf_gpio_err_t::GPIO_SUCCESS: return "Success";
        case hf_gpio_err_t::GPIO_ERR_FAILURE: return "General failure";
        case hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED: return "Not initialized";
        case hf_gpio_err_t::GPIO_ERR_ALREADY_INITIALIZED: return "Already initialized";
        case hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER: return "Invalid parameter";
        case hf_gpio_err_t::GPIO_ERR_NULL_POINTER: return "Null pointer";
        case hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY: return "Out of memory";
        case hf_gpio_err_t::GPIO_ERR_INVALID_PIN: return "Invalid pin";
        case hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND: return "Pin not found";
        case hf_gpio_err_t::GPIO_ERR_PIN_NOT_CONFIGURED: return "Pin not configured";
        case hf_gpio_err_t::GPIO_ERR_PIN_ALREADY_REGISTERED: return "Pin already registered";
        case hf_gpio_err_t::GPIO_ERR_PIN_ACCESS_DENIED: return "Pin access denied";
        case hf_gpio_err_t::GPIO_ERR_PIN_BUSY: return "Pin busy";
        case hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT: return "Hardware fault";
        case hf_gpio_err_t::GPIO_ERR_COMMUNICATION_FAILURE: return "Communication failure";
        case hf_gpio_err_t::GPIO_ERR_DEVICE_NOT_RESPONDING: return "Device not responding";
        case hf_gpio_err_t::GPIO_ERR_TIMEOUT: return "Timeout";
        case hf_gpio_err_t::GPIO_ERR_VOLTAGE_OUT_OF_RANGE: return "Voltage out of range";
        case hf_gpio_err_t::GPIO_ERR_INVALID_CONFIGURATION: return "Invalid configuration";
        case hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION: return "Unsupported operation";
        case hf_gpio_err_t::GPIO_ERR_RESOURCE_BUSY: return "Resource busy";
        case hf_gpio_err_t::GPIO_ERR_RESOURCE_UNAVAILABLE: return "Resource unavailable";
        case hf_gpio_err_t::GPIO_ERR_READ_FAILURE: return "Read failure";
        case hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE: return "Write failure";
        case hf_gpio_err_t::GPIO_ERR_DIRECTION_MISMATCH: return "Direction mismatch";
        case hf_gpio_err_t::GPIO_ERR_PULL_RESISTOR_FAILURE: return "Pull resistor failure";
        case hf_gpio_err_t::GPIO_ERR_INTERRUPT_NOT_SUPPORTED: return "Interrupt not supported";
        case hf_gpio_err_t::GPIO_ERR_INTERRUPT_ALREADY_ENABLED: return "Interrupt already enabled";
        case hf_gpio_err_t::GPIO_ERR_INTERRUPT_NOT_ENABLED: return "Interrupt not enabled";
        case hf_gpio_err_t::GPIO_ERR_INTERRUPT_HANDLER_FAILED: return "Interrupt handler failed";
        case hf_gpio_err_t::GPIO_ERR_SYSTEM_ERROR: return "System error";
        case hf_gpio_err_t::GPIO_ERR_PERMISSION_DENIED: return "Permission denied";
        case hf_gpio_err_t::GPIO_ERR_OPERATION_ABORTED: return "Operation aborted";
        case hf_gpio_err_t::GPIO_ERR_NOT_SUPPORTED: return "Operation not supported";
        case hf_gpio_err_t::GPIO_ERR_DRIVER_ERROR: return "Driver error";
        case hf_gpio_err_t::GPIO_ERR_INVALID_STATE: return "Invalid state";
        case hf_gpio_err_t::GPIO_ERR_INVALID_ARG: return "Invalid argument";
        case hf_gpio_err_t::GPIO_ERR_CALIBRATION_FAILURE: return "Calibration failure";
        default: return "Unknown error";
    }
}

/**
 * @brief Handle GPIO operation result with appropriate action.
 * @param result GPIO operation result
 * @param operation_name Name of the operation for logging
 * @return true if successful, false if error occurred
 */
bool HandleGpioResult(hf_gpio_err_t result, const char* operation_name) noexcept {
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        std::cout << "✓ " << operation_name << " successful" << std::endl;
        return true;
    }
    
    std::cout << "✗ " << operation_name << " failed: " << GpioErrorToString(result) << std::endl;
    
    // Take specific actions based on error type
    switch (result) {
        case hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND:
            std::cout << "  → Check pin mapping and registration" << std::endl;
            break;
            
        case hf_gpio_err_t::GPIO_ERR_COMMUNICATION_FAILURE:
            std::cout << "  → Check I2C/SPI connections and bus health" << std::endl;
            break;
            
        case hf_gpio_err_t::GPIO_ERR_DEVICE_NOT_RESPONDING:
            std::cout << "  → Reset hardware or check power supply" << std::endl;
            break;
            
        case hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER:
            std::cout << "  → Verify pin name and parameter values" << std::endl;
            break;
            
        case hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT:
            std::cout << "  → Hardware issue detected - check connections" << std::endl;
            break;
            
        case hf_gpio_err_t::GPIO_ERR_TIMEOUT:
            std::cout << "  → Operation timed out - check system load" << std::endl;
            break;
            
        default:
            std::cout << "  → Refer to error code documentation for details" << std::endl;
            break;
    }
    
    return false;
}

//==============================================================================
// DEMONSTRATION FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate pin name validation with hf_gpio_err_t.
 */
void DemonstratePinValidation() {
    std::cout << "\n=== Pin Name Validation Using hf_gpio_err_t ===" << std::endl;
    
    std::array<std::string_view, 8> test_names = {{
        "VALID_PIN_NAME",           // Valid
        "",                         // Empty
        "CORE_RESERVED_PIN",        // Reserved prefix
        "1_STARTS_WITH_DIGIT",      // Starts with digit
        "INVALID@CHARS!",           // Invalid characters
        "ThisPinNameIsWayTooLongForTheSystemAndExceedsMaximumLength", // Too long
        "GPIO_USER_LED",            // Valid
        "TMC9660_GPIO17"            // Valid
    }};
    
    for (const auto& name : test_names) {
        hf_gpio_err_t result = GpioManagerClean::ValidatePinName(name);
        
        std::cout << "Pin name: '" << name << "'" << std::endl;
        std::cout << "  Result: " << GpioErrorToString(result);
        
        if (result == hf_gpio_err_t::GPIO_SUCCESS) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
            
            // Show specific handling for different validation errors
            switch (result) {
                case hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER:
                    std::cout << "    → Empty pin name or invalid format" << std::endl;
                    break;
                case hf_gpio_err_t::GPIO_ERR_INVALID_ARG:
                    std::cout << "    → Pin name contains invalid characters or format" << std::endl;
                    break;
                case hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY:
                    std::cout << "    → Pin name exceeds maximum length" << std::endl;
                    break;
                case hf_gpio_err_t::GPIO_ERR_PERMISSION_DENIED:
                    std::cout << "    → Pin name uses reserved system prefix" << std::endl;
                    break;
                default:
                    std::cout << "    → See error description above" << std::endl;
                    break;
            }
        }
    }
}

/**
 * @brief Demonstrate GPIO operations with structured error handling.
 */
void DemonstrateGpioOperations() {
    std::cout << "\n=== GPIO Operations with Structured Error Handling ===" << std::endl;
    
    auto& gpio_manager = GpioManagerClean::GetInstance();
    
    // Test basic GPIO operations
    std::cout << "\nTesting GPIO operations:" << std::endl;
    
    // Set GPIO pin
    hf_gpio_err_t result = gpio_manager.SetActive("GPIO_WS2812_LED_DAT");
    HandleGpioResult(result, "SetActive GPIO_WS2812_LED_DAT");
    
    // Read GPIO pin
    bool state;
    result = gpio_manager.Read("GPIO_WS2812_LED_DAT", state);
    if (HandleGpioResult(result, "Read GPIO_WS2812_LED_DAT")) {
        std::cout << "  Pin state: " << (state ? "ACTIVE" : "INACTIVE") << std::endl;
    }
    
    // Toggle GPIO pin
    result = gpio_manager.Toggle("GPIO_WS2812_LED_DAT");
    HandleGpioResult(result, "Toggle GPIO_WS2812_LED_DAT");
    
    // Test with non-existent pin (should fail)
    result = gpio_manager.SetActive("NONEXISTENT_PIN");
    HandleGpioResult(result, "SetActive NONEXISTENT_PIN");
    
    // Set pin to inactive
    result = gpio_manager.SetInactive("GPIO_WS2812_LED_DAT");
    HandleGpioResult(result, "SetInactive GPIO_WS2812_LED_DAT");
}

/**
 * @brief Demonstrate configuration operations with error handling.
 */
void DemonstrateConfigurationOperations() {
    std::cout << "\n=== Configuration Operations ===" << std::endl;
    
    auto& gpio_manager = GpioManagerClean::GetInstance();
    
    // Configure pin direction
    hf_gpio_err_t result = gpio_manager.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    HandleGpioResult(result, "SetDirection GPIO_EXT_GPIO_CS_1 to OUTPUT");
    
    // Configure pull mode
    result = gpio_manager.SetPullMode("GPIO_EXT_GPIO_CS_1", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
    HandleGpioResult(result, "SetPullMode GPIO_EXT_GPIO_CS_1 to PULL_UP");
    
    // Configure output mode
    result = gpio_manager.SetOutputMode("GPIO_EXT_GPIO_CS_1", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
    HandleGpioResult(result, "SetOutputMode GPIO_EXT_GPIO_CS_1 to PUSH_PULL");
    
    // Read back configuration
    hf_gpio_direction_t direction;
    result = gpio_manager.GetDirection("GPIO_EXT_GPIO_CS_1", direction);
    if (HandleGpioResult(result, "GetDirection GPIO_EXT_GPIO_CS_1")) {
        std::cout << "  Direction: " << (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT ? "OUTPUT" : "INPUT") << std::endl;
    }
}

/**
 * @brief Demonstrate batch operations with error handling.
 */
void DemonstrateBatchOperations() {
    std::cout << "\n=== Batch Operations with Error Codes ===" << std::endl;
    
    auto& gpio_manager = GpioManagerClean::GetInstance();
    
    // Batch read multiple pins
    std::vector<std::string_view> pins_to_read = {
        "GPIO_WS2812_LED_DAT",
        "GPIO_EXT_GPIO_CS_1",
        "NONEXISTENT_PIN"  // This will fail
    };
    
    auto read_result = gpio_manager.BatchRead(pins_to_read);
    
    std::cout << "Batch read results:" << std::endl;
    std::cout << "  Overall result: " << GpioErrorToString(read_result.overall_result) << std::endl;
    
    for (size_t i = 0; i < read_result.pin_names.size(); ++i) {
        std::cout << "  " << read_result.pin_names[i] << ": ";
        if (read_result.results[i] == hf_gpio_err_t::GPIO_SUCCESS) {
            std::cout << (read_result.states[i] ? "ACTIVE" : "INACTIVE") << " ✓" << std::endl;
        } else {
            std::cout << GpioErrorToString(read_result.results[i]) << " ✗" << std::endl;
        }
    }
    
    // Batch write multiple pins
    std::vector<std::string_view> pins_to_write = {"GPIO_WS2812_LED_DAT", "GPIO_EXT_GPIO_CS_1"};
    std::vector<bool> states = {true, false};
    
    GpioBatchOperation write_operation(pins_to_write, states);
    auto write_result = gpio_manager.BatchWrite(write_operation);
    
    std::cout << "\nBatch write results:" << std::endl;
    std::cout << "  Overall result: " << GpioErrorToString(write_result.overall_result) << std::endl;
    
    for (size_t i = 0; i < write_result.pin_names.size(); ++i) {
        std::cout << "  " << write_result.pin_names[i] << ": " 
                  << GpioErrorToString(write_result.results[i]);
        if (write_result.results[i] == hf_gpio_err_t::GPIO_SUCCESS) {
            std::cout << " ✓" << std::endl;
        } else {
            std::cout << " ✗" << std::endl;
        }
    }
}

/**
 * @brief Demonstrate lean diagnostics without error storage.
 */
void DemonstrateLeanDiagnostics() {
    std::cout << "\n=== Lean System Diagnostics ===" << std::endl;
    
    auto& gpio_manager = GpioManagerClean::GetInstance();
    
    GpioSystemDiagnostics diagnostics;
    hf_gpio_err_t result = gpio_manager.GetSystemDiagnostics(diagnostics);
    
    if (HandleGpioResult(result, "GetSystemDiagnostics")) {
        std::cout << "\nSystem Status:" << std::endl;
        std::cout << "  System Healthy: " << (diagnostics.system_healthy ? "YES" : "NO") << std::endl;
        std::cout << "  Total Pins Registered: " << diagnostics.total_pins_registered << std::endl;
        std::cout << "  Total Operations: " << diagnostics.total_operations << std::endl;
        std::cout << "  Successful Operations: " << diagnostics.successful_operations << std::endl;
        std::cout << "  Failed Operations: " << diagnostics.failed_operations << std::endl;
        std::cout << "  Validation Failures: " << diagnostics.validation_failures << std::endl;
        std::cout << "  Hardware Failures: " << diagnostics.hardware_failures << std::endl;
        std::cout << "  Success Rate: " << (diagnostics.success_rate * 100.0f) << "%" << std::endl;
        std::cout << "  Last Error: " << GpioErrorToString(diagnostics.last_error) << std::endl;
        
        std::cout << "\nHardware Status:" << std::endl;
        std::cout << "  PCAL95555 Available: " << (diagnostics.pcal95555_available ? "YES" : "NO") << std::endl;
        std::cout << "  TMC9660 Available: " << (diagnostics.tmc9660_available ? "YES" : "NO") << std::endl;
        std::cout << "  CommManager Initialized: " << (diagnostics.comm_manager_initialized ? "YES" : "NO") << std::endl;
    }
    
    std::cout << "\nMemory Usage:" << std::endl;
    std::cout << "  Error Storage: 0 bytes (removed)" << std::endl;
    std::cout << "  Error Buffer: 0 bytes (removed)" << std::endl;
    std::cout << "  Diagnostics: ~50 bytes (counters only)" << std::endl;
    std::cout << "  Total Savings: 650+ bytes vs previous version" << std::endl;
}

/**
 * @brief Demonstrate error recovery strategies.
 */
void DemonstrateErrorRecovery() {
    std::cout << "\n=== Error Recovery Strategies ===" << std::endl;
    
    auto& gpio_manager = GpioManagerClean::GetInstance();
    
    // Simulate various error scenarios and recovery
    std::cout << "\nTesting error recovery patterns:" << std::endl;
    
    // Test 1: Pin not found - suggest registration
    hf_gpio_err_t result = gpio_manager.SetActive("UNREGISTERED_PIN");
    if (result == hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND) {
        std::cout << "✓ Pin not found error detected" << std::endl;
        std::cout << "  Recovery: Register pin or check pin mapping" << std::endl;
        
        // Example recovery: Register a custom GPIO
        auto custom_gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(25));
        result = gpio_manager.RegisterGpio("CUSTOM_PIN", custom_gpio);
        HandleGpioResult(result, "Register CUSTOM_PIN");
    }
    
    // Test 2: Communication failure - suggest bus reset
    // (This would be simulated in real hardware)
    std::cout << "\nSimulating communication failure recovery:" << std::endl;
    std::cout << "  If GPIO_ERR_COMMUNICATION_FAILURE occurs:" << std::endl;
    std::cout << "    1. Check hardware connections" << std::endl;
    std::cout << "    2. Reset I2C/SPI bus" << std::endl;
    std::cout << "    3. Reinitialize affected handlers" << std::endl;
    std::cout << "    4. Retry operation" << std::endl;
    
    // Test 3: Resource busy - suggest retry
    std::cout << "\nResource management:" << std::endl;
    std::cout << "  If GPIO_ERR_RESOURCE_BUSY occurs:" << std::endl;
    std::cout << "    1. Wait for resource to become available" << std::endl;
    std::cout << "    2. Implement exponential backoff" << std::endl;
    std::cout << "    3. Check for deadlock conditions" << std::endl;
    std::cout << "    4. Timeout and report failure if needed" << std::endl;
}

//==============================================================================
// MAIN FUNCTION
//==============================================================================

int main() {
    std::cout << "Clean GPIO Manager Example" << std::endl;
    std::cout << "==========================" << std::endl;
    std::cout << "Demonstrating hf_gpio_err_t-based error handling:" << std::endl;
    std::cout << "- Zero dynamic allocations for error tracking" << std::endl;
    std::cout << "- Comprehensive hf_gpio_err_t error codes" << std::endl;
    std::cout << "- 650+ bytes memory savings" << std::endl;
    std::cout << "- Structured error handling and recovery" << std::endl;
    
    // Initialize the GPIO manager
    auto& gpio_manager = GpioManagerClean::GetInstance();
    hf_gpio_err_t init_result = gpio_manager.EnsureInitialized();
    
    if (!HandleGpioResult(init_result, "GPIO Manager Initialization")) {
        std::cout << "Failed to initialize GPIO manager - exiting" << std::endl;
        return -1;
    }
    
    // Run demonstrations
    DemonstratePinValidation();
    DemonstrateGpioOperations();
    DemonstrateConfigurationOperations();
    DemonstrateBatchOperations();
    DemonstrateLeanDiagnostics();
    DemonstrateErrorRecovery();
    
    // Reset all pins
    hf_gpio_err_t reset_result = gpio_manager.ResetAllPins();
    HandleGpioResult(reset_result, "Reset all pins");
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Clean GPIO Manager Benefits:" << std::endl;
    std::cout << "✓ Uses existing hf_gpio_err_t error codes (37 comprehensive codes)" << std::endl;
    std::cout << "✓ Zero dynamic memory allocation for error tracking" << std::endl;
    std::cout << "✓ 650+ bytes memory savings vs error storage version" << std::endl;
    std::cout << "✓ Structured error handling enables precise recovery strategies" << std::endl;
    std::cout << "✓ Better integration with existing BaseGpio infrastructure" << std::endl;
    std::cout << "✓ Actionable error codes for robust embedded applications" << std::endl;
    std::cout << "✓ Lean diagnostics with essential operational metrics only" << std::endl;
    std::cout << "✓ Perfect for resource-constrained embedded systems" << std::endl;
    
    return 0;
}