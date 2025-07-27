/**
 * @file AdcManagerExample.cpp
 * @brief Comprehensive example demonstrating the new AdcManager capabilities.
 * 
 * @details This example shows how to use the refactored AdcManager with:
 * - Platform mapping integration for automatic channel discovery
 * - ESP32 internal ADC and TMC9660 ADC support
 * - Complete BaseAdc function coverage through string-based routing
 * - Batch operations and advanced diagnostics
 * - Thread-safe operations and error handling
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features Demonstrated:
 * - Automatic initialization and channel registration
 * - Single channel reading operations
 * - Batch reading operations
 * - Statistics and diagnostics
 * - System health monitoring
 * - Error handling and recovery
 * - Platform mapping integration
 * - TMC9660 ADC channel types (AIN, Current, Voltage, Temperature, Motor Data)
 * 
 * @note This example assumes the AdcManager has been properly initialized
 *       with platform mapping and hardware handlers.
 */

#include "AdcManager.h"
#include "MotorController.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/ConsolePort.h"

#include <chrono>
#include <thread>
#include <vector>
#include <string>

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate basic ADC channel reading operations.
 */
void DemonstrateBasicReading() {
    ConsolePort::Printf("\n=== Basic ADC Reading Operations ===\n");
    
    // Get ADC manager instance
    auto& adc_manager = GetAdcManager();
    
    // Ensure system is initialized
    hf_adc_err_t init_result = adc_manager.EnsureInitialized();
    if (init_result != hf_adc_err_t::ADC_SUCCESS) {
        ConsolePort::Printf("Failed to initialize AdcManager: %d\n", static_cast<int>(init_result));
        return;
    }
    
    ConsolePort::Printf("AdcManager initialized successfully\n");
    
    // Log all registered channels
    adc_manager.LogAllRegisteredChannels();
    
    // Example channel names from platform mapping
    std::vector<std::string_view> test_channels = {
        "TMC9660_AIN3",           // Temperature sensor (connected)
        "TMC9660_CURRENT_I0",     // Current sense I0
        "TMC9660_SUPPLY_VOLTAGE", // Supply voltage
        "TMC9660_CHIP_TEMPERATURE", // Chip temperature
        "TMC9660_MOTOR_CURRENT"   // Motor current
    };
    
    // Test reading from each channel
    for (const auto& channel_name : test_channels) {
        if (adc_manager.Contains(channel_name)) {
            ConsolePort::Printf("\nReading from channel: %.*s\n", 
                               static_cast<int>(channel_name.length()), channel_name.data());
            
            // Read voltage only
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("  Voltage: %.3fV\n", voltage);
            } else {
                ConsolePort::Printf("  Error reading voltage: %d\n", static_cast<int>(result));
            }
            
            // Read raw count only
            hf_u32_t raw_value;
            result = adc_manager.ReadChannelCount(channel_name, raw_value);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("  Raw Count: %u\n", raw_value);
            } else {
                ConsolePort::Printf("  Error reading raw count: %d\n", static_cast<int>(result));
            }
            
            // Read both voltage and raw count
            hf_u32_t raw_value2;
            float voltage2;
            result = adc_manager.ReadChannel(channel_name, raw_value2, voltage2);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("  Combined Read - Voltage: %.3fV, Raw: %u\n", voltage2, raw_value2);
            } else {
                ConsolePort::Printf("  Error reading combined: %d\n", static_cast<int>(result));
            }
        } else {
            ConsolePort::Printf("Channel %.*s not found\n", 
                               static_cast<int>(channel_name.length()), channel_name.data());
        }
    }
}

/**
 * @brief Demonstrate TMC9660-specific channel reading.
 */
void DemonstrateTmc9660Channels() {
    ConsolePort::Printf("\n=== TMC9660 ADC Channel Reading ===\n");
    
    auto& adc_manager = GetAdcManager();
    
    // Test AIN channels (external analog inputs)
    ConsolePort::Printf("\n--- AIN Channels (External Analog Inputs) ---\n");
    for (int i = 0; i <= 3; ++i) {
        std::string channel_name = "TMC9660_AIN" + std::to_string(i);
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("AIN%d: %.3fV\n", i, voltage);
            } else {
                ConsolePort::Printf("AIN%d: Error %d\n", i, static_cast<int>(result));
            }
        }
    }
    
    // Test current sense channels
    ConsolePort::Printf("\n--- Current Sense Channels ---\n");
    for (int i = 0; i <= 3; ++i) {
        std::string channel_name = "TMC9660_CURRENT_I" + std::to_string(i);
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("Current I%d: %.3fV\n", i, voltage);
            } else {
                ConsolePort::Printf("Current I%d: Error %d\n", i, static_cast<int>(result));
            }
        }
    }
    
    // Test voltage monitoring channels
    ConsolePort::Printf("\n--- Voltage Monitoring Channels ---\n");
    std::vector<std::string> voltage_channels = {"TMC9660_SUPPLY_VOLTAGE", "TMC9660_DRIVER_VOLTAGE"};
    for (const auto& channel_name : voltage_channels) {
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("%s: %.3fV\n", channel_name.c_str(), voltage);
            } else {
                ConsolePort::Printf("%s: Error %d\n", channel_name.c_str(), static_cast<int>(result));
            }
        }
    }
    
    // Test temperature channels
    ConsolePort::Printf("\n--- Temperature Channels ---\n");
    std::vector<std::string> temp_channels = {"TMC9660_CHIP_TEMPERATURE", "TMC9660_EXTERNAL_TEMPERATURE"};
    for (const auto& channel_name : temp_channels) {
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                ConsolePort::Printf("%s: %.1f°C\n", channel_name.c_str(), voltage); // voltage contains temperature
            } else {
                ConsolePort::Printf("%s: Error %d\n", channel_name.c_str(), static_cast<int>(result));
            }
        }
    }
    
    // Test motor data channels
    ConsolePort::Printf("\n--- Motor Data Channels ---\n");
    std::vector<std::string> motor_channels = {"TMC9660_MOTOR_CURRENT", "TMC9660_MOTOR_VELOCITY", "TMC9660_MOTOR_POSITION"};
    for (const auto& channel_name : motor_channels) {
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                if (channel_name == "TMC9660_MOTOR_CURRENT") {
                    ConsolePort::Printf("%s: %.3fA\n", channel_name.c_str(), voltage);
                } else {
                    ConsolePort::Printf("%s: %.0f\n", channel_name.c_str(), voltage);
                }
            } else {
                ConsolePort::Printf("%s: Error %d\n", channel_name.c_str(), static_cast<int>(result));
            }
        }
    }
}

/**
 * @brief Demonstrate batch reading operations.
 */
void DemonstrateBatchReading() {
    ConsolePort::Printf("\n=== Batch Reading Operations ===\n");
    
    auto& adc_manager = GetAdcManager();
    
    // Define channels to read in batch
    std::vector<std::string_view> batch_channels = {
        "TMC9660_AIN3",
        "TMC9660_CURRENT_I0", 
        "TMC9660_SUPPLY_VOLTAGE",
        "TMC9660_CHIP_TEMPERATURE"
    };
    
    // Read multiple channels simultaneously
    std::vector<hf_u32_t> raw_values(batch_channels.size());
    std::vector<float> voltages(batch_channels.size());
    
    hf_adc_err_t result = adc_manager.ReadMultipleChannels(
        batch_channels.data(), 
        static_cast<hf_u8_t>(batch_channels.size()),
        raw_values.data(),
        voltages.data()
    );
    
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        ConsolePort::Printf("Batch read successful:\n");
        for (size_t i = 0; i < batch_channels.size(); ++i) {
            ConsolePort::Printf("  %.*s: Raw=%u, Voltage=%.3fV\n",
                               static_cast<int>(batch_channels[i].length()), batch_channels[i].data(),
                               raw_values[i], voltages[i]);
        }
    } else {
        ConsolePort::Printf("Batch read failed: %d\n", static_cast<int>(result));
    }
}

/**
 * @brief Demonstrate statistics and diagnostics.
 */
void DemonstrateStatisticsAndDiagnostics() {
    ConsolePort::Printf("\n=== Statistics and Diagnostics ===\n");
    
    auto& adc_manager = GetAdcManager();
    
    // Get system statistics
    AdcSystemStatistics system_stats;
    hf_adc_err_t result = adc_manager.GetSystemStatistics(system_stats);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        ConsolePort::Printf("System Statistics:\n");
        ConsolePort::Printf("  Total Operations: %u\n", system_stats.total_operations);
        ConsolePort::Printf("  Successful Operations: %u\n", system_stats.successful_operations);
        ConsolePort::Printf("  Failed Operations: %u\n", system_stats.failed_operations);
        ConsolePort::Printf("  Communication Errors: %u\n", system_stats.communication_errors);
        ConsolePort::Printf("  Hardware Errors: %u\n", system_stats.hardware_errors);
    }
    
    // Get system health
    AdcSystemHealth system_health;
    result = adc_manager.GetSystemHealth(system_health);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        ConsolePort::Printf("System Health:\n");
        ConsolePort::Printf("  Overall Health: %s\n", system_health.overall_healthy ? "Healthy" : "Unhealthy");
        ConsolePort::Printf("  Active Channels: %u\n", system_health.active_channels);
        ConsolePort::Printf("  Error Rate: %.2f%%\n", system_health.error_rate_percent);
        ConsolePort::Printf("  Uptime: %llu seconds\n", system_health.uptime_seconds);
    }
    
    // Get channel-specific statistics
    std::string test_channel = "TMC9660_AIN3";
    if (adc_manager.Contains(test_channel)) {
        BaseAdc::AdcStatistics channel_stats;
        result = adc_manager.GetStatistics(test_channel, channel_stats);
        if (result == hf_adc_err_t::ADC_SUCCESS) {
            ConsolePort::Printf("\nChannel Statistics for %s:\n", test_channel.c_str());
            ConsolePort::Printf("  Total Conversions: %u\n", channel_stats.totalConversions);
            ConsolePort::Printf("  Successful Conversions: %u\n", channel_stats.successfulConversions);
            ConsolePort::Printf("  Failed Conversions: %u\n", channel_stats.failedConversions);
            ConsolePort::Printf("  Average Conversion Time: %u us\n", channel_stats.averageConversionTimeUs);
        }
    }
}

/**
 * @brief Demonstrate error handling and recovery.
 */
void DemonstrateErrorHandling() {
    ConsolePort::Printf("\n=== Error Handling and Recovery ===\n");
    
    auto& adc_manager = GetAdcManager();
    
    // Try to read from a non-existent channel
    std::string non_existent_channel = "NON_EXISTENT_CHANNEL";
    float voltage;
    hf_adc_err_t result = adc_manager.ReadChannelV(non_existent_channel, voltage);
    ConsolePort::Printf("Reading from non-existent channel: %d\n", static_cast<int>(result));
    
    // Try to read from a valid channel with invalid parameters
    std::string valid_channel = "TMC9660_AIN3";
    if (adc_manager.Contains(valid_channel)) {
        result = adc_manager.ReadChannelV(valid_channel, voltage, 0, 1000); // Invalid sample count
        ConsolePort::Printf("Reading with invalid parameters: %d\n", static_cast<int>(result));
    }
    
    // Demonstrate recovery by reading from a valid channel
    if (adc_manager.Contains(valid_channel)) {
        result = adc_manager.ReadChannelV(valid_channel, voltage);
        ConsolePort::Printf("Recovery read from valid channel: %d (voltage=%.3fV)\n", 
                           static_cast<int>(result), voltage);
    }
}

/**
 * @brief Main example function.
 */
void RunAdcManagerExample() {
    ConsolePort::Printf("=== AdcManager Example - Enhanced TMC9660 Support ===\n");
    
    // Demonstrate all features
    DemonstrateBasicReading();
    DemonstrateTmc9660Channels();
    DemonstrateBatchReading();
    DemonstrateStatisticsAndDiagnostics();
    DemonstrateErrorHandling();
    
    ConsolePort::Printf("\n=== Example Complete ===\n");
}

//==============================================================================
// MAIN FUNCTION (for standalone testing)
//==============================================================================

#ifdef ADC_MANAGER_EXAMPLE_STANDALONE
int main() {
    RunAdcManagerExample();
    return 0;
}
#endif 