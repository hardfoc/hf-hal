/**
 * @file As5047uHandlerExample.cpp
 * @brief Comprehensive example demonstrating AS5047U handler usage and capabilities.
 *
 * This example shows how to use the AS5047U handler with various features:
 * - Lazy initialization pattern
 * - Complete sensor measurements
 * - Configuration management
 * - Error handling and diagnostics
 * - Advanced features (DAEC, OTP programming)
 * - Integration with CommChannelsManager
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#include "As5047uHandler.h"
#include "CommChannelsManager.h"
#include "ConsolePort.h"
#include <cstdio>
#include <cmath>

//======================================================//
// EXAMPLE CONFIGURATION
//======================================================//

/**
 * @brief Create optimized AS5047U configuration for motor control application
 */
static As5047uConfig CreateMotorControlConfig() noexcept {
    As5047uConfig config = As5047uHandler::GetDefaultConfig();
    
    // Use 24-bit SPI frames for CRC protection
    config.frame_format = FrameFormat::SPI_24;
    config.crc_retries = 3;
    
    // Enable advanced features
    config.enable_daec = true;              // Dynamic Angle Error Compensation
    config.enable_adaptive_filter = true;   // Adaptive filtering for noise reduction
    
    // Configure for motor applications
    config.enable_abi_output = true;        // Incremental encoder output
    config.abi_resolution_bits = 14;        // Full 14-bit resolution
    config.enable_uvw_output = true;        // Commutation signals
    config.uvw_pole_pairs = 4;              // 4 pole pairs (common for BLDC motors)
    
    // Standard temperature mode (lower noise)
    config.high_temperature_mode = false;
    
    return config;
}

//======================================================//
// BASIC USAGE EXAMPLE
//======================================================//

/**
 * @brief Basic AS5047U sensor usage example
 */
void BasicAs5047uExample() {
    printf("\n=== AS5047U Basic Usage Example ===\n");
    
    // Get SPI interface from CommChannelsManager
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto* spi_interface = comm_manager.GetSpi(0);  // Get first SPI bus
    
    if (!spi_interface) {
        printf("ERROR: Failed to get SPI interface\n");
        return;
    }
    
    // Create AS5047U handler with default configuration
    auto as5047u_handler = CreateAs5047uHandler(*spi_interface);
    if (!as5047u_handler) {
        printf("ERROR: Failed to create AS5047U handler\n");
        return;
    }
    
    // Initialize sensor (lazy initialization)
    As5047uError result = as5047u_handler->Initialize();
    if (result != As5047uError::SUCCESS) {
        printf("ERROR: AS5047U initialization failed: %s\n", As5047uErrorToString(result));
        return;
    }
    
    printf("AS5047U sensor initialized successfully\n");
    
    // Read basic measurements
    uint16_t angle = 0;
    result = as5047u_handler->ReadAngle(angle);
    if (result == As5047uError::SUCCESS) {
        double angle_degrees = As5047uHandler::LSBToDegrees(angle);
        double angle_radians = As5047uHandler::LSBToRadians(angle);
        printf("Current angle: %u LSB (%.2f°, %.4f rad)\n", 
               angle, angle_degrees, angle_radians);
    } else {
        printf("ERROR: Failed to read angle: %s\n", As5047uErrorToString(result));
    }
    
    // Read velocity
    double velocity_rpm = 0.0;
    result = as5047u_handler->ReadVelocityRPM(velocity_rpm);
    if (result == As5047uError::SUCCESS) {
        printf("Current velocity: %.2f RPM\n", velocity_rpm);
    } else {
        printf("ERROR: Failed to read velocity: %s\n", As5047uErrorToString(result));
    }
    
    // Check magnetic field quality
    bool field_ok = false;
    result = as5047u_handler->IsMagneticFieldOK(field_ok);
    if (result == As5047uError::SUCCESS) {
        printf("Magnetic field status: %s\n", field_ok ? "OK" : "WEAK/STRONG");
    }
    
    printf("Basic example completed\n\n");
}

//======================================================//
// ADVANCED MEASUREMENTS EXAMPLE
//======================================================//

/**
 * @brief Advanced measurement and diagnostics example
 */
void AdvancedMeasurementsExample() {
    printf("\n=== AS5047U Advanced Measurements Example ===\n");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto* spi_interface = comm_manager.GetSpi(0);
    
    if (!spi_interface) {
        printf("ERROR: SPI interface not available\n");
        return;
    }
    
    // Create handler with motor control configuration
    As5047uConfig config = CreateMotorControlConfig();
    auto handler = CreateAs5047uHandler(*spi_interface, config);
    
    if (!handler || handler->Initialize() != As5047uError::SUCCESS) {
        printf("ERROR: Failed to initialize AS5047U with advanced config\n");
        return;
    }
    
    // Read complete measurement data
    As5047uMeasurement measurement;
    As5047uError result = handler->ReadMeasurement(measurement);
    
    if (result == As5047uError::SUCCESS && measurement.valid) {
        printf("=== Complete Sensor Measurement ===\n");
        printf("Compensated angle: %u LSB (%.3f°)\n", 
               measurement.angle_compensated, 
               As5047uHandler::LSBToDegrees(measurement.angle_compensated));
        printf("Raw angle:         %u LSB (%.3f°)\n", 
               measurement.angle_raw, 
               As5047uHandler::LSBToDegrees(measurement.angle_raw));
        printf("Velocity (raw):    %d LSB\n", measurement.velocity_raw);
        printf("Velocity (deg/s):  %.2f\n", measurement.velocity_deg_per_sec);
        printf("Velocity (rad/s):  %.4f\n", measurement.velocity_rad_per_sec);
        printf("Velocity (RPM):    %.2f\n", measurement.velocity_rpm);
        printf("AGC value:         %u (0-255)\n", measurement.agc_value);
        printf("Magnitude:         %u (0-16383)\n", measurement.magnitude);
        printf("Error flags:       0x%04X\n", measurement.error_flags);
        
        // Interpret AGC value
        if (measurement.agc_value < 30) {
            printf("WARNING: Magnetic field too strong (AGC < 30)\n");
        } else if (measurement.agc_value > 225) {
            printf("WARNING: Magnetic field too weak (AGC > 225)\n");
        } else {
            printf("INFO: Magnetic field strength optimal\n");
        }
        
    } else {
        printf("ERROR: Failed to read measurement or invalid data\n");
        if (measurement.error_flags != 0) {
            printf("Sensor error flags: 0x%04X\n", measurement.error_flags);
        }
    }
    
    // Read comprehensive diagnostics
    As5047uDiagnostics diagnostics;
    result = handler->ReadDiagnostics(diagnostics);
    if (result == As5047uError::SUCCESS) {
        printf("\n=== Sensor Diagnostics ===\n");
        printf("Magnetic field OK:        %s\n", diagnostics.magnetic_field_ok ? "YES" : "NO");
        printf("AGC warning:              %s\n", diagnostics.agc_warning ? "YES" : "NO");
        printf("CORDIC overflow:          %s\n", diagnostics.cordic_overflow ? "YES" : "NO");
        printf("Offset compensation:      %s\n", diagnostics.offset_compensation_ok ? "OK" : "FAILED");
        printf("Communication OK:         %s\n", diagnostics.communication_ok ? "YES" : "NO");
        printf("Total measurements:       %u\n", diagnostics.total_measurements);
        printf("Communication errors:     %u\n", diagnostics.communication_errors);
        
        if (diagnostics.total_measurements > 0) {
            double error_rate = (double)diagnostics.communication_errors / diagnostics.total_measurements * 100.0;
            printf("Error rate:               %.2f%%\n", error_rate);
        }
    }
    
    printf("Advanced measurements example completed\n\n");
}

//======================================================//
// CONFIGURATION AND CALIBRATION EXAMPLE
//======================================================//

/**
 * @brief Configuration management and calibration example
 */
void ConfigurationAndCalibrationExample() {
    printf("\n=== AS5047U Configuration & Calibration Example ===\n");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto* spi_interface = comm_manager.GetSpi(0);
    
    if (!spi_interface) {
        printf("ERROR: SPI interface not available\n");
        return;
    }
    
    auto handler = CreateAs5047uHandler(*spi_interface);
    if (!handler || handler->Initialize() != As5047uError::SUCCESS) {
        printf("ERROR: Failed to initialize AS5047U\n");
        return;
    }
    
    // Read current configuration
    As5047uConfig current_config;
    As5047uError result = handler->GetConfiguration(current_config);
    if (result == As5047uError::SUCCESS) {
        printf("Current configuration:\n");
        printf("  Frame format:         %s\n", 
               current_config.frame_format == FrameFormat::SPI_16 ? "16-bit" :
               current_config.frame_format == FrameFormat::SPI_24 ? "24-bit" : "32-bit");
        printf("  CRC retries:          %u\n", current_config.crc_retries);
        printf("  DAEC enabled:         %s\n", current_config.enable_daec ? "YES" : "NO");
        printf("  Adaptive filter:      %s\n", current_config.enable_adaptive_filter ? "YES" : "NO");
        printf("  Zero position:        %u LSB (%.2f°)\n", 
               current_config.zero_position, 
               As5047uHandler::LSBToDegrees(current_config.zero_position));
        printf("  ABI output:           %s (%u bits)\n", 
               current_config.enable_abi_output ? "ENABLED" : "DISABLED",
               current_config.abi_resolution_bits);
        printf("  UVW output:           %s (%u pole pairs)\n", 
               current_config.enable_uvw_output ? "ENABLED" : "DISABLED",
               current_config.uvw_pole_pairs);
        printf("  PWM output:           %s\n", current_config.enable_pwm_output ? "ENABLED" : "DISABLED");
        printf("  High temp mode:       %s\n", current_config.high_temperature_mode ? "150°C" : "125°C");
    }
    
    // Demonstrate calibration
    printf("\n=== Performing Calibration ===\n");
    
    // Read angle before calibration
    uint16_t angle_before = 0;
    handler->ReadAngle(angle_before);
    printf("Angle before calibration: %u LSB (%.2f°)\n", 
           angle_before, As5047uHandler::LSBToDegrees(angle_before));
    
    // Perform calibration (sets current position as zero)
    result = handler->PerformCalibration();
    if (result == As5047uError::SUCCESS) {
        printf("Calibration successful\n");
        
        // Read angle after calibration
        uint16_t angle_after = 0;
        handler->ReadAngle(angle_after);
        printf("Angle after calibration: %u LSB (%.2f°)\n", 
               angle_after, As5047uHandler::LSBToDegrees(angle_after));
        
        // Verify zero position was updated
        uint16_t zero_position = 0;
        handler->GetZeroPosition(zero_position);
        printf("New zero position: %u LSB\n", zero_position);
        
    } else {
        printf("ERROR: Calibration failed: %s\n", As5047uErrorToString(result));
    }
    
    // Demonstrate configuration update
    printf("\n=== Updating Configuration ===\n");
    As5047uConfig new_config = CreateMotorControlConfig();
    new_config.enable_abi_output = true;
    new_config.abi_resolution_bits = 12;  // Lower resolution for testing
    
    result = handler->UpdateConfiguration(new_config);
    if (result == As5047uError::SUCCESS) {
        printf("Configuration updated successfully\n");
        printf("ABI resolution changed to %u bits\n", new_config.abi_resolution_bits);
    } else {
        printf("ERROR: Configuration update failed: %s\n", As5047uErrorToString(result));
    }
    
    printf("Configuration and calibration example completed\n\n");
}

//======================================================//
// ADVANCED FEATURES EXAMPLE
//======================================================//

/**
 * @brief Advanced features demonstration (DAEC, filtering, OTP)
 */
void AdvancedFeaturesExample() {
    printf("\n=== AS5047U Advanced Features Example ===\n");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto* spi_interface = comm_manager.GetSpi(0);
    
    if (!spi_interface) {
        printf("ERROR: SPI interface not available\n");
        return;
    }
    
    auto handler = CreateAs5047uHandler(*spi_interface);
    if (!handler || handler->Initialize() != As5047uError::SUCCESS) {
        printf("ERROR: Failed to initialize AS5047U\n");
        return;
    }
    
    // Demonstrate DAEC comparison
    printf("=== Dynamic Angle Error Compensation (DAEC) Comparison ===\n");
    
    // Enable DAEC
    As5047uError result = handler->SetDAEC(true);
    if (result == As5047uError::SUCCESS) {
        uint16_t angle_with_daec = 0;
        handler->ReadAngle(angle_with_daec);
        
        uint16_t raw_angle = 0;
        handler->ReadRawAngle(raw_angle);
        
        printf("Angle with DAEC:    %u LSB (%.3f°)\n", 
               angle_with_daec, As5047uHandler::LSBToDegrees(angle_with_daec));
        printf("Raw angle:          %u LSB (%.3f°)\n", 
               raw_angle, As5047uHandler::LSBToDegrees(raw_angle));
        
        int16_t compensation = static_cast<int16_t>(angle_with_daec) - static_cast<int16_t>(raw_angle);
        printf("DAEC compensation:  %d LSB (%.3f°)\n", 
               compensation, As5047uHandler::LSBToDegrees(std::abs(compensation)));
    }
    
    // Test adaptive filtering
    printf("\n=== Adaptive Filtering Test ===\n");
    
    // Disable adaptive filter
    handler->SetAdaptiveFilter(false);
    printf("Adaptive filter disabled\n");
    
    // Take several measurements without filter
    double velocity_sum_no_filter = 0.0;
    const int num_samples = 10;
    for (int i = 0; i < num_samples; i++) {
        double velocity = 0.0;
        handler->ReadVelocityDegPerSec(velocity);
        velocity_sum_no_filter += std::abs(velocity);
    }
    double avg_noise_no_filter = velocity_sum_no_filter / num_samples;
    
    // Enable adaptive filter
    handler->SetAdaptiveFilter(true);
    printf("Adaptive filter enabled\n");
    
    // Take several measurements with filter
    double velocity_sum_with_filter = 0.0;
    for (int i = 0; i < num_samples; i++) {
        double velocity = 0.0;
        handler->ReadVelocityDegPerSec(velocity);
        velocity_sum_with_filter += std::abs(velocity);
    }
    double avg_noise_with_filter = velocity_sum_with_filter / num_samples;
    
    printf("Average velocity noise without filter: %.2f deg/s\n", avg_noise_no_filter);
    printf("Average velocity noise with filter:    %.2f deg/s\n", avg_noise_with_filter);
    
    if (avg_noise_with_filter < avg_noise_no_filter) {
        double improvement = ((avg_noise_no_filter - avg_noise_with_filter) / avg_noise_no_filter) * 100.0;
        printf("Filter improvement: %.1f%% noise reduction\n", improvement);
    }
    
    // Demonstrate interface configuration
    printf("\n=== Interface Configuration ===\n");
    
    // Configure for motor control application
    result = handler->ConfigureInterface(true, true, false);  // ABI + UVW, no PWM
    if (result == As5047uError::SUCCESS) {
        printf("Configured for motor control: ABI + UVW outputs enabled\n");
        
        // Set ABI resolution
        handler->SetABIResolution(14);
        printf("ABI resolution set to 14 bits (16384 pulses per revolution)\n");
        
        // Set UVW pole pairs
        handler->SetUVWPolePairs(4);
        printf("UVW configured for 4 pole pairs\n");
    }
    
    // WARNING: OTP programming example (commented out for safety)
    printf("\n=== OTP Programming (DEMONSTRATION ONLY) ===\n");
    printf("WARNING: OTP programming is a ONE-TIME operation!\n");
    printf("This would permanently program current settings:\n");
    
    As5047uConfig current_config;
    handler->GetConfiguration(current_config);
    printf("  - Zero position: %u LSB\n", current_config.zero_position);
    printf("  - DAEC: %s\n", current_config.enable_daec ? "Enabled" : "Disabled");
    printf("  - Adaptive filter: %s\n", current_config.enable_adaptive_filter ? "Enabled" : "Disabled");
    printf("  - Interfaces: ABI=%s, UVW=%s, PWM=%s\n",
           current_config.enable_abi_output ? "ON" : "OFF",
           current_config.enable_uvw_output ? "ON" : "OFF",
           current_config.enable_pwm_output ? "ON" : "OFF");
    
    printf("\nTo actually program OTP, uncomment the following line:\n");
    printf("// result = handler->ProgramOTP();\n");
    printf("CAUTION: Only program OTP with verified configuration!\n");
    
    printf("Advanced features example completed\n\n");
}

//======================================================//
// CONTINUOUS MONITORING EXAMPLE
//======================================================//

/**
 * @brief Continuous sensor monitoring with error handling
 */
void ContinuousMonitoringExample() {
    printf("\n=== AS5047U Continuous Monitoring Example ===\n");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto* spi_interface = comm_manager.GetSpi(0);
    
    if (!spi_interface) {
        printf("ERROR: SPI interface not available\n");
        return;
    }
    
    auto handler = CreateAs5047uHandler(*spi_interface, CreateMotorControlConfig());
    if (!handler || handler->Initialize() != As5047uError::SUCCESS) {
        printf("ERROR: Failed to initialize AS5047U for monitoring\n");
        return;
    }
    
    printf("Starting continuous monitoring (10 samples)...\n");
    printf("Sample | Angle(°)  | Vel(RPM) | AGC | Mag  | Errors | Status\n");
    printf("-------|-----------|----------|-----|------|--------|---------\n");
    
    for (int sample = 1; sample <= 10; sample++) {
        As5047uMeasurement measurement;
        As5047uError result = handler->ReadMeasurement(measurement);
        
        if (result == As5047uError::SUCCESS) {
            printf("  %2d   | %8.2f | %8.1f | %3u | %4u | 0x%04X | %s\n",
                   sample,
                   As5047uHandler::LSBToDegrees(measurement.angle_compensated),
                   measurement.velocity_rpm,
                   measurement.agc_value,
                   measurement.magnitude,
                   measurement.error_flags,
                   measurement.valid ? "OK" : "ERROR");
                   
            // Check for specific warnings
            if (measurement.agc_value < 30 || measurement.agc_value > 225) {
                printf("         WARNING: Magnetic field strength suboptimal\n");
            }
            if (measurement.error_flags & static_cast<uint16_t>(AS5047U_Error::CordicOverflow)) {
                printf("         WARNING: CORDIC overflow detected\n");
            }
            
        } else {
            printf("  %2d   |    ERROR  |    ERROR | --- | ---- |   ---- | FAIL\n", sample);
            printf("         Error: %s\n", As5047uErrorToString(result));
        }
        
        // Small delay between measurements
        // In real application, this would be your control loop timing
        // vTaskDelay(pdMS_TO_TICKS(100));  // 100ms delay
    }
    
    // Final diagnostics summary
    As5047uDiagnostics diagnostics;
    if (handler->ReadDiagnostics(diagnostics) == As5047uError::SUCCESS) {
        printf("\n=== Monitoring Summary ===\n");
        printf("Total measurements:     %u\n", diagnostics.total_measurements);
        printf("Communication errors:   %u\n", diagnostics.communication_errors);
        if (diagnostics.total_measurements > 0) {
            double success_rate = (double)(diagnostics.total_measurements - diagnostics.communication_errors) / 
                                 diagnostics.total_measurements * 100.0;
            printf("Success rate:           %.1f%%\n", success_rate);
        }
        printf("Sensor health:          %s\n", 
               (diagnostics.magnetic_field_ok && diagnostics.communication_ok && 
                diagnostics.offset_compensation_ok) ? "GOOD" : "CHECK REQUIRED");
    }
    
    printf("Continuous monitoring example completed\n\n");
}

//======================================================//
// MAIN EXAMPLE RUNNER
//======================================================//

/**
 * @brief Run all AS5047U handler examples
 */
void RunAs5047uHandlerExamples() {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║                   AS5047U Handler Examples                  ║\n");
    printf("║              Magnetic Rotary Position Sensor                ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    
    // Initialize communication manager
    auto& comm_manager = CommChannelsManager::GetInstance();
    if (!comm_manager.IsInitialized()) {
        printf("ERROR: CommChannelsManager not initialized\n");
        return;
    }
    
    // Run examples
    BasicAs5047uExample();
    AdvancedMeasurementsExample();
    ConfigurationAndCalibrationExample();
    AdvancedFeaturesExample();
    ContinuousMonitoringExample();
    
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║                    Examples Completed                       ║\n");
    printf("║   AS5047U Handler demonstrates industry-leading embedded    ║\n");
    printf("║   architecture with lazy initialization, shared pointers,   ║\n");
    printf("║   exception-free design, and comprehensive sensor features  ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
}
