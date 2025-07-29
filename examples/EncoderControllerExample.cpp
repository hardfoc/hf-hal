/**
 * @file EncoderControllerExample.cpp
 * @brief Comprehensive example demonstrating EncoderController usage with AS5047U handlers.
 *
 * This example shows how to:
 * - Initialize the EncoderController singleton
 * - Access onboard AS5047U encoder
 * - Create external AS5047U devices
 * - Read angle and velocity measurements
 * - Monitor encoder health and diagnostics
 * - Handle multiple encoders simultaneously
 * - Use encoder data for closed-loop control
 *
 * Hardware Requirements:
 * - HardFOC board with onboard AS5047U encoder
 * - Optional: External AS5047U encoders connected via SPI
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

#include "component-handlers/EncoderController.h"
#include "component-handlers/CommChannelsManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

// Example configuration
constexpr uint8_t EXAMPLE_RUNTIME_SECONDS = 30;
constexpr uint16_t MEASUREMENT_INTERVAL_MS = 100;
constexpr double VELOCITY_THRESHOLD_RPM = 10.0;

/**
 * @brief Basic encoder reading example
 */
void BasicEncoderExample() {
    std::cout << "\n=== BASIC ENCODER EXAMPLE ===\n";
    
    // Get EncoderController singleton and ensure initialization
    auto& encoder_controller = EncoderController::GetInstance();
    if (!encoder_controller.EnsureInitialized()) {
        std::cout << "Failed to initialize EncoderController\n";
        return;
    }
    
    std::cout << "EncoderController initialized successfully\n";
    std::cout << "Active encoders: " << static_cast<int>(encoder_controller.GetDeviceCount()) << "\n\n";
    
    // Read from onboard encoder (device 0)
    uint16_t angle_lsb;
    double angle_degrees;
    double velocity_rpm;
    
    As5047uError result1 = encoder_controller.ReadAngle(0, angle_lsb);
    As5047uError result2 = encoder_controller.ReadAngleDegrees(0, angle_degrees);
    As5047uError result3 = encoder_controller.ReadVelocityRPM(0, velocity_rpm);
    
    if (result1 == As5047uError::SUCCESS) {
        std::cout << "Onboard Encoder (Device 0):\n";
        std::cout << "  Raw Angle: " << angle_lsb << " LSB\n";
        std::cout << "  Angle: " << std::fixed << std::setprecision(2) << angle_degrees << "°\n";
        
        if (result3 == As5047uError::SUCCESS) {
            std::cout << "  Velocity: " << std::fixed << std::setprecision(1) << velocity_rpm << " RPM\n";
        }
    } else {
        std::cout << "Failed to read onboard encoder: " << As5047uErrorToString(result1) << "\n";
    }
    
    // Check encoder health
    As5047uDiagnostics diagnostics;
    if (encoder_controller.ReadDiagnostics(0, diagnostics) == As5047uError::SUCCESS) {
        std::cout << "  Health Status:\n";
        std::cout << "    Magnetic Field: " << (diagnostics.magnetic_field_ok ? "OK" : "ERROR") << "\n";
        std::cout << "    Communication: " << (diagnostics.communication_ok ? "OK" : "ERROR") << "\n";
        std::cout << "    AGC Warning: " << (diagnostics.agc_warning ? "YES" : "NO") << "\n";
    }
}

/**
 * @brief Multi-encoder reading example
 */
void MultiEncoderExample() {
    std::cout << "\n=== MULTI-ENCODER EXAMPLE ===\n";
    
    auto& encoder_controller = EncoderController::GetInstance();
    
    // Try to create external encoders (this may fail if hardware not present)
    std::cout << "Attempting to create external encoders...\n";
    
    // Note: These would require actual external SPI device IDs
    // For demonstration, we'll show the pattern
    /*
    bool ext1_created = encoder_controller.CreateExternalDevice(1, SpiDeviceId::EXTERNAL_DEVICE_1);
    bool ext2_created = encoder_controller.CreateExternalDevice(2, SpiDeviceId::EXTERNAL_DEVICE_2);
    
    if (ext1_created) std::cout << "External encoder 1 created successfully\n";
    if (ext2_created) std::cout << "External encoder 2 created successfully\n";
    */
    
    // Read all active encoders
    std::vector<uint16_t> angles;
    std::vector<uint8_t> device_indices;
    std::vector<As5047uError> errors = encoder_controller.ReadAllAngles(angles, device_indices);
    
    std::cout << "\nAll Active Encoders:\n";
    for (size_t i = 0; i < device_indices.size(); ++i) {
        std::cout << "  Device " << static_cast<int>(device_indices[i]) << ": ";
        
        if (errors[i] == As5047uError::SUCCESS) {
            double angle_deg = As5047uHandler::LSBToDegrees(angles[i]);
            std::cout << std::fixed << std::setprecision(2) << angle_deg << "° (" << angles[i] << " LSB)\n";
        } else {
            std::cout << "ERROR - " << As5047uErrorToString(errors[i]) << "\n";
        }
    }
    
    // Read all velocities
    std::vector<double> velocities_rpm;
    std::vector<As5047uError> vel_errors = encoder_controller.ReadAllVelocities(velocities_rpm, device_indices);
    
    std::cout << "\nVelocity Readings:\n";
    for (size_t i = 0; i < device_indices.size(); ++i) {
        std::cout << "  Device " << static_cast<int>(device_indices[i]) << ": ";
        
        if (vel_errors[i] == As5047uError::SUCCESS) {
            std::cout << std::fixed << std::setprecision(1) << velocities_rpm[i] << " RPM\n";
        } else {
            std::cout << "ERROR - " << As5047uErrorToString(vel_errors[i]) << "\n";
        }
    }
}

/**
 * @brief Continuous monitoring example
 */
void ContinuousMonitoringExample() {
    std::cout << "\n=== CONTINUOUS MONITORING EXAMPLE ===\n";
    std::cout << "Monitoring encoders for " << EXAMPLE_RUNTIME_SECONDS << " seconds...\n";
    std::cout << "Rotate the encoder to see real-time data\n\n";
    
    auto& encoder_controller = EncoderController::GetInstance();
    
    auto start_time = std::chrono::steady_clock::now();
    auto last_measurement = start_time;
    
    // Statistics tracking
    uint32_t measurement_count = 0;
    double min_angle = 360.0;
    double max_angle = 0.0;
    double total_rotation = 0.0;
    double last_angle = 0.0;
    bool first_measurement = true;
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        
        if (elapsed.count() >= EXAMPLE_RUNTIME_SECONDS) {
            break;
        }
        
        // Take measurement every MEASUREMENT_INTERVAL_MS
        auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_measurement);
        if (time_since_last.count() >= MEASUREMENT_INTERVAL_MS) {
            double angle_degrees;
            double velocity_rpm;
            
            As5047uError angle_result = encoder_controller.ReadAngleDegrees(0, angle_degrees);
            As5047uError velocity_result = encoder_controller.ReadVelocityRPM(0, velocity_rpm);
            
            if (angle_result == As5047uError::SUCCESS) {
                measurement_count++;
                
                // Update statistics
                if (angle_degrees < min_angle) min_angle = angle_degrees;
                if (angle_degrees > max_angle) max_angle = angle_degrees;
                
                // Calculate total rotation (handle wraparound)
                if (!first_measurement) {
                    double angle_diff = angle_degrees - last_angle;
                    
                    // Handle 360° wraparound
                    if (angle_diff > 180.0) {
                        angle_diff -= 360.0;
                    } else if (angle_diff < -180.0) {
                        angle_diff += 360.0;
                    }
                    
                    total_rotation += std::abs(angle_diff);
                }
                
                last_angle = angle_degrees;
                first_measurement = false;
                
                // Display current readings
                std::cout << "\rTime: " << std::setw(2) << elapsed.count() << "s | "
                         << "Angle: " << std::fixed << std::setprecision(1) << std::setw(6) << angle_degrees << "° | ";
                
                if (velocity_result == As5047uError::SUCCESS) {
                    std::cout << "Velocity: " << std::setw(6) << velocity_rpm << " RPM | ";
                    
                    // Indicate if encoder is moving
                    if (std::abs(velocity_rpm) > VELOCITY_THRESHOLD_RPM) {
                        std::cout << "MOVING";
                    } else {
                        std::cout << "STATIC";
                    }
                } else {
                    std::cout << "Velocity: ERROR";
                }
                
                std::cout << std::flush;
            } else {
                std::cout << "\rEncoder read error: " << As5047uErrorToString(angle_result) << std::flush;
            }
            
            last_measurement = current_time;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Display final statistics
    std::cout << "\n\n=== MONITORING STATISTICS ===\n";
    std::cout << "Total Measurements: " << measurement_count << "\n";
    std::cout << "Measurement Rate: " << (measurement_count * 1000.0 / (EXAMPLE_RUNTIME_SECONDS * 1000)) << " Hz\n";
    std::cout << "Angle Range: " << std::fixed << std::setprecision(1) << min_angle << "° to " << max_angle << "°\n";
    std::cout << "Total Rotation: " << std::fixed << std::setprecision(1) << total_rotation << "°\n";
    std::cout << "Average Rotation: " << std::fixed << std::setprecision(2) << (total_rotation / EXAMPLE_RUNTIME_SECONDS) << "°/s\n";
}

/**
 * @brief Encoder calibration example
 */
void EncoderCalibrationExample() {
    std::cout << "\n=== ENCODER CALIBRATION EXAMPLE ===\n";
    
    auto& encoder_controller = EncoderController::GetInstance();
    
    // Read current position
    double current_angle;
    if (encoder_controller.ReadAngleDegrees(0, current_angle) != As5047uError::SUCCESS) {
        std::cout << "Failed to read current encoder position\n";
        return;
    }
    
    std::cout << "Current encoder position: " << std::fixed << std::setprecision(2) << current_angle << "°\n";
    std::cout << "Setting current position as zero reference...\n";
    
    // Set current position as zero
    uint16_t current_lsb = As5047uHandler::DegreesToLSB(current_angle);
    As5047uError result = encoder_controller.SetZeroPosition(0, current_lsb);
    
    if (result == As5047uError::SUCCESS) {
        std::cout << "Zero position set successfully\n";
        
        // Verify the change
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        double new_angle;
        if (encoder_controller.ReadAngleDegrees(0, new_angle) == As5047uError::SUCCESS) {
            std::cout << "New encoder reading: " << std::fixed << std::setprecision(2) << new_angle << "°\n";
            std::cout << "Zero offset applied: " << std::fixed << std::setprecision(2) << (current_angle - new_angle) << "°\n";
        }
    } else {
        std::cout << "Failed to set zero position: " << As5047uErrorToString(result) << "\n";
    }
}

/**
 * @brief Health monitoring example
 */
void HealthMonitoringExample() {
    std::cout << "\n=== HEALTH MONITORING EXAMPLE ===\n";
    
    auto& encoder_controller = EncoderController::GetInstance();
    
    // Check overall system health
    bool all_healthy = encoder_controller.CheckAllDevicesHealth();
    std::cout << "Overall System Health: " << (all_healthy ? "HEALTHY" : "ISSUES DETECTED") << "\n\n";
    
    // Get detailed status report
    std::string status_report = encoder_controller.GetStatusReport();
    std::cout << status_report << "\n";
    
    // Dump comprehensive diagnostics
    std::cout << "=== DETAILED DIAGNOSTICS ===\n";
    encoder_controller.DumpAllDiagnostics();
}

/**
 * @brief Closed-loop position control example (conceptual)
 */
void ClosedLoopControlExample() {
    std::cout << "\n=== CLOSED-LOOP CONTROL EXAMPLE ===\n";
    std::cout << "This example shows how encoder feedback can be used for position control\n";
    
    auto& encoder_controller = EncoderController::GetInstance();
    
    // Set target position
    double target_angle = 180.0; // 180 degrees
    double tolerance = 1.0;      // ±1 degree tolerance
    
    std::cout << "Target Position: " << target_angle << "°\n";
    std::cout << "Position Tolerance: ±" << tolerance << "°\n\n";
    
    // Simple position control loop (conceptual - would need motor control)
    for (int i = 0; i < 50; ++i) {
        double current_angle;
        As5047uError result = encoder_controller.ReadAngleDegrees(0, current_angle);
        
        if (result == As5047uError::SUCCESS) {
            double position_error = target_angle - current_angle;
            
            // Handle angle wraparound
            if (position_error > 180.0) {
                position_error -= 360.0;
            } else if (position_error < -180.0) {
                position_error += 360.0;
            }
            
            std::cout << "Step " << std::setw(2) << i + 1 << ": "
                     << "Current=" << std::fixed << std::setprecision(1) << std::setw(6) << current_angle << "° | "
                     << "Error=" << std::setw(6) << position_error << "° | ";
            
            if (std::abs(position_error) <= tolerance) {
                std::cout << "TARGET REACHED ✓\n";
                break;
            } else {
                // Calculate control output (simple proportional control)
                double control_gain = 0.1;
                double control_output = control_gain * position_error;
                std::cout << "Control=" << std::setw(6) << control_output << "\n";
                
                // In a real system, this control output would drive a motor
                // For this example, we just simulate the movement
            }
        } else {
            std::cout << "Encoder read error: " << As5047uErrorToString(result) << "\n";
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief Main example function
 */
int main() {
    std::cout << "=== ENCODER CONTROLLER EXAMPLE ===\n";
    std::cout << "This example demonstrates EncoderController usage with AS5047U handlers\n";
    std::cout << "Hardware: HardFOC board with AS5047U encoder\n\n";
    
    try {
        // Initialize the logger
        Logger::GetInstance().SetLogLevel(LogLevel::INFO);
        
        // Run all examples
        BasicEncoderExample();
        MultiEncoderExample();
        HealthMonitoringExample();
        EncoderCalibrationExample();
        ContinuousMonitoringExample();
        ClosedLoopControlExample();
        
        std::cout << "\n=== EXAMPLE COMPLETED ===\n";
        std::cout << "All encoder examples executed successfully!\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Example failed with exception: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

/**
 * @brief Simple usage demonstration for quick reference
 */
void QuickUsageExample() {
    // Get EncoderController singleton
    auto& encoder = EncoderController::GetInstance();
    
    // Initialize
    if (!encoder.EnsureInitialized()) {
        std::cout << "Initialization failed\n";
        return;
    }
    
    // Read onboard encoder angle
    double angle_degrees;
    if (encoder.ReadAngleDegrees(0, angle_degrees) == As5047uError::SUCCESS) {
        std::cout << "Encoder angle: " << angle_degrees << "°\n";
    }
    
    // Read velocity
    double velocity_rpm;
    if (encoder.ReadVelocityRPM(0, velocity_rpm) == As5047uError::SUCCESS) {
        std::cout << "Encoder velocity: " << velocity_rpm << " RPM\n";
    }
    
    // Check health
    bool healthy = encoder.CheckAllDevicesHealth();
    std::cout << "System health: " << (healthy ? "OK" : "ERROR") << "\n";
}