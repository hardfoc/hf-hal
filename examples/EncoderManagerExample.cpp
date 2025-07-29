/**
 * @file EncoderManagerExample.cpp
 * @brief Comprehensive example demonstrating EncoderManager usage with AS5047U encoder handlers.
 *
 * This example shows how to:
 * - Initialize the EncoderManager singleton (following ImuManager pattern)
 * - Access onboard AS5047U encoder (automatic initialization)
 * - Create external AS5047U devices dynamically
 * - Read angle and velocity measurements from multiple encoders
 * - Monitor encoder health and diagnostics
 * - Handle multiple encoders simultaneously
 * - Use encoder data for closed-loop control applications
 * - Handle interrupts for high-frequency encoder updates
 *
 * Hardware Requirements:
 * - HardFOC board with onboard AS5047U encoder
 * - Optional: External AS5047U encoders connected via SPI
 * - Proper magnetic field for sensor operation (4-7mm magnet distance)
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
#include <functional>

#include "component-handlers/EncoderManager.h"
#include "component-handlers/CommChannelsManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

// Example configuration
constexpr uint8_t EXAMPLE_RUNTIME_SECONDS = 30;
constexpr uint16_t MEASUREMENT_INTERVAL_MS = 50;
constexpr double VELOCITY_THRESHOLD_RPM = 5.0;

/**
 * @brief Basic encoder access example (mirrors ImuManager pattern)
 */
void BasicEncoderExample() {
    std::cout << "\n=== BASIC ENCODER EXAMPLE ===\n";
    
    // Get EncoderManager singleton and ensure initialization (like ImuManager)
    auto& encoder_mgr = EncoderManager::GetInstance();
    if (!encoder_mgr.EnsureInitialized()) {
        std::cout << "Failed to initialize EncoderManager\n";
        return;
    }
    
    std::cout << "EncoderManager initialized successfully\n";
    std::cout << "Active encoders: " << static_cast<int>(encoder_mgr.GetDeviceCount()) << "\n\n";
    
    // Access onboard AS5047U (device 0) - auto-created like onboard BNO08x
    As5047uHandler* onboard_handler = encoder_mgr.GetAs5047uHandler(0);
    if (onboard_handler) {
        std::cout << "Onboard AS5047U Handler (Device 0):\n";
        
        // Read angle using handler directly
        uint16_t angle_lsb;
        As5047uError result = onboard_handler->ReadAngle(angle_lsb);
        
        if (result == As5047uError::SUCCESS) {
            double angle_deg = As5047uHandler::LSBToDegrees(angle_lsb);
            std::cout << "  Raw Angle: " << angle_lsb << " LSB\n";
            std::cout << "  Angle: " << std::fixed << std::setprecision(2) << angle_deg << "°\n";
            
            // Read velocity
            double velocity_rpm;
            if (onboard_handler->ReadVelocityRPM(velocity_rpm) == As5047uError::SUCCESS) {
                std::cout << "  Velocity: " << std::fixed << std::setprecision(1) << velocity_rpm << " RPM\n";
            }
            
            // Read diagnostics
            As5047uDiagnostics diagnostics;
            if (onboard_handler->ReadDiagnostics(diagnostics) == As5047uError::SUCCESS) {
                std::cout << "  Health Status:\n";
                std::cout << "    Magnetic Field: " << (diagnostics.magnetic_field_ok ? "OK" : "ERROR") << "\n";
                std::cout << "    Communication: " << (diagnostics.communication_ok ? "OK" : "ERROR") << "\n";
                std::cout << "    AGC Warning: " << (diagnostics.agc_warning ? "YES" : "NO") << "\n";
            }
        } else {
            std::cout << "Failed to read onboard encoder: " << As5047uErrorToString(result) << "\n";
        }
    } else {
        std::cout << "Onboard AS5047U handler not available\n";
    }
    
    // Access driver directly (like GetBno085Driver)
    auto sensor_driver = encoder_mgr.GetAs5047uDriver(0);
    if (sensor_driver) {
        std::cout << "  Direct AS5047U driver access: Available\n";
    }
}

/**
 * @brief External device creation example (mirrors ImuManager external device pattern)
 */
void ExternalDeviceExample() {
    std::cout << "\n=== EXTERNAL DEVICE EXAMPLE ===\n";
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Check external slot availability
    std::cout << "External Device Slot Availability:\n";
    for (uint8_t i = 1; i <= 3; ++i) {
        bool available = encoder_mgr.IsExternalSlotAvailable(i);
        std::cout << "  Slot " << static_cast<int>(i) << ": " << (available ? "AVAILABLE" : "OCCUPIED") << "\n";
    }
    
    // Try to create external AS5047U devices (like ImuManager creates external BNO08x)
    std::cout << "\nAttempting to create external AS5047U devices...\n";
    
    // Method 1: Using SPI device ID (like ImuManager SPI creation)
    /*
    bool ext1_created = encoder_mgr.CreateExternalAs5047uDevice(1, SpiDeviceId::EXTERNAL_DEVICE_1);
    if (ext1_created) {
        std::cout << "External AS5047U device 1 created successfully\n";
        
        // Access the external handler
        As5047uHandler* ext_handler = encoder_mgr.GetAs5047uHandler(1);
        if (ext_handler) {
            // Configure external encoder
            ext_handler->SetZeroPosition(0);  // Set zero reference
            std::cout << "External encoder 1 configured\n";
        }
    } else {
        std::cout << "Failed to create external AS5047U device 1\n";
    }
    */
    
    // Method 2: Using direct SPI interface (like ImuManager direct interface creation)
    /*
    auto& comm_mgr = CommChannelsManager::GetInstance();
    BaseSpi* external_spi = comm_mgr.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_2);
    if (external_spi) {
        bool ext2_created = encoder_mgr.CreateExternalAs5047uDevice(2, *external_spi);
        if (ext2_created) {
            std::cout << "External AS5047U device 2 (direct SPI) created successfully\n";
        }
    }
    */
    
    std::cout << "Note: External device creation requires actual hardware connections\n";
    
    // Show all active devices
    auto active_devices = encoder_mgr.GetActiveDeviceIndices();
    std::cout << "\nActive encoder devices:\n";
    for (uint8_t device_idx : active_devices) {
        std::string device_type = encoder_mgr.GetDeviceType(device_idx);
        std::cout << "  Device " << static_cast<int>(device_idx) << ": " << device_type;
        if (device_idx == 0) {
            std::cout << " (Onboard)";
        } else {
            std::cout << " (External)";
        }
        std::cout << "\n";
    }
}

/**
 * @brief High-level encoder operations example (using manager convenience methods)
 */
void HighLevelOperationsExample() {
    std::cout << "\n=== HIGH-LEVEL OPERATIONS EXAMPLE ===\n";
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Read angle using manager convenience method
    double angle_degrees;
    As5047uError result = encoder_mgr.ReadAngleDegrees(0, angle_degrees);
    if (result == As5047uError::SUCCESS) {
        std::cout << "Onboard encoder angle: " << std::fixed << std::setprecision(2) << angle_degrees << "°\n";
    }
    
    // Read velocity using manager convenience method
    double velocity_rpm;
    result = encoder_mgr.ReadVelocityRPM(0, velocity_rpm);
    if (result == As5047uError::SUCCESS) {
        std::cout << "Onboard encoder velocity: " << std::fixed << std::setprecision(1) << velocity_rpm << " RPM\n";
    }
    
    // Set zero position using manager convenience method
    std::cout << "Setting current position as zero reference...\n";
    uint16_t current_lsb = As5047uHandler::DegreesToLSB(angle_degrees);
    result = encoder_mgr.SetZeroPosition(0, current_lsb);
    if (result == As5047uError::SUCCESS) {
        std::cout << "Zero position set successfully\n";
        
        // Verify the change
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double new_angle;
        if (encoder_mgr.ReadAngleDegrees(0, new_angle) == As5047uError::SUCCESS) {
            std::cout << "New angle reading: " << std::fixed << std::setprecision(2) << new_angle << "°\n";
        }
    }
    
    // Read all active encoders at once
    std::vector<uint16_t> angles;
    std::vector<uint8_t> device_indices;
    std::vector<As5047uError> errors = encoder_mgr.ReadAllAngles(angles, device_indices);
    
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
    
    // Check system health
    bool all_healthy = encoder_mgr.CheckAllDevicesHealth();
    std::cout << "\nSystem Health: " << (all_healthy ? "HEALTHY" : "ISSUES DETECTED") << "\n";
}

/**
 * @brief Continuous monitoring example with statistics (like ImuManager sensor monitoring)
 */
void ContinuousMonitoringExample() {
    std::cout << "\n=== CONTINUOUS MONITORING EXAMPLE ===\n";
    std::cout << "Monitoring encoders for " << EXAMPLE_RUNTIME_SECONDS << " seconds...\n";
    std::cout << "Rotate the encoder to see real-time data\n\n";
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
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
            
            As5047uError angle_result = encoder_mgr.ReadAngleDegrees(0, angle_degrees);
            As5047uError velocity_result = encoder_mgr.ReadVelocityRPM(0, velocity_rpm);
            
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
    std::cout << "Measurement Rate: " << std::fixed << std::setprecision(1) 
              << (measurement_count * 1000.0 / (EXAMPLE_RUNTIME_SECONDS * 1000)) << " Hz\n";
    std::cout << "Angle Range: " << std::fixed << std::setprecision(1) << min_angle << "° to " << max_angle << "°\n";
    std::cout << "Total Rotation: " << std::fixed << std::setprecision(1) << total_rotation << "°\n";
    std::cout << "Average Rotation: " << std::fixed << std::setprecision(2) 
              << (total_rotation / EXAMPLE_RUNTIME_SECONDS) << "°/s\n";
}

/**
 * @brief Interrupt handling example (mirrors ImuManager interrupt pattern)
 */
void InterruptExample() {
    std::cout << "\n=== INTERRUPT EXAMPLE ===\n";
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Configure interrupt callback (like ImuManager)
    std::function<void()> interrupt_callback = []() {
        // Keep minimal - we're in interrupt context
        static uint32_t callback_count = 0;
        callback_count++;
        // Could trigger a semaphore or set a flag here
    };
    
    // Configure interrupt for onboard encoder
    bool interrupt_configured = encoder_mgr.ConfigureInterrupt(0, interrupt_callback);
    if (interrupt_configured) {
        std::cout << "Interrupt configured for onboard encoder\n";
        
        // Enable interrupt
        if (encoder_mgr.EnableInterrupt(0)) {
            std::cout << "Interrupt enabled\n";
            
            // Monitor interrupt activity
            std::cout << "Monitoring interrupts for 5 seconds...\n";
            for (int i = 0; i < 5; ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                uint32_t interrupt_count = encoder_mgr.GetInterruptCount(0);
                std::cout << "Interrupt count: " << interrupt_count << "\n";
            }
            
            // Wait for interrupt with timeout
            std::cout << "Waiting for next interrupt (5 second timeout)...\n";
            bool interrupt_received = encoder_mgr.WaitForInterrupt(0, 5000);
            std::cout << "Interrupt " << (interrupt_received ? "received" : "timed out") << "\n";
            
            // Disable interrupt
            encoder_mgr.DisableInterrupt(0);
            std::cout << "Interrupt disabled\n";
        }
    } else {
        std::cout << "Interrupt configuration not available (implementation pending)\n";
    }
}

/**
 * @brief Device initialization and status example
 */
void DeviceStatusExample() {
    std::cout << "\n=== DEVICE STATUS EXAMPLE ===\n";
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Initialize all devices
    std::cout << "Initializing all devices...\n";
    std::vector<bool> init_results = encoder_mgr.InitializeAllDevices();
    
    std::cout << "Initialization results:\n";
    auto active_indices = encoder_mgr.GetActiveDeviceIndices();
    for (size_t i = 0; i < init_results.size() && i < active_indices.size(); ++i) {
        std::cout << "  Device " << static_cast<int>(active_indices[i]) << ": " 
                  << (init_results[i] ? "SUCCESS" : "FAILED") << "\n";
    }
    
    // Get initialization status
    std::vector<bool> status = encoder_mgr.GetInitializationStatus();
    std::cout << "\nCurrent initialization status:\n";
    for (size_t i = 0; i < status.size() && i < active_indices.size(); ++i) {
        std::cout << "  Device " << static_cast<int>(active_indices[i]) << ": " 
                  << (status[i] ? "INITIALIZED" : "NOT INITIALIZED") << "\n";
    }
    
    // Get available devices info
    auto available_devices = encoder_mgr.GetAvailableDevices();
    std::cout << "\nAvailable devices:\n";
    for (const auto& device : available_devices) {
        std::cout << "  " << device << "\n";
    }
    
    // Check device validity
    std::cout << "\nDevice validity check:\n";
    for (uint8_t i = 0; i < 4; ++i) {
        bool valid = encoder_mgr.IsDeviceValid(i);
        std::cout << "  Device " << static_cast<int>(i) << ": " << (valid ? "VALID" : "INVALID") << "\n";
    }
    
    // Dump comprehensive statistics (like ImuManager)
    std::cout << "\n=== COMPREHENSIVE STATISTICS ===\n";
    encoder_mgr.DumpStatistics();
}

/**
 * @brief Closed-loop control integration example
 */
void ClosedLoopControlExample() {
    std::cout << "\n=== CLOSED-LOOP CONTROL EXAMPLE ===\n";
    std::cout << "This example shows encoder feedback integration for motor control\n";
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Set target position
    double target_angle = 180.0; // 180 degrees
    double tolerance = 2.0;      // ±2 degree tolerance
    
    std::cout << "Target Position: " << target_angle << "°\n";
    std::cout << "Position Tolerance: ±" << tolerance << "°\n\n";
    
    // Simple position control loop (conceptual - would integrate with MotorController)
    for (int i = 0; i < 20; ++i) {
        double current_angle;
        As5047uError result = encoder_mgr.ReadAngleDegrees(0, current_angle);
        
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
                
                // In a real system, this would be:
                // auto& motor_mgr = MotorController::GetInstance();
                // motor_mgr.SetVelocity(0, control_output);
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
    std::cout << "=== ENCODER MANAGER EXAMPLE ===\n";
    std::cout << "This example demonstrates EncoderManager usage following ImuManager patterns\n";
    std::cout << "Hardware: HardFOC board with AS5047U encoder\n\n";
    
    try {
        // Initialize the logger
        Logger::GetInstance().SetLogLevel(LogLevel::INFO);
        
        // Run all examples
        BasicEncoderExample();
        ExternalDeviceExample();
        HighLevelOperationsExample();
        DeviceStatusExample();
        InterruptExample();
        ContinuousMonitoringExample();
        ClosedLoopControlExample();
        
        std::cout << "\n=== EXAMPLE COMPLETED ===\n";
        std::cout << "All encoder manager examples executed successfully!\n";
        
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
    // Get EncoderManager singleton (like ImuManager)
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Ensure initialization (automatic onboard device creation)
    if (!encoder_mgr.EnsureInitialized()) {
        std::cout << "Initialization failed\n";
        return;
    }
    
    // Access onboard encoder handler (device 0)
    As5047uHandler* handler = encoder_mgr.GetAs5047uHandler(0);
    if (handler) {
        // Read encoder angle
        uint16_t angle;
        if (handler->ReadAngle(angle) == As5047uError::SUCCESS) {
            std::cout << "Encoder angle: " << angle << " LSB\n";
        }
    }
    
    // Or use high-level convenience methods
    double angle_degrees;
    if (encoder_mgr.ReadAngleDegrees(0, angle_degrees) == As5047uError::SUCCESS) {
        std::cout << "Encoder angle: " << angle_degrees << "°\n";
    }
    
    // Check system health
    bool healthy = encoder_mgr.CheckAllDevicesHealth();
    std::cout << "System health: " << (healthy ? "OK" : "ERROR") << "\n";
}