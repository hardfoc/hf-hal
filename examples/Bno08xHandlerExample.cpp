/**
 * @file Bno08xHandlerExample.cpp
 * @brief Comprehensive example demonstrating BNO08x IMU handler usage.
 *
 * This example showcases all major features of the Bno08xHandler including:
 * - I2C and SPI interface usage
 * - Complete IMU data reading (acceleration, gyroscope, magnetometer)
 * - Quaternion and Euler angle orientation
 * - Activity and gesture detection
 * - Calibration management
 * - Advanced sensor configuration
 * - Callback-based event handling
 * - Hardware control and diagnostics
 * - Multi-interface factory methods
 * - Real-time sensor fusion monitoring
 *
 * Architecture demonstrates the same excellence as TMC9660Handler, PCAL95555Handler,
 * and AS5047UHandler examples with comprehensive feature coverage.
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#include "utils-and-drivers/driver-handlers/Bno08xHandler.h"
#include "component-handers/CommChannelsManager.h"
#include "utils/RtosTask.h"
#include "utils/Logger.h"
#include <cmath>

//======================================================//
// EXAMPLE CONFIGURATION CONSTANTS
//======================================================//

namespace Bno08xExample {
    // Timing constants
    constexpr uint32_t IMU_UPDATE_INTERVAL_MS = 20;      // 50 Hz update rate
    constexpr uint32_t CALIBRATION_TIMEOUT_MS = 60000;   // 60 second calibration timeout
    constexpr uint32_t ACTIVITY_CHECK_INTERVAL_MS = 100; // 10 Hz activity monitoring
    constexpr uint32_t STATUS_REPORT_INTERVAL_MS = 1000; // 1 Hz status reporting
    
    // Accuracy thresholds
    constexpr uint8_t MIN_ACCURACY_THRESHOLD = 2;        // Minimum accuracy for reliable data
    constexpr float MOTION_THRESHOLD = 0.1f;             // Motion detection threshold (m/s²)
    constexpr float ROTATION_THRESHOLD = 0.05f;          // Rotation detection threshold (rad/s)
    
    // GPIO pin assignments (example - adjust for your hardware)
    constexpr uint8_t RESET_PIN = 12;
    constexpr uint8_t INT_PIN = 13;
    constexpr uint8_t WAKE_PIN = 14;  // SPI only
}

//======================================================//
// EXAMPLE DATA STRUCTURES
//======================================================//

/**
 * @brief IMU monitoring state
 */
struct ImuMonitoringState {
    bool motion_detected;
    bool orientation_stable;
    bool calibration_complete;
    uint32_t sample_count;
    uint64_t last_update_us;
    float max_acceleration;
    float max_angular_velocity;
    
    ImuMonitoringState() 
        : motion_detected(false)
        , orientation_stable(true)
        , calibration_complete(false)
        , sample_count(0)
        , last_update_us(0)
        , max_acceleration(0.0f)
        , max_angular_velocity(0.0f) {}
};

/**
 * @brief Activity detection summary
 */
struct ActivitySummary {
    uint32_t total_steps;
    uint32_t tap_count;
    uint32_t shake_count;
    uint32_t pickup_count;
    bool significant_motion_detected;
    uint64_t last_activity_us;
    
    ActivitySummary()
        : total_steps(0)
        , tap_count(0)
        , shake_count(0)
        , pickup_count(0)
        , significant_motion_detected(false)
        , last_activity_us(0) {}
};

//======================================================//
// GLOBAL EXAMPLE STATE
//======================================================//

static ImuMonitoringState g_imu_state;
static ActivitySummary g_activity_summary;
static std::unique_ptr<Bno08xHandler> g_bno08x_handler;
static bool g_example_running = false;

//======================================================//
// SENSOR EVENT CALLBACK
//======================================================//

/**
 * @brief Callback function for sensor events
 * @param event Sensor event from BNO08x
 */
void SensorEventCallback(const SensorEvent& event) {
    // Process different sensor events
    switch (event.sensorType) {
        case BNO085Sensor::ACCELEROMETER:
            g_imu_state.max_acceleration = std::max(g_imu_state.max_acceleration, 
                                                   std::sqrt(event.data.x * event.data.x +
                                                           event.data.y * event.data.y +
                                                           event.data.z * event.data.z));
            break;
            
        case BNO085Sensor::GYROSCOPE_CALIBRATED:
            g_imu_state.max_angular_velocity = std::max(g_imu_state.max_angular_velocity,
                                                       std::sqrt(event.data.x * event.data.x +
                                                               event.data.y * event.data.y +
                                                               event.data.z * event.data.z));
            break;
            
        case BNO085Sensor::TAP_DETECTOR:
            g_activity_summary.tap_count++;
            g_activity_summary.last_activity_us = event.timeStamp;
            Logger::Info("Tap detected! Count: {}", g_activity_summary.tap_count);
            break;
            
        case BNO085Sensor::STEP_COUNTER:
            g_activity_summary.total_steps = static_cast<uint32_t>(event.data.x);
            g_activity_summary.last_activity_us = event.timeStamp;
            Logger::Info("Step detected! Total: {}", g_activity_summary.total_steps);
            break;
            
        case BNO085Sensor::SHAKE_DETECTOR:
            g_activity_summary.shake_count++;
            g_activity_summary.last_activity_us = event.timeStamp;
            Logger::Info("Shake detected! Count: {}", g_activity_summary.shake_count);
            break;
            
        default:
            // Handle other sensor types as needed
            break;
    }
    
    g_imu_state.sample_count++;
    g_imu_state.last_update_us = event.timeStamp;
}

//======================================================//
// CONFIGURATION EXAMPLES
//======================================================//

/**
 * @brief Create basic IMU configuration for orientation tracking
 */
Bno08xConfig CreateBasicImuConfig() {
    auto config = Bno08xHandler::GetDefaultConfig();
    
    // Enable core IMU sensors
    config.enable_accelerometer = true;
    config.enable_gyroscope = true;
    config.enable_magnetometer = true;
    config.enable_rotation_vector = true;
    config.enable_linear_acceleration = true;
    config.enable_gravity = true;
    
    // Disable activity detection for basic config
    config.enable_tap_detector = false;
    config.enable_step_counter = false;
    config.enable_shake_detector = false;
    config.enable_pickup_detector = false;
    config.enable_significant_motion = false;
    config.enable_activity_classifier = false;
    
    // Set high-rate intervals for responsive orientation
    config.accelerometer_interval_ms = 20;   // 50 Hz
    config.gyroscope_interval_ms = 20;       // 50 Hz
    config.magnetometer_interval_ms = 50;    // 20 Hz
    config.rotation_interval_ms = 20;        // 50 Hz
    config.linear_accel_interval_ms = 20;    // 50 Hz
    config.gravity_interval_ms = 50;         // 20 Hz
    
    config.auto_calibration = true;
    config.calibration_timeout_s = 30.0f;
    
    return config;
}

/**
 * @brief Create advanced configuration with full activity detection
 */
Bno08xConfig CreateAdvancedActivityConfig() {
    auto config = CreateBasicImuConfig();
    
    // Enable all activity detection features
    config.enable_tap_detector = true;
    config.enable_step_counter = true;
    config.enable_shake_detector = true;
    config.enable_pickup_detector = true;
    config.enable_significant_motion = true;
    config.enable_activity_classifier = true;
    
    // Enable game rotation for applications without magnetometer
    config.enable_game_rotation = true;
    
    return config;
}

/**
 * @brief Create low-power configuration for battery applications
 */
Bno08xConfig CreateLowPowerConfig() {
    auto config = Bno08xHandler::GetDefaultConfig();
    
    // Enable minimal sensors
    config.enable_accelerometer = true;
    config.enable_gyroscope = false;        // Disable gyro for power saving
    config.enable_magnetometer = false;     // Disable mag for power saving
    config.enable_rotation_vector = false;  // Disable rotation vector
    config.enable_linear_acceleration = false;
    config.enable_gravity = true;           // Enable gravity for orientation
    
    // Enable only essential activity detection
    config.enable_tap_detector = true;
    config.enable_step_counter = true;
    config.enable_significant_motion = true;
    
    // Use lower sample rates
    config.accelerometer_interval_ms = 100;  // 10 Hz
    config.gravity_interval_ms = 200;        // 5 Hz
    
    config.auto_calibration = false;  // Disable auto-calibration for power saving
    
    return config;
}

//======================================================//
// IMU DATA PROCESSING EXAMPLES
//======================================================//

/**
 * @brief Process and display complete IMU data
 */
void ProcessImuData() {
    if (!g_bno08x_handler || !g_bno08x_handler->IsSensorReady()) {
        return;
    }
    
    Bno08xImuData imu_data;
    Bno08xError result = g_bno08x_handler->ReadImuData(imu_data);
    
    if (result == Bno08xError::SUCCESS && imu_data.valid) {
        // Check for motion detection
        float total_accel = std::sqrt(imu_data.acceleration.x * imu_data.acceleration.x +
                                    imu_data.acceleration.y * imu_data.acceleration.y +
                                    imu_data.acceleration.z * imu_data.acceleration.z);
        
        float total_gyro = std::sqrt(imu_data.gyroscope.x * imu_data.gyroscope.x +
                                   imu_data.gyroscope.y * imu_data.gyroscope.y +
                                   imu_data.gyroscope.z * imu_data.gyroscope.z);
        
        g_imu_state.motion_detected = (total_accel > (9.81f + Bno08xExample::MOTION_THRESHOLD)) ||
                                     (total_gyro > Bno08xExample::ROTATION_THRESHOLD);
        
        // Check orientation stability
        g_imu_state.orientation_stable = (imu_data.rotation.accuracy >= Bno08xExample::MIN_ACCURACY_THRESHOLD);
        
        // Log detailed IMU data periodically
        static uint32_t log_counter = 0;
        if (++log_counter >= 50) { // Every ~1 second at 50Hz
            log_counter = 0;
            
            Logger::Info("=== IMU Data Report ===");
            Logger::Info("Acceleration: [{:.3f}, {:.3f}, {:.3f}] m/s² (acc: {})",
                        imu_data.acceleration.x, imu_data.acceleration.y, 
                        imu_data.acceleration.z, imu_data.acceleration.accuracy);
            
            Logger::Info("Gyroscope: [{:.3f}, {:.3f}, {:.3f}] rad/s (acc: {})",
                        imu_data.gyroscope.x, imu_data.gyroscope.y,
                        imu_data.gyroscope.z, imu_data.gyroscope.accuracy);
            
            Logger::Info("Magnetometer: [{:.3f}, {:.3f}, {:.3f}] µT (acc: {})",
                        imu_data.magnetometer.x, imu_data.magnetometer.y,
                        imu_data.magnetometer.z, imu_data.magnetometer.accuracy);
            
            Logger::Info("Quaternion: [{:.3f}, {:.3f}, {:.3f}, {:.3f}] (acc: {})",
                        imu_data.rotation.w, imu_data.rotation.x,
                        imu_data.rotation.y, imu_data.rotation.z, imu_data.rotation.accuracy);
            
            Logger::Info("Euler Angles: Roll={:.1f}°, Pitch={:.1f}°, Yaw={:.1f}°",
                        imu_data.euler.roll * 180.0f / M_PI,
                        imu_data.euler.pitch * 180.0f / M_PI,
                        imu_data.euler.yaw * 180.0f / M_PI);
            
            Logger::Info("Linear Accel: [{:.3f}, {:.3f}, {:.3f}] m/s²",
                        imu_data.linear_acceleration.x, imu_data.linear_acceleration.y,
                        imu_data.linear_acceleration.z);
            
            Logger::Info("Gravity: [{:.3f}, {:.3f}, {:.3f}] m/s²",
                        imu_data.gravity.x, imu_data.gravity.y, imu_data.gravity.z);
            
            Logger::Info("Motion: {}, Stable: {}, Samples: {}",
                        g_imu_state.motion_detected ? "YES" : "NO",
                        g_imu_state.orientation_stable ? "YES" : "NO",
                        g_imu_state.sample_count);
        }
        
    } else {
        Logger::Warning("Failed to read IMU data: {}", Bno08xErrorToString(result));
    }
}

/**
 * @brief Process activity and gesture detection
 */
void ProcessActivityData() {
    if (!g_bno08x_handler || !g_bno08x_handler->IsSensorReady()) {
        return;
    }
    
    Bno08xActivityData activity_data;
    Bno08xError result = g_bno08x_handler->ReadActivityData(activity_data);
    
    if (result == Bno08xError::SUCCESS && activity_data.valid) {
        // Update activity summary
        if (activity_data.step_detected) {
            g_activity_summary.total_steps = activity_data.step_count;
        }
        
        if (activity_data.tap_detected) {
            g_activity_summary.tap_count++;
            Logger::Info("Tap detected! Type: {}, Direction: {}", 
                        activity_data.double_tap ? "Double" : "Single",
                        activity_data.tap_direction);
        }
        
        if (activity_data.shake_detected) {
            g_activity_summary.shake_count++;
            Logger::Info("Shake gesture detected!");
        }
        
        if (activity_data.pickup_detected) {
            g_activity_summary.pickup_count++;
            Logger::Info("Pickup gesture detected!");
        }
        
        if (activity_data.significant_motion) {
            g_activity_summary.significant_motion_detected = true;
            Logger::Info("Significant motion detected!");
        }
        
        // Log activity summary periodically
        static uint32_t activity_log_counter = 0;
        if (++activity_log_counter >= 10) { // Every ~1 second at 10Hz
            activity_log_counter = 0;
            
            Logger::Info("=== Activity Summary ===");
            Logger::Info("Steps: {}, Taps: {}, Shakes: {}, Pickups: {}",
                        g_activity_summary.total_steps,
                        g_activity_summary.tap_count,
                        g_activity_summary.shake_count,
                        g_activity_summary.pickup_count);
            Logger::Info("Activity Class: {}, Stability: {}",
                        activity_data.activity_class,
                        activity_data.stability_class);
        }
        
    } else {
        // Activity data might not be available if features are disabled
        if (result != Bno08xError::SENSOR_NOT_ENABLED) {
            Logger::Warning("Failed to read activity data: {}", Bno08xErrorToString(result));
        }
    }
}

/**
 * @brief Monitor calibration status and progress
 */
void MonitorCalibration() {
    if (!g_bno08x_handler || !g_bno08x_handler->IsSensorReady()) {
        return;
    }
    
    Bno08xCalibrationStatus calibration_status;
    Bno08xError result = g_bno08x_handler->ReadCalibrationStatus(calibration_status);
    
    if (result == Bno08xError::SUCCESS) {
        bool previously_calibrated = g_imu_state.calibration_complete;
        g_imu_state.calibration_complete = calibration_status.calibration_complete;
        
        // Log calibration progress
        static uint32_t cal_log_counter = 0;
        if (++cal_log_counter >= 50) { // Every ~5 seconds at 10Hz
            cal_log_counter = 0;
            
            Logger::Info("=== Calibration Status ===");
            Logger::Info("Accelerometer: {}/3, Gyroscope: {}/3, Magnetometer: {}/3",
                        calibration_status.accelerometer_accuracy,
                        calibration_status.gyroscope_accuracy,
                        calibration_status.magnetometer_accuracy);
            Logger::Info("System: {}/3, Complete: {}, Time: {}s",
                        calibration_status.system_accuracy,
                        calibration_status.calibration_complete ? "YES" : "NO",
                        calibration_status.calibration_time_s);
        }
        
        // Alert when calibration completes
        if (g_imu_state.calibration_complete && !previously_calibrated) {
            Logger::Success("Sensor calibration completed!");
            
            // Save calibration to sensor memory
            Bno08xError save_result = g_bno08x_handler->SaveCalibration();
            if (save_result == Bno08xError::SUCCESS) {
                Logger::Success("Calibration saved to sensor memory");
            } else {
                Logger::Warning("Failed to save calibration: {}", Bno08xErrorToString(save_result));
            }
        }
        
    } else {
        Logger::Warning("Failed to read calibration status: {}", Bno08xErrorToString(result));
    }
}

//======================================================//
// I2C INTERFACE EXAMPLE
//======================================================//

/**
 * @brief Demonstrate BNO08x usage with I2C interface
 */
bool ExampleI2cInterface() {
    Logger::Info("=== BNO08x I2C Interface Example ===");
    
    // Get I2C interface from CommChannelsManager
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto i2c_interface = comm_manager.GetI2c1(); // Assuming I2C1 for IMU
    
    if (!i2c_interface) {
        Logger::Error("Failed to get I2C interface from CommChannelsManager");
        return false;
    }
    
    // Get GPIO interfaces for control pins
    auto reset_gpio = comm_manager.GetGpio(Bno08xExample::RESET_PIN);
    auto int_gpio = comm_manager.GetGpio(Bno08xExample::INT_PIN);
    
    if (!reset_gpio || !int_gpio) {
        Logger::Warning("GPIO pins not available - continuing without hardware control");
    }
    
    // Create advanced configuration with activity detection
    auto config = CreateAdvancedActivityConfig();
    config.interface_type = BNO085Interface::I2C;
    
    // Create BNO08x handler using factory method
    g_bno08x_handler = CreateBno08xHandlerI2c(*i2c_interface, config, 
                                             reset_gpio.get(), int_gpio.get());
    
    if (!g_bno08x_handler) {
        Logger::Error("Failed to create BNO08x I2C handler");
        return false;
    }
    
    // Set sensor event callback
    g_bno08x_handler->SetSensorCallback(SensorEventCallback);
    
    // Initialize the sensor
    Logger::Info("Initializing BNO08x sensor via I2C...");
    Bno08xError result = g_bno08x_handler->Initialize();
    
    if (result != Bno08xError::SUCCESS) {
        Logger::Error("BNO08x initialization failed: {}", Bno08xErrorToString(result));
        return false;
    }
    
    Logger::Success("BNO08x I2C handler initialized successfully!");
    Logger::Info("Interface: {}", g_bno08x_handler->GetDescription());
    Logger::Info("Starting calibration process...");
    
    // Start calibration
    result = g_bno08x_handler->StartCalibration();
    if (result != Bno08xError::SUCCESS) {
        Logger::Warning("Failed to start calibration: {}", Bno08xErrorToString(result));
    }
    
    return true;
}

//======================================================//
// SPI INTERFACE EXAMPLE
//======================================================//

/**
 * @brief Demonstrate BNO08x usage with SPI interface
 */
bool ExampleSpiInterface() {
    Logger::Info("=== BNO08x SPI Interface Example ===");
    
    // Get SPI interface from CommChannelsManager
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto spi_interface = comm_manager.GetSpi2(); // Assuming SPI2 for IMU
    
    if (!spi_interface) {
        Logger::Error("Failed to get SPI interface from CommChannelsManager");
        return false;
    }
    
    // Get GPIO interfaces for control pins
    auto wake_gpio = comm_manager.GetGpio(Bno08xExample::WAKE_PIN);
    auto reset_gpio = comm_manager.GetGpio(Bno08xExample::RESET_PIN);
    auto int_gpio = comm_manager.GetGpio(Bno08xExample::INT_PIN);
    
    if (!wake_gpio || !reset_gpio || !int_gpio) {
        Logger::Warning("GPIO pins not available - continuing without hardware control");
    }
    
    // Create basic IMU configuration for SPI
    auto config = CreateBasicImuConfig();
    config.interface_type = BNO085Interface::SPI;
    
    // Create BNO08x handler using factory method
    g_bno08x_handler = CreateBno08xHandlerSpi(*spi_interface, config,
                                             wake_gpio.get(), reset_gpio.get(), int_gpio.get());
    
    if (!g_bno08x_handler) {
        Logger::Error("Failed to create BNO08x SPI handler");
        return false;
    }
    
    // Set sensor event callback
    g_bno08x_handler->SetSensorCallback(SensorEventCallback);
    
    // Initialize the sensor
    Logger::Info("Initializing BNO08x sensor via SPI...");
    Bno08xError result = g_bno08x_handler->Initialize();
    
    if (result != Bno08xError::SUCCESS) {
        Logger::Error("BNO08x initialization failed: {}", Bno08xErrorToString(result));
        return false;
    }
    
    Logger::Success("BNO08x SPI handler initialized successfully!");
    Logger::Info("Interface: {}", g_bno08x_handler->GetDescription());
    
    return true;
}

//======================================================//
// DYNAMIC CONFIGURATION EXAMPLE
//======================================================//

/**
 * @brief Demonstrate dynamic sensor configuration changes
 */
void ExampleDynamicConfiguration() {
    if (!g_bno08x_handler || !g_bno08x_handler->IsSensorReady()) {
        Logger::Warning("BNO08x not ready for configuration changes");
        return;
    }
    
    Logger::Info("=== Dynamic Configuration Example ===");
    
    // Enable high-rate gyroscope for motion detection
    Logger::Info("Enabling high-rate gyroscope (100Hz)...");
    Bno08xError result = g_bno08x_handler->EnableSensor(BNO085Sensor::GYROSCOPE_CALIBRATED, 10);
    
    if (result == Bno08xError::SUCCESS) {
        Logger::Success("High-rate gyroscope enabled");
    } else {
        Logger::Warning("Failed to enable gyroscope: {}", Bno08xErrorToString(result));
    }
    
    // Wait and then switch to lower rate
    RtosTask::Delay(5000);
    
    Logger::Info("Switching to low-rate gyroscope (10Hz)...");
    result = g_bno08x_handler->EnableSensor(BNO085Sensor::GYROSCOPE_CALIBRATED, 100);
    
    if (result == Bno08xError::SUCCESS) {
        Logger::Success("Low-rate gyroscope enabled");
    } else {
        Logger::Warning("Failed to reconfigure gyroscope: {}", Bno08xErrorToString(result));
    }
    
    // Enable step counter
    Logger::Info("Enabling step counter...");
    result = g_bno08x_handler->EnableSensor(BNO085Sensor::STEP_COUNTER, 0);
    
    if (result == Bno08xError::SUCCESS) {
        Logger::Success("Step counter enabled");
    } else {
        Logger::Warning("Failed to enable step counter: {}", Bno08xErrorToString(result));
    }
    
    // Enable tap detector
    Logger::Info("Enabling tap detector...");
    result = g_bno08x_handler->EnableSensor(BNO085Sensor::TAP_DETECTOR, 0);
    
    if (result == Bno08xError::SUCCESS) {
        Logger::Success("Tap detector enabled");
    } else {
        Logger::Warning("Failed to enable tap detector: {}", Bno08xErrorToString(result));
    }
}

//======================================================//
// HARDWARE CONTROL EXAMPLE
//======================================================//

/**
 * @brief Demonstrate hardware control features
 */
void ExampleHardwareControl() {
    if (!g_bno08x_handler) {
        Logger::Warning("BNO08x handler not available");
        return;
    }
    
    Logger::Info("=== Hardware Control Example ===");
    
    // Perform hardware reset
    Logger::Info("Performing hardware reset...");
    Bno08xError result = g_bno08x_handler->HardwareReset(20);
    
    if (result == Bno08xError::SUCCESS) {
        Logger::Success("Hardware reset completed");
        
        // Wait for sensor to restart
        RtosTask::Delay(1000);
        
        // Re-initialize after reset
        Logger::Info("Re-initializing after reset...");
        result = g_bno08x_handler->Initialize();
        
        if (result == Bno08xError::SUCCESS) {
            Logger::Success("Sensor re-initialized successfully");
        } else {
            Logger::Error("Failed to re-initialize: {}", Bno08xErrorToString(result));
        }
        
    } else {
        Logger::Warning("Hardware reset failed: {}", Bno08xErrorToString(result));
    }
    
    // Demonstrate wake pin control (SPI only)
    if (g_bno08x_handler->GetInterfaceType() == BNO085Interface::SPI) {
        Logger::Info("Controlling wake pin...");
        
        result = g_bno08x_handler->SetWakePin(false);
        if (result == Bno08xError::SUCCESS) {
            Logger::Info("Wake pin set LOW");
            RtosTask::Delay(100);
            
            result = g_bno08x_handler->SetWakePin(true);
            if (result == Bno08xError::SUCCESS) {
                Logger::Info("Wake pin set HIGH");
            }
        }
    }
}

//======================================================//
// MAIN MONITORING TASK
//======================================================//

/**
 * @brief Main monitoring task for BNO08x sensor
 */
void BnoMonitoringTask(void* parameters) {
    (void)parameters;
    
    Logger::Info("Starting BNO08x monitoring task...");
    g_example_running = true;
    
    uint32_t imu_counter = 0;
    uint32_t activity_counter = 0;
    uint32_t status_counter = 0;
    
    while (g_example_running) {
        // Update sensor
        if (g_bno08x_handler && g_bno08x_handler->IsSensorReady()) {
            g_bno08x_handler->Update();
            
            // Process IMU data at high rate
            if (++imu_counter >= (Bno08xExample::IMU_UPDATE_INTERVAL_MS / 10)) {
                imu_counter = 0;
                ProcessImuData();
            }
            
            // Process activity data at medium rate
            if (++activity_counter >= (Bno08xExample::ACTIVITY_CHECK_INTERVAL_MS / 10)) {
                activity_counter = 0;
                ProcessActivityData();
            }
            
            // Status monitoring at low rate
            if (++status_counter >= (Bno08xExample::STATUS_REPORT_INTERVAL_MS / 10)) {
                status_counter = 0;
                MonitorCalibration();
            }
        }
        
        RtosTask::Delay(10); // 100 Hz task rate
    }
    
    Logger::Info("BNO08x monitoring task ended");
}

//======================================================//
// MAIN EXAMPLE FUNCTIONS
//======================================================//

/**
 * @brief Main BNO08x handler example
 */
void RunBno08xHandlerExample() {
    Logger::Info("========================================");
    Logger::Info("       BNO08x Handler Example");
    Logger::Info("========================================");
    
    // Initialize example state
    g_imu_state = ImuMonitoringState();
    g_activity_summary = ActivitySummary();
    
    // Try I2C interface first
    if (ExampleI2cInterface()) {
        Logger::Info("Using I2C interface for BNO08x communication");
    } else {
        Logger::Warning("I2C interface failed, trying SPI...");
        
        // Try SPI interface as fallback
        if (ExampleSpiInterface()) {
            Logger::Info("Using SPI interface for BNO08x communication");
        } else {
            Logger::Error("Both I2C and SPI interfaces failed!");
            return;
        }
    }
    
    // Start monitoring task
    RtosTask::Create("BnoMonitor", BnoMonitoringTask, nullptr, 
                     4096, RtosTask::Priority::NORMAL);
    
    // Let the system run for initial calibration
    Logger::Info("Running initial calibration and configuration...");
    RtosTask::Delay(5000);
    
    // Demonstrate dynamic configuration
    ExampleDynamicConfiguration();
    
    // Demonstrate hardware control
    RtosTask::Delay(2000);
    ExampleHardwareControl();
    
    // Run monitoring for extended period
    Logger::Info("Running extended monitoring (press any key to stop)...");
    
    // In a real application, this would run indefinitely or until stop condition
    for (int i = 0; i < 30; i++) { // Run for 30 seconds
        RtosTask::Delay(1000);
        
        // Display periodic status
        Logger::Info("Runtime: {}s, Samples: {}, Motion: {}, Calibrated: {}",
                    i + 1, g_imu_state.sample_count,
                    g_imu_state.motion_detected ? "YES" : "NO",
                    g_imu_state.calibration_complete ? "YES" : "NO");
    }
    
    // Cleanup
    g_example_running = false;
    RtosTask::Delay(1000); // Allow task to finish
    
    if (g_bno08x_handler) {
        g_bno08x_handler->ClearSensorCallback();
        g_bno08x_handler->Deinitialize();
        g_bno08x_handler.reset();
    }
    
    Logger::Success("BNO08x Handler Example completed successfully!");
    Logger::Info("Final Statistics:");
    Logger::Info("- Total samples: {}", g_imu_state.sample_count);
    Logger::Info("- Max acceleration: {:.3f} m/s²", g_imu_state.max_acceleration);
    Logger::Info("- Max angular velocity: {:.3f} rad/s", g_imu_state.max_angular_velocity);
    Logger::Info("- Total steps: {}", g_activity_summary.total_steps);
    Logger::Info("- Tap events: {}", g_activity_summary.tap_count);
    Logger::Info("- Shake events: {}", g_activity_summary.shake_count);
    Logger::Info("- Pickup events: {}", g_activity_summary.pickup_count);
}

/**
 * @brief Quick test function for basic functionality
 */
void QuickBno08xTest() {
    Logger::Info("=== Quick BNO08x Test ===");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    auto i2c_interface = comm_manager.GetI2c1();
    
    if (!i2c_interface) {
        Logger::Error("No I2C interface available for quick test");
        return;
    }
    
    // Create simple handler
    auto config = CreateBasicImuConfig();
    auto handler = CreateBno08xHandlerI2c(*i2c_interface, config);
    
    if (!handler) {
        Logger::Error("Failed to create BNO08x handler");
        return;
    }
    
    // Initialize
    if (handler->Initialize() != Bno08xError::SUCCESS) {
        Logger::Error("Failed to initialize BNO08x");
        return;
    }
    
    Logger::Success("BNO08x quick test initialization successful!");
    
    // Read a few samples
    for (int i = 0; i < 5; i++) {
        handler->Update();
        
        Bno08xVector3 acceleration;
        if (handler->ReadAcceleration(acceleration) == Bno08xError::SUCCESS) {
            Logger::Info("Acceleration {}: [{:.3f}, {:.3f}, {:.3f}] m/s²", 
                        i + 1, acceleration.x, acceleration.y, acceleration.z);
        }
        
        Bno08xQuaternion quaternion;
        if (handler->ReadQuaternion(quaternion) == Bno08xError::SUCCESS) {
            Logger::Info("Quaternion {}: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                        i + 1, quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        }
        
        RtosTask::Delay(100);
    }
    
    handler->Deinitialize();
    Logger::Success("Quick BNO08x test completed!");
}
