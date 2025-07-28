# ImuManager - IMU Sensor Management System

<div align="center">

![Component](https://img.shields.io/badge/component-ImuManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Sensors](https://img.shields.io/badge/sensors-9--axis%20IMU-orange.svg)

**Advanced IMU sensor management with fusion algorithms and motion detection**

</div>

## 📋 Overview

The `ImuManager` is a singleton component handler that provides high-level management of IMU (Inertial Measurement Unit) sensors on the HardFOC platform. It integrates with BNO08x sensor handlers to deliver advanced motion sensing, orientation tracking, and activity recognition capabilities.

### ✨ Key Features

- **🧭 Advanced Sensor Fusion**: High-precision orientation and motion data
- **🎯 Multiple Output Formats**: Quaternions, Euler angles, acceleration vectors
- **📊 Activity Recognition**: Walking, running, vehicle detection, tap sensing
- **🔄 Automatic Calibration**: Self-calibrating sensors with accuracy monitoring
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **⚡ High Performance**: Optimized data processing and filtering
- **🏥 Health Monitoring**: Sensor status, temperature, and error detection
- **🔧 Easy Integration**: Simple API for motion-based applications

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     ImuManager                                │
├─────────────────────────────────────────────────────────────────┤
│  Motion Processing  │ Sensor fusion, filtering, calibration   │
├─────────────────────────────────────────────────────────────────┤
│  Activity Detection │ Step counting, tap detection, stability │
├─────────────────────────────────────────────────────────────────┤
│  Data Management    │ Buffering, threading, synchronization   │
├─────────────────────────────────────────────────────────────────┤
│  BNO08x Handler     │ Hardware sensor driver integration      │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic IMU Operations

```cpp
#include "component-handlers/ImuManager.h"

void imu_basic_example() {
    // Get singleton instance
    auto& imu = ImuManager::GetInstance();
    
    // Initialize IMU manager
    if (!imu.EnsureInitialized()) {
        printf("Failed to initialize IMU manager\n");
        return;
    }
    
    printf("IMU Manager ready\n");
    
    // Read orientation data
    for (int i = 0; i < 100; i++) {
        if (imu.IsDataReady()) {
            auto orientation = imu.GetOrientation();
            printf("Orientation: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
                   orientation.roll, orientation.pitch, orientation.yaw);
            
            auto motion = imu.GetMotionData();
            printf("Acceleration: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
                   motion.linear_acceleration.x,
                   motion.linear_acceleration.y,
                   motion.linear_acceleration.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 📖 API Reference

### Core Operations

#### Initialization and Management
```cpp
class ImuManager {
public:
    // Singleton access
    static ImuManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool EnsureDeinitialized() noexcept;
    bool IsInitialized() const noexcept;
    
    // Data availability
    bool IsDataReady() const noexcept;
    void Update() noexcept;  // Force update from sensors
    
private:
    bool Initialize() noexcept;
    void Deinitialize() noexcept;
};
```

#### Orientation Data
```cpp
// Orientation representation
struct Orientation {
    float roll, pitch, yaw;     // Degrees
    float accuracy;             // Accuracy estimate (degrees)
    uint32_t timestamp_ms;      // Timestamp
};

// Quaternion representation
struct QuaternionData {
    float w, x, y, z;          // Quaternion components
    float accuracy;            // Accuracy estimate (radians)
    uint32_t timestamp_ms;     // Timestamp
};

// Rotation vector
struct RotationVector {
    float i, j, k;             // Rotation vector components
    float accuracy;            // Accuracy estimate
    uint32_t timestamp_ms;     // Timestamp
};

Orientation GetOrientation() noexcept;
QuaternionData GetQuaternion() noexcept;
RotationVector GetRotationVector() noexcept;
```

#### Motion Data
```cpp
// Motion data structure
struct MotionData {
    Vector3 linear_acceleration;    // m/s² (gravity removed)
    Vector3 angular_velocity;       // rad/s
    Vector3 magnetic_field;         // µT
    Vector3 gravity_vector;         // m/s²
    uint32_t timestamp_ms;
};

// Raw sensor data
struct RawSensorData {
    Vector3 accelerometer;          // m/s² (with gravity)
    Vector3 gyroscope;             // rad/s
    Vector3 magnetometer;          // µT
    uint8_t accuracy_accel;
    uint8_t accuracy_gyro;
    uint8_t accuracy_mag;
    uint32_t timestamp_ms;
};

MotionData GetMotionData() noexcept;
RawSensorData GetRawSensorData() noexcept;
```

#### Activity Detection
```cpp
// Activity classification
struct ActivityInfo {
    bool walking;
    bool running;
    bool stationary;
    bool in_vehicle;
    bool unknown;
    uint8_t confidence;           // 0-100%
    uint32_t timestamp_ms;
};

// Tap detection
struct TapInfo {
    bool single_tap_detected;
    bool double_tap_detected;
    uint8_t tap_direction;        // Direction of tap
    uint32_t timestamp_ms;
};

// Step counting
struct StepInfo {
    uint32_t step_count;
    uint32_t steps_since_reset;
    float step_frequency;         // Steps per minute
    uint32_t last_step_time_ms;
};

// Stability detection
struct StabilityInfo {
    bool is_stable;
    uint8_t stability_confidence;
    uint32_t stable_duration_ms;
    uint32_t timestamp_ms;
};

ActivityInfo GetActivity() noexcept;
TapInfo GetTapDetection() noexcept;
StepInfo GetStepInfo() noexcept;
StabilityInfo GetStability() noexcept;
```

#### Configuration and Calibration
```cpp
// IMU configuration
struct ImuConfig {
    uint16_t orientation_rate_hz = 50;      // Orientation update rate
    uint16_t motion_rate_hz = 100;          // Motion data update rate
    uint16_t activity_rate_hz = 1;          // Activity classification rate
    
    bool enable_tap_detection = true;
    bool enable_step_counter = true;
    bool enable_stability_classifier = true;
    bool enable_activity_classifier = true;
    
    bool auto_calibration = true;
    float calibration_timeout_sec = 120.0f;
};

bool Configure(const ImuConfig& config) noexcept;
ImuConfig GetConfiguration() const noexcept;

// Calibration control
bool StartCalibration() noexcept;
bool SaveCalibration() noexcept;
bool LoadCalibration() noexcept;
bool IsCalibrated() const noexcept;
uint8_t GetCalibrationProgress() const noexcept;  // 0-100%
```

#### Status and Diagnostics
```cpp
// System status
struct ImuStatus {
    bool sensor_connected;
    bool sensor_initialized;
    bool calibration_active;
    bool calibration_complete;
    float temperature_celsius;
    uint32_t update_rate_hz;
    uint32_t error_count;
    uint32_t total_updates;
};

ImuStatus GetStatus() const noexcept;
float GetTemperature() const noexcept;
uint32_t GetUpdateRate() const noexcept;
uint32_t GetErrorCount() const noexcept;
```

## 🎯 Motion Processing Features

### Advanced Sensor Fusion

The ImuManager integrates data from multiple sensors to provide accurate motion information:

| Sensor | Data | Fusion Role | Update Rate |
|--------|------|-------------|-------------|
| **Accelerometer** | Linear acceleration | Tilt, motion detection | Up to 1000Hz |
| **Gyroscope** | Angular velocity | Rotation, orientation | Up to 1000Hz |
| **Magnetometer** | Magnetic field | Absolute heading | Up to 100Hz |
| **Fusion Engine** | Combined output | Drift-free orientation | Configurable |

### Motion Detection Capabilities

```cpp
// Motion detection thresholds and parameters
struct MotionThresholds {
    float acceleration_threshold = 2.0f;    // m/s² for motion detection
    float gyro_threshold = 1.0f;            // rad/s for rotation detection
    float stillness_threshold = 0.1f;       // m/s² for stillness detection
    uint32_t motion_timeout_ms = 5000;      // Motion timeout
    uint32_t stillness_timeout_ms = 3000;   // Stillness timeout
};

bool SetMotionThresholds(const MotionThresholds& thresholds) noexcept;
MotionThresholds GetMotionThresholds() const noexcept;

// Motion state detection
enum class MotionState {
    STATIONARY,
    MOVING,
    ROTATING,
    ACCELERATING,
    FREE_FALL,
    IMPACT
};

MotionState GetMotionState() noexcept;
bool IsMotionDetected() noexcept;
bool IsStillnessDetected() noexcept;
```

## 🔧 Configuration

### Initialization Configuration

```cpp
// Default IMU configuration
constexpr ImuConfig kDefaultImuConfig = {
    .orientation_rate_hz = 50,              // 50 Hz orientation
    .motion_rate_hz = 100,                  // 100 Hz motion data
    .activity_rate_hz = 1,                  // 1 Hz activity updates
    
    .enable_tap_detection = true,
    .enable_step_counter = true,
    .enable_stability_classifier = true,
    .enable_activity_classifier = true,
    
    .auto_calibration = true,
    .calibration_timeout_sec = 120.0f
};

// Performance-optimized configuration
constexpr ImuConfig kHighPerformanceConfig = {
    .orientation_rate_hz = 200,             // 200 Hz orientation
    .motion_rate_hz = 500,                  // 500 Hz motion data
    .activity_rate_hz = 10,                 // 10 Hz activity updates
    
    .enable_tap_detection = true,
    .enable_step_counter = true,
    .enable_stability_classifier = true,
    .enable_activity_classifier = true,
    
    .auto_calibration = true,
    .calibration_timeout_sec = 60.0f
};

// Low-power configuration
constexpr ImuConfig kLowPowerConfig = {
    .orientation_rate_hz = 10,              // 10 Hz orientation
    .motion_rate_hz = 20,                   // 20 Hz motion data
    .activity_rate_hz = 0,                  // Disable activity classification
    
    .enable_tap_detection = false,
    .enable_step_counter = false,
    .enable_stability_classifier = false,
    .enable_activity_classifier = false,
    
    .auto_calibration = true,
    .calibration_timeout_sec = 180.0f
};
```

### Filter Configuration

```cpp
// Motion filtering parameters
struct FilterConfig {
    float acceleration_filter_alpha = 0.1f;    // Low-pass filter for acceleration
    float gyro_filter_alpha = 0.2f;            // Low-pass filter for gyroscope
    float orientation_filter_alpha = 0.05f;    // Low-pass filter for orientation
    
    bool enable_gravity_compensation = true;
    bool enable_bias_compensation = true;
    bool enable_noise_reduction = true;
    
    uint32_t filter_window_size = 10;          // Samples for moving average
};

bool SetFilterConfig(const FilterConfig& config) noexcept;
FilterConfig GetFilterConfig() const noexcept;
```

## 📊 Examples

### Basic Orientation Monitoring

```cpp
#include "component-handlers/ImuManager.h"

void orientation_monitoring_example() {
    auto& imu = ImuManager::GetInstance();
    
    if (!imu.EnsureInitialized()) {
        printf("IMU Manager initialization failed\n");
        return;
    }
    
    printf("IMU orientation monitoring started\n");
    
    for (int i = 0; i < 1000; i++) {
        if (imu.IsDataReady()) {
            // Get orientation data
            auto orientation = imu.GetOrientation();
            auto status = imu.GetStatus();
            
            printf("Orientation: R=%6.1f° P=%6.1f° Y=%6.1f° (acc=%.1f°)\n",
                   orientation.roll, orientation.pitch, orientation.yaw, 
                   orientation.accuracy);
            
            // Check calibration status
            if (!status.calibration_complete) {
                uint8_t progress = imu.GetCalibrationProgress();
                printf("Calibration in progress: %u%%\n", progress);
            }
            
            // Check for errors
            if (status.error_count > 0) {
                printf("Warning: %lu errors detected\n", status.error_count);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### Motion and Activity Detection

```cpp
void motion_activity_detection_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Configure for activity detection
    ImuConfig config = imu.GetConfiguration();
    config.enable_activity_classifier = true;
    config.enable_step_counter = true;
    config.enable_tap_detection = true;
    config.activity_rate_hz = 5;  // 5 Hz activity updates
    imu.Configure(config);
    
    printf("Motion and activity detection started\n");
    
    uint32_t last_step_count = 0;
    
    for (int i = 0; i < 6000; i++) {  // 10 minutes
        if (imu.IsDataReady()) {
            // Get motion state
            auto motion_state = imu.GetMotionState();
            static auto last_motion_state = MotionState::STATIONARY;
            
            if (motion_state != last_motion_state) {
                switch (motion_state) {
                    case MotionState::STATIONARY:
                        printf("Motion: STATIONARY\n");
                        break;
                    case MotionState::MOVING:
                        printf("Motion: MOVING\n");
                        break;
                    case MotionState::ROTATING:
                        printf("Motion: ROTATING\n");
                        break;
                    case MotionState::ACCELERATING:
                        printf("Motion: ACCELERATING\n");
                        break;
                    case MotionState::FREE_FALL:
                        printf("Motion: FREE FALL DETECTED!\n");
                        break;
                    case MotionState::IMPACT:
                        printf("Motion: IMPACT DETECTED!\n");
                        break;
                }
                last_motion_state = motion_state;
            }
            
            // Check for tap detection
            auto tap = imu.GetTapDetection();
            if (tap.single_tap_detected) {
                printf("Single tap detected!\n");
            }
            if (tap.double_tap_detected) {
                printf("Double tap detected!\n");
            }
            
            // Check activity classification
            auto activity = imu.GetActivity();
            if (activity.confidence > 75) {  // High confidence only
                static bool last_walking = false;
                static bool last_running = false;
                static bool last_in_vehicle = false;
                
                if (activity.walking && !last_walking) {
                    printf("Activity: WALKING (confidence: %u%%)\n", activity.confidence);
                }
                if (activity.running && !last_running) {
                    printf("Activity: RUNNING (confidence: %u%%)\n", activity.confidence);
                }
                if (activity.in_vehicle && !last_in_vehicle) {
                    printf("Activity: IN VEHICLE (confidence: %u%%)\n", activity.confidence);
                }
                
                last_walking = activity.walking;
                last_running = activity.running;
                last_in_vehicle = activity.in_vehicle;
            }
            
            // Check step counter
            auto step_info = imu.GetStepInfo();
            if (step_info.step_count != last_step_count) {
                printf("Steps: %lu (frequency: %.1f steps/min)\n", 
                       step_info.step_count, step_info.step_frequency);
                last_step_count = step_info.step_count;
            }
            
            // Check stability
            auto stability = imu.GetStability();
            static bool was_stable = true;
            if (stability.is_stable != was_stable) {
                printf("Stability: %s (confidence: %u%%)\n",
                       stability.is_stable ? "STABLE" : "UNSTABLE",
                       stability.stability_confidence);
                was_stable = stability.is_stable;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### High-Performance Motion Tracking

```cpp
void high_performance_motion_tracking() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Configure for high-performance tracking
    ImuConfig config = kHighPerformanceConfig;
    imu.Configure(config);
    
    printf("High-performance motion tracking started\n");
    printf("Time,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ\n");
    
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    for (int i = 0; i < 10000; i++) {
        if (imu.IsDataReady()) {
            uint32_t timestamp = (esp_timer_get_time() / 1000) - start_time;
            
            // Get all motion data
            auto orientation = imu.GetOrientation();
            auto motion = imu.GetMotionData();
            auto raw = imu.GetRawSensorData();
            
            // Output CSV format for analysis
            printf("%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n",
                   timestamp,
                   orientation.roll, orientation.pitch, orientation.yaw,
                   motion.linear_acceleration.x, motion.linear_acceleration.y, motion.linear_acceleration.z,
                   motion.angular_velocity.x, motion.angular_velocity.y, motion.angular_velocity.z,
                   motion.magnetic_field.x, motion.magnetic_field.y, motion.magnetic_field.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2));  // 500 Hz loop
    }
}
```

### IMU Calibration Procedure

```cpp
void imu_calibration_procedure() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    printf("IMU Calibration Procedure\n");
    printf("=========================\n");
    
    // Start calibration process
    if (!imu.StartCalibration()) {
        printf("Failed to start calibration\n");
        return;
    }
    
    printf("Calibration started. Follow these instructions:\n");
    printf("1. Keep device stationary for 5 seconds\n");
    printf("2. Slowly rotate around all axes\n");
    printf("3. Move in figure-8 pattern for magnetometer\n");
    printf("4. Wait for completion\n\n");
    
    bool calibration_complete = false;
    uint32_t calibration_start = esp_timer_get_time() / 1000000;
    uint8_t last_progress = 0;
    
    while (!calibration_complete && 
           ((esp_timer_get_time() / 1000000) - calibration_start) < 180) {  // 3 min timeout
        
        if (imu.IsDataReady()) {
            uint8_t progress = imu.GetCalibrationProgress();
            auto status = imu.GetStatus();
            
            // Update progress display
            if (progress != last_progress) {
                printf("Calibration progress: %u%%\n", progress);
                
                if (progress < 25) {
                    printf("  Keep device stationary...\n");
                } else if (progress < 50) {
                    printf("  Slowly rotate around X-axis...\n");
                } else if (progress < 75) {
                    printf("  Slowly rotate around Y-axis...\n");
                } else if (progress < 90) {
                    printf("  Move in figure-8 pattern...\n");
                } else {
                    printf("  Almost complete...\n");
                }
                
                last_progress = progress;
            }
            
            // Check if calibration is complete
            if (status.calibration_complete) {
                calibration_complete = true;
                printf("Calibration completed successfully!\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    if (calibration_complete) {
        // Save calibration data
        if (imu.SaveCalibration()) {
            printf("Calibration data saved\n");
        } else {
            printf("Warning: Failed to save calibration data\n");
        }
        
        // Test calibrated accuracy
        printf("\nTesting calibrated performance:\n");
        for (int i = 0; i < 20; i++) {
            if (imu.IsDataReady()) {
                auto orientation = imu.GetOrientation();
                printf("Accuracy: %.2f° - Orientation: R=%.1f° P=%.1f° Y=%.1f°\n",
                       orientation.accuracy, orientation.roll, orientation.pitch, orientation.yaw);
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    } else {
        printf("Calibration timeout. Please try again.\n");
    }
}
```

### Real-Time Motion Analysis

```cpp
void real_time_motion_analysis() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    printf("Real-time motion analysis\n");
    
    // Motion analysis variables
    float max_acceleration = 0.0f;
    float max_angular_velocity = 0.0f;
    uint32_t motion_duration = 0;
    uint32_t stillness_duration = 0;
    uint32_t last_motion_time = 0;
    
    for (int i = 0; i < 6000; i++) {  // 10 minutes
        if (imu.IsDataReady()) {
            auto motion = imu.GetMotionData();
            auto motion_state = imu.GetMotionState();
            uint32_t current_time = esp_timer_get_time() / 1000;
            
            // Calculate motion magnitude
            float accel_magnitude = sqrtf(
                motion.linear_acceleration.x * motion.linear_acceleration.x +
                motion.linear_acceleration.y * motion.linear_acceleration.y +
                motion.linear_acceleration.z * motion.linear_acceleration.z
            );
            
            float gyro_magnitude = sqrtf(
                motion.angular_velocity.x * motion.angular_velocity.x +
                motion.angular_velocity.y * motion.angular_velocity.y +
                motion.angular_velocity.z * motion.angular_velocity.z
            );
            
            // Update maximums
            if (accel_magnitude > max_acceleration) {
                max_acceleration = accel_magnitude;
                printf("New max acceleration: %.2f m/s²\n", max_acceleration);
            }
            
            if (gyro_magnitude > max_angular_velocity) {
                max_angular_velocity = gyro_magnitude;
                printf("New max angular velocity: %.2f rad/s\n", max_angular_velocity);
            }
            
            // Track motion/stillness durations
            if (motion_state != MotionState::STATIONARY) {
                if (last_motion_time == 0) {
                    last_motion_time = current_time;
                }
                motion_duration = current_time - last_motion_time;
                stillness_duration = 0;
            } else {
                if (motion_duration > 0) {
                    printf("Motion duration: %lu ms\n", motion_duration);
                    motion_duration = 0;
                    last_motion_time = 0;
                }
                stillness_duration++;
            }
            
            // Periodic summary
            if (i % 1000 == 0 && i > 0) {
                printf("\nMotion Analysis Summary (%.1f minutes):\n", i / 600.0f);
                printf("  Max acceleration: %.2f m/s²\n", max_acceleration);
                printf("  Max angular velocity: %.2f rad/s\n", max_angular_velocity);
                printf("  Current motion state: %d\n", static_cast<int>(motion_state));
                printf("  Temperature: %.1f°C\n", imu.GetTemperature());
                printf("  Update rate: %lu Hz\n", imu.GetUpdateRate());
                printf("  Error count: %lu\n", imu.GetErrorCount());
                printf("\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    printf("Motion analysis complete\n");
    printf("Final statistics:\n");
    printf("  Max acceleration: %.2f m/s²\n", max_acceleration);
    printf("  Max angular velocity: %.2f rad/s\n", max_angular_velocity);
}
```

### Performance Benchmarking

```cpp
void imu_performance_benchmark() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    printf("IMU Performance Benchmark\n");
    printf("=========================\n");
    
    // Test different update rates
    uint16_t test_rates[] = {10, 20, 50, 100, 200};
    
    for (auto rate : test_rates) {
        printf("\nTesting %u Hz update rate:\n", rate);
        
        // Configure for test rate
        ImuConfig config = imu.GetConfiguration();
        config.orientation_rate_hz = rate;
        config.motion_rate_hz = rate;
        imu.Configure(config);
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Settle time
        
        // Measure performance
        uint32_t update_count = 0;
        uint32_t total_latency = 0;
        uint32_t start_time = esp_timer_get_time() / 1000;
        
        for (int i = 0; i < 2000; i++) {
            uint32_t loop_start = esp_timer_get_time();
            
            if (imu.IsDataReady()) {
                auto orientation = imu.GetOrientation();
                auto motion = imu.GetMotionData();
                update_count++;
                
                uint32_t latency = (esp_timer_get_time() - loop_start);
                total_latency += latency;
            }
            
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        uint32_t test_duration = (esp_timer_get_time() / 1000) - start_time;
        float actual_rate = (float)update_count * 1000.0f / test_duration;
        float avg_latency = (float)total_latency / update_count;
        float cpu_usage = (avg_latency * actual_rate) / 10000.0f;
        
        printf("  Requested rate: %u Hz\n", rate);
        printf("  Actual rate: %.1f Hz\n", actual_rate);
        printf("  Average latency: %.2f µs\n", avg_latency);
        printf("  CPU usage: %.1f%%\n", cpu_usage);
        printf("  Efficiency: %.1f%%\n", (actual_rate / rate) * 100.0f);
    }
    
    // Memory usage analysis
    printf("\nMemory Usage:\n");
    printf("  ImuManager size: %zu bytes\n", sizeof(ImuManager));
    
    printf("\nBenchmark complete\n");
}
```

## 🔍 Advanced Usage

### Custom Motion Detection

```cpp
// Example of extending ImuManager with custom motion detection
class AdvancedMotionDetector {
public:
    AdvancedMotionDetector(ImuManager& imu) : imu_(imu) {}
    
    void Initialize() {
        // Configure for high-rate motion detection
        ImuConfig config = imu_.GetConfiguration();
        config.motion_rate_hz = 200;
        config.enable_tap_detection = true;
        imu_.Configure(config);
        
        // Initialize detection parameters
        gesture_buffer_.reserve(gesture_window_size_);
    }
    
    void Update() {
        if (imu_.IsDataReady()) {
            auto motion = imu_.GetMotionData();
            
            // Add to gesture buffer
            AddGestureSample(motion);
            
            // Detect custom gestures
            DetectShakeGesture();
            DetectCircularMotion();
            DetectLinearSwipe();
        }
    }
    
    bool IsShakeGestureDetected() const { return shake_detected_; }
    bool IsCircularMotionDetected() const { return circular_detected_; }
    bool IsLinearSwipeDetected() const { return swipe_detected_; }
    
private:
    ImuManager& imu_;
    
    // Gesture detection parameters
    static constexpr size_t gesture_window_size_ = 50;
    static constexpr float shake_threshold_ = 5.0f;        // m/s²
    static constexpr float circular_threshold_ = 2.0f;     // rad/s
    static constexpr float swipe_threshold_ = 3.0f;        // m/s²
    
    // Detection states
    bool shake_detected_ = false;
    bool circular_detected_ = false;
    bool swipe_detected_ = false;
    
    // Gesture buffer
    std::vector<MotionData> gesture_buffer_;
    
    void AddGestureSample(const MotionData& sample) {
        gesture_buffer_.push_back(sample);
        if (gesture_buffer_.size() > gesture_window_size_) {
            gesture_buffer_.erase(gesture_buffer_.begin());
        }
    }
    
    void DetectShakeGesture() {
        if (gesture_buffer_.size() < gesture_window_size_) return;
        
        // Calculate acceleration variance
        float mean_accel = 0.0f;
        for (const auto& sample : gesture_buffer_) {
            float magnitude = sqrtf(
                sample.linear_acceleration.x * sample.linear_acceleration.x +
                sample.linear_acceleration.y * sample.linear_acceleration.y +
                sample.linear_acceleration.z * sample.linear_acceleration.z
            );
            mean_accel += magnitude;
        }
        mean_accel /= gesture_buffer_.size();
        
        float variance = 0.0f;
        for (const auto& sample : gesture_buffer_) {
            float magnitude = sqrtf(
                sample.linear_acceleration.x * sample.linear_acceleration.x +
                sample.linear_acceleration.y * sample.linear_acceleration.y +
                sample.linear_acceleration.z * sample.linear_acceleration.z
            );
            variance += (magnitude - mean_accel) * (magnitude - mean_accel);
        }
        variance /= gesture_buffer_.size();
        
        shake_detected_ = (sqrtf(variance) > shake_threshold_);
    }
    
    void DetectCircularMotion() {
        if (gesture_buffer_.size() < gesture_window_size_) return;
        
        // Detect consistent angular velocity
        float avg_angular_mag = 0.0f;
        for (const auto& sample : gesture_buffer_) {
            float magnitude = sqrtf(
                sample.angular_velocity.x * sample.angular_velocity.x +
                sample.angular_velocity.y * sample.angular_velocity.y +
                sample.angular_velocity.z * sample.angular_velocity.z
            );
            avg_angular_mag += magnitude;
        }
        avg_angular_mag /= gesture_buffer_.size();
        
        circular_detected_ = (avg_angular_mag > circular_threshold_);
    }
    
    void DetectLinearSwipe() {
        if (gesture_buffer_.size() < gesture_window_size_) return;
        
        // Detect dominant acceleration direction
        Vector3 total_accel = {0.0f, 0.0f, 0.0f};
        for (const auto& sample : gesture_buffer_) {
            total_accel.x += sample.linear_acceleration.x;
            total_accel.y += sample.linear_acceleration.y;
            total_accel.z += sample.linear_acceleration.z;
        }
        
        float magnitude = sqrtf(total_accel.x * total_accel.x + 
                               total_accel.y * total_accel.y + 
                               total_accel.z * total_accel.z);
        
        swipe_detected_ = (magnitude > swipe_threshold_ * gesture_window_size_);
    }
};
```

## 🚨 Error Handling

### Comprehensive Error Management

```cpp
void comprehensive_imu_error_handling() {
    auto& imu = ImuManager::GetInstance();
    
    // Test initialization
    if (!imu.EnsureInitialized()) {
        printf("ERROR: IMU Manager initialization failed\n");
        return;
    }
    
    printf("IMU error handling test\n");
    
    // Monitor for errors during operation
    uint32_t consecutive_errors = 0;
    uint32_t total_errors = 0;
    uint32_t total_updates = 0;
    
    for (int i = 0; i < 5000; i++) {
        total_updates++;
        
        if (imu.IsDataReady()) {
            auto status = imu.GetStatus();
            
            // Check for sensor connection
            if (!status.sensor_connected) {
                printf("ERROR: IMU sensor disconnected\n");
                consecutive_errors++;
            } else {
                consecutive_errors = 0;
            }
            
            // Check for initialization issues
            if (!status.sensor_initialized) {
                printf("ERROR: IMU sensor not properly initialized\n");
            }
            
            // Monitor error rate
            if (status.error_count > total_errors) {
                printf("ERROR: New IMU errors detected (total: %lu)\n", status.error_count);
                total_errors = status.error_count;
            }
            
            // Check data quality
            auto orientation = imu.GetOrientation();
            if (orientation.accuracy > 15.0f) {  // Poor accuracy
                printf("WARNING: Poor orientation accuracy (%.1f°)\n", orientation.accuracy);
            }
            
            // Check temperature
            float temp = imu.GetTemperature();
            if (temp > 75.0f || temp < -25.0f) {
                printf("WARNING: Extreme temperature (%.1f°C)\n", temp);
            }
            
            // Check update rate
            uint32_t update_rate = imu.GetUpdateRate();
            if (update_rate < 10) {  // Very low update rate
                printf("WARNING: Low update rate (%lu Hz)\n", update_rate);
            }
            
            // Critical error handling
            if (consecutive_errors > 100) {
                printf("CRITICAL: Too many consecutive errors, attempting recovery\n");
                
                // Attempt recovery
                if (imu.EnsureDeinitialized()) {
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    if (imu.EnsureInitialized()) {
                        printf("Recovery successful\n");
                        consecutive_errors = 0;
                    } else {
                        printf("Recovery failed\n");
                        break;
                    }
                }
            }
            
        } else {
            consecutive_errors++;
            if (consecutive_errors % 100 == 0) {
                printf("WARNING: %lu consecutive timeouts\n", consecutive_errors);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    // Final statistics
    float error_rate = 100.0f * total_errors / total_updates;
    printf("IMU error handling test complete:\n");
    printf("  Total updates: %lu\n", total_updates);
    printf("  Total errors: %lu (%.2f%%)\n", total_errors, error_rate);
    
    if (error_rate < 1.0f) {
        printf("IMU system operating normally\n");
    } else if (error_rate < 5.0f) {
        printf("Some issues detected, monitor system\n");
    } else {
        printf("Significant issues detected, service required\n");
    }
}
```

## 📚 See Also

- **[Bno08xHandler Documentation](../driver-handlers/BNO08X_HANDLER_README.md)** - BNO08x IMU driver
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[Motion Detection Guide](../tutorials/MOTION_DETECTION_GUIDE.md)** - Advanced motion detection
- **[I2C Interface Guide](../hardware/I2C_INTERFACE_GUIDE.md)** - I2C communication setup
- **[Sensor Fusion Guide](../tutorials/SENSOR_FUSION_GUIDE.md)** - Sensor fusion algorithms

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*