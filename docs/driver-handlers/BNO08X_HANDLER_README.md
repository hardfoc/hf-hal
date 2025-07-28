# Bno08xHandler - IMU Sensor Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Bno08xHandler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-BNO08x-orange.svg)
![Interface](https://img.shields.io/badge/interface-I2C-green.svg)

**Advanced 9-axis IMU sensor driver handler with sensor fusion and calibration**

</div>

## 📋 Overview

The `Bno08xHandler` is a comprehensive driver handler for the BNO08x family of 9-axis IMU sensors. It provides advanced sensor fusion, automatic calibration, and high-precision orientation data with multiple output formats including quaternions, Euler angles, and rotation vectors.

### ✨ Key Features

- **🧭 9-Axis Sensor Fusion**: Accelerometer, gyroscope, and magnetometer
- **🎯 High Precision**: Advanced sensor fusion algorithms
- **📡 I2C Interface**: Flexible communication with interrupt support
- **🔄 Multiple Output Formats**: Quaternions, Euler angles, rotation vectors
- **⚙️ Automatic Calibration**: Dynamic accuracy improvement
- **🔔 Interrupt Support**: Data-ready and motion detection
- **📊 Rich Feature Set**: Step counter, stability classifier, tap detection
- **🏥 Health Monitoring**: Sensor status and accuracy reporting

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Bno08xHandler                               │
├─────────────────────────────────────────────────────────────────┤
│  Sensor Fusion     │ Quaternions, Euler angles, vectors      │
├─────────────────────────────────────────────────────────────────┤
│  I2C Communication │ SHTP protocol with packet handling       │
├─────────────────────────────────────────────────────────────────┤
│  Feature Engine    │ Motion detection, tap, step counter     │
├─────────────────────────────────────────────────────────────────┤
│  BNO08x Driver     │ Low-level sensor and calibration        │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic IMU Reading

```cpp
#include "utils-and-drivers/driver-handlers/Bno08xHandler.h"
#include "component-handlers/CommChannelsManager.h"

void bno08x_basic_example() {
    // Get I2C interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) {
        printf("I2C interface not available\n");
        return;
    }
    
    // Create BNO08x handler
    Bno08xHandler imu(*i2c, 0x4A);  // Address 0x4A
    
    // Initialize IMU
    if (!imu.Initialize()) {
        printf("Failed to initialize BNO08x\n");
        return;
    }
    
    printf("BNO08x IMU ready\n");
    
    // Read orientation continuously
    for (int i = 0; i < 100; i++) {
        if (imu.DataAvailable()) {
            auto euler = imu.GetEulerAngles();
            auto quat = imu.GetQuaternion();
            
            printf("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
                   euler.roll, euler.pitch, euler.yaw);
            printf("Quat: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                   quat.w, quat.x, quat.y, quat.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class Bno08xHandler {
public:
    // Constructor
    Bno08xHandler(BaseI2c& i2c_interface, uint8_t device_address);
    
    // Initialization
    bool Initialize() noexcept;
    bool IsInitialized() const noexcept;
    void Deinitialize() noexcept;
    
    // Communication testing
    bool TestCommunication() noexcept;
    
    // Data availability
    bool DataAvailable() noexcept;
    void Update() noexcept;  // Process new data
};
```

#### Orientation Data
```cpp
// Quaternion representation
struct Quaternion {
    float w, x, y, z;
    float accuracy;  // Accuracy estimate (radians)
};

// Euler angles representation
struct EulerAngles {
    float roll, pitch, yaw;  // Degrees
    float accuracy;          // Accuracy estimate (degrees)
};

// Rotation vector
struct RotationVector {
    float i, j, k;    // Rotation vector components
    float accuracy;   // Accuracy estimate
};

Quaternion GetQuaternion() noexcept;
EulerAngles GetEulerAngles() noexcept;
RotationVector GetRotationVector() noexcept;
```

#### Raw Sensor Data
```cpp
// Raw sensor readings
struct Vector3 {
    float x, y, z;
};

struct AccelerometerData {
    Vector3 acceleration;  // m/s²
    uint8_t accuracy;
};

struct GyroscopeData {
    Vector3 angular_velocity;  // rad/s
    uint8_t accuracy;
};

struct MagnetometerData {
    Vector3 magnetic_field;  // µT
    uint8_t accuracy;
};

AccelerometerData GetAccelerometer() noexcept;
GyroscopeData GetGyroscope() noexcept;
MagnetometerData GetMagnetometer() noexcept;
```

#### Feature Detection
```cpp
// Motion and activity detection
struct ActivityData {
    bool walking;
    bool running;
    bool stationary;
    bool in_vehicle;
    uint8_t confidence;
};

struct TapData {
    bool single_tap;
    bool double_tap;
    uint8_t direction;  // Tap direction
};

struct StabilityData {
    bool stable;
    uint8_t classification;
};

ActivityData GetActivity() noexcept;
TapData GetTapDetection() noexcept;
StabilityData GetStability() noexcept;
uint32_t GetStepCount() noexcept;
```

#### Configuration and Calibration
```cpp
// Report configuration
bool EnableRotationVector(uint16_t period_ms) noexcept;
bool EnableQuaternion(uint16_t period_ms) noexcept;
bool EnableEulerAngles(uint16_t period_ms) noexcept;
bool EnableAccelerometer(uint16_t period_ms) noexcept;
bool EnableGyroscope(uint16_t period_ms) noexcept;
bool EnableMagnetometer(uint16_t period_ms) noexcept;

// Feature enables
bool EnableActivityClassifier(uint16_t period_ms) noexcept;
bool EnableTapDetector(bool single_tap, bool double_tap) noexcept;
bool EnableStepCounter() noexcept;
bool EnableStabilityClassifier() noexcept;

// Calibration
bool StartCalibration() noexcept;
bool SaveCalibration() noexcept;
bool LoadCalibration() noexcept;
uint8_t GetCalibrationStatus() noexcept;
```

### Status and Diagnostics

```cpp
// System status
struct SystemStatus {
    bool sensor_error;
    bool calibration_complete;
    bool system_ready;
    uint8_t accuracy_magnetometer;
    uint8_t accuracy_accelerometer;
    uint8_t accuracy_gyroscope;
    uint8_t system_calibration;
};

SystemStatus GetSystemStatus() noexcept;
bool IsCalibrated() noexcept;
float GetTemperature() noexcept;  // °C
```

## 🎯 Hardware Features

### BNO08x Capabilities

| Feature | Specification | Description |
|---------|---------------|-------------|
| **Processor** | 32-bit Cortex M0+ | Dedicated sensor fusion processor |
| **Accelerometer** | ±2/4/8/16g | 3-axis MEMS accelerometer |
| **Gyroscope** | ±125/250/500/1000/2000°/s | 3-axis MEMS gyroscope |
| **Magnetometer** | ±1300µT | 3-axis magnetic sensor |
| **Update Rate** | Up to 1000Hz | Configurable output rates |
| **I2C Interface** | Standard/Fast mode | Up to 400kHz |
| **Supply Voltage** | 2.4V - 3.6V | Low power operation |
| **Temperature Range** | -40°C to +85°C | Industrial temperature range |

### Sensor Fusion Features

```cpp
// Available sensor fusion outputs
enum class SensorOutput {
    ROTATION_VECTOR = 0x05,      // Most accurate orientation
    GAME_ROTATION_VECTOR = 0x08, // No magnetometer
    GEOMAGNETIC_ROTATION = 0x09, // Geomagnetic reference
    QUATERNION = 0x05,           // Quaternion format
    EULER_ANGLES = 0x01,         // Roll, pitch, yaw
    LINEAR_ACCELERATION = 0x04,  // Gravity removed
    GRAVITY_VECTOR = 0x06,       // Gravity component
    ACCELEROMETER = 0x01,        // Raw accelerometer
    GYROSCOPE = 0x02,            // Raw gyroscope  
    MAGNETOMETER = 0x03,         // Raw magnetometer
    PRESSURE = 0x0A,             // Barometric pressure (if available)
    TEMPERATURE = 0x0B           // Temperature sensor
};
```

## 🔧 Configuration

### I2C Communication Setup

```cpp
// BNO08x I2C configuration
struct Bno08xI2cConfig {
    uint8_t device_address = 0x4A;      // Default address (0x4B alt)
    uint32_t clock_speed_hz = 400000;   // 400 kHz fast mode
    uint32_t timeout_ms = 100;          // Communication timeout
    bool use_interrupts = true;         // Use interrupt pin
    uint8_t interrupt_pin = GPIO_NUM_NC; // Interrupt GPIO pin
};
```

### Initialization Configuration

```cpp
// Initialization options
struct Bno08xConfig {
    // Default report rates (0 = disabled)
    uint16_t rotation_vector_period_ms = 20;    // 50 Hz
    uint16_t quaternion_period_ms = 0;          // Disabled
    uint16_t euler_angles_period_ms = 0;        // Disabled
    uint16_t accelerometer_period_ms = 0;       // Disabled
    uint16_t gyroscope_period_ms = 0;           // Disabled
    uint16_t magnetometer_period_ms = 0;        // Disabled
    
    // Feature enables
    bool enable_activity_classifier = false;
    bool enable_tap_detector = false;
    bool enable_step_counter = false;
    bool enable_stability_classifier = false;
    
    // Calibration options
    bool auto_calibration = true;
    bool save_calibration_on_shutdown = true;
    
    // Power management
    bool low_power_mode = false;
    uint16_t wake_up_delay_ms = 100;
};
```

## 📊 Examples

### Basic Orientation Monitoring

```cpp
#include "utils-and-drivers/driver-handlers/Bno08xHandler.h"

void orientation_monitoring_example() {
    // Setup I2C communication
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    
    // Create IMU handler
    Bno08xHandler imu(*i2c, 0x4A);
    
    if (!imu.Initialize()) {
        printf("BNO08x initialization failed\n");
        return;
    }
    
    // Enable rotation vector at 50Hz
    imu.EnableRotationVector(20);
    
    printf("BNO08x orientation monitoring started\n");
    
    for (int i = 0; i < 1000; i++) {
        if (imu.DataAvailable()) {
            imu.Update();
            
            // Get orientation data
            auto euler = imu.GetEulerAngles();
            auto quat = imu.GetQuaternion();
            
            // Display orientation
            printf("Orientation: R=%6.1f° P=%6.1f° Y=%6.1f° (acc=%.1f°)\n",
                   euler.roll, euler.pitch, euler.yaw, euler.accuracy);
            
            // Check calibration status
            auto status = imu.GetSystemStatus();
            if (!status.calibration_complete) {
                printf("Calibrating... Mag:%u Acc:%u Gyr:%u Sys:%u\n",
                       status.accuracy_magnetometer,
                       status.accuracy_accelerometer,
                       status.accuracy_gyroscope,
                       status.system_calibration);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### Multi-Sensor Data Logging

```cpp
void multi_sensor_logging_example() {
    Bno08xHandler imu(*i2c, 0x4A);
    imu.Initialize();
    
    // Enable multiple sensors
    imu.EnableRotationVector(20);    // 50 Hz orientation
    imu.EnableAccelerometer(10);     // 100 Hz accelerometer
    imu.EnableGyroscope(10);         // 100 Hz gyroscope
    imu.EnableMagnetometer(50);      // 20 Hz magnetometer
    
    printf("Multi-sensor data logging\n");
    printf("Time,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ\n");
    
    uint32_t start_time = esp_timer_get_time() / 1000;
    
    for (int i = 0; i < 2000; i++) {
        if (imu.DataAvailable()) {
            imu.Update();
            
            uint32_t timestamp = (esp_timer_get_time() / 1000) - start_time;
            
            // Get all sensor data
            auto euler = imu.GetEulerAngles();
            auto accel = imu.GetAccelerometer();
            auto gyro = imu.GetGyroscope();
            auto mag = imu.GetMagnetometer();
            
            // Log to CSV format
            printf("%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n",
                   timestamp,
                   euler.roll, euler.pitch, euler.yaw,
                   accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                   gyro.angular_velocity.x, gyro.angular_velocity.y, gyro.angular_velocity.z,
                   mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));  // 200 Hz loop
    }
}
```

### Motion Detection and Activity Classification

```cpp
void motion_detection_example() {
    Bno08xHandler imu(*i2c, 0x4A);
    imu.Initialize();
    
    // Enable motion features
    imu.EnableActivityClassifier(1000);  // 1 Hz activity updates
    imu.EnableTapDetector(true, true);   // Single and double tap
    imu.EnableStepCounter();
    imu.EnableStabilityClassifier();
    
    printf("Motion detection and activity classification\n");
    
    uint32_t last_step_count = 0;
    
    for (int i = 0; i < 6000; i++) {  // 10 minutes
        if (imu.DataAvailable()) {
            imu.Update();
            
            // Check for tap detection
            auto tap = imu.GetTapDetection();
            if (tap.single_tap) {
                printf("Single tap detected!\n");
            }
            if (tap.double_tap) {
                printf("Double tap detected!\n");
            }
            
            // Check activity classification
            auto activity = imu.GetActivity();
            if (activity.confidence > 50) {  // High confidence
                if (activity.walking) {
                    printf("Activity: Walking (confidence: %u%%)\n", activity.confidence);
                } else if (activity.running) {
                    printf("Activity: Running (confidence: %u%%)\n", activity.confidence);
                } else if (activity.in_vehicle) {
                    printf("Activity: In Vehicle (confidence: %u%%)\n", activity.confidence);
                } else if (activity.stationary) {
                    printf("Activity: Stationary (confidence: %u%%)\n", activity.confidence);
                }
            }
            
            // Check step counter
            uint32_t step_count = imu.GetStepCount();
            if (step_count != last_step_count) {
                printf("Step count: %lu (new steps: %lu)\n", 
                       step_count, step_count - last_step_count);
                last_step_count = step_count;
            }
            
            // Check stability
            auto stability = imu.GetStability();
            static bool was_stable = true;
            if (stability.stable != was_stable) {
                printf("Stability changed: %s\n", stability.stable ? "STABLE" : "UNSTABLE");
                was_stable = stability.stable;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Calibration Procedure

```cpp
void calibration_procedure_example() {
    Bno08xHandler imu(*i2c, 0x4A);
    imu.Initialize();
    
    // Enable sensors for calibration
    imu.EnableRotationVector(50);  // 20 Hz for calibration
    
    printf("BNO08x Calibration Procedure\n");
    printf("============================\n");
    
    // Start calibration
    if (!imu.StartCalibration()) {
        printf("Failed to start calibration\n");
        return;
    }
    
    printf("Calibration started. Follow these steps:\n");
    printf("1. Place device on flat surface for 5 seconds\n");
    printf("2. Slowly rotate around all 3 axes\n");
    printf("3. Move in figure-8 pattern for magnetometer\n");
    printf("4. Wait for calibration to complete\n\n");
    
    bool calibration_complete = false;
    uint32_t calibration_start = esp_timer_get_time() / 1000000;
    
    while (!calibration_complete && 
           ((esp_timer_get_time() / 1000000) - calibration_start) < 120) {  // 2 min timeout
        
        if (imu.DataAvailable()) {
            imu.Update();
            
            auto status = imu.GetSystemStatus();
            
            printf("Calibration Status: Mag:%u Acc:%u Gyr:%u Sys:%u",
                   status.accuracy_magnetometer,
                   status.accuracy_accelerometer,
                   status.accuracy_gyroscope,
                   status.system_calibration);
            
            // Display calibration progress
            if (status.accuracy_magnetometer < 2) {
                printf(" [Move in figure-8]");
            } else if (status.accuracy_accelerometer < 2) {
                printf(" [Rotate slowly around all axes]");
            } else if (status.accuracy_gyroscope < 2) {
                printf(" [Keep moving slowly]");
            } else if (status.system_calibration < 3) {
                printf(" [Almost complete...]");
            } else {
                printf(" [COMPLETE!]");
                calibration_complete = true;
            }
            
            printf("\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    if (calibration_complete) {
        printf("Calibration completed successfully!\n");
        
        // Save calibration data
        if (imu.SaveCalibration()) {
            printf("Calibration data saved to sensor memory\n");
        } else {
            printf("Failed to save calibration data\n");
        }
        
        // Test calibrated accuracy
        printf("\nTesting calibrated accuracy:\n");
        for (int i = 0; i < 20; i++) {
            if (imu.DataAvailable()) {
                imu.Update();
                auto euler = imu.GetEulerAngles();
                printf("Accuracy: %.2f° - Orientation: R=%.1f° P=%.1f° Y=%.1f°\n",
                       euler.accuracy, euler.roll, euler.pitch, euler.yaw);
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    } else {
        printf("Calibration timeout. Please try again.\n");
    }
}
```

### Advanced Quaternion Processing

```cpp
void quaternion_processing_example() {
    Bno08xHandler imu(*i2c, 0x4A);
    imu.Initialize();
    
    // Enable quaternion output at high rate
    imu.EnableQuaternion(5);  // 200 Hz
    
    printf("Advanced quaternion processing\n");
    
    Quaternion reference_quat = {1.0f, 0.0f, 0.0f, 0.0f};  // Identity quaternion
    bool reference_set = false;
    
    for (int i = 0; i < 2000; i++) {
        if (imu.DataAvailable()) {
            imu.Update();
            
            auto current_quat = imu.GetQuaternion();
            
            // Set reference orientation on first stable reading
            if (!reference_set && current_quat.accuracy < 5.0f) {  // Good accuracy
                reference_quat = current_quat;
                reference_set = true;
                printf("Reference orientation set\n");
            }
            
            if (reference_set) {
                // Calculate relative rotation from reference
                Quaternion relative = QuaternionDifference(reference_quat, current_quat);
                
                // Convert to axis-angle representation
                auto axis_angle = QuaternionToAxisAngle(relative);
                
                // Convert to Euler angles
                auto euler_relative = QuaternionToEuler(relative);
                
                printf("Relative: R=%6.1f° P=%6.1f° Y=%6.1f° | Axis: (%.2f,%.2f,%.2f) Angle: %.1f°\n",
                       euler_relative.roll, euler_relative.pitch, euler_relative.yaw,
                       axis_angle.axis.x, axis_angle.axis.y, axis_angle.axis.z,
                       axis_angle.angle * 180.0f / M_PI);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Helper functions for quaternion math
struct AxisAngle {
    Vector3 axis;
    float angle;  // radians
};

Quaternion QuaternionDifference(const Quaternion& q1, const Quaternion& q2) {
    // Calculate q1^-1 * q2
    Quaternion q1_inv = {q1.w, -q1.x, -q1.y, -q1.z};  // Conjugate for unit quaternion
    return QuaternionMultiply(q1_inv, q2);
}

Quaternion QuaternionMultiply(const Quaternion& q1, const Quaternion& q2) {
    return {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
        0.0f  // Accuracy not calculated
    };
}

AxisAngle QuaternionToAxisAngle(const Quaternion& q) {
    AxisAngle result;
    result.angle = 2.0f * acosf(fabsf(q.w));
    float sin_half_angle = sinf(result.angle / 2.0f);
    
    if (sin_half_angle > 0.001f) {
        result.axis.x = q.x / sin_half_angle;
        result.axis.y = q.y / sin_half_angle;
        result.axis.z = q.z / sin_half_angle;
    } else {
        result.axis = {1.0f, 0.0f, 0.0f};  // Default axis
    }
    
    return result;
}

EulerAngles QuaternionToEuler(const Quaternion& q) {
    EulerAngles euler;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1)
        euler.pitch = copysignf(90.0f, sinp);  // Use 90 degrees if out of range
    else
        euler.pitch = asinf(sinp) * 180.0f / M_PI;
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    euler.accuracy = q.accuracy * 180.0f / M_PI;  // Convert to degrees
    
    return euler;
}
```

### Performance Optimization

```cpp
void performance_optimization_example() {
    Bno08xHandler imu(*i2c, 0x4A);
    imu.Initialize();
    
    printf("BNO08x Performance Optimization\n");
    printf("===============================\n");
    
    // Test different update rates
    uint16_t test_rates[] = {5, 10, 20, 50, 100};  // ms
    
    for (auto rate : test_rates) {
        printf("\nTesting %u ms update rate (%.1f Hz):\n", rate, 1000.0f / rate);
        
        // Configure sensor
        imu.EnableRotationVector(rate);
        vTaskDelay(pdMS_TO_TICKS(500));  // Settle time
        
        // Measure performance
        uint32_t update_count = 0;
        uint32_t total_latency = 0;
        uint32_t start_time = esp_timer_get_time() / 1000;
        
        for (int i = 0; i < 1000; i++) {
            uint32_t loop_start = esp_timer_get_time();
            
            if (imu.DataAvailable()) {
                imu.Update();
                update_count++;
                
                uint32_t latency = (esp_timer_get_time() - loop_start);
                total_latency += latency;
            }
            
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        uint32_t test_duration = (esp_timer_get_time() / 1000) - start_time;
        float actual_rate = (float)update_count * 1000.0f / test_duration;
        float avg_latency = (float)total_latency / update_count;
        
        printf("  Actual rate: %.1f Hz\n", actual_rate);
        printf("  Average latency: %.2f µs\n", avg_latency);
        printf("  CPU usage: %.1f%%\n", (avg_latency * actual_rate) / 10000.0f);
    }
    
    // Memory usage analysis
    printf("\nMemory usage:\n");
    printf("  Handler size: %zu bytes\n", sizeof(Bno08xHandler));
    
    // Test batch vs individual reads
    printf("\nTesting batch operations:\n");
    
    imu.EnableRotationVector(10);
    imu.EnableAccelerometer(10);
    imu.EnableGyroscope(10);
    
    auto start_time = esp_timer_get_time();
    
    for (int i = 0; i < 100; i++) {
        if (imu.DataAvailable()) {
            imu.Update();
            
            // All data available from single update
            auto euler = imu.GetEulerAngles();
            auto accel = imu.GetAccelerometer();
            auto gyro = imu.GetGyroscope();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    auto batch_time = esp_timer_get_time() - start_time;
    printf("  Batch read time: %llu µs\n", batch_time);
    
    printf("Performance optimization complete\n");
}
```

## 🔍 Advanced Usage

### Custom Motion Detection

```cpp
// Example of creating custom motion detection algorithms
class CustomMotionDetector {
public:
    CustomMotionDetector(Bno08xHandler& imu) : imu_(imu) {}
    
    void Initialize() {
        // Enable high-rate sensor data
        imu_.EnableAccelerometer(5);   // 200 Hz
        imu_.EnableGyroscope(5);       // 200 Hz
        
        // Initialize detection parameters
        shake_threshold_ = 2.0f;       // g
        rotation_threshold_ = 3.0f;    // rad/s
        stillness_threshold_ = 0.1f;   // g
        window_size_ = 20;             // samples
    }
    
    void Update() {
        if (imu_.DataAvailable()) {
            imu_.Update();
            
            auto accel = imu_.GetAccelerometer();
            auto gyro = imu_.GetGyroscope();
            
            // Add to circular buffers
            AddAccelSample(accel.acceleration);
            AddGyroSample(gyro.angular_velocity);
            
            // Detect motion patterns
            DetectShake();
            DetectFreefall();
            DetectStillness();
            DetectRotation();
        }
    }
    
    bool IsShaking() const { return shake_detected_; }
    bool IsFreefalling() const { return freefall_detected_; }
    bool IsStill() const { return still_detected_; }
    bool IsRotating() const { return rotation_detected_; }
    
private:
    Bno08xHandler& imu_;
    
    // Detection thresholds
    float shake_threshold_;
    float rotation_threshold_;
    float stillness_threshold_;
    size_t window_size_;
    
    // Detection states
    bool shake_detected_ = false;
    bool freefall_detected_ = false;
    bool still_detected_ = false;
    bool rotation_detected_ = false;
    
    // Circular buffers for sample history
    std::vector<Vector3> accel_history_;
    std::vector<Vector3> gyro_history_;
    size_t buffer_index_ = 0;
    
    void AddAccelSample(const Vector3& sample) {
        if (accel_history_.size() < window_size_) {
            accel_history_.push_back(sample);
        } else {
            accel_history_[buffer_index_] = sample;
        }
    }
    
    void AddGyroSample(const Vector3& sample) {
        if (gyro_history_.size() < window_size_) {
            gyro_history_.push_back(sample);
        } else {
            gyro_history_[buffer_index_] = sample;
        }
        
        buffer_index_ = (buffer_index_ + 1) % window_size_;
    }
    
    void DetectShake() {
        if (accel_history_.size() < window_size_) return;
        
        // Calculate acceleration magnitude variance
        float mean_mag = 0.0f;
        for (const auto& sample : accel_history_) {
            float mag = sqrtf(sample.x*sample.x + sample.y*sample.y + sample.z*sample.z);
            mean_mag += mag;
        }
        mean_mag /= accel_history_.size();
        
        float variance = 0.0f;
        for (const auto& sample : accel_history_) {
            float mag = sqrtf(sample.x*sample.x + sample.y*sample.y + sample.z*sample.z);
            variance += (mag - mean_mag) * (mag - mean_mag);
        }
        variance /= accel_history_.size();
        
        shake_detected_ = (sqrtf(variance) > shake_threshold_);
    }
    
    void DetectFreefall() {
        if (accel_history_.empty()) return;
        
        const auto& latest = accel_history_.back();
        float total_accel = sqrtf(latest.x*latest.x + latest.y*latest.y + latest.z*latest.z);
        
        // Freefall detected when total acceleration is close to zero
        freefall_detected_ = (total_accel < 0.5f);  // Less than 0.5g
    }
    
    void DetectStillness() {
        if (accel_history_.size() < window_size_) return;
        
        // Check if all recent samples are within stillness threshold
        bool all_still = true;
        for (const auto& sample : accel_history_) {
            float deviation = fabsf(sqrtf(sample.x*sample.x + sample.y*sample.y + sample.z*sample.z) - 9.81f);
            if (deviation > stillness_threshold_) {
                all_still = false;
                break;
            }
        }
        
        still_detected_ = all_still;
    }
    
    void DetectRotation() {
        if (gyro_history_.empty()) return;
        
        const auto& latest = gyro_history_.back();
        float total_rotation = sqrtf(latest.x*latest.x + latest.y*latest.y + latest.z*latest.z);
        
        rotation_detected_ = (total_rotation > rotation_threshold_);
    }
};
```

## 🚨 Error Handling

### Comprehensive Error Management

```cpp
void comprehensive_error_handling() {
    auto& comm = CommChannelsManager::GetInstance();
    if (!comm.EnsureInitialized()) {
        printf("ERROR: Communication manager failed\n");
        return;
    }
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) {
        printf("ERROR: I2C device not available\n");
        return;
    }
    
    if (!i2c->IsInitialized()) {
        printf("ERROR: I2C interface not initialized\n");
        return;
    }
    
    try {
        Bno08xHandler imu(*i2c, 0x4A);
        
        // Test communication on both possible addresses
        if (!imu.TestCommunication()) {
            printf("Trying alternative address 0x4B...\n");
            Bno08xHandler imu_alt(*i2c, 0x4B);
            if (!imu_alt.TestCommunication()) {
                printf("ERROR: BNO08x not responding on either address\n");
                return;
            }
            printf("Found BNO08x at address 0x4B\n");
            // Continue with alternative address device
        }
        
        // Initialize with error checking
        if (!imu.Initialize()) {
            printf("ERROR: BNO08x initialization failed\n");
            return;
        }
        
        printf("BNO08x initialized successfully\n");
        
        // Enable basic sensor
        if (!imu.EnableRotationVector(50)) {
            printf("ERROR: Failed to enable rotation vector\n");
            return;
        }
        
        // Monitor for errors during operation
        uint32_t error_count = 0;
        uint32_t total_updates = 0;
        uint32_t timeout_count = 0;
        
        for (int i = 0; i < 2000; i++) {
            total_updates++;
            
            if (imu.DataAvailable()) {
                imu.Update();
                
                // Check system status
                auto status = imu.GetSystemStatus();
                
                if (status.sensor_error) {
                    error_count++;
                    printf("ERROR %u: Sensor error detected\n", error_count);
                }
                
                if (!status.system_ready) {
                    printf("WARNING: System not ready\n");
                }
                
                // Check data quality
                auto euler = imu.GetEulerAngles();
                if (euler.accuracy > 10.0f) {  // Poor accuracy
                    printf("WARNING: Poor orientation accuracy (%.1f°)\n", euler.accuracy);
                }
                
                // Check temperature
                float temp = imu.GetTemperature();
                if (temp > 70.0f || temp < -30.0f) {
                    printf("WARNING: Extreme temperature (%.1f°C)\n", temp);
                }
                
            } else {
                timeout_count++;
                if (timeout_count % 100 == 0) {
                    printf("WARNING: %u timeouts in communication\n", timeout_count);
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        
        float error_rate = 100.0f * error_count / total_updates;
        float timeout_rate = 100.0f * timeout_count / total_updates;
        
        printf("Operation completed:\n");
        printf("  Errors: %u (%.2f%%)\n", error_count, error_rate);
        printf("  Timeouts: %u (%.2f%%)\n", timeout_count, timeout_rate);
        
        if (error_rate < 1.0f && timeout_rate < 5.0f) {
            printf("System operating normally\n");
        } else if (error_rate < 5.0f && timeout_rate < 20.0f) {
            printf("Some issues detected, check I2C connections\n");
        } else {
            printf("Significant issues detected, hardware service required\n");
        }
        
    } catch (const std::exception& e) {
        printf("EXCEPTION: %s\n", e.what());
    }
}
```

## 📚 See Also

- **[ImuManager Documentation](../component-handlers/IMU_MANAGER_README.md)** - IMU management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - I2C interface setup
- **[BNO08x Datasheet](https://www.ceva-dsp.com/product/bno080-085/)** - Official hardware documentation
- **[I2C Interface Guide](../hardware/I2C_INTERFACE_GUIDE.md)** - I2C communication setup
- **[Motion Detection Guide](../tutorials/MOTION_DETECTION_GUIDE.md)** - Advanced motion detection

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*