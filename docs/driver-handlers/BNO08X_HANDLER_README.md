# Bno08xHandler - IMU Sensor Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Bno08xHandler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-BNO08x-orange.svg)
![Interface](https://img.shields.io/badge/interface-I2C%20|%20SPI-green.svg)

**Unified handler for BNO08x IMU sensor family with multi-interface support**

</div>

## 📋 Overview

The `Bno08xHandler` is a unified handler for BNO08x IMU sensor family that provides a modern, comprehensive interface for 9-DOF sensor fusion. It supports both I2C and SPI communication interfaces, offers advanced motion detection, activity classification, and calibration management with complete exception-free design.

### ✨ Key Features

- **🔬 9-DOF Sensor Fusion**: Accelerometer, gyroscope, magnetometer integration
- **📡 Multi-Interface Support**: I2C and SPI communication interfaces
- **🎯 Advanced Motion Detection**: Tap, shake, step counting, pickup detection
- **📊 Activity Classification**: Real-time activity and gesture recognition
- **🔧 Calibration Management**: Automatic and manual sensor calibration
- **🛡️ Thread-Safe**: Concurrent access from multiple tasks
- **⚡ High Performance**: Optimized sensor fusion algorithms
- **🏥 Health Monitoring**: Comprehensive diagnostics and error handling

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   Bno08xHandler                                │
├─────────────────────────────────────────────────────────────────┤
│  Sensor Fusion    │ 9-DOF orientation and motion tracking      │
├─────────────────────────────────────────────────────────────────┤
│  Multi-Interface  │ I2C/SPI communication with adapters        │
├─────────────────────────────────────────────────────────────────┤
│  Motion Detection │ Tap, shake, step, activity classification  │
├─────────────────────────────────────────────────────────────────┤
│  BNO08x Driver    │ Low-level sensor register control          │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic IMU Operations

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
    
    // Create BNO08x handler with default configuration
    Bno08xHandler handler(*i2c);
    
    // Initialize handler
    if (handler.Initialize() != Bno08xError::SUCCESS) {
        printf("Failed to initialize BNO08x\n");
        return;
    }
    
    // Read IMU data
    Bno08xImuData imu_data;
    if (handler.ReadImuData(imu_data) == Bno08xError::SUCCESS) {
        printf("Acceleration: %.2f, %.2f, %.2f m/s²\n", 
               imu_data.acceleration.x, imu_data.acceleration.y, imu_data.acceleration.z);
        printf("Orientation: %.2f, %.2f, %.2f rad\n", 
               imu_data.euler.roll, imu_data.euler.pitch, imu_data.euler.yaw);
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class Bno08xHandler {
public:
    // I2C constructor
    explicit Bno08xHandler(BaseI2c& i2c_interface, 
                          const Bno08xConfig& config = GetDefaultConfig(),
                          BaseGpio* reset_gpio = nullptr,
                          BaseGpio* int_gpio = nullptr) noexcept;

    // SPI constructor
    explicit Bno08xHandler(BaseSpi& spi_interface,
                          const Bno08xConfig& config = GetDefaultConfig(),
                          BaseGpio* wake_gpio = nullptr,
                          BaseGpio* reset_gpio = nullptr,
                          BaseGpio* int_gpio = nullptr) noexcept;

    // Initialization
    Bno08xError Initialize() noexcept;
    Bno08xError Deinitialize() noexcept;
    bool IsInitialized() const noexcept;
    bool IsSensorReady() const noexcept;
    std::shared_ptr<BNO085> GetSensor() noexcept;
    Bno08xError Update() noexcept;
    bool HasNewData(BNO085Sensor sensor) const noexcept;
};
```

#### Sensor Data Reading
```cpp
// Complete IMU data
Bno08xError ReadImuData(Bno08xImuData& imu_data) noexcept;

// Individual sensor readings
Bno08xError ReadAcceleration(Bno08xVector3& acceleration) noexcept;
Bno08xError ReadGyroscope(Bno08xVector3& gyroscope) noexcept;
Bno08xError ReadMagnetometer(Bno08xVector3& magnetometer) noexcept;
Bno08xError ReadQuaternion(Bno08xQuaternion& quaternion) noexcept;
Bno08xError ReadEulerAngles(Bno08xEulerAngles& euler_angles) noexcept;
Bno08xError ReadLinearAcceleration(Bno08xVector3& linear_accel) noexcept;
Bno08xError ReadGravity(Bno08xVector3& gravity) noexcept;
```

#### Activity and Gesture Detection
```cpp
// Activity data
Bno08xError ReadActivityData(Bno08xActivityData& activity_data) noexcept;

// Step detection
Bno08xError CheckStepDetection(bool& step_detected, uint32_t& step_count) noexcept;

// Tap detection
Bno08xError CheckTapDetection(bool& tap_detected, bool& double_tap, uint8_t& direction) noexcept;

// Gesture detection
Bno08xError CheckGestureDetection(bool& shake, bool& pickup, bool& significant_motion) noexcept;
```

#### Calibration and Accuracy
```cpp
// Calibration management
Bno08xError ReadCalibrationStatus(Bno08xCalibrationStatus& calibration_status) noexcept;
Bno08xError StartCalibration() noexcept;
Bno08xError SaveCalibration() noexcept;
Bno08xError IsCalibrated(bool& calibrated) noexcept;
```

#### Sensor Configuration
```cpp
// Sensor enable/disable
Bno08xError EnableSensor(BNO085Sensor sensor, uint32_t interval_ms, float sensitivity = 0.0f) noexcept;
Bno08xError DisableSensor(BNO085Sensor sensor) noexcept;

// Configuration management
Bno08xError UpdateConfiguration(const Bno08xConfig& config) noexcept;
Bno08xError GetConfiguration(Bno08xConfig& config) noexcept;
```

#### Hardware Control
```cpp
// Hardware control
Bno08xError HardwareReset(uint32_t reset_duration_ms = 10) noexcept;
Bno08xError SetWakePin(bool wake_state) noexcept;
Bno08xError SetBootPin(bool boot_state) noexcept;
```

#### Callback Management
```cpp
// Event callbacks
void SetSensorCallback(SensorCallback callback) noexcept;
void ClearSensorCallback() noexcept;
```

#### Utility Methods
```cpp
// Conversion utilities
static void QuaternionToEuler(const Bno08xQuaternion& quaternion, Bno08xEulerAngles& euler_angles) noexcept;
static Bno08xConfig GetDefaultConfig() noexcept;

// Information
const char* GetDescription() const noexcept;
Bno08xError GetLastError() const noexcept;
BNO085Interface GetInterfaceType() const noexcept;
void DumpDiagnostics() const noexcept;
```

## 🎯 Hardware Support

### BNO08x Features

- **9-DOF Sensor Fusion**: Accelerometer, gyroscope, magnetometer
- **Orientation Tracking**: Quaternion and Euler angle output
- **Motion Detection**: Tap, shake, step counting, pickup detection
- **Activity Classification**: Real-time activity recognition
- **Magnetic Field Sensing**: Compass and heading calculation
- **Communication**: I2C (0x4A/0x4B) and SPI interfaces
- **Interrupt Support**: Configurable interrupt generation
- **Calibration**: Automatic and manual calibration procedures

### Communication Interfaces

The handler supports both I2C and SPI communication through transport adapters that bridge the HardFOC `BaseI2c` and `BaseSpi` interfaces with the BNO08x driver's transport layer.

## 📊 Examples

### Basic IMU Reading

```cpp
void basic_imu_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) return;
    
    Bno08xHandler handler(*i2c);
    if (handler.Initialize() != Bno08xError::SUCCESS) return;
    
    // Enable required sensors
    handler.EnableSensor(BNO085Sensor::ACCELEROMETER, 50);  // 20Hz
    handler.EnableSensor(BNO085Sensor::GYROSCOPE, 50);      // 20Hz
    handler.EnableSensor(BNO085Sensor::ROTATION_VECTOR, 50); // 20Hz
    
    // Read sensor data
    Bno08xImuData imu_data;
    if (handler.ReadImuData(imu_data) == Bno08xError::SUCCESS) {
        printf("Acceleration: %.2f, %.2f, %.2f m/s²\n", 
               imu_data.acceleration.x, imu_data.acceleration.y, imu_data.acceleration.z);
        printf("Angular velocity: %.2f, %.2f, %.2f rad/s\n", 
               imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z);
        printf("Orientation: %.2f, %.2f, %.2f rad\n", 
               imu_data.euler.roll, imu_data.euler.pitch, imu_data.euler.yaw);
    }
}
```

### Motion Detection

```cpp
void motion_detection_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) return;
    
    Bno08xHandler handler(*i2c);
    if (handler.Initialize() != Bno08xError::SUCCESS) return;
    
    // Enable motion detection features
    handler.EnableSensor(BNO085Sensor::TAP_DETECTOR, 0);
    handler.EnableSensor(BNO085Sensor::STEP_COUNTER, 0);
    handler.EnableSensor(BNO085Sensor::SHAKE_DETECTOR, 0);
    
    // Monitor for events
    Bno08xActivityData activity_data;
    if (handler.ReadActivityData(activity_data) == Bno08xError::SUCCESS) {
        if (activity_data.tap_detected) {
            printf("Tap detected! Double: %s, Direction: %d\n", 
                   activity_data.double_tap ? "Yes" : "No", activity_data.tap_direction);
        }
        
        if (activity_data.step_detected) {
            printf("Step detected! Total steps: %u\n", activity_data.step_count);
        }
        
        if (activity_data.shake_detected) {
            printf("Shake detected!\n");
        }
    }
}
```

### SPI Interface Usage

```cpp
void spi_interface_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_1);
    if (!spi) return;
    
    // Create handler with SPI interface
    Bno08xHandler handler(*spi);
    if (handler.Initialize() != Bno08xError::SUCCESS) return;
    
    // Enable sensors
    handler.EnableSensor(BNO085Sensor::ACCELEROMETER, 100);  // 10Hz
    handler.EnableSensor(BNO085Sensor::ROTATION_VECTOR, 100); // 10Hz
    
    // Read data
    Bno08xVector3 acceleration;
    if (handler.ReadAcceleration(acceleration) == Bno08xError::SUCCESS) {
        printf("SPI Acceleration: %.2f, %.2f, %.2f m/s²\n", 
               acceleration.x, acceleration.y, acceleration.z);
    }
}
```

### Calibration Management

```cpp
void calibration_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) return;
    
    Bno08xHandler handler(*i2c);
    if (handler.Initialize() != Bno08xError::SUCCESS) return;
    
    // Check calibration status
    Bno08xCalibrationStatus cal_status;
    if (handler.ReadCalibrationStatus(cal_status) == Bno08xError::SUCCESS) {
        printf("Accelerometer accuracy: %d\n", cal_status.accelerometer_accuracy);
        printf("Gyroscope accuracy: %d\n", cal_status.gyroscope_accuracy);
        printf("Magnetometer accuracy: %d\n", cal_status.magnetometer_accuracy);
        printf("System accuracy: %d\n", cal_status.system_accuracy);
        printf("Calibration complete: %s\n", cal_status.calibration_complete ? "Yes" : "No");
    }
    
    // Start calibration if needed
    bool calibrated;
    if (handler.IsCalibrated(calibrated) == Bno08xError::SUCCESS && !calibrated) {
        printf("Starting calibration...\n");
        if (handler.StartCalibration() == Bno08xError::SUCCESS) {
            printf("Calibration started successfully\n");
        }
    }
}
```

### Advanced Configuration

```cpp
void advanced_config_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) return;
    
    // Create custom configuration
    Bno08xConfig config = Bno08xHandler::GetDefaultConfig();
    config.enable_accelerometer = true;
    config.enable_gyroscope = true;
    config.enable_magnetometer = true;
    config.enable_rotation_vector = true;
    config.enable_tap_detector = true;
    config.enable_step_counter = true;
    config.accelerometer_interval_ms = 20;  // 50Hz
    config.gyroscope_interval_ms = 20;      // 50Hz
    config.rotation_interval_ms = 20;       // 50Hz
    config.auto_calibration = true;
    
    Bno08xHandler handler(*i2c, config);
    if (handler.Initialize() != Bno08xError::SUCCESS) return;
    
    // Set up sensor callback
    handler.SetSensorCallback([](const SensorEvent& event) {
        printf("Sensor event: type=%d, data=%f\n", 
               static_cast<int>(event.sensor), event.data);
    });
    
    // Continuous monitoring
    while (true) {
        handler.Update();  // Process sensor data
        
        Bno08xImuData imu_data;
        if (handler.ReadImuData(imu_data) == Bno08xError::SUCCESS) {
            // Process IMU data
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz update rate
    }
}
```

### Error Handling

```cpp
void error_handling_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!i2c) return;
    
    Bno08xHandler handler(*i2c);
    
    // Check initialization
    Bno08xError result = handler.Initialize();
    if (result != Bno08xError::SUCCESS) {
        printf("ERROR: Failed to initialize BNO08x: %s\n", 
               Bno08xErrorToString(result));
        return;
    }
    
    // Safe sensor operations
    Bno08xVector3 acceleration;
    result = handler.ReadAcceleration(acceleration);
    if (result != Bno08xError::SUCCESS) {
        printf("ERROR: Failed to read acceleration: %s\n", 
               Bno08xErrorToString(result));
        return;
    }
    
    // Check sensor readiness
    if (!handler.IsSensorReady()) {
        printf("ERROR: Sensor not ready\n");
        return;
    }
    
    // Get last error for debugging
    Bno08xError last_error = handler.GetLastError();
    if (last_error != Bno08xError::SUCCESS) {
        printf("Last error: %s\n", Bno08xErrorToString(last_error));
    }
}
```

## 🔍 Advanced Usage

### Factory Methods

```cpp
void factory_methods_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Create I2C handler using factory method
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (i2c) {
        auto i2c_handler = CreateBno08xHandlerI2c(*i2c);
        if (i2c_handler && i2c_handler->Initialize() == Bno08xError::SUCCESS) {
            printf("I2C handler created successfully\n");
        }
    }
    
    // Create SPI handler using factory method
    auto* spi = comm.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_1);
    if (spi) {
        auto spi_handler = CreateBno08xHandlerSpi(*spi);
        if (spi_handler && spi_handler->Initialize() == Bno08xError::SUCCESS) {
            printf("SPI handler created successfully\n");
        }
    }
}
```

### Multi-Sensor Integration

```cpp
void multi_sensor_integration() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Create multiple BNO08x handlers
    std::vector<std::unique_ptr<Bno08xHandler>> handlers;
    
    // I2C handler
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (i2c) {
        auto handler = CreateBno08xHandlerI2c(*i2c);
        if (handler && handler->Initialize() == Bno08xError::SUCCESS) {
            handlers.push_back(std::move(handler));
        }
    }
    
    // SPI handler
    auto* spi = comm.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_1);
    if (spi) {
        auto handler = CreateBno08xHandlerSpi(*spi);
        if (handler && handler->Initialize() == Bno08xError::SUCCESS) {
            handlers.push_back(std::move(handler));
        }
    }
    
    // Process data from all sensors
    for (size_t i = 0; i < handlers.size(); i++) {
        Bno08xImuData imu_data;
        if (handlers[i]->ReadImuData(imu_data) == Bno08xError::SUCCESS) {
            printf("Sensor %zu: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", 
                   i, imu_data.euler.roll, imu_data.euler.pitch, imu_data.euler.yaw);
        }
    }
}
```

## 📚 See Also

- **[ImuManager Documentation](../component-handlers/IMU_MANAGER_README.md)** - IMU management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[TMC9660 Handler Documentation](TMC9660_HANDLER_README.md)** - Motor controller handler
- **[AS5047U Handler Documentation](AS5047U_HANDLER_README.md)** - Position encoder handler

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*