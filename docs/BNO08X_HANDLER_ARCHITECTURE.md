# BNO08x Handler Architecture Documentation

## Overview

The `Bno08xHandler` represents the pinnacle of IMU sensor handler design within the HardFOC ecosystem, following the same architectural excellence established by `TMC9660Handler`, `PCAL95555Handler`, and `AS5047UHandler`. This handler provides comprehensive management for the BNO08x family of 9-DOF sensor fusion devices with industry-leading embedded design patterns.

## 🏗️ Architectural Foundation

### Core Design Principles

1. **Bridge Pattern Integration**: Seamlessly connects HardFOC base interfaces to BNO08x transport layer
2. **Lazy Initialization**: Lightweight construction with deferred object creation during `Initialize()`
3. **Shared Pointer Management**: Safe cross-component sharing with automatic lifecycle management
4. **Exception-Free Design**: Complete `noexcept` methods with comprehensive error code returns
5. **Thread Safety**: `RtosMutex` protection with RAII lock guards for concurrent access
6. **Multi-Interface Support**: Unified API supporting I2C, SPI, and UART communication protocols

### Handler Hierarchy Position

```
BaseHandler (Abstract)
├── TMC9660Handler (Motor Driver)
├── PCAL95555Handler (GPIO Expander)  
├── AS5047UHandler (Magnetic Encoder)
└── Bno08xHandler (9-DOF IMU) ← Current Implementation
```

## 🌉 Bridge Pattern Architecture

The `Bno08xHandler` implements sophisticated bridge adapters to connect HardFOC base interfaces with the BNO08x driver's transport abstraction.

### Transport Adapter Classes

#### Bno08xI2cAdapter
```cpp
class Bno08xI2cAdapter : public IBNO085Transport {
    BaseI2c& i2c_interface_;      // HardFOC I2C interface
    BaseGpio* reset_gpio_;        // Hardware reset control
    BaseGpio* int_gpio_;          // Interrupt monitoring
    bool is_open_;                // Connection state
};
```

**Key Features:**
- Implements `IBNO085Transport` virtual interface using `BaseI2c`
- Hardware reset control via GPIO pin management
- Interrupt-driven data availability detection
- I2C address management (default 0x4A)
- Timing services integration with RTOS

#### Bno08xSpiAdapter
```cpp
class Bno08xSpiAdapter : public IBNO085Transport {
    BaseSpi& spi_interface_;      // HardFOC SPI interface
    BaseGpio* wake_gpio_;         // Wake pin control (SPI specific)
    BaseGpio* reset_gpio_;        // Hardware reset control
    BaseGpio* int_gpio_;          // Interrupt monitoring
    bool is_open_;                // Connection state
};
```

**Key Features:**
- SPI Mode 3 (CPOL=1, CPHA=1) configuration for BNO08x compatibility
- 3 MHz maximum clock frequency compliance
- Wake pin control for SPI interface selection
- Full-duplex transfer management
- Hardware control pin orchestration

### Bridge Interface Mapping

| IBNO085Transport Method | BaseI2c/BaseSpi Implementation |
|------------------------|--------------------------------|
| `open()` | `Initialize()` + GPIO setup |
| `close()` | `Deinitialize()` |
| `write()` | `Write()` / `Transfer()` |
| `read()` | `Read()` / `Transfer()` |
| `dataAvailable()` | GPIO interrupt pin monitoring |
| `delay()` | `RtosTask::Delay()` |
| `getTimeUs()` | RTOS tick conversion |
| `setReset()` | GPIO reset pin control |
| `setBoot()` | Boot mode pin control |
| `setWake()` | SPI wake pin control |

## 📊 Data Structure Architecture

### Enhanced Vector with Metadata
```cpp
struct Bno08xVector3 {
    float x, y, z;                // 3D vector components
    uint8_t accuracy;             // Sensor accuracy (0-3)
    uint64_t timestamp_us;        // Microsecond timestamp
    bool valid;                   // Data validity flag
};
```

### Enhanced Quaternion with Metadata
```cpp
struct Bno08xQuaternion {
    float w, x, y, z;            // Quaternion components
    uint8_t accuracy;            // Sensor accuracy (0-3)
    uint64_t timestamp_us;       // Microsecond timestamp
    bool valid;                  // Data validity flag
};
```

### Comprehensive IMU Data Structure
```cpp
struct Bno08xImuData {
    Bno08xVector3 acceleration;        // Calibrated acceleration (m/s²)
    Bno08xVector3 gyroscope;           // Angular velocity (rad/s)
    Bno08xVector3 magnetometer;        // Magnetic field (µT)
    Bno08xVector3 linear_acceleration; // Linear acceleration (no gravity)
    Bno08xVector3 gravity;             // Gravity vector
    Bno08xQuaternion rotation;         // Orientation quaternion
    Bno08xEulerAngles euler;           // Derived Euler angles
    uint64_t timestamp_us;             // Overall timestamp
    bool valid;                        // Overall validity
};
```

### Activity Detection Data
```cpp
struct Bno08xActivityData {
    bool tap_detected;          // Single/double tap detection
    bool double_tap;            // Double tap flag
    uint8_t tap_direction;      // Tap direction (0-5)
    uint32_t step_count;        // Total step count
    bool step_detected;         // New step flag
    bool shake_detected;        // Shake gesture
    bool pickup_detected;       // Pickup gesture
    bool significant_motion;    // Significant motion
    uint8_t stability_class;    // Stability classification
    uint8_t activity_class;     // Activity classification
    uint64_t timestamp_us;      // Timestamp
    bool valid;                 // Data validity
};
```

## 🔧 Configuration Management

### Hierarchical Configuration System
```cpp
struct Bno08xConfig {
    // Core IMU sensors
    bool enable_accelerometer;
    bool enable_gyroscope;
    bool enable_magnetometer;
    bool enable_rotation_vector;
    bool enable_linear_acceleration;
    bool enable_gravity;
    bool enable_game_rotation;
    
    // Activity detection
    bool enable_tap_detector;
    bool enable_step_counter;
    bool enable_shake_detector;
    bool enable_pickup_detector;
    bool enable_significant_motion;
    bool enable_activity_classifier;
    
    // Sensor intervals (milliseconds)
    uint32_t accelerometer_interval_ms;
    uint32_t gyroscope_interval_ms;
    uint32_t magnetometer_interval_ms;
    uint32_t rotation_interval_ms;
    uint32_t linear_accel_interval_ms;
    uint32_t gravity_interval_ms;
    
    // Interface and calibration
    BNO085Interface interface_type;
    bool auto_calibration;
    float calibration_timeout_s;
};
```

### Configuration Presets

#### Basic IMU Configuration
- Core sensors: Accelerometer, Gyroscope, Magnetometer
- Rotation vector for orientation
- 50 Hz update rates for responsive performance
- Automatic calibration enabled

#### Advanced Activity Configuration
- All IMU sensors enabled
- Full activity detection suite
- Gesture recognition capabilities
- Game rotation vector for magnetometer-free operation

#### Low Power Configuration
- Accelerometer and gravity only
- Essential activity detection
- Reduced update rates (10-20 Hz)
- Disabled auto-calibration

## 🚀 Lazy Initialization Pattern

### Construction Phase
```cpp
Bno08xHandler::Bno08xHandler(BaseI2c& i2c_interface, const Bno08xConfig& config)
    : transport_adapter_(std::make_unique<Bno08xI2cAdapter>(i2c_interface))
    , bno08x_sensor_(nullptr)  // Lazy initialization
    , config_(config)
    , initialized_(false) {
    // Minimal construction - no hardware access
}
```

### Initialization Phase
```cpp
Bno08xError Bno08xHandler::Initialize() noexcept {
    // Thread-safe lazy initialization
    RtosMutex::LockGuard lock(handler_mutex_);
    
    if (initialized_) return Bno08xError::SUCCESS;
    
    // Open transport interface
    transport_adapter_->open();
    
    // Create sensor instance with transport
    bno08x_sensor_ = std::make_shared<BNO085>(*transport_adapter_);
    
    // Initialize sensor and apply configuration
    bno08x_sensor_->initialize();
    ApplyConfiguration(config_);
    
    initialized_ = true;
    return Bno08xError::SUCCESS;
}
```

## 🔒 Thread Safety Architecture

### Mutex Protection Strategy
```cpp
class Bno08xHandler {
private:
    mutable RtosMutex handler_mutex_;  // Thread safety protection
    
public:
    Bno08xError ReadImuData(Bno08xImuData& imu_data) noexcept {
        RtosMutex::LockGuard lock(handler_mutex_);
        if (!lock.IsLocked()) {
            return Bno08xError::MUTEX_LOCK_FAILED;
        }
        
        // Protected sensor operations
        return ReadAllSensorData(imu_data);
    }
};
```

### RAII Lock Management
- Automatic lock acquisition/release
- Exception-safe operations
- Lock failure detection and reporting
- Consistent locking across all public methods

## 🎯 Sensor Management

### BNO08x Sensor Types Support
The handler supports all 40+ BNO08x sensor types:

#### Primary IMU Sensors
- `ACCELEROMETER` - Calibrated 3-axis acceleration
- `GYROSCOPE_CALIBRATED` - Calibrated angular velocity
- `MAGNETIC_FIELD_CALIBRATED` - Calibrated magnetic field
- `ROTATION_VECTOR` - 9-DOF sensor fusion quaternion
- `LINEAR_ACCELERATION` - Acceleration without gravity
- `GRAVITY` - Gravity vector estimation
- `GAME_ROTATION_VECTOR` - 6-DOF quaternion (no magnetometer)

#### Activity Detection
- `TAP_DETECTOR` - Single/double tap detection
- `STEP_COUNTER` - Step counting and detection
- `SHAKE_DETECTOR` - Shake gesture recognition
- `PICKUP_DETECTOR` - Pickup gesture detection
- `SIGNIFICANT_MOTION` - Significant motion detection
- `ACTIVITY_CLASSIFIER` - Activity classification

#### Advanced Features
- `STABILITY_CLASSIFIER` - Stability detection
- `PERSONAL_ACTIVITY_CLASSIFIER` - Personal activity learning
- `ARVR_STABILIZED_RV` - AR/VR optimized rotation vector
- `ARVR_STABILIZED_GRV` - AR/VR optimized game rotation

### Dynamic Sensor Control
```cpp
// Enable sensor with specific interval
handler->EnableSensor(BNO085Sensor::ACCELEROMETER, 50); // 50ms = 20Hz

// Disable sensor
handler->DisableSensor(BNO085Sensor::MAGNETOMETER);

// Change sensor rate
handler->EnableSensor(BNO085Sensor::GYROSCOPE_CALIBRATED, 20); // 50Hz
```

## 📡 Communication Interface Support

### Multi-Interface Architecture
The handler supports three communication interfaces through unified API:

#### I2C Interface
- **Address**: 0x4A (default)
- **Speed**: Up to 400 kHz
- **Features**: Standard I2C protocol with interrupt support
- **GPIO Requirements**: Optional reset and interrupt pins

#### SPI Interface  
- **Mode**: Mode 3 (CPOL=1, CPHA=1)
- **Speed**: Up to 3 MHz
- **Features**: Full-duplex communication
- **GPIO Requirements**: Wake, reset, and interrupt pins

#### UART Interface (Future)
- **Baud Rate**: Configurable (typically 115200)
- **Features**: Serial communication with flow control
- **GPIO Requirements**: Reset and interrupt pins

### Interface Selection
```cpp
// Factory methods for different interfaces
auto i2c_handler = CreateBno08xHandlerI2c(i2c_interface, config);
auto spi_handler = CreateBno08xHandlerSpi(spi_interface, config);
```

## 🎮 Hardware Control

### GPIO Pin Management
```cpp
// Hardware reset sequence
Bno08xError HardwareReset(uint32_t reset_duration_ms = 10);

// Wake pin control (SPI only)
Bno08xError SetWakePin(bool wake_state);

// Boot pin control for DFU mode
Bno08xError SetBootPin(bool boot_state);
```

### Reset Sequence Protocol
1. Assert reset pin LOW
2. Hold for specified duration (default 10ms)
3. Release reset pin HIGH
4. Wait for sensor restart (100ms)
5. Re-initialize sensor configuration

### Interface Pin Configuration

#### I2C Mode Pins
- **Reset**: Hardware reset control
- **INT**: Interrupt notification (active low)
- **SDA/SCL**: I2C data/clock lines

#### SPI Mode Pins
- **Reset**: Hardware reset control
- **INT**: Interrupt notification (active low)
- **Wake**: Interface selection (HIGH for SPI)
- **MOSI/MISO/SCK/CS**: SPI communication lines

## 🔧 Calibration Management

### Calibration Architecture
```cpp
struct Bno08xCalibrationStatus {
    uint8_t accelerometer_accuracy;    // 0-3 accuracy level
    uint8_t gyroscope_accuracy;        // 0-3 accuracy level
    uint8_t magnetometer_accuracy;     // 0-3 accuracy level
    uint8_t system_accuracy;           // Overall system accuracy
    bool calibration_complete;         // All sensors calibrated
    uint32_t calibration_time_s;       // Time spent calibrating
};
```

### Calibration Process
1. **Automatic Calibration**: Continuous background calibration
2. **Manual Calibration**: User-initiated calibration sequences
3. **Calibration Saving**: Persistent storage to sensor memory
4. **Accuracy Monitoring**: Real-time accuracy reporting

### Calibration Workflow
```cpp
// Start calibration
handler->StartCalibration();

// Monitor progress
Bno08xCalibrationStatus status;
handler->ReadCalibrationStatus(status);

// Save when complete
if (status.calibration_complete) {
    handler->SaveCalibration();
}
```

## 🎯 Callback Event System

### Event-Driven Architecture
```cpp
// Sensor event callback type
using SensorCallback = std::function<void(const SensorEvent&)>;

// Set callback for real-time events
handler->SetSensorCallback([](const SensorEvent& event) {
    // Process sensor events immediately
    switch (event.sensorType) {
        case BNO085Sensor::TAP_DETECTOR:
            HandleTapEvent(event);
            break;
        case BNO085Sensor::STEP_COUNTER:
            HandleStepEvent(event);
            break;
        // ... handle other events
    }
});
```

### Event Processing
- **Real-time notifications** for activity detection
- **High-frequency data** for continuous sensors
- **Timestamp synchronization** across all events
- **Error handling** for invalid or corrupted events

## 📈 Performance Optimization

### Memory Management
- **Shared pointer usage** for safe cross-component sharing
- **Stack-based structures** for data exchange
- **Minimal heap allocation** during runtime
- **RAII principles** for automatic cleanup

### Timing Optimization
- **Lazy initialization** reduces startup time
- **Efficient polling** with interrupt-driven data availability
- **Configurable update rates** for power optimization
- **Batch data reading** for multiple sensors

### Error Handling Strategy
```cpp
enum class Bno08xError : uint8_t {
    SUCCESS = 0,
    NOT_INITIALIZED,
    INITIALIZATION_FAILED,
    INVALID_PARAMETER,
    COMMUNICATION_FAILED,
    SENSOR_NOT_RESPONDING,
    CALIBRATION_FAILED,
    TIMEOUT,
    MUTEX_LOCK_FAILED,
    // ... comprehensive error codes
};
```

## 🔄 Integration with HardFOC Ecosystem

### CommChannelsManager Integration
```cpp
// Seamless integration with communication manager
auto& comm_manager = CommChannelsManager::GetInstance();
auto i2c_interface = comm_manager.GetI2c1();
auto reset_gpio = comm_manager.GetGpio(RESET_PIN);

// Create handler with manager resources
auto handler = CreateBno08xHandlerI2c(*i2c_interface, config, reset_gpio.get());
```

### Motor Control Integration
- **Orientation feedback** for motor control algorithms
- **Vibration monitoring** for motor health diagnostics  
- **Motion detection** for safety systems
- **Tilt compensation** for orientation-sensitive applications

### System Architecture Position
```
Application Layer
├── Motor Control Systems
├── Safety Systems
└── User Interface

Handler Layer
├── TMC9660Handler (Motor Driver)
├── AS5047UHandler (Position Sensor)
├── Bno08xHandler (IMU Sensor) ← Orientation/Motion
└── PCAL95555Handler (GPIO Expansion)

Base Interface Layer
├── BaseI2c / BaseSpi / BaseUart
├── BaseGpio
└── BaseTimer

Hardware Abstraction Layer
└── Platform-specific implementations
```

## 🚀 Usage Examples

### Basic IMU Reading
```cpp
// Initialize handler
auto handler = CreateBno08xHandlerI2c(i2c_interface, config);
handler->Initialize();

// Read complete IMU data
Bno08xImuData imu_data;
if (handler->ReadImuData(imu_data) == Bno08xError::SUCCESS) {
    // Process acceleration, gyroscope, magnetometer
    ProcessOrientation(imu_data.euler);
    ProcessMotion(imu_data.linear_acceleration);
}
```

### Activity Detection
```cpp
// Enable activity detection
handler->EnableSensor(BNO085Sensor::TAP_DETECTOR, 0);
handler->EnableSensor(BNO085Sensor::STEP_COUNTER, 0);

// Read activity data
Bno08xActivityData activity;
if (handler->ReadActivityData(activity) == Bno08xError::SUCCESS) {
    if (activity.tap_detected) {
        HandleTapEvent(activity.double_tap, activity.tap_direction);
    }
    if (activity.step_detected) {
        UpdateStepCount(activity.step_count);
    }
}
```

### Dynamic Configuration
```cpp
// Runtime sensor reconfiguration
handler->EnableSensor(BNO085Sensor::GYROSCOPE_CALIBRATED, 20);  // 50Hz
handler->DisableSensor(BNO085Sensor::MAGNETOMETER);

// Update configuration
Bno08xConfig new_config = CreateLowPowerConfig();
handler->UpdateConfiguration(new_config);
```

## 🏆 Architectural Excellence

The `Bno08xHandler` demonstrates the same architectural excellence as other HardFOC handlers:

### ✅ Design Pattern Implementation
- **Bridge Pattern**: Seamless interface adaptation
- **Lazy Initialization**: Optimal resource management
- **RAII**: Automatic resource cleanup
- **Factory Methods**: Flexible object creation
- **Observer Pattern**: Event-driven callbacks

### ✅ Embedded Best Practices
- **Exception-free design**: Complete `noexcept` implementation
- **Thread safety**: Comprehensive mutex protection
- **Memory efficiency**: Minimal heap allocation
- **Real-time capability**: Predictable execution timing
- **Error handling**: Comprehensive error reporting

### ✅ Code Quality Standards
- **Type safety**: Strong typing throughout
- **Const correctness**: Proper const usage
- **Documentation**: Comprehensive inline documentation
- **Testing**: Extensive example coverage
- **Maintainability**: Clear separation of concerns

This architecture establishes the `Bno08xHandler` as the definitive reference implementation for IMU sensor management within embedded systems, maintaining the highest standards of engineering excellence established by the HardFOC component handler ecosystem.
