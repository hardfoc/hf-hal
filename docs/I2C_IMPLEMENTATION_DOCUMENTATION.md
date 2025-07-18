# I2C Implementation and IMU Manager Redesign Documentation

## Overview

This document describes the implementation of ESP-IDF v5.5+ I2C communication for BNO08x IMU and PCAL95555 GPIO expander devices, along with the complete redesign of the IMU manager to eliminate BaseImuDriver abstraction in favor of direct device object access.

## Architecture Summary

### Key Components

1. **CommChannelsManager**: Enhanced with proper I2C bus initialization for BNO08x and PCAL95555
2. **ImuManager**: Completely redesigned for direct BNO085 device access (no BaseImuDriver)
3. **EspI2cBno085Transport**: Bridge between BNO085 driver and ESP-IDF v5.5+ I2C implementation
4. **I2cBno08xExample**: Comprehensive example demonstrating the new architecture

### Design Principles

- **Direct Device Access**: No base class abstractions - users get direct access to BNO085 objects
- **ESP-IDF v5.5+ Compliance**: Uses latest I2C master bus-device model with advanced features
- **Type Safety**: Eliminates runtime type checking through compile-time type safety
- **Resource Management**: Proper RAII and exception safety throughout the implementation
- **Performance**: Optimized I2C configuration for high-frequency sensor data

## Implementation Details

### 1. I2C Bus Configuration (CommChannelsManager)

#### Bus Configuration
```cpp
hf_i2c_master_bus_config_t i2c_config = {};
i2c_config.i2c_port = I2C_NUM_0;                                               // I2C port 0 (ESP32C6 has 1 I2C port)
i2c_config.sda_io_num = sda_map->physical_pin;                                 // SDA GPIO pin from board mapping
i2c_config.scl_io_num = scl_map->physical_pin;                                 // SCL GPIO pin from board mapping
i2c_config.enable_internal_pullup = sda_map->has_pullup && scl_map->has_pullup; // Use internal pullups if available
i2c_config.clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT;        // Default clock source (APB)
i2c_config.glitch_ignore_cnt = hf_i2c_glitch_filter_t::HF_I2C_GLITCH_FILTER_7_CYCLES; // 7-cycle glitch filter
i2c_config.trans_queue_depth = 8;                                              // Transaction queue depth for async ops
i2c_config.intr_priority = 5;                                                  // Interrupt priority (0-7, 5=medium)
```

#### Device Registration
- **BNO08x IMU**: Address 0x4A, 400kHz operation
- **PCAL95555 GPIO Expander**: Address 0x20, 400kHz operation

#### ESP-IDF v5.5+ Features Used
- `i2c_new_master_bus()`: Bus-device model for better resource management
- `i2c_master_bus_add_device()`: Device registration with individual configurations
- Digital glitch filtering: 7-cycle filter for noise immunity
- Transaction queue depth: Support for asynchronous operations
- Configurable interrupt priorities: Medium priority for balanced performance

### 2. IMU Manager Redesign

#### Previous Architecture (Eliminated)
```cpp
// OLD: BaseImuDriver abstraction with runtime type checking
BaseImuDriver* GetImuByType(ImuType type);
std::vector<std::unique_ptr<BaseImuDriver>> imus_;
```

#### New Architecture (Direct Access)
```cpp
// NEW: Direct BNO085 device access with compile-time type safety
BNO085& GetBno08x();                    // Throws if not available
BNO085* GetBno08xPtr() noexcept;        // Returns nullptr if not available
bool IsBno08xAvailable() const noexcept; // Check availability
```

#### Benefits of Redesign
1. **Type Safety**: Compile-time checking instead of runtime type casts
2. **Performance**: No virtual function overhead or dynamic_cast operations
3. **API Clarity**: Clear, specific methods for each device type
4. **Memory Efficiency**: No base class overhead or virtual tables
5. **Exception Safety**: Proper RAII and exception specifications

### 3. BNO085 I2C Transport Implementation

#### Transport Bridge Design
```cpp
class EspI2cBno085Transport : public IBNO085Transport {
public:
    explicit EspI2cBno085Transport(EspI2c& i2c_bus, uint8_t device_addr = 0x4A) noexcept;
    
    // IBNO085Transport interface implementation
    bool open() override;
    void close() override;
    int write(const uint8_t* data, uint32_t length) override;
    int read(uint8_t* data, uint32_t length) override;
    bool dataAvailable() override;
    void delay(uint32_t ms) override;
    uint32_t getTimeUs() override;
    
private:
    EspI2c& i2c_bus_;
    uint8_t device_address_;
};
```

#### Integration Features
- **Bus Sharing**: Multiple devices on same I2C bus (BNO08x + PCAL95555)
- **Error Handling**: Proper ESP-IDF error code translation
- **Timing**: ESP32 timer integration for microsecond timestamps
- **Polling Mode**: Optimized for continuous sensor data collection

### 4. Example Usage Patterns

#### Basic Initialization
```cpp
// Initialize communication channels
auto& comm_mgr = CommChannelsManager::GetInstance();
comm_mgr.EnsureInitialized();

// Initialize IMU manager
auto& imu_mgr = ImuManager::GetInstance();
imu_mgr.EnsureInitialized();

// Get direct access to BNO08x
if (imu_mgr.IsBno08xAvailable()) {
    BNO085& bno08x = imu_mgr.GetBno08x();
    // Use bno08x directly...
}
```

#### Sensor Configuration
```cpp
// Enable rotation vector at 50Hz
bno08x.enableSensor(BNO085Sensor::RotationVector, 20);

// Enable linear acceleration at 100Hz
bno08x.enableSensor(BNO085Sensor::LinearAcceleration, 10);

// Set up callback for sensor events
bno08x.setCallback([](const BNO085::SensorEvent& event) {
    // Process sensor data...
});
```

#### Data Collection Methods
```cpp
// Method 1: Callback-based (event-driven)
bno08x.setCallback(sensorEventCallback);
while (running) {
    bno08x.update(); // Process incoming data
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Method 2: Polling-based
while (running) {
    bno08x.update();
    if (bno08x.hasNewData(BNO085Sensor::RotationVector)) {
        auto event = bno08x.getLatest(BNO085Sensor::RotationVector);
        // Process event...
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

## Technical Specifications

### I2C Bus Characteristics
- **Frequency**: 400kHz (Fast I2C mode)
- **Addressing**: 7-bit device addressing
- **Pullups**: Internal or external (4.7kΩ recommended)
- **Glitch Filter**: 7-cycle digital filter for noise immunity
- **Queue Depth**: 8 transactions for asynchronous operations

### BNO08x Configuration
- **I2C Address**: 0x4A (standard)
- **Interface**: I2C mode (PS0=0, PS1=0)
- **Data Rate**: Up to 1000Hz for individual sensors
- **Protocols**: SHTP (Sensor Hub Transport Protocol) over I2C

### PCAL95555 Configuration
- **I2C Address**: 0x20 (A0=A1=A2=0, configurable)
- **GPIO Count**: 16 pins (2 x 8-bit ports)
- **Features**: Input/output, interrupt, pull-up/down configuration

### Performance Characteristics
- **Sensor Update Rate**: 10-100Hz typical
- **I2C Transaction Time**: ~100μs per transaction at 400kHz
- **Memory Usage**: Minimal overhead with direct device access
- **CPU Usage**: Low overhead with interrupt-driven I2C

## Error Handling and Diagnostics

### Error Categories
1. **Initialization Errors**: I2C bus setup, device detection
2. **Communication Errors**: I2C transaction failures, timeouts
3. **Sensor Errors**: BNO085 SH-2 protocol errors, calibration issues
4. **Resource Errors**: Memory allocation, device unavailability

### Diagnostic Features
- **ESP-IDF Logging**: Comprehensive log messages with severity levels
- **Device Status**: Real-time availability checking
- **Error Codes**: ESP-IDF and BNO085 error code propagation
- **Statistics**: I2C transaction statistics and performance monitoring

### Recovery Mechanisms
- **Automatic Retry**: I2C transaction retry with exponential backoff
- **Bus Recovery**: ESP-IDF v5.5+ automatic bus recovery
- **Device Reset**: Software and hardware reset capabilities
- **Graceful Degradation**: Continue operation with partial functionality

## Files Modified/Created

### Modified Files
1. **CommChannelsManager.h**: Updated interface for I2C access
2. **CommChannelsManager.cpp**: Complete I2C initialization with device registration
3. **ImuManager.h**: Complete redesign for direct BNO085 access
4. **ImuManager.cpp**: New implementation with I2C transport integration

### Created Files
1. **I2cBno08xExample.cpp**: Comprehensive usage example
2. **I2C_IMPLEMENTATION_DOCUMENTATION.md**: This documentation file

## Integration with Existing Components

### EspI2c Integration
- Uses existing ESP-IDF v5.5+ wrapper for I2C operations
- Leverages bus-device model for proper resource management
- Maintains compatibility with other I2C devices

### Board Configuration Integration
- Uses existing HfFunctionalGpioPin mapping system
- Automatic GPIO configuration based on board definitions
- Support for internal/external pullup configuration

### Error System Integration
- Uses existing hf_i2c_err_t error codes
- Integrates with ESP-IDF logging system
- Proper exception handling throughout the stack

## Future Enhancements

### Planned Features
1. **Multi-Device Support**: Framework for additional IMU types
2. **Advanced Calibration**: Automatic sensor calibration routines
3. **Data Fusion**: Sensor fusion algorithms for enhanced accuracy
4. **Power Management**: Sleep/wake control for battery operation
5. **Interrupt Support**: GPIO interrupt-driven data collection

### Scalability Considerations
- **Additional I2C Devices**: Framework supports easy addition of new devices
- **Multiple I2C Buses**: Support for ESP32 variants with multiple I2C controllers
- **DMA Integration**: Future support for DMA-based I2C transfers
- **Real-time Constraints**: Integration with FreeRTOS real-time features

## Conclusion

The I2C implementation and IMU manager redesign provides a solid foundation for sensor communication in the HardFOC system. The direct device access pattern eliminates abstraction overhead while maintaining type safety and performance. The ESP-IDF v5.5+ integration ensures compatibility with the latest ESP32 features and provides a path for future enhancements.

Key achievements:
- ✅ ESP-IDF v5.5+ I2C bus-device model implementation
- ✅ BNO08x and PCAL95555 device registration and configuration
- ✅ Complete elimination of BaseImuDriver abstraction
- ✅ Direct BNO085 device object access with type safety
- ✅ Comprehensive error handling and diagnostics
- ✅ Production-ready example code and documentation

The implementation is ready for integration into the broader HardFOC system and provides a solid foundation for advanced sensor-based applications.
