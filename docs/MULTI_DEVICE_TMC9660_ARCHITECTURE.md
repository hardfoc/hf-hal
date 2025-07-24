# Multi-Device TMC9660 MotorController Architecture

## Overview

The MotorController has been redesigned to support multiple TMC9660 devices with array-based access. This enables applications to manage multiple motor controllers on the same board efficiently and safely.

## Key Features

### 1. **Multi-Device Support**
- Support for up to 8 TMC9660 devices (configurable via `MAX_TMC9660_DEVICES`)
- Array-based indexing (0-based) for device access
- Dynamic device registration at runtime
- Thread-safe device management

### 2. **Backward Compatibility**
- Default device index of 0 maintains single-device compatibility
- Existing code continues to work without modification
- Gradual migration path for multi-device adoption

### 3. **Device Management**
- `RegisterDevice()`: Add new TMC9660Handler instances
- `GetDeviceCount()`: Query number of registered devices
- `IsDeviceValid()`: Validate device indices before access
- `GetInitializationStatus()`: Check per-device initialization status

### 4. **Array-Based Access Pattern**

```cpp
// Single device (backward compatible)
auto& gpio17 = motorController.gpio(17);           // Device 0 (default)
auto& adc = motorController.adc();                 // Device 0 (default)
auto driver = motorController.driver();            // Device 0 (default)

// Multi-device explicit indexing
auto& gpio17_dev0 = motorController.gpio(17, 0);   // Device 0, GPIO 17
auto& gpio18_dev1 = motorController.gpio(18, 1);   // Device 1, GPIO 18
auto& adc_dev2 = motorController.adc(2);           // Device 2 ADC
auto driver1 = motorController.driver(1);          // Device 1 driver
```

## Usage Examples

### Device Registration

```cpp
auto& motorController = MotorController::GetInstance();

// Create communication interfaces
static ExampleSpi spi1, spi2;
static ExampleUart uart1;

// Register devices
auto device0 = std::make_unique<Tmc9660Handler>(spi1, 0x01);
uint8_t idx0 = motorController.RegisterDevice(std::move(device0));

auto device1 = std::make_unique<Tmc9660Handler>(uart1, 0x02);
uint8_t idx1 = motorController.RegisterDevice(std::move(device1));

auto device2 = std::make_unique<Tmc9660Handler>(spi2, 0x03);
uint8_t idx2 = motorController.RegisterDevice(std::move(device2));
```

### Initialization and Error Handling

```cpp
// Initialize all devices
bool allSuccess = motorController.Initialize();

// Or get detailed status
auto initResults = motorController.InitializeAllDevices();
for (size_t i = 0; i < initResults.size(); ++i) {
    if (!initResults[i]) {
        std::cout << "Device " << i << " failed to initialize" << std::endl;
    }
}
```

### Safe Device Access

```cpp
// Always check device validity
if (motorController.IsDeviceValid(deviceIndex)) {
    try {
        auto& handler = motorController.handler(deviceIndex);
        auto& gpio17 = motorController.gpio(17, deviceIndex);
        // Use devices safely
    } catch (const std::exception& e) {
        std::cout << "Device access error: " << e.what() << std::endl;
    }
}
```

## Thread Safety

- All device access methods are thread-safe
- Device registration is protected by mutex
- Individual TMC9660Handler operations depend on their implementations
- Safe for multi-threaded applications

## Error Handling

### Exception-Based Error Reporting
- `std::out_of_range`: Invalid device index
- `std::runtime_error`: Device not initialized or driver unavailable

### Return Value Validation
- `RegisterDevice()`: Returns `MAX_TMC9660_DEVICES` on failure
- `IsDeviceValid()`: Check before device access
- `GetDeviceCount()`: Verify devices are registered

## Migration Guide

### From Single Device
```cpp
// Old code (still works)
auto& motorController = MotorController::GetInstance();
auto& gpio17 = motorController.gpio(17);

// New equivalent
auto& motorController = MotorController::GetInstance();
auto& gpio17 = motorController.gpio(17, 0);  // Explicit device 0
```

### To Multi-Device
```cpp
// Register multiple devices
auto device0 = std::make_unique<Tmc9660Handler>(spi1, 0x01);
auto device1 = std::make_unique<Tmc9660Handler>(spi2, 0x02);

uint8_t idx0 = motorController.RegisterDevice(std::move(device0));
uint8_t idx1 = motorController.RegisterDevice(std::move(device1));

// Initialize all
motorController.Initialize();

// Access specific devices
auto& motor0_gpio17 = motorController.gpio(17, idx0);
auto& motor1_gpio18 = motorController.gpio(18, idx1);
```

## Configuration

### Device Limits
```cpp
static constexpr uint8_t MAX_TMC9660_DEVICES = 8;  // Configurable
```

### Device Array Structure
```cpp
std::array<std::unique_ptr<Tmc9660Handler>, MAX_TMC9660_DEVICES> tmcHandlers_;
std::array<bool, MAX_TMC9660_DEVICES> deviceInitialized_;
```

## Benefits

1. **Scalability**: Support multiple motors on complex systems
2. **Isolation**: Individual device failures don't affect others  
3. **Flexibility**: Mix SPI and UART devices as needed
4. **Safety**: Thread-safe access with proper error handling
5. **Performance**: Direct array indexing for fast device access
6. **Maintainability**: Clear separation of device responsibilities

## Best Practices

1. **Register all devices before calling Initialize()**
2. **Always check IsDeviceValid() before device access**
3. **Use try-catch blocks for device operations**
4. **Store device indices returned by RegisterDevice()**
5. **Check initialization status after Initialize()**
6. **Use consistent device indexing throughout application**

## Architecture Benefits

This multi-device architecture provides a robust foundation for:
- Complex motor control systems
- Multi-axis applications
- Redundant motor configurations
- Mixed communication interface setups
- Scalable embedded motor control platforms

The design maintains simplicity for single-device use while providing powerful multi-device capabilities for advanced applications.
