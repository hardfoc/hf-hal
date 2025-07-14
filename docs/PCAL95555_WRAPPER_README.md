# PCAL95555 GPIO Wrapper Documentation

## Overview

The PCAL95555 GPIO wrapper provides a modern, platform-agnostic interface for the PCAL95555 I2C GPIO expander chip within the HardFOC HAL system. This wrapper implements the `BaseGpio` interface and integrates seamlessly with the ESP32 I2C system and platform mapping configuration.

## Architecture

### Design Principles

1. **Platform Abstraction**: The wrapper uses `BaseI2c` interface, making it platform-agnostic
2. **Interface Compliance**: Full implementation of the `BaseGpio` interface
3. **Thread Safety**: All operations are thread-safe with proper mutex protection
4. **Error Handling**: Comprehensive error handling with detailed diagnostics
5. **Lazy Initialization**: Pins are initialized only when needed
6. **Memory Management**: RAII-compliant with proper cleanup

### Class Structure

```
Pcal95555GpioWrapper
├── Pcal95555I2cAdapter (implements PCAL95555::i2cBus)
├── PCAL95555 driver instance
├── Pin mapping system
└── Diagnostics and error handling
```

## Key Features

### 1. Platform-Agnostic I2C Communication

The wrapper uses the `BaseI2c` interface, allowing it to work with any I2C implementation:

```cpp
// Works with ESP32 I2C
auto esp32_i2c = std::make_shared<EspI2c>();
auto wrapper = CreatePcal95555GpioWrapper(*esp32_i2c);

// Works with any other BaseI2c implementation
auto other_i2c = std::make_shared<SomeOtherI2c>();
auto wrapper = CreatePcal95555GpioWrapper(*other_i2c);
```

### 2. Functional Pin Mapping

Pins can be created using functional pin identifiers that are platform-agnostic:

```cpp
// Create pins using functional identifiers
auto motor_enable = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
auto led_status = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::LED_STATUS_OK);
```

### 3. Direct Chip Pin Access

For advanced users, direct chip pin access is also available:

```cpp
// Create pins using direct chip pin numbers
auto pin_0 = wrapper->CreateGpioPin(Pcal95555Chip1Pin::MOTOR_ENABLE_1);
auto pin_1 = wrapper->CreateGpioPin(Pcal95555Chip1Pin::MOTOR_BRAKE_1);
```

### 4. Full BaseGpio Interface Support

All `BaseGpio` interface methods are implemented:

- Pin direction control
- Active/inactive state management
- Toggle operations
- Pull-up/pull-down configuration
- Output mode configuration (push-pull/open-drain)
- Interrupt support
- Pin information and diagnostics

## Usage Examples

### Basic Usage

```cpp
#include "component-handler/Pcal95555GpioWrapper.h"
#include "mcu/esp32/EspI2c.h"

// Initialize I2C bus
auto esp32_i2c = std::make_shared<EspI2c>();
esp32_i2c->Initialize();

// Create wrapper
auto wrapper = CreatePcal95555GpioWrapper(*esp32_i2c);
if (!wrapper || !wrapper->IsHealthy()) {
    // Handle error
    return;
}

// Create GPIO pins
auto motor_enable = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
auto led_status = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::LED_STATUS_OK);

// Configure and use pins
motor_enable->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
motor_enable->SetActive();  // Enable motor
led_status->Toggle();       // Toggle LED
```

### Advanced Configuration

```cpp
// Configure pin with specific settings
auto fault_pin = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_FAULT_STATUS);

// Set as input with pull-up
fault_pin->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
fault_pin->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);

// Read fault status
bool fault_active = false;
hf_gpio_err_t result = fault_pin->IsActive(fault_active);
if (result == hf_gpio_err_t::GPIO_SUCCESS) {
    console_info(TAG, "Fault status: %s", fault_active ? "FAULT" : "OK");
}
```

### Diagnostics and Monitoring

```cpp
// Get wrapper diagnostics
Pcal95555GpioWrapper::Diagnostics diagnostics;
if (wrapper->GetDiagnostics(diagnostics)) {
    console_info(TAG, "Chip initialized: %s", diagnostics.chip_initialized ? "Yes" : "No");
    console_info(TAG, "Chip responsive: %s", diagnostics.chip_responsive ? "Yes" : "No");
    console_info(TAG, "Error flags: 0x%04X", diagnostics.error_flags);
    console_info(TAG, "Input Port 0: 0x%02X", diagnostics.input_port_0);
    console_info(TAG, "Input Port 1: 0x%02X", diagnostics.input_port_1);
}

// Get pin information
auto pin = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
console_info(TAG, "Functional Pin: %d", static_cast<int>(pin->GetFunctionalPin()));
console_info(TAG, "Chip Pin: %d", static_cast<int>(pin->GetChipPin()));
console_info(TAG, "Direction: %s", BaseGpio::ToString(pin->GetDirection()));
console_info(TAG, "Description: %s", pin->GetDescription());
```

## Configuration

### I2C Address Configuration

The wrapper supports configurable I2C addresses:

```cpp
// Default address (0x20)
auto wrapper1 = CreatePcal95555GpioWrapper(*i2c_bus);

// Custom address
auto wrapper2 = CreatePcal95555GpioWrapper(*i2c_bus, 0x21);
```

### Platform Mapping Integration

The wrapper integrates with the HardFOC platform mapping system:

```cpp
// Pins are automatically mapped based on platform configuration
auto motor_enable = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
// The actual chip pin is determined by the platform mapping configuration
```

## Error Handling

### Error Codes

The wrapper uses the standard HardFOC GPIO error codes:

- `GPIO_SUCCESS`: Operation completed successfully
- `GPIO_ERR_NOT_INITIALIZED`: Pin not initialized
- `GPIO_ERR_INVALID_PARAMETER`: Invalid parameter provided
- `GPIO_ERR_COMMUNICATION_FAILURE`: I2C communication failed
- `GPIO_ERR_DEVICE_NOT_FOUND`: PCAL95555 chip not responding
- `GPIO_ERR_UNSUPPORTED_OPERATION`: Operation not supported

### Error Recovery

```cpp
// Check if wrapper is healthy
if (!wrapper->IsHealthy()) {
    console_error(TAG, "PCAL95555 chip is not responding");
    
    // Try to reinitialize
    wrapper->Deinitialize();
    if (!wrapper->Initialize()) {
        console_error(TAG, "Failed to reinitialize PCAL95555");
        return;
    }
}

// Handle pin operation errors
hf_gpio_err_t result = pin->SetActive();
if (result != hf_gpio_err_t::GPIO_SUCCESS) {
    console_error(TAG, "Failed to set pin active: %s", HfGpioErrToString(result));
    
    // Check if it's a communication error
    if (result == hf_gpio_err_t::GPIO_ERR_COMMUNICATION_FAILURE) {
        // Handle communication failure
    }
}
```

## Thread Safety

### Concurrent Access

The wrapper is designed for concurrent access:

```cpp
// Multiple threads can safely access the same wrapper
std::thread thread1([&wrapper]() {
    auto pin = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
    pin->SetActive();
});

std::thread thread2([&wrapper]() {
    auto pin = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::LED_STATUS_OK);
    pin->Toggle();
});

thread1.join();
thread2.join();
```

### I2C Bus Protection

The I2C adapter includes thread-safe communication:

```cpp
// All I2C operations are protected by mutex
// No additional synchronization needed
```

## Performance Considerations

### Initialization Overhead

- **Wrapper Creation**: ~1ms (I2C communication test)
- **Pin Creation**: ~0.1ms (no I2C communication)
- **Pin Initialization**: ~2ms (I2C configuration)

### Operation Latency

- **Read Operations**: ~1-2ms (I2C read)
- **Write Operations**: ~1-2ms (I2C write)
- **Toggle Operations**: ~2-3ms (read + write)

### Memory Usage

- **Wrapper Instance**: ~1KB
- **Pin Instance**: ~100 bytes
- **I2C Adapter**: ~200 bytes

## Integration with HardFOC System

### GPIO Manager Integration

The wrapper integrates with the HardFOC GPIO manager:

```cpp
// The GPIO manager can automatically create PCAL95555 pins
// based on platform configuration
GpioManager& gpio_manager = GpioManager::GetInstance();
auto pin = gpio_manager.CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
```

### Platform Mapping

Pins are automatically mapped based on the platform configuration:

```cpp
// Platform mapping determines which PCAL95555 chip and pin
// corresponds to each functional pin identifier
```

## Migration from Old Implementation

### Key Changes

1. **Interface**: Now implements `BaseGpio` instead of custom interfaces
2. **Abstraction**: Uses `BaseI2c` instead of `EspI2c` directly
3. **Thread Safety**: Built-in thread safety with mutex protection
4. **Error Handling**: Standardized error codes and diagnostics
5. **Platform Mapping**: Integration with platform mapping system

### Migration Guide

#### Old Code
```cpp
// Old implementation
Pcal95555Gpio gpio(esp32_i2c, 0x20);
gpio.Initialize();
gpio.SetPinDirection(0, GPIO_DIRECTION_OUTPUT);
gpio.SetPinState(0, true);
```

#### New Code
```cpp
// New implementation
auto wrapper = CreatePcal95555GpioWrapper(*esp32_i2c, 0x20);
auto pin = wrapper->CreateGpioPin(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
pin->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
pin->SetActive();
```

## Troubleshooting

### Common Issues

1. **Chip Not Responding**
   - Check I2C address configuration
   - Verify I2C bus initialization
   - Check power supply to PCAL95555

2. **Communication Errors**
   - Check I2C bus speed (should be ≤ 400kHz)
   - Verify pull-up resistors on SDA/SCL
   - Check for bus conflicts

3. **Pin Configuration Issues**
   - Verify platform mapping configuration
   - Check pin direction settings
   - Ensure proper initialization sequence

### Debug Information

Enable debug logging to get detailed information:

```cpp
// Debug information is available through console logging
// Check console output for detailed error messages
```

## Future Enhancements

### Planned Features

1. **Interrupt Support**: Full interrupt handling and callback system
2. **Power Management**: Sleep/wake functionality
3. **Advanced Diagnostics**: Temperature monitoring, voltage monitoring
4. **Multi-Chip Support**: Support for multiple PCAL95555 chips
5. **Configuration Persistence**: Save/restore pin configurations

### Extension Points

The wrapper is designed for easy extension:

```cpp
// Custom I2C adapter can be implemented
class CustomI2cAdapter : public PCAL95555::i2cBus {
    // Implement custom I2C communication
};

// Custom pin mapping can be added
// Extend platform mapping system for new pin types
```

## API Reference

### Pcal95555GpioWrapper

#### Constructor
```cpp
explicit Pcal95555GpioWrapper(BaseI2c& i2c_bus, uint8_t i2c_address = 0x20) noexcept;
```

#### Factory Function
```cpp
std::shared_ptr<Pcal95555GpioWrapper> CreatePcal95555GpioWrapper(
    BaseI2c& i2c_bus, uint8_t i2c_address = 0x20) noexcept;
```

#### Core Methods
```cpp
bool Initialize() noexcept;
bool Deinitialize() noexcept;
bool IsHealthy() const noexcept;
bool GetDiagnostics(Diagnostics& diagnostics) const noexcept;
```

#### Pin Creation
```cpp
std::shared_ptr<BaseGpio> CreateGpioPin(HardFOC::FunctionalGpioPin functional_pin) noexcept;
std::shared_ptr<BaseGpio> CreateGpioPin(Pcal95555Chip1Pin chip_pin) noexcept;
```

### Pcal95555I2cAdapter

#### Constructor
```cpp
explicit Pcal95555I2cAdapter(BaseI2c& i2c_bus, uint8_t i2c_address) noexcept;
```

#### Interface Methods
```cpp
bool write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) override;
bool read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) override;
```

## License

This wrapper is part of the HardFOC project and is licensed under the same terms as the main project.

## Contributing

When contributing to the PCAL95555 wrapper:

1. Follow the existing code style and patterns
2. Add comprehensive error handling
3. Include unit tests for new features
4. Update documentation for API changes
5. Ensure thread safety for all operations
6. Maintain platform abstraction principles 