# GPIO Manager System Documentation

## Overview

The new GpioManager system provides a comprehensive, thread-safe GPIO management solution for the HardFOC HAL project. It supports multi-chip GPIO management across ESP32, PCAL95555 I2C GPIO expander, and TMC9660 motor controller chips, with full platform mapping integration and advanced features.

## Key Features

### 🚀 Multi-Chip Support
- **ESP32 Native GPIO**: Direct control of ESP32 GPIO pins
- **PCAL95555 I2C Expander**: 16 additional GPIO pins via I2C
- **TMC9660 Motor Controller**: GPIO pins integrated with motor control
- **Extensible Architecture**: Easy to add new GPIO chip types

### 🔧 Platform Mapping Integration
- **Functional Pin Identifiers**: Use descriptive pin names instead of hardware numbers
- **Hardware Resource Discovery**: Automatic mapping to physical hardware
- **Configuration Validation**: Ensures pin configurations are valid for target hardware

### 🛡️ Thread Safety & Error Handling
- **Mutex Protection**: All operations are thread-safe
- **Comprehensive Error Handling**: Detailed error reporting and recovery
- **Operation Statistics**: Track performance and error rates

### ⚡ Advanced Features
- **GPIO Interrupts**: Edge-triggered interrupts with callbacks
- **Batch Operations**: Execute multiple GPIO operations atomically
- **Diagnostics**: System health monitoring and troubleshooting
- **Configuration Management**: Runtime pin configuration and validation

## Architecture

### Core Components

```
GpioManager (Singleton)
├── Chip Drivers
│   ├── Esp32GpioDriver (BaseGpio implementation)
│   ├── Pcal95555GpioWrapper (BaseGpio implementation)
│   └── Tmc9660Gpio (BaseGpio implementation)
├── Platform Mapping
│   ├── Hardware Resource Discovery
│   └── Pin Configuration Validation
├── Interrupt Management
│   ├── Interrupt Registration
│   └── Callback Dispatch
└── Thread Safety
    ├── Mutex Protection
    └── Atomic Operations
```

### Data Flow

1. **Pin Registration**: Functional pin → Platform mapping → Hardware resource → Chip driver
2. **GPIO Operations**: GpioManager → Chip driver → Hardware → Response
3. **Interrupts**: Hardware → Chip driver → GpioManager → User callback

## Usage Examples

### Basic GPIO Operations

```cpp
#include "component-handler/All.h"

// Get the GPIO manager instance
GpioManager& gpio = GpioManager::GetInstance();

// Initialize the system
if (!gpio.EnsureInitialized()) {
    console_error(TAG, "Failed to initialize GPIO system");
    return;
}

// Configure a pin
GpioConfig config = {
    .direction = GpioDirection::GPIO_OUTPUT,
    .pullUpDown = GpioPullUpDown::GPIO_PULL_NONE,
    .driveStrength = GpioDriveStrength::GPIO_DRIVE_12MA
};

if (!gpio.ConfigurePin(GpioPin::GPIO_ESP32_USER_LED, config)) {
    console_error(TAG, "Failed to configure LED pin");
    return;
}

// Set pin state
if (gpio.SetActive(GpioPin::GPIO_ESP32_USER_LED)) {
    console_info(TAG, "LED turned ON");
}

// Read pin state
bool isActive = gpio.IsActive(GpioPin::GPIO_ESP32_USER_BUTTON);
console_info(TAG, "Button state: %s", isActive ? "PRESSED" : "RELEASED");
```

### Multi-Chip GPIO Management

```cpp
// ESP32 GPIO
gpio.SetActive(GpioPin::GPIO_ESP32_USER_LED);

// PCAL95555 GPIO
gpio.SetActive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
gpio.SetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_2);

// TMC9660 GPIO (via motor controller)
Tmc9660MotorController& tmc = Tmc9660MotorController::GetInstance();
tmc.SetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 17, true);
```

### Batch Operations

```cpp
// Create a batch operation
GpioBatchOperation batch;

// Add multiple operations
batch.AddSetActive(GpioPin::GPIO_ESP32_USER_LED);
batch.AddSetActive(GpioPin::GPIO_PCAL95555_OUTPUT_1);
batch.AddSetInactive(GpioPin::GPIO_PCAL95555_OUTPUT_2);

// Execute all operations atomically
if (gpio.ExecuteBatch(batch)) {
    console_info(TAG, "Batch operation completed successfully");
}
```

### GPIO Interrupts

```cpp
// Create interrupt callback
class MyInterruptHandler : public IGpioInterruptCallback {
public:
    void OnGpioInterrupt(GpioPin pin, bool active, uint64_t timestamp) override {
        console_info(TAG, "Interrupt on pin %s: %s", 
                     GpioPinToString(pin).data(),
                     active ? "ACTIVE" : "INACTIVE");
    }
};

MyInterruptHandler handler;

// Enable interrupt
if (gpio.EnableInterrupt(GpioPin::GPIO_ESP32_USER_BUTTON,
                        GpioInterruptTrigger::GPIO_INTR_NEGEDGE,
                        &handler)) {
    console_info(TAG, "Button interrupt enabled");
}
```

### TMC9660 Integration

```cpp
// Initialize TMC9660 system
Tmc9660MotorController& tmc = Tmc9660MotorController::GetInstance();

// Configure TMC9660 chip
Tmc9660Config config = {
    .chipId = Tmc9660ChipId::TMC9660_CHIP_1,
    .primaryInterface = Tmc9660CommInterface::TMC_COMM_SPI,
    .spiFrequency = 1000000
};

Tmc9660GpioConfig gpioConfig = {
    .gpio17Pin = GpioPin::GPIO_TMC_GPIO17,
    .gpio18Pin = GpioPin::GPIO_TMC_GPIO18,
    .faultStatusPin = GpioPin::GPIO_TMC_nFAULT_STATUS,
    .driverEnablePin = GpioPin::GPIO_TMC_DRV_EN
};

// Register the chip
if (tmc.RegisterTmc9660Chip(Tmc9660ChipId::TMC9660_CHIP_1, 
                           config, gpioConfig, adcConfig)) {
    console_info(TAG, "TMC9660 chip registered successfully");
}

// Control TMC9660 GPIO
tmc.SetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 17, true);
bool gpioState = tmc.GetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 17);

// Handle faults
if (tmc.HasFault(Tmc9660ChipId::TMC9660_CHIP_1)) {
    console_error(TAG, "TMC9660 fault detected");
    tmc.ClearFault(Tmc9660ChipId::TMC9660_CHIP_1);
}
```

### System Diagnostics

```cpp
// Get system status
GpioManagerStatus status;
if (gpio.GetStatus(status)) {
    console_info(TAG, "GPIO System Status:");
    console_info(TAG, "  Initialized: %s", status.isInitialized ? "Yes" : "No");
    console_info(TAG, "  Registered pins: %d", status.registeredPinCount);
    console_info(TAG, "  Active interrupts: %d", status.activeInterruptCount);
    console_info(TAG, "  Total operations: %lu", status.totalOperations);
    console_info(TAG, "  Error count: %lu", status.errorCount);
}

// Run diagnostics
GpioDiagnostics diagnostics;
if (gpio.RunDiagnostics(diagnostics)) {
    console_info(TAG, "Diagnostics Results:");
    console_info(TAG, "  System healthy: %s", diagnostics.isSystemHealthy ? "Yes" : "No");
    console_info(TAG, "  All chips responding: %s", diagnostics.allChipsResponding ? "Yes" : "No");
    
    if (!diagnostics.isSystemHealthy) {
        for (const auto& issue : diagnostics.issues) {
            console_error(TAG, "  Issue: %s", issue.c_str());
        }
    }
}
```

## Platform Mapping

### Functional Pin Identifiers

The system uses descriptive pin identifiers that map to hardware resources:

```cpp
// ESP32 pins
GpioPin::GPIO_ESP32_USER_LED
GpioPin::GPIO_ESP32_USER_BUTTON
GpioPin::GPIO_ESP32_GPIO_2
GpioPin::GPIO_ESP32_GPIO_4

// PCAL95555 pins
GpioPin::GPIO_PCAL95555_OUTPUT_1
GpioPin::GPIO_PCAL95555_OUTPUT_2
GpioPin::GPIO_PCAL95555_INPUT_1
GpioPin::GPIO_PCAL95555_INT

// TMC9660 pins
GpioPin::GPIO_TMC_GPIO17
GpioPin::GPIO_TMC_GPIO18
GpioPin::GPIO_TMC_nFAULT_STATUS
GpioPin::GPIO_TMC_DRV_EN
GpioPin::GPIO_TMC_RST_CTRL
GpioPin::GPIO_TMC_SPI_COMM_nEN
GpioPin::GPIO_TMC_nWAKE_CTRL
```

### Hardware Resource Discovery

```cpp
// Get hardware resource for a functional pin
HardwareResource hwResource;
if (gpio.GetHardwareResource(GpioPin::GPIO_ESP32_USER_LED, hwResource)) {
    console_info(TAG, "Hardware Resource:");
    console_info(TAG, "  Chip: %s", HardwareChipIdToString(hwResource.chipId).data());
    console_info(TAG, "  Pin: %d", hwResource.pinNumber);
    console_info(TAG, "  Direction: %s", GpioDirectionToString(hwResource.direction).data());
}
```

## Configuration

### Pin Configuration

```cpp
GpioConfig config = {
    .direction = GpioDirection::GPIO_OUTPUT,        // Input or Output
    .pullUpDown = GpioPullUpDown::GPIO_PULL_UP,     // Pull-up, Pull-down, or None
    .driveStrength = GpioDriveStrength::GPIO_DRIVE_12MA  // Drive strength
};
```

### Interrupt Configuration

```cpp
// Available interrupt triggers
GpioInterruptTrigger::GPIO_INTR_DISABLE      // Disable interrupt
GpioInterruptTrigger::GPIO_INTR_POSEDGE      // Rising edge
GpioInterruptTrigger::GPIO_INTR_NEGEDGE      // Falling edge
GpioInterruptTrigger::GPIO_INTR_ANYEDGE      // Both edges
GpioInterruptTrigger::GPIO_INTR_LOW_LEVEL    // Low level
GpioInterruptTrigger::GPIO_INTR_HIGH_LEVEL   // High level
```

## Error Handling

### Common Error Scenarios

1. **Pin Not Registered**: Attempting to use a pin before registration
2. **Invalid Configuration**: Configuration not supported by target hardware
3. **Communication Errors**: I2C/SPI communication failures with external chips
4. **Resource Conflicts**: Multiple registrations of the same pin
5. **Hardware Faults**: Physical hardware issues

### Error Recovery

```cpp
// Check if operation succeeded
if (!gpio.SetActive(GpioPin::GPIO_ESP32_USER_LED)) {
    console_error(TAG, "Failed to set LED active");
    
    // Get detailed error information
    GpioManagerStatus status;
    if (gpio.GetStatus(status)) {
        console_error(TAG, "Error count: %lu", status.errorCount);
    }
    
    // Run diagnostics to identify issues
    GpioDiagnostics diagnostics;
    if (gpio.RunDiagnostics(diagnostics)) {
        for (const auto& issue : diagnostics.issues) {
            console_error(TAG, "Issue: %s", issue.c_str());
        }
    }
}
```

## Performance Considerations

### Thread Safety
- All GPIO operations are protected by mutexes
- Interrupt callbacks are executed in interrupt context
- Batch operations reduce mutex contention

### Optimization Tips
1. **Use Batch Operations**: Group multiple GPIO operations together
2. **Minimize Interrupt Overhead**: Keep interrupt handlers short
3. **Cache Pin States**: Avoid repeated reads of the same pin
4. **Use Appropriate Drive Strength**: Match drive strength to load requirements

### Memory Usage
- Static allocation for chip instances
- Dynamic allocation for interrupt callbacks
- Configurable batch operation buffer sizes

## Integration with Other Systems

### ADC Manager Integration
```cpp
// TMC9660 ADC integration
uint32_t adcValue;
if (tmc.ReadAdcValue(Tmc9660ChipId::TMC9660_CHIP_1, 1, adcValue)) {
    console_info(TAG, "TMC9660 AIN1: %lu", adcValue);
}

float voltage;
if (tmc.ReadAdcVoltage(Tmc9660ChipId::TMC9660_CHIP_1, 1, voltage)) {
    console_info(TAG, "TMC9660 AIN1: %.3fV", voltage);
}
```

### RTOS Integration
```cpp
// Thread-safe GPIO operations
class GpioWorkerThread : public BaseThread {
protected:
    void Run() override {
        while (!ShouldStop()) {
            // GPIO operations are automatically thread-safe
            gpio.SetActive(GpioPin::GPIO_ESP32_USER_LED);
            os_delay_msec(1000);
            gpio.SetInactive(GpioPin::GPIO_ESP32_USER_LED);
            os_delay_msec(1000);
        }
    }
};
```

## Troubleshooting

### Common Issues

1. **PCAL95555 Not Responding**
   - Check I2C bus configuration
   - Verify device address
   - Check power supply and connections

2. **TMC9660 Communication Issues**
   - Verify SPI/UART configuration
   - Check fault status pin
   - Ensure proper power sequencing

3. **Interrupt Not Triggering**
   - Verify interrupt trigger configuration
   - Check callback registration
   - Ensure pin is configured as input

4. **Performance Issues**
   - Use batch operations for multiple pins
   - Minimize interrupt frequency
   - Check for mutex contention

### Debug Information

```cpp
// Enable debug logging
console_set_level(LOG_LEVEL_DEBUG);

// Get detailed system information
gpio.PrintSystemStatus();

// Run comprehensive diagnostics
GpioDiagnostics diagnostics;
gpio.RunDiagnostics(diagnostics);
```

## API Reference

### Core Classes

- **GpioManager**: Main GPIO management singleton
- **GpioBatchOperation**: Batch GPIO operations
- **IGpioInterruptCallback**: Interrupt callback interface
- **Tmc9660MotorController**: TMC9660 motor controller integration

### Key Methods

#### GpioManager
- `EnsureInitialized()`: Initialize the system
- `RegisterPin()`: Register a GPIO pin
- `ConfigurePin()`: Configure pin settings
- `SetActive()/SetInactive()`: Set pin state
- `IsActive()`: Read pin state
- `EnableInterrupt()`: Enable pin interrupt
- `ExecuteBatch()`: Execute batch operations
- `GetStatus()`: Get system status
- `RunDiagnostics()`: Run system diagnostics

#### Tmc9660MotorController
- `RegisterTmc9660Chip()`: Register TMC9660 chip
- `EnableDriver()/DisableDriver()`: Control motor driver
- `SetGpioState()/GetGpioState()`: Control TMC9660 GPIO
- `ReadAdcValue()/ReadAdcVoltage()`: Read ADC values
- `HasFault()/ClearFault()`: Handle faults
- `TestCommunication()`: Test chip communication

## Examples

See the following example files for complete usage demonstrations:

- `examples/AdvancedGpioManagerExample.cpp`: Comprehensive example with all features
- `examples/UnifiedGpioInterruptExample.cpp`: Interrupt handling examples
- `component-handler/ModernHalExample.cpp`: Basic usage examples

## Migration Guide

### From Old GPIO System

1. **Replace direct GPIO calls** with GpioManager calls
2. **Use functional pin identifiers** instead of hardware pin numbers
3. **Register pins** before using them
4. **Use batch operations** for multiple GPIO changes
5. **Implement interrupt callbacks** for event-driven operations

### Example Migration

```cpp
// Old way
gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
gpio_set_level(GPIO_NUM_2, 1);

// New way
GpioConfig config = {.direction = GpioDirection::GPIO_OUTPUT};
gpioManager.ConfigurePin(GpioPin::GPIO_ESP32_GPIO_2, config);
gpioManager.SetActive(GpioPin::GPIO_ESP32_GPIO_2);
```

## Future Enhancements

- **Additional Chip Support**: Support for more GPIO expander chips
- **Advanced Interrupt Features**: Debouncing, filtering, and priority levels
- **Power Management**: Dynamic power control for GPIO chips
- **Configuration Persistence**: Save/restore GPIO configurations
- **Web Interface**: Web-based GPIO monitoring and control

## Support

For questions, issues, or contributions:

1. Check the troubleshooting section above
2. Review the example code
3. Examine the API documentation
4. Run system diagnostics
5. Contact the development team

---

*This documentation covers the comprehensive GPIO management system for the HardFOC HAL project. The system provides a modern, thread-safe, and extensible solution for managing GPIO across multiple hardware chips with full platform mapping integration.* 