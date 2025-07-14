# HardFOC Component Handler

This directory contains the **core GPIO and ADC handling components** for the HardFOC system, implementing a truly hardware-agnostic architecture.

## 🎯 Architecture Overview

The component handler provides a clean, hardware-agnostic interface for all GPIO and ADC operations. The system uses **functional identifiers** (e.g., `MOTOR_ENABLE`, `MOTOR_CURRENT_PHASE_A`) that describe **WHAT** the pin/channel does, not **WHERE** it's located.

### Key Design Principles

1. **Hardware Agnostic**: Application code uses only functional identifiers
2. **Platform Portable**: Porting requires only updating configuration files
3. **Thread Safe**: All operations are protected with appropriate synchronization
4. **Multi-Source**: Supports GPIO/ADC from multiple chips (ESP32, PCAL95555, TMC9660)
5. **Performance Optimized**: Efficient batch operations and minimal overhead

## 📁 File Structure

### Core Data Management
- **`AdcData.h/.cpp`** - Comprehensive ADC data management singleton
- **`GpioData.h/.cpp`** - Multi-source GPIO data management singleton

### Handler Interfaces
- **`GpioHandler.h/.cpp`** - High-level GPIO interface with initialization
- **`SystemInit.h/.cpp`** - System-wide initialization coordinator

### Hardware Integration
- **`Tmc9660MotorController.h/.cpp`** - TMC9660 motor controller integration
- **`TMC9660Controller.h/.cpp`** - TMC9660 specific implementation
- **`Tmc9660Gpio.h/.cpp`** - TMC9660 GPIO wrapper
- **`Pcal95555GpioWrapper.h/.cpp`** - PCAL95555 GPIO expander wrapper

### Support Files
- **`CommonIDs.h`** - Legacy compatibility and convenience aliases
- **`All.h`** - Consolidated header with convenience namespace
- **`ThingsToString.h`** - String conversion utilities
- **`MultiReadings.h`** - Generic multi-reading template
- **`AdcMultiCountReading.h`** - ADC-specific multi-reading implementation

### Examples
- **`ExampleUsage.cpp`** - Comprehensive usage examples
- **`Tmc9660Example.cpp`** - TMC9660-specific examples

## 🚀 Quick Start

### Basic GPIO Control
```cpp
#include "All.h"

using namespace HardFocComponentHandler;

// Initialize the system
if (!InitializeAll()) {
    // Handle initialization error
    return;
}

// Get GPIO system
auto& gpio = GetGpioSystem();

// Turn on status LED using functional identifier
gpio.SetPinActive(FunctionalGpioPin::LED_STATUS_OK);

// Enable motor
gpio.SetPinActive(FunctionalGpioPin::MOTOR_ENABLE);
```

### Basic ADC Reading
```cpp
#include "All.h"

using namespace HardFocComponentHandler;

// Get ADC system
auto& adc = GetAdcSystem();

// Read motor current using functional identifier
float current_volts = 0.0f;
auto result = adc.GetVolt(FunctionalAdcChannel::MOTOR_CURRENT_PHASE_A, 
                          current_volts, 5, 1);

if (result == HfAdcErr::HF_ADC_ERR_NONE) {
    printf("Motor Phase A Current: %.3fV\n", current_volts);
}
```

## 🔧 Hardware Abstraction

### Functional Identifiers

The system uses functional identifiers defined in the platform mapping:

**ADC Channels:**
- `MOTOR_CURRENT_PHASE_A/B/C` - Motor phase current measurements
- `SYSTEM_VOLTAGE_3V3/5V/12V` - Power rail monitoring
- `SYSTEM_TEMPERATURE_AMBIENT` - Ambient temperature
- `USER_ANALOG_INPUT_1/2` - User expansion inputs

**GPIO Pins:**
- `MOTOR_ENABLE/BRAKE/FAULT_STATUS` - Motor control signals
- `LED_STATUS_OK/ERROR/COMM` - Status indicators  
- `USER_OUTPUT_1/2` - User expansion outputs
- `SPI_*`, `COMM_CAN_*`, `COMM_I2C_*` - Communication interfaces

### Platform Mapping

All hardware-specific details are isolated in:
`../utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp`

## 🌍 Porting to New Hardware

To port to a new microcontroller platform:

1. **Update Platform Mapping**: Modify `hf_platform_mapping.hpp` with new hardware resources
2. **Add Drivers**: Ensure appropriate drivers exist in `hf-core-drivers`
3. **Done!** All application code remains unchanged

Example mapping change:
```cpp
// Change from ESP32-C6 to STM32
[static_cast<uint8_t>(FunctionalGpioPin::LED_STATUS_OK)] = {
    .chip_id = HardwareChip::STM32_INTERNAL_GPIO,  // Changed
    .pin_id = 5,   // Changed to STM32 pin
    .is_inverted = false,
    .has_pullup = true,
    .has_pulldown = false,
    .max_current_ma = 20
}
```

## 📋 Supported Hardware

### Current Platform: ESP32-C6 + TMC9660 + PCAL95555

- **ESP32-C6**: Native GPIO pins and internal ADC units
- **PCAL95555**: 16-bit I2C GPIO expander for additional I/O
- **TMC9660**: Motor controller with integrated ADC for current sensing

### Multi-Source GPIO Support
- Thread-safe I2C communication via `SfI2cBus`
- SPI communication for TMC9660
- Native ESP32-C6 GPIO
- Automatic error handling and recovery

## 📖 Documentation

For comprehensive documentation on the GPIO and ADC data sourcing system, including:
- System architecture and design principles
- Usage examples and integration guides
- Pin mappings and hardware configurations  
- Health monitoring and error handling
- Performance considerations and troubleshooting

Please refer to: **[../docs/HARDFOC_GPIO_ADC_SYSTEM.md](../docs/HARDFOC_GPIO_ADC_SYSTEM.md)**

## 🎯 Key Benefits

1. **Hardware Independence**: Code works across different microcontrollers
2. **Clean Architecture**: Clear separation of concerns
3. **Easy Maintenance**: Hardware changes isolated to configuration files
4. **Robust Error Handling**: Comprehensive error detection and recovery
5. **Performance**: Optimized for real-time motor control applications
6. **Scalable**: Easy to add new hardware sources

## 🔍 Example Applications

- Motor current monitoring and control
- System health monitoring (voltages, temperatures)
- Status indication via LEDs
- User interface inputs/outputs
- Multi-axis motor control systems
- Industrial automation controllers

---

**Note**: This architecture represents a best-practice approach to embedded hardware abstraction, making the HardFOC system truly portable across microcontroller platforms.
- **PCAL95555**: I2C GPIO expander (up to 2 chips, 16 pins each)
- **TMC9660**: Motor controller GPIO pins and ADC channels

### Basic Usage
```cpp
#include "component-handler/All.h"

// Initialize systems
auto& gpioHandler = GpioHandler::GetInstance();
auto& i2cBus = SfI2cBus::GetInstance();
gpioHandler.Initialize(i2cBus);

// Use GPIO pins
gpioHandler.SetPin(GPIO_ESP32_PIN_2, true);
bool state = gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_0);
```

See the comprehensive documentation for detailed usage examples and integration instructions.
