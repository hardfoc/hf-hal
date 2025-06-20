# TMC9660 Motor Controller System

A comprehensive TMC9660 motor controller management system for the HardFOC project, designed for ESP32-C6 with external GPIO expanders and integrated with the existing ADC/GPIO systems.

## Overview

The TMC9660 Motor Controller system provides a unified, thread-safe interface for managing multiple TMC9660 motor controller chips. It integrates seamlessly with the HardFOC component handler system and supports both SPI and UART communication interfaces.

## Key Features

- **Thread-safe singleton pattern** for TMC9660 management
- **Multiple chip support** with unique identification
- **Dual communication interfaces** (SPI and UART) with runtime switching
- **Integrated GPIO control** through PCAL95555 GPIO expander
- **Built-in ADC support** for TMC9660's AIN1-3 channels
- **Comprehensive fault handling** and diagnostics
- **Automatic system health monitoring**
- **Extensible design** for future TMC9660 chip additions

## Hardware Configuration

### Primary TMC9660 Chip (TMC9660_CHIP_1)

#### Communication Interfaces
- **SPI Interface**: SPI0 bus with CS on GPIO18
- **UART Interface**: UART0 (GPIO4/GPIO5)
- **SPI Enable Control**: PCAL95555 P1_5 (TMC_SPI_COMM_nEN, active low)

#### Control Signals (via PCAL95555)
- **P0_3**: TMC_nFAULT_STATUS (fault status input, active low)
- **P0_4**: TMC_DRV_EN (driver enable output)
- **P0_5**: TMC_RST_CTRL (reset control output)
- **P1_6**: TMC_nWAKE_CTRL (wake control output, active low)

#### GPIO Pins (via PCAL95555)
- **P0_0**: TMC_GPIO17 (TMC9660 GPIO17)
- **P0_1**: TMC_GPIO18 (TMC9660 GPIO18)

#### ADC Channels
- **AIN1**: TMC9660 analog input 1
- **AIN2**: TMC9660 analog input 2
- **AIN3**: TMC9660 analog input 3

## Quick Start

### Basic Setup

```cpp
#include "component-handler/All.h"

void app_main() {
    using namespace HardFocComponentHandler;
    
    // Initialize the entire system (includes TMC9660)
    if (!Initialize()) {
        console_error("main", "Failed to initialize HardFOC system");
        return;
    }
    
    // System is ready to use
    console_info("main", "HardFOC system with TMC9660 initialized");
}
```

### Basic Motor Control

```cpp
using namespace HardFocComponentHandler;

MotorController& motorController = GetMotorController();
Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;

// Enable the motor driver
if (motorController.EnableDriver(primaryChip)) {
    console_info("main", "Motor driver enabled");
}

// Check for faults
if (motorController.HasFault(primaryChip)) {
    console_warning("main", "Motor fault detected!");
    motorController.ClearFault(primaryChip);
}

// Disable the motor driver
motorController.DisableDriver(primaryChip);
```

### Communication Interface Control

```cpp
using namespace HardFocComponentHandler;

MotorController& motorController = GetMotorController();
Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;

// Switch to SPI communication
if (motorController.SwitchCommunicationInterface(primaryChip, Tmc9660CommInterface::TMC_COMM_SPI)) {
    console_info("main", "Using SPI communication");
    
    // Perform SPI operations...
    
    // Switch back to UART
    motorController.SwitchCommunicationInterface(primaryChip, Tmc9660CommInterface::TMC_COMM_UART);
}
```

### ADC Reading

```cpp
using namespace HardFocComponentHandler;

MotorController& motorController = GetMotorController();
Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;

// Read motor current from AIN1
float current;
if (motorController.ReadAdcVoltage(primaryChip, 1, current)) {
    console_info("main", "Motor current: %.3f V", current);
}

// Read temperature from AIN2
float temperature;
if (motorController.ReadAdcVoltage(primaryChip, 2, temperature)) {
    console_info("main", "Motor temperature: %.3f V", temperature);
}
```

### GPIO Control

```cpp
using namespace HardFocComponentHandler;

MotorController& motorController = GetMotorController();
Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;

// Control GPIO17 and GPIO18
motorController.SetGpioState(primaryChip, 17, true);  // Set GPIO17 high
motorController.SetGpioState(primaryChip, 18, false); // Set GPIO18 low

// Read GPIO states
bool gpio17State = motorController.GetGpioState(primaryChip, 17);
bool gpio18State = motorController.GetGpioState(primaryChip, 18);
```

## API Reference

### Core Classes

#### Tmc9660MotorController
Main singleton class for TMC9660 management.

**Key Methods:**
- `GetInstance()` - Get singleton instance
- `RegisterTmc9660Chip()` - Register a new TMC9660 chip
- `EnableDriver()` / `DisableDriver()` - Control motor driver output
- `SwitchCommunicationInterface()` - Switch between SPI and UART
- `HasFault()` / `ClearFault()` - Fault management
- `ReadAdcValue()` / `ReadAdcVoltage()` - ADC operations
- `SetGpioState()` / `GetGpioState()` - GPIO control
- `TestCommunication()` - Communication test
- `RunDiagnostics()` - System diagnostics

### Configuration Structures

#### Tmc9660Config
TMC9660 chip configuration including communication settings.

#### Tmc9660GpioConfig
GPIO pin mapping configuration for TMC9660 control signals.

#### Tmc9660AdcConfig
ADC channel mapping configuration for TMC9660 analog inputs.

#### Tmc9660Status
Current status information for a TMC9660 chip.

### Enumerations

#### Tmc9660ChipId
- `TMC9660_CHIP_1` - Primary TMC9660 chip
- `TMC9660_CHIP_2` - Secondary chip (future expansion)
- `TMC9660_CHIP_3` - Third chip (future expansion)
- `TMC9660_CHIP_4` - Fourth chip (future expansion)

#### Tmc9660CommInterface
- `TMC_COMM_SPI` - SPI communication
- `TMC_COMM_UART` - UART communication

## System Integration

### Integration with HardFOC Systems

The TMC9660 system integrates with:

1. **ADC System**: TMC9660 ADC channels are registered as `ADC_TMC9660_AIN1`, `ADC_TMC9660_AIN2`, `ADC_TMC9660_AIN3`
2. **GPIO System**: TMC9660 control pins are mapped through the PCAL95555 GPIO expander
3. **System Initialization**: Automatic initialization through `SystemInit::InitializeTmc9660System()`
4. **Health Monitoring**: Integrated with overall system health checks

### Expanding to Multiple TMC9660 Chips

The system is designed to support up to 4 TMC9660 chips:

```cpp
// Register additional TMC9660 chips
Tmc9660Config chip2Config = {
    .chipId = Tmc9660ChipId::TMC9660_CHIP_2,
    .primaryInterface = Tmc9660CommInterface::TMC_COMM_UART,
    .uartPort = 1,  // Different UART port
    // ... other configuration
};

motorController.RegisterTmc9660Chip(Tmc9660ChipId::TMC9660_CHIP_2, 
                                   chip2Config, gpio2Config, adc2Config);
```

## Error Handling

### Fault Detection and Recovery

```cpp
using namespace HardFocComponentHandler;

MotorController& motorController = GetMotorController();
Tmc9660ChipId primaryChip = Tmc9660ChipId::TMC9660_CHIP_1;

// Check for faults in control loop
if (motorController.HasFault(primaryChip)) {
    console_warning("motor", "Fault detected, attempting recovery");
    
    // Disable driver
    motorController.DisableDriver(primaryChip);
    
    // Clear fault
    if (motorController.ClearFault(primaryChip)) {
        console_info("motor", "Fault cleared, re-enabling driver");
        motorController.EnableDriver(primaryChip);
    } else {
        console_error("motor", "Failed to clear fault, manual intervention required");
    }
}
```

### Communication Error Handling

```cpp
// Test communication before critical operations
if (!motorController.TestCommunication()) {
    console_error("motor", "Communication test failed");
    // Implement recovery or safe shutdown
}
```

## Diagnostics and Monitoring

### System Health Monitoring

```cpp
using namespace HardFocComponentHandler;

// Quick health check
if (!IsHealthy()) {
    console_warning("main", "System health degraded");
    
    // Get detailed status
    GetMotorController().PrintSystemStatus();
    
    // Run comprehensive diagnostics
    if (!GetMotorController().RunDiagnostics()) {
        console_error("main", "Diagnostics failed");
    }
}
```

### Periodic Maintenance

```cpp
void main_loop() {
    while (true) {
        // Perform periodic maintenance
        HardFocComponentHandler::Maintain();
        
        // Your application code here
        
        os_delay_msec(100);
    }
}
```

## Best Practices

### 1. Always Check Return Values
```cpp
if (!motorController.EnableDriver(primaryChip)) {
    console_error("motor", "Failed to enable driver");
    // Handle error appropriately
}
```

### 2. Use Proper Error Recovery
```cpp
// Always disable driver before attempting fault recovery
motorController.DisableDriver(primaryChip);
motorController.ClearFault(primaryChip);
```

### 3. Monitor System Health
```cpp
// Check system health regularly
if (!HardFocComponentHandler::IsHealthy()) {
    // Implement degraded mode operation
}
```

### 4. Use Communication Interface Switching Wisely
```cpp
// Switch to SPI only when needed for high-speed operations
motorController.SwitchCommunicationInterface(primaryChip, Tmc9660CommInterface::TMC_COMM_SPI);
// Perform high-speed operations
motorController.SwitchCommunicationInterface(primaryChip, Tmc9660CommInterface::TMC_COMM_UART);
```

## Future Expansion

The system is designed to easily accommodate additional TMC9660 chips and features:

1. **Additional Chips**: Simply register new chips with different configurations
2. **Extended GPIO**: GPIO expander pins can be easily reassigned
3. **Additional ADC**: More ADC channels can be mapped as needed
4. **Enhanced Communication**: Protocol extensions can be added without breaking existing code

## See Also

- `Tmc9660Example.cpp` - Complete usage examples
- `All.h` - Main include file with convenience functions
- `CommonIDs.h` - Enumeration definitions
- `SystemInit.h` - System initialization functions
