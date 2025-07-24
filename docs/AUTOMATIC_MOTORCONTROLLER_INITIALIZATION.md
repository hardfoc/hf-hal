# Automatic MotorController Initialization Architecture

## Overview

The MotorController has been enhanced to support automatic onboard device creation through integration with CommChannelsManager. This simplifies the initialization process while maintaining the board-aware device management capabilities.

## Key Features

### 1. **Automatic Onboard Device Creation**
- Onboard TMC9660 device is created automatically during `Initialize()`
- Uses CommChannelsManager to obtain the appropriate SPI interface
- No manual device creation required for onboard functionality
- Default address 0x01 and kDefaultBootConfig applied automatically

### 2. **Simplified External Device Creation**
- External devices specify SPI channel via `SpiDeviceId` enum
- No need to manually manage SPI interface references
- CommChannelsManager handles interface retrieval automatically
- Supports both SPI and UART interfaces for external devices

### 3. **Board-Aware Architecture**
- Onboard device: Index 0 (permanent, auto-created)
- External devices: Indices 2-3 (dynamic, user-created)
- Index 1 reserved for AS5047U position encoder
- Clean separation between permanent and external devices

## Architecture Details

### Initialization Flow

```cpp
auto& motorController = MotorController::GetInstance();
bool success = motorController.Initialize();  // Onboard device created automatically
```

**Internal Process:**
1. Initialize CommChannelsManager
2. Get SPI interface for TMC9660_MOTOR_CONTROLLER
3. Create Tmc9660Handler with default configuration
4. Register at ONBOARD_TMC9660_INDEX (0)
5. Initialize all active devices

### External Device Creation

```cpp
// Create external device using SPI device ID
bool success = motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX,  // Board index (2 or 3)
    SpiDeviceId::EXTERNAL_DEVICE_1,            // SPI channel specification
    0x02                                       // TMC9660 address
);
```

**Internal Process:**
1. Validate device index (must be 2 or 3)
2. Check slot availability
3. Get SPI interface from CommChannelsManager
4. Create Tmc9660Handler with specified configuration
5. Register at specified index
6. Initialize if system already initialized

## Usage Patterns

### 1. **Simple Initialization**
```cpp
auto& motorController = MotorController::GetInstance();

// One-line initialization - onboard device ready to use
if (motorController.Initialize()) {
    // Access onboard device immediately
    auto& gpio17 = motorController.handler().gpio(17);
    auto& adc = motorController.handler().adc();
}
```

### 2. **Mixed Device Setup**
```cpp
auto& motorController = MotorController::GetInstance();

// Initialize with automatic onboard device creation
motorController.Initialize();

// Add external devices as needed
motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX,
    SpiDeviceId::EXTERNAL_DEVICE_1,
    0x02
);

// Access devices by index
auto& onboardHandler = motorController.handler(0);  // Onboard
auto& externalHandler = motorController.handler(2); // External
```

### 3. **UART External Device**
```cpp
// UART interface still requires manual reference
BaseUart& uartInterface = /* obtain UART interface */;
motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_2_INDEX,
    uartInterface,
    0x03
);
```

## Board Index Mapping

| Index | Device Type | CS Line | Auto-Created | Deletable |
|-------|-------------|---------|--------------|-----------|
| 0     | Onboard TMC9660 | SPI2_CS_TMC9660 | Yes | No |
| 1     | AS5047U Encoder | SPI2_CS_AS5047 | No | N/A |
| 2     | External TMC9660 #1 | EXT_GPIO_CS_1 | No | Yes |
| 3     | External TMC9660 #2 | EXT_GPIO_CS_2 | No | Yes |

## SPI Device ID Mapping

| SpiDeviceId | Purpose | Auto-Used |
|-------------|---------|-----------|
| `TMC9660_MOTOR_CONTROLLER` | Onboard TMC9660 | Yes |
| `AS5047U_POSITION_ENCODER` | Position encoder | No |
| `EXTERNAL_DEVICE_1` | External TMC9660 #1 | No |
| `EXTERNAL_DEVICE_2` | External TMC9660 #2 | No |

## API Changes

### New Behavior
```cpp
// OLD: Manual onboard device creation required
motorController.CreateOnboardDevice(spiInterface, 0x01);
motorController.Initialize();

// NEW: Automatic onboard device creation
motorController.Initialize();  // Done!
```

### External Device Creation
```cpp
// OLD: Manual SPI interface management
BaseSpi& spiInterface = /* manage yourself */;
motorController.CreateExternalDevice(2, spiInterface, 0x02);

// NEW: SPI device ID specification
motorController.CreateExternalDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1, 0x02);
```

## Error Handling

### CommChannelsManager Dependency
- `Initialize()` returns false if CommChannelsManager fails
- External device creation validates CommChannelsManager state
- SPI interface availability checked before device creation

### Device Creation Validation
- Index validation (external devices: 2-3 only)
- Slot availability checking
- SPI interface availability verification
- Thread-safe device registration

## Benefits

1. **Simplified API**: One-line initialization for common use cases
2. **Automatic Configuration**: Default settings applied automatically
3. **Board Awareness**: CS pin assignments handled transparently
4. **Resource Management**: CommChannelsManager handles interface lifecycle
5. **Flexibility**: Manual creation still available for custom configurations
6. **Error Prevention**: Invalid configurations caught at creation time

## Migration Guide

### From Manual Creation
```cpp
// Old pattern
auto& commManager = CommChannelsManager::GetInstance();
commManager.EnsureInitialized();
BaseSpi* spi = commManager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
motorController.CreateOnboardDevice(*spi, 0x01);
motorController.Initialize();

// New pattern
motorController.Initialize();  // Everything handled automatically
```

### External Device Updates
```cpp
// Old pattern
BaseSpi& spi1 = /* obtain SPI interface */;
motorController.CreateExternalDevice(2, spi1, 0x02);

// New pattern
motorController.CreateExternalDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1, 0x02);
```

## Future Enhancements

1. **Configurable Default Address**: Allow override of default TMC9660 address
2. **Boot Config Override**: Support custom bootloader configurations
3. **UART Auto-Creation**: Extend automatic creation to UART interfaces
4. **Hot-Plug Support**: Dynamic device detection and creation
5. **Configuration Persistence**: Save/restore device configurations

This architecture provides a clean, intuitive interface while maintaining the power and flexibility needed for complex motor control applications.
