# EncoderController - AS5047U Encoder Management

![License](https://img.shields.io/badge/license-HardFOC-blue.svg)
![Component](https://img.shields.io/badge/component-EncoderController-green.svg)
![Hardware](https://img.shields.io/badge/hardware-AS5047U-orange.svg)

**Singleton class for managing multiple AS5047U position encoders with thread-safe access**

## Overview

The `EncoderController` is a sophisticated singleton class that provides centralized management for multiple AS5047U magnetic position encoders. It follows the same architectural excellence as `MotorController`, offering thread-safe, board-aware encoder management with automatic initialization and comprehensive error handling.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    EncoderController                        │
│  ┌─────────────────┬──────────────────┬─────────────────┐   │
│  │  Device Index 0 │  Device Index 1  │  Device Index 2 │   │
│  │   (Onboard)     │   (External 1)   │   (External 2)  │   │
│  └─────────────────┴──────────────────┴─────────────────┘   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              As5047uHandler Array                       │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
           │                    │                    │
    ┌──────▼──────┐      ┌──────▼──────┐      ┌──────▼──────┐
    │ As5047uHandler │    │ As5047uHandler │    │ As5047uHandler │
    │  (Onboard)   │    │  (External)   │    │  (External)   │
    └──────────────┘      └──────────────┘      └──────────────┘
           │                    │                    │
    ┌──────▼──────┐      ┌──────▼──────┐      ┌──────▼──────┐
    │  AS5047U     │    │  AS5047U     │    │  AS5047U     │
    │   Driver     │    │   Driver     │    │   Driver     │
    └──────────────┘      └──────────────┘      └──────────────┘
```

## Features

### Core Capabilities
- **Singleton Management**: Thread-safe singleton pattern for global encoder access
- **Multi-Device Support**: Up to 4 AS5047U encoders (1 onboard + 3 external)
- **Automatic Initialization**: Onboard encoder automatically created and initialized
- **Dynamic Device Management**: Runtime creation/deletion of external encoders
- **Thread-Safe Operations**: Full mutex protection for concurrent access

### High-Level Operations
- **Position Measurement**: Raw LSB and degree angle readings
- **Velocity Measurement**: RPM velocity calculations
- **Health Monitoring**: Comprehensive diagnostic checking
- **Multi-Encoder Operations**: Batch operations across all active encoders
- **Zero Position Setting**: Runtime calibration and reference setting

### Error Handling
- **Comprehensive Error Codes**: Detailed error reporting with `As5047uError` enum
- **Graceful Degradation**: Continues operation even if some encoders fail
- **Diagnostic Logging**: Detailed logging for troubleshooting

## Quick Start

### Basic Usage

```cpp
#include "component-handlers/EncoderController.h"

void basic_encoder_example() {
    // Get singleton instance
    auto& encoder = EncoderController::GetInstance();
    
    // Initialize (automatic for onboard encoder)
    if (!encoder.EnsureInitialized()) {
        printf("Encoder controller initialization failed\n");
        return;
    }
    
    // Read onboard encoder angle
    double angle_degrees;
    As5047uError result = encoder.ReadAngleDegrees(0, angle_degrees);
    
    if (result == As5047uError::SUCCESS) {
        printf("Encoder angle: %.2f°\n", angle_degrees);
    } else {
        printf("Read error: %s\n", As5047uErrorToString(result));
    }
}
```

### Handler Access Pattern

```cpp
void advanced_encoder_example() {
    auto& encoder = EncoderController::GetInstance();
    
    // Get direct handler access for advanced operations
    As5047uHandler* handler = encoder.handler(0);  // Onboard encoder
    if (handler) {
        // Use handler for advanced AS5047U features
        As5047uMeasurement measurement;
        if (handler->ReadMeasurement(measurement) == As5047uError::SUCCESS) {
            printf("Complete measurement data available\n");
        }
    }
    
    // Get direct sensor access
    auto sensor = encoder.sensor(0);
    if (sensor) {
        // Direct access to AS5047U driver
        printf("Direct sensor access available\n");
    }
}
```

## Device Management

### Device Indices

| Index | Description | Hardware Connection | Auto-Created |
|-------|-------------|-------------------|--------------|
| 0 | Onboard Encoder | SPI2_CS_AS5047U | ✅ Yes |
| 1 | External Device 1 | EXT_GPIO_CS_1 | ❌ Manual |
| 2 | External Device 2 | EXT_GPIO_CS_2 | ❌ Manual |
| 3 | External Device 3 | EXT_GPIO_CS_3 | ❌ Manual |

### Creating External Encoders

```cpp
void create_external_encoders() {
    auto& encoder = EncoderController::GetInstance();
    
    // Create external encoder on device index 1
    bool success = encoder.CreateExternalDevice(
        1,                                    // Device index
        SpiDeviceId::EXTERNAL_DEVICE_1,      // SPI device ID
        As5047uHandler::GetDefaultConfig()   // Configuration
    );
    
    if (success) {
        printf("External encoder 1 created successfully\n");
    }
}
```

### Deleting External Encoders

```cpp
void cleanup_external_encoders() {
    auto& encoder = EncoderController::GetInstance();
    
    // Delete external encoder
    if (encoder.DeleteExternalDevice(1)) {
        printf("External encoder 1 deleted\n");
    }
    
    // Note: Cannot delete onboard encoder (index 0)
}
```

## Measurement Operations

### Single Encoder Measurements

```cpp
void single_encoder_measurements() {
    auto& encoder = EncoderController::GetInstance();
    
    // Read raw angle (0-16383 LSB)
    uint16_t angle_lsb;
    encoder.ReadAngle(0, angle_lsb);
    
    // Read angle in degrees (0.0-359.978°)
    double angle_degrees;
    encoder.ReadAngleDegrees(0, angle_degrees);
    
    // Read velocity in RPM
    double velocity_rpm;
    encoder.ReadVelocityRPM(0, velocity_rpm);
    
    // Read comprehensive diagnostics
    As5047uDiagnostics diagnostics;
    encoder.ReadDiagnostics(0, diagnostics);
    
    printf("Angle: %.2f°, Velocity: %.1f RPM\n", angle_degrees, velocity_rpm);
    printf("Health: %s\n", diagnostics.magnetic_field_ok ? "OK" : "ERROR");
}
```

### Multi-Encoder Operations

```cpp
void multi_encoder_operations() {
    auto& encoder = EncoderController::GetInstance();
    
    // Read all active encoders simultaneously
    std::vector<uint16_t> angles;
    std::vector<uint8_t> device_indices;
    std::vector<As5047uError> errors = encoder.ReadAllAngles(angles, device_indices);
    
    printf("Active encoders: %zu\n", device_indices.size());
    for (size_t i = 0; i < device_indices.size(); ++i) {
        if (errors[i] == As5047uError::SUCCESS) {
            double angle_deg = As5047uHandler::LSBToDegrees(angles[i]);
            printf("  Device %u: %.2f°\n", device_indices[i], angle_deg);
        }
    }
    
    // Check health of all encoders
    bool all_healthy = encoder.CheckAllDevicesHealth();
    printf("System health: %s\n", all_healthy ? "OK" : "ISSUES");
}
```

## Configuration and Calibration

### Zero Position Setting

```cpp
void calibrate_encoder() {
    auto& encoder = EncoderController::GetInstance();
    
    // Read current position
    double current_angle;
    if (encoder.ReadAngleDegrees(0, current_angle) == As5047uError::SUCCESS) {
        printf("Current position: %.2f°\n", current_angle);
        
        // Set current position as zero reference
        uint16_t zero_lsb = As5047uHandler::DegreesToLSB(current_angle);
        As5047uError result = encoder.SetZeroPosition(0, zero_lsb);
        
        if (result == As5047uError::SUCCESS) {
            printf("Zero position set successfully\n");
        }
    }
}
```

### Advanced Configuration

```cpp
void advanced_configuration() {
    auto& encoder = EncoderController::GetInstance();
    
    // Get handler for advanced configuration
    As5047uHandler* handler = encoder.handler(0);
    if (handler) {
        // Configure DAEC (Dynamic Angle Error Compensation)
        handler->SetDAEC(true);
        
        // Configure rotation direction
        handler->SetRotationDirection(true);  // Clockwise positive
        
        // Configure interface outputs
        handler->ConfigureInterface(true, true, false);  // ABI, UVW, no PWM
        
        printf("Advanced configuration applied\n");
    }
}
```

## Health Monitoring and Diagnostics

### System Health Check

```cpp
void monitor_system_health() {
    auto& encoder = EncoderController::GetInstance();
    
    // Quick health check
    bool healthy = encoder.CheckAllDevicesHealth();
    printf("System health: %s\n", healthy ? "HEALTHY" : "ISSUES");
    
    // Detailed status report
    std::string status = encoder.GetStatusReport();
    printf("\n%s\n", status.c_str());
    
    // Comprehensive diagnostics dump
    encoder.DumpAllDiagnostics();
}
```

### Error Handling

```cpp
void error_handling_example() {
    auto& encoder = EncoderController::GetInstance();
    
    double angle;
    As5047uError result = encoder.ReadAngleDegrees(0, angle);
    
    switch (result) {
        case As5047uError::SUCCESS:
            printf("Angle: %.2f°\n", angle);
            break;
            
        case As5047uError::NOT_INITIALIZED:
            printf("Encoder not initialized\n");
            encoder.EnsureInitialized();  // Retry initialization
            break;
            
        case As5047uError::SPI_COMMUNICATION_FAILED:
            printf("SPI communication error\n");
            // Check hardware connections
            break;
            
        case As5047uError::SENSOR_ERROR:
            printf("Sensor hardware error\n");
            // Check magnetic field and sensor placement
            break;
            
        default:
            printf("Error: %s\n", As5047uErrorToString(result));
            break;
    }
}
```

## Closed-Loop Control Integration

### Position Control Example

```cpp
void position_control_example() {
    auto& encoder = EncoderController::GetInstance();
    auto& motor = MotorController::GetInstance();  // Assuming motor control
    
    double target_position = 180.0;  // Target 180°
    double tolerance = 1.0;          // ±1° tolerance
    
    for (int i = 0; i < 100; ++i) {
        // Read current position
        double current_position;
        if (encoder.ReadAngleDegrees(0, current_position) != As5047uError::SUCCESS) {
            printf("Encoder read error\n");
            break;
        }
        
        // Calculate position error
        double error = target_position - current_position;
        
        // Handle angle wraparound
        if (error > 180.0) error -= 360.0;
        else if (error < -180.0) error += 360.0;
        
        // Check if target reached
        if (std::abs(error) <= tolerance) {
            printf("Target position reached!\n");
            break;
        }
        
        // Calculate control output (simple P controller)
        double control_output = 0.1 * error;  // Proportional gain = 0.1
        
        // Apply control to motor (pseudo-code)
        // motor.SetVelocity(0, control_output);
        
        printf("Position: %.1f°, Error: %.1f°, Control: %.2f\n", 
               current_position, error, control_output);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
```

## Utility Functions

### Angle Conversions

```cpp
void angle_conversion_examples() {
    // LSB to degrees conversion
    uint16_t angle_lsb = 8192;  // Half scale (180°)
    double angle_deg = As5047uHandler::LSBToDegrees(angle_lsb);
    printf("LSB %u = %.2f°\n", angle_lsb, angle_deg);
    
    // Degrees to LSB conversion
    double target_deg = 90.0;
    uint16_t target_lsb = As5047uHandler::DegreesToLSB(target_deg);
    printf("%.1f° = LSB %u\n", target_deg, target_lsb);
    
    // Radians conversion
    double angle_rad = As5047uHandler::LSBToRadians(angle_lsb);
    uint16_t from_rad_lsb = As5047uHandler::RadiansToLSB(angle_rad);
    printf("LSB %u = %.4f rad = LSB %u\n", angle_lsb, angle_rad, from_rad_lsb);
}
```

## API Reference

### Core Methods

| Method | Description | Return Type |
|--------|-------------|-------------|
| `GetInstance()` | Get singleton instance | `EncoderController&` |
| `EnsureInitialized()` | Ensure initialization | `bool` |
| `IsInitialized()` | Check initialization status | `bool` |

### Handler Access

| Method | Description | Return Type |
|--------|-------------|-------------|
| `handler(deviceIndex)` | Get AS5047U handler | `As5047uHandler*` |
| `sensor(deviceIndex)` | Get AS5047U sensor | `std::shared_ptr<AS5047U>` |

### Device Management

| Method | Description | Return Type |
|--------|-------------|-------------|
| `CreateExternalDevice()` | Create external encoder | `bool` |
| `DeleteExternalDevice()` | Delete external encoder | `bool` |
| `GetDeviceCount()` | Get active device count | `uint8_t` |
| `GetActiveDeviceIndices()` | Get active device list | `std::vector<uint8_t>` |
| `IsDeviceActive()` | Check device status | `bool` |

### Measurements

| Method | Description | Return Type |
|--------|-------------|-------------|
| `ReadAngle()` | Read raw angle (LSB) | `As5047uError` |
| `ReadAngleDegrees()` | Read angle in degrees | `As5047uError` |
| `ReadVelocityRPM()` | Read velocity in RPM | `As5047uError` |
| `ReadDiagnostics()` | Read diagnostics | `As5047uError` |
| `SetZeroPosition()` | Set zero reference | `As5047uError` |

### Multi-Encoder Operations

| Method | Description | Return Type |
|--------|-------------|-------------|
| `ReadAllAngles()` | Read all encoder angles | `std::vector<As5047uError>` |
| `ReadAllVelocities()` | Read all velocities | `std::vector<As5047uError>` |
| `CheckAllDevicesHealth()` | Check system health | `bool` |

### Diagnostics

| Method | Description | Return Type |
|--------|-------------|-------------|
| `GetStatusReport()` | Get status string | `std::string` |
| `DumpAllDiagnostics()` | Dump diagnostics to log | `void` |
| `ResetAllStatistics()` | Reset statistics | `void` |

## Error Codes

| Error Code | Description |
|------------|-------------|
| `SUCCESS` | Operation completed successfully |
| `NOT_INITIALIZED` | Controller or device not initialized |
| `INITIALIZATION_FAILED` | Device initialization failed |
| `INVALID_PARAMETER` | Invalid parameter provided |
| `SPI_COMMUNICATION_FAILED` | SPI communication error |
| `CRC_ERROR` | Data integrity error |
| `FRAMING_ERROR` | Protocol framing error |
| `SENSOR_ERROR` | Hardware sensor error |
| `TIMEOUT` | Operation timeout |
| `MUTEX_LOCK_FAILED` | Thread synchronization error |

## Thread Safety

The EncoderController is fully thread-safe:

- **Singleton Access**: Thread-safe singleton initialization
- **Device Management**: Mutex-protected device operations
- **Measurement Operations**: Atomic measurement operations
- **Handler Access**: Safe concurrent handler access

## Best Practices

### Initialization
```cpp
// Always check initialization
auto& encoder = EncoderController::GetInstance();
if (!encoder.EnsureInitialized()) {
    // Handle initialization failure
    return;
}
```

### Error Handling
```cpp
// Always check return codes
As5047uError result = encoder.ReadAngleDegrees(0, angle);
if (result != As5047uError::SUCCESS) {
    // Handle error appropriately
    printf("Error: %s\n", As5047uErrorToString(result));
}
```

### Resource Management
```cpp
// External devices can be created/deleted dynamically
if (encoder.CreateExternalDevice(1, SpiDeviceId::EXTERNAL_DEVICE_1)) {
    // Use external device
    // ...
    
    // Clean up when done
    encoder.DeleteExternalDevice(1);
}
```

## Integration Examples

See `/examples/EncoderControllerExample.cpp` for comprehensive usage examples including:

- Basic encoder reading
- Multi-encoder operations
- Continuous monitoring
- Health checking
- Calibration procedures
- Closed-loop control integration

## Hardware Requirements

- HardFOC board with onboard AS5047U encoder
- Optional: External AS5047U encoders on SPI bus
- Proper magnetic field for sensor operation (4-7mm magnet distance)

## See Also

- [AS5047U Handler Documentation](../driver-handlers/AS5047U_HANDLER_README.md)
- [Motor Controller Documentation](MOTOR_CONTROLLER_README.md)
- [Communication Channels Manager](COMM_CHANNELS_MANAGER_README.md)