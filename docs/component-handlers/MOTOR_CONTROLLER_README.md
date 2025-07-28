# MotorController - Motor Control Management System

<div align="center">

![Component](https://img.shields.io/badge/component-MotorController-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)

**Advanced motor controller management system for the HardFOC platform**

</div>

## 📋 Overview

The `MotorController` is a singleton component handler that provides unified management of multiple TMC9660 motor controllers. It supports the onboard TMC9660 device and up to 3 external TMC9660 devices, providing array-based access to individual motor controllers and their associated GPIO/ADC resources.

### ✨ Key Features

- **🎛️ Multi-Device Management**: Support for up to 4 TMC9660 controllers
- **🔗 Unified Interface**: Single API for all motor controllers
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **🏗️ Dynamic Device Management**: Runtime creation/deletion of external devices
- **📊 Advanced Diagnostics**: Per-device health monitoring and statistics
- **⚡ High-Performance Control**: Optimized motor control operations
- **🛡️ Safety Features**: Fault monitoring and protection
- **🔌 Flexible Communication**: SPI and UART interfaces supported

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     MotorController                            │
├─────────────────────────────────────────────────────────────────┤
│  Device Management   │ Array-based TMC9660 device access      │
├─────────────────────────────────────────────────────────────────┤
│  Handler Access      │ Individual Tmc9660Handler instances    │
├─────────────────────────────────────────────────────────────────┤
│  Communication       │ SPI/UART interface management          │
├─────────────────────────────────────────────────────────────────┤
│  TMC9660 Drivers     │ Direct TMC9660 device instances        │
└─────────────────────────────────────────────────────────────────┘
```

### Device Index Mapping

| Index | Device Type | CS Line | Description |
|-------|-------------|---------|-------------|
| **0** | Onboard | SPI2_CS_TMC9660 | Primary motor controller (always present) |
| **1** | Reserved | - | Reserved for future use |
| **2** | External | EXT_GPIO_CS_1 | External device 1 (optional) |
| **3** | External | EXT_GPIO_CS_2 | External device 2 (optional) |

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/MotorController.h"

void motor_example() {
    // Get singleton instance
    auto& motor = MotorController::GetInstance();
    
    // Initialize the manager
    if (!motor.EnsureInitialized()) {
        printf("Failed to initialize motor controller\n");
        return;
    }
    
    // Get onboard TMC9660 handler
    auto* handler = motor.handler(0);  // Index 0 = onboard device
    if (handler && handler->Initialize()) {
        auto tmc = handler->GetTmc9660Driver();
        
        // Configure motor parameters
        tmc->SetTargetVelocity(1000);  // RPM
        tmc->SetMaxCurrent(1000);      // mA
        
        // Enable motor
        tmc->EnableMotor(true);
        
        printf("Motor controller ready\n");
    }
}
```

## 📖 API Reference

### Core Operations

#### Initialization
```cpp
class MotorController {
public:
    // Singleton access
    static MotorController& GetInstance();
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### Device Management
```cpp
// Handler access
Tmc9660Handler* handler(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

// Direct driver access
std::shared_ptr<TMC9660> driver(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

// Device creation/deletion
bool CreateExternalDevice(uint8_t csDeviceIndex, 
                         SpiDeviceId spiDeviceId, 
                         uint8_t address,
                         const tmc9660::BootloaderConfig* bootCfg = nullptr);

bool DeleteExternalDevice(uint8_t csDeviceIndex);

// Device information
uint8_t GetDeviceCount() const noexcept;
```

#### Device Constants
```cpp
static constexpr uint8_t MAX_TMC9660_DEVICES = 4;
static constexpr uint8_t ONBOARD_TMC9660_INDEX = 0;
static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 2;
static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 3;
```

### TMC9660 Handler Access

Each TMC9660 device is managed through a `Tmc9660Handler` instance:

```cpp
// Get handler for specific device
auto* handler = motor.handler(0);  // Onboard device

if (handler) {
    // Initialize the handler
    if (handler->Initialize()) {
        // Access TMC9660 driver
        auto tmc = handler->GetTmc9660Driver();
        
        // Access GPIO wrapper
        auto gpio = handler->GetGpioWrapper();
        
        // Access ADC wrapper
        auto adc = handler->GetAdcWrapper();
    }
}
```

## 🎯 Hardware Support

### TMC9660 Motor Controller Features

| Feature | Description | Support |
|---------|-------------|---------|
| **Motor Control** | Stepper/BLDC motor control | ✅ Full support |
| **Current Control** | Configurable current limits | ✅ 0-2A range |
| **Velocity Control** | RPM-based speed control | ✅ Up to 10,000 RPM |
| **Position Control** | Absolute/relative positioning | ✅ 32-bit resolution |
| **GPIO** | 8 configurable GPIO pins | ✅ Digital I/O |
| **ADC** | 3 analog input channels | ✅ 12-bit resolution |
| **Fault Detection** | Overcurrent, overtemperature | ✅ Automatic protection |
| **Communication** | SPI/UART interfaces | ✅ Configurable |

### Communication Interfaces

```cpp
// SPI Configuration
struct SpiConfig {
    uint32_t clock_speed_hz = 1000000;  // 1 MHz
    uint8_t mode = 3;                   // SPI Mode 3
    uint8_t bits_per_word = 8;
};

// UART Configuration  
struct UartConfig {
    uint32_t baud_rate = 115200;
    uint8_t data_bits = 8;
    uint8_t stop_bits = 1;
    uart_parity_t parity = UART_PARITY_DISABLE;
};
```

## 🔧 Configuration

### Default Bootloader Configuration

```cpp
// Default TMC9660 bootloader settings
static const tmc9660::BootloaderConfig kDefaultBootConfig = {
    .motor_type = tmc9660::MotorType::STEPPER,
    .current_limit_ma = 1000,
    .microstep_resolution = 256,
    .velocity_limit_rpm = 1000,
    .acceleration_limit_rpm_per_s = 1000,
    .enable_stallguard = true,
    .stallguard_threshold = 100,
    .enable_coolstep = true,
    .enable_spreadcycle = true
};
```

### External Device Configuration

```cpp
// Create external TMC9660 device
bool success = motor.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX,  // Device index 2
    SpiDeviceId::EXTERNAL_DEVICE_1,            // SPI device ID
    0x01,                                      // TMC9660 address
    &custom_boot_config                        // Optional config
);
```

## 📊 Examples

### Basic Motor Control

```cpp
#include "component-handlers/MotorController.h"

void basic_motor_control() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Get onboard motor controller
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) {
        printf("Failed to initialize motor handler\n");
        return;
    }
    
    auto tmc = handler->GetTmc9660Driver();
    
    // Configure motor parameters
    tmc->SetMotorType(tmc9660::MotorType::STEPPER);
    tmc->SetMaxCurrent(800);           // 800mA
    tmc->SetMicrostepResolution(256);  // 256 microsteps
    tmc->SetVelocityLimit(500);        // 500 RPM
    tmc->SetAccelerationLimit(1000);   // 1000 RPM/s
    
    // Enable motor
    tmc->EnableMotor(true);
    
    // Move motor
    printf("Moving motor...\n");
    tmc->MoveToPosition(1000);  // Move to position 1000
    
    // Wait for completion
    while (tmc->IsMoving()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    printf("Move completed\n");
    tmc->EnableMotor(false);
}
```

### Velocity Control

```cpp
void velocity_control_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) return;
    
    auto tmc = handler->GetTmc9660Driver();
    
    // Configure for velocity mode
    tmc->SetControlMode(tmc9660::ControlMode::VELOCITY);
    tmc->SetMaxCurrent(1000);
    tmc->SetAccelerationLimit(2000);
    
    tmc->EnableMotor(true);
    
    // Velocity ramp test
    printf("Velocity ramp test:\n");
    for (int rpm = 0; rpm <= 1000; rpm += 100) {
        printf("Setting velocity: %d RPM\n", rpm);
        tmc->SetTargetVelocity(rpm);
        vTaskDelay(pdMS_TO_TICKS(2000));  // Wait 2 seconds
        
        // Read actual velocity
        int32_t actual_vel = tmc->GetActualVelocity();
        printf("  Actual velocity: %ld RPM\n", actual_vel);
    }
    
    // Stop motor
    tmc->SetTargetVelocity(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    tmc->EnableMotor(false);
}
```

### Multi-Motor Control

```cpp
void multi_motor_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Create external motor controller
    bool external_created = motor.CreateExternalDevice(
        MotorController::EXTERNAL_DEVICE_1_INDEX,
        SpiDeviceId::EXTERNAL_DEVICE_1,
        0x01  // Device address
    );
    
    if (!external_created) {
        printf("Failed to create external motor device\n");
        return;
    }
    
    // Initialize both motors
    auto* motor1 = motor.handler(0);  // Onboard
    auto* motor2 = motor.handler(2);  // External
    
    if (!motor1 || !motor1->Initialize() ||
        !motor2 || !motor2->Initialize()) {
        printf("Failed to initialize motors\n");
        return;
    }
    
    auto tmc1 = motor1->GetTmc9660Driver();
    auto tmc2 = motor2->GetTmc9660Driver();
    
    // Configure both motors
    for (auto* tmc : {tmc1.get(), tmc2.get()}) {
        tmc->SetMaxCurrent(800);
        tmc->SetMicrostepResolution(256);
        tmc->SetVelocityLimit(500);
        tmc->EnableMotor(true);
    }
    
    // Synchronized movement
    printf("Synchronized motor movement\n");
    tmc1->MoveToPosition(1000);
    tmc2->MoveToPosition(-1000);  // Opposite direction
    
    // Wait for both to complete
    while (tmc1->IsMoving() || tmc2->IsMoving()) {
        printf("Motor 1 pos: %ld, Motor 2 pos: %ld\n",
               tmc1->GetActualPosition(), tmc2->GetActualPosition());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    printf("Synchronized movement completed\n");
    
    // Disable motors
    tmc1->EnableMotor(false);
    tmc2->EnableMotor(false);
}
```

### Motor GPIO and ADC Usage

```cpp
void motor_io_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) return;
    
    // Access GPIO wrapper
    auto gpio = handler->GetGpioWrapper();
    auto adc = handler->GetAdcWrapper();
    
    if (gpio && adc) {
        // Configure GPIO pins
        gpio->ConfigurePin(0, false);  // Pin 0 as output
        gpio->ConfigurePin(1, true);   // Pin 1 as input
        
        printf("TMC9660 GPIO and ADC monitoring:\n");
        
        for (int i = 0; i < 10; i++) {
            // Toggle output pin
            gpio->SetPin(0, i % 2);
            
            // Read input pin
            bool input_state = gpio->GetPin(1);
            
            // Read ADC channels
            uint16_t ain1 = adc->ReadRaw(0);  // AIN1
            uint16_t ain2 = adc->ReadRaw(1);  // AIN2
            uint16_t ain3 = adc->ReadRaw(2);  // AIN3
            
            printf("GPIO1: %s, AIN1: %u, AIN2: %u, AIN3: %u\n",
                   input_state ? "HIGH" : "LOW", ain1, ain2, ain3);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
```

### Fault Monitoring

```cpp
void fault_monitoring_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) return;
    
    auto tmc = handler->GetTmc9660Driver();
    
    // Enable fault monitoring
    tmc->EnableStallguard(true);
    tmc->SetStallguardThreshold(100);
    tmc->EnableOvertemperatureProtection(true);
    tmc->EnableOvercurrentProtection(true);
    
    tmc->EnableMotor(true);
    
    printf("Motor fault monitoring active...\n");
    
    while (true) {
        // Check for faults
        auto status = tmc->GetDriverStatus();
        
        if (status.stall_detected) {
            printf("FAULT: Motor stall detected!\n");
            tmc->EnableMotor(false);
            break;
        }
        
        if (status.overtemperature_warning) {
            printf("WARNING: Motor overtemperature!\n");
        }
        
        if (status.overcurrent_detected) {
            printf("FAULT: Overcurrent detected!\n");
            tmc->EnableMotor(false);
            break;
        }
        
        // Print status
        printf("Current: %dmA, Temp: %d°C, Pos: %ld\n",
               status.actual_current_ma,
               status.temperature_celsius,
               tmc->GetActualPosition());
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    printf("Fault monitoring stopped\n");
}
```

### Communication Interface Testing

```cpp
void communication_test() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    printf("Testing TMC9660 communication...\n");
    
    for (uint8_t device_idx = 0; device_idx < motor.GetDeviceCount(); device_idx++) {
        auto* handler = motor.handler(device_idx);
        if (!handler) continue;
        
        printf("Testing device %u:\n", device_idx);
        
        // Test communication
        bool comm_ok = handler->TestCommunication();
        printf("  Communication: %s\n", comm_ok ? "OK" : "FAILED");
        
        if (comm_ok && handler->Initialize()) {
            auto tmc = handler->GetTmc9660Driver();
            
            // Read device info
            uint32_t version = tmc->GetChipVersion();
            printf("  Chip version: 0x%08lX\n", version);
            
            // Test register access
            tmc->SetMaxCurrent(500);
            uint16_t current = tmc->GetMaxCurrent();
            printf("  Current setting test: %s\n", 
                   (current == 500) ? "PASS" : "FAIL");
        }
    }
}
```

## 🔍 Advanced Usage

### Custom Boot Configuration

```cpp
void custom_boot_config_example() {
    // Define custom bootloader configuration
    tmc9660::BootloaderConfig custom_config = {
        .motor_type = tmc9660::MotorType::BLDC,
        .current_limit_ma = 1500,
        .microstep_resolution = 512,
        .velocity_limit_rpm = 2000,
        .acceleration_limit_rpm_per_s = 5000,
        .enable_stallguard = false,
        .stallguard_threshold = 0,
        .enable_coolstep = true,
        .enable_spreadcycle = false,
        .pwm_frequency_khz = 35,
        .enable_stealth_chop = true
    };
    
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Create external device with custom config
    bool success = motor.CreateExternalDevice(
        MotorController::EXTERNAL_DEVICE_1_INDEX,
        SpiDeviceId::EXTERNAL_DEVICE_1,
        0x01,
        &custom_config
    );
    
    if (success) {
        printf("External motor created with custom configuration\n");
        
        auto* handler = motor.handler(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (handler && handler->Initialize()) {
            auto tmc = handler->GetTmc9660Driver();
            printf("BLDC motor controller ready\n");
        }
    }
}
```

### Performance Optimization

```cpp
void performance_optimization() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) return;
    
    auto tmc = handler->GetTmc9660Driver();
    
    // Optimize for high-performance operation
    tmc->SetControlMode(tmc9660::ControlMode::VELOCITY);
    tmc->SetMicrostepResolution(256);  // Higher resolution
    tmc->EnableSpreadCycle(true);      // Better high-speed performance
    tmc->EnableStealthChop(false);     // Disable for performance
    tmc->SetPwmFrequency(70000);       // 70kHz PWM
    
    // Set aggressive acceleration
    tmc->SetAccelerationLimit(10000);  // 10000 RPM/s
    tmc->SetVelocityLimit(5000);       // 5000 RPM
    
    tmc->EnableMotor(true);
    
    // High-speed velocity profile
    std::array<int32_t, 5> velocities = {1000, 2000, 3000, 4000, 5000};
    
    for (auto velocity : velocities) {
        printf("Accelerating to %ld RPM...\n", velocity);
        auto start_time = esp_timer_get_time();
        
        tmc->SetTargetVelocity(velocity);
        
        // Wait for velocity to stabilize
        while (std::abs(tmc->GetActualVelocity() - velocity) > 50) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        auto end_time = esp_timer_get_time();
        printf("  Reached in %.1f ms\n", (end_time - start_time) / 1000.0f);
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Hold velocity
    }
    
    // Decelerate to stop
    tmc->SetTargetVelocity(0);
    tmc->EnableMotor(false);
}
```

## 🚨 Error Handling

### Comprehensive Error Handling

```cpp
void error_handling_example() {
    auto& motor = MotorController::GetInstance();
    
    // Check initialization
    if (!motor.EnsureInitialized()) {
        printf("ERROR: Failed to initialize motor controller\n");
        return;
    }
    
    // Validate device exists
    auto* handler = motor.handler(0);
    if (!handler) {
        printf("ERROR: Motor handler not available\n");
        return;
    }
    
    // Initialize with error checking
    if (!handler->Initialize()) {
        printf("ERROR: Failed to initialize TMC9660 handler\n");
        return;
    }
    
    auto tmc = handler->GetTmc9660Driver();
    if (!tmc) {
        printf("ERROR: TMC9660 driver not available\n");
        return;
    }
    
    // Test communication before use
    if (!handler->TestCommunication()) {
        printf("ERROR: TMC9660 communication failed\n");
        return;
    }
    
    // Monitor for runtime errors
    tmc->EnableMotor(true);
    
    for (int i = 0; i < 100; i++) {
        auto status = tmc->GetDriverStatus();
        
        if (status.communication_error) {
            printf("ERROR: Communication error detected\n");
            break;
        }
        
        if (status.stall_detected) {
            printf("ERROR: Motor stall detected\n");
            tmc->EnableMotor(false);
            break;
        }
        
        if (status.overtemperature_error) {
            printf("ERROR: Overtemperature shutdown\n");
            tmc->EnableMotor(false);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    printf("Motor control completed successfully\n");
}
```

## 🔗 Integration

### Integration with GPIO and ADC Managers

```cpp
#include "component-handlers/All.h"

void integrated_motor_control() {
    // Initialize all managers
    auto& gpio = GpioManager::GetInstance();
    auto& adc = AdcManager::GetInstance();
    auto& motor = MotorController::GetInstance();
    
    gpio.EnsureInitialized();
    adc.Initialize();
    motor.EnsureInitialized();
    
    // Configure external enable pin
    gpio.ConfigurePin("ESP32_GPIO_5", false);  // Motor enable output
    gpio.ConfigurePin("ESP32_GPIO_6", true);   // Emergency stop input
    
    // Motor control with external safety
    auto* handler = motor.handler(0);
    if (handler && handler->Initialize()) {
        auto tmc = handler->GetTmc9660Driver();
        
        // Enable motor via external GPIO
        gpio.SetPin("ESP32_GPIO_5", true);
        tmc->EnableMotor(true);
        
        printf("Integrated motor control active\n");
        
        while (true) {
            // Check emergency stop
            bool e_stop = !gpio.GetPin("ESP32_GPIO_6");  // Active low
            if (e_stop) {
                printf("Emergency stop activated!\n");
                tmc->EnableMotor(false);
                gpio.SetPin("ESP32_GPIO_5", false);
                break;
            }
            
            // Monitor supply voltage via ADC
            float supply_voltage = adc.ReadVoltage("ESP32_ADC1_CH0");
            if (supply_voltage < 10.0f) {
                printf("Low voltage detected: %.2fV\n", supply_voltage);
                tmc->EnableMotor(false);
                break;
            }
            
            // Normal operation
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 device driver
- **[Complete Motor Control Guide](../HARDFOC_GPIO_ADC_SYSTEM.md)** - Comprehensive system documentation

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*