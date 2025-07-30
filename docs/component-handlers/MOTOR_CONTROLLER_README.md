# MotorController - Motor Control Management System

<div align="center">

![Component](https://img.shields.io/badge/component-MotorController-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)

**Advanced motor controller management system for the HardFOC platform**

</div>

## 📋 Overview

The `MotorController` is a singleton component handler that provides unified management of multiple TMC9660 motor controllers. It supports the onboard TMC9660 device and up to 2 external TMC9660 devices, providing array-based access to individual motor controllers and their associated GPIO/ADC resources.

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
        auto tmc = motor.driver(0);  // Get TMC9660 driver
        
        if (tmc) {
            // Configure motor parameters
            tmc->SetTargetVelocity(1000);  // RPM
            tmc->SetMaxCurrent(1000);      // mA
            
            // Enable motor
            tmc->EnableMotor(true);
            
            printf("Motor controller ready\n");
        }
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
bool IsDeviceValid(uint8_t deviceIndex) const noexcept;
bool IsExternalSlotAvailable(uint8_t csDeviceIndex) const noexcept;
std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;
```

#### Device Constants
```cpp
static constexpr uint8_t MAX_TMC9660_DEVICES = 4;
static constexpr uint8_t ONBOARD_TMC9660_INDEX = 0;
static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 2;
static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 3;
```

#### System Management
```cpp
// Device initialization
std::vector<bool> InitializeAllDevices();
std::vector<bool> GetInitializationStatus() const;

// Statistics
void DumpStatistics() const noexcept;
```

## 🎯 Hardware Support

### TMC9660 Motor Controller Features

| Feature | Description | Support |
|---------|-------------|---------|
| **Motor Control** | Stepper/BLDC motor control | ✅ Full support |
| **Current Control** | Configurable current limits | ✅ 0-2A range |
| **Velocity Control** | RPM-based speed control | ✅ Up to 10,000 RPM |
| **Position Control** | Absolute/relative positioning | ✅ 32-bit resolution |
| **GPIO** | 18 configurable GPIO pins | ✅ Digital I/O |
| **ADC** | 12 analog input channels | ✅ 12-bit resolution |
| **Fault Detection** | Overcurrent, overtemperature | ✅ Automatic protection |
| **Communication** | SPI/UART interfaces | ✅ Configurable |

## 📊 Examples

### Basic Motor Control

```cpp
void basic_motor_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Get onboard motor controller
    auto* handler = motor.handler(0);
    if (handler && handler->Initialize()) {
        auto tmc = motor.driver(0);
        if (tmc) {
            // Configure motor
            tmc->SetMotorType(tmc9660::MotorType::BLDC);
            tmc->SetMaxCurrent(1000);  // 1A
            tmc->SetTargetVelocity(500);  // 500 RPM
            
            // Enable motor
            tmc->EnableMotor(true);
            
            printf("Motor enabled at 500 RPM\n");
            
            // Run for 5 seconds
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            // Stop motor
            tmc->SetTargetVelocity(0);
            tmc->EnableMotor(false);
            
            printf("Motor stopped\n");
        }
    }
}
```

### Multi-Device Management

```cpp
void multi_device_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Create external device
    if (motor.CreateExternalDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1, 0x01)) {
        printf("External device created at index 2\n");
        
        // Initialize external device
        auto* ext_handler = motor.handler(2);
        if (ext_handler && ext_handler->Initialize()) {
            auto ext_tmc = motor.driver(2);
            if (ext_tmc) {
                // Configure external motor
                ext_tmc->SetMotorType(tmc9660::MotorType::STEPPER);
                ext_tmc->SetMaxCurrent(800);  // 800mA
                ext_tmc->SetTargetVelocity(1000);  // 1000 RPM
                ext_tmc->EnableMotor(true);
                
                printf("External motor enabled\n");
            }
        }
    }
    
    // Control both motors
    auto onboard_tmc = motor.driver(0);
    auto external_tmc = motor.driver(2);
    
    if (onboard_tmc && external_tmc) {
        // Synchronized motor control
        onboard_tmc->SetTargetVelocity(750);
        external_tmc->SetTargetVelocity(750);
        
        printf("Both motors synchronized at 750 RPM\n");
    }
}
```

### Device Information and Status

```cpp
void device_info_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Get device count
    uint8_t device_count = motor.GetDeviceCount();
    printf("Active devices: %u\n", device_count);
    
    // Get active device indices
    auto active_indices = motor.GetActiveDeviceIndices();
    printf("Active device indices: ");
    for (auto index : active_indices) {
        printf("%u ", index);
    }
    printf("\n");
    
    // Check device validity
    for (uint8_t i = 0; i < 4; i++) {
        if (motor.IsDeviceValid(i)) {
            printf("Device %u is valid\n", i);
            
            // Check initialization status
            auto* handler = motor.handler(i);
            if (handler) {
                printf("  Handler available\n");
                
                auto tmc = motor.driver(i);
                if (tmc) {
                    printf("  Driver available\n");
                    
                    // Get motor status
                    bool enabled = tmc->IsMotorEnabled();
                    float velocity = tmc->GetCurrentVelocity();
                    float current = tmc->GetCurrentCurrent();
                    
                    printf("  Motor enabled: %s\n", enabled ? "YES" : "NO");
                    printf("  Current velocity: %.1f RPM\n", velocity);
                    printf("  Current current: %.1f mA\n", current);
                }
            }
        }
    }
}
```

### External Device Management

```cpp
void external_device_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Check if external slots are available
    if (motor.IsExternalSlotAvailable(2)) {
        printf("External slot 2 is available\n");
        
        // Create external device with custom configuration
        tmc9660::BootloaderConfig custom_config{};
        custom_config.motor_type = tmc9660::MotorType::BLDC;
        custom_config.current_limit_ma = 1500;
        custom_config.velocity_limit_rpm = 2000;
        
        if (motor.CreateExternalDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1, 0x02, &custom_config)) {
            printf("External device created with custom config\n");
            
            // Use the external device
            auto* handler = motor.handler(2);
            if (handler && handler->Initialize()) {
                auto tmc = motor.driver(2);
                if (tmc) {
                    tmc->EnableMotor(true);
                    tmc->SetTargetVelocity(1000);
                    printf("External motor running\n");
                }
            }
        }
    }
    
    // Delete external device when done
    if (motor.DeleteExternalDevice(2)) {
        printf("External device deleted\n");
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    // Initialize all devices
    auto init_results = motor.InitializeAllDevices();
    printf("Device initialization results:\n");
    for (size_t i = 0; i < init_results.size(); i++) {
        printf("  Device %zu: %s\n", i, init_results[i] ? "SUCCESS" : "FAILED");
    }
    
    // Get initialization status
    auto status = motor.GetInitializationStatus();
    printf("Device initialization status:\n");
    for (size_t i = 0; i < status.size(); i++) {
        printf("  Device %zu: %s\n", i, status[i] ? "INITIALIZED" : "NOT INITIALIZED");
    }
    
    // Dump comprehensive statistics
    motor.DumpStatistics();
}
```

### Advanced Motor Control

```cpp
void advanced_control_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    auto* handler = motor.handler(0);
    if (handler && handler->Initialize()) {
        auto tmc = motor.driver(0);
        if (tmc) {
            // Configure for BLDC motor
            tmc->SetMotorType(tmc9660::MotorType::BLDC);
            tmc->SetMaxCurrent(1200);  // 1.2A
            tmc->SetVelocityLimit(2000);  // 2000 RPM max
            tmc->SetAccelerationLimit(1000);  // 1000 RPM/s acceleration
            
            // Enable motor
            tmc->EnableMotor(true);
            
            // Velocity control loop
            for (int i = 0; i < 10; i++) {
                float target_velocity = 500.0f + (i * 100.0f);  // Ramp up
                tmc->SetTargetVelocity(target_velocity);
                
                printf("Target velocity: %.1f RPM\n", target_velocity);
                
                // Monitor actual velocity
                for (int j = 0; j < 10; j++) {
                    float actual_velocity = tmc->GetCurrentVelocity();
                    float current = tmc->GetCurrentCurrent();
                    
                    printf("  Actual: %.1f RPM, Current: %.1f mA\n", 
                           actual_velocity, current);
                    
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
            
            // Ramp down
            for (int i = 10; i >= 0; i--) {
                float target_velocity = 500.0f + (i * 100.0f);
                tmc->SetTargetVelocity(target_velocity);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            
            // Stop motor
            tmc->SetTargetVelocity(0);
            tmc->EnableMotor(false);
            
            printf("Motor control complete\n");
        }
    }
}
```

## 🔍 Advanced Usage

### Error Handling

```cpp
void error_handling_example() {
    auto& motor = MotorController::GetInstance();
    
    // Check initialization
    if (!motor.EnsureInitialized()) {
        printf("ERROR: Failed to initialize motor controller\n");
        return;
    }
    
    // Validate device before use
    if (!motor.IsDeviceValid(0)) {
        printf("ERROR: Onboard device not valid\n");
        return;
    }
    
    // Safe device access
    auto* handler = motor.handler(0);
    if (!handler) {
        printf("ERROR: Handler not available\n");
        return;
    }
    
    if (!handler->Initialize()) {
        printf("ERROR: Failed to initialize handler\n");
        return;
    }
    
    auto tmc = motor.driver(0);
    if (!tmc) {
        printf("ERROR: TMC9660 driver not available\n");
        return;
    }
    
    // Safe motor operations
    try {
        tmc->SetTargetVelocity(500);
        tmc->EnableMotor(true);
        printf("Motor operations successful\n");
    } catch (const std::exception& e) {
        printf("ERROR: Motor operation failed: %s\n", e.what());
    }
}
```

### Integration with Other Managers

```cpp
#include "component-handlers/MotorController.h"

void integrated_example() {
    // Initialize all managers
    auto& motor = MotorController::GetInstance();
    auto& gpio = GpioManager::GetInstance();
    auto& adc = AdcManager::GetInstance();
    auto& comm = CommChannelsManager::GetInstance();
    
    motor.EnsureInitialized();
    gpio.EnsureInitialized();
    adc.Initialize();
    comm.EnsureInitialized();
    
    // Get motor controller
    auto* handler = motor.handler(0);
    if (handler && handler->Initialize()) {
        auto tmc = motor.driver(0);
        if (tmc) {
            // Use GPIO for motor enable/disable
            gpio.SetDirection("TMC9660_ENABLE", HF_GPIO_DIRECTION_OUTPUT);
            gpio.SetActive("TMC9660_ENABLE");
            
            // Monitor motor current via ADC
            float motor_current;
            if (adc.ReadChannelV("TMC9660_CURRENT_I0", motor_current) == HF_ADC_SUCCESS) {
                printf("Motor current: %.2fA\n", motor_current);
                
                // Check for overcurrent
                if (motor_current > 1.5f) {
                    printf("Overcurrent detected! Disabling motor\n");
                    gpio.SetInactive("TMC9660_ENABLE");
                    tmc->EnableMotor(false);
                }
            }
            
            // Monitor motor fault via GPIO
            gpio.SetDirection("TMC9660_FAULT", HF_GPIO_DIRECTION_INPUT);
            bool fault_state;
            if (gpio.Read("TMC9660_FAULT", fault_state) == HF_GPIO_SUCCESS) {
                if (!fault_state) {  // Fault is active low
                    printf("Motor fault detected!\n");
                    tmc->EnableMotor(false);
                }
            }
        }
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*