# HardFOC Hardware Abstraction Layer (HAL)

<div align="center">

![Version](https://img.shields.io/badge/version-2.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--C6-green.svg)
![License](https://img.shields.io/badge/license-MIT-yellow.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)

**Advanced hardware abstraction layer for the HardFOC motor controller platform**

[🚀 Quick Start](#quick-start) • [📚 Documentation](#documentation) • [🏗️ Architecture](#architecture) • [🔧 API Reference](#api-reference) • [🧪 Examples](#examples)

</div>

## 🎯 Overview

The HardFOC HAL provides a comprehensive, thread-safe hardware abstraction layer for motor control applications on ESP32-C6. It features unified management of GPIO, ADC, communication interfaces, and motor controllers with support for multiple hardware sources including onboard ESP32-C6, PCAL95555 GPIO expanders, and TMC9660 motor controllers.

### ✨ Key Features

- **🔧 Unified Hardware Management**: Single API for GPIO, ADC, SPI, I2C, UART, and CAN
- **🎛️ Multi-Source Support**: ESP32-C6, PCAL95555, TMC9660 seamlessly integrated
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📊 Advanced Diagnostics**: Real-time health monitoring and error tracking
- **⚡ High Performance**: Optimized batch operations and interrupt handling
- **🛡️ Safety Features**: Pin validation, conflict detection, and fault recovery
- **🔌 Extensible Design**: Easy integration of new hardware components

### 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        HardFOC HAL                             │
├─────────────────────────────────────────────────────────────────┤
│  API Layer           │ Public interfaces & integration        │
├─────────────────────────────────────────────────────────────────┤
│  Component Handlers  │ Managers: GPIO, ADC, Comm, IMU, Motor  │
├─────────────────────────────────────────────────────────────────┤
│  Driver Handlers     │ TMC9660, PCAL95555, AS5047U, BNO08x    │
├─────────────────────────────────────────────────────────────────┤
│  Hardware Drivers    │ ESP32 interfaces & external drivers    │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### 1. Include the HAL

```cpp
#include "API/All.h"  // Complete HAL system
// OR
#include "component-handlers/All.h"  // Component handlers only
```

### 2. Initialize the System

```cpp
#include "API/All.h"

int main() {
    // Initialize the complete HardFOC system
    if (!HARDFOC_INIT()) {
        printf("Failed to initialize HardFOC system\n");
        return -1;
    }
    
    // System is ready for use
    printf("HardFOC HAL initialized successfully\n");
    
    while (true) {
        // Periodic maintenance
        HARDFOC_MAINTAIN();
        
        // Check system health
        if (!HARDFOC_HEALTHY()) {
            printf("System health check failed\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### 3. Use Hardware Resources

```cpp
// GPIO Operations
auto& gpio = GpioManager::GetInstance();
gpio.EnsureInitialized();

// Set output pin
gpio.SetPin("ESP32_GPIO_2", true);

// Read input pin  
bool state = gpio.GetPin("PCAL95555_CHIP1_PIN_0");

// ADC Operations
auto& adc = AdcManager::GetInstance();
adc.Initialize();

// Read raw ADC value
uint16_t raw = adc.ReadRaw("ESP32_ADC1_CH0");

// Read calibrated voltage
float voltage = adc.ReadVoltage("TMC9660_AIN1");

// Motor Control
auto& motor = MotorController::GetInstance();
auto* handler = motor.handler(0);  // Get onboard TMC9660
if (handler && handler->Initialize()) {
    auto tmc = handler->GetTmc9660Driver();
    // Configure and control motor
}
```

## 📚 Documentation

### 📖 Complete Documentation Index
- **[📋 Documentation Index](DOCUMENTATION_INDEX.md)** - Complete guide to all documentation

### 🎯 Core System Documentation
- **[🔧 GPIO & ADC System](docs/HARDFOC_GPIO_ADC_SYSTEM.md)** - Comprehensive system guide
- **[🏗️ Hardware Abstraction Architecture](docs/HARDFOC_HARDWARE_ABSTRACTION_ARCHITECTURE.md)** - Core architecture
- **[⚡ GPIO Architecture](docs/GPIO_ARCHITECTURE_FINAL.md)** - GPIO system design
- **[📊 PWM Architecture](docs/PWM_ARCHITECTURE.md)** - PWM system design

### 📚 Component Handler Documentation
- **[🎛️ GPIO Manager](docs/component-handlers/GPIO_MANAGER_README.md)** - GPIO management system
- **[📊 ADC Manager](docs/component-handlers/ADC_MANAGER_README.md)** - ADC management system
- **[📡 Communication Manager](docs/component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[🧭 IMU Manager](docs/component-handlers/IMU_MANAGER_README.md)** - IMU sensor management
- **[🎛️ Motor Controller](docs/component-handlers/MOTOR_CONTROLLER_README.md)** - Motor control system

### 🔧 Driver Handler Documentation
- **[🎛️ TMC9660 Handler](docs/driver-handlers/TMC9660_HANDLER_README.md)** - Motor controller driver
- **[🔌 PCAL95555 Handler](docs/driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander driver
- **[📐 AS5047U Handler](docs/driver-handlers/AS5047U_HANDLER_README.md)** - Position encoder driver
- **[🧭 BNO08x Handler](docs/driver-handlers/BNO08X_HANDLER_README.md)** - IMU sensor driver

### 🔍 API Documentation
- **[🔌 Public API Reference](docs/api/API_REFERENCE.md)** - Complete API documentation
- **[🚀 Integration Guide](docs/api/INTEGRATION_GUIDE.md)** - System integration examples
- **[⚙️ System Initialization](docs/api/SYSTEM_INITIALIZATION.md)** - Initialization procedures

## 🏗️ Architecture

### Component Handlers (Managers)
High-level singleton managers providing unified interfaces:

| Manager | Purpose | Hardware Sources |
|---------|---------|------------------|
| **GpioManager** | GPIO pin management | ESP32-C6, PCAL95555, TMC9660 |
| **AdcManager** | ADC channel management | ESP32-C6, TMC9660 |
| **CommChannelsManager** | Communication interfaces | ESP32-C6 SPI/I2C/UART/CAN |
| **ImuManager** | IMU sensor management | BNO08x via I2C |
| **MotorController** | Motor controller management | TMC9660 via SPI |

### Driver Handlers
Hardware-specific drivers providing device interfaces:

| Handler | Device | Interface | Purpose |
|---------|--------|-----------|---------|
| **Tmc9660Handler** | TMC9660 | SPI/UART | Motor controller with GPIO/ADC |
| **Pcal95555Handler** | PCAL95555 | I2C | 16-bit GPIO expander |
| **As5047uHandler** | AS5047U | SPI | Magnetic position encoder |
| **Bno08xHandler** | BNO08x | I2C | 9-axis IMU sensor |

### Hardware Support Matrix

| Feature | ESP32-C6 | PCAL95555 | TMC9660 | AS5047U | BNO08x |
|---------|-----------|-----------|---------|---------|--------|
| **GPIO** | ✅ 40+ pins | ✅ 32 pins | ✅ 8 pins | ❌ | ❌ |
| **ADC** | ✅ 6 channels | ❌ | ✅ 3 channels | ❌ | ❌ |
| **SPI** | ✅ Master | ❌ | ✅ Slave | ✅ Slave | ❌ |
| **I2C** | ✅ Master | ✅ Slave | ❌ | ❌ | ✅ Slave |
| **UART** | ✅ 3 ports | ❌ | ✅ TMCL | ❌ | ❌ |
| **PWM** | ✅ 6 channels | ❌ | ✅ Motor PWM | ❌ | ❌ |
| **Interrupts** | ✅ GPIO | ✅ GPIO | ✅ Fault | ❌ | ✅ Data ready |

## 🔧 API Reference

### Core Initialization
```cpp
// System initialization
bool HARDFOC_INIT();
void HARDFOC_MAINTAIN();
bool HARDFOC_HEALTHY();

// Manager access
GpioManager& GpioManager::GetInstance();
AdcManager& AdcManager::GetInstance();
CommChannelsManager& CommChannelsManager::GetInstance();
MotorController& MotorController::GetInstance();
```

### GPIO Operations
```cpp
// Pin operations
bool SetPin(std::string_view pin_name, bool state);
bool GetPin(std::string_view pin_name);
bool TogglePin(std::string_view pin_name);

// Configuration
bool ConfigurePin(std::string_view pin_name, bool is_input, bool pull_up = false);
bool SetInterrupt(std::string_view pin_name, GpioInterruptCallback callback);
```

### ADC Operations
```cpp
// Single channel reads
uint16_t ReadRaw(std::string_view channel_name);
float ReadVoltage(std::string_view channel_name);

// Multi-channel reads
AdcMultiReading<8> ReadMultipleRaw(const std::array<std::string_view, 8>& channels);
AdcMultiReading<8> ReadMultipleVoltages(const std::array<std::string_view, 8>& channels);
```

## 🧪 Examples

### Basic GPIO Control
```cpp
#include "component-handlers/All.h"

void gpio_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure LED pin as output
    gpio.ConfigurePin("ESP32_GPIO_2", false);  // false = output
    
    // Blink LED
    for (int i = 0; i < 10; i++) {
        gpio.SetPin("ESP32_GPIO_2", true);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio.SetPin("ESP32_GPIO_2", false);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### Multi-Channel ADC Reading
```cpp
#include "component-handlers/All.h"

void adc_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Read multiple channels simultaneously
    std::array<std::string_view, 4> channels = {
        "ESP32_ADC1_CH0",
        "ESP32_ADC1_CH1", 
        "TMC9660_AIN1",
        "TMC9660_AIN2"
    };
    
    auto readings = adc.ReadMultipleVoltages(channels);
    
    for (size_t i = 0; i < channels.size(); i++) {
        printf("%s: %.3fV\n", channels[i].data(), readings.voltages[i]);
    }
}
```

### Motor Control
```cpp
#include "component-handlers/All.h"

void motor_example() {
    auto& motor = MotorController::GetInstance();
    motor.EnsureInitialized();
    
    auto* handler = motor.handler(0);  // Onboard TMC9660
    if (handler && handler->Initialize()) {
        auto tmc = handler->GetTmc9660Driver();
        
        // Configure motor parameters
        tmc->SetTargetVelocity(1000);  // RPM
        tmc->SetMaxCurrent(1000);      // mA
        
        // Enable motor
        tmc->EnableMotor(true);
        
        printf("Motor controller initialized and running\n");
    }
}
```

## 🔗 Hardware Integration

### Pin Mappings
- **[📍 ESP32-C6 Pin Configuration](docs/hardware/ESP32_C6_PIN_MAPPING.md)**
- **[🔌 PCAL95555 GPIO Expander](docs/hardware/PCAL95555_CONFIGURATION.md)**
- **[🎛️ TMC9660 Motor Controller](docs/hardware/TMC9660_CONFIGURATION.md)**

### Board Configuration
- **[🏗️ HardFOC Vortex V1 Board](docs/hardware/HARDFOC_VORTEX_V1.md)**
- **[⚡ Power Management](docs/hardware/POWER_MANAGEMENT.md)**
- **[🔧 Hardware Setup Guide](docs/hardware/HARDWARE_SETUP.md)**

## 🧪 Testing

### Test Suite
```bash
# Build and run tests
cd tests
idf.py build
idf.py flash monitor
```

### Test Coverage
- **[🧪 Test Documentation](tests/README.md)** - Complete test suite guide
- **Component Handler Tests** - Manager class validation
- **Driver Handler Tests** - Hardware driver validation  
- **Integration Tests** - System-level testing
- **Hardware-in-Loop Tests** - Real hardware validation

## 🤝 Contributing

### Development Guidelines
- **[📝 Coding Standards](docs/development/CODING_STANDARDS.md)**
- **[🏗️ Architecture Guidelines](docs/development/ARCHITECTURE_GUIDELINES.md)**
- **[🧪 Testing Requirements](docs/development/TESTING_REQUIREMENTS.md)**
- **[📚 Documentation Standards](docs/development/DOCUMENTATION_STANDARDS.md)**

### Getting Started
1. Fork the repository
2. Create a feature branch
3. Follow coding standards
4. Add comprehensive tests
5. Update documentation
6. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **📚 Documentation**: Start with [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)
- **🐛 Issues**: Report bugs via GitHub Issues
- **💬 Discussions**: Use GitHub Discussions for questions
- **📧 Contact**: HardFOC Team

---

<div align="center">

**Built with ❤️ by the HardFOC Team**

[⭐ Star us on GitHub](https://github.com/hardfoc/hf-hal) • [🐛 Report Bug](https://github.com/hardfoc/hf-hal/issues) • [💡 Request Feature](https://github.com/hardfoc/hf-hal/issues)

</div>
