# GpioManager - GPIO Management System

<div align="center">

![Component](https://img.shields.io/badge/component-GpioManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-ESP32--C6%20|%20PCAL95555%20|%20TMC9660-orange.svg)

**Advanced GPIO management system for the HardFOC platform**

</div>

## 📋 Overview

The `GpioManager` is a singleton component handler that provides unified, thread-safe access to GPIO pins across multiple hardware sources. It abstracts the complexity of managing GPIOs from ESP32-C6, PCAL95555 GPIO expanders, and TMC9660 motor controllers through a single, consistent API.

### ✨ Key Features

- **🔗 Multi-Source GPIO Management**: ESP32-C6, PCAL95555, TMC9660
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📍 String-Based Pin Identification**: Flexible, extensible pin naming
- **🛡️ Platform Mapping Integration**: Automatic hardware discovery
- **📊 Advanced Diagnostics**: Real-time health monitoring
- **⚡ Batch Operations**: Optimized multi-pin operations
- **🔔 Interrupt Support**: Edge-triggered callbacks
- **🏥 Health Monitoring**: Per-chip and per-pin statistics

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        GpioManager                             │
├─────────────────────────────────────────────────────────────────┤
│  String-Based API    │ pin_name → hardware mapping            │
├─────────────────────────────────────────────────────────────────┤
│  Platform Integration│ Automatic pin discovery & registration │
├─────────────────────────────────────────────────────────────────┤
│  Hardware Handlers   │ ESP32, PCAL95555, TMC9660 handlers     │
├─────────────────────────────────────────────────────────────────┤
│  BaseGpio Interface  │ Unified GPIO operations                 │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/GpioManager.h"

void gpio_example() {
    // Get singleton instance
    auto& gpio = GpioManager::GetInstance();
    
    // Initialize the manager
    if (!gpio.EnsureInitialized()) {
        printf("Failed to initialize GPIO manager\n");
        return;
    }
    
    // Configure pin as output
    gpio.ConfigurePin("ESP32_GPIO_2", false);  // false = output
    
    // Set pin high
    gpio.SetPin("ESP32_GPIO_2", true);
    
    // Read pin state
    bool state = gpio.GetPin("ESP32_GPIO_2");
    printf("Pin state: %s\n", state ? "HIGH" : "LOW");
}
```

## 📖 API Reference

### Core Operations

#### Initialization
```cpp
class GpioManager {
public:
    // Singleton access
    static GpioManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool IsInitialized() const noexcept;
    bool Initialize() noexcept;
    void Deinitialize() noexcept;
};
```

#### Pin Operations
```cpp
// Basic pin control
bool SetPin(std::string_view pin_name, bool state) noexcept;
bool GetPin(std::string_view pin_name) noexcept;
bool TogglePin(std::string_view pin_name) noexcept;

// Pin configuration
bool ConfigurePin(std::string_view pin_name, bool is_input, bool pull_up = false) noexcept;
bool IsPinConfigured(std::string_view pin_name) const noexcept;
```

#### Batch Operations
```cpp
// Multi-pin operations
template<size_t N>
bool SetMultiplePins(const std::array<std::string_view, N>& pin_names,
                     const std::array<bool, N>& states) noexcept;

template<size_t N>
std::array<bool, N> GetMultiplePins(const std::array<std::string_view, N>& pin_names) noexcept;
```

#### Interrupt Management
```cpp
// Interrupt configuration
using GpioInterruptCallback = std::function<void(std::string_view pin_name, bool state)>;

bool SetInterrupt(std::string_view pin_name, 
                  GpioInterruptCallback callback,
                  GpioInterruptType type = GpioInterruptType::RISING_EDGE) noexcept;
bool RemoveInterrupt(std::string_view pin_name) noexcept;
```

### Diagnostics and Monitoring

#### System Status
```cpp
// Health monitoring
struct GpioSystemStatus {
    bool overall_healthy;
    uint32_t total_pins_registered;
    uint32_t esp32_pins_active;
    uint32_t pcal95555_pins_active;
    uint32_t tmc9660_pins_active;
    uint32_t total_errors;
    uint64_t last_error_time;
};

GpioSystemStatus GetSystemStatus() const noexcept;
bool IsSystemHealthy() const noexcept;
```

#### Pin Information
```cpp
// Pin details
struct GpioInfo {
    std::string_view name;
    std::shared_ptr<BaseGpio> gpio_driver;
    HfFunctionalGpioPin functional_pin;
    HfPinCategory category;
    HfGpioChipType hardware_chip;
    uint8_t hardware_pin_id;
    bool is_registered;
    bool is_input;
    bool current_state;
    uint32_t access_count;
    uint32_t error_count;
    uint64_t last_access_time;
};

std::optional<GpioInfo> GetPinInfo(std::string_view pin_name) const noexcept;
std::vector<std::string> GetRegisteredPins() const noexcept;
```

## 🎯 Hardware Support

### Supported Hardware Sources

| Hardware | Pins Available | Features |
|----------|----------------|----------|
| **ESP32-C6** | 40+ GPIO pins | Native GPIO, interrupts, pull-up/down |
| **PCAL95555** | 32 GPIO pins (2×16) | I2C expander, interrupt support |
| **TMC9660** | 8 GPIO pins | Motor controller GPIOs, fault monitoring |

### Pin Naming Convention

```cpp
// ESP32-C6 pins
"ESP32_GPIO_0" to "ESP32_GPIO_21"    // Standard GPIO pins
"ESP32_GPIO_BOOT"                    // Boot button
"ESP32_GPIO_EN"                      // Enable pin

// PCAL95555 pins (up to 2 chips)
"PCAL95555_CHIP1_PIN_0" to "PCAL95555_CHIP1_PIN_15"  // First chip
"PCAL95555_CHIP2_PIN_0" to "PCAL95555_CHIP2_PIN_15"  // Second chip

// TMC9660 pins
"TMC9660_GPIO_0" to "TMC9660_GPIO_7"     // Motor controller GPIOs
"TMC9660_FAULT"                          // Fault indication pin
"TMC9660_ENABLE"                         // Motor enable pin
```

## 🔧 Configuration

### Platform Integration

The GpioManager automatically integrates with the platform mapping system:

```cpp
// Platform mapping integration
#include "hf_functional_pin_config_vortex_v1.hpp"

// Automatic pin discovery based on platform configuration
// Pins are registered automatically during initialization
```

### Hardware Handler Configuration

```cpp
// ESP32-C6 GPIO configuration
struct EspGpioConfig {
    bool enable_pullup = false;
    bool enable_pulldown = false;
    bool enable_interrupts = true;
    gpio_drive_cap_t drive_strength = GPIO_DRIVE_CAP_DEFAULT;
};

// PCAL95555 configuration
struct Pcal95555Config {
    uint8_t device_address = 0x20;
    bool enable_interrupts = true;
    uint32_t interrupt_pin = GPIO_NUM_NC;
};

// TMC9660 configuration  
struct Tmc9660GpioConfig {
    bool enable_fault_monitoring = true;
    bool enable_gpio_interrupts = true;
};
```

## 📊 Examples

### Basic GPIO Control

```cpp
#include "component-handlers/GpioManager.h"

void basic_gpio_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure pins
    gpio.ConfigurePin("ESP32_GPIO_2", false);      // LED output
    gpio.ConfigurePin("ESP32_GPIO_9", true, true); // Button input with pullup
    
    // Control LED based on button
    while (true) {
        bool button_pressed = !gpio.GetPin("ESP32_GPIO_9");  // Active low
        gpio.SetPin("ESP32_GPIO_2", button_pressed);
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### Multi-Pin Operations

```cpp
void multi_pin_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure multiple LEDs
    std::array<std::string_view, 4> led_pins = {
        "ESP32_GPIO_2", "ESP32_GPIO_3", "ESP32_GPIO_4", "ESP32_GPIO_5"
    };
    
    for (const auto& pin : led_pins) {
        gpio.ConfigurePin(pin, false);  // Configure as outputs
    }
    
    // Create LED patterns
    std::array<bool, 4> pattern1 = {true, false, true, false};
    std::array<bool, 4> pattern2 = {false, true, false, true};
    
    // Alternate patterns
    for (int i = 0; i < 10; i++) {
        gpio.SetMultiplePins(led_pins, pattern1);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        gpio.SetMultiplePins(led_pins, pattern2);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### GPIO Interrupts

```cpp
void interrupt_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure interrupt pin
    gpio.ConfigurePin("ESP32_GPIO_9", true, true);  // Input with pullup
    
    // Set interrupt callback
    auto callback = [](std::string_view pin_name, bool state) {
        printf("Interrupt on %.*s: %s\n", 
               static_cast<int>(pin_name.size()), pin_name.data(),
               state ? "HIGH" : "LOW");
    };
    
    gpio.SetInterrupt("ESP32_GPIO_9", callback, GpioInterruptType::BOTH_EDGES);
    
    printf("Interrupt configured. Press button to trigger...\n");
    
    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### GPIO Expander Usage

```cpp
void expander_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Use PCAL95555 GPIO expander pins
    gpio.ConfigurePin("PCAL95555_CHIP1_PIN_0", false);  // Output
    gpio.ConfigurePin("PCAL95555_CHIP1_PIN_1", true);   // Input
    
    // Control expander pin
    gpio.SetPin("PCAL95555_CHIP1_PIN_0", true);
    
    // Read expander pin
    bool state = gpio.GetPin("PCAL95555_CHIP1_PIN_1");
    printf("Expander pin state: %s\n", state ? "HIGH" : "LOW");
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Get system status
    auto status = gpio.GetSystemStatus();
    
    printf("GPIO System Status:\n");
    printf("  Overall healthy: %s\n", status.overall_healthy ? "YES" : "NO");
    printf("  Total pins: %u\n", status.total_pins_registered);
    printf("  ESP32 pins: %u\n", status.esp32_pins_active);
    printf("  PCAL95555 pins: %u\n", status.pcal95555_pins_active);
    printf("  TMC9660 pins: %u\n", status.tmc9660_pins_active);
    printf("  Total errors: %u\n", status.total_errors);
    
    // Get information about specific pin
    auto pin_info = gpio.GetPinInfo("ESP32_GPIO_2");
    if (pin_info) {
        printf("\nPin ESP32_GPIO_2 info:\n");
        printf("  Hardware chip: %d\n", static_cast<int>(pin_info->hardware_chip));
        printf("  Hardware pin ID: %u\n", pin_info->hardware_pin_id);
        printf("  Is input: %s\n", pin_info->is_input ? "YES" : "NO");
        printf("  Current state: %s\n", pin_info->current_state ? "HIGH" : "LOW");
        printf("  Access count: %u\n", pin_info->access_count);
        printf("  Error count: %u\n", pin_info->error_count);
    }
    
    // List all registered pins
    auto pins = gpio.GetRegisteredPins();
    printf("\nRegistered pins (%zu total):\n", pins.size());
    for (const auto& pin : pins) {
        printf("  - %s\n", pin.c_str());
    }
}
```

## 🔍 Advanced Usage

### Custom Hardware Integration

```cpp
class CustomGpioHandler {
public:
    static bool RegisterCustomPin(const std::string& pin_name, 
                                  std::shared_ptr<BaseGpio> gpio_driver) {
        auto& manager = GpioManager::GetInstance();
        
        // Register custom GPIO with the manager
        // Implementation depends on internal GPIO registration API
        return true;
    }
};
```

### Performance Optimization

```cpp
void optimized_gpio_usage() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Use batch operations for better performance
    std::array<std::string_view, 8> pins = {
        "ESP32_GPIO_2", "ESP32_GPIO_3", "ESP32_GPIO_4", "ESP32_GPIO_5",
        "PCAL95555_CHIP1_PIN_0", "PCAL95555_CHIP1_PIN_1", 
        "PCAL95555_CHIP1_PIN_2", "PCAL95555_CHIP1_PIN_3"
    };
    
    std::array<bool, 8> states = {true, false, true, false, true, false, true, false};
    
    // Single call sets all pins efficiently
    gpio.SetMultiplePins(pins, states);
    
    // Single call reads all pins efficiently  
    auto readings = gpio.GetMultiplePins(pins);
}
```

## 🚨 Error Handling

### Common Error Scenarios

```cpp
void error_handling_example() {
    auto& gpio = GpioManager::GetInstance();
    
    // Check initialization
    if (!gpio.EnsureInitialized()) {
        printf("ERROR: Failed to initialize GPIO manager\n");
        return;
    }
    
    // Validate pin exists before use
    if (!gpio.IsPinConfigured("ESP32_GPIO_2")) {
        printf("ERROR: Pin ESP32_GPIO_2 not configured\n");
        return;
    }
    
    // Safe pin operations with error checking
    if (!gpio.SetPin("ESP32_GPIO_2", true)) {
        printf("ERROR: Failed to set pin ESP32_GPIO_2\n");
    }
    
    // Monitor system health
    if (!gpio.IsSystemHealthy()) {
        printf("WARNING: GPIO system health check failed\n");
        auto status = gpio.GetSystemStatus();
        printf("Total errors: %u\n", status.total_errors);
    }
}
```

## 🔗 Integration

### With Other Managers

```cpp
#include "component-handlers/All.h"

void integrated_example() {
    // Initialize all managers
    auto& gpio = GpioManager::GetInstance();
    auto& adc = AdcManager::GetInstance();
    auto& motor = MotorController::GetInstance();
    
    gpio.EnsureInitialized();
    adc.Initialize();
    motor.EnsureInitialized();
    
    // Use GPIO to control motor enable
    gpio.ConfigurePin("TMC9660_ENABLE", false);
    gpio.SetPin("TMC9660_ENABLE", true);
    
    // Monitor motor fault via GPIO
    gpio.ConfigurePin("TMC9660_FAULT", true);
    auto fault_callback = [&motor](std::string_view pin, bool state) {
        if (!state) {  // Fault is active low
            printf("Motor fault detected!\n");
            // Handle fault condition
        }
    };
    gpio.SetInterrupt("TMC9660_FAULT", fault_callback, GpioInterruptType::FALLING_EDGE);
}
```

## 📚 See Also

- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver
- **[PCAL95555 Handler Documentation](../driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander driver
- **[Complete GPIO System Guide](../HARDFOC_GPIO_ADC_SYSTEM.md)** - Comprehensive system documentation

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*