# GpioManager - Advanced GPIO Management System

<div align="center">

![Component](https://img.shields.io/badge/component-GpioManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-ESP32--C6%20|%20PCAL95555%20|%20TMC9660-orange.svg)

**Comprehensive GPIO management system for the HardFOC platform**

</div>

## 📋 Overview

The `GpioManager` is a singleton component handler that provides unified, thread-safe access to GPIO pins across multiple hardware sources. It integrates with the platform mapping system to automatically manage GPIOs from ESP32-C6, PCAL95555 GPIO expanders, and TMC9660 motor controllers through a single, consistent API using string-based pin identification.

### ✨ Key Features

- **🔗 Multi-Source GPIO Management**: ESP32-C6, PCAL95555, TMC9660
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📍 String-Based Pin Identification**: Flexible, extensible pin naming
- **🛡️ Platform Mapping Integration**: Automatic hardware discovery
- **📊 Advanced Diagnostics**: Real-time health monitoring
- **⚡ Batch Operations**: Optimized multi-pin operations
- **🔔 Complete Interrupt Support**: Edge-triggered callbacks
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
    gpio.SetDirection("ESP32_GPIO_2", HF_GPIO_DIRECTION_OUTPUT);
    
    // Set pin active
    gpio.SetActive("ESP32_GPIO_2");
    
    // Read pin state
    bool state;
    if (gpio.Read("ESP32_GPIO_2", state) == HF_GPIO_SUCCESS) {
        printf("Pin state: %s\n", state ? "ACTIVE" : "INACTIVE");
    }
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
    bool Shutdown() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### GPIO Registration and Management
```cpp
// Pin registration
hf_gpio_err_t RegisterGpio(std::string_view name, std::shared_ptr<BaseGpio> gpio) noexcept;
std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;
bool Contains(std::string_view name) const noexcept;
size_t Size() const noexcept;
```

#### Basic GPIO Operations
```cpp
// Pin control
hf_gpio_err_t Set(std::string_view name, bool value) noexcept;
hf_gpio_err_t SetActive(std::string_view name) noexcept;
hf_gpio_err_t SetInactive(std::string_view name) noexcept;
hf_gpio_err_t Read(std::string_view name, bool& state) noexcept;
hf_gpio_err_t Toggle(std::string_view name) noexcept;
hf_gpio_err_t IsActive(std::string_view name, bool& active) noexcept;
```

#### Pin Configuration
```cpp
// Pin configuration
hf_gpio_err_t SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept;
hf_gpio_err_t SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept;
hf_gpio_err_t SetOutputMode(std::string_view name, hf_gpio_output_mode_t output_mode) noexcept;

// Configuration queries
hf_gpio_err_t GetDirection(std::string_view name, hf_gpio_direction_t& direction) const noexcept;
hf_gpio_err_t GetPullMode(std::string_view name, hf_gpio_pull_mode_t& pull_mode) const noexcept;
hf_gpio_err_t GetOutputMode(std::string_view name, hf_gpio_output_mode_t& output_mode) const noexcept;
```

#### Interrupt Support
```cpp
// Interrupt configuration
hf_gpio_err_t ConfigureInterrupt(std::string_view name,
                                 hf_gpio_interrupt_trigger_t trigger,
                                 BaseGpio::InterruptCallback callback,
                                 void* user_data = nullptr) noexcept;
hf_gpio_err_t EnableInterrupt(std::string_view name) noexcept;
hf_gpio_err_t DisableInterrupt(std::string_view name) noexcept;
bool SupportsInterrupts(std::string_view name) const noexcept;
```

#### Batch Operations
```cpp
// Multi-pin operations
GpioBatchResult BatchWrite(const GpioBatchOperation& operation) noexcept;
GpioBatchResult BatchRead(const std::vector<std::string_view>& pin_names) noexcept;
GpioBatchResult SetMultipleActive(const std::vector<std::string_view>& pin_names) noexcept;
GpioBatchResult SetMultipleInactive(const std::vector<std::string_view>& pin_names) noexcept;
```

#### Statistics and Diagnostics
```cpp
// System diagnostics
hf_gpio_err_t GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
hf_gpio_err_t GetStatistics(std::string_view name, BaseGpio::PinStatistics& statistics) const noexcept;
hf_gpio_err_t ResetStatistics(std::string_view name) noexcept;
void DumpStatistics() const noexcept;
```

### Data Structures

#### GpioInfo
```cpp
struct GpioInfo {
    std::string_view name;                      // Human-readable name
    std::shared_ptr<BaseGpio> gpio_driver;      // GPIO driver instance
    HfFunctionalGpioPin functional_pin;         // Functional pin identifier
    HfPinCategory category;                     // Pin category (CORE, COMM, GPIO, USER)
    HfGpioChipType hardware_chip;               // Hardware chip identifier
    uint8_t hardware_pin_id;                    // Hardware pin ID within the chip
    bool is_registered;                         // Registration status
    bool is_input;                              // Pin direction
    bool current_state;                         // Last known pin state
    uint32_t access_count;                      // Number of times accessed
    uint32_t error_count;                       // Number of errors encountered
    uint64_t last_access_time;                  // Timestamp of last access
};
```

#### GpioSystemDiagnostics
```cpp
struct GpioSystemDiagnostics {
    bool system_healthy;                           // Overall system health
    uint32_t total_pins_registered;                // Total pins registered
    uint32_t pins_by_chip[4];                      // Pins per chip
    uint32_t pins_by_category[4];                  // Pins per category
    uint32_t total_operations;                     // Total operations performed
    uint32_t successful_operations;                // Successful operations
    uint32_t failed_operations;                    // Failed operations
    uint32_t communication_errors;                 // Communication errors
    uint32_t hardware_errors;                      // Hardware errors
    uint64_t system_uptime_ms;                     // System uptime
    hf_gpio_err_t last_error;                      // Last error encountered
};
```

#### GpioBatchOperation
```cpp
struct GpioBatchOperation {
    std::vector<std::string_view> pin_names;    // Pin names to operate on
    std::vector<bool> states;                   // Desired states (for write operations)
    bool is_write_operation;                    // true for write, false for read
};
```

#### GpioBatchResult
```cpp
struct GpioBatchResult {
    std::vector<std::string_view> pin_names;    // Pin names operated on
    std::vector<bool> states;                   // Resulting states
    std::vector<hf_gpio_err_t> results;         // Individual operation results
    hf_gpio_err_t overall_result;               // Overall operation result
    
    bool AllSuccessful() const noexcept;        // Check if all operations were successful
};
```

## 🎯 Hardware Support

### Supported Hardware Sources

| Hardware | Pins Available | Features |
|----------|----------------|----------|
| **ESP32-C6** | 40+ GPIO pins | Native GPIO, interrupts, pull-up/down |
| **PCAL95555** | 32 GPIO pins (2×16) | I2C expander, interrupt support |
| **TMC9660** | 18 GPIO pins | Motor controller GPIOs, fault monitoring |

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
"TMC9660_GPIO_0" to "TMC9660_GPIO_17"    // Motor controller GPIOs
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

### Error Handling

The GpioManager uses comprehensive error handling with specific error codes:

```cpp
// Common error codes
hf_gpio_err_t::HF_GPIO_SUCCESS              // Operation successful
hf_gpio_err_t::HF_GPIO_ERR_INVALID_PARAMETER // Invalid pin name or parameters
hf_gpio_err_t::HF_GPIO_ERR_NOT_INITIALIZED   // GPIO manager not initialized
hf_gpio_err_t::HF_GPIO_ERR_HARDWARE_FAULT    // Hardware communication error
hf_gpio_err_t::HF_GPIO_ERR_PERMISSION_DENIED // Pin access denied
hf_gpio_err_t::HF_GPIO_ERR_OUT_OF_MEMORY     // Memory allocation failed
```

## 📊 Examples

### Basic GPIO Control

```cpp
#include "component-handlers/GpioManager.h"

void basic_gpio_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure pins
    gpio.SetDirection("ESP32_GPIO_2", HF_GPIO_DIRECTION_OUTPUT);
    gpio.SetDirection("ESP32_GPIO_9", HF_GPIO_DIRECTION_INPUT);
    gpio.SetPullMode("ESP32_GPIO_9", HF_GPIO_PULL_MODE_UP);
    
    // Control LED based on button
    while (true) {
        bool button_state;
        if (gpio.Read("ESP32_GPIO_9", button_state) == HF_GPIO_SUCCESS) {
            gpio.Set("ESP32_GPIO_2", !button_state);  // Active low button
        }
        
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
    std::vector<std::string_view> led_pins = {
        "ESP32_GPIO_2", "ESP32_GPIO_3", "ESP32_GPIO_4", "ESP32_GPIO_5"
    };
    
    for (const auto& pin : led_pins) {
        gpio.SetDirection(pin, HF_GPIO_DIRECTION_OUTPUT);
    }
    
    // Create LED patterns
    std::vector<bool> pattern1 = {true, false, true, false};
    std::vector<bool> pattern2 = {false, true, false, true};
    
    // Alternate patterns using batch operations
    for (int i = 0; i < 10; i++) {
        GpioBatchOperation op1(led_pins, pattern1);
        auto result1 = gpio.BatchWrite(op1);
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        GpioBatchOperation op2(led_pins, pattern2);
        auto result2 = gpio.BatchWrite(op2);
        
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
    gpio.SetDirection("ESP32_GPIO_9", HF_GPIO_DIRECTION_INPUT);
    gpio.SetPullMode("ESP32_GPIO_9", HF_GPIO_PULL_MODE_UP);
    
    // Set interrupt callback
    auto callback = [](std::string_view pin_name, bool state, void* user_data) {
        printf("Interrupt on %.*s: %s\n", 
               static_cast<int>(pin_name.size()), pin_name.data(),
               state ? "ACTIVE" : "INACTIVE");
    };
    
    gpio.ConfigureInterrupt("ESP32_GPIO_9", HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES, callback);
    gpio.EnableInterrupt("ESP32_GPIO_9");
    
    printf("Interrupt configured. Press button to trigger...\n");
    
    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Get system status
    GpioSystemDiagnostics diagnostics;
    if (gpio.GetSystemDiagnostics(diagnostics) == HF_GPIO_SUCCESS) {
        printf("GPIO System Status:\n");
        printf("  Overall healthy: %s\n", diagnostics.system_healthy ? "YES" : "NO");
        printf("  Total pins: %u\n", diagnostics.total_pins_registered);
        printf("  Total operations: %u\n", diagnostics.total_operations);
        printf("  Successful operations: %u\n", diagnostics.successful_operations);
        printf("  Failed operations: %u\n", diagnostics.failed_operations);
        printf("  Communication errors: %u\n", diagnostics.communication_errors);
        printf("  Hardware errors: %u\n", diagnostics.hardware_errors);
        printf("  System uptime: %llu ms\n", diagnostics.system_uptime_ms);
    }
    
    // Get pin statistics
    BaseGpio::PinStatistics stats;
    if (gpio.GetStatistics("ESP32_GPIO_2", stats) == HF_GPIO_SUCCESS) {
        printf("\nPin ESP32_GPIO_2 statistics:\n");
        printf("  Access count: %u\n", stats.access_count);
        printf("  Error count: %u\n", stats.error_count);
        printf("  Last access time: %llu\n", stats.last_access_time);
    }
    
    // Dump all statistics
    gpio.DumpStatistics();
}
```

## 🔍 Advanced Usage

### Performance Optimization

```cpp
void optimized_gpio_usage() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Use batch operations for better performance
    std::vector<std::string_view> pins = {
        "ESP32_GPIO_2", "ESP32_GPIO_3", "ESP32_GPIO_4", "ESP32_GPIO_5",
        "PCAL95555_CHIP1_PIN_0", "PCAL95555_CHIP1_PIN_1", 
        "PCAL95555_CHIP1_PIN_2", "PCAL95555_CHIP1_PIN_3"
    };
    
    std::vector<bool> states = {true, false, true, false, true, false, true, false};
    
    // Single call sets all pins efficiently
    GpioBatchOperation op(pins, states);
    auto result = gpio.BatchWrite(op);
    
    if (result.AllSuccessful()) {
        printf("All pins set successfully\n");
    } else {
        printf("Some pin operations failed\n");
    }
    
    // Single call reads all pins efficiently  
    auto read_result = gpio.BatchRead(pins);
    for (size_t i = 0; i < pins.size(); i++) {
        if (read_result.results[i] == HF_GPIO_SUCCESS) {
            printf("Pin %.*s: %s\n", 
                   static_cast<int>(pins[i].size()), pins[i].data(),
                   read_result.states[i] ? "ACTIVE" : "INACTIVE");
        }
    }
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
    if (!gpio.Contains("ESP32_GPIO_2")) {
        printf("ERROR: Pin ESP32_GPIO_2 not registered\n");
        return;
    }
    
    // Safe pin operations with error checking
    auto result = gpio.SetActive("ESP32_GPIO_2");
    if (result != HF_GPIO_SUCCESS) {
        printf("ERROR: Failed to set pin ESP32_GPIO_2: %d\n", static_cast<int>(result));
    }
    
    // Monitor system health
    GpioSystemDiagnostics diagnostics;
    if (gpio.GetSystemDiagnostics(diagnostics) == HF_GPIO_SUCCESS) {
        if (!diagnostics.system_healthy) {
            printf("WARNING: GPIO system health check failed\n");
            printf("Total errors: %u\n", diagnostics.failed_operations);
        }
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
    gpio.SetDirection("TMC9660_ENABLE", HF_GPIO_DIRECTION_OUTPUT);
    gpio.SetActive("TMC9660_ENABLE");
    
    // Monitor motor fault via GPIO
    gpio.SetDirection("TMC9660_FAULT", HF_GPIO_DIRECTION_INPUT);
    auto fault_callback = [&motor](std::string_view pin, bool state, void* user_data) {
        if (!state) {  // Fault is active low
            printf("Motor fault detected!\n");
            // Handle fault condition
        }
    };
    gpio.ConfigureInterrupt("TMC9660_FAULT", HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE, fault_callback);
    gpio.EnableInterrupt("TMC9660_FAULT");
}
```

## 📚 See Also

- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver
- **[PCAL95555 Handler Documentation](../driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander driver

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*