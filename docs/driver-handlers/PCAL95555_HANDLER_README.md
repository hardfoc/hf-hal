# Pcal95555Handler - GPIO Expander Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Pcal95555Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-PCAL95555-orange.svg)
![Interface](https://img.shields.io/badge/interface-I2C-green.svg)

**Minimal, platform-agnostic handler for PCAL95555 16-bit I2C GPIO expander**

</div>

## 📋 Overview

The `Pcal95555Handler` is a focused, platform-agnostic handler for the PCAL95555 16-bit I2C GPIO expander. It provides direct pin control operations by pin number (0-15) with no platform mapping or diagnostics, offering a minimal, modern C++ interface for GPIO expansion.

### ✨ Key Features

- **🔌 16-Bit GPIO Expansion**: Full 16-pin GPIO expansion per chip
- **📡 I2C Interface**: Flexible I2C communication with configurable addressing
- **🔔 Interrupt Support**: Hardware interrupt capabilities with edge detection
- **⚡ Direct Pin Control**: Pin-level operations without platform abstractions
- **🔧 BaseGpio Compatible**: Drop-in replacement for standard GPIO interfaces
- **🛡️ Thread-Safe**: Concurrent access from multiple tasks
- **📊 Minimal Design**: Focused functionality without unnecessary features
- **🏥 Error Handling**: Comprehensive error detection and reporting

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   Pcal95555Handler                             │
├─────────────────────────────────────────────────────────────────┤
│  Direct Pin Control │ Pin-level operations (0-15)              │
├─────────────────────────────────────────────────────────────────┤
│  I2C Communication  │ Efficient register access via adapter    │
├─────────────────────────────────────────────────────────────────┤
│  Interrupt System   │ Hardware interrupt handling              │
├─────────────────────────────────────────────────────────────────┤
│  PCAL95555 Driver   │ Low-level device register control        │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic GPIO Operations

```cpp
#include "utils-and-drivers/driver-handlers/Pcal95555Handler.h"
#include "component-handlers/CommChannelsManager.h"

void pcal95555_basic_example() {
    // Get I2C interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) {
        logger.Info("PCAL95555", "I2C interface not available\n");
        return;
    }
    
    // Create PCAL95555 handler
    Pcal95555Handler handler(*i2c);
    
    // Initialize handler
    if (!handler.EnsureInitialized()) {
        logger.Info("PCAL95555", "Failed to initialize PCAL95555\n");
        return;
    }
    
    // Configure pins
    handler.SetDirection(0, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    handler.SetDirection(1, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    
    // Basic operations
    handler.SetOutput(0, true);         // Set pin 0 high
    hf_bool_t state;
    handler.ReadInput(1, state);        // Read pin 1
    
    logger.Info("PCAL95555", "PCAL95555 GPIO operations complete\n");
    logger.Info("PCAL95555", "Pin 1 state: %s\n", state ? "HIGH" : "LOW");
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class Pcal95555Handler {
public:
    // Constructor
    explicit Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin = nullptr) noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool EnsureDeinitialized() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### Basic GPIO Operations
```cpp
// Pin direction control
hf_gpio_err_t SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept;
hf_gpio_err_t SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept;

// Pin output control
hf_gpio_err_t SetOutput(hf_u8_t pin, hf_bool_t active) noexcept;
hf_gpio_err_t Toggle(hf_u8_t pin) noexcept;

// Pin input reading
hf_gpio_err_t ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept;

// Pull resistor configuration
hf_gpio_err_t SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept;
hf_gpio_err_t GetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept;
hf_gpio_err_t SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept;
```

#### Interrupt Management
```cpp
// Interrupt support
bool HasInterruptSupport() const noexcept;
bool IsInterruptConfigured() const noexcept;

// Interrupt configuration
hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                InterruptCallback callback = nullptr,
                                void* user_data = nullptr) noexcept;
hf_gpio_err_t EnableInterrupt() noexcept;
hf_gpio_err_t DisableInterrupt() noexcept;

// Interrupt status
hf_gpio_err_t GetAllInterruptMasks(uint16_t& mask) noexcept;
hf_gpio_err_t GetAllInterruptStatus(uint16_t& status) noexcept;
```

#### BaseGpio Interface
```cpp
// Pin creation and management
std::shared_ptr<BaseGpio> CreateGpioPin(
    hf_pin_num_t pin,
    hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
    hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
    hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
    hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;

std::shared_ptr<BaseGpio> GetGpioPin(hf_pin_num_t pin) noexcept;
bool IsPinCreated(hf_pin_num_t pin) const noexcept;
```

#### Advanced Features
```cpp
// Polarity inversion
hf_bool_t SetPolarityInversion(hf_pin_num_t pin, hf_bool_t invert) noexcept;
hf_bool_t GetPolarityInversion(hf_pin_num_t pin, hf_bool_t& invert) noexcept;

// Interrupt masking
hf_bool_t SetInterruptMask(hf_pin_num_t pin, hf_bool_t mask) noexcept;
hf_bool_t GetInterruptMask(hf_pin_num_t pin, hf_bool_t& mask) noexcept;
hf_bool_t GetInterruptStatus(hf_pin_num_t pin, hf_bool_t& status) noexcept;

// Pin validation
static constexpr hf_u8_t PinCount() noexcept { return 16; }
```

## 🎯 Hardware Support

### PCAL95555 Features

- **16 GPIO Pins**: Independent input/output control
- **I2C Interface**: 7-bit addressing (0x20-0x27)
- **Interrupt Support**: Configurable interrupt generation
- **Pull-up/Pull-down**: Programmable pull resistors
- **Polarity Inversion**: Configurable active high/low
- **Input Filtering**: Built-in noise filtering
- **ESD Protection**: Robust electrostatic discharge protection

### Communication Interface

The handler uses an I2C adapter that bridges the `BaseI2c` interface with the PCAL95555 driver's `i2cBus` interface, providing type-safe communication and address validation.

## 📊 Examples

### Basic GPIO Control

```cpp
void basic_gpio_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    Pcal95555Handler handler(*i2c);
    if (!handler.EnsureInitialized()) return;
    
    // Configure pins
    handler.SetDirection(0, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    handler.SetDirection(1, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    handler.SetPullMode(1, hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULLUP);
    
    // Control output
    handler.SetOutput(0, true);
    handler.Toggle(0);
    
    // Read input
    hf_bool_t state;
    if (handler.ReadInput(1, state) == hf_gpio_err_t::HF_GPIO_ERR_NONE) {
        logger.Info("PCAL95555", "Pin 1 state: %s\n", state ? "HIGH" : "LOW");
    }
}
```

### Interrupt Handling

```cpp
void interrupt_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    // Create handler with interrupt pin
    BaseGpio* int_pin = nullptr; // Get from GPIO manager
    Pcal95555Handler handler(*i2c, int_pin);
    
    if (!handler.EnsureInitialized()) return;
    
    // Configure interrupt
    if (handler.HasInterruptSupport()) {
        handler.ConfigureInterrupt(
            hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE,
            [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
                logger.Info("PCAL95555", "PCAL95555 interrupt triggered\n");
            }
        );
        
        handler.EnableInterrupt();
    }
    
    // Monitor interrupt status
    uint16_t interrupt_status;
    if (handler.GetAllInterruptStatus(interrupt_status) == hf_gpio_err_t::HF_GPIO_ERR_NONE) {
        logger.Info("PCAL95555", "Interrupt status: 0x%04X\n", interrupt_status);
    }
}
```

### BaseGpio Integration

```cpp
void base_gpio_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    Pcal95555Handler handler(*i2c);
    if (!handler.EnsureInitialized()) return;
    
    // Create BaseGpio pin
    auto pin = handler.CreateGpioPin(0, 
        hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT,
        hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH);
    
    if (pin) {
        // Use standard BaseGpio interface
        pin->SetActive(true);
        pin->Toggle();
        
        // Configure interrupt
        pin->ConfigureInterrupt(
            hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
            [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
                logger.Info("PCAL95555", "Pin interrupt triggered\n");
            }
        );
    }
}
```

### Batch Operations

```cpp
void batch_operations_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    Pcal95555Handler handler(*i2c);
    if (!handler.EnsureInitialized()) return;
    
    // Configure multiple pins as outputs
    uint16_t output_mask = 0x000F; // Pins 0-3
    handler.SetDirections(output_mask, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    
    // Set multiple outputs
    handler.SetOutputs(output_mask, true);
    
    // Configure pull resistors for input pins
    uint16_t input_mask = 0x00F0; // Pins 4-7
    handler.SetPullModes(input_mask, hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULLUP);
}
```

### Error Handling

```cpp
void error_handling_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    Pcal95555Handler handler(*i2c);
    
    // Check initialization
    if (!handler.EnsureInitialized()) {
        logger.Info("PCAL95555", "ERROR: Failed to initialize PCAL95555\n");
        return;
    }
    
    // Validate pin operations
    hf_gpio_err_t result = handler.SetDirection(20, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    if (result != hf_gpio_err_t::HF_GPIO_ERR_NONE) {
        logger.Info("PCAL95555", "ERROR: Invalid pin operation: %d\n", static_cast<int>(result));
        return;
    }
    
    // Safe pin operations
    for (hf_u8_t pin = 0; pin < handler.PinCount(); pin++) {
        result = handler.SetDirection(pin, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
        if (result != hf_gpio_err_t::HF_GPIO_ERR_NONE) {
            logger.Info("PCAL95555", "ERROR: Failed to configure pin %d: %d\n", pin, static_cast<int>(result));
        }
    }
}
```

## 🔍 Advanced Usage

### Multi-Pin Management

```cpp
void multi_pin_management() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    Pcal95555Handler handler(*i2c);
    if (!handler.EnsureInitialized()) return;
    
    // Create multiple BaseGpio pins
    std::vector<std::shared_ptr<BaseGpio>> pins;
    for (hf_u8_t pin_num = 0; pin_num < handler.PinCount(); pin_num++) {
        auto pin = handler.CreateGpioPin(pin_num);
        if (pin) {
            pins.push_back(pin);
        }
    }
    
    // Use pins through BaseGpio interface
    for (auto& pin : pins) {
        pin->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        pin->SetActive(true);
    }
}
```

### Interrupt Monitoring

```cpp
void interrupt_monitoring() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c) return;
    
    BaseGpio* int_pin = nullptr; // Get from GPIO manager
    Pcal95555Handler handler(*i2c, int_pin);
    
    if (!handler.EnsureInitialized()) return;
    
    // Configure interrupt monitoring
    if (handler.HasInterruptSupport()) {
        // Set up interrupt masks
        for (hf_u8_t pin = 0; pin < handler.PinCount(); pin++) {
            handler.SetInterruptMask(pin, true);
        }
        
        // Enable interrupt
        handler.EnableInterrupt();
        
        // Monitor interrupt status
        uint16_t interrupt_status;
        uint16_t interrupt_masks;
        
        handler.GetAllInterruptMasks(interrupt_masks);
        handler.GetAllInterruptStatus(interrupt_status);
        
        logger.Info("PCAL95555", "Interrupt masks: 0x%04X\n", interrupt_masks);
        logger.Info("PCAL95555", "Interrupt status: 0x%04X\n", interrupt_status);
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](../component-handlers/GPIO_MANAGER_README.md)** - GPIO management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[TMC9660 Handler Documentation](TMC9660_HANDLER_README.md)** - Motor controller with GPIO
- **[BNO08x Handler Documentation](BNO08X_HANDLER_README.md)** - IMU sensor handler

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*