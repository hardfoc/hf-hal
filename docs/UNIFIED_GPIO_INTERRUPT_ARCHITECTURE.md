# Unified GPIO Interrupt Architecture

## Overview

The HardFOC GPIO architecture now provides **unified interrupt support** directly integrated into the `BaseGpio` class. This eliminates the need for separate interrupt classes and provides a clean, polymorphic interface for GPIO interrupts across all implementations.

## Key Benefits

### 🎯 **Architectural Improvements**
1. **Single Class Design**: No separate `DigitalExternalIRQ` class needed
2. **Polymorphic Interrupts**: Works seamlessly across MCU, I2C expander, and future GPIO types
3. **Optional Functionality**: Non-interrupt capable implementations gracefully return "not supported"
4. **Consistent API**: Same interrupt interface regardless of underlying hardware

### 🔧 **Implementation Flexibility**
- **MCU GPIOs**: Direct hardware interrupt support with ESP32/STM32 registers
- **I2C Expanders**: Can use GPIO expander interrupt pins tied to MCU interrupt pins
- **SPI Devices**: Future SPI GPIO expanders can implement interrupt forwarding
- **Mixed Systems**: Seamlessly combine interrupt-capable and polling-only pins

### 🚀 **Developer Experience**
- **Simplified Code**: No need to choose between different GPIO classes
- **Runtime Discovery**: `SupportsInterrupts()` method for capability checking
- **Graceful Degradation**: Automatically falls back to polling when interrupts not available
- **Unified Error Handling**: Consistent error codes across all implementations

## Class Architecture

```cpp
BaseGpio (Unified Base Class)
├── Core GPIO Features (direction, state, pull resistors, etc.)
├── Interrupt Support (optional - default returns "not supported")
│   ├── ConfigureInterrupt(trigger, callback, user_data)
│   ├── EnableInterrupt() / DisableInterrupt()
│   ├── WaitForInterrupt(timeout)
│   └── GetInterruptStatus() / ClearInterruptStats()
└── Derived Implementations
    ├── McuDigitalGpio (ESP32C6 with full interrupt support)
    ├── Pcal95555DigitalGpio (I2C expander with optional interrupt forwarding)
    └── [Future implementations]
```

## Interrupt Support Levels

### Level 1: Full Hardware Interrupt Support
**Example**: `McuDigitalGpio` (ESP32C6 direct pins)
- Hardware interrupt generation
- Callback-based handling
- Blocking wait with timeout
- Interrupt statistics/counting
- All trigger types supported

### Level 2: Forwarded Interrupt Support  
**Example**: `Pcal95555DigitalGpio` with interrupt pin
- GPIO expander interrupt pin connected to MCU interrupt pin
- Single shared interrupt for all expander pins
- Software demultiplexing of interrupt sources
- Slightly higher latency than direct interrupts

### Level 3: No Interrupt Support
**Example**: GPIO expanders without interrupt pins
- `SupportsInterrupts()` returns `false`
- All interrupt methods return `GPIO_ERR_INTERRUPT_NOT_SUPPORTED`
- Application can fall back to polling
- No impact on other GPIO functionality

## Interrupt Configuration

### Basic Setup
```cpp
McuDigitalGpio button(GPIO_NUM_0, BaseGpio::Direction::Input, 
                     BaseGpio::ActiveState::Low);

// Check if interrupts are supported
if (button.SupportsInterrupts()) {
    // Configure falling edge interrupt with callback
    button.ConfigureInterrupt(
        BaseGpio::InterruptTrigger::FallingEdge,
        [](BaseGpio* gpio, BaseGpio::InterruptTrigger trigger, void* user_data) {
            printf("Button pressed on pin %d!\n", gpio->GetPin());
        }
    );
    
    button.EnableInterrupt();
}
```

### Polling Fallback
```cpp
void CheckGpioState(BaseGpio* gpio) {
    if (gpio->SupportsInterrupts()) {
        // Use interrupt-based detection
        HfGpioErr result = gpio->WaitForInterrupt(100);  // 100ms timeout
        if (result == HfGpioErr::GPIO_SUCCESS) {
            printf("Interrupt detected!\n");
        }
    } else {
        // Fall back to polling
        bool is_active = false;
        if (gpio->IsActive(is_active) == HfGpioErr::GPIO_SUCCESS && is_active) {
            printf("State change detected via polling!\n");
        }
    }
}
```

### Polymorphic Usage
```cpp
class InputManager {
public:
    void AddInput(std::unique_ptr<BaseGpio> input, const std::string& name) {
        if (!input->Initialize()) return;
        
        if (input->SupportsInterrupts()) {
            input->ConfigureInterrupt(BaseGpio::InterruptTrigger::BothEdges,
                                     InputInterruptHandler, 
                                     const_cast<char*>(name.c_str()));
            input->EnableInterrupt();
            printf("%s: Using interrupt mode\n", name.c_str());
        } else {
            printf("%s: Using polling mode\n", name.c_str());
        }
        
        inputs_.push_back({std::move(input), name});
    }

private:
    struct InputInfo {
        std::unique_ptr<BaseGpio> gpio;
        std::string name;
    };
    std::vector<InputInfo> inputs_;
};

// Usage with mixed GPIO types
InputManager manager;
manager.AddInput(std::make_unique<McuDigitalGpio>(GPIO_NUM_0), "Button1");
manager.AddInput(std::make_unique<Pcal95555DigitalGpio>(...), "Button2");
```

## Interrupt Trigger Types

| Trigger Type | Description | Use Cases |
|--------------|-------------|-----------|
| `None` | No interrupt (disabled) | Default state |
| `RisingEdge` | Low → High transition | Button release, positive edge detection |
| `FallingEdge` | High → Low transition | Button press, negative edge detection |
| `BothEdges` | Any state change | Motion detection, state monitoring |
| `LowLevel` | Continuous low level | Active-low signals, fault conditions |
| `HighLevel` | Continuous high level | Active-high signals, ready conditions |

## Implementation Guidelines

### For MCU GPIO Implementations
```cpp
class McuDigitalGpio : public BaseGpio {
public:
    bool SupportsInterrupts() const noexcept override {
        return true;  // MCU pins typically support interrupts
    }
    
    HfGpioErr ConfigureInterrupt(InterruptTrigger trigger, 
                                InterruptCallback callback, 
                                void* user_data) noexcept override {
        // Store configuration
        interrupt_trigger_ = trigger;
        interrupt_callback_ = callback;
        interrupt_user_data_ = user_data;
        
        // Configure hardware registers
        return ConfigureHardwareInterrupt();
    }
    
private:
    static void IRAM_ATTR StaticInterruptHandler(void* arg) {
        static_cast<McuDigitalGpio*>(arg)->HandleInterrupt();
    }
    
    void HandleInterrupt() noexcept {
        interrupt_count_++;
        if (interrupt_callback_) {
            interrupt_callback_(this, interrupt_trigger_, interrupt_user_data_);
        }
    }
};
```

### For I2C GPIO Expander Implementations
```cpp
class Pcal95555DigitalGpio : public BaseGpio {
public:
    bool SupportsInterrupts() const noexcept override {
        return has_interrupt_pin_;  // Only if hardware interrupt pin connected
    }
    
    HfGpioErr ConfigureInterrupt(InterruptTrigger trigger, 
                                InterruptCallback callback, 
                                void* user_data) noexcept override {
        if (!has_interrupt_pin_) {
            return HfGpioErr::GPIO_ERR_INTERRUPT_NOT_SUPPORTED;
        }
        
        // Configure expander interrupt registers
        // Set up shared interrupt handler
        return ConfigureExpanderInterrupt();
    }
};
```

### For Non-Interrupt Implementations
```cpp
class SimpleGpioExpander : public BaseGpio {
public:
    bool SupportsInterrupts() const noexcept override {
        return false;  // This expander doesn't support interrupts
    }
    
    // All interrupt methods use default BaseGpio implementation
    // which returns GPIO_ERR_INTERRUPT_NOT_SUPPORTED
};
```

## Error Handling

### Interrupt-Specific Error Codes
- `GPIO_ERR_INTERRUPT_NOT_SUPPORTED` - Implementation doesn't support interrupts
- `GPIO_ERR_INTERRUPT_ALREADY_ENABLED` - Interrupt already enabled
- `GPIO_ERR_INTERRUPT_NOT_ENABLED` - Trying to disable non-enabled interrupt
- `GPIO_ERR_INTERRUPT_HANDLER_FAILED` - Interrupt handler installation failed

### Graceful Error Handling
```cpp
HfGpioErr result = gpio->ConfigureInterrupt(BaseGpio::InterruptTrigger::RisingEdge, callback);

switch (result) {
    case HfGpioErr::GPIO_SUCCESS:
        printf("Interrupt configured successfully\n");
        break;
        
    case HfGpioErr::GPIO_ERR_INTERRUPT_NOT_SUPPORTED:
        printf("Interrupts not supported, using polling mode\n");
        // Fall back to polling
        break;
        
    case HfGpioErr::GPIO_ERR_INVALID_PARAMETER:
        printf("Invalid interrupt configuration\n");
        break;
        
    default:
        printf("Interrupt configuration failed: %s\n", HfGpioErrToString(result));
        break;
}
```

## Performance Characteristics

### MCU Direct Interrupts
- **Latency**: < 10μs (hardware interrupt)
- **Overhead**: Minimal (direct hardware)
- **Throughput**: High (hardware-limited)

### I2C Expander Interrupts  
- **Latency**: 50-200μs (I2C communication + demux)
- **Overhead**: Medium (shared interrupt + I2C)
- **Throughput**: Medium (I2C bandwidth limited)

### Polling Fallback
- **Latency**: Depends on polling interval
- **Overhead**: Low to High (depends on frequency)
- **Throughput**: Application-dependent

## Migration from DigitalExternalIRQ

### Before (Old Architecture)
```cpp
#include "DigitalExternalIRQ.h"

DigitalExternalIRQ irq_pin(GPIO_NUM_0, hf_gpio_intr_type_t::FallingEdge);
if (irq_pin.Initialize()) {
    irq_pin.Enable();
    bool triggered = irq_pin.Wait(5000);
}
```

### After (New Architecture)
```cpp
#include "McuDigitalGpio.h"

McuDigitalGpio gpio_pin(GPIO_NUM_0, BaseGpio::Direction::Input);
if (gpio_pin.Initialize() && gpio_pin.SupportsInterrupts()) {
    gpio_pin.ConfigureInterrupt(BaseGpio::InterruptTrigger::FallingEdge);
    gpio_pin.EnableInterrupt();
    HfGpioErr result = gpio_pin.WaitForInterrupt(5000);
}
```

## Best Practices

### 1. Always Check Interrupt Support
```cpp
if (gpio->SupportsInterrupts()) {
    // Use interrupt mode
} else {
    // Implement polling fallback
}
```

### 2. Handle Errors Gracefully
```cpp
HfGpioErr result = gpio->EnableInterrupt();
if (result != HfGpioErr::GPIO_SUCCESS) {
    // Handle error or fall back to polling
}
```

### 3. Use RAII for Resource Management
```cpp
class InterruptGuard {
public:
    InterruptGuard(BaseGpio* gpio) : gpio_(gpio) {
        if (gpio_->SupportsInterrupts()) {
            gpio_->EnableInterrupt();
        }
    }
    
    ~InterruptGuard() {
        if (gpio_->SupportsInterrupts()) {
            gpio_->DisableInterrupt();
        }
    }
    
private:
    BaseGpio* gpio_;
};
```

### 4. Design for Mixed Implementations
```cpp
void ProcessInputs(const std::vector<BaseGpio*>& inputs) {
    for (auto* input : inputs) {
        if (input->SupportsInterrupts()) {
            // Fast interrupt-based processing
            ProcessWithInterrupts(input);
        } else {
            // Fallback polling processing
            ProcessWithPolling(input);
        }
    }
}
```

## Conclusion

The unified GPIO interrupt architecture provides:

✅ **Simplified Development** - Single class for all GPIO operations
✅ **Hardware Flexibility** - Works with any GPIO implementation  
✅ **Graceful Degradation** - Automatic fallback to polling
✅ **Future-Proof Design** - Easy to add new GPIO hardware
✅ **Consistent API** - Same interface across all implementations
✅ **Performance Optimization** - Use interrupts where available, polling where not

This architecture eliminates the complexity of choosing between different GPIO classes while providing maximum flexibility for both current and future hardware implementations.
