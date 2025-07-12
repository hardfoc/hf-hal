# GPIO Architecture Final Documentation

## Overview

The HardFOC GPIO architecture has been successfully modernized and unified under a single, comprehensive `BaseGpio` class. This design provides a clean, extensible foundation for all GPIO implementations while maintaining full compatibility with existing digital GPIO functionality.

## Architecture Summary

### Core Design Principles

1. **Single Base Class**: `BaseGpio` serves as the unified base for all GPIO implementations
2. **Hardware Abstraction**: Platform-specific implementations inherit from `BaseGpio`
3. **Comprehensive Functionality**: All digital GPIO features are built into the base class
4. **Extensibility**: Easy to add new GPIO hardware (I2C expanders, SPI devices, etc.)
5. **Error Handling**: Robust error management with comprehensive error codes

### Key Features

- **Dynamic Mode Switching**: Runtime switching between input and output modes
- **Polarity Configuration**: Active-high/active-low support
- **Pull Resistor Management**: Floating, pull-up, pull-down configuration
- **Output Drive Modes**: Push-pull and open-drain support
- **Thread-Safe Design**: Proper state management for concurrent access
- **Lazy Initialization**: Initialize hardware only when needed

## Class Hierarchy

```
BaseGpio (Unified Base Class)
├── McuDigitalGpio (ESP32C6, STM32, etc.)
├── Pcal95555DigitalGpio (I2C GPIO Expander)
├── Tmc9660GpioPin (TMC Motor Controller)
├── DigitalExternalIRQ (Interrupt-capable GPIO)
├── PwmOutput (PWM-capable GPIO)
└── [Future GPIO implementations]
```

## File Structure

### Core Files
- `BaseGpio.h` - Unified base class with all GPIO functionality
- `McuDigitalGpio.h/.cpp` - MCU-specific implementation
- `GpioGuard.h/.cpp` - RAII GPIO output management
- `DigitalExternalIRQ.h/.cpp` - External interrupt support

### Hardware-Specific Files
- `Pcal95555DigitalGpio.h/.cpp` - PCAL95555 I2C GPIO expander
- `Tmc9660Gpio.h/.cpp` - TMC9660 motor controller GPIO
- `PwmOutput.h/.cpp` - PWM output functionality

### Support Files
- `PlatformTypes.h` - Platform-agnostic type definitions
- `HfGpioErr` - Comprehensive error enumeration

## BaseGpio Class Features

### Enumerations

```cpp
enum class State : uint8_t {
    Inactive = 0,
    Active = 1
};

enum class ActiveState : uint8_t {
    Low = 0,      // Active when pin is low
    High = 1      // Active when pin is high
};

enum class Direction : uint8_t {
    Input = 0,
    Output = 1
};

enum class OutputMode : uint8_t {
    PushPull = 0,    // Strong drive both high and low
    OpenDrain = 1    // Strong low, high-impedance high
};

enum class PullMode : uint8_t {
    Floating = 0,    // No internal pull resistor
    PullUp = 1,      // Internal pull-up enabled
    PullDown = 2     // Internal pull-down enabled
};
```

### Error Handling

Comprehensive error enumeration with 29 specific error codes:
- Success/failure codes
- Pin-specific errors
- Hardware communication errors
- Configuration errors
- I/O operation errors
- System-level errors

### Key Methods

```cpp
// Core GPIO operations
virtual HfGpioErr SetActive() noexcept = 0;
virtual HfGpioErr SetInactive() noexcept = 0;
virtual HfGpioErr Toggle() noexcept = 0;
virtual HfGpioErr IsActive(bool& is_active) noexcept = 0;

// Configuration management
HfGpioErr SetDirection(Direction direction) noexcept;
HfGpioErr SetOutputMode(OutputMode mode) noexcept;
HfGpioErr SetPullMode(PullMode mode) noexcept;

// State and status
bool IsInitialized() const noexcept;
bool EnsureInitialized() noexcept;
Direction GetDirection() const noexcept;
State GetCurrentState() const noexcept;
```

## Implementation Classes

### McuDigitalGpio

**Purpose**: Direct MCU GPIO pin control (ESP32C6, STM32, etc.)

**Features**:
- Platform-specific GPIO register access
- Hardware-specific pin validation
- Optimized for performance
- Supports all GPIO modes and configurations

**Usage**:
```cpp
McuDigitalGpio led(GPIO_NUM_2, BaseGpio::Direction::Output, 
                   BaseGpio::ActiveState::High);
led.Initialize();
led.SetActive();  // Turn on LED
```

### Pcal95555DigitalGpio

**Purpose**: I2C GPIO expander support

**Features**:
- Thread-safe I2C communication
- Supports up to 16 GPIO pins per chip
- Dynamic configuration
- Comprehensive error handling

**Usage**:
```cpp
auto pcal_gpio = std::make_unique<Pcal95555DigitalGpio>(
    0, pcal_driver, 0x20, BaseGpio::Direction::Output);
pcal_gpio->Initialize();
pcal_gpio->SetActive();
```

### DigitalExternalIRQ

**Purpose**: Interrupt-capable GPIO pins

**Features**:
- External interrupt support
- Configurable trigger conditions
- Callback mechanism
- Debouncing support

### GpioGuard

**Purpose**: RAII-based GPIO output management

**Features**:
- Automatic active/inactive state management
- Exception-safe cleanup
- Configurable output mode switching
- Error state tracking

**Usage**:
```cpp
{
    GpioGuard guard(gpio_pin);
    // GPIO is automatically set active
    // ... perform operations ...
}  // GPIO automatically set inactive
```

## Migration from Legacy Code

### Before (Legacy DigitalGpio)
```cpp
#include "DigitalGpio.h"

class MyGpio : public DigitalGpio {
    // Implementation
};
```

### After (New BaseGpio)
```cpp
#include "BaseGpio.h"

class MyGpio : public BaseGpio {
    // Same implementation, but inherits from BaseGpio
};
```

## Benefits of the New Architecture

### 1. Simplified Hierarchy
- Single base class reduces complexity
- Eliminates unnecessary abstraction layers
- Clearer inheritance relationships

### 2. Unified Interface
- All digital GPIO functionality in one place
- Consistent API across all implementations
- Reduced cognitive load for developers

### 3. Better Maintainability
- Fewer files to maintain
- Centralized feature development
- Consistent error handling patterns

### 4. Enhanced Extensibility
- Easy to add new GPIO hardware
- Common functionality automatically available
- Standardized implementation patterns

### 5. Improved Error Handling
- Comprehensive error enumeration
- Consistent error reporting
- Better debugging capabilities

## Best Practices

### 1. Always Check Initialization
```cpp
if (!gpio.EnsureInitialized()) {
    // Handle initialization failure
}
```

### 2. Use RAII Patterns
```cpp
// Prefer GpioGuard for temporary output operations
GpioGuard guard(gpio);
```

### 3. Handle Errors Properly
```cpp
HfGpioErr result = gpio.SetActive();
if (result != HfGpioErr::GPIO_SUCCESS) {
    console_error("GPIO", "Failed to set active: %s", 
                  HfGpioErrToString(result).data());
}
```

### 4. Validate Hardware Before Use
```cpp
if (!gpio.IsPinAvailable()) {
    // Pin is not available for GPIO operations
}
```

## Testing

The architecture has been validated with:
- Unit tests for all core functionality
- Integration tests with real hardware
- Mock implementations for testing
- Error condition testing

## Future Enhancements

1. **Additional Hardware Support**:
   - SPI GPIO expanders
   - CAN-based GPIO controllers
   - Serial GPIO interfaces

2. **Advanced Features**:
   - GPIO synchronization primitives
   - Hardware PWM integration
   - DMA-based GPIO operations

3. **Performance Optimizations**:
   - Batch GPIO operations
   - Hardware-accelerated state changes
   - Low-latency interrupt handling

## Conclusion

The unified BaseGpio architecture provides a robust, extensible foundation for all GPIO operations in the HardFOC system. By consolidating functionality into a single base class, we've achieved better maintainability, clearer code structure, and improved developer experience while maintaining full compatibility with existing digital GPIO features.

The architecture is now ready for production use and provides a solid foundation for future GPIO hardware additions and feature enhancements.
