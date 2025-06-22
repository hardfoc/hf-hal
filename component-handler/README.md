# HardFOC Component Handler

This directory contains the core GPIO and ADC handling components for the HardFOC system.

## Documentation

For comprehensive documentation on the GPIO and ADC data sourcing system, including:
- System architecture and design principles
- Usage examples and integration guides
- Pin mappings and hardware configurations  
- Health monitoring and error handling
- Performance considerations and troubleshooting

Please refer to: **[../docs/HARDFOC_GPIO_ADC_SYSTEM.md](../docs/HARDFOC_GPIO_ADC_SYSTEM.md)**

## Quick Reference

### Core Classes
- **GpioData**: Pure data management for GPIO operations
- **GpioHandler**: Main interface for GPIO operations with initialization and health monitoring
- **AdcData**: Pure data management for ADC operations  
- **AdcHandler**: Main interface for ADC operations with initialization and health monitoring

### Supported Hardware
- **ESP32-C6**: Native GPIO pins and internal ADC units
- **PCAL95555**: I2C GPIO expander (up to 2 chips, 16 pins each)
- **TMC9660**: Motor controller GPIO pins and ADC channels

### Basic Usage
```cpp
#include "component-handler/All.h"

// Initialize systems
auto& gpioHandler = GpioHandler::GetInstance();
auto& i2cBus = SfI2cBus::GetInstance();
gpioHandler.Initialize(i2cBus);

// Use GPIO pins
gpioHandler.SetPin(GPIO_ESP32_PIN_2, true);
bool state = gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_0);
```

See the comprehensive documentation for detailed usage examples and integration instructions.
