# HardFOC Documentation Index

This document provides an index of all available documentation for the HardFOC system.

## Primary Documentation

### 🎯 GPIO and ADC System Documentation
**[docs/HARDFOC_GPIO_ADC_SYSTEM.md](docs/HARDFOC_GPIO_ADC_SYSTEM.md)**
- **Comprehensive guide** for the GPIO and ADC data sourcing system
- **System architecture** and design principles  
- **Hardware support**: ESP32-C6, PCAL95555, TMC9660
- **Complete usage examples** and integration guides
- **Pin mappings** and hardware configurations
- **Health monitoring** and error handling
- **Performance considerations** and troubleshooting

### 📚 Component Handler Quick Reference  
**[component-handler/README.md](component-handler/README.md)**
- **Quick reference** for the component handler system
- **Basic usage examples**
- **Core class overview**
- **Hardware support summary**

## Code Organization

### Core Components
```
hf-hal/
├── component-handler/          # Main GPIO and ADC handling components
│   ├── GpioData.h/cpp         # Pure GPIO data management
│   ├── GpioHandler.h/cpp      # Main GPIO interface with health monitoring
│   ├── AdcData.h/cpp          # Pure ADC data management
│   ├── AdcHandler.h/cpp       # Main ADC interface with health monitoring
│   ├── CommonIDs.h            # System-wide enumerations and constants
│   ├── SystemInit.h/cpp       # System initialization utilities
│   └── All.h                  # Master include file
├── API/                        # Public API interfaces
│   ├── All.h                  # Public API master include
│   ├── HardFocIntegration.h/cpp # Integration examples
│   └── SystemInit.h/cpp       # Public system initialization
└── docs/                       # Documentation
    └── HARDFOC_GPIO_ADC_SYSTEM.md # Comprehensive system documentation
```

### Hardware Drivers
```
hf-hal/utils-and-drivers/hf-core-drivers/
├── external/                   # External device drivers
│   ├── hf-pcal95555-driver/   # PCAL95555 I2C GPIO expander
│   └── hf-tmc9660-driver/     # TMC9660 motor controller
└── internal/                   # Internal ESP32-C6 drivers
    ├── hf-internal-interface-wrap/ # Hardware abstraction layer
    └── hf-pincfg/             # Pin configuration and safety validation
```

## Quick Start Guide

### 1. Include the System
```cpp
#include "component-handler/All.h"
```

### 2. Initialize Hardware
```cpp
// Initialize I2C for PCAL95555 expanders
auto& i2cBus = SfI2cBus::GetInstance();
// Configure and initialize I2C bus

// Initialize GPIO system
auto& gpioHandler = GpioHandler::GetInstance();  
gpioHandler.Initialize(i2cBus);

// Initialize ADC system
auto& adcHandler = AdcHandler::GetInstance();
adcHandler.Initialize();
```

### 3. Use GPIO and ADC
```cpp
// GPIO operations
gpioHandler.SetPin(GPIO_ESP32_PIN_2, true);
bool state = gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_0);

// ADC operations  
uint16_t rawValue = adcHandler.ReadRaw(ADC_ESP32_ADC1_CH0);
float voltage = adcHandler.ReadVoltage(ADC_ESP32_ADC1_CH0);
```

## System Features

### GPIO Sources Supported
- ✅ **ESP32-C6 Native GPIO**: 40+ pins with safety validation
- ✅ **PCAL95555 I2C Expanders**: Up to 2 chips (32 additional pins)  
- ✅ **TMC9660 Motor Controller**: Specialized GPIO pins for motor control

### ADC Sources Supported
- ✅ **ESP32-C6 Internal ADC**: ADC1 and ADC2 units with calibration
- ✅ **TMC9660 Motor Controller**: AIN1, AIN2, AIN3 analog inputs

### Key Features
- 🔒 **Thread-safe operation** with mutex protection
- 🏥 **Health monitoring** for all hardware sources
- ⚡ **Performance optimized** with batch operations
- 🛡️ **Pin safety validation** for ESP32-C6
- 🔧 **Comprehensive error handling** and diagnostics
- 📊 **Real-time system status** monitoring

## Documentation Guidelines

When working with the HardFOC system:

1. **Start with**: [docs/HARDFOC_GPIO_ADC_SYSTEM.md](docs/HARDFOC_GPIO_ADC_SYSTEM.md) for comprehensive understanding
2. **Reference**: [component-handler/README.md](component-handler/README.md) for quick lookups
3. **Check**: Header files for detailed API documentation
4. **Follow**: Integration examples in HardFocIntegration.h/cpp

## Getting Help

- **Comprehensive Documentation**: See [docs/HARDFOC_GPIO_ADC_SYSTEM.md](docs/HARDFOC_GPIO_ADC_SYSTEM.md)
- **API Reference**: Check individual header files for detailed method documentation
- **Integration Examples**: See HardFocIntegration.h/cpp for working examples
- **Troubleshooting**: Comprehensive troubleshooting section in main documentation

---

*This documentation index was last updated to reflect the clean GPIO and ADC system architecture.*
