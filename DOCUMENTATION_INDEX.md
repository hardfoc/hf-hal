# HardFOC Documentation Index

This document provides an index of all available documentation for the HardFOC system.

## Primary Documentation

### 🎯 Core System Documentation
**[docs/HARDFOC_GPIO_ADC_SYSTEM.md](docs/HARDFOC_GPIO_ADC_SYSTEM.md)**
- **Comprehensive guide** for the GPIO and ADC data sourcing system
- **System architecture** and design principles  
- **Hardware support**: ESP32-C6, PCAL95555, TMC9660
- **Complete usage examples** and integration guides
- **Pin mappings** and hardware configurations
- **Health monitoring** and error handling
- **Performance considerations** and troubleshooting

**[component-handler/GPIO_MANAGER_README.md](component-handler/GPIO_MANAGER_README.md)**
- **Complete documentation** for the new GpioManager system

**[component-handler/ADC_MANAGER_README.md](component-handler/ADC_MANAGER_README.md)**
- **Complete documentation** for the new AdcManager system
- **Multi-chip GPIO management** (ESP32, PCAL95555, TMC9660)
- **Platform mapping integration** with functional pin identifiers
- **Thread-safe operations** and advanced features
- **TMC9660 motor controller integration** with GPIO control
- **Comprehensive examples** and troubleshooting guide

### 📚 Component Handler Quick Reference  
**[component-handler/README.md](component-handler/README.md)**
- **Quick reference** for the component handler system
- **Basic usage examples**
- **Core class overview**
- **Hardware support summary**

### 🏗️ Internal Interface Architecture
**[docs/INTERNAL_INTERFACE_WRAPPER_DEEP_DIVE_ANALYSIS.md](docs/INTERNAL_INTERFACE_WRAPPER_DEEP_DIVE_ANALYSIS.md)**
- **Deep architectural analysis** of internal interface wrappers
- **Base classes** and ESP32-C6 implementations
- **Performance rankings** and recommendations
- **Advanced features** and DMA support analysis

### 🧪 Test Suite Documentation
**[tests/README.md](tests/README.md)**
- **Consolidated test suite** structure and usage
- **Component handler, interface wrapper, and external driver tests**
- **Build and execution instructions**
- **Test contribution guidelines**

## Architecture Documentation

### Core Hardware Abstraction
**[docs/HARDFOC_HARDWARE_ABSTRACTION_ARCHITECTURE.md](docs/HARDFOC_HARDWARE_ABSTRACTION_ARCHITECTURE.md)**
- Complete hardware abstraction layer architecture
- Component interactions and design patterns
- System-wide architectural decisions

### GPIO System Architecture
**[docs/GPIO_ARCHITECTURE_FINAL.md](docs/GPIO_ARCHITECTURE_FINAL.md)**
- Final GPIO system architecture and implementation
- Multi-source GPIO management
- Interrupt handling and safety features

### PWM System Architecture  
**[docs/PWM_ARCHITECTURE.md](docs/PWM_ARCHITECTURE.md)**
- PWM system design and implementation
- Channel management and configuration
- Performance characteristics

### Unified GPIO Interrupt Architecture
**[docs/UNIFIED_GPIO_INTERRUPT_ARCHITECTURE.md](docs/UNIFIED_GPIO_INTERRUPT_ARCHITECTURE.md)**
- Modern GPIO interrupt handling system
- Event-driven architecture
- Safety and performance optimizations

### Final Implementation Report
**[docs/FINAL_IMPLEMENTATION_REPORT.md](docs/FINAL_IMPLEMENTATION_REPORT.md)**
- Complete implementation status and summary
- System testing results and validation
- Known limitations and future improvements

## Code Organization

### Core Components
```
hf-hal/
├── component-handler/          # Main GPIO and ADC handling components
│   ├── GpioManager.h/cpp       # Modern consolidated GPIO management
│   ├── Pcal95555GpioWrapper.h/cpp # PCAL95555 I2C GPIO expander wrapper
│   ├── Tmc9660Gpio.h/cpp       # TMC9660 GPIO integration
│   ├── Tmc9660MotorController.h/cpp # TMC9660 motor controller with GPIO
│   ├── AdcManager.h/cpp        # Modern consolidated ADC management
│   ├── CommonIDs.h            # System-wide enumerations and constants
│   ├── SystemInit.h/cpp       # System initialization utilities
│   ├── GPIO_MANAGER_README.md # Complete GPIO manager documentation
│   ├── ADC_MANAGER_README.md  # Complete ADC manager documentation
│   └── All.h                  # Master include file
├── API/                        # Public API interfaces
│   ├── All.h                  # Public API master include
│   ├── HardFocIntegration.h/cpp # Integration examples
│   └── SystemInit.h/cpp       # Public system initialization
├── docs/                       # Documentation
│   ├── HARDFOC_GPIO_ADC_SYSTEM.md              # Comprehensive system documentation
│   ├── INTERNAL_INTERFACE_WRAPPER_DEEP_DIVE_ANALYSIS.md # Architecture analysis
│   ├── GPIO_ARCHITECTURE_FINAL.md              # GPIO architecture
│   ├── PWM_ARCHITECTURE.md                     # PWM architecture
│   ├── HARDFOC_HARDWARE_ABSTRACTION_ARCHITECTURE.md # Core hardware abstraction
│   ├── UNIFIED_GPIO_INTERRUPT_ARCHITECTURE.md  # Interrupt architecture
│   └── FINAL_IMPLEMENTATION_REPORT.md         # Implementation status
└── tests/                      # Consolidated test suite
    ├── README.md               # Test suite documentation
    ├── CMakeLists.txt          # Master test build configuration
    ├── component-handler/      # Component handler tests
    ├── interface-wrapper/      # Internal interface wrapper tests
    └── external-drivers/       # External driver tests
```

### Hardware Drivers
```
hf-hal/utils-and-drivers/hf-core-drivers/
├── external/                   # External device drivers
│   ├── hf-pcal95555-driver/   # PCAL95555 I2C GPIO expander
│   └── hf-tmc9660-driver/     # TMC9660 motor controller
└── internal/                   # Internal ESP32-C6 drivers
    ├── hf-internal-interface-wrap/ # Hardware abstraction layer
    │   ├── include/            # Header files organized by functionality
    │   │   ├── base/           # Abstract base classes
    │   │   ├── mcu/            # MCU-specific implementations
    │   │   ├── thread_safe/    # Thread-safe wrappers
    │   │   └── utils/          # Utility classes
    │   ├── src/                # Implementation files
    │   └── docs/               # Internal interface documentation
    └── hf-pincfg/             # Pin configuration and safety validation
```

## Quick Start Guide

### 1. Include the System
```cpp
#include "component-handler/All.h"
```

### 2. Initialize Hardware
```cpp
// Initialize communication channels
auto& commManager = CommChannelsManager::GetInstance();
commManager.EnsureInitialized();

// Initialize GPIO system
auto& gpioManager = GpioManager::GetInstance();  
gpioManager.EnsureInitialized();

// Initialize ADC system
auto& adcManager = AdcManager::GetInstance();
adcManager.Initialize();
```

### 3. Use GPIO and ADC
```cpp
// GPIO operations
gpioManager.SetPin(GPIO_ESP32_PIN_2, true);
bool state = gpioManager.GetPin(GPIO_PCAL95555_CHIP1_PIN_0);

// ADC operations
uint16_t rawValue = adcManager.ReadRaw(ADC_ESP32_ADC1_CH0);
float voltage = adcManager.ReadVoltage(ADC_ESP32_ADC1_CH0);
```

## System Features

### GPIO Sources Supported
- ✅ **ESP32-C6 Native GPIO**: 40+ pins with safety validation and platform mapping
- ✅ **PCAL95555 I2C Expanders**: Up to 2 chips (32 additional pins) with full I2C integration
- ✅ **TMC9660 Motor Controller**: Specialized GPIO pins for motor control with fault handling

### ADC Sources Supported
- ✅ **ESP32-C6 Internal ADC**: ADC1 and ADC2 units with calibration
- ✅ **TMC9660 Motor Controller**: AIN1, AIN2, AIN3 analog inputs

### Key Features
- 🔒 **Thread-safe operation** with mutex protection and atomic operations
- 🏥 **Health monitoring** for all hardware sources with detailed diagnostics
- ⚡ **Performance optimized** with batch operations and interrupt handling
- 🛡️ **Pin safety validation** for ESP32-C6 with platform mapping integration
- 🔧 **Comprehensive error handling** and diagnostics with fault recovery
- 📊 **Real-time system status** monitoring with chip-specific statistics
- 🔄 **GPIO interrupts** with edge-triggered callbacks and event handling
- 🎯 **Functional pin identifiers** with automatic hardware resource discovery

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
