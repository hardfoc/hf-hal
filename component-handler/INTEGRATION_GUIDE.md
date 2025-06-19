# HardFOC Component Handler Integration Guide

This guide explains how the GPIO handler and ADC handler now instantiate and manage their respective hardware drivers.

## Overview

The system now includes:

1. **GPIO Handler**: Instantiates and manages PCAL95555 GPIO expander chips
2. **ADC Handler**: Instantiates and manages ESP32-C6 internal ADC units
3. **System Integration**: Automatic hardware discovery and registration

## GPIO Handler Updates

### PCAL95555 Integration

The `GpioHandler` class now:

- **Instantiates PCAL95555 drivers** internally using `Pcal95555Chip` wrapper classes
- **Manages I2C communication** through the `SfI2cBus` interface
- **Registers all expander pins** automatically with the `GpioData` system
- **Provides transparent access** to expander pins through the unified GPIO API

#### Key Components:

```cpp
class GpioHandler {
    // Hardware instances
    std::unique_ptr<Pcal95555Chip> chip1_;  // PCAL95555 at 0x20
    std::unique_ptr<Pcal95555Chip> chip2_;  // PCAL95555 at 0x21
    
    // Initialization
    bool Initialize(SfI2cBus &i2cBus);
    bool InitializePcal95555Chips(SfI2cBus &i2cBus);
    bool RegisterPcal95555Pins();
};
```

#### Pin Mapping:

- **Chip 1 (0x20)**: `GPIO_PCAL95555_CHIP1_PIN_0` through `GPIO_PCAL95555_CHIP1_PIN_15`
- **Chip 2 (0x21)**: `GPIO_PCAL95555_CHIP2_PIN_0` through `GPIO_PCAL95555_CHIP2_PIN_15`

## ADC Handler Updates

### ESP32-C6 ADC Integration

The `AdcHandler` class now:

- **Instantiates ESP32-C6 ADC drivers** using `Esp32C6Adc` class
- **Manages both ADC units** (ADC_UNIT_1 and ADC_UNIT_2)
- **Registers all internal channels** automatically with the `AdcData` system
- **Provides transparent access** to ESP32-C6 ADC channels through the unified ADC API

#### Key Components:

```cpp
class AdcHandler {
    // Hardware instances
    std::unique_ptr<Esp32C6Adc> esp32c6_adc1_;  // ADC Unit 1
    std::unique_ptr<Esp32C6Adc> esp32c6_adc2_;  // ADC Unit 2
    
    // Initialization
    bool Initialize();
    bool InitializeEsp32C6Adcs();
    bool RegisterEsp32C6Channels();
};
```

#### Channel Mapping:

- **ADC Unit 1**: `ADC_INTERNAL_CH0` through `ADC_INTERNAL_CH6` (ADC1_CHANNEL_0-6)
- **ADC Unit 2**: `ADC_INTERNAL_CH7` through `ADC_INTERNAL_CH12` (ADC2_CHANNEL_0-5)

## System Initialization

### Updated SystemInit

The `SystemInit` class now:

- **Creates I2C bus instance** for GPIO expanders
- **Initializes hardware drivers** automatically
- **Registers all available pins/channels** with the data systems

#### Initialization Flow:

1. **I2C Bus Setup**: Creates `SfI2cBus` instance for PCAL95555 communication
2. **GPIO Handler Init**: Instantiates PCAL95555 chips and registers pins
3. **ADC Handler Init**: Instantiates ESP32-C6 ADCs and registers channels
4. **Functional Mapping**: Maps logical names to hardware resources

## Usage Examples

### ADC Usage

```cpp
#include "component-handler/AdcData.h"

// Get ADC system instance
AdcData& adc = AdcData::GetInstance();

// Read motor current (mapped to internal ADC channel)
float current_a;
if (adc.GetVolt(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, current_a, 10, 1)) {
    printf("Motor Current A: %.3f V\n", current_a);
}

// Read system voltage (mapped to internal ADC channel)  
uint32_t count;
float voltage;
if (adc.GetCountAndVolt(AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3, count, voltage, 5, 2)) {
    printf("3.3V Rail: %lu counts, %.3f V\n", count, voltage);
}
```

### GPIO Usage

```cpp
#include "component-handler/GpioData.h"

// Get GPIO system instance
GpioData& gpio = GpioData::GetInstance();

// Control motor enable (mapped to PCAL95555 pin)
gpio.SetActive(GpioPin::GPIO_MOTOR_ENABLE);

// Read fault status (mapped to PCAL95555 pin)
bool fault = gpio.IsActive(GpioPin::GPIO_MOTOR_FAULT);

// Control status LED (mapped to PCAL95555 pin)
gpio.SetActive(GpioPin::GPIO_LED_STATUS);
```

### Integration in main.cpp

```cpp
#include "component-handler/HardFocIntegration.h"

extern "C" void app_main(void) {
    // Initialize hardware pin configuration
    init_mcu_pinconfig();
    
    // Initialize HardFOC system with hardware drivers
    if (!HardFocIntegration::Initialize()) {
        console_error("main", "Failed to initialize HardFOC system");
        return;
    }
    
    // System is now ready - all hardware is instantiated and registered
    
    while (true) {
        // Run periodic maintenance
        RunPeriodicSystemMaintenance();
        
        // Demo usage
        HardFocIntegration::DemoAdcUsage();
        HardFocIntegration::DemoGpioUsage();
        
        os_delay_msec(1000);
    }
}
```

## Hardware Configuration

### I2C Configuration

- **Bus**: I2C_NUM_0
- **SDA Pin**: Defined in `hf_gpio_config.hpp` as `I2C_SDA_PIN`
- **SCL Pin**: Defined in `hf_gpio_config.hpp` as `I2C_SCL_PIN` 
- **Speed**: 100kHz
- **Pull-ups**: Enabled

### ADC Configuration

- **Resolution**: 12-bit (ADC_BITWIDTH_12)
- **Attenuation**: 11dB (ADC_ATTEN_DB_11) for 0-3.1V range
- **Units**: Both ADC_UNIT_1 and ADC_UNIT_2

### PCAL95555 Configuration

- **Chip 1 Address**: 0x20
- **Chip 2 Address**: 0x21
- **Pin Count**: 16 pins per chip (32 total)
- **Direction**: Configurable per pin

## Benefits

1. **Automatic Hardware Management**: No manual driver instantiation required
2. **Unified Access**: Single API for all GPIO/ADC resources
3. **Runtime Discovery**: Hardware automatically detected and registered
4. **Error Handling**: Comprehensive error checking and health monitoring
5. **Backward Compatibility**: Legacy handler interfaces still supported
6. **Thread Safety**: All operations are thread-safe
7. **Multi-chip Support**: Handles multiple PCAL95555 and ADC instances

## Migration Notes

For existing code using the old system:

1. **No Changes Required**: Legacy interfaces are preserved
2. **Enhanced Features**: New multi-channel/multi-pin operations available
3. **Better Diagnostics**: Improved error reporting and health monitoring
4. **Automatic Initialization**: Hardware setup is now handled automatically

The new system is fully backward compatible while providing enhanced functionality and better hardware abstraction.
