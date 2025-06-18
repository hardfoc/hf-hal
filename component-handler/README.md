# HardFOC Component Handler System

A comprehensive ADC and GPIO management system designed for the HardFOC motor controller based on ESP32-C6 and external chips like PCAL95555 GPIO expanders.

## Overview

This system provides a unified, thread-safe interface for all ADC and GPIO operations in the HardFOC system. It follows the same architectural patterns as the ConMed AdcData system but is adapted for embedded ESP32-C6 development with ESP-IDF.

## Key Features

- **Thread-safe singleton pattern** for ADC and GPIO management
- **Unified interface** for internal ESP32-C6 peripherals and external chips
- **Comprehensive error handling** and health monitoring
- **Multi-channel operations** for efficient sensor reading
- **Flexible time unit support** (microseconds, milliseconds, seconds)
- **Backward compatibility** with legacy AdcHandler/GpioHandler interfaces
- **Extensive logging and diagnostics** capabilities
- **Runtime configuration** with compile-time optimization

## Architecture

### Core Components

1. **AdcData** - Main ADC management singleton
2. **GpioData** - Main GPIO management singleton  
3. **SystemInit** - System initialization and configuration
4. **HardFocIntegration** - Application integration layer
5. **CommonIDs** - Enumerations for all sensors and pins
6. **MultiReadings** - Template for efficient multi-channel operations

### Supported Hardware

#### ADC Sources
- ESP32-C6 internal ADC channels
- External SPI ADC chips (e.g., ADS7952)
- External I2C ADC chips

#### GPIO Sources  
- ESP32-C6 native GPIO pins
- PCAL95555 I2C GPIO expanders
- Other digital I/O expanders

## Quick Start

### Basic Setup

```cpp
#include "component-handler/All.h"

void app_main() {
    // Initialize the entire system
    if (!HARDFOC_INIT()) {
        console_error("main", "Failed to initialize HardFOC system");
        return;
    }
    
    // Main loop
    while (true) {
        // Your application code here
        
        // Periodic maintenance
        HARDFOC_MAINTAIN();
        
        // Check system health
        if (!HARDFOC_HEALTHY()) {
            console_warning("main", "System health degraded");
        }
        
        os_delay_msec(100);
    }
}
```

### ADC Usage

```cpp
using namespace HardFocComponentHandler;

// Single channel reading
AdcSystem& adc = GetAdcSystem();
float voltage;
uint32_t count;

// Read motor current
if (adc.GetVolt(AdcSensor::ADC_MOTOR_CURRENT_PHASE_A, voltage, 10, 1)) {
    console_info("main", "Motor current A: %.3f V", voltage);
}

// Read with both count and voltage
if (adc.GetCountAndVolt(AdcSensor::ADC_MOTOR_VOLTAGE_BUS, count, voltage, 5, 2)) {
    console_info("main", "Bus voltage: %lu counts, %.3f V", count, voltage);
}

// Multi-channel reading
std::vector<AdcInputSensorReadSpec> sensors = {
    {AdcSensor::ADC_MOTOR_CURRENT_PHASE_A, 10},
    {AdcSensor::ADC_MOTOR_CURRENT_PHASE_B, 10},
    {AdcSensor::ADC_MOTOR_CURRENT_PHASE_C, 10}
};

if (adc.GetMultiVolt(sensors, 3, 5, TimeUnit::TIME_UNIT_MS)) {
    for (const auto& sensor : sensors) {
        console_info("main", "%s: %.3f V", 
                    AdcInputSensorToString(sensor.sensor).data(),
                    sensor.channelAvgReadingVoltageStorage);
    }
}
```

### GPIO Usage

```cpp
using namespace HardFocComponentHandler;

GpioSystem& gpio = GetGpioSystem();

// Single pin operations
gpio.SetActive(GpioPin::GPIO_MOTOR_ENABLE);
gpio.SetInactive(GpioPin::GPIO_MOTOR_BRAKE);

if (gpio.IsActive(GpioPin::GPIO_MOTOR_FAULT)) {
    console_warning("main", "Motor fault detected!");
}

// Multi-pin operations
std::vector<GpioPin> leds = {
    GpioPin::GPIO_LED_STATUS,
    GpioPin::GPIO_LED_COMM,
    GpioPin::GPIO_LED_ERROR
};

// Set LED pattern (binary: 101 = status and error on, comm off)
gpio.SetPinPattern(leds, 0b101);

// Read current pattern
uint32_t pattern;
if (gpio.GetPinPattern(leds, pattern)) {
    console_info("main", "LED pattern: 0x%02lX", pattern);
}

// Convenience functions
gpio.PulsePin(GpioPin::GPIO_USER_OUTPUT_1, 100); // 100ms pulse
gpio.BlinkPin(GpioPin::GPIO_LED_STATUS, 500, 500, 3); // Blink 3 times
```

## Configuration

### Pin Mapping

Pin assignments are defined in:
- `hf_gpio_config.hpp` - ESP32-C6 native pins
- `hf_ext_pins_enum.hpp` - External GPIO expander pins

### ADC Channel Mapping

ADC channels are mapped in `CommonIDs.h` using the `AdcInputSensor` enum:

```cpp
enum class AdcInputSensor : uint8_t {
    // Motor control
    ADC_MOTOR_CURRENT_PHASE_A,
    ADC_MOTOR_CURRENT_PHASE_B,
    ADC_MOTOR_CURRENT_PHASE_C,
    ADC_MOTOR_VOLTAGE_BUS,
    ADC_MOTOR_TEMPERATURE,
    
    // System monitoring
    ADC_SYSTEM_VOLTAGE_3V3,
    ADC_SYSTEM_VOLTAGE_5V,
    ADC_SYSTEM_VOLTAGE_12V,
    
    // User expansion
    ADC_USER_INPUT_1,
    ADC_USER_INPUT_2,
    // ...
};
```

### GPIO Pin Mapping

GPIO pins are mapped using the `GpioPin` enum:

```cpp
enum class GpioPin : uint8_t {
    // ESP32-C6 native pins
    GPIO_ESP32_PIN_0,
    GPIO_ESP32_PIN_1,
    // ...
    
    // PCAL95555 expander pins
    GPIO_EXT_PIN_0,
    GPIO_EXT_PIN_1,
    // ...
    
    // Functional mappings
    GPIO_MOTOR_ENABLE,
    GPIO_MOTOR_FAULT,
    GPIO_LED_STATUS,
    // ...
};
```

## Advanced Usage

### Custom Initialization

```cpp
// Custom initialization sequence
SystemInit::InitializeGpioSystem();
SystemInit::InitializeAdcSystem();
SystemInit::RegisterNativeGpioPins();
SystemInit::RegisterExpanderGpioPins();
SystemInit::RegisterInternalAdcChannels();
SystemInit::RegisterExternalAdcChannels();
```

### Health Monitoring

```cpp
// Comprehensive health check
if (SystemInit::GetSystemHealth()) {
    console_info("main", "All systems operational");
} else {
    SystemInit::PrintSystemStatus();
    SystemInit::RunSystemSelfTest();
}

// Individual component health
AdcData& adc = AdcData::GetInstance();
GpioData& gpio = GpioData::GetInstance();

if (!adc.IsResponding(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A)) {
    console_error("main", "Motor current sensor not responding");
}

if (!gpio.IsResponding(GpioPin::GPIO_MOTOR_ENABLE)) {
    console_error("main", "Motor enable pin not responding");
}
```

### Error Handling

```cpp
AdcData& adc = AdcData::GetInstance();
float voltage;

// Robust error handling
if (!adc.GetVolt(AdcInputSensor::ADC_MOTOR_TEMPERATURE, voltage, 5, 2)) {
    console_error("main", "Failed to read motor temperature");
    
    // Check if sensor is responding
    if (!adc.IsResponding(AdcInputSensor::ADC_MOTOR_TEMPERATURE)) {
        console_error("main", "Motor temperature sensor not responding");
        // Handle sensor failure
    }
    
    // Check ADC chip communication
    if (!adc.IsCommunicating(AdcChip::ADC_ESP32_INTERNAL)) {
        console_error("main", "Internal ADC not communicating");
        // Handle ADC failure
    }
}
```

## Migration from Legacy System

### From Old AdcHandler

```cpp
// Old way
AdcHandler<32>& adc = AdcHandler<32>::Instance();
adc.RegisterChannel("motor_current_a", adcDriver, 0);
float voltage;
adc.ReadChannelV("motor_current_a", voltage);

// New way
AdcData& adc = AdcData::GetInstance();
adc.RegisterAdcChannel(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, adcDriver, 0);
float voltage;
adc.GetVolt(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, voltage);
```

### From Old GpioHandler

```cpp
// Old way  
GpioHandler<64>& gpio = GpioHandler<64>::Instance();
gpio.RegisterPin("motor_enable", gpioDriver);
gpio.GetPin("motor_enable")->SetActive();

// New way
GpioData& gpio = GpioData::GetInstance();
gpio.RegisterGpioPin(GpioPin::GPIO_MOTOR_ENABLE, gpioDriver, "motor_enable");
gpio.SetActive(GpioPin::GPIO_MOTOR_ENABLE);
```

## Debugging and Diagnostics

### System Status

```cpp
// Print comprehensive system status
SystemInit::PrintSystemStatus();

// Run full system test
if (SystemInit::RunSystemSelfTest()) {
    console_info("main", "All system tests passed");
}

// Run component-specific tests
GpioData::GetInstance().RunGpioTest();
```

### Logging Levels

The system uses different log levels:
- `console_info()` - Normal operation messages
- `console_warning()` - Non-critical issues
- `console_error()` - Critical errors
- Debug messages can be enabled/disabled per component

## Performance Considerations

- **Thread Safety**: All operations are thread-safe but may block briefly
- **Memory Usage**: Static allocation, no dynamic memory allocation
- **Timing**: Multi-channel operations are more efficient than individual calls
- **Caching**: System caches initialization status to avoid repeated setup

## Thread Safety

All components are designed to be thread-safe:
- Mutex protection for shared data structures
- Atomic operations for status flags
- Lock-free operations where possible
- Deadlock prevention through consistent lock ordering

## Future Enhancements

- SPI ADC chip drivers integration
- I2C ADC chip drivers integration  
- Additional GPIO expander support
- DMA-based multi-channel ADC reading
- Interrupt-driven GPIO change detection
- Power management integration
- Real-time performance optimizations

## License

This code is part of the HardFOC project. See LICENSE file for details.
