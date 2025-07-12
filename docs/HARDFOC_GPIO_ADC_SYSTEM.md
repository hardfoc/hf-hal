# HardFOC GPIO and ADC Data Sourcing System

> **Comprehensive documentation for the HardFOC multi-source GPIO and ADC management system**

[![ESP32-C6](https://img.shields.io/badge/Platform-ESP32--C6-blue.svg)](https://www.espressif.com/en/products/socs/esp32-c6)
[![Thread-Safe](https://img.shields.io/badge/Design-Thread--Safe-green.svg)]()
[![Multi-Source](https://img.shields.io/badge/GPIO-Multi--Source-orange.svg)]()

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [GPIO Sources](#gpio-sources)
4. [ADC Sources](#adc-sources)
5. [Core Classes](#core-classes)
6. [Usage Examples](#usage-examples)
7. [Health Monitoring](#health-monitoring)
8. [Integration Guide](#integration-guide)
9. [Pin Mappings](#pin-mappings)
10. [Error Handling](#error-handling)
11. [Performance Considerations](#performance-considerations)
12. [Troubleshooting](#troubleshooting)

## System Overview

The HardFOC GPIO and ADC system provides a unified, thread-safe interface for managing GPIO pins and ADC channels from multiple hardware sources. The system is designed around a clean separation of concerns:

- **GpioData**: Pure data management class - handles storage, retrieval, and generic operations
- **GpioHandler**: Main interface class - handles initialization, validation, health monitoring, and source-specific logic
- **AdcData**: Pure data management for ADC channels
- **AdcHandler**: Main interface for ADC operations

### Supported Hardware Sources

#### GPIO Sources
- **ESP32-C6**: Native GPIO pins (40+ pins available)
- **PCAL95555**: I2C GPIO expander (up to 2 chips, 16 pins each)
- **TMC9660**: Motor controller GPIO pins (GPIO17, GPIO18)

#### ADC Sources
- **ESP32-C6**: Internal ADC units (ADC1, ADC2)
- **TMC9660**: Motor controller ADC channels (AIN1, AIN2, AIN3)

## Architecture

### System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        Application Layer                        │
├─────────────────────────────────────────────────────────────────┤
│                  GpioHandler / AdcHandler                      │
│              (Main Interface & Control Logic)                  │
├─────────────────────────────────────────────────────────────────┤
│                   GpioData / AdcData                           │
│                 (Pure Data Management)                         │
├─────────────────────────────────────────────────────────────────┤
│                    Hardware Drivers                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ...        │
│  │   ESP32-C6  │  │  PCAL95555  │  │   TMC9660   │  ...        │
│  │   (Native)  │  │ (I2C Exp.)  │  │  (Motor)    │  ...        │
│  └─────────────┘  └─────────────┘  └─────────────┘  ...        │
└─────────────────────────────────────────────────────────────────┘
```

### Class Responsibilities

#### GpioData (Pure Data Class)
- **Storage**: GPIO pin information, readings, configurations
- **Retrieval**: Thread-safe access to GPIO data
- **Batch Operations**: Efficient multi-pin operations
- **Generic Health**: Basic system statistics (no source-specific logic)

#### GpioHandler (Main Interface)
- **Initialization**: Hardware setup and registration
- **Validation**: Pin safety and source verification
- **Health Monitoring**: Source-specific health tracking
- **Error Handling**: Communication error management
- **Public API**: Application interface methods

## GPIO Sources

### 1. ESP32-C6 Native GPIO

The ESP32-C6 provides 40+ GPIO pins with various capabilities:

#### Available Pins
```cpp
// Standard GPIO pins
GPIO_ESP32_PIN_0  through GPIO_ESP32_PIN_23
GPIO_ESP32_PIN_48 through GPIO_ESP32_PIN_54

// Special function pins (use with caution)
GPIO_ESP32_PIN_24 through GPIO_ESP32_PIN_47
```

#### Pin Capabilities
- **Digital I/O**: All pins support digital input/output
- **Analog Input**: Selected pins support ADC functionality
- **PWM Output**: Most pins support PWM generation
- **Communication**: Dedicated pins for SPI, I2C, UART

#### Hardware Configuration
```cpp
// Native ESP32 pins require no additional setup
// Pins are directly accessible through ESP-IDF GPIO driver
```

### 2. PCAL95555 I2C GPIO Expander

The system supports up to 2 PCAL95555 chips, providing 32 additional GPIO pins:

#### Chip Configuration
- **Chip 1**: I2C address 0x20 (16 pins: P0_0 to P1_7)
- **Chip 2**: I2C address 0x21 (16 pins: P0_0 to P1_7)

#### Pin Mapping
```cpp
// Chip 1 pins
GPIO_PCAL95555_CHIP1_PIN_0  // P0_0
GPIO_PCAL95555_CHIP1_PIN_1  // P0_1
...
GPIO_PCAL95555_CHIP1_PIN_15 // P1_7

// Chip 2 pins  
GPIO_PCAL95555_CHIP2_PIN_0  // P0_0
GPIO_PCAL95555_CHIP2_PIN_1  // P0_1
...
GPIO_PCAL95555_CHIP2_PIN_15 // P1_7
```

#### Hardware Setup
```cpp
// I2C configuration
I2C_SCL: GPIO21
I2C_SDA: GPIO20
I2C_FREQ: 400kHz
```

### 3. TMC9660 Motor Controller GPIO

The TMC9660 provides specialized GPIO pins for motor control applications:

#### Available Pins
```cpp
GPIO_TMC9660_CHIP1_GPIO17  // TMC9660 GPIO17
GPIO_TMC9660_CHIP1_GPIO18  // TMC9660 GPIO18
```

#### Control Signals (via PCAL95555)
```cpp
TMC_nFAULT_STATUS  // P0_3 - Fault status (input, active low)
TMC_DRV_EN         // P0_4 - Driver enable (output)
TMC_RST_CTRL       // P0_5 - Reset control (output)
TMC_nWAKE_CTRL     // P1_6 - Wake control (output, active low)
TMC_SPI_COMM_nEN   // P1_5 - SPI enable (output, active low)
```

## ADC Sources

### 1. ESP32-C6 Internal ADC

The ESP32-C6 provides two ADC units with multiple channels:

#### ADC Unit 1
```cpp
ADC_ESP32_ADC1_CH0  // GPIO0
ADC_ESP32_ADC1_CH1  // GPIO1
ADC_ESP32_ADC1_CH2  // GPIO2
ADC_ESP32_ADC1_CH3  // GPIO3
ADC_ESP32_ADC1_CH4  // GPIO4
ADC_ESP32_ADC1_CH5  // GPIO5
ADC_ESP32_ADC1_CH6  // GPIO6
```

#### ADC Unit 2
```cpp
ADC_ESP32_ADC2_CH0  // GPIO7
ADC_ESP32_ADC2_CH1  // GPIO8
ADC_ESP32_ADC2_CH2  // GPIO9
ADC_ESP32_ADC2_CH3  // GPIO10
ADC_ESP32_ADC2_CH4  // GPIO11
ADC_ESP32_ADC2_CH5  // GPIO12
```

### 2. TMC9660 Motor Controller ADC

The TMC9660 provides 3 analog input channels for motor sensing:

#### Available Channels
```cpp
ADC_TMC9660_CHIP1_AIN1  // Analog input 1
ADC_TMC9660_CHIP1_AIN2  // Analog input 2  
ADC_TMC9660_CHIP1_AIN3  // Analog input 3
```

## Core Classes

### GpioData Class

Pure data management class with no hardware-specific logic:

```cpp
class GpioData {
public:
    // Data access methods
    bool GetPinState(GpioPin pin) const;
    bool SetPinState(GpioPin pin, bool state);
    
    // Batch operations
    std::vector<bool> GetMultiplePinStates(const std::vector<GpioPin>& pins) const;
    bool SetMultiplePinStates(const std::vector<GpioPin>& pins, 
                             const std::vector<bool>& states);
    
    // Generic health information
    GpioSystemHealth GetSystemHealth() const;
    
    // Thread-safe singleton access
    static GpioData& GetInstance();
};
```

### GpioHandler Class

Main interface with initialization and health monitoring:

```cpp
class GpioHandler {
public:
    // Initialization
    bool Initialize(SfI2cBus& i2cBus);
    bool InitializeEsp32Gpio();
    bool InitializePcal95555(SfI2cBus& i2cBus);
    bool InitializeTmc9660(Tmc9660MotorController& controller);
    
    // Health monitoring
    bool IsEsp32GpioHealthy() const;
    bool IsPcal95555Healthy() const;
    bool IsTmc9660GpioHealthy() const;
    bool IsSystemHealthy() const;
    
    // Pin operations (delegates to GpioData)
    bool SetPin(GpioPin pin, bool state);
    bool GetPin(GpioPin pin) const;
    
    // Thread-safe singleton access
    static GpioHandler& GetInstance();
};
```

### AdcData Class

Pure data management for ADC operations:

```cpp
class AdcData {
public:
    // Reading methods
    uint16_t GetRawReading(AdcChannel channel) const;
    float GetVoltageReading(AdcChannel channel) const;
    
    // Multi-channel operations
    std::vector<uint16_t> GetMultipleRawReadings(
        const std::vector<AdcChannel>& channels) const;
    
    // Generic health information
    AdcSystemHealth GetSystemHealth() const;
    
    // Thread-safe singleton access
    static AdcData& GetInstance();
};
```

### AdcHandler Class

Main interface for ADC operations:

```cpp
class AdcHandler {
public:
    // Initialization
    bool Initialize();
    bool InitializeEsp32Adc();
    bool InitializeTmc9660Adc(Tmc9660MotorController& controller);
    
    // Health monitoring
    bool IsEsp32AdcHealthy() const;
    bool IsTmc9660AdcHealthy() const;
    bool IsSystemHealthy() const;
    
    // Reading operations (delegates to AdcData)
    uint16_t ReadRaw(AdcChannel channel);
    float ReadVoltage(AdcChannel channel);
    
    // Thread-safe singleton access
    static AdcHandler& GetInstance();
};
```

## Usage Examples

### Basic GPIO Operations

```cpp
#include "component-handler/All.h"

void BasicGpioExample() {
    // Get handler instances
    auto& gpioHandler = GpioHandler::GetInstance();
    auto& i2cBus = SfI2cBus::GetInstance();
    
    // Initialize the system
    if (!gpioHandler.Initialize(i2cBus)) {
        ESP_LOGE("GPIO", "Failed to initialize GPIO system");
        return;
    }
    
    // Set an ESP32 pin
    gpioHandler.SetPin(GPIO_ESP32_PIN_2, true);
    
    // Set a PCAL95555 pin
    gpioHandler.SetPin(GPIO_PCAL95555_CHIP1_PIN_0, false);
    
    // Read pin states
    bool esp32State = gpioHandler.GetPin(GPIO_ESP32_PIN_2);
    bool pcal95555State = gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_0);
    
    ESP_LOGI("GPIO", "ESP32 pin state: %d", esp32State);
    ESP_LOGI("GPIO", "PCAL95555 pin state: %d", pcal95555State);
}
```

### Batch GPIO Operations

```cpp
void BatchGpioExample() {
    auto& gpioData = GpioData::GetInstance();
    
    // Prepare pin lists
    std::vector<GpioPin> pins = {
        GPIO_ESP32_PIN_0,
        GPIO_ESP32_PIN_1,
        GPIO_PCAL95555_CHIP1_PIN_0,
        GPIO_PCAL95555_CHIP1_PIN_1
    };
    
    std::vector<bool> states = {true, false, true, false};
    
    // Set multiple pins at once
    if (gpioData.SetMultiplePinStates(pins, states)) {
        ESP_LOGI("GPIO", "Batch write successful");
    }
    
    // Read multiple pins at once
    auto readStates = gpioData.GetMultiplePinStates(pins);
    for (size_t i = 0; i < pins.size(); ++i) {
        ESP_LOGI("GPIO", "Pin %d state: %d", 
                static_cast<int>(pins[i]), readStates[i]);
    }
}
```

### ADC Operations

```cpp
void AdcExample() {
    auto& adcHandler = AdcHandler::GetInstance();
    auto& tmc9660 = Tmc9660MotorController::GetInstance();
    
    // Initialize ADC system
    if (!adcHandler.Initialize()) {
        ESP_LOGE("ADC", "Failed to initialize ADC system");
        return;
    }
    
    // Initialize TMC9660 ADC
    if (!adcHandler.InitializeTmc9660Adc(tmc9660)) {
        ESP_LOGE("ADC", "Failed to initialize TMC9660 ADC");
        return;
    }
    
    // Read ESP32 ADC channel
    uint16_t esp32Raw = adcHandler.ReadRaw(ADC_ESP32_ADC1_CH0);
    float esp32Voltage = adcHandler.ReadVoltage(ADC_ESP32_ADC1_CH0);
    
    // Read TMC9660 ADC channel
    uint16_t tmcRaw = adcHandler.ReadRaw(ADC_TMC9660_CHIP1_AIN1);
    float tmcVoltage = adcHandler.ReadVoltage(ADC_TMC9660_CHIP1_AIN1);
    
    ESP_LOGI("ADC", "ESP32 ADC: %d raw, %.3f V", esp32Raw, esp32Voltage);
    ESP_LOGI("ADC", "TMC9660 ADC: %d raw, %.3f V", tmcRaw, tmcVoltage);
}
```

### TMC9660 Integration

```cpp
void Tmc9660Example() {
    auto& gpioHandler = GpioHandler::GetInstance();
    auto& tmc9660 = Tmc9660MotorController::GetInstance();
    auto& i2cBus = SfI2cBus::GetInstance();
    
    // Initialize systems
    gpioHandler.Initialize(i2cBus);
    
    // Initialize TMC9660 GPIO
    if (!gpioHandler.InitializeTmc9660(tmc9660)) {
        ESP_LOGE("TMC", "Failed to initialize TMC9660 GPIO");
        return;
    }
    
    // Enable TMC9660 driver
    gpioHandler.SetPin(GPIO_PCAL95555_CHIP1_PIN_4, true); // TMC_DRV_EN
    
    // Check fault status
    bool faultActive = !gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_3); // TMC_nFAULT_STATUS
    
    if (faultActive) {
        ESP_LOGW("TMC", "TMC9660 fault detected!");
        // Handle fault condition
    }
    
    // Use TMC9660 GPIO pins
    gpioHandler.SetPin(GPIO_TMC9660_CHIP1_GPIO17, true);
    gpioHandler.SetPin(GPIO_TMC9660_CHIP1_GPIO18, false);
}
```

## Health Monitoring

### System Health Overview

The system provides comprehensive health monitoring at multiple levels:

```cpp
struct GpioSystemHealth {
    uint32_t totalRegisteredPins;        // Total pins registered
    uint32_t totalCommunicationErrors;   // Generic communication errors
    uint64_t lastHealthCheckTime;        // Last health check timestamp
};

struct AdcSystemHealth {
    uint32_t totalRegisteredChannels;    // Total channels registered
    uint32_t totalReadingErrors;         // Generic reading errors
    uint64_t lastHealthCheckTime;        // Last health check timestamp
};
```

### Source-Specific Health Monitoring

GpioHandler and AdcHandler track health for each hardware source:

```cpp
void MonitorSystemHealth() {
    auto& gpioHandler = GpioHandler::GetInstance();
    auto& adcHandler = AdcHandler::GetInstance();
    
    // Check GPIO health
    if (!gpioHandler.IsSystemHealthy()) {
        ESP_LOGW("HEALTH", "GPIO system unhealthy");
        
        if (!gpioHandler.IsEsp32GpioHealthy()) {
            ESP_LOGW("HEALTH", "ESP32 GPIO issues detected");
        }
        
        if (!gpioHandler.IsPcal95555Healthy()) {
            ESP_LOGW("HEALTH", "PCAL95555 communication issues");
        }
        
        if (!gpioHandler.IsTmc9660GpioHealthy()) {
            ESP_LOGW("HEALTH", "TMC9660 GPIO issues");
        }
    }
    
    // Check ADC health
    if (!adcHandler.IsSystemHealthy()) {
        ESP_LOGW("HEALTH", "ADC system unhealthy");
        
        if (!adcHandler.IsEsp32AdcHealthy()) {
            ESP_LOGW("HEALTH", "ESP32 ADC issues detected");
        }
        
        if (!adcHandler.IsTmc9660AdcHealthy()) {
            ESP_LOGW("HEALTH", "TMC9660 ADC issues");
        }
    }
    
    // Get detailed health information
    auto gpioHealth = GpioData::GetInstance().GetSystemHealth();
    auto adcHealth = AdcData::GetInstance().GetSystemHealth();
    
    ESP_LOGI("HEALTH", "GPIO: %d pins, %d errors", 
            gpioHealth.totalRegisteredPins, 
            gpioHealth.totalCommunicationErrors);
    
    ESP_LOGI("HEALTH", "ADC: %d channels, %d errors", 
            adcHealth.totalRegisteredChannels, 
            adcHealth.totalReadingErrors);
}
```

## Integration Guide

### Step 1: Include Headers

```cpp
#include "component-handler/All.h"
// Or individually:
#include "component-handler/GpioHandler.h"
#include "component-handler/AdcHandler.h"
#include "component-handler/CommonIDs.h"
```

### Step 2: Initialize I2C Bus

```cpp
void InitializeI2C() {
    auto& i2cBus = SfI2cBus::GetInstance();
    
    // Configure I2C parameters
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 20;  // GPIO20
    conf.scl_io_num = 21;  // GPIO21
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;  // 400kHz
    
    if (!i2cBus.Initialize(I2C_NUM_0, &conf)) {
        ESP_LOGE("I2C", "Failed to initialize I2C bus");
    }
}
```

### Step 3: Initialize GPIO System

```cpp
void InitializeGpioSystem() {
    auto& gpioHandler = GpioHandler::GetInstance();
    auto& i2cBus = SfI2cBus::GetInstance();
    
    // Initialize complete GPIO system
    if (!gpioHandler.Initialize(i2cBus)) {
        ESP_LOGE("INIT", "Failed to initialize GPIO system");
        return;
    }
    
    ESP_LOGI("INIT", "GPIO system initialized successfully");
}
```

### Step 4: Initialize ADC System

```cpp
void InitializeAdcSystem() {
    auto& adcHandler = AdcHandler::GetInstance();
    
    // Initialize complete ADC system
    if (!adcHandler.Initialize()) {
        ESP_LOGE("INIT", "Failed to initialize ADC system");
        return;
    }
    
    ESP_LOGI("INIT", "ADC system initialized successfully");
}
```

### Step 5: Initialize TMC9660 (Optional)

```cpp
void InitializeTmc9660System() {
    auto& gpioHandler = GpioHandler::GetInstance();
    auto& adcHandler = AdcHandler::GetInstance();
    auto& tmc9660 = Tmc9660MotorController::GetInstance();
    
    // Initialize TMC9660 motor controller
    if (!tmc9660.Initialize()) {
        ESP_LOGE("TMC", "Failed to initialize TMC9660");
        return;
    }
    
    // Initialize TMC9660 GPIO
    if (!gpioHandler.InitializeTmc9660(tmc9660)) {
        ESP_LOGE("TMC", "Failed to initialize TMC9660 GPIO");
        return;
    }
    
    // Initialize TMC9660 ADC
    if (!adcHandler.InitializeTmc9660Adc(tmc9660)) {
        ESP_LOGE("TMC", "Failed to initialize TMC9660 ADC");
        return;
    }
    
    ESP_LOGI("TMC", "TMC9660 system initialized successfully");
}
```

## Pin Mappings

### ESP32-C6 GPIO Pin Map

| Pin ID | GPIO Number | Special Functions | Notes |
|--------|-------------|-------------------|-------|
| GPIO_ESP32_PIN_0 | GPIO0 | ADC1_CH0, Boot | Boot pin, use carefully |
| GPIO_ESP32_PIN_1 | GPIO1 | ADC1_CH1 | General purpose |
| GPIO_ESP32_PIN_2 | GPIO2 | ADC1_CH2 | General purpose |
| GPIO_ESP32_PIN_3 | GPIO3 | ADC1_CH3 | General purpose |
| GPIO_ESP32_PIN_4 | GPIO4 | ADC1_CH4, UART TX | UART0 default |
| GPIO_ESP32_PIN_5 | GPIO5 | ADC1_CH5, UART RX | UART0 default |
| GPIO_ESP32_PIN_6 | GPIO6 | ADC1_CH6 | General purpose |
| GPIO_ESP32_PIN_7 | GPIO7 | ADC2_CH0 | General purpose |
| ... | ... | ... | ... |
| GPIO_ESP32_PIN_20 | GPIO20 | I2C SDA | Default I2C SDA |
| GPIO_ESP32_PIN_21 | GPIO21 | I2C SCL | Default I2C SCL |

### PCAL95555 Pin Map

#### Chip 1 (I2C Address 0x20)

| Pin ID | Port.Pin | Description | Usage |
|--------|----------|-------------|-------|
| GPIO_PCAL95555_CHIP1_PIN_0 | P0.0 | TMC_GPIO17 | TMC9660 GPIO17 |
| GPIO_PCAL95555_CHIP1_PIN_1 | P0.1 | TMC_GPIO18 | TMC9660 GPIO18 |
| GPIO_PCAL95555_CHIP1_PIN_2 | P0.2 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_3 | P0.3 | TMC_nFAULT_STATUS | TMC9660 fault input |
| GPIO_PCAL95555_CHIP1_PIN_4 | P0.4 | TMC_DRV_EN | TMC9660 driver enable |
| GPIO_PCAL95555_CHIP1_PIN_5 | P0.5 | TMC_RST_CTRL | TMC9660 reset control |
| GPIO_PCAL95555_CHIP1_PIN_6 | P0.6 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_7 | P0.7 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_8 | P1.0 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_9 | P1.1 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_10 | P1.2 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_11 | P1.3 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_12 | P1.4 | General | Available |
| GPIO_PCAL95555_CHIP1_PIN_13 | P1.5 | TMC_SPI_COMM_nEN | TMC9660 SPI enable |
| GPIO_PCAL95555_CHIP1_PIN_14 | P1.6 | TMC_nWAKE_CTRL | TMC9660 wake control |
| GPIO_PCAL95555_CHIP1_PIN_15 | P1.7 | General | Available |

#### Chip 2 (I2C Address 0x21)

| Pin ID | Port.Pin | Description | Usage |
|--------|----------|-------------|-------|
| GPIO_PCAL95555_CHIP2_PIN_0 | P0.0 | General | Available |
| GPIO_PCAL95555_CHIP2_PIN_1 | P0.1 | General | Available |
| ... | ... | ... | ... |
| GPIO_PCAL95555_CHIP2_PIN_15 | P1.7 | General | Available |

### TMC9660 Pin Map

| Pin ID | TMC9660 Pin | Description | Access Method |
|--------|-------------|-------------|---------------|
| GPIO_TMC9660_CHIP1_GPIO17 | GPIO17 | General purpose GPIO | Via PCAL95555 P0.0 |
| GPIO_TMC9660_CHIP1_GPIO18 | GPIO18 | General purpose GPIO | Via PCAL95555 P0.1 |

### ADC Channel Map

#### ESP32-C6 ADC Channels

| Channel ID | GPIO Pin | ADC Unit | Channel | Voltage Range |
|------------|----------|----------|---------|---------------|
| ADC_ESP32_ADC1_CH0 | GPIO0 | ADC1 | CH0 | 0-3.3V |
| ADC_ESP32_ADC1_CH1 | GPIO1 | ADC1 | CH1 | 0-3.3V |
| ADC_ESP32_ADC1_CH2 | GPIO2 | ADC1 | CH2 | 0-3.3V |
| ADC_ESP32_ADC1_CH3 | GPIO3 | ADC1 | CH3 | 0-3.3V |
| ADC_ESP32_ADC1_CH4 | GPIO4 | ADC1 | CH4 | 0-3.3V |
| ADC_ESP32_ADC1_CH5 | GPIO5 | ADC1 | CH5 | 0-3.3V |
| ADC_ESP32_ADC1_CH6 | GPIO6 | ADC1 | CH6 | 0-3.3V |
| ADC_ESP32_ADC2_CH0 | GPIO7 | ADC2 | CH0 | 0-3.3V |
| ADC_ESP32_ADC2_CH1 | GPIO8 | ADC2 | CH1 | 0-3.3V |
| ADC_ESP32_ADC2_CH2 | GPIO9 | ADC2 | CH2 | 0-3.3V |
| ADC_ESP32_ADC2_CH3 | GPIO10 | ADC2 | CH3 | 0-3.3V |
| ADC_ESP32_ADC2_CH4 | GPIO11 | ADC2 | CH4 | 0-3.3V |
| ADC_ESP32_ADC2_CH5 | GPIO12 | ADC2 | CH5 | 0-3.3V |

#### TMC9660 ADC Channels

| Channel ID | TMC9660 Pin | Description | Voltage Range |
|------------|-------------|-------------|---------------|
| ADC_TMC9660_CHIP1_AIN1 | AIN1 | Analog input 1 | 0-5V |
| ADC_TMC9660_CHIP1_AIN2 | AIN2 | Analog input 2 | 0-5V |
| ADC_TMC9660_CHIP1_AIN3 | AIN3 | Analog input 3 | 0-5V |

## Error Handling

### Error Types and Handling

```cpp
// GPIO errors
enum class GpioError {
    SUCCESS = 0,
    INVALID_PIN,
    COMMUNICATION_FAILURE,
    INITIALIZATION_FAILED,
    SOURCE_UNHEALTHY
};

// ADC errors
enum class AdcError {
    SUCCESS = 0,
    INVALID_CHANNEL,
    READING_FAILED,
    CALIBRATION_ERROR,
    SOURCE_UNHEALTHY
};
```

### Error Handling Examples

```cpp
void HandleGpioErrors() {
    auto& gpioHandler = GpioHandler::GetInstance();
    
    // Check system health before operations
    if (!gpioHandler.IsSystemHealthy()) {
        ESP_LOGW("GPIO", "System unhealthy, operations may fail");
        
        // Check individual sources
        if (!gpioHandler.IsEsp32GpioHealthy()) {
            ESP_LOGE("GPIO", "ESP32 GPIO system failure");
            // Attempt to reinitialize ESP32 GPIO
            gpioHandler.InitializeEsp32Gpio();
        }
        
        if (!gpioHandler.IsPcal95555Healthy()) {
            ESP_LOGE("GPIO", "PCAL95555 I2C communication failure");
            // Check I2C bus, attempt to reinitialize
        }
    }
    
    // Safe pin operations with error checking
    if (!gpioHandler.SetPin(GPIO_ESP32_PIN_2, true)) {
        ESP_LOGE("GPIO", "Failed to set pin GPIO_ESP32_PIN_2");
    }
}

void HandleAdcErrors() {
    auto& adcHandler = AdcHandler::GetInstance();
    
    // Validate channel before reading
    constexpr AdcChannel channel = ADC_ESP32_ADC1_CH0;
    
    if (!adcHandler.IsSystemHealthy()) {
        ESP_LOGW("ADC", "System unhealthy, readings may be invalid");
    }
    
    // Attempt reading with error handling
    uint16_t rawValue = adcHandler.ReadRaw(channel);
    if (rawValue == 0xFFFF) {  // Error value
        ESP_LOGE("ADC", "Failed to read ADC channel");
        return;
    }
    
    float voltage = adcHandler.ReadVoltage(channel);
    if (voltage < 0) {  // Error indicator
        ESP_LOGE("ADC", "Failed to convert ADC reading to voltage");
        return;
    }
}
```

## Performance Considerations

### Threading and Concurrency

The system is designed for thread-safe operation:

```cpp
// All classes use internal mutexes for thread safety
// Multiple threads can safely call methods simultaneously

void ThreadSafeExample() {
    // Thread 1
    std::thread t1([]() {
        auto& gpio = GpioData::GetInstance();
        gpio.SetPinState(GPIO_ESP32_PIN_0, true);
    });
    
    // Thread 2
    std::thread t2([]() {
        auto& gpio = GpioData::GetInstance();
        bool state = gpio.GetPinState(GPIO_ESP32_PIN_1);
    });
    
    t1.join();
    t2.join();
}
```

### Batch Operations for Performance

Use batch operations for better performance:

```cpp
void OptimizedOperations() {
    auto& gpioData = GpioData::GetInstance();
    
    // Instead of individual operations:
    // gpioData.SetPinState(GPIO_ESP32_PIN_0, true);
    // gpioData.SetPinState(GPIO_ESP32_PIN_1, false);
    // gpioData.SetPinState(GPIO_ESP32_PIN_2, true);
    
    // Use batch operations:
    std::vector<GpioPin> pins = {
        GPIO_ESP32_PIN_0, GPIO_ESP32_PIN_1, GPIO_ESP32_PIN_2
    };
    std::vector<bool> states = {true, false, true};
    
    // Single mutex lock, single hardware transaction
    gpioData.SetMultiplePinStates(pins, states);
}
```

### Memory Considerations

- All classes use singleton pattern to minimize memory usage
- Pin registrations are stored in efficient containers
- Health monitoring uses minimal memory overhead

## Troubleshooting

### Common Issues and Solutions

#### 1. GPIO Initialization Failures

**Symptoms:**
- GPIO operations fail silently
- `IsSystemHealthy()` returns false
- Initialization errors in logs

**Solutions:**
```cpp
void DiagnoseGpioIssues() {
    auto& gpioHandler = GpioHandler::GetInstance();
    
    // Check individual components
    if (!gpioHandler.IsEsp32GpioHealthy()) {
        ESP_LOGE("DIAG", "ESP32 GPIO driver issue");
        // Check ESP-IDF installation and configuration
    }
    
    if (!gpioHandler.IsPcal95555Healthy()) {
        ESP_LOGE("DIAG", "PCAL95555 I2C communication issue");
        // Check I2C wiring, pull-ups, device addresses
        // Verify I2C bus configuration
    }
    
    if (!gpioHandler.IsTmc9660GpioHealthy()) {
        ESP_LOGE("DIAG", "TMC9660 communication issue");
        // Check TMC9660 power, communication interface
    }
}
```

#### 2. I2C Communication Problems

**Symptoms:**
- PCAL95555 operations fail
- I2C timeout errors
- Inconsistent GPIO states

**Solutions:**
```cpp
void DiagnoseI2cIssues() {
    auto& i2cBus = SfI2cBus::GetInstance();
    
    // Test I2C bus connectivity
    uint8_t deviceAddr = 0x20;  // PCAL95555 chip 1
    uint8_t testData = 0x00;
    
    if (!i2cBus.Write(deviceAddr, &testData, 1)) {
        ESP_LOGE("I2C", "Cannot communicate with device 0x%02X", deviceAddr);
        // Check physical connections, pull-up resistors
        // Verify device power supply
        // Check for address conflicts
    }
}
```

#### 3. ADC Reading Issues

**Symptoms:**
- Invalid ADC readings (0xFFFF, negative voltages)
- Inconsistent readings
- ADC initialization failures

**Solutions:**
```cpp
void DiagnoseAdcIssues() {
    auto& adcHandler = AdcHandler::GetInstance();
    
    // Check system health
    if (!adcHandler.IsSystemHealthy()) {
        if (!adcHandler.IsEsp32AdcHealthy()) {
            ESP_LOGE("ADC", "ESP32 ADC calibration or driver issue");
            // Check ADC calibration
            // Verify GPIO pin configuration
        }
        
        if (!adcHandler.IsTmc9660AdcHealthy()) {
            ESP_LOGE("ADC", "TMC9660 ADC communication issue");
            // Check TMC9660 configuration
            // Verify SPI/UART communication
        }
    }
}
```

#### 4. TMC9660 Integration Issues

**Symptoms:**
- TMC9660 GPIO pins not responding
- Motor controller faults
- Communication timeouts

**Solutions:**
```cpp
void DiagnoseTmc9660Issues() {
    auto& gpioHandler = GpioHandler::GetInstance();
    
    // Check TMC9660 control signals
    bool drvEnabled = gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_4); // TMC_DRV_EN
    bool faultActive = !gpioHandler.GetPin(GPIO_PCAL95555_CHIP1_PIN_3); // TMC_nFAULT_STATUS
    
    if (!drvEnabled) {
        ESP_LOGW("TMC", "TMC9660 driver not enabled");
        gpioHandler.SetPin(GPIO_PCAL95555_CHIP1_PIN_4, true);
    }
    
    if (faultActive) {
        ESP_LOGE("TMC", "TMC9660 fault condition detected");
        // Reset TMC9660
        gpioHandler.SetPin(GPIO_PCAL95555_CHIP1_PIN_5, false); // TMC_RST_CTRL
        vTaskDelay(pdMS_TO_TICKS(10));
        gpioHandler.SetPin(GPIO_PCAL95555_CHIP1_PIN_5, true);
    }
}
```

### Debug Tools

#### System Health Monitoring

```cpp
void MonitorSystemHealth() {
    // Create a periodic task to monitor system health
    xTaskCreate([](void* param) {
        while (true) {
            auto& gpioHandler = GpioHandler::GetInstance();
            auto& adcHandler = AdcHandler::GetInstance();
            
            // Log health status
            ESP_LOGI("HEALTH", "GPIO System: %s", 
                    gpioHandler.IsSystemHealthy() ? "HEALTHY" : "UNHEALTHY");
            ESP_LOGI("HEALTH", "ADC System: %s", 
                    adcHandler.IsSystemHealthy() ? "HEALTHY" : "UNHEALTHY");
            
            // Detailed health information
            auto gpioHealth = GpioData::GetInstance().GetSystemHealth();
            ESP_LOGI("HEALTH", "GPIO: %d pins, %d errors", 
                    gpioHealth.totalRegisteredPins, 
                    gpioHealth.totalCommunicationErrors);
            
            vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
        }
    }, "HealthMonitor", 2048, nullptr, 5, nullptr);
}
```

#### Debug Logging

Enable detailed logging for troubleshooting:

```cpp
// In sdkconfig or menuconfig:
// CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
// CONFIG_LOG_MAXIMUM_EQUALS_DEFAULT=y

// Or programmatically:
esp_log_level_set("GPIO", ESP_LOG_DEBUG);
esp_log_level_set("ADC", ESP_LOG_DEBUG);
esp_log_level_set("I2C", ESP_LOG_DEBUG);
esp_log_level_set("TMC", ESP_LOG_DEBUG);
```

---

This comprehensive documentation covers all aspects of the HardFOC GPIO and ADC data sourcing system. For additional support or questions, refer to the individual class documentation in the header files or contact the development team.
