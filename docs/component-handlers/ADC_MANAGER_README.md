# AdcManager - ADC Management System

<div align="center">

![Component](https://img.shields.io/badge/component-AdcManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-ESP32--C6%20|%20TMC9660-orange.svg)

**Advanced ADC management system for the HardFOC platform**

</div>

## 📋 Overview

The `AdcManager` is a singleton component handler that provides unified, thread-safe access to ADC channels across multiple hardware sources. It abstracts the complexity of managing ADC channels from ESP32-C6 internal ADC and TMC9660 motor controller ADC through a single, consistent API.

### ✨ Key Features

- **🔗 Multi-Source ADC Management**: ESP32-C6 internal ADC, TMC9660 ADC
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📍 String-Based Channel Identification**: Flexible, extensible channel naming
- **🛡️ Platform Mapping Integration**: Automatic hardware discovery
- **📊 Advanced Diagnostics**: Real-time health monitoring and calibration
- **⚡ Batch Operations**: Optimized multi-channel operations
- **🎯 Voltage Calibration**: Automatic voltage conversion with reference scaling
- **🏥 Health Monitoring**: Per-chip and per-channel statistics

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        AdcManager                              │
├─────────────────────────────────────────────────────────────────┤
│  String-Based API    │ channel_name → hardware mapping        │
├─────────────────────────────────────────────────────────────────┤
│  Platform Integration│ Automatic channel discovery & registration│
├─────────────────────────────────────────────────────────────────┤
│  Hardware Handlers   │ ESP32 ADC, TMC9660 ADC handlers        │
├─────────────────────────────────────────────────────────────────┤
│  BaseAdc Interface   │ Unified ADC operations                  │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/AdcManager.h"

void adc_example() {
    // Get singleton instance
    auto& adc = AdcManager::GetInstance();
    
    // Initialize the manager
    if (!adc.Initialize()) {
        printf("Failed to initialize ADC manager\n");
        return;
    }
    
    // Read raw ADC value
    uint16_t raw = adc.ReadRaw("ESP32_ADC1_CH0");
    printf("Raw ADC value: %u\n", raw);
    
    // Read calibrated voltage
    float voltage = adc.ReadVoltage("ESP32_ADC1_CH0");
    printf("Voltage: %.3fV\n", voltage);
}
```

## 📖 API Reference

### Core Operations

#### Initialization
```cpp
class AdcManager {
public:
    // Singleton access
    static AdcManager& GetInstance() noexcept;
    
    // Initialization
    bool Initialize() noexcept;
    bool IsInitialized() const noexcept;
    void Deinitialize() noexcept;
};
```

#### Single Channel Operations
```cpp
// Raw ADC readings
uint16_t ReadRaw(std::string_view channel_name) noexcept;
uint32_t ReadRaw32(std::string_view channel_name) noexcept;

// Voltage readings
float ReadVoltage(std::string_view channel_name) noexcept;
float ReadVoltageFiltered(std::string_view channel_name, uint8_t samples = 16) noexcept;

// Channel validation
bool IsChannelAvailable(std::string_view channel_name) const noexcept;
bool IsChannelCalibrated(std::string_view channel_name) const noexcept;
```

#### Batch Operations
```cpp
// Multi-channel raw readings
template<size_t N>
AdcMultiReading<N> ReadMultipleRaw(const std::array<std::string_view, N>& channels) noexcept;

// Multi-channel voltage readings
template<size_t N>
AdcMultiReading<N> ReadMultipleVoltages(const std::array<std::string_view, N>& channels) noexcept;

// Optimized batch structures
template<size_t N>
struct AdcMultiReading {
    std::array<uint16_t, N> raw_values;
    std::array<float, N> voltages;
    std::array<bool, N> success_flags;
    uint64_t timestamp;
    bool all_successful;
};
```

#### Calibration and Configuration
```cpp
// Channel calibration
bool CalibrateChannel(std::string_view channel_name) noexcept;
bool SetChannelCalibration(std::string_view channel_name, 
                          float reference_voltage, 
                          float voltage_divider = 1.0f) noexcept;

// Channel configuration
bool ConfigureChannel(std::string_view channel_name, 
                     AdcResolution resolution = AdcResolution::ADC_12_BIT,
                     AdcAttenuation attenuation = AdcAttenuation::ADC_ATTEN_DB_11) noexcept;
```

### Diagnostics and Monitoring

#### System Status
```cpp
// Health monitoring
struct AdcSystemStatus {
    bool overall_healthy;
    uint32_t total_channels_registered;
    uint32_t esp32_channels_active;
    uint32_t tmc9660_channels_active;
    uint32_t total_errors;
    uint32_t calibration_errors;
    uint64_t last_error_time;
    float average_read_time_us;
};

AdcSystemStatus GetSystemStatus() const noexcept;
bool IsSystemHealthy() const noexcept;
```

#### Channel Information
```cpp
// Channel details
struct AdcChannelInfo {
    std::string_view name;
    std::unique_ptr<BaseAdc> adc_driver;
    HfFunctionalAdcChannel functional_channel;
    HfAdcChipType hardware_chip;
    uint8_t hardware_channel_id;
    bool is_registered;
    float reference_voltage;
    uint32_t resolution_bits;
    uint32_t max_voltage_mv;
    float voltage_divider;
    uint32_t access_count;
    uint32_t error_count;
    uint64_t last_access_time;
};

std::optional<AdcChannelInfo> GetChannelInfo(std::string_view channel_name) const noexcept;
std::vector<std::string> GetRegisteredChannels() const noexcept;
```

## 🎯 Hardware Support

### Supported Hardware Sources

| Hardware | Channels Available | Resolution | Reference Voltage | Features |
|----------|-------------------|------------|------------------|----------|
| **ESP32-C6** | 6 channels (ADC1: 0-4, ADC2: 0) | 12-bit | 3.3V | Calibration, attenuation |
| **TMC9660** | 3 channels (AIN1, AIN2, AIN3) | 12-bit | 3.3V | Motor feedback, diagnostics |

### Channel Naming Convention

```cpp
// ESP32-C6 ADC channels
"ESP32_ADC1_CH0" to "ESP32_ADC1_CH4"    // ADC1 channels 0-4
"ESP32_ADC2_CH0"                        // ADC2 channel 0

// TMC9660 ADC channels
"TMC9660_AIN1"                          // Analog input 1
"TMC9660_AIN2"                          // Analog input 2  
"TMC9660_AIN3"                          // Analog input 3

// Functional channel names (platform-mapped)
"MOTOR_CURRENT_SENSE"                   // Motor current feedback
"BATTERY_VOLTAGE"                       // Battery voltage monitoring
"TEMPERATURE_SENSOR"                    // Temperature monitoring
"USER_ANALOG_1"                         // User-defined analog input
```

## 🔧 Configuration

### Platform Integration

The AdcManager automatically integrates with the platform mapping system:

```cpp
// Platform mapping integration
#include "hf_functional_pin_config_vortex_v1.hpp"

// Automatic channel discovery based on platform configuration
// Channels are registered automatically during initialization
```

### Hardware Configuration

```cpp
// ESP32-C6 ADC configuration
enum class AdcResolution {
    ADC_9_BIT = 9,
    ADC_10_BIT = 10,
    ADC_11_BIT = 11,
    ADC_12_BIT = 12
};

enum class AdcAttenuation {
    ADC_ATTEN_DB_0 = 0,    // 0dB, input range: 0-1.1V
    ADC_ATTEN_DB_2_5,      // 2.5dB, input range: 0-1.5V
    ADC_ATTEN_DB_6,        // 6dB, input range: 0-2.2V
    ADC_ATTEN_DB_11        // 11dB, input range: 0-3.9V
};

// TMC9660 ADC configuration
struct Tmc9660AdcConfig {
    uint32_t sampling_rate_hz = 1000;
    bool enable_filtering = true;
    uint8_t filter_samples = 8;
};
```

## 📊 Examples

### Basic ADC Reading

```cpp
#include "component-handlers/AdcManager.h"

void basic_adc_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Single channel readings
    uint16_t raw = adc.ReadRaw("ESP32_ADC1_CH0");
    float voltage = adc.ReadVoltage("ESP32_ADC1_CH0");
    
    printf("Channel ESP32_ADC1_CH0:\n");
    printf("  Raw: %u\n", raw);
    printf("  Voltage: %.3fV\n", voltage);
    
    // Filtered reading for more stability
    float filtered_voltage = adc.ReadVoltageFiltered("ESP32_ADC1_CH0", 16);
    printf("  Filtered: %.3fV\n", filtered_voltage);
}
```

### Multi-Channel Reading

```cpp
void multi_channel_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Define channels to read
    std::array<std::string_view, 4> channels = {
        "ESP32_ADC1_CH0",
        "ESP32_ADC1_CH1",
        "TMC9660_AIN1",
        "TMC9660_AIN2"
    };
    
    // Read all channels simultaneously
    auto readings = adc.ReadMultipleVoltages(channels);
    
    printf("Multi-channel readings (timestamp: %llu):\n", readings.timestamp);
    for (size_t i = 0; i < channels.size(); i++) {
        if (readings.success_flags[i]) {
            printf("  %.*s: %.3fV (raw: %u)\n",
                   static_cast<int>(channels[i].size()), channels[i].data(),
                   readings.voltages[i], readings.raw_values[i]);
        } else {
            printf("  %.*s: ERROR\n",
                   static_cast<int>(channels[i].size()), channels[i].data());
        }
    }
    
    printf("All successful: %s\n", readings.all_successful ? "YES" : "NO");
}
```

### Motor Current Monitoring

```cpp
void motor_current_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Configure current sense channel with appropriate scaling
    adc.SetChannelCalibration("TMC9660_AIN1", 3.3f, 10.0f);  // 10:1 voltage divider
    
    // Monitor motor current
    printf("Motor current monitoring:\n");
    for (int i = 0; i < 100; i++) {
        float voltage = adc.ReadVoltage("TMC9660_AIN1");
        
        // Convert voltage to current (example: 0.1V/A current sensor)
        float current = voltage / 0.1f;
        
        printf("Current: %.2fA (%.3fV)\n", current, voltage);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Battery Voltage Monitoring

```cpp
void battery_voltage_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Configure battery voltage channel (assuming voltage divider)
    adc.SetChannelCalibration("ESP32_ADC1_CH2", 3.3f, 3.0f);  // 3:1 voltage divider
    
    // Monitor battery voltage with filtering
    while (true) {
        float battery_voltage = adc.ReadVoltageFiltered("ESP32_ADC1_CH2", 32);
        
        printf("Battery: %.2fV", battery_voltage);
        
        // Battery status indication
        if (battery_voltage > 12.0f) {
            printf(" [GOOD]\n");
        } else if (battery_voltage > 10.5f) {
            printf(" [LOW]\n");
        } else {
            printf(" [CRITICAL]\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Get system status
    auto status = adc.GetSystemStatus();
    
    printf("ADC System Status:\n");
    printf("  Overall healthy: %s\n", status.overall_healthy ? "YES" : "NO");
    printf("  Total channels: %u\n", status.total_channels_registered);
    printf("  ESP32 channels: %u\n", status.esp32_channels_active);
    printf("  TMC9660 channels: %u\n", status.tmc9660_channels_active);
    printf("  Total errors: %u\n", status.total_errors);
    printf("  Calibration errors: %u\n", status.calibration_errors);
    printf("  Avg read time: %.2fµs\n", status.average_read_time_us);
    
    // Get detailed channel information
    auto channels = adc.GetRegisteredChannels();
    printf("\nRegistered channels (%zu total):\n", channels.size());
    
    for (const auto& channel : channels) {
        auto info = adc.GetChannelInfo(channel);
        if (info) {
            printf("  %s:\n", channel.c_str());
            printf("    Hardware chip: %d\n", static_cast<int>(info->hardware_chip));
            printf("    Channel ID: %u\n", info->hardware_channel_id);
            printf("    Reference: %.3fV\n", info->reference_voltage);
            printf("    Resolution: %u bits\n", info->resolution_bits);
            printf("    Voltage divider: %.2f\n", info->voltage_divider);
            printf("    Access count: %u\n", info->access_count);
            printf("    Errors: %u\n", info->error_count);
        }
    }
}
```

### Advanced Calibration

```cpp
void calibration_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Calibrate specific channels
    if (adc.CalibrateChannel("ESP32_ADC1_CH0")) {
        printf("ESP32_ADC1_CH0 calibrated successfully\n");
    }
    
    // Custom calibration for voltage divider circuits
    adc.SetChannelCalibration("ESP32_ADC1_CH1", 3.3f, 2.0f);  // 2:1 divider
    adc.SetChannelCalibration("ESP32_ADC1_CH2", 3.3f, 4.7f);  // 4.7:1 divider
    
    // Verify calibration
    for (const auto& channel : {"ESP32_ADC1_CH0", "ESP32_ADC1_CH1", "ESP32_ADC1_CH2"}) {
        if (adc.IsChannelCalibrated(channel)) {
            printf("Channel %s: Calibrated\n", channel);
        } else {
            printf("Channel %s: Not calibrated\n", channel);
        }
    }
}
```

## 🔍 Advanced Usage

### High-Speed Sampling

```cpp
void high_speed_sampling() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Configure for high-speed sampling
    adc.ConfigureChannel("ESP32_ADC1_CH0", AdcResolution::ADC_12_BIT, 
                        AdcAttenuation::ADC_ATTEN_DB_11);
    
    // Collect samples at high rate
    constexpr size_t SAMPLE_COUNT = 1000;
    std::array<uint16_t, SAMPLE_COUNT> samples;
    
    auto start_time = esp_timer_get_time();
    
    for (size_t i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = adc.ReadRaw("ESP32_ADC1_CH0");
    }
    
    auto end_time = esp_timer_get_time();
    float sample_rate = SAMPLE_COUNT * 1000000.0f / (end_time - start_time);
    
    printf("Collected %zu samples at %.1f samples/sec\n", SAMPLE_COUNT, sample_rate);
    
    // Calculate statistics
    uint32_t sum = 0;
    uint16_t min_val = samples[0], max_val = samples[0];
    
    for (const auto& sample : samples) {
        sum += sample;
        min_val = std::min(min_val, sample);
        max_val = std::max(max_val, sample);
    }
    
    float average = static_cast<float>(sum) / SAMPLE_COUNT;
    printf("Statistics: min=%u, max=%u, avg=%.1f\n", min_val, max_val, average);
}
```

### Performance Optimization

```cpp
void optimized_adc_usage() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Use batch operations for better performance
    std::array<std::string_view, 6> channels = {
        "ESP32_ADC1_CH0", "ESP32_ADC1_CH1", "ESP32_ADC1_CH2",
        "TMC9660_AIN1", "TMC9660_AIN2", "TMC9660_AIN3"
    };
    
    // Benchmark single vs batch reads
    auto start_time = esp_timer_get_time();
    
    // Single reads (slower)
    for (int i = 0; i < 100; i++) {
        for (const auto& channel : channels) {
            adc.ReadVoltage(channel);
        }
    }
    
    auto single_time = esp_timer_get_time() - start_time;
    
    start_time = esp_timer_get_time();
    
    // Batch reads (faster)
    for (int i = 0; i < 100; i++) {
        adc.ReadMultipleVoltages(channels);
    }
    
    auto batch_time = esp_timer_get_time() - start_time;
    
    printf("Performance comparison:\n");
    printf("  Single reads: %lld µs\n", single_time);
    printf("  Batch reads: %lld µs\n", batch_time);
    printf("  Speedup: %.1fx\n", static_cast<float>(single_time) / batch_time);
}
```

## 🚨 Error Handling

### Common Error Scenarios

```cpp
void error_handling_example() {
    auto& adc = AdcManager::GetInstance();
    
    // Check initialization
    if (!adc.Initialize()) {
        printf("ERROR: Failed to initialize ADC manager\n");
        return;
    }
    
    // Validate channel exists before use
    if (!adc.IsChannelAvailable("ESP32_ADC1_CH0")) {
        printf("ERROR: Channel ESP32_ADC1_CH0 not available\n");
        return;
    }
    
    // Safe ADC operations with error checking
    uint16_t raw = adc.ReadRaw("ESP32_ADC1_CH0");
    if (raw == 0xFFFF) {  // Error indicator
        printf("ERROR: Failed to read ADC channel\n");
    }
    
    // Monitor system health
    if (!adc.IsSystemHealthy()) {
        printf("WARNING: ADC system health check failed\n");
        auto status = adc.GetSystemStatus();
        printf("Total errors: %u\n", status.total_errors);
        printf("Calibration errors: %u\n", status.calibration_errors);
    }
}
```

## 🔗 Integration

### With Motor Controller

```cpp
#include "component-handlers/All.h"

void motor_adc_integration() {
    auto& adc = AdcManager::GetInstance();
    auto& motor = MotorController::GetInstance();
    
    adc.Initialize();
    motor.EnsureInitialized();
    
    // Monitor motor parameters via ADC
    while (true) {
        // Read motor current from TMC9660 ADC
        float motor_current = adc.ReadVoltage("TMC9660_AIN1");
        
        // Read supply voltage
        float supply_voltage = adc.ReadVoltage("ESP32_ADC1_CH0");
        
        // Check for overcurrent condition
        if (motor_current > 5.0f) {
            printf("Overcurrent detected: %.2fA\n", motor_current);
            // Disable motor or reduce current
        }
        
        // Check for undervoltage condition
        if (supply_voltage < 10.0f) {
            printf("Undervoltage detected: %.2fV\n", supply_voltage);
            // Implement protection measures
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver with ADC
- **[Complete ADC System Guide](../HARDFOC_GPIO_ADC_SYSTEM.md)** - Comprehensive system documentation

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*