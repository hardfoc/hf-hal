# AdcManager - Advanced ADC Management System

<div align="center">

![Component](https://img.shields.io/badge/component-AdcManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-ESP32--C6%20|%20TMC9660-orange.svg)

**Comprehensive ADC management system for the HardFOC platform**

</div>

## 📋 Overview

The `AdcManager` is a singleton component handler that provides unified, thread-safe access to ADC channels across multiple hardware sources. It integrates with the platform mapping system to automatically manage ADC channels from ESP32-C6 internal ADC and TMC9660 motor controller ADC through a single, consistent API using string-based channel identification.

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
    uint32_t raw_value;
    if (adc.ReadChannelCount("ESP32_ADC1_CH0", raw_value) == HF_ADC_SUCCESS) {
        printf("Raw ADC value: %u\n", raw_value);
    }
    
    // Read calibrated voltage
    float voltage;
    if (adc.ReadChannelV("ESP32_ADC1_CH0", voltage) == HF_ADC_SUCCESS) {
        printf("Voltage: %.3fV\n", voltage);
    }
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

#### ADC Registration and Management
```cpp
// Channel registration
hf_adc_err_t RegisterAdc(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept;
BaseAdc* Get(std::string_view name) noexcept;
bool Contains(std::string_view name) const noexcept;
size_t Size() const noexcept;
```

#### Single Channel Operations
```cpp
// Raw ADC readings
hf_adc_err_t ReadChannelCount(std::string_view name, uint32_t& count, 
                             uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;

// Voltage readings
hf_adc_err_t ReadChannelV(std::string_view name, float& voltage,
                         uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;

// Channel validation
bool IsChannelAvailable(std::string_view name) const noexcept;
```

#### Batch Operations
```cpp
// Multi-channel operations
AdcBatchResult BatchRead(const AdcBatchOperation& operation) noexcept;
AdcBatchResult BatchReadVoltages(const std::vector<std::string_view>& channels,
                                uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;
AdcBatchResult BatchReadCounts(const std::vector<std::string_view>& channels,
                              uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;
```

#### Statistics and Diagnostics
```cpp
// System diagnostics
hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;
hf_adc_err_t GetStatistics(std::string_view name, BaseAdc::AdcStatistics& statistics) const noexcept;
hf_adc_err_t ResetStatistics(std::string_view name) noexcept;
void DumpStatistics() const noexcept;
```

### Data Structures

#### AdcChannelInfo
```cpp
struct AdcChannelInfo {
    std::string_view name;                      // Human-readable name
    std::unique_ptr<BaseAdc> adc_driver;        // ADC driver instance
    HfFunctionalAdcChannel functional_channel;  // Functional channel identifier
    HfAdcChipType hardware_chip;                // Hardware chip identifier
    uint8_t hardware_channel_id;                // Hardware channel ID within the chip
    bool is_registered;                         // Registration status
    
    // Hardware configuration
    float reference_voltage;                    // Reference voltage for conversion
    uint32_t resolution_bits;                   // ADC resolution in bits
    uint32_t max_voltage_mv;                    // Maximum voltage in millivolts
    float voltage_divider;                      // Voltage divider ratio
    
    // Statistics and monitoring
    uint32_t access_count;                      // Number of times accessed
    uint32_t error_count;                       // Number of errors encountered
    uint64_t last_access_time;                  // Timestamp of last access
};
```

#### AdcSystemDiagnostics
```cpp
struct AdcSystemDiagnostics {
    bool system_healthy;                           // Overall system health
    uint32_t total_channels_registered;            // Total channels registered
    uint32_t channels_by_chip[4];                  // Channels per chip
    uint32_t total_operations;                     // Total operations performed
    uint32_t successful_operations;                // Successful operations
    uint32_t failed_operations;                    // Failed operations
    uint32_t calibration_errors;                   // Calibration errors
    uint32_t communication_errors;                 // Communication errors
    uint32_t hardware_errors;                      // Hardware errors
    uint64_t system_uptime_ms;                     // System uptime
    float average_read_time_us;                    // Average read time in microseconds
    hf_adc_err_t last_error;                       // Last error encountered
};
```

#### AdcBatchOperation
```cpp
struct AdcBatchOperation {
    std::vector<std::string_view> channel_names;    // Channel names to operate on
    std::vector<uint8_t> samples_per_channel;       // Samples per channel
    std::vector<uint16_t> intervals_ms;             // Intervals between samples in ms
    bool use_individual_specs;                      // Use individual specs or common settings
    uint8_t common_samples;                         // Common number of samples
    uint16_t common_interval_ms;                    // Common sampling interval
};
```

#### AdcBatchResult
```cpp
struct AdcBatchResult {
    std::vector<std::string_view> channel_names;    // Channel names operated on
    std::vector<float> voltages;                    // Resulting voltage readings
    std::vector<uint32_t> raw_values;               // Raw ADC values
    std::vector<hf_adc_err_t> results;              // Individual operation results
    hf_adc_err_t overall_result;                    // Overall operation result
    uint32_t total_time_ms;                         // Total operation time
    
    bool AllSuccessful() const noexcept;            // Check if all operations were successful
};
```

## 🎯 Hardware Support

### Supported Hardware Sources

| Hardware | Channels Available | Resolution | Reference Voltage | Features |
|----------|-------------------|------------|------------------|----------|
| **ESP32-C6** | 6 channels (ADC1: 0-4, ADC2: 0) | 12-bit | 3.3V | Calibration, attenuation |
| **TMC9660** | 12 channels (4 current + 4 analog + 4 voltage/temp) | 12-bit | 3.3V | Motor feedback, diagnostics |

### Channel Naming Convention

```cpp
// ESP32-C6 ADC channels
"ESP32_ADC1_CH0" to "ESP32_ADC1_CH4"    // ADC1 channels 0-4
"ESP32_ADC2_CH0"                        // ADC2 channel 0

// TMC9660 ADC channels
"TMC9660_CURRENT_I0" to "TMC9660_CURRENT_I3"     // Current sense channels
"TMC9660_AIN0" to "TMC9660_AIN3"                 // External analog inputs
"TMC9660_VOLTAGE_SUPPLY"                         // Supply voltage monitoring
"TMC9660_VOLTAGE_DRIVER"                         // Driver voltage monitoring
"TMC9660_TEMP_CHIP"                             // Chip temperature
"TMC9660_TEMP_EXTERNAL"                         // External temperature

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

### Error Handling

The AdcManager uses comprehensive error handling with specific error codes:

```cpp
// Common error codes
hf_adc_err_t::HF_ADC_SUCCESS              // Operation successful
hf_adc_err_t::HF_ADC_ERR_INVALID_PARAMETER // Invalid channel name or parameters
hf_adc_err_t::HF_ADC_ERR_NOT_INITIALIZED   // ADC manager not initialized
hf_adc_err_t::HF_ADC_ERR_HARDWARE_FAULT    // Hardware communication error
hf_adc_err_t::HF_ADC_ERR_CALIBRATION       // Calibration error
hf_adc_err_t::HF_ADC_ERR_OUT_OF_MEMORY     // Memory allocation failed
```

## 📊 Examples

### Basic ADC Reading

```cpp
#include "component-handlers/AdcManager.h"

void basic_adc_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Single channel readings
    uint32_t raw_value;
    float voltage;
    
    if (adc.ReadChannelCount("ESP32_ADC1_CH0", raw_value) == HF_ADC_SUCCESS) {
        printf("Channel ESP32_ADC1_CH0 raw: %u\n", raw_value);
    }
    
    if (adc.ReadChannelV("ESP32_ADC1_CH0", voltage) == HF_ADC_SUCCESS) {
        printf("Channel ESP32_ADC1_CH0 voltage: %.3fV\n", voltage);
    }
    
    // Multiple samples for stability
    if (adc.ReadChannelV("ESP32_ADC1_CH0", voltage, 16) == HF_ADC_SUCCESS) {
        printf("Channel ESP32_ADC1_CH0 filtered: %.3fV\n", voltage);
    }
}
```

### Multi-Channel Reading

```cpp
void multi_channel_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Define channels to read
    std::vector<std::string_view> channels = {
        "ESP32_ADC1_CH0",
        "ESP32_ADC1_CH1",
        "TMC9660_AIN0",
        "TMC9660_AIN1"
    };
    
    // Read all channels simultaneously
    auto readings = adc.BatchReadVoltages(channels, 4);  // 4 samples each
    
    printf("Multi-channel readings (time: %u ms):\n", readings.total_time_ms);
    for (size_t i = 0; i < channels.size(); i++) {
        if (readings.results[i] == HF_ADC_SUCCESS) {
            printf("  %.*s: %.3fV (raw: %u)\n",
                   static_cast<int>(channels[i].size()), channels[i].data(),
                   readings.voltages[i], readings.raw_values[i]);
        } else {
            printf("  %.*s: ERROR\n",
                   static_cast<int>(channels[i].size()), channels[i].data());
        }
    }
    
    printf("All successful: %s\n", readings.AllSuccessful() ? "YES" : "NO");
}
```

### Motor Current Monitoring

```cpp
void motor_current_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Monitor motor current from TMC9660
    printf("Motor current monitoring:\n");
    for (int i = 0; i < 100; i++) {
        float voltage;
        if (adc.ReadChannelV("TMC9660_CURRENT_I0", voltage, 8) == HF_ADC_SUCCESS) {
            // Convert voltage to current (example: 0.1V/A current sensor)
            float current = voltage / 0.1f;
            printf("Current: %.2fA (%.3fV)\n", current, voltage);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Battery Voltage Monitoring

```cpp
void battery_voltage_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Monitor battery voltage with filtering
    while (true) {
        float battery_voltage;
        if (adc.ReadChannelV("ESP32_ADC1_CH2", battery_voltage, 32) == HF_ADC_SUCCESS) {
            printf("Battery: %.2fV", battery_voltage);
            
            // Battery status indication
            if (battery_voltage > 12.0f) {
                printf(" [GOOD]\n");
            } else if (battery_voltage > 10.5f) {
                printf(" [LOW]\n");
            } else {
                printf(" [CRITICAL]\n");
            }
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
    AdcSystemDiagnostics diagnostics;
    if (adc.GetSystemDiagnostics(diagnostics) == HF_ADC_SUCCESS) {
        printf("ADC System Status:\n");
        printf("  Overall healthy: %s\n", diagnostics.system_healthy ? "YES" : "NO");
        printf("  Total channels: %u\n", diagnostics.total_channels_registered);
        printf("  Total operations: %u\n", diagnostics.total_operations);
        printf("  Successful operations: %u\n", diagnostics.successful_operations);
        printf("  Failed operations: %u\n", diagnostics.failed_operations);
        printf("  Calibration errors: %u\n", diagnostics.calibration_errors);
        printf("  Communication errors: %u\n", diagnostics.communication_errors);
        printf("  Hardware errors: %u\n", diagnostics.hardware_errors);
        printf("  Avg read time: %.2fµs\n", diagnostics.average_read_time_us);
        printf("  System uptime: %llu ms\n", diagnostics.system_uptime_ms);
    }
    
    // Get channel statistics
    BaseAdc::AdcStatistics stats;
    if (adc.GetStatistics("ESP32_ADC1_CH0", stats) == HF_ADC_SUCCESS) {
        printf("\nChannel ESP32_ADC1_CH0 statistics:\n");
        printf("  Access count: %u\n", stats.access_count);
        printf("  Error count: %u\n", stats.error_count);
        printf("  Last access time: %llu\n", stats.last_access_time);
    }
    
    // Dump all statistics
    adc.DumpStatistics();
}
```

## 🔍 Advanced Usage

### Performance Optimization

```cpp
void optimized_adc_usage() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Use batch operations for better performance
    std::vector<std::string_view> channels = {
        "ESP32_ADC1_CH0", "ESP32_ADC1_CH1", "ESP32_ADC1_CH2",
        "TMC9660_AIN0", "TMC9660_AIN1", "TMC9660_AIN2"
    };
    
    // Benchmark single vs batch reads
    auto start_time = esp_timer_get_time();
    
    // Single reads (slower)
    for (int i = 0; i < 100; i++) {
        for (const auto& channel : channels) {
            float voltage;
            adc.ReadChannelV(channel, voltage);
        }
    }
    
    auto single_time = esp_timer_get_time() - start_time;
    
    start_time = esp_timer_get_time();
    
    // Batch reads (faster)
    for (int i = 0; i < 100; i++) {
        adc.BatchReadVoltages(channels, 1);
    }
    
    auto batch_time = esp_timer_get_time() - start_time;
    
    printf("Performance comparison:\n");
    printf("  Single reads: %lld µs\n", single_time);
    printf("  Batch reads: %lld µs\n", batch_time);
    printf("  Speedup: %.1fx\n", static_cast<float>(single_time) / batch_time);
}
```

### Advanced Batch Operations

```cpp
void advanced_batch_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Create batch operation with individual specifications
    std::vector<std::string_view> channels = {
        "ESP32_ADC1_CH0", "ESP32_ADC1_CH1", "TMC9660_AIN0"
    };
    
    std::vector<uint8_t> samples = {16, 8, 32};  // Different sample counts
    std::vector<uint16_t> intervals = {0, 10, 5}; // Different intervals
    
    AdcBatchOperation operation(channels, samples, intervals);
    auto result = adc.BatchRead(operation);
    
    printf("Advanced batch operation results:\n");
    for (size_t i = 0; i < channels.size(); i++) {
        if (result.results[i] == HF_ADC_SUCCESS) {
            printf("  %.*s: %.3fV (raw: %u, samples: %u, interval: %u ms)\n",
                   static_cast<int>(channels[i].size()), channels[i].data(),
                   result.voltages[i], result.raw_values[i],
                   samples[i], intervals[i]);
        }
    }
    
    printf("Total time: %u ms\n", result.total_time_ms);
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
    float voltage;
    auto result = adc.ReadChannelV("ESP32_ADC1_CH0", voltage);
    if (result != HF_ADC_SUCCESS) {
        printf("ERROR: Failed to read ADC channel: %d\n", static_cast<int>(result));
    }
    
    // Monitor system health
    AdcSystemDiagnostics diagnostics;
    if (adc.GetSystemDiagnostics(diagnostics) == HF_ADC_SUCCESS) {
        if (!diagnostics.system_healthy) {
            printf("WARNING: ADC system health check failed\n");
            printf("Failed operations: %u\n", diagnostics.failed_operations);
            printf("Calibration errors: %u\n", diagnostics.calibration_errors);
        }
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
        float motor_current;
        if (adc.ReadChannelV("TMC9660_CURRENT_I0", motor_current) == HF_ADC_SUCCESS) {
            // Check for overcurrent condition
            if (motor_current > 5.0f) {
                printf("Overcurrent detected: %.2fA\n", motor_current);
                // Disable motor or reduce current
            }
        }
        
        // Read supply voltage
        float supply_voltage;
        if (adc.ReadChannelV("ESP32_ADC1_CH0", supply_voltage) == HF_ADC_SUCCESS) {
            // Check for undervoltage condition
            if (supply_voltage < 10.0f) {
                printf("Undervoltage detected: %.2fV\n", supply_voltage);
                // Implement protection measures
            }
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

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*