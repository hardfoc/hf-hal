# ADC Manager - Advanced ADC Management System

## Overview

The ADC Manager is a comprehensive ADC management system for the HardFOC platform that provides a unified, hardware-agnostic interface for all ADC operations across multiple hardware sources. It integrates with the platform mapping system to automatically manage ADC channels from various chips (ESP32-C6 internal ADC, TMC9660 ADC) using string-based sensor identification for maximum flexibility and extensibility.

## Key Features

### 🚀 **Core Capabilities**
- **Platform Mapping Integration**: Automatic channel discovery and registration
- **Multi-Chip Support**: ESP32-C6 internal ADC and TMC9660 ADC
- **String-Based API**: Extensible sensor identification using `string_view`
- **Thread-Safe Operations**: Full RTOS compatibility with `RtosMutex`
- **Comprehensive Error Handling**: Detailed error codes with `hf_adc_err_t`

### 📊 **Advanced Features**
- **Batch Operations**: Efficient multi-channel reading
- **Smart Sampling**: Multi-sample averaging with variance validation
- **Digital Filtering**: Configurable exponential filtering
- **Calibration Support**: Runtime calibration with reference points
- **Statistics & Diagnostics**: Comprehensive system health monitoring
- **Self-Test Capability**: Built-in system validation

### 🎯 **Performance Optimizations**
- **Lazy Initialization**: Hardware resources initialized on demand
- **Atomic Operations**: Lock-free statistics for performance
- **Memory Efficient**: Static string references for embedded systems
- **Hardware Abstraction**: Complete `BaseAdc` function coverage

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ADC Manager                             │
├─────────────────────────────────────────────────────────────┤
│  String-Based API │ Platform Mapping │ Error Handling      │
├─────────────────────────────────────────────────────────────┤
│               Hardware Abstraction Layer                    │
├─────────────────────────────────────────────────────────────┤
│     ESP32-C6 ADC    │    TMC9660 ADC    │   Future ADCs    │
└─────────────────────────────────────────────────────────────┘
```

### Design Principles

1. **Hardware Agnostic**: All operations use sensor names, not hardware addresses
2. **Platform Driven**: Configuration comes from platform mapping system
3. **Thread Safe**: Full concurrent access support with proper locking
4. **Error Transparent**: Rich error information without compromising performance
5. **Future Proof**: Extensible design for additional ADC hardware

## Quick Start

### Basic Usage

```cpp
#include "AdcManager.h"

// Get the singleton instance
auto& adcManager = AdcManager::GetInstance();

// Initialize with TMC9660 controller
Tmc9660MotorController tmc9660Controller;
auto init_result = adcManager.Initialize(tmc9660Controller);
if (init_result != hf_adc_err_t::ADC_SUCCESS) {
    // Handle initialization error
}

// Read a channel
AdcReading reading;
auto result = adcManager.ReadChannel("ADC_SYSTEM_VOLTAGE_3V3", reading);
if (result == hf_adc_err_t::ADC_SUCCESS) {
    printf("Voltage: %.3fV (Raw: %lu)\n", reading.voltage, reading.rawValue);
}
```

### Advanced Sampling

```cpp
// Configure multi-sample reading with filtering
AdcSamplingSpec spec("ADC_MOTOR_CURRENT_PHASE_A", 10, 5, true, 0.2f, 0.05f);

AdcReading averaged_reading;
auto result = adcManager.ReadChannelWithSampling("ADC_MOTOR_CURRENT_PHASE_A", spec, averaged_reading);
if (result == hf_adc_err_t::ADC_SUCCESS) {
    printf("Filtered current: %.3fA\n", averaged_reading.calibratedValue);
}
```

### Batch Operations

```cpp
// Read multiple channels efficiently
std::vector<std::string_view> channels = {
    "ADC_SYSTEM_VOLTAGE_3V3",
    "ADC_MOTOR_CURRENT_PHASE_A", 
    "ADC_MOTOR_CURRENT_PHASE_B",
    "ADC_TEMPERATURE_SENSOR"
};

auto batch_result = adcManager.BatchRead(channels);
if (batch_result.AllSuccessful()) {
    for (size_t i = 0; i < batch_result.readings.size(); ++i) {
        printf("%.*s: %.3fV\n", 
               static_cast<int>(batch_result.sensor_names[i].length()),
               batch_result.sensor_names[i].data(),
               batch_result.readings[i].voltage);
    }
}
```

## API Reference

### Initialization & Lifecycle

```cpp
class AdcManager {
public:
    // Singleton access
    static AdcManager& GetInstance() noexcept;
    
    // Initialization
    hf_adc_err_t EnsureInitialized() noexcept;
    hf_adc_err_t Initialize(Tmc9660MotorController& controller) noexcept;
    hf_adc_err_t Shutdown() noexcept;
    bool IsInitialized() const noexcept;
    
    // System diagnostics
    hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;
};
```

### Channel Management

```cpp
// Registration
hf_adc_err_t RegisterChannel(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept;
hf_adc_err_t RegisterChannel(AdcInputSensor sensor, std::string_view name,
                             float referenceVoltage = 3.3f,
                             float calibrationScale = 1.0f,
                             float calibrationOffset = 0.0f) noexcept;

// Access
BaseAdc* Get(std::string_view name) noexcept;
bool Contains(std::string_view name) const noexcept;
size_t Size() const noexcept;
void LogAllRegisteredChannels() const noexcept;
```

### Reading Operations

```cpp
// Basic reading
hf_adc_err_t ReadChannel(std::string_view name, AdcReading& reading) noexcept;
hf_adc_err_t ReadChannel(AdcInputSensor sensor, AdcReading& reading) noexcept;
hf_adc_err_t ReadRawValue(std::string_view name, uint32_t& value) noexcept;
hf_adc_err_t ReadVoltage(std::string_view name, float& voltage) noexcept;

// Advanced reading
hf_adc_err_t ReadChannelWithSampling(std::string_view name, 
                                     const AdcSamplingSpec& spec, 
                                     AdcReading& reading) noexcept;
hf_adc_err_t ReadFilteredValue(std::string_view name, float& value) noexcept;

// Batch operations
AdcBatchResult BatchRead(const std::vector<std::string_view>& sensor_names) noexcept;
AdcBatchResult BatchRead(const AdcBatchOperation& operation) noexcept;
AdcBatchResult ReadAllChannels() noexcept;
AdcBatchResult BatchReadWithSampling(const std::vector<AdcSamplingSpec>& specs) noexcept;
```

### Configuration & Calibration

```cpp
// Calibration
hf_adc_err_t CalibrateChannel(std::string_view name,
                              float referenceVoltage,
                              uint32_t measuredValue) noexcept;

// Configuration
hf_adc_err_t SetChannelRange(std::string_view name,
                             float minVoltage, float maxVoltage) noexcept;
hf_adc_err_t ConfigureFiltering(std::string_view name,
                                bool enableFiltering,
                                float filterWeight = 0.1f) noexcept;
hf_adc_err_t SetReferenceVoltage(std::string_view name,
                                 float referenceVoltage) noexcept;
```

### Diagnostics & Statistics

```cpp
// Statistics
hf_adc_err_t GetStatistics(std::string_view name, BaseAdc::AdcStatistics& statistics) const noexcept;
hf_adc_err_t ResetStatistics(std::string_view name) noexcept;
hf_adc_err_t ResetAllChannels() noexcept;

// System health
hf_adc_err_t GetSystemHealth(std::string& health_info) const noexcept;
hf_adc_err_t GetSystemStatistics(std::string& stats) const noexcept;
hf_adc_err_t PerformSelfTest(std::string& result) noexcept;
```

## Data Structures

### AdcReading

```cpp
struct AdcReading {
    uint32_t rawValue;                  // Raw ADC count value
    float voltage;                      // Converted voltage value  
    float calibratedValue;              // Calibrated final value
    std::chrono::steady_clock::time_point timestamp;  // Reading timestamp
    hf_adc_err_t result;                // Reading result status
    uint8_t hardwareChannel;            // Hardware channel used
    AdcChip sourceChip;                 // Source ADC chip
    
    bool IsValid() const noexcept;
    uint32_t GetAgeMs() const noexcept;
};
```

### AdcSamplingSpec

```cpp
struct AdcSamplingSpec {
    AdcInputSensor sensor;              // Sensor identifier
    uint8_t numberOfSamples;            // Number of samples to take
    uint16_t samplingIntervalMs;        // Interval between samples
    bool enableFiltering;               // Enable digital filtering
    float filterWeight;                 // Filter weight (0.0-1.0)
    float maxAllowedVariance;           // Maximum allowed variance
    
    bool IsValid() const noexcept;
};
```

### AdcBatchResult

```cpp
struct AdcBatchResult {
    std::vector<std::string_view> sensor_names;  // Sensor names read
    std::vector<AdcReading> readings;            // Individual readings
    std::vector<hf_adc_err_t> results;           // Individual results
    hf_adc_err_t overall_result;                 // Overall operation result
    std::chrono::steady_clock::time_point timestamp;  // Batch timestamp
    uint32_t total_time_ms;                      // Total operation time
    
    bool AllSuccessful() const noexcept;
    float GetSuccessRate() const noexcept;
};
```

## Error Handling

### Error Codes

```cpp
enum class hf_adc_err_t : uint8_t {
    ADC_SUCCESS = 0,                    // Operation successful
    ADC_ERR_NOT_INITIALIZED,            // ADC system not initialized
    ADC_ERR_ALREADY_INITIALIZED,        // Already initialized
    ADC_ERR_INVALID_PARAMETER,          // Invalid parameter
    ADC_ERR_SENSOR_NOT_FOUND,           // Sensor not registered
    ADC_ERR_SENSOR_ALREADY_REGISTERED,  // Sensor already registered
    ADC_ERR_HARDWARE_FAULT,             // Hardware communication failure
    ADC_ERR_TIMEOUT,                    // Operation timeout
    ADC_ERR_OUT_OF_RANGE,               // Reading out of range
    ADC_ERR_CALIBRATION_FAILED,         // Calibration failed
    ADC_ERR_COMMUNICATION_FAILURE,      // I2C/SPI communication error
    ADC_ERR_SAMPLING_ERROR,             // Sampling operation error
    ADC_ERR_FILTERING_ERROR,            // Filtering operation error
    ADC_ERR_BATCH_OPERATION_FAILED,     // Batch operation failed
    ADC_ERR_SYSTEM_OVERLOAD,            // System overload detected
    ADC_ERR_UNKNOWN                     // Unknown error
};
```

### Error Handling Best Practices

```cpp
// Always check return codes
auto result = adcManager.ReadChannel("ADC_VOLTAGE", reading);
switch (result) {
    case hf_adc_err_t::ADC_SUCCESS:
        // Use reading data
        break;
    case hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND:
        // Handle sensor not found
        break;
    case hf_adc_err_t::ADC_ERR_HARDWARE_FAULT:
        // Handle hardware issue
        break;
    default:
        // Handle other errors
        break;
}
```

## Platform Integration

### Automatic Channel Discovery

The ADC Manager integrates with the platform mapping system to automatically discover and register ADC channels:

```cpp
// Platform mapping drives channel registration
for (const auto& mapping : ADC_PLATFORM_MAPPING) {
    if (mapping.category == AdcCategory::MEASUREMENT) {
        auto adc_driver = CreateAdcDriver(mapping);
        RegisterChannel(mapping.sensor, mapping.name, mapping.reference_voltage);
    }
}
```

### Hardware Abstraction

```cpp
// ESP32-C6 ADC channels
"ADC_ESP32_CH0" -> ESP32 Internal ADC Channel 0
"ADC_ESP32_CH1" -> ESP32 Internal ADC Channel 1

// TMC9660 ADC channels  
"ADC_TMC9660_CURRENT_A" -> TMC9660 Device 0 Channel 0
"ADC_TMC9660_CURRENT_B" -> TMC9660 Device 0 Channel 1
"ADC_TMC9660_CURRENT_C" -> TMC9660 Device 0 Channel 2
```

## Performance Characteristics

### Timing Performance

| Operation | Typical Time | Notes |
|-----------|--------------|--------|
| Single Read | 10-50 µs | Depends on hardware |
| Batch Read (4 channels) | 40-200 µs | Parallel where possible |
| Multi-sample (10 samples) | 100-500 µs | Includes inter-sample delays |
| Calibration | 1-5 ms | Includes validation |

### Memory Usage

| Component | Memory Usage | Notes |
|-----------|--------------|--------|
| Manager Instance | ~2KB | Singleton overhead |
| Per Channel | ~200 bytes | Including statistics |
| Batch Operation | ~100 bytes + data | Temporary allocation |

### Thread Safety

- **Read Operations**: Fully concurrent, lock-free where possible
- **Configuration**: Thread-safe with mutex protection
- **Statistics**: Atomic operations for performance
- **Registration**: Mutex-protected during initialization only

## Advanced Usage

### Custom Calibration

```cpp
// Two-point calibration
float lowRef = 0.5f, highRef = 2.5f;
uint32_t lowRaw = 620, highRaw = 3100;

// Calculate calibration parameters
float scale = (highRef - lowRef) / (highRaw - lowRaw);
float offset = lowRef - (scale * lowRaw);

// Apply calibration
adcManager.CalibrateChannel("ADC_CUSTOM_SENSOR", highRef, highRaw);
```

### Real-Time Monitoring

```cpp
// Configure for real-time current monitoring
AdcSamplingSpec current_spec("ADC_MOTOR_CURRENT", 5, 1, true, 0.3f, 0.02f);

while (motor_running) {
    AdcReading current;
    if (adcManager.ReadChannelWithSampling("ADC_MOTOR_CURRENT", current_spec, current) == hf_adc_err_t::ADC_SUCCESS) {
        // Real-time current control
        motor_controller.UpdateCurrent(current.calibratedValue);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // 1kHz update rate
}
```

### System Health Monitoring

```cpp
// Periodic health check
void PerformHealthCheck() {
    AdcSystemDiagnostics diagnostics;
    if (adcManager.GetSystemDiagnostics(diagnostics) == hf_adc_err_t::ADC_SUCCESS) {
        if (!diagnostics.system_healthy) {
            printf("ADC System Health Warning:\n");
            printf("- Success Rate: %.1f%%\n", diagnostics.average_success_rate);
            printf("- Communication Errors: %u\n", diagnostics.communication_errors);
            printf("- Hardware Errors: %u\n", diagnostics.hardware_errors);
            
            // Take corrective action
            if (diagnostics.communication_errors > 100) {
                // Reset communication interfaces
            }
        }
    }
}
```

## Troubleshooting

### Common Issues

1. **Sensor Not Found**
   ```cpp
   // Check if sensor is registered
   if (!adcManager.Contains("ADC_SENSOR_NAME")) {
       // Sensor not registered - check platform mapping
   }
   ```

2. **Reading Out of Range**
   ```cpp
   // Configure expected range
   adcManager.SetChannelRange("ADC_SENSOR", 0.0f, 3.3f);
   ```

3. **High Error Rate**
   ```cpp
   // Check system health
   std::string health;
   adcManager.GetSystemHealth(health);
   printf("%s", health.c_str());
   ```

### Debugging

```cpp
// Enable verbose logging
adcManager.LogAllRegisteredChannels();

// Perform self-test
std::string test_result;
auto test_status = adcManager.PerformSelfTest(test_result);
printf("Self-test: %s\n", test_result.c_str());
```

## Migration from Legacy ADC Systems

### From AdcData/AdcHandler

```cpp
// Old way
AdcData adcData;
AdcHandler adcHandler;
uint32_t raw = adcHandler.ReadRaw(ADC_CHANNEL_0);
float voltage = adcData.ConvertToVoltage(raw);

// New way
AdcManager& adc = AdcManager::GetInstance();
AdcReading reading;
adc.ReadChannel("ADC_SYSTEM_VOLTAGE", reading);
// reading.voltage is already converted
```

### Benefits of Migration

- **Unified API**: Single interface for all ADC operations
- **Better Error Handling**: Rich error information
- **Thread Safety**: Safe concurrent access
- **Platform Integration**: Automatic hardware discovery
- **Advanced Features**: Filtering, calibration, batch operations

## Version History

- **v2.0**: Complete rewrite with platform mapping integration
- **v1.0**: Legacy AdcData/AdcHandler system (deprecated)

---

*For more information, see the [HardFOC Documentation Index](../DOCUMENTATION_INDEX.md)*