# NtcTemperatureHandler - NTC Thermistor Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-NtcTemperatureHandler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-NTC%20Thermistor-orange.svg)
![Interface](https://img.shields.io/badge/interface-ADC-green.svg)

**Unified handler for NTC thermistor temperature sensors with ADC integration**

</div>

## 📋 Overview

The `NtcTemperatureHandler` is a unified handler for NTC thermistor temperature sensors that provides a modern, comprehensive interface for temperature measurement and monitoring. It implements the BaseTemperature interface, supports multiple NTC types, offers dual conversion methods, and provides comprehensive diagnostics and error handling.

### ✨ Key Features

- **🌡️ Temperature Sensing**: High-precision NTC thermistor temperature measurement
- **📊 Multiple NTC Types**: Support for various NTC thermistor specifications
- **🔧 Dual Conversion**: Lookup table and mathematical conversion methods
- **📡 ADC Integration**: Hardware-agnostic design using BaseAdc interface
- **🛡️ Thread-Safe**: Concurrent access from multiple tasks
- **🏥 Comprehensive Diagnostics**: Statistics, monitoring, and error detection
- **⚙️ Advanced Configuration**: Calibration, filtering, and threshold monitoring
- **🔍 Health Monitoring**: Temperature range validation and error reporting

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                NtcTemperatureHandler                           │
├─────────────────────────────────────────────────────────────────┤
│  Temperature Conversion │ NTC to temperature conversion        │
├─────────────────────────────────────────────────────────────────┤
│  ADC Integration       │ Hardware-agnostic ADC interface      │
├─────────────────────────────────────────────────────────────────┤
│  BaseTemperature       │ Standard temperature interface        │
├─────────────────────────────────────────────────────────────────┤
│  NTC Driver           │ Low-level thermistor calculations     │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Temperature Measurement

```cpp
#include "utils-and-drivers/driver-handlers/NtcTemperatureHandler.h"
#include "component-handlers/AdcManager.h"

void ntc_basic_example() {
    // Get ADC interface
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);  // Get first ADC
    if (!adc) {
        printf("ADC interface not available\n");
        return;
    }
    
    // Create NTC temperature handler
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;  // Use ADC channel 0
    config.series_resistance = 10000.0f;  // 10kΩ series resistor
    config.reference_voltage = 3.3f;  // 3.3V reference
    
    NtcTemperatureHandler handler(adc, config);
    
    // Initialize handler
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Failed to initialize NTC temperature handler\n");
        return;
    }
    
    // Read temperature
    float temperature;
    if (handler.ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Temperature: %.2f°C\n", temperature);
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class NtcTemperatureHandler : public BaseTemperature {
public:
    // Constructor
    explicit NtcTemperatureHandler(BaseAdc* adc_interface,
                                 const ntc_temp_handler_config_t& config) noexcept;

    // Initialization
    hf_temp_err_t Initialize() noexcept override;
    hf_temp_err_t Deinitialize() noexcept override;
    bool IsInitialized() const noexcept override;
};
```

#### Temperature Reading
```cpp
// Basic temperature reading
hf_temp_err_t ReadTemperature(float& temperature_celsius) noexcept override;

// Range and threshold management
hf_temp_err_t SetRange(float min_celsius, float max_celsius) noexcept override;
hf_temp_err_t SetThresholds(float low_threshold_celsius, float high_threshold_celsius) noexcept override;

// Monitoring and callbacks
hf_temp_err_t EnableThresholdMonitoring(hf_temp_threshold_callback_t callback, void* user_data) noexcept override;
hf_temp_err_t StartContinuousMonitoring(hf_u32_t sample_rate_hz, hf_temp_reading_callback_t callback, void* user_data) noexcept override;

// Calibration
hf_temp_err_t SetCalibrationOffset(float offset_celsius) noexcept override;
```

#### Statistics and Diagnostics
```cpp
// Statistics and diagnostics
hf_temp_err_t GetStatistics(hf_temp_statistics_t& statistics) noexcept override;
hf_temp_err_t GetDiagnostics(hf_temp_diagnostics_t& diagnostics) noexcept override;
hf_temp_err_t ResetStatistics() noexcept override;
hf_temp_err_t ResetDiagnostics() noexcept override;

// Capabilities
hf_u32_t GetCapabilities() const noexcept override;
```

#### NTC-Specific Configuration
```cpp
// NTC configuration
ntc_err_t SetNtcConfiguration(const ntc_config_t& config) noexcept;
ntc_err_t Calibrate(float reference_temperature_celsius) noexcept;
ntc_err_t SetConversionMethod(ntc_conversion_method_t method) noexcept;

// Voltage divider configuration
ntc_err_t SetVoltageDivider(float series_resistance) noexcept;
ntc_err_t SetReferenceVoltage(float reference_voltage) noexcept;
ntc_err_t SetBetaValue(float beta_value) noexcept;

// ADC configuration
ntc_err_t SetAdcChannel(uint8_t adc_channel) noexcept;
ntc_err_t SetSamplingParameters(uint32_t sample_count, uint32_t sample_delay_ms) noexcept;

// Filtering
ntc_err_t SetFiltering(bool enable, float alpha = 0.1f) noexcept;
```

## 🎯 Hardware Support

### NTC Thermistor Features

- **Temperature Range**: -40°C to +125°C (configurable)
- **Multiple NTC Types**: Support for various NTC specifications
- **Conversion Methods**: Lookup table and mathematical conversion
- **ADC Integration**: Hardware-agnostic ADC interface
- **Voltage Divider**: Configurable series resistance
- **Calibration**: Offset and reference temperature calibration
- **Filtering**: Configurable temperature filtering
- **Threshold Monitoring**: High/low temperature thresholds
- **Continuous Monitoring**: Configurable sampling rates

### Supported NTC Types

The handler supports various NTC thermistor types including:
- **NTCG163JFT103FT1S**: 10kΩ at 25°C, β = 3435K
- **NTCG164JFT103FT1S**: 10kΩ at 25°C, β = 3435K
- **NTCG165JFT103FT1S**: 10kΩ at 25°C, β = 3435K
- **Custom types**: Configurable resistance and beta values

## 📊 Examples

### Basic Temperature Reading

```cpp
void basic_temperature_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    // Create handler with default configuration
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // Read temperature
    float temperature;
    if (handler.ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Temperature: %.2f°C\n", temperature);
    }
    
    // Get temperature statistics
    hf_temp_statistics_t stats;
    if (handler.GetStatistics(stats) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Temperature statistics:\n");
        printf("  Min: %.2f°C\n", stats.min_temperature_celsius);
        printf("  Max: %.2f°C\n", stats.max_temperature_celsius);
        printf("  Average: %.2f°C\n", stats.average_temperature_celsius);
        printf("  Readings: %u\n", stats.total_readings);
    }
}
```

### Advanced Configuration

```cpp
void advanced_config_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    // Create custom configuration
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.ntc_type = NTC_TYPE_NTCG163JFT103FT1S;
    config.adc_channel = 1;
    config.series_resistance = 10000.0f;  // 10kΩ
    config.reference_voltage = 3.3f;
    config.calibration_offset = 0.5f;  // 0.5°C offset
    config.conversion_method = NTC_CONVERSION_LOOKUP_TABLE;
    config.sample_count = 5;  // Average 5 samples
    config.sample_delay_ms = 10;  // 10ms between samples
    config.min_temperature = -40.0f;
    config.max_temperature = 125.0f;
    config.enable_filtering = true;
    config.filter_alpha = 0.1f;
    config.sensor_name = "Motor_Temperature";
    config.sensor_description = "Motor winding temperature sensor";
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // Set temperature range
    handler.SetRange(-40.0f, 125.0f);
    
    // Set thresholds for monitoring
    handler.SetThresholds(60.0f, 80.0f);  // Warning at 60°C, critical at 80°C
    
    printf("Advanced NTC configuration applied\n");
}
```

### Threshold Monitoring

```cpp
void threshold_monitoring_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // Set temperature thresholds
    handler.SetThresholds(50.0f, 70.0f);
    
    // Enable threshold monitoring with callback
    handler.EnableThresholdMonitoring(
        [](float temperature_celsius, hf_temp_threshold_type_t threshold_type, void* user_data) {
            switch (threshold_type) {
                case hf_temp_threshold_type_t::HF_TEMP_THRESHOLD_LOW:
                    printf("Temperature below threshold: %.2f°C\n", temperature_celsius);
                    break;
                case hf_temp_threshold_type_t::HF_TEMP_THRESHOLD_HIGH:
                    printf("Temperature above threshold: %.2f°C\n", temperature_celsius);
                    break;
            }
        },
        nullptr
    );
    
    // Continuous monitoring
    for (int i = 0; i < 100; i++) {
        float temperature;
        if (handler.ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
            printf("Temperature: %.2f°C\n", temperature);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1Hz monitoring
    }
}
```

### Calibration and Filtering

```cpp
void calibration_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    config.enable_filtering = true;
    config.filter_alpha = 0.1f;
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // Set calibration offset
    handler.SetCalibrationOffset(1.5f);  // 1.5°C offset
    
    // Calibrate with reference temperature
    float reference_temp = 25.0f;  // Room temperature
    if (handler.Calibrate(reference_temp) == ntc_err_t::NTC_ERR_NONE) {
        printf("Calibration completed with reference temperature %.1f°C\n", reference_temp);
    }
    
    // Configure filtering
    handler.SetFiltering(true, 0.05f);  // Strong filtering
    
    // Read filtered temperature
    float temperature;
    if (handler.ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Filtered temperature: %.2f°C\n", temperature);
    }
}
```

### Continuous Monitoring

```cpp
void continuous_monitoring_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    config.sample_count = 3;  // Average 3 samples
    config.sample_delay_ms = 5;  // 5ms between samples
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // Start continuous monitoring
    handler.StartContinuousMonitoring(
        10,  // 10Hz sampling rate
        [](float temperature_celsius, hf_u64_t timestamp_us, void* user_data) {
            printf("Continuous reading: %.2f°C at %llu us\n", temperature_celsius, timestamp_us);
        },
        nullptr
    );
    
    // Monitor for 10 seconds
    for (int i = 0; i < 100; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz loop
        
        // Get diagnostics
        hf_temp_diagnostics_t diagnostics;
        if (handler.GetDiagnostics(diagnostics) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
            printf("Diagnostics: errors=%u, warnings=%u, last_error=%d\n",
                   diagnostics.total_errors, diagnostics.total_warnings, 
                   static_cast<int>(diagnostics.last_error));
        }
    }
}
```

### Error Handling

```cpp
void error_handling_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    
    NtcTemperatureHandler handler(adc, config);
    
    // Check initialization
    hf_temp_err_t result = handler.Initialize();
    if (result != hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("ERROR: Failed to initialize NTC handler: %d\n", static_cast<int>(result));
        return;
    }
    
    // Safe temperature reading
    float temperature;
    result = handler.ReadTemperature(temperature);
    if (result != hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("ERROR: Failed to read temperature: %d\n", static_cast<int>(result));
        return;
    }
    
    // Check temperature range
    if (temperature < -40.0f || temperature > 125.0f) {
        printf("WARNING: Temperature out of range: %.2f°C\n", temperature);
    }
    
    // Get diagnostics
    hf_temp_diagnostics_t diagnostics;
    if (handler.GetDiagnostics(diagnostics) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        if (diagnostics.total_errors > 0) {
            printf("WARNING: %u errors detected\n", diagnostics.total_errors);
        }
        
        if (diagnostics.last_error != hf_temp_err_t::HF_TEMP_ERR_NONE) {
            printf("Last error: %d\n", static_cast<int>(diagnostics.last_error));
        }
    }
    
    printf("All operations successful\n");
}
```

## 🔍 Advanced Usage

### Multi-Sensor Configuration

```cpp
void multi_sensor_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    // Create multiple NTC handlers for different sensors
    std::vector<std::unique_ptr<NtcTemperatureHandler>> handlers;
    
    // Motor temperature sensor
    auto* adc0 = adc_manager.Get(0);
    if (adc0) {
        ntc_temp_handler_config_t motor_config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
        motor_config.adc_channel = 0;
        motor_config.sensor_name = "Motor_Temp";
        motor_config.min_temperature = 0.0f;
        motor_config.max_temperature = 100.0f;
        
        auto motor_handler = std::make_unique<NtcTemperatureHandler>(adc0, motor_config);
        if (motor_handler->Initialize() == hf_temp_err_t::HF_TEMP_ERR_NONE) {
            handlers.push_back(std::move(motor_handler));
        }
    }
    
    // Ambient temperature sensor
    auto* adc1 = adc_manager.Get(1);
    if (adc1) {
        ntc_temp_handler_config_t ambient_config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
        ambient_config.adc_channel = 1;
        ambient_config.sensor_name = "Ambient_Temp";
        ambient_config.min_temperature = -40.0f;
        ambient_config.max_temperature = 85.0f;
        
        auto ambient_handler = std::make_unique<NtcTemperatureHandler>(adc1, ambient_config);
        if (ambient_handler->Initialize() == hf_temp_err_t::HF_TEMP_ERR_NONE) {
            handlers.push_back(std::move(ambient_handler));
        }
    }
    
    // Read from all sensors
    for (size_t i = 0; i < handlers.size(); i++) {
        float temperature;
        if (handlers[i]->ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
            printf("Sensor %zu: %.2f°C\n", i, temperature);
        }
    }
}
```

### Custom NTC Configuration

```cpp
void custom_ntc_example() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    // Create custom NTC configuration
    ntc_config_t ntc_config = {};
    ntc_config.ntc_type = NTC_TYPE_CUSTOM;
    ntc_config.resistance_25c = 5000.0f;  // 5kΩ at 25°C
    ntc_config.beta_value = 3950.0f;      // Beta value
    ntc_config.series_resistance = 10000.0f;  // 10kΩ series resistor
    ntc_config.reference_voltage = 3.3f;
    ntc_config.conversion_method = NTC_CONVERSION_MATHEMATICAL;
    
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // Apply custom NTC configuration
    if (handler.SetNtcConfiguration(ntc_config) == ntc_err_t::NTC_ERR_NONE) {
        printf("Custom NTC configuration applied\n");
    }
    
    // Read temperature with custom configuration
    float temperature;
    if (handler.ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Custom NTC temperature: %.2f°C\n", temperature);
    }
}
```

### Performance Optimization

```cpp
void performance_optimization() {
    auto& adc_manager = AdcManager::GetInstance();
    adc_manager.EnsureInitialized();
    
    auto* adc = adc_manager.Get(0);
    if (!adc) return;
    
    // Optimize for high-speed reading
    ntc_temp_handler_config_t config = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    config.adc_channel = 0;
    config.sample_count = 1;  // Single sample for speed
    config.sample_delay_ms = 0;  // No delay
    config.enable_filtering = false;  // Disable filtering for speed
    config.conversion_method = NTC_CONVERSION_LOOKUP_TABLE;  // Faster than mathematical
    
    NtcTemperatureHandler handler(adc, config);
    if (handler.Initialize() != hf_temp_err_t::HF_TEMP_ERR_NONE) return;
    
    // High-speed temperature reading
    constexpr int READ_COUNT = 1000;
    auto start_time = esp_timer_get_time();
    
    for (int i = 0; i < READ_COUNT; i++) {
        float temperature;
        handler.ReadTemperature(temperature);
    }
    
    auto end_time = esp_timer_get_time();
    float avg_time_us = (end_time - start_time) / static_cast<float>(READ_COUNT);
    float reads_per_sec = 1000000.0f / avg_time_us;
    
    printf("Performance test:\n");
    printf("  Average read time: %.2f µs\n", avg_time_us);
    printf("  Reads per second: %.0f\n", reads_per_sec);
}
```

## 📚 See Also

- **[AdcManager Documentation](../component-handlers/ADC_MANAGER_README.md)** - ADC management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[TMC9660 Handler Documentation](TMC9660_HANDLER_README.md)** - Motor controller with temperature monitoring
- **[BNO08x Handler Documentation](BNO08X_HANDLER_README.md)** - IMU sensor handler

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).* 