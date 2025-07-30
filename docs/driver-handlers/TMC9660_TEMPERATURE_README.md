# TMC9660 Internal Temperature Sensor

## Overview

The TMC9660 motor controller includes an internal temperature sensor that can be accessed through the unified `BaseTemperature` interface. This allows the TMC9660's chip temperature to be monitored and integrated into temperature management systems alongside other temperature sensors.

## Features

- **Internal Chip Temperature Monitoring**: Read the TMC9660's internal temperature sensor
- **Unified Interface**: Implements the `BaseTemperature` interface for consistency with other temperature sensors
- **Multiple Unit Support**: Read temperature in Celsius, Fahrenheit, Kelvin, or Rankine
- **Comprehensive Statistics**: Track operation statistics, min/max temperatures, and success rates
- **Diagnostic Information**: Monitor sensor health, availability, and error conditions
- **Thread-Safe**: Protected by mutex for concurrent access
- **Error Handling**: Robust error detection and reporting

## Temperature Conversion

The TMC9660 internal temperature sensor uses the following conversion formula:
```
T(°C) = raw_value × 0.01615 - 268.15
```

Where:
- `raw_value` is the 16-bit raw temperature reading from the `CHIP_TEMPERATURE` parameter
- The result is the temperature in degrees Celsius

## Usage

### Basic Temperature Reading

```cpp
#include "utils-and-drivers/driver-handlers/Tmc9660Handler.h"

// Create TMC9660 handler (assuming SPI interface)
Tmc9660Handler tmc9660_handler(spi_interface, device_address);

// Initialize the handler
if (!tmc9660_handler.Initialize()) {
    // Handle initialization error
    return;
}

// Get temperature sensor interface
auto& temp_sensor = tmc9660_handler.temperature();

// Read temperature in Celsius
float temperature_celsius = 0.0f;
if (temp_sensor.ReadTemperatureCelsius(&temperature_celsius) == hf_temp_err_t::TEMP_SUCCESS) {
    logger.Info("TMC9660", "Chip temperature: %.2f°C\n", temperature_celsius);
} else {
    logger.Info("TMC9660", "Failed to read temperature\n");
}
```

### Reading in Different Units

```cpp
// Read in Fahrenheit
float temp_fahrenheit = 0.0f;
temp_sensor.ReadTemperatureFahrenheit(&temp_fahrenheit);

// Read in Kelvin
float temp_kelvin = 0.0f;
temp_sensor.ReadTemperatureKelvin(&temp_kelvin);

// Read in any unit
float temp_rankine = 0.0f;
temp_sensor.ReadTemperatureUnit(&temp_rankine, HF_TEMP_UNIT_RANKINE);
```

### Getting Sensor Information

```cpp
hf_temp_sensor_info_t sensor_info;
if (temp_sensor.GetSensorInfo(&sensor_info) == hf_temp_err_t::TEMP_SUCCESS) {
    logger.Info("TMC9660", "Sensor: %s %s\n", sensor_info.manufacturer, sensor_info.model);
    logger.Info("TMC9660", "Range: %.1f°C to %.1f°C\n", 
           sensor_info.min_temp_celsius, sensor_info.max_temp_celsius);
    logger.Info("TMC9660", "Resolution: %.2f°C\n", sensor_info.resolution_celsius);
    logger.Info("TMC9660", "Accuracy: %.1f°C\n", sensor_info.accuracy_celsius);
}
```

### Monitoring and Statistics

```cpp
// Get operation statistics
hf_temp_statistics_t stats;
if (temp_sensor.GetStatistics(stats) == hf_temp_err_t::TEMP_SUCCESS) {
    logger.Info("TMC9660", "Total readings: %d\n", stats.temperature_readings);
    logger.Info("TMC9660", "Success rate: %.1f%%\n", 
           (float)stats.successful_operations / stats.total_operations * 100.0f);
    logger.Info("TMC9660", "Min temperature: %.2f°C\n", stats.min_temperature_celsius);
    logger.Info("TMC9660", "Max temperature: %.2f°C\n", stats.max_temperature_celsius);
    logger.Info("TMC9660", "Average temperature: %.2f°C\n", stats.avg_temperature_celsius);
}

// Get diagnostic information
hf_temp_diagnostics_t diagnostics;
if (temp_sensor.GetDiagnostics(diagnostics) == hf_temp_err_t::TEMP_SUCCESS) {
    logger.Info("TMC9660", "Sensor healthy: %s\n", diagnostics.sensor_healthy ? "YES" : "NO");
    logger.Info("TMC9660", "Sensor available: %s\n", diagnostics.sensor_available ? "YES" : "NO");
    logger.Info("TMC9660", "Consecutive errors: %d\n", diagnostics.consecutive_errors);
}
```

### Full Temperature Reading

```cpp
hf_temp_reading_t reading;
if (temp_sensor.ReadTemperature(&reading) == hf_temp_err_t::TEMP_SUCCESS) {
    logger.Info("TMC9660", "Temperature: %.2f°C\n", reading.temperature_celsius);
    logger.Info("TMC9660", "Raw value: %.2f\n", reading.temperature_raw);
    logger.Info("TMC9660", "Timestamp: %llu us\n", reading.timestamp_us);
    logger.Info("TMC9660", "Accuracy: %.2f°C\n", reading.accuracy_celsius);
    logger.Info("TMC9660", "Valid: %s\n", reading.is_valid ? "YES" : "NO");
}
```

## Sensor Capabilities

The TMC9660 temperature sensor supports the following capabilities:

- `HF_TEMP_CAP_HIGH_PRECISION`: High precision temperature measurements
- `HF_TEMP_CAP_FAST_RESPONSE`: Fast response time for temperature changes

```cpp
hf_u32_t capabilities = temp_sensor.GetCapabilities();
if (temp_sensor.HasCapability(HF_TEMP_CAP_HIGH_PRECISION)) {
    logger.Info("TMC9660", "High precision supported\n");
}
if (temp_sensor.HasCapability(HF_TEMP_CAP_FAST_RESPONSE)) {
    logger.Info("TMC9660", "Fast response supported\n");
}
```

## Error Handling

The temperature sensor provides comprehensive error handling:

```cpp
float temperature = 0.0f;
hf_temp_err_t error = temp_sensor.ReadTemperatureCelsius(&temperature);

switch (error) {
    case hf_temp_err_t::TEMP_SUCCESS:
        logger.Info("TMC9660", "Temperature read successfully: %.2f°C\n", temperature);
        break;
    case hf_temp_err_t::TEMP_ERR_NOT_INITIALIZED:
        logger.Info("TMC9660", "Temperature sensor not initialized\n");
        break;
    case hf_temp_err_t::TEMP_ERR_READ_FAILED:
        logger.Info("TMC9660", "Failed to read temperature from TMC9660\n");
        break;
    case hf_temp_err_t::TEMP_ERR_OUT_OF_RANGE:
        logger.Info("TMC9660", "Temperature reading out of expected range\n");
        break;
    default:
        logger.Info("TMC9660", "Unknown error: %s\n", GetTempErrorString(error));
        break;
}
```

## Integration with Temperature Manager

The TMC9660 temperature sensor can be integrated into a temperature management system:

```cpp
// Example: Multiple temperature sensors
std::vector<BaseTemperature*> temperature_sensors;

// Add TMC9660 temperature sensor
temperature_sensors.push_back(&tmc9660_handler.temperature());

// Add other temperature sensors (e.g., external sensors)
// temperature_sensors.push_back(&external_temp_sensor);

// Read all temperatures
for (auto* sensor : temperature_sensors) {
    float temp = 0.0f;
    if (sensor->ReadTemperatureCelsius(&temp) == hf_temp_err_t::TEMP_SUCCESS) {
        logger.Info("TMC9660", "Temperature: %.2f°C\n", temp);
    }
}
```

## Technical Details

### Temperature Range
- **Typical Range**: -40°C to +150°C
- **Resolution**: ~0.1°C
- **Accuracy**: ±2°C (typical for chip temperature sensors)
- **Response Time**: ~100ms

### Error Conditions
- **Read Failure**: TMC9660 returns -273.0°C when unable to read temperature
- **Out of Range**: Temperature readings outside -40°C to +150°C are flagged as errors
- **Not Initialized**: Parent TMC9660 driver must be initialized

### Thread Safety
- All temperature reading operations are protected by a mutex
- Safe for concurrent access from multiple threads
- Statistics and diagnostics are updated atomically

### Memory Usage
- Temperature wrapper: ~200 bytes (including mutex, statistics, diagnostics)
- Minimal overhead compared to main TMC9660 handler

## Example Applications

1. **Motor Temperature Monitoring**: Monitor chip temperature during motor operation
2. **Thermal Protection**: Implement temperature-based protection systems
3. **System Health Monitoring**: Track temperature trends for system diagnostics
4. **Multi-Sensor Temperature Management**: Integrate with other temperature sensors

## Troubleshooting

### Common Issues

1. **Temperature reads -273°C**: TMC9660 communication issue or sensor not ready
2. **Out of range errors**: Check if temperature is within -40°C to +150°C
3. **Not initialized errors**: Ensure TMC9660 handler is properly initialized
4. **High consecutive error count**: Check TMC9660 communication interface

### Debug Information

Use the comprehensive diagnostics to troubleshoot issues:

```cpp
// Dump all TMC9660 diagnostics including temperature
tmc9660_handler.DumpDiagnostics();

// Get specific temperature diagnostics
hf_temp_diagnostics_t diagnostics;
temp_sensor.GetDiagnostics(diagnostics);
logger.Info("TMC9660", "Last error: %s\n", GetTempErrorString(diagnostics.last_error_code));
logger.Info("TMC9660", "Consecutive errors: %d\n", diagnostics.consecutive_errors);
```

## See Also

- [TMC9660 Handler Documentation](../TMC9660_HANDLER_README.md)
- [BaseTemperature Interface Documentation](../../development/BASE_TEMPERATURE_INTERFACE.md)
- [Temperature Management Guidelines](../../development/TEMPERATURE_MANAGEMENT_GUIDELINES.md) 