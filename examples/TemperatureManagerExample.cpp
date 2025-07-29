/**
 * @file TemperatureManagerExample.cpp
 * @brief Example demonstrating the usage of the TemperatureManager class.
 *
 * This example shows how to:
 * - Initialize the TemperatureManager
 * - Register ESP32 internal temperature sensor
 * - Register NTC temperature sensor
 * - Read temperatures from multiple sensors
 * - Use batch operations
 * - Monitor temperature thresholds
 * - Access sensor statistics and diagnostics
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "component-handlers/TemperatureManager.h"
#include "component-handlers/AdcManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "TempManagerExample";

//==============================================================================
// EXAMPLE CONFIGURATION
//==============================================================================

// NTC thermistor configuration
static const ntc_temp_handler_config_t NTC_CONFIG = NTC_TEMP_HANDLER_CONFIG_DEFAULT_NTCG163JFT103FT1S();

// ESP32 temperature sensor configuration
static const esp_temp_config_t ESP_TEMP_CONFIG = ESP_TEMP_CONFIG_DEFAULT();

//==============================================================================
// CALLBACK FUNCTIONS
//==============================================================================

void temperature_threshold_callback(BaseTemperature* sensor, float temperature_celsius, 
                                   uint32_t threshold_type, void* user_data) {
    const char* threshold_name = (threshold_type == 0) ? "LOW" : "HIGH";
    ESP_LOGW(TAG, "Temperature threshold exceeded: %.2f°C (%s)", temperature_celsius, threshold_name);
}

void temperature_reading_callback(BaseTemperature* sensor, const hf_temp_reading_t* reading, 
                                 void* user_data) {
    if (reading && reading->is_valid) {
        ESP_LOGI(TAG, "Continuous reading: %.2f°C (accuracy: %.2f°C)", 
                 reading->temperature_celsius, reading->accuracy_celsius);
    }
}

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

void example_basic_temperature_reading() {
    ESP_LOGI(TAG, "=== Basic Temperature Reading Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Initialize temperature manager
    if (temp_manager.EnsureInitialized() != TEMP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize temperature manager");
        return;
    }
    
    // Register ESP32 internal temperature sensor
    if (temp_manager.RegisterEspTemperatureSensor("esp32_internal", ESP_TEMP_CONFIG) != TEMP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to register ESP32 temperature sensor");
        return;
    }
    
    // Read temperature from ESP32 internal sensor
    float esp32_temp;
    if (temp_manager.ReadTemperatureCelsius("esp32_internal", &esp32_temp) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "ESP32 Internal Temperature: %.2f°C", esp32_temp);
        
        // Convert to Fahrenheit
        float esp32_temp_f;
        if (temp_manager.ReadTemperatureFahrenheit("esp32_internal", &esp32_temp_f) == TEMP_SUCCESS) {
            ESP_LOGI(TAG, "ESP32 Internal Temperature: %.2f°F", esp32_temp_f);
        }
        
        // Convert to Kelvin
        float esp32_temp_k;
        if (temp_manager.ReadTemperatureKelvin("esp32_internal", &esp32_temp_k) == TEMP_SUCCESS) {
            ESP_LOGI(TAG, "ESP32 Internal Temperature: %.2f K", esp32_temp_k);
        }
    }
    
    // Get sensor information
    hf_temp_sensor_info_t sensor_info = {};
    if (temp_manager.GetSensorInfo("esp32_internal", &sensor_info) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Sensor Info:");
        ESP_LOGI(TAG, "  Type: %d", sensor_info.sensor_type);
        ESP_LOGI(TAG, "  Range: %.1f°C to %.1f°C", sensor_info.min_temp_celsius, sensor_info.max_temp_celsius);
        ESP_LOGI(TAG, "  Resolution: %.2f°C", sensor_info.resolution_celsius);
        ESP_LOGI(TAG, "  Accuracy: %.2f°C", sensor_info.accuracy_celsius);
        ESP_LOGI(TAG, "  Response Time: %u ms", sensor_info.response_time_ms);
    }
}

void example_ntc_temperature_sensor() {
    ESP_LOGI(TAG, "=== NTC Temperature Sensor Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    AdcManager& adc_manager = AdcManager::GetInstance();
    
    // Initialize ADC manager first
    if (adc_manager.EnsureInitialized() != hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize ADC manager");
        return;
    }
    
    // Register NTC temperature sensor (assuming ADC channel "ntc_thermistor" exists)
    if (temp_manager.RegisterNtcTemperatureSensor("ntc_sensor", "ntc_thermistor", NTC_CONFIG) != TEMP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to register NTC temperature sensor");
        return;
    }
    
    // Read temperature from NTC sensor
    float ntc_temp;
    if (temp_manager.ReadTemperatureCelsius("ntc_sensor", &ntc_temp) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "NTC Temperature: %.2f°C", ntc_temp);
    }
    
    // Get NTC sensor capabilities
    uint32_t capabilities = temp_manager.GetSensorCapabilities("ntc_sensor");
    ESP_LOGI(TAG, "NTC Sensor Capabilities: 0x%08X", capabilities);
    
    if (temp_manager.HasSensorCapability("ntc_sensor", HF_TEMP_CAP_CALIBRATION)) {
        ESP_LOGI(TAG, "NTC sensor supports calibration");
        
        // Perform calibration with known reference temperature
        if (temp_manager.Calibrate("ntc_sensor", 25.0f) == TEMP_SUCCESS) {
            ESP_LOGI(TAG, "NTC sensor calibrated at 25.0°C");
        }
    }
}

void example_tmc9660_temperature_sensor() {
    ESP_LOGI(TAG, "=== TMC9660 Temperature Sensor Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Note: In a real application, you would have a TMC9660Handler instance
    // For this example, we'll show the registration pattern
    // TMC9660Handler& tmc9660_handler = GetTmc9660HandlerInstance(); // Get from your system
    
    // Register TMC9660 temperature sensor
    // if (temp_manager.RegisterTmc9660TemperatureSensor("tmc9660_chip", tmc9660_handler) != TEMP_SUCCESS) {
    //     ESP_LOGE(TAG, "Failed to register TMC9660 temperature sensor");
    //     return;
    // }
    
    // Read temperature from TMC9660 sensor
    // float tmc9660_temp;
    // if (temp_manager.ReadTemperatureCelsius("tmc9660_chip", &tmc9660_temp) == TEMP_SUCCESS) {
    //     ESP_LOGI(TAG, "TMC9660 Chip Temperature: %.2f°C", tmc9660_temp);
    // }
    
    // Get TMC9660 sensor capabilities
    // uint32_t capabilities = temp_manager.GetSensorCapabilities("tmc9660_chip");
    // ESP_LOGI(TAG, "TMC9660 Sensor Capabilities: 0x%08X", capabilities);
    
    ESP_LOGI(TAG, "TMC9660 temperature sensor integration demonstrated (commented out for example)");
}

void example_batch_operations() {
    ESP_LOGI(TAG, "=== Batch Operations Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Read temperatures from multiple sensors
    std::vector<std::string_view> sensor_names = {"esp32_internal"};
    
    // Add NTC sensor if available
    if (temp_manager.IsSensorRegistered("ntc_sensor")) {
        sensor_names.push_back("ntc_sensor");
    }
    
    // Add TMC9660 sensor if available
    if (temp_manager.IsSensorRegistered("tmc9660_chip")) {
        sensor_names.push_back("tmc9660_chip");
    }
    
    std::vector<float> temperatures_celsius;
    if (temp_manager.ReadMultipleTemperaturesCelsius(sensor_names, temperatures_celsius) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Batch temperature readings:");
        for (size_t i = 0; i < sensor_names.size(); ++i) {
            ESP_LOGI(TAG, "  %s: %.2f°C", std::string(sensor_names[i]).c_str(), temperatures_celsius[i]);
        }
    }
    
    // Perform batch operation with configuration
    TempBatchOperation batch_op(sensor_names, 3, 100); // 3 samples, 100ms interval
    TempBatchResult batch_result = temp_manager.PerformBatchRead(batch_op);
    
    if (batch_result.AllSuccessful()) {
        ESP_LOGI(TAG, "Batch operation successful (%.1f%% success rate)", batch_result.GetSuccessRate());
        ESP_LOGI(TAG, "Total time: %u ms", batch_result.total_time_ms);
        
        for (size_t i = 0; i < batch_result.sensor_names.size(); ++i) {
            ESP_LOGI(TAG, "  %s: %.2f°C", std::string(batch_result.sensor_names[i]).c_str(), 
                     batch_result.temperatures_celsius[i]);
        }
    }
}

void example_threshold_monitoring() {
    ESP_LOGI(TAG, "=== Threshold Monitoring Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Set thresholds for ESP32 internal sensor
    if (temp_manager.SetThresholds("esp32_internal", 20.0f, 80.0f) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Set thresholds: 20.0°C to 80.0°C");
        
        // Enable threshold monitoring
        if (temp_manager.EnableThresholdMonitoring("esp32_internal", temperature_threshold_callback, nullptr) == TEMP_SUCCESS) {
            ESP_LOGI(TAG, "Threshold monitoring enabled");
            
            // Read temperature to trigger threshold check
            float temp;
            temp_manager.ReadTemperatureCelsius("esp32_internal", &temp);
            
            // Disable threshold monitoring
            temp_manager.DisableThresholdMonitoring("esp32_internal");
        }
    }
}

void example_continuous_monitoring() {
    ESP_LOGI(TAG, "=== Continuous Monitoring Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Start continuous monitoring for ESP32 internal sensor
    if (temp_manager.StartContinuousMonitoring("esp32_internal", 1, temperature_reading_callback, nullptr) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Continuous monitoring started at 1 Hz");
        
        // Let it run for a few seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // Stop continuous monitoring
        temp_manager.StopContinuousMonitoring("esp32_internal");
        ESP_LOGI(TAG, "Continuous monitoring stopped");
    }
}

void example_statistics_and_diagnostics() {
    ESP_LOGI(TAG, "=== Statistics and Diagnostics Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Get sensor statistics
    hf_temp_statistics_t statistics = {};
    if (temp_manager.GetSensorStatistics("esp32_internal", statistics) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "ESP32 Internal Sensor Statistics:");
        ESP_LOGI(TAG, "  Total Operations: %u", statistics.total_operations);
        ESP_LOGI(TAG, "  Successful Operations: %u", statistics.successful_operations);
        ESP_LOGI(TAG, "  Failed Operations: %u", statistics.failed_operations);
        ESP_LOGI(TAG, "  Temperature Readings: %u", statistics.temperature_readings);
        ESP_LOGI(TAG, "  Min Temperature: %.2f°C", statistics.min_temperature_celsius);
        ESP_LOGI(TAG, "  Max Temperature: %.2f°C", statistics.max_temperature_celsius);
        ESP_LOGI(TAG, "  Avg Temperature: %.2f°C", statistics.avg_temperature_celsius);
    }
    
    // Get sensor diagnostics
    hf_temp_diagnostics_t diagnostics = {};
    if (temp_manager.GetSensorDiagnostics("esp32_internal", diagnostics) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "ESP32 Internal Sensor Diagnostics:");
        ESP_LOGI(TAG, "  Sensor Healthy: %s", diagnostics.sensor_healthy ? "Yes" : "No");
        ESP_LOGI(TAG, "  Sensor Available: %s", diagnostics.sensor_available ? "Yes" : "No");
        ESP_LOGI(TAG, "  Threshold Monitoring: %s", diagnostics.threshold_monitoring_enabled ? "Enabled" : "Disabled");
        ESP_LOGI(TAG, "  Continuous Monitoring: %s", diagnostics.continuous_monitoring_active ? "Active" : "Inactive");
        ESP_LOGI(TAG, "  Calibration Valid: %s", diagnostics.calibration_valid ? "Yes" : "No");
        ESP_LOGI(TAG, "  Consecutive Errors: %u", diagnostics.consecutive_errors);
    }
    
    // Get system diagnostics
    TempSystemDiagnostics system_diagnostics = {};
    if (temp_manager.GetSystemDiagnostics(system_diagnostics) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "System Diagnostics:");
        ESP_LOGI(TAG, "  System Healthy: %s", system_diagnostics.system_healthy ? "Yes" : "No");
        ESP_LOGI(TAG, "  Total Sensors: %u", system_diagnostics.total_sensors_registered);
        ESP_LOGI(TAG, "  Total Operations: %u", system_diagnostics.total_operations);
        ESP_LOGI(TAG, "  Successful Operations: %u", system_diagnostics.successful_operations);
        ESP_LOGI(TAG, "  Failed Operations: %u", system_diagnostics.failed_operations);
        ESP_LOGI(TAG, "  System Min Temp: %.2f°C", system_diagnostics.system_min_temp_celsius);
        ESP_LOGI(TAG, "  System Max Temp: %.2f°C", system_diagnostics.system_max_temp_celsius);
        ESP_LOGI(TAG, "  System Avg Temp: %.2f°C", system_diagnostics.system_avg_temp_celsius);
        ESP_LOGI(TAG, "  System Uptime: %llu ms", system_diagnostics.system_uptime_ms);
    }
}

void example_utility_functions() {
    ESP_LOGI(TAG, "=== Utility Functions Example ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Get all sensor names
    std::vector<std::string_view> sensor_names = temp_manager.GetSensorNames();
    ESP_LOGI(TAG, "Registered sensors (%zu):", sensor_names.size());
    for (const auto& name : sensor_names) {
        ESP_LOGI(TAG, "  - %s", std::string(name).c_str());
    }
    
    // Get sensors by type
    std::vector<std::string_view> internal_sensors = temp_manager.GetSensorsByType(HF_TEMP_SENSOR_TYPE_INTERNAL);
    ESP_LOGI(TAG, "Internal sensors (%zu):", internal_sensors.size());
    for (const auto& name : internal_sensors) {
        ESP_LOGI(TAG, "  - %s", std::string(name).c_str());
    }
    
    std::vector<std::string_view> thermistor_sensors = temp_manager.GetSensorsByType(HF_TEMP_SENSOR_TYPE_THERMISTOR);
    ESP_LOGI(TAG, "Thermistor sensors (%zu):", thermistor_sensors.size());
    for (const auto& name : thermistor_sensors) {
        ESP_LOGI(TAG, "  - %s", std::string(name).c_str());
    }
    
    // Get sensors with specific capability
    std::vector<std::string_view> calibrated_sensors = temp_manager.GetSensorsWithCapability(HF_TEMP_CAP_CALIBRATION);
    ESP_LOGI(TAG, "Sensors with calibration capability (%zu):", calibrated_sensors.size());
    for (const auto& name : calibrated_sensors) {
        ESP_LOGI(TAG, "  - %s", std::string(name).c_str());
    }
    
    // Get system temperature statistics
    float min_temp, max_temp, avg_temp;
    if (temp_manager.GetSystemTemperatureStats(&min_temp, &max_temp, &avg_temp) == TEMP_SUCCESS) {
        ESP_LOGI(TAG, "System Temperature Statistics:");
        ESP_LOGI(TAG, "  Min: %.2f°C", min_temp);
        ESP_LOGI(TAG, "  Max: %.2f°C", max_temp);
        ESP_LOGI(TAG, "  Avg: %.2f°C", avg_temp);
    }
}

//==============================================================================
// MAIN EXAMPLE FUNCTION
//==============================================================================

extern "C" void app_main() {
    ESP_LOGI(TAG, "Temperature Manager Example Starting...");
    
    // Initialize logger
    Logger::GetInstance().Initialize();
    
    // Run examples
    example_basic_temperature_reading();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_ntc_temperature_sensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_tmc9660_temperature_sensor();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_batch_operations();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_threshold_monitoring();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_continuous_monitoring();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_statistics_and_diagnostics();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_utility_functions();
    
    ESP_LOGI(TAG, "Temperature Manager Example Completed!");
} 