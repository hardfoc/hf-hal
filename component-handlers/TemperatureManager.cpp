/**
 * @file TemperatureManager.cpp
 * @brief Implementation of advanced temperature sensor management system for the HardFOC platform.
 *
 * This file implements the TemperatureManager class, which provides a comprehensive
 * temperature sensor management system that manages onboard temperature sensors
 * including ESP32 internal temperature sensor and NTC temperature sensors.
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "TemperatureManager.h"
#include "AdcManager.h"
#include "Logger.h"
#include <esp_timer.h>
#include <esp_log.h>

static const char* TAG = "TempManager";

//==============================================================================
// SINGLETON IMPLEMENTATION
//==============================================================================

TemperatureManager& TemperatureManager::GetInstance() noexcept {
    static TemperatureManager instance;
    return instance;
}

//==============================================================================
// CONSTRUCTOR AND DESTRUCTOR
//==============================================================================

TemperatureManager::TemperatureManager() noexcept
    : initialized_(false)
    , next_sensor_id_(0)
    , system_start_time_ms_(0) {
    
    // Initialize system diagnostics
    system_diagnostics_ = {};
    system_diagnostics_.system_healthy = true;
    system_diagnostics_.system_min_temp_celsius = FLT_MAX;
    system_diagnostics_.system_max_temp_celsius = -FLT_MAX;
    system_diagnostics_.system_avg_temp_celsius = 0.0f;
}

TemperatureManager::~TemperatureManager() noexcept {
    Shutdown();
}

//==============================================================================
// LIFECYCLE MANAGEMENT
//==============================================================================

hf_temp_err_t TemperatureManager::EnsureInitialized() noexcept {
    if (initialized_.load()) {
        return TEMP_SUCCESS;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_.load()) {
        return TEMP_SUCCESS;
    }
    
    return Initialize();
}

hf_temp_err_t TemperatureManager::Shutdown() noexcept {
    if (!initialized_.load()) {
        return TEMP_SUCCESS;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return TEMP_SUCCESS;
    }
    
    return Deinitialize();
}

hf_temp_err_t TemperatureManager::Initialize() noexcept {
    if (initialized_.load()) {
        ESP_LOGW(TAG, "Already initialized");
        return TEMP_SUCCESS;
    }
    
    // Clear any existing data
    sensors_.clear();
    sensor_map_.clear();
    next_sensor_id_ = 0;
    
    // Set system start time
    system_start_time_ms_.store(esp_timer_get_time() / 1000);
    
    // Initialize system diagnostics
    system_diagnostics_ = {};
    system_diagnostics_.system_healthy = true;
    system_diagnostics_.system_min_temp_celsius = FLT_MAX;
    system_diagnostics_.system_max_temp_celsius = -FLT_MAX;
    system_diagnostics_.system_avg_temp_celsius = 0.0f;
    
    initialized_.store(true);
    
    ESP_LOGI(TAG, "Temperature manager initialized successfully");
    return TEMP_SUCCESS;
}

hf_temp_err_t TemperatureManager::Deinitialize() noexcept {
    if (!initialized_.load()) {
        return TEMP_SUCCESS;
    }
    
    // Stop all continuous monitoring
    for (auto& sensor_info : sensors_) {
        if (sensor_info.sensor) {
            sensor_info.sensor->StopContinuousMonitoring();
        }
    }
    
    // Clear all sensors
    sensors_.clear();
    sensor_map_.clear();
    next_sensor_id_ = 0;
    
    initialized_.store(false);
    
    ESP_LOGI(TAG, "Temperature manager deinitialized");
    return TEMP_SUCCESS;
}

//==============================================================================
// TMC9660 TEMPERATURE WRAPPER IMPLEMENTATION
//==============================================================================

Tmc9660TemperatureWrapper::Tmc9660TemperatureWrapper(Tmc9660Handler& tmc9660_handler) noexcept
    : tmc9660_handler_(tmc9660_handler) {
}

bool Tmc9660TemperatureWrapper::Initialize() noexcept {
    if (IsInitialized()) {
        ESP_LOGW(TAG, "TMC9660 temperature wrapper already initialized");
        return true;
    }
    
    // Check if TMC9660 handler is ready
    if (!tmc9660_handler_.IsDriverReady()) {
        ESP_LOGE(TAG, "TMC9660 handler not ready");
        return false;
    }
    
    // Initialize the TMC9660 temperature wrapper
    Tmc9660Handler::Temperature& temp_wrapper = tmc9660_handler_.temperature();
    if (!temp_wrapper.EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize TMC9660 temperature wrapper");
        return false;
    }
    
    initialized_ = true;
    current_state_ = HF_TEMP_STATE_INITIALIZED;
    
    ESP_LOGI(TAG, "TMC9660 temperature wrapper initialized successfully");
    return true;
}

bool Tmc9660TemperatureWrapper::Deinitialize() noexcept {
    if (!IsInitialized()) {
        return true;
    }
    
    initialized_ = false;
    current_state_ = HF_TEMP_STATE_UNINITIALIZED;
    
    ESP_LOGI(TAG, "TMC9660 temperature wrapper deinitialized");
    return true;
}

hf_temp_err_t Tmc9660TemperatureWrapper::ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept {
    if (temperature_celsius == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    if (!IsInitialized()) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    // Check if TMC9660 handler is still ready
    if (!tmc9660_handler_.IsDriverReady()) {
        return TEMP_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Read temperature from TMC9660 temperature wrapper
    Tmc9660Handler::Temperature& temp_wrapper = tmc9660_handler_.temperature();
    return temp_wrapper.ReadTemperatureCelsius(temperature_celsius);
}

hf_temp_err_t Tmc9660TemperatureWrapper::GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept {
    if (info == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    if (!IsInitialized()) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    // Get sensor info from TMC9660 temperature wrapper
    Tmc9660Handler::Temperature& temp_wrapper = tmc9660_handler_.temperature();
    return temp_wrapper.GetSensorInfo(info);
}

hf_u32_t Tmc9660TemperatureWrapper::GetCapabilities() const noexcept {
    if (!IsInitialized()) {
        return HF_TEMP_CAP_NONE;
    }
    
    // Get capabilities from TMC9660 temperature wrapper
    Tmc9660Handler::Temperature& temp_wrapper = tmc9660_handler_.temperature();
    return temp_wrapper.GetCapabilities();
}

//==============================================================================
// SENSOR REGISTRATION AND MANAGEMENT
//==============================================================================

hf_temp_err_t TemperatureManager::RegisterEspTemperatureSensor(std::string_view name, 
                                                              const esp_temp_config_t& config) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    if (name.empty()) {
        return TEMP_ERR_INVALID_PARAMETER;
    }
    
    // Check if sensor already exists
    if (FindSensor(name) != sensors_.end()) {
        ESP_LOGW(TAG, "Sensor '%s' already registered", std::string(name).c_str());
        return TEMP_ERR_ALREADY_INITIALIZED;
    }
    
    // Create ESP32 temperature sensor
    auto esp_sensor = std::make_unique<EspTemperature>(config);
    if (!esp_sensor) {
        ESP_LOGE(TAG, "Failed to create ESP32 temperature sensor");
        return TEMP_ERR_OUT_OF_MEMORY;
    }
    
    // Initialize the sensor
    if (!esp_sensor->EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize ESP32 temperature sensor");
        return TEMP_ERR_FAILURE;
    }
    
    // Get sensor information for configuration
    hf_temp_sensor_info_t sensor_info = {};
    if (esp_sensor->GetSensorInfo(&sensor_info) != TEMP_SUCCESS) {
        ESP_LOGW(TAG, "Failed to get ESP32 sensor info, using defaults");
        sensor_info.min_temp_celsius = -40.0f;
        sensor_info.max_temp_celsius = 125.0f;
        sensor_info.resolution_celsius = 0.25f;
        sensor_info.accuracy_celsius = 1.0f;
    }
    
    // Create sensor info entry
    TempSensorInfo temp_info(name, std::move(esp_sensor), HF_TEMP_SENSOR_TYPE_INTERNAL, 
                            next_sensor_id_++, sensor_info.min_temp_celsius, sensor_info.max_temp_celsius,
                            sensor_info.resolution_celsius, sensor_info.accuracy_celsius);
    
    // Add to sensors list and map
    sensors_.push_back(std::move(temp_info));
    sensor_map_[name] = static_cast<uint32_t>(sensors_.size() - 1);
    
    // Update system diagnostics
    system_diagnostics_.total_sensors_registered++;
    system_diagnostics_.sensors_by_type[HF_TEMP_SENSOR_TYPE_INTERNAL]++;
    
    ESP_LOGI(TAG, "ESP32 temperature sensor '%s' registered successfully", std::string(name).c_str());
    return TEMP_SUCCESS;
}

hf_temp_err_t TemperatureManager::RegisterNtcTemperatureSensor(std::string_view name, 
                                                              std::string_view adc_channel_name,
                                                              const ntc_temp_handler_config_t& config) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    if (name.empty() || adc_channel_name.empty()) {
        return TEMP_ERR_INVALID_PARAMETER;
    }
    
    // Check if sensor already exists
    if (FindSensor(name) != sensors_.end()) {
        ESP_LOGW(TAG, "Sensor '%s' already registered", std::string(name).c_str());
        return TEMP_ERR_ALREADY_INITIALIZED;
    }
    
    // Get ADC interface from AdcManager
    AdcManager& adc_manager = AdcManager::GetInstance();
    if (adc_manager.EnsureInitialized() != hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGE(TAG, "Failed to initialize ADC manager");
        return TEMP_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Get ADC channel by name
    BaseAdc* adc_interface = adc_manager.GetAdcInterface(adc_channel_name);
    if (!adc_interface) {
        ESP_LOGE(TAG, "ADC channel '%s' not found", std::string(adc_channel_name).c_str());
        return TEMP_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Create NTC temperature sensor handler
    auto ntc_sensor = std::make_unique<NtcTemperatureHandler>(adc_interface, config);
    if (!ntc_sensor) {
        ESP_LOGE(TAG, "Failed to create NTC temperature sensor handler");
        return TEMP_ERR_OUT_OF_MEMORY;
    }
    
    // Initialize the sensor
    if (!ntc_sensor->EnsureInitialized()) {
        ESP_LOGE(TAG, "Failed to initialize NTC temperature sensor");
        return TEMP_ERR_FAILURE;
    }
    
    // Get sensor information for configuration
    hf_temp_sensor_info_t sensor_info = {};
    if (ntc_sensor->GetSensorInfo(&sensor_info) != TEMP_SUCCESS) {
        ESP_LOGW(TAG, "Failed to get NTC sensor info, using defaults");
        sensor_info.min_temp_celsius = -40.0f;
        sensor_info.max_temp_celsius = 125.0f;
        sensor_info.resolution_celsius = 0.1f;
        sensor_info.accuracy_celsius = 1.0f;
    }
    
    // Create sensor info entry
    TempSensorInfo temp_info(name, std::move(ntc_sensor), HF_TEMP_SENSOR_TYPE_THERMISTOR, 
                            next_sensor_id_++, sensor_info.min_temp_celsius, sensor_info.max_temp_celsius,
                            sensor_info.resolution_celsius, sensor_info.accuracy_celsius);
    
    // Add to sensors list and map
    sensors_.push_back(std::move(temp_info));
    sensor_map_[name] = static_cast<uint32_t>(sensors_.size() - 1);
    
    // Update system diagnostics
    system_diagnostics_.total_sensors_registered++;
    system_diagnostics_.sensors_by_type[HF_TEMP_SENSOR_TYPE_THERMISTOR]++;
    
    ESP_LOGI(TAG, "NTC temperature sensor '%s' registered successfully", std::string(name).c_str());
    return TEMP_SUCCESS;
}

hf_temp_err_t TemperatureManager::RegisterTmc9660TemperatureSensor(std::string_view name, 
                                                                   Tmc9660Handler& tmc9660_handler) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    if (name.empty()) {
        return TEMP_ERR_INVALID_PARAMETER;
    }
    
    // Check if sensor already exists
    if (FindSensor(name) != sensors_.end()) {
        ESP_LOGW(TAG, "Sensor '%s' already registered", std::string(name).c_str());
        return TEMP_ERR_ALREADY_INITIALIZED;
    }
    
    // Check if TMC9660 handler is initialized
    if (!tmc9660_handler.IsDriverReady()) {
        ESP_LOGE(TAG, "TMC9660 handler not ready");
        return TEMP_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Get TMC9660 temperature wrapper
    Tmc9660Handler::Temperature& tmc9660_temp = tmc9660_handler.temperature();
    
    // Check if temperature wrapper is available
    if (!tmc9660_temp.IsInitialized()) {
        ESP_LOGE(TAG, "TMC9660 temperature wrapper not initialized");
        return TEMP_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Get sensor information for configuration
    hf_temp_sensor_info_t sensor_info = {};
    if (tmc9660_temp.GetSensorInfo(&sensor_info) != TEMP_SUCCESS) {
        ESP_LOGW(TAG, "Failed to get TMC9660 sensor info, using defaults");
        sensor_info.min_temp_celsius = -40.0f;
        sensor_info.max_temp_celsius = 150.0f;
        sensor_info.resolution_celsius = 0.1f;
        sensor_info.accuracy_celsius = 2.0f;
    }
    
    // Create a unique_ptr to the TMC9660 temperature wrapper
    // Note: We need to create a wrapper that owns the reference to the TMC9660 temperature
    auto tmc9660_temp_wrapper = std::make_unique<Tmc9660TemperatureWrapper>(tmc9660_handler);
    
    // Create sensor info entry
    TempSensorInfo temp_info(name, std::move(tmc9660_temp_wrapper), HF_TEMP_SENSOR_TYPE_INTERNAL, 
                            next_sensor_id_++, sensor_info.min_temp_celsius, sensor_info.max_temp_celsius,
                            sensor_info.resolution_celsius, sensor_info.accuracy_celsius);
    
    // Add to sensors list and map
    sensors_.push_back(std::move(temp_info));
    sensor_map_[name] = static_cast<uint32_t>(sensors_.size() - 1);
    
    // Update system diagnostics
    system_diagnostics_.total_sensors_registered++;
    system_diagnostics_.sensors_by_type[HF_TEMP_SENSOR_TYPE_INTERNAL]++;
    
    ESP_LOGI(TAG, "TMC9660 temperature sensor '%s' registered successfully", std::string(name).c_str());
    return TEMP_SUCCESS;
}

hf_temp_err_t TemperatureManager::UnregisterSensor(std::string_view name) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    auto it = FindSensor(name);
    if (it == sensors_.end()) {
        ESP_LOGW(TAG, "Sensor '%s' not found", std::string(name).c_str());
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    // Stop continuous monitoring if active
    if (it->sensor) {
        it->sensor->StopContinuousMonitoring();
    }
    
    // Update system diagnostics
    system_diagnostics_.total_sensors_registered--;
    system_diagnostics_.sensors_by_type[it->sensor_type]--;
    
    // Remove from map and list
    sensor_map_.erase(name);
    sensors_.erase(it);
    
    // Rebuild map indices
    sensor_map_.clear();
    for (size_t i = 0; i < sensors_.size(); ++i) {
        sensor_map_[sensors_[i].name] = static_cast<uint32_t>(i);
    }
    
    ESP_LOGI(TAG, "Sensor '%s' unregistered successfully", std::string(name).c_str());
    return TEMP_SUCCESS;
}

bool TemperatureManager::IsSensorRegistered(std::string_view name) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return FindSensor(name) != sensors_.end();
}

uint32_t TemperatureManager::GetSensorCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint32_t>(sensors_.size());
}

const TempSensorInfo* TemperatureManager::GetSensorInfo(std::string_view name) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = FindSensor(name);
    return (it != sensors_.end()) ? &(*it) : nullptr;
}

const TempSensorInfo* TemperatureManager::GetSensorInfoByIndex(uint32_t index) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return (index < sensors_.size()) ? &sensors_[index] : nullptr;
}

std::vector<std::string_view> TemperatureManager::GetSensorNames() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string_view> names;
    names.reserve(sensors_.size());
    for (const auto& sensor : sensors_) {
        names.push_back(sensor.name);
    }
    return names;
}

//==============================================================================
// CORE TEMPERATURE OPERATIONS (STRING-BASED)
//==============================================================================

hf_temp_err_t TemperatureManager::ReadTemperatureCelsius(std::string_view sensor_name, float* temperature_celsius) noexcept {
    if (temperature_celsius == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = FindSensor(sensor_name);
    if (it == sensors_.end()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    if (!it->sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = it->sensor->ReadTemperatureCelsius(temperature_celsius);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&(*it), result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        it->last_reading_celsius = *temperature_celsius;
        it->last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperatureFahrenheit(std::string_view sensor_name, float* temperature_fahrenheit) noexcept {
    if (temperature_fahrenheit == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = FindSensor(sensor_name);
    if (it == sensors_.end()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    if (!it->sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = it->sensor->ReadTemperatureFahrenheit(temperature_fahrenheit);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&(*it), result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        it->last_reading_celsius = HF_TEMP_FAHRENHEIT_TO_CELSIUS(*temperature_fahrenheit);
        it->last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperatureKelvin(std::string_view sensor_name, float* temperature_kelvin) noexcept {
    if (temperature_kelvin == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = FindSensor(sensor_name);
    if (it == sensors_.end()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    if (!it->sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = it->sensor->ReadTemperatureKelvin(temperature_kelvin);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&(*it), result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        it->last_reading_celsius = HF_TEMP_KELVIN_TO_CELSIUS(*temperature_kelvin);
        it->last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperature(std::string_view sensor_name, hf_temp_reading_t* reading) noexcept {
    if (reading == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = FindSensor(sensor_name);
    if (it == sensors_.end()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    if (!it->sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = it->sensor->ReadTemperature(reading);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&(*it), result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        it->last_reading_celsius = reading->temperature_celsius;
        it->last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperatureUnit(std::string_view sensor_name, float* temperature, hf_temp_unit_t unit) noexcept {
    if (temperature == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = FindSensor(sensor_name);
    if (it == sensors_.end()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    if (!it->sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = it->sensor->ReadTemperatureUnit(temperature, unit);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&(*it), result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        // Convert to Celsius for storage
        switch (unit) {
            case HF_TEMP_UNIT_CELSIUS:
                it->last_reading_celsius = *temperature;
                break;
            case HF_TEMP_UNIT_FAHRENHEIT:
                it->last_reading_celsius = HF_TEMP_FAHRENHEIT_TO_CELSIUS(*temperature);
                break;
            case HF_TEMP_UNIT_KELVIN:
                it->last_reading_celsius = HF_TEMP_KELVIN_TO_CELSIUS(*temperature);
                break;
            case HF_TEMP_UNIT_RANKINE:
                it->last_reading_celsius = HF_TEMP_KELVIN_TO_CELSIUS(*temperature * 5.0f / 9.0f);
                break;
            default:
                break;
        }
        it->last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

//==============================================================================
// CORE TEMPERATURE OPERATIONS (INDEX-BASED)
//==============================================================================

hf_temp_err_t TemperatureManager::ReadTemperatureCelsiusByIndex(uint32_t index, float* temperature_celsius) noexcept {
    if (temperature_celsius == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= sensors_.size()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    auto& sensor_info = sensors_[index];
    if (!sensor_info.sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = sensor_info.sensor->ReadTemperatureCelsius(temperature_celsius);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&sensor_info, result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        sensor_info.last_reading_celsius = *temperature_celsius;
        sensor_info.last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperatureFahrenheitByIndex(uint32_t index, float* temperature_fahrenheit) noexcept {
    if (temperature_fahrenheit == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= sensors_.size()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    auto& sensor_info = sensors_[index];
    if (!sensor_info.sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = sensor_info.sensor->ReadTemperatureFahrenheit(temperature_fahrenheit);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&sensor_info, result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        sensor_info.last_reading_celsius = HF_TEMP_FAHRENHEIT_TO_CELSIUS(*temperature_fahrenheit);
        sensor_info.last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperatureKelvinByIndex(uint32_t index, float* temperature_kelvin) noexcept {
    if (temperature_kelvin == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= sensors_.size()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    auto& sensor_info = sensors_[index];
    if (!sensor_info.sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = sensor_info.sensor->ReadTemperatureKelvin(temperature_kelvin);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&sensor_info, result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        sensor_info.last_reading_celsius = HF_TEMP_KELVIN_TO_CELSIUS(*temperature_kelvin);
        sensor_info.last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

hf_temp_err_t TemperatureManager::ReadTemperatureByIndex(uint32_t index, hf_temp_reading_t* reading) noexcept {
    if (reading == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= sensors_.size()) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    auto& sensor_info = sensors_[index];
    if (!sensor_info.sensor) {
        return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    
    const auto start_time = esp_timer_get_time();
    hf_temp_err_t result = sensor_info.sensor->ReadTemperature(reading);
    const auto end_time = esp_timer_get_time();
    
    // Update statistics
    UpdateSensorStatistics(&sensor_info, result == TEMP_SUCCESS);
    
    if (result == TEMP_SUCCESS) {
        sensor_info.last_reading_celsius = reading->temperature_celsius;
        sensor_info.last_access_time = static_cast<uint64_t>(end_time / 1000); // Convert to ms
    }
    
    return result;
}

//==============================================================================
// PRIVATE HELPER METHODS
//==============================================================================

std::vector<TempSensorInfo>::iterator TemperatureManager::FindSensor(std::string_view name) noexcept {
    auto map_it = sensor_map_.find(name);
    if (map_it == sensor_map_.end()) {
        return sensors_.end();
    }
    return sensors_.begin() + map_it->second;
}

std::vector<TempSensorInfo>::const_iterator TemperatureManager::FindSensor(std::string_view name) const noexcept {
    auto map_it = sensor_map_.find(name);
    if (map_it == sensor_map_.end()) {
        return sensors_.end();
    }
    return sensors_.begin() + map_it->second;
}

void TemperatureManager::UpdateSensorStatistics(TempSensorInfo* sensor_info, bool operation_successful) noexcept {
    if (!sensor_info) {
        return;
    }
    
    sensor_info->access_count++;
    
    if (operation_successful) {
        system_diagnostics_.successful_operations++;
    } else {
        sensor_info->error_count++;
        system_diagnostics_.failed_operations++;
    }
    
    system_diagnostics_.total_operations++;
    
    // Update system temperature statistics
    if (operation_successful && sensor_info->last_reading_celsius != 0.0f) {
        if (sensor_info->last_reading_celsius < system_diagnostics_.system_min_temp_celsius) {
            system_diagnostics_.system_min_temp_celsius = sensor_info->last_reading_celsius;
        }
        if (sensor_info->last_reading_celsius > system_diagnostics_.system_max_temp_celsius) {
            system_diagnostics_.system_max_temp_celsius = sensor_info->last_reading_celsius;
        }
        
        // Update average (simple moving average)
        if (system_diagnostics_.successful_operations > 0) {
            system_diagnostics_.system_avg_temp_celsius = 
                (system_diagnostics_.system_avg_temp_celsius * (system_diagnostics_.successful_operations - 1) + 
                 sensor_info->last_reading_celsius) / system_diagnostics_.successful_operations;
        }
    }
    
    // Update system uptime
    system_diagnostics_.system_uptime_ms = 
        static_cast<uint64_t>(esp_timer_get_time() / 1000) - system_start_time_ms_.load();
} 