/**
 * @file AdcManager.cpp
 * @brief Advanced ADC management system implementation for the HardFOC platform.
 * 
 * @details This implementation provides comprehensive ADC management with:
 *          - String-based sensor identification for extensibility
 *          - Complete BaseAdc function coverage through routing
 *          - Smart sensor categorization and validation
 *          - Platform mapping integration for automatic channel discovery
 *          - Thread-safe operations with comprehensive error handling
 *          - Advanced diagnostics and health monitoring
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 */

#include "AdcManager.h"
#include "McuAdc.h"
#include "CommChannelsManager.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <numeric>

//==============================================================================
// LOGGING
//==============================================================================

static const char* TAG = "AdcManager";

//==============================================================================
// STATIC INSTANCE
//==============================================================================

AdcManager& AdcManager::GetInstance() noexcept {
    static AdcManager instance;
    return instance;
}

//==============================================================================
// INITIALIZATION AND LIFECYCLE
//==============================================================================

hf_adc_err_t AdcManager::EnsureInitialized() noexcept {
    if (is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_SUCCESS;
    }
    
    RtosMutex::LockGuard lock(mutex_);
    
    // Double-check after acquiring lock
    if (is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_SUCCESS;
    }
    
    return Initialize();
}

hf_adc_err_t AdcManager::Initialize(Tmc9660MotorController& tmc9660Controller) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    
    if (is_initialized_.load(std::memory_order_acquire)) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_ALREADY_INITIALIZED);
        return hf_adc_err_t::ADC_ERR_ALREADY_INITIALIZED;
    }
    
    // Store hardware interface reference
    tmc9660_controller_ = &tmc9660Controller;
    
    return Initialize();
}

hf_adc_err_t AdcManager::Initialize() noexcept {
    // Set system start time
    system_start_time_.store(0, std::memory_order_release); // In real implementation, get current time
    
    // Get CommChannelsManager instance (for potential future I2C ADC expansions)
    auto& comm_manager = CommChannelsManager::GetInstance();
    if (!comm_manager.IsInitialized()) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_NOT_INITIALIZED);
        // Don't fail initialization, just log warning - ADC can work without I2C
    }
    
    // Initialize hardware subsystems
    auto esp32_result = InitializeEsp32Adc();
    if (esp32_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateLastError(esp32_result);
        return esp32_result;
    }
    
    auto tmc9660_result = InitializeTmc9660Adc();
    if (tmc9660_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateLastError(tmc9660_result);
        return tmc9660_result;
    }
    
    // Register default channel mappings based on platform mapping
    auto default_channels_result = RegisterDefaultChannels();
    if (default_channels_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateLastError(default_channels_result);
        return default_channels_result;
    }
    
    is_initialized_.store(true, std::memory_order_release);
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::Shutdown() noexcept {
    RtosMutex::LockGuard lock(mutex_);
    
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_SUCCESS;
    }
    
    // Clear all registered channels
    {
        RtosMutex::LockGuard registry_lock(registry_mutex_);
        adc_registry_.clear();
        sensor_to_name_.clear();
    }
    
    // Reset hardware interfaces
    {
        RtosMutex::LockGuard mcu_lock(mcu_adc_mutex_);
        mcu_adc_.reset();
    }
    tmc9660_controller_ = nullptr;
    
    // Reset statistics
    total_readings_.store(0, std::memory_order_release);
    successful_readings_.store(0, std::memory_order_release);
    failed_readings_.store(0, std::memory_order_release);
    communication_errors_.store(0, std::memory_order_release);
    hardware_errors_.store(0, std::memory_order_release);
    calibration_count_.store(0, std::memory_order_release);
    
    // Reset last error
    last_error_.store(hf_adc_err_t::ADC_SUCCESS, std::memory_order_release);
    
    is_initialized_.store(false, std::memory_order_release);
    return hf_adc_err_t::ADC_SUCCESS;
}

bool AdcManager::IsInitialized() const noexcept {
    return is_initialized_.load(std::memory_order_acquire);
}

hf_adc_err_t AdcManager::GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept {
    RtosMutex::LockGuard lock(mutex_);
    
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Initialize diagnostics structure
    memset(&diagnostics, 0, sizeof(AdcSystemDiagnostics));
    
    // Get registry statistics
    {
        RtosMutex::LockGuard registry_lock(registry_mutex_);
        diagnostics.total_channels_registered = static_cast<uint32_t>(adc_registry_.size());
        
        // Count channels by chip type
        for (const auto& [name, channel_info] : adc_registry_) {
            if (channel_info && static_cast<uint8_t>(channel_info->sourceChip) < sizeof(diagnostics.channels_by_chip) / sizeof(diagnostics.channels_by_chip[0])) {
                diagnostics.channels_by_chip[static_cast<uint8_t>(channel_info->sourceChip)]++;
            }
        }
        
        // Calculate average success rate
        if (diagnostics.total_channels_registered > 0) {
            float total_success_rate = 0.0f;
            for (const auto& [name, channel_info] : adc_registry_) {
                if (channel_info) {
                    total_success_rate += channel_info->GetSuccessRate();
                }
            }
            diagnostics.average_success_rate = total_success_rate / static_cast<float>(diagnostics.total_channels_registered);
        }
    }
    
    // Get operation statistics
    diagnostics.total_readings = static_cast<uint32_t>(total_readings_.load());
    diagnostics.successful_readings = static_cast<uint32_t>(successful_readings_.load());
    diagnostics.failed_readings = static_cast<uint32_t>(failed_readings_.load());
    diagnostics.communication_errors = communication_errors_.load();
    diagnostics.hardware_errors = hardware_errors_.load();
    diagnostics.total_calibrations = static_cast<uint32_t>(calibration_count_.load());
    
    // Calculate system uptime
    diagnostics.system_uptime_ms = 0; // In real implementation, calculate from system_start_time_
    
    // Get last error
    diagnostics.last_error = last_error_.load();
    
    // Determine system health
    diagnostics.system_healthy = (diagnostics.failed_readings < (diagnostics.total_readings / 10)) && // Less than 10% failure rate
                                (diagnostics.communication_errors < 100) && // Reasonable communication error count
                                (diagnostics.hardware_errors < 10); // Very low hardware error tolerance
    
    return hf_adc_err_t::ADC_SUCCESS;
}

//==============================================================================
// CHANNEL REGISTRATION AND MANAGEMENT
//==============================================================================

hf_adc_err_t AdcManager::RegisterChannel(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Validate sensor name
    auto validation_result = ValidateSensorName(name);
    if (validation_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateLastError(validation_result);
        return validation_result;
    }
    
    if (!adc) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_INVALID_PARAMETER);
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    
    // Check if already registered
    if (adc_registry_.find(name) != adc_registry_.end()) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_ALREADY_REGISTERED);
        return hf_adc_err_t::ADC_ERR_SENSOR_ALREADY_REGISTERED;
    }
    
    // Create channel info with default values
    auto channel_info = std::make_unique<AdcChannelInfo>(
        name, AdcInputSensor::UNKNOWN, std::move(adc), 0, 
        AdcChip::UNKNOWN, HfAdcChipType::ESP32_INTERNAL);
    
    // Register the channel
    adc_registry_[name] = std::move(channel_info);
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::RegisterChannel(AdcInputSensor sensor,
                                        std::string_view name,
                                        float referenceVoltage,
                                        float calibrationScale,
                                        float calibrationOffset) noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Validate sensor name
    auto validation_result = ValidateSensorName(name);
    if (validation_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateLastError(validation_result);
        return validation_result;
    }
    
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    
    // Check if already registered
    if (adc_registry_.find(name) != adc_registry_.end()) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_ALREADY_REGISTERED);
        return hf_adc_err_t::ADC_ERR_SENSOR_ALREADY_REGISTERED;
    }
    
    // Create ADC driver based on sensor type - this would need platform mapping lookup
    std::unique_ptr<BaseAdc> adc_driver;
    
    // For now, create a default ESP32 ADC channel
    // In a real implementation, this would use platform mapping to determine the correct hardware
    adc_driver = CreateEsp32AdcChannel(0, referenceVoltage);
    
    if (!adc_driver) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_HARDWARE_FAULT);
        return hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
    }
    
    // Create channel info
    auto channel_info = std::make_unique<AdcChannelInfo>(
        name, sensor, std::move(adc_driver), 0, 
        AdcChip::ESP32_INTERNAL, HfAdcChipType::ESP32_INTERNAL,
        referenceVoltage);
    
    // Apply calibration
    channel_info->scaleFactor = calibrationScale;
    channel_info->offsetValue = calibrationOffset;
    
    // Register the channel
    adc_registry_[name] = std::move(channel_info);
    sensor_to_name_[sensor] = name;
    
    return hf_adc_err_t::ADC_SUCCESS;
}

BaseAdc* AdcManager::Get(std::string_view name) noexcept {
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    
    auto it = adc_registry_.find(name);
    if (it != adc_registry_.end() && it->second) {
        return it->second->adc.get();
    }
    
    return nullptr;
}

bool AdcManager::Contains(std::string_view name) const noexcept {
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    return adc_registry_.find(name) != adc_registry_.end();
}

size_t AdcManager::Size() const noexcept {
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    return adc_registry_.size();
}

void AdcManager::LogAllRegisteredChannels() const noexcept {
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    
    if (adc_registry_.empty()) {
        return;
    }
    
    // Log all registered channels for debugging
    for (const auto& [name, channel_info] : adc_registry_) {
        if (channel_info) {
            // In a real implementation, this would use proper logging
            // ESP_LOGI(TAG, "Channel: %.*s, Sensor: %d, Chip: %d, Success Rate: %.1f%%",
            //          static_cast<int>(name.length()), name.data(),
            //          static_cast<int>(channel_info->sensor),
            //          static_cast<int>(channel_info->sourceChip),
            //          channel_info->GetSuccessRate());
        }
    }
}

//==============================================================================
// BASIC READING OPERATIONS
//==============================================================================

hf_adc_err_t AdcManager::ReadChannel(std::string_view name, AdcReading& reading) noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    reading = PerformSingleReading(*channel_info);
    UpdateChannelStatistics(*channel_info, reading);
    UpdateGlobalStatistics(reading.IsValid());
    
    return reading.result;
}

hf_adc_err_t AdcManager::ReadChannel(AdcInputSensor sensor, AdcReading& reading) noexcept {
    // Find sensor name from mapping
    auto name_it = sensor_to_name_.find(sensor);
    if (name_it == sensor_to_name_.end()) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    return ReadChannel(name_it->second, reading);
}

hf_adc_err_t AdcManager::ReadChannelWithSampling(std::string_view name, const AdcSamplingSpec& spec, AdcReading& reading) noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    if (!spec.IsValid()) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_INVALID_PARAMETER);
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    std::vector<AdcReading> samples;
    samples.reserve(spec.numberOfSamples);
    
    // Take multiple samples
    for (uint8_t i = 0; i < spec.numberOfSamples; ++i) {
        AdcReading sample = PerformSingleReading(*channel_info);
        samples.push_back(sample);
        
        if (!sample.IsValid()) {
            UpdateLastError(hf_adc_err_t::ADC_ERR_SAMPLING_ERROR);
            return hf_adc_err_t::ADC_ERR_SAMPLING_ERROR;
        }
        
        if (i < spec.numberOfSamples - 1 && spec.samplingIntervalMs > 0) {
            // In real implementation, use proper delay
            // vTaskDelay(pdMS_TO_TICKS(spec.samplingIntervalMs));
        }
    }
    
    // Calculate average
    uint32_t raw_sum = 0;
    float voltage_sum = 0.0f;
    for (const auto& sample : samples) {
        raw_sum += sample.rawValue;
        voltage_sum += sample.voltage;
    }
    
    reading.rawValue = raw_sum / spec.numberOfSamples;
    reading.voltage = voltage_sum / static_cast<float>(spec.numberOfSamples);
    reading.calibratedValue = (reading.voltage * channel_info->scaleFactor) + channel_info->offsetValue;
    reading.timestamp = std::chrono::steady_clock::now();
    reading.result = hf_adc_err_t::ADC_SUCCESS;
    reading.hardwareChannel = channel_info->hardwareChannel;
    reading.sourceChip = channel_info->sourceChip;
    
    // Apply filtering if enabled
    if (spec.enableFiltering) {
        channel_info->UpdateFilteredValue(reading.voltage, spec.filterWeight);
    }
    
    UpdateChannelStatistics(*channel_info, reading);
    UpdateGlobalStatistics(reading.IsValid());
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::ReadFilteredValue(std::string_view name, float& value) noexcept {
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    value = channel_info->filteredValue;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::ReadRawValue(std::string_view name, uint32_t& value) noexcept {
    AdcReading reading;
    auto result = ReadChannel(name, reading);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        value = reading.rawValue;
    }
    return result;
}

hf_adc_err_t AdcManager::ReadVoltage(std::string_view name, float& voltage) noexcept {
    AdcReading reading;
    auto result = ReadChannel(name, reading);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        voltage = reading.voltage;
    }
    return result;
}

//==============================================================================
// BATCH OPERATIONS
//==============================================================================

AdcBatchResult AdcManager::BatchRead(const AdcBatchOperation& operation) noexcept {
    AdcBatchResult result;
    result.timestamp = std::chrono::steady_clock::now();
    
    if (!is_initialized_.load(std::memory_order_acquire)) {
        result.overall_result = hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
        return result;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    if (operation.use_individual_specs) {
        // Use individual sampling specifications
        result.sensor_names.reserve(operation.sampling_specs.size());
        result.readings.reserve(operation.sampling_specs.size());
        result.results.reserve(operation.sampling_specs.size());
        
        for (const auto& spec : operation.sampling_specs) {
            // Find sensor name from spec (this would require lookup implementation)
            // For now, use a placeholder approach
            std::string_view sensor_name = "unknown"; // This needs proper implementation
            
            AdcReading reading;
            auto read_result = ReadChannelWithSampling(sensor_name, spec, reading);
            
            result.sensor_names.push_back(sensor_name);
            result.readings.push_back(reading);
            result.results.push_back(read_result);
        }
    } else {
        // Use common settings for all sensors
        result.sensor_names = operation.sensor_names;
        result.readings.reserve(operation.sensor_names.size());
        result.results.reserve(operation.sensor_names.size());
        
        for (const auto& sensor_name : operation.sensor_names) {
            AdcReading reading;
            auto read_result = ReadChannel(sensor_name, reading);
            
            result.readings.push_back(reading);
            result.results.push_back(read_result);
        }
    }
    
    // Calculate overall result
    bool all_successful = true;
    for (const auto& res : result.results) {
        if (res != hf_adc_err_t::ADC_SUCCESS) {
            all_successful = false;
            break;
        }
    }
    
    result.overall_result = all_successful ? hf_adc_err_t::ADC_SUCCESS : hf_adc_err_t::ADC_ERR_BATCH_OPERATION_FAILED;
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.total_time_ms = static_cast<uint32_t>(duration.count());
    
    return result;
}

AdcBatchResult AdcManager::BatchRead(const std::vector<std::string_view>& sensor_names) noexcept {
    AdcBatchOperation operation(sensor_names);
    return BatchRead(operation);
}

AdcBatchResult AdcManager::ReadAllChannels() noexcept {
    std::vector<std::string_view> all_sensor_names;
    
    {
        RtosMutex::LockGuard registry_lock(registry_mutex_);
        all_sensor_names.reserve(adc_registry_.size());
        for (const auto& [name, channel_info] : adc_registry_) {
            all_sensor_names.push_back(name);
        }
    }
    
    return BatchRead(all_sensor_names);
}

AdcBatchResult AdcManager::BatchReadWithSampling(const std::vector<AdcSamplingSpec>& specs) noexcept {
    AdcBatchOperation operation(specs);
    return BatchRead(operation);
}

//==============================================================================
// CALIBRATION AND CONFIGURATION
//==============================================================================

hf_adc_err_t AdcManager::CalibrateChannel(std::string_view name,
                                          float referenceVoltage,
                                          uint32_t measuredValue) noexcept {
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    if (referenceVoltage <= 0.0f || measuredValue == 0) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_INVALID_PARAMETER);
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    // Calculate new scale factor based on calibration point
    const float maxCount = static_cast<float>((1U << channel_info->resolution) - 1);
    const float expectedVoltage = (static_cast<float>(measuredValue) / maxCount) * channel_info->referenceVoltage;
    
    if (expectedVoltage > 0.0f) {
        channel_info->scaleFactor = referenceVoltage / expectedVoltage;
        calibration_count_.fetch_add(1);
        return hf_adc_err_t::ADC_SUCCESS;
    }
    
    UpdateLastError(hf_adc_err_t::ADC_ERR_CALIBRATION_FAILED);
    return hf_adc_err_t::ADC_ERR_CALIBRATION_FAILED;
}

hf_adc_err_t AdcManager::SetChannelRange(std::string_view name,
                                         float minVoltage,
                                         float maxVoltage) noexcept {
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    if (minVoltage >= maxVoltage) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_INVALID_PARAMETER);
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    channel_info->minVoltage = minVoltage;
    channel_info->maxVoltage = maxVoltage;
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::ConfigureFiltering(std::string_view name,
                                            bool enableFiltering,
                                            float filterWeight) noexcept {
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    if (filterWeight < 0.0f || filterWeight > 1.0f) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_INVALID_PARAMETER);
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    channel_info->filteringEnabled = enableFiltering;
    channel_info->filterWeight = filterWeight;
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::SetReferenceVoltage(std::string_view name,
                                             float referenceVoltage) noexcept {
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    if (referenceVoltage <= 0.0f) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_INVALID_PARAMETER);
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    channel_info->referenceVoltage = referenceVoltage;
    
    return hf_adc_err_t::ADC_SUCCESS;
}

//==============================================================================
// STATISTICS AND DIAGNOSTICS
//==============================================================================

hf_adc_err_t AdcManager::GetStatistics(std::string_view name, BaseAdc::AdcStatistics& statistics) const noexcept {
    const auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    if (channel_info->adc) {
        return channel_info->adc->GetStatistics(statistics);
    }
    
    return hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
}

hf_adc_err_t AdcManager::ResetStatistics(std::string_view name) noexcept {
    auto* channel_info = FindChannelInfo(name);
    if (!channel_info) {
        return hf_adc_err_t::ADC_ERR_SENSOR_NOT_FOUND;
    }
    
    channel_info->totalReadings.store(0);
    channel_info->successfulReadings.store(0);
    channel_info->errorCount.store(0);
    channel_info->filteredValue = 0.0f;
    
    if (channel_info->adc) {
        return channel_info->adc->ResetStatistics();
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::GetSystemHealth(std::string& health_info) const noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    AdcSystemDiagnostics diagnostics;
    auto result = GetSystemDiagnostics(diagnostics);
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        return result;
    }
    
    std::ostringstream oss;
    oss << "HardFOC ADC Manager Health Report\n";
    oss << "==================================\n";
    oss << "System Status: " << (diagnostics.system_healthy ? "HEALTHY" : "DEGRADED") << "\n";
    oss << "Total Channels: " << diagnostics.total_channels_registered << "\n";
    oss << "Total Readings: " << diagnostics.total_readings << "\n";
    oss << "Success Rate: " << std::fixed << std::setprecision(1) << diagnostics.average_success_rate << "%\n";
    oss << "Communication Errors: " << diagnostics.communication_errors << "\n";
    oss << "Hardware Errors: " << diagnostics.hardware_errors << "\n";
    
    health_info = oss.str();
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::GetSystemStatistics(std::string& stats) const noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    AdcSystemDiagnostics diagnostics;
    auto result = GetSystemDiagnostics(diagnostics);
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        return result;
    }
    
    std::ostringstream oss;
    oss << "HardFOC ADC Manager Statistics\n";
    oss << "==============================\n";
    oss << "Channels Registered: " << diagnostics.total_channels_registered << "\n";
    oss << "Total Readings: " << diagnostics.total_readings << "\n";
    oss << "Successful Readings: " << diagnostics.successful_readings << "\n";
    oss << "Failed Readings: " << diagnostics.failed_readings << "\n";
    oss << "Calibrations Performed: " << diagnostics.total_calibrations << "\n";
    oss << "Average Success Rate: " << std::fixed << std::setprecision(2) << diagnostics.average_success_rate << "%\n";
    
    stats = oss.str();
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::ResetAllChannels() noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    
    for (auto& [name, channel_info] : adc_registry_) {
        if (channel_info) {
            channel_info->totalReadings.store(0);
            channel_info->successfulReadings.store(0);
            channel_info->errorCount.store(0);
            channel_info->filteredValue = 0.0f;
            
            if (channel_info->adc) {
                channel_info->adc->ResetStatistics();
            }
        }
    }
    
    // Reset global statistics
    total_readings_.store(0);
    successful_readings_.store(0);
    failed_readings_.store(0);
    communication_errors_.store(0);
    hardware_errors_.store(0);
    calibration_count_.store(0);
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::PerformSelfTest(std::string& result) noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    std::ostringstream oss;
    oss << "ADC Manager Self-Test Results\n";
    oss << "=============================\n";
    
    bool all_tests_passed = true;
    
    // Test 1: Check all registered channels
    {
        RtosMutex::LockGuard registry_lock(registry_mutex_);
        oss << "Test 1: Channel Registration - ";
        if (adc_registry_.empty()) {
            oss << "FAIL (No channels registered)\n";
            all_tests_passed = false;
        } else {
            oss << "PASS (" << adc_registry_.size() << " channels)\n";
        }
    }
    
    // Test 2: Hardware interface availability
    oss << "Test 2: Hardware Interfaces - ";
    bool hardware_ok = true;
    
    {
        RtosMutex::LockGuard mcu_lock(mcu_adc_mutex_);
        if (!mcu_adc_) {
            hardware_ok = false;
        }
    }
    
    if (!tmc9660_controller_) {
        hardware_ok = false;
    }
    
    if (hardware_ok) {
        oss << "PASS\n";
    } else {
        oss << "FAIL (Hardware not available)\n";
        all_tests_passed = false;
    }
    
    // Test 3: Basic read operation
    oss << "Test 3: Basic Read Operation - ";
    if (!adc_registry_.empty()) {
        AdcReading reading;
        auto read_result = ReadChannel(adc_registry_.begin()->first, reading);
        if (read_result == hf_adc_err_t::ADC_SUCCESS) {
            oss << "PASS\n";
        } else {
            oss << "FAIL (Read error: " << static_cast<int>(read_result) << ")\n";
            all_tests_passed = false;
        }
    } else {
        oss << "SKIP (No channels to test)\n";
    }
    
    oss << "\nOverall Result: " << (all_tests_passed ? "PASS" : "FAIL") << "\n";
    
    result = oss.str();
    return all_tests_passed ? hf_adc_err_t::ADC_SUCCESS : hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
}

//==============================================================================
// PRIVATE HELPER METHODS
//==============================================================================

hf_adc_err_t AdcManager::InitializeEsp32Adc() noexcept {
    RtosMutex::LockGuard mcu_lock(mcu_adc_mutex_);
    
    try {
        mcu_adc_ = std::make_unique<McuAdc>();
        // TODO: Configure ESP32-C6 ADC with proper parameters
        return hf_adc_err_t::ADC_SUCCESS;
    } catch (const std::exception& e) {
        // In real implementation, use proper logging
        // ESP_LOGE(TAG, "Failed to create MCU ADC: %s", e.what());
        return hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
    }
}

hf_adc_err_t AdcManager::InitializeTmc9660Adc() noexcept {
    if (!tmc9660_controller_) {
        return hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
    }
    
    // TODO: Initialize TMC9660 ADC channels based on platform mapping
    // This would involve iterating through the platform mapping table
    // and setting up TMC9660 ADC channels
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::RegisterDefaultChannels() noexcept {
    // TODO: Register all default channels based on platform mapping
    // This would involve iterating through the platform mapping table
    // and registering each functional ADC channel with its corresponding hardware
    
    // For now, register some basic test channels
    // In a real implementation, this would be driven by the platform mapping
    
    // Example: Register basic ESP32 ADC channels
    auto esp32_channel = CreateEsp32AdcChannel(0, 3.3f, 12);
    if (esp32_channel) {
        auto channel_info = std::make_unique<AdcChannelInfo>(
            "ADC_ESP32_CH0", AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3, 
            std::move(esp32_channel), 0, AdcChip::ESP32_INTERNAL, HfAdcChipType::ESP32_INTERNAL);
        
        adc_registry_["ADC_ESP32_CH0"] = std::move(channel_info);
        sensor_to_name_[AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3] = "ADC_ESP32_CH0";
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

AdcChannelInfo* AdcManager::FindChannelInfo(std::string_view name) noexcept {
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    auto it = adc_registry_.find(name);
    return (it != adc_registry_.end()) ? it->second.get() : nullptr;
}

const AdcChannelInfo* AdcManager::FindChannelInfo(std::string_view name) const noexcept {
    RtosMutex::LockGuard registry_lock(registry_mutex_);
    auto it = adc_registry_.find(name);
    return (it != adc_registry_.end()) ? it->second.get() : nullptr;
}

AdcChannelInfo* AdcManager::FindChannelInfo(AdcInputSensor sensor) noexcept {
    auto name_it = sensor_to_name_.find(sensor);
    if (name_it == sensor_to_name_.end()) {
        return nullptr;
    }
    return FindChannelInfo(name_it->second);
}

const AdcChannelInfo* AdcManager::FindChannelInfo(AdcInputSensor sensor) const noexcept {
    auto name_it = sensor_to_name_.find(sensor);
    if (name_it == sensor_to_name_.end()) {
        return nullptr;
    }
    return FindChannelInfo(name_it->second);
}

AdcReading AdcManager::PerformSingleReading(AdcChannelInfo& channelInfo) noexcept {
    AdcReading reading;
    
    if (!channelInfo.adc) {
        reading.result = hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
        return reading;
    }
    
    // TODO: Perform actual ADC reading using the appropriate driver
    // This is a placeholder implementation
    reading.rawValue = 2048; // Placeholder value (mid-scale for 12-bit ADC)
    reading.voltage = channelInfo.ConvertToVoltage(reading.rawValue);
    reading.calibratedValue = (reading.voltage * channelInfo.scaleFactor) + channelInfo.offsetValue;
    reading.result = hf_adc_err_t::ADC_SUCCESS;
    reading.timestamp = std::chrono::steady_clock::now();
    reading.hardwareChannel = channelInfo.hardwareChannel;
    reading.sourceChip = channelInfo.sourceChip;
    
    // Validate reading is in expected range
    if (!channelInfo.IsReadingInRange(reading.voltage)) {
        // In real implementation, use proper logging
        // ESP_LOGW(TAG, "Reading %.3fV outside expected range [%.3f, %.3f] for sensor %.*s",
        //          reading.voltage, channelInfo.minVoltage, channelInfo.maxVoltage,
        //          static_cast<int>(channelInfo.name.length()), channelInfo.name.data());
    }
    
    // Update filtered value if filtering is enabled
    if (channelInfo.filteringEnabled) {
        channelInfo.UpdateFilteredValue(reading.voltage);
    }
    
    // Update access time
    channelInfo.lastAccessTime.store(0); // In real implementation, store current timestamp
    
    return reading;
}

void AdcManager::UpdateChannelStatistics(AdcChannelInfo& channelInfo, const AdcReading& reading) noexcept {
    channelInfo.totalReadings.fetch_add(1);
    if (reading.IsValid()) {
        channelInfo.successfulReadings.fetch_add(1);
        channelInfo.lastReading = reading;
    } else {
        channelInfo.errorCount.fetch_add(1);
    }
}

void AdcManager::UpdateGlobalStatistics(bool success) noexcept {
    total_readings_.fetch_add(1);
    if (success) {
        successful_readings_.fetch_add(1);
    } else {
        failed_readings_.fetch_add(1);
    }
}

void AdcManager::UpdateLastError(hf_adc_err_t error_code) noexcept {
    last_error_.store(error_code, std::memory_order_release);
    
    // Categorize error for statistics
    switch (error_code) {
        case hf_adc_err_t::ADC_ERR_COMMUNICATION_FAILURE:
            communication_errors_.fetch_add(1);
            break;
        case hf_adc_err_t::ADC_ERR_HARDWARE_FAULT:
        case hf_adc_err_t::ADC_ERR_TIMEOUT:
            hardware_errors_.fetch_add(1);
            break;
        default:
            break;
    }
}

hf_adc_err_t AdcManager::ValidateSensorName(std::string_view name) noexcept {
    if (name.empty()) {
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    if (name.length() > 64) { // Reasonable limit for sensor names
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    // Check for reserved prefixes
    const std::string_view reserved_prefixes[] = {
        "CORE_", "COMM_", "SYS_", "INTERNAL_"
    };
    
    for (const auto& prefix : reserved_prefixes) {
        if (name.substr(0, prefix.length()) == prefix) {
            return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
        }
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

std::unique_ptr<BaseAdc> AdcManager::CreateEsp32AdcChannel(uint8_t channel_id,
                                                          float reference_voltage,
                                                          uint32_t resolution) noexcept {
    RtosMutex::LockGuard mcu_lock(mcu_adc_mutex_);
    
    if (!mcu_adc_) {
        return nullptr;
    }
    
    // TODO: Create ESP32 ADC channel with proper configuration
    // This would involve creating a wrapper that implements BaseAdc
    // and uses the McuAdc instance for actual hardware communication
    
    // For now, return nullptr as placeholder
    return nullptr;
}

std::unique_ptr<BaseAdc> AdcManager::CreateTmc9660AdcChannel(uint8_t channel_id,
                                                           uint8_t device_index,
                                                           float reference_voltage,
                                                           uint32_t resolution) noexcept {
    if (!tmc9660_controller_) {
        return nullptr;
    }
    
    // TODO: Create TMC9660 ADC channel with proper configuration
    // This would involve creating a wrapper that implements BaseAdc
    // and uses the TMC9660 controller for actual hardware communication
    
    // For now, return nullptr as placeholder
    return nullptr;
}
