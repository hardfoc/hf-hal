#include "AdcManager.h"
#include "McuAdc.h"
#include "esp_log.h"
#include <algorithm>
#include <numeric>

/**
 * @file AdcManager.cpp
 * @brief Implementation of the consolidated ADC manager for HardFOC.
 * 
 * This file implements the unified ADC management system that replaces
 * the legacy AdcData and AdcHandler classes.
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

static const char* TAG = "AdcManager";

//==============================================================================
// SINGLETON IMPLEMENTATION
//==============================================================================

AdcManager& AdcManager::GetInstance() noexcept {
    static AdcManager instance;
    return instance;
}

//==============================================================================
// LIFECYCLE METHODS
//==============================================================================

Result<void> AdcManager::Initialize(Tmc9660MotorController& tmc9660Controller) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (isInitialized_.load()) {
        ESP_LOGW(TAG, "ADC manager already initialized");
        return Result<void>(ResultCode::ERROR_ALREADY_INITIALIZED);
    }
    
    ESP_LOGI(TAG, "Initializing HardFOC ADC Manager v2.0");
    
    // Store hardware interface reference
    tmc9660Controller_ = &tmc9660Controller;
    
    // Initialize hardware subsystems
    auto esp32Result = InitializeEsp32Adc();
    if (esp32Result.IsError()) {
        ESP_LOGE(TAG, "Failed to initialize ESP32 ADC: %s", 
                 GetResultDescription(esp32Result.GetResult()).data());
        return esp32Result;
    }
    
    auto tmcResult = InitializeTmc9660Adc();
    if (tmcResult.IsError()) {
        ESP_LOGE(TAG, "Failed to initialize TMC9660 ADC: %s", 
                 GetResultDescription(tmcResult.GetResult()).data());
        return tmcResult;
    }
    
    // Register default channel mappings
    auto regResult = RegisterDefaultChannels();
    if (regResult.IsError()) {
        ESP_LOGE(TAG, "Failed to register default channels: %s", 
                 GetResultDescription(regResult.GetResult()).data());
        return regResult;
    }
    
    isInitialized_.store(true);
    ESP_LOGI(TAG, "ADC manager initialized successfully with %zu channels", 
             adcRegistry_.size());
    
    return HARDFOC_SUCCESS();
}

Result<void> AdcManager::Shutdown() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    ESP_LOGI(TAG, "Shutting down ADC manager");
    
    // Clear registrations
    adcRegistry_.clear();
    nameToSensor_.clear();
    
    // Reset hardware interfaces
    mcuAdc_.reset();
    tmc9660Controller_ = nullptr;
    
    isInitialized_.store(false);
    ESP_LOGI(TAG, "ADC manager shutdown complete");
    
    return HARDFOC_SUCCESS();
}

bool AdcManager::IsInitialized() const noexcept {
    return isInitialized_.load();
}

//==============================================================================
// CHANNEL REGISTRATION METHODS
//==============================================================================

Result<void> AdcManager::RegisterChannel(AdcInputSensor sensor, 
                                               float referenceVoltage,
                                               float calibrationScale,
                                               float calibrationOffset) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto it = adcRegistry_.find(sensor);
    if (it != adcRegistry_.end()) {
        ESP_LOGW(TAG, "Channel already registered: %d", static_cast<int>(sensor));
        return Result<void>(ResultCode::ERROR_ADC_ALREADY_REGISTERED);
    }
    
    // TODO: Implement channel registration logic based on sensor mapping
    // This would involve:
    // 1. Looking up the sensor in the platform mapping
    // 2. Creating the appropriate ADC driver instance
    // 3. Configuring the channel parameters
    // 4. Adding to the registry
    
    ESP_LOGD(TAG, "Registered channel %d (ref: %.2fV, scale: %.3f, offset: %.3f)", 
             static_cast<int>(sensor), referenceVoltage, calibrationScale, calibrationOffset);
    
    return HARDFOC_SUCCESS();
}

Result<void> AdcManager::UnregisterChannel(AdcInputSensor sensor) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto it = adcRegistry_.find(sensor);
    if (it == adcRegistry_.end()) {
        return Result<void>(ResultCode::ERROR_ADC_CHANNEL_NOT_FOUND);
    }
    
    adcRegistry_.erase(it);
    ESP_LOGD(TAG, "Unregistered channel %d", static_cast<int>(sensor));
    
    return HARDFOC_SUCCESS();
}

bool AdcManager::IsChannelRegistered(AdcInputSensor sensor) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return adcRegistry_.find(sensor) != adcRegistry_.end();
}

Result<const AdcChannelInfo*> AdcManager::GetChannelInfo(AdcInputSensor sensor) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<const AdcChannelInfo*>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    const auto* info = FindChannelInfo(sensor);
    if (!info) {
        return Result<const AdcChannelInfo*>(ResultCode::ERROR_ADC_CHANNEL_NOT_FOUND);
    }
    
    return Result<const AdcChannelInfo*>(info);
}

//==============================================================================
// BASIC READING OPERATIONS
//==============================================================================

Result<AdcReading> AdcManager::ReadChannel(AdcInputSensor sensor) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        UpdateGlobalStatistics(false);
        return Result<AdcReading>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto* info = FindChannelInfo(sensor);
    if (!info) {
        UpdateGlobalStatistics(false);
        return Result<AdcReading>(ResultCode::ERROR_ADC_CHANNEL_NOT_FOUND);
    }
    
    // Perform the ADC reading
    AdcReading reading = PerformSingleReading(*info);
    
    // Update statistics
    UpdateChannelStatistics(*info, reading);
    UpdateGlobalStatistics(reading.IsValid());
    
    ESP_LOGV(TAG, "Read channel %d: raw=%lu, voltage=%.3f", 
             static_cast<int>(sensor), reading.rawValue, reading.voltage);
    
    return Result<AdcReading>(reading);
}

Result<AdcReading> AdcManager::ReadChannelWithSampling(const AdcSamplingSpec& spec) noexcept {
    if (!spec.IsValid()) {
        return Result<AdcReading>(ResultCode::ERROR_ADC_INVALID_SAMPLES);
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        UpdateGlobalStatistics(false);
        return Result<AdcReading>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    auto* info = FindChannelInfo(spec.sensor);
    if (!info) {
        UpdateGlobalStatistics(false);
        return Result<AdcReading>(ResultCode::ERROR_ADC_CHANNEL_NOT_FOUND);
    }
    
    // Collect multiple samples
    std::vector<AdcReading> samples;
    samples.reserve(spec.numberOfSamples);
    
    for (uint8_t i = 0; i < spec.numberOfSamples; ++i) {
        AdcReading reading = PerformSingleReading(*info);
        if (reading.IsValid()) {
            samples.push_back(reading);
        }
        
        // Inter-sample delay if specified
        if (spec.samplingIntervalMs > 0 && i < spec.numberOfSamples - 1) {
            vTaskDelay(pdMS_TO_TICKS(spec.samplingIntervalMs));
        }
    }
    
    if (samples.empty()) {
        UpdateGlobalStatistics(false);
        return Result<AdcReading>(ResultCode::ERROR_ADC_READ_FAILED);
    }
    
    // Calculate average
    uint64_t totalRaw = 0;
    double totalVoltage = 0.0;
    
    for (const auto& sample : samples) {
        totalRaw += sample.rawValue;
        totalVoltage += sample.voltage;
    }
    
    AdcReading averagedReading;
    averagedReading.rawValue = static_cast<uint32_t>(totalRaw / samples.size());
    averagedReading.voltage = static_cast<float>(totalVoltage / samples.size());
    averagedReading.result = ResultCode::SUCCESS;
    averagedReading.timestamp = std::chrono::steady_clock::now();
    
    // Apply filtering if enabled
    if (spec.enableFiltering) {
        info->UpdateFilteredValue(averagedReading.voltage, spec.filterWeight);
    }
    
    UpdateChannelStatistics(*info, averagedReading);
    UpdateGlobalStatistics(true);
    
    ESP_LOGV(TAG, "Sampled channel %d (%d samples): raw=%lu, voltage=%.3f", 
             static_cast<int>(spec.sensor), samples.size(), 
             averagedReading.rawValue, averagedReading.voltage);
    
    return Result<AdcReading>(averagedReading);
}

Result<float> AdcManager::ReadFilteredValue(AdcInputSensor sensor) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<float>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    const auto* info = FindChannelInfo(sensor);
    if (!info) {
        return Result<float>(ResultCode::ERROR_ADC_CHANNEL_NOT_FOUND);
    }
    
    if (info->totalReadings == 0) {
        // No readings taken yet, perform one
        auto readResult = const_cast<AdcManager*>(this)->ReadChannel(sensor);
        if (readResult.IsError()) {
            return Result<float>(readResult.GetResult());
        }
    }
    
    return Result<float>(info->filteredValue);
}

Result<uint32_t> AdcManager::ReadRawValue(AdcInputSensor sensor) noexcept {
    auto readResult = ReadChannel(sensor);
    if (readResult.IsError()) {
        return Result<uint32_t>(readResult.GetResult());
    }
    
    return Result<uint32_t>(readResult.GetValue().rawValue);
}

//==============================================================================
// SYSTEM INFORMATION METHODS
//==============================================================================

size_t AdcManager::GetRegisteredChannelCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return adcRegistry_.size();
}

std::vector<AdcInputSensor> AdcManager::GetRegisteredSensors() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<AdcInputSensor> sensors;
    sensors.reserve(adcRegistry_.size());
    
    for (const auto& pair : adcRegistry_) {
        sensors.push_back(pair.first);
    }
    
    return sensors;
}

Result<std::string> AdcManager::GetSystemHealth() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<std::string>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    std::string health;
    health += "HardFOC ADC Manager Health Report\n";
    health += "=================================\n";
    health += "Status: " + std::string(isInitialized_.load() ? "Initialized" : "Not Initialized") + "\n";
    health += "Registered Channels: " + std::to_string(adcRegistry_.size()) + "\n";
    health += "Total Readings: " + std::to_string(totalReadings_.load()) + "\n";
    health += "Successful Readings: " + std::to_string(successfulReadings_.load()) + "\n";
    health += "Failed Readings: " + std::to_string(failedReadings_.load()) + "\n";
    health += "Calibrations Performed: " + std::to_string(calibrationCount_.load()) + "\n";
    
    if (totalReadings_.load() > 0) {
        float successRate = (static_cast<float>(successfulReadings_.load()) / 
                            static_cast<float>(totalReadings_.load())) * 100.0f;
        health += "Success Rate: " + std::to_string(successRate) + "%\n";
    }
    
    return Result<std::string>(std::move(health));
}

Result<void> AdcManager::ResetAllChannels() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!isInitialized_.load()) {
        return Result<void>(ResultCode::ERROR_NOT_INITIALIZED);
    }
    
    for (auto& pair : adcRegistry_) {
        auto& info = pair.second;
        info->filteredValue = 0.0f;
        info->totalReadings = 0;
        info->successfulReadings = 0;
        info->lastReading = AdcReading{};
    }
    
    ESP_LOGI(TAG, "Reset all %zu ADC channels", adcRegistry_.size());
    return HARDFOC_SUCCESS();
}

//==============================================================================
// PRIVATE IMPLEMENTATION METHODS
//==============================================================================

Result<void> AdcManager::InitializeEsp32Adc() noexcept {
    ESP_LOGD(TAG, "Initializing ESP32-C6 internal ADC");
    
    try {
        mcuAdc_ = std::make_unique<McuAdc>();
        // TODO: Configure ESP32-C6 ADC
        return HARDFOC_SUCCESS();
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Failed to create MCU ADC: %s", e.what());
        return Result<void>(ResultCode::ERROR_INIT_FAILED);
    }
}

Result<void> AdcManager::InitializeTmc9660Adc() noexcept {
    ESP_LOGD(TAG, "Initializing TMC9660 ADC");
    
    if (!tmc9660Controller_) {
        return Result<void>(ResultCode::ERROR_DEPENDENCY_MISSING);
    }
    
    // TODO: Initialize TMC9660 ADC channels
    return HARDFOC_SUCCESS();
}

Result<void> AdcManager::RegisterDefaultChannels() noexcept {
    ESP_LOGD(TAG, "Registering default channel mappings");
    
    // TODO: Register all default channels based on platform mapping
    // This would involve iterating through the platform mapping table
    // and registering each functional channel with its corresponding hardware
    
    return HARDFOC_SUCCESS();
}

AdcChannelInfo* AdcManager::FindChannelInfo(AdcInputSensor sensor) noexcept {
    auto it = adcRegistry_.find(sensor);
    return (it != adcRegistry_.end()) ? it->second.get() : nullptr;
}

const AdcChannelInfo* AdcManager::FindChannelInfo(AdcInputSensor sensor) const noexcept {
    auto it = adcRegistry_.find(sensor);
    return (it != adcRegistry_.end()) ? it->second.get() : nullptr;
}

AdcReading AdcManager::PerformSingleReading(AdcChannelInfo& channelInfo) noexcept {
    AdcReading reading;
    
    if (!channelInfo.adc) {
        reading.result = ResultCode::ERROR_ADC_CHIP_NOT_FOUND;
        return reading;
    }
    
    // TODO: Perform actual ADC reading using the appropriate driver
    // This is a placeholder implementation
    reading.rawValue = 2048; // Placeholder value
    reading.voltage = channelInfo.ConvertToVoltage(reading.rawValue);
    reading.result = ResultCode::SUCCESS;
    reading.timestamp = std::chrono::steady_clock::now();
    
    // Validate reading is in expected range
    if (!channelInfo.IsReadingInRange(reading.voltage)) {
        ESP_LOGW(TAG, "Reading %.3fV outside expected range [%.3f, %.3f] for sensor %d",
                 reading.voltage, channelInfo.minVoltage, channelInfo.maxVoltage,
                 static_cast<int>(channelInfo.sensor));
    }
    
    return reading;
}

void AdcManager::UpdateChannelStatistics(AdcChannelInfo& channelInfo, 
                                               const AdcReading& reading) noexcept {
    channelInfo.totalReadings++;
    if (reading.IsValid()) {
        channelInfo.successfulReadings++;
        channelInfo.lastReading = reading;
    }
}

void AdcManager::UpdateGlobalStatistics(bool success) noexcept {
    totalReadings_.fetch_add(1);
    if (success) {
        successfulReadings_.fetch_add(1);
    } else {
        failedReadings_.fetch_add(1);
    }
}
