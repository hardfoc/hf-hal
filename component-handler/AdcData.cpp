#include "AdcData.h"
#include "OsUtility.h"
#include "ConsolePort.h"
#include <algorithm>
#include <vector>

/**
 * @file AdcData.cpp
 * @brief Implementation of the AdcData class.
 * 
 * This file contains the implementation of the comprehensive ADC data management
 * system for the HardFOC project.
 */

static const char* TAG = "AdcData";

// Static member definition
AdcData& AdcData::GetInstance() noexcept {
    static AdcData instance;
    return instance;
}

AdcData::AdcData() noexcept
    : initialized_(false)
{
    // Reserve space for maximum expected channels for performance
    adcTable_.reserve(MAX_CHANNELS);
    console_info(TAG, "AdcData constructor called");
}

AdcData::~AdcData() noexcept {
    console_info(TAG, "AdcData destructor called");
}

bool AdcData::EnsureInitialized() noexcept {
    if (!initialized_.load()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_.load()) {
            console_info(TAG, "Initializing ADC system");
            if (Initialize()) {
                initialized_.store(true);
                console_info(TAG, "ADC system initialized successfully");
            } else {
                console_error(TAG, "Failed to initialize ADC system");
                return false;
            }
        }
    }
    return true;
}

bool AdcData::Initialize() noexcept {
    console_info(TAG, "AdcData::Initialize() - Setting up ADC channels");
    
    // Clear any existing registrations
    adcTable_.clear();
    
    // TODO: Register actual ADC channels here
    // This would be done by the application during startup
    // For now, we just prepare the system for registration
    
    console_info(TAG, "ADC system ready for channel registration");
    return true;
}

HfAdcErr AdcData::RegisterAdcChannel(AdcInputSensor sensor, BaseAdc& adc, uint8_t channel) noexcept {
    if (!EnsureInitialized()) {
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (adcTable_.size() >= MAX_CHANNELS) {
        console_error(TAG, "Cannot register more ADC channels - table full");
        return HfAdcErr::ADC_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Check if sensor is already registered
    auto it = std::find_if(adcTable_.begin(), adcTable_.end(),
                          [sensor](const AdcInfo& info) { 
                              return info.sensor == sensor && info.isValid; 
                          });
    
    if (it != adcTable_.end()) {
        console_warning(TAG, "Sensor %s already registered", 
                      AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_CHANNEL_ALREADY_REGISTERED;
    }
    
    // Validate the ADC and channel
    if (!adc.IsChannelAvailable(channel)) {
        console_error(TAG, "Channel %d not available on ADC for sensor %s", 
                     channel, AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Add the new channel
    try {
        adcTable_.emplace_back(sensor, adc, channel);
        console_info(TAG, "Registered ADC channel: %s (channel %d)", 
                     AdcInputSensorToString(sensor).data(), channel);
        return HfAdcErr::ADC_SUCCESS;
    } catch (const std::exception&) {
        console_error(TAG, "Failed to register ADC channel due to memory allocation");
        return HfAdcErr::ADC_ERR_OUT_OF_MEMORY;
    }
}

HfAdcErr AdcData::UnregisterAdcChannel(AdcInputSensor sensor) noexcept {
    if (!EnsureInitialized()) {
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::find_if(adcTable_.begin(), adcTable_.end(),
                          [sensor](const AdcInfo& info) { 
                              return info.sensor == sensor && info.isValid; 
                          });

    if (it != adcTable_.end()) {
        console_info(TAG, "Unregistered ADC channel: %s", AdcInputSensorToString(sensor).data());
        adcTable_.erase(it);
        return HfAdcErr::ADC_SUCCESS;
    }

    console_warning(TAG, "Sensor %s not found for unregistration", AdcInputSensorToString(sensor).data());
    return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
}

HfAdcErr AdcData::UnregisterAllChannels() noexcept {
    if (!EnsureInitialized()) {
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    
    size_t count = adcTable_.size();
    adcTable_.clear();
    
    console_info(TAG, "Unregistered %zu ADC channels", count);
    return HfAdcErr::ADC_SUCCESS;
}

uint8_t AdcData::GetRegisteredChannelCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<uint8_t>(adcTable_.size());
}

const AdcInfo* AdcData::GetChannelInfo(AdcInputSensor sensor) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = std::find_if(adcTable_.cbegin(), adcTable_.cend(),
                          [sensor](const AdcInfo& info) { 
                              return info.sensor == sensor && info.isValid; 
                          });
    
    return (it != adcTable_.cend()) ? &(*it) : nullptr;
}

bool AdcData::IsSensorRegistered(AdcInputSensor sensor) const noexcept {
    return GetChannelInfo(sensor) != nullptr;
}

std::vector<AdcInputSensor> AdcData::GetRegisteredSensors() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<AdcInputSensor> sensors;
    sensors.reserve(adcTable_.size());
    
    for (const auto& info : adcTable_) {
        if (info.isValid) {
            sensors.push_back(info.sensor);
        }
    }
    
    return sensors;
}

AdcInfo* AdcData::FindAdcInfo(AdcInputSensor sensor) noexcept {
    auto it = std::find_if(adcTable_.begin(), adcTable_.end(),
                          [sensor](const AdcInfo& info) { 
                              return info.sensor == sensor && info.isValid; 
                          });
    
    return (it != adcTable_.end()) ? &(*it) : nullptr;
}

HfAdcErr AdcData::ValidateParameters(AdcInputSensor sensor, uint8_t num_of_samples_to_avg) const noexcept {
    if (!initialized_.load()) {
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    if (num_of_samples_to_avg == 0) {
        return HfAdcErr::ADC_ERR_INVALID_SAMPLE_COUNT;
    }
    
    if (!IsSensorRegistered(sensor)) {
        return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    return HfAdcErr::ADC_SUCCESS;
}

HfAdcErr AdcData::GetCount(AdcInputSensor sensor, uint32_t& count, 
                           uint8_t num_of_samples_to_avg, 
                           uint32_t timeBetweenSamples, 
                           TimeUnit timeUnit) noexcept {
    // Validate parameters first
    HfAdcErr validationResult = ValidateParameters(sensor, num_of_samples_to_avg);
    if (validationResult != HfAdcErr::ADC_SUCCESS) {
        return validationResult;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(sensor);
    if (!adcInfo) {
        console_error(TAG, "ADC sensor %s not found", 
                     AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Ensure the ADC is initialized
    if (!adcInfo->adc.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Read the channel
    HfAdcErr result = adcInfo->adc.ReadChannelCount(
        adcInfo->channel, count, num_of_samples_to_avg, 
        ConvertToMilliseconds(timeBetweenSamples, timeUnit));
    
    if (result != HfAdcErr::ADC_SUCCESS) {
        console_error(TAG, "Failed to read ADC count for sensor %s: %s", 
                     AdcInputSensorToString(sensor).data(),
                     HfAdcErrToString(result).data());
    }
    
    return result;
}

HfAdcErr AdcData::GetVolt(AdcInputSensor sensor, float& adc_volts, 
                          uint8_t num_of_samples_to_avg, 
                          uint32_t timeBetweenSamples, 
                          TimeUnit timeUnit) noexcept {
    // Validate parameters first
    HfAdcErr validationResult = ValidateParameters(sensor, num_of_samples_to_avg);
    if (validationResult != HfAdcErr::ADC_SUCCESS) {
        return validationResult;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(sensor);
    if (!adcInfo) {
        console_error(TAG, "ADC sensor %s not found", 
                     AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Ensure the ADC is initialized
    if (!adcInfo->adc.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Read the channel
    HfAdcErr result = adcInfo->adc.ReadChannelV(
        adcInfo->channel, adc_volts, num_of_samples_to_avg, 
        ConvertToMilliseconds(timeBetweenSamples, timeUnit));
    
    if (result != HfAdcErr::ADC_SUCCESS) {
        console_error(TAG, "Failed to read ADC voltage for sensor %s: %s", 
                     AdcInputSensorToString(sensor).data(),
                     HfAdcErrToString(result).data());
    }
    
    return result;
}

HfAdcErr AdcData::GetCountAndVolt(AdcInputSensor sensor, uint32_t& count, float& adc_volts,
                                  uint8_t num_of_samples_to_avg, 
                                  uint32_t timeBetweenSamples,
                                  TimeUnit timeUnit) noexcept {
    // Validate parameters first
    HfAdcErr validationResult = ValidateParameters(sensor, num_of_samples_to_avg);
    if (validationResult != HfAdcErr::ADC_SUCCESS) {
        return validationResult;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(sensor);
    if (!adcInfo) {
        console_error(TAG, "ADC sensor %s not found", 
                     AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Ensure the ADC is initialized
    if (!adcInfo->adc.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Read both count and voltage
    HfAdcErr result = adcInfo->adc.ReadChannel(
        adcInfo->channel, count, adc_volts, num_of_samples_to_avg, 
        ConvertToMilliseconds(timeBetweenSamples, timeUnit));
    
    if (result != HfAdcErr::ADC_SUCCESS) {
        console_error(TAG, "Failed to read ADC count and voltage for sensor %s: %s", 
                     AdcInputSensorToString(sensor).data(),
                     HfAdcErrToString(result).data());
    }
    
    return result;
}

HfAdcErr AdcData::GetMultiCount(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                                uint8_t num_of_readings_to_avg, 
                                uint32_t timeBetweenReadings, 
                                TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    if (sensorReadSpec.empty()) {
        return HfAdcErr::ADC_ERR_INVALID_PARAMETER;
    }
    
    if (num_of_readings_to_avg == 0) {
        return HfAdcErr::ADC_ERR_INVALID_SAMPLE_COUNT;
    }
    
    HfAdcErr overallResult = HfAdcErr::ADC_SUCCESS;
    
    for (uint8_t reading = 0; reading < num_of_readings_to_avg; ++reading) {
        for (auto& spec : sensorReadSpec) {
            if (!spec.IsValid()) {
                overallResult = HfAdcErr::ADC_ERR_INVALID_PARAMETER;
                continue;
            }
            
            uint32_t count;
            HfAdcErr result = GetCount(spec.sensor, count, spec.numberOfSampleToAvgPerReadings, 0, TimeUnit::TIME_UNIT_MS);
            
            if (result == HfAdcErr::ADC_SUCCESS) {
                spec.channelAvgReadingCountStorage += count;
                spec.numberOfSuccessfulReadings++;
            } else {
                overallResult = result;  // Keep the last error
            }
        }
        
        if (reading < num_of_readings_to_avg - 1 && timeBetweenReadings > 0) {
            PerformDelay(timeBetweenReadings, timeUnit);
        }
    }
    
    // Calculate averages
    for (auto& spec : sensorReadSpec) {
        if (spec.numberOfSuccessfulReadings > 0) {
            spec.channelAvgReadingCountStorage /= spec.numberOfSuccessfulReadings;
        }
    }
    
    return overallResult;
}

HfAdcErr AdcData::GetMultiVolt(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                               uint8_t num_of_readings_to_avg, 
                               uint32_t timeBetweenReadings, 
                               TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return HfAdcErr::ADC_ERR_NOT_INITIALIZED;
    }
    
    if (sensorReadSpec.empty()) {
        return HfAdcErr::ADC_ERR_INVALID_PARAMETER;
    }
    
    if (num_of_readings_to_avg == 0) {
        return HfAdcErr::ADC_ERR_INVALID_SAMPLE_COUNT;
    }
    
    HfAdcErr overallResult = HfAdcErr::ADC_SUCCESS;
    
    for (uint8_t reading = 0; reading < num_of_readings_to_avg; ++reading) {
        for (auto& spec : sensorReadSpec) {
            if (!spec.IsValid()) {
                overallResult = HfAdcErr::ADC_ERR_INVALID_PARAMETER;
                continue;
            }
            
            float voltage;
            HfAdcErr result = GetVolt(spec.sensor, voltage, spec.numberOfSampleToAvgPerReadings, 0, TimeUnit::TIME_UNIT_MS);
            
            if (result == HfAdcErr::ADC_SUCCESS) {
                spec.channelAvgReadingVoltageStorage += voltage;
                spec.numberOfSuccessfulReadings++;
            } else {
                overallResult = result;  // Keep the last error
            }
        }
        
        if (reading < num_of_readings_to_avg - 1 && timeBetweenReadings > 0) {
            PerformDelay(timeBetweenReadings, timeUnit);
        }
    }
    
    // Calculate averages
    for (auto& spec : sensorReadSpec) {
        if (spec.numberOfSuccessfulReadings > 0) {
            spec.channelAvgReadingVoltageStorage /= spec.numberOfSuccessfulReadings;
        }
    }
    
    return overallResult;
}

bool AdcData::IsCommunicating(AdcChip adcChip) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if any channels for this chip are responding
    for (const auto& info : adcTable_) {
        if (!info.isValid) continue;
        
        // This is a simplified check - in a real implementation,
        // you would map sensors to chips and test communication
        uint32_t dummy_count;
        HfAdcErr result = info.adc.ReadChannelCount(info.channel, dummy_count, 1, 0);
        if (result == HfAdcErr::ADC_SUCCESS) {
            return true;
        }
    }
    
    return false;
}

bool AdcData::IsResponding(AdcInputSensor adcInputSensor) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(adcInputSensor);
    if (!adcInfo) {
        return false;
    }
    
    // Try to read a single sample
    uint32_t dummy_count;
    HfAdcErr result = adcInfo->adc.ReadChannelCount(adcInfo->channel, dummy_count, 1, 0);
    
    return (result == HfAdcErr::ADC_SUCCESS);
}

uint32_t AdcData::ConvertToMilliseconds(uint32_t delay, TimeUnit timeUnit) const noexcept {
    switch (timeUnit) {
        case TimeUnit::TIME_UNIT_US:
            return delay / 1000; // Convert microseconds to milliseconds
        case TimeUnit::TIME_UNIT_S:
            return delay * 1000; // Convert seconds to milliseconds
        case TimeUnit::TIME_UNIT_MS:
        default:
            return delay; // Already in milliseconds
    }
}

void AdcData::PerformDelay(uint32_t delay, TimeUnit timeUnit) const noexcept {
    switch (timeUnit) {
        case TimeUnit::TIME_UNIT_US:
            os_delay_usec(delay);
            break;
        case TimeUnit::TIME_UNIT_S:
            os_delay_msec(delay * 1000);
            break;
        case TimeUnit::TIME_UNIT_MS:
        default:
            os_delay_msec(delay);
            break;
    }
}
