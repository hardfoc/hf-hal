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
    : initialized_(false),
      adcTable_{
          // Initialize with dummy entries - will be populated during initialization
      },
      registeredChannelCount_(0)
{
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
    
    // Reset the registration count
    registeredChannelCount_ = 0;
    
    // TODO: Register actual ADC channels here
    // This would be done by the application during startup
    // For now, we just prepare the system for registration
    
    console_info(TAG, "ADC system ready for channel registration");
    return true;
}

bool AdcData::RegisterAdcChannel(AdcInputSensor sensor, BaseAdc& adc, uint8_t channel) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (registeredChannelCount_ >= static_cast<uint8_t>(AdcInputSensor::ADC_INPUT_COUNT)) {
        console_error(TAG, "Cannot register more ADC channels - table full");
        return false;
    }
    
    // Check if sensor is already registered
    for (uint8_t i = 0; i < registeredChannelCount_; ++i) {
        if (adcTable_[i].sensor == sensor) {
            console_warning(TAG, "Sensor %s already registered", 
                          AdcInputSensorToString(sensor).data());
            return false;
        }
    }
    
    // Add the new channel
    adcTable_[registeredChannelCount_] = AdcInfo(sensor, adc, channel);
    registeredChannelCount_++;
    
    console_info(TAG, "Registered ADC channel: %s (channel %d)", 
                 AdcInputSensorToString(sensor).data(), channel);
    
    return true;
}

uint8_t AdcData::GetRegisteredChannelCount() const noexcept {
    return registeredChannelCount_;
}

const AdcInfo* AdcData::GetChannelInfo(AdcInputSensor sensor) const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (uint8_t i = 0; i < registeredChannelCount_; ++i) {
        if (adcTable_[i].sensor == sensor) {
            return &adcTable_[i];
        }
    }
    
    return nullptr;
}

AdcInfo* AdcData::FindAdcInfo(AdcInputSensor sensor) noexcept {
    for (uint8_t i = 0; i < registeredChannelCount_; ++i) {
        if (adcTable_[i].sensor == sensor) {
            return &adcTable_[i];
        }
    }
    return nullptr;
}

bool AdcData::GetCount(AdcInputSensor sensor, uint32_t& count, 
                       uint8_t num_of_samples_to_avg, 
                       uint32_t timeBetweenSamples, 
                       TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(sensor);
    if (!adcInfo) {
        console_error(TAG, "ADC sensor %s not found", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    // Ensure the ADC is initialized
    if (!adcInfo->adc.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    // Read the channel
    BaseAdc::AdcErr result = adcInfo->adc.ReadChannelCount(
        adcInfo->channel, count, num_of_samples_to_avg, 
        ConvertToMilliseconds(timeBetweenSamples, timeUnit));
    
    if (result != BaseAdc::AdcErr::ADC_SUCCESS) {
        console_error(TAG, "Failed to read ADC count for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    return true;
}

bool AdcData::GetVolt(AdcInputSensor sensor, float& adc_volts, 
                      uint8_t num_of_samples_to_avg, 
                      uint32_t timeBetweenSamples, 
                      TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(sensor);
    if (!adcInfo) {
        console_error(TAG, "ADC sensor %s not found", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    // Ensure the ADC is initialized
    if (!adcInfo->adc.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    // Read the channel
    BaseAdc::AdcErr result = adcInfo->adc.ReadChannelV(
        adcInfo->channel, adc_volts, num_of_samples_to_avg, 
        ConvertToMilliseconds(timeBetweenSamples, timeUnit));
    
    if (result != BaseAdc::AdcErr::ADC_SUCCESS) {
        console_error(TAG, "Failed to read ADC voltage for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    return true;
}

bool AdcData::GetCountAndVolt(AdcInputSensor sensor, uint32_t& count, float& adc_volts,
                              uint8_t num_of_samples_to_avg, 
                              uint32_t timeBetweenSamples,
                              TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    AdcInfo* adcInfo = FindAdcInfo(sensor);
    if (!adcInfo) {
        console_error(TAG, "ADC sensor %s not found", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    // Ensure the ADC is initialized
    if (!adcInfo->adc.EnsureInitialized()) {
        console_error(TAG, "Failed to initialize ADC for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    // Read both count and voltage
    BaseAdc::AdcErr result = adcInfo->adc.ReadChannel(
        adcInfo->channel, count, adc_volts, num_of_samples_to_avg, 
        ConvertToMilliseconds(timeBetweenSamples, timeUnit));
    
    if (result != BaseAdc::AdcErr::ADC_SUCCESS) {
        console_error(TAG, "Failed to read ADC count and voltage for sensor %s", 
                     AdcInputSensorToString(sensor).data());
        return false;
    }
    
    return true;
}

bool AdcData::GetMultiCount(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                            uint8_t num_of_readings_to_avg, 
                            uint32_t timeBetweenReadings, 
                            TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    bool overallSuccess = true;
    
    for (uint8_t reading = 0; reading < num_of_readings_to_avg; ++reading) {
        for (auto& spec : sensorReadSpec) {
            uint32_t count;
            bool success = GetCount(spec.sensor, count, spec.numberOfSampleToAvgPerReadings, 0, TimeUnit::TIME_UNIT_MS);
            
            if (success) {
                spec.channelAvgReadingCountStorage += count;
                spec.numberOfSuccessfulReadings++;
            } else {
                overallSuccess = false;
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
    
    return overallSuccess;
}

bool AdcData::GetMultiVolt(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                           uint8_t num_of_readings_to_avg, 
                           uint32_t timeBetweenReadings, 
                           TimeUnit timeUnit) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    bool overallSuccess = true;
    
    for (uint8_t reading = 0; reading < num_of_readings_to_avg; ++reading) {
        for (auto& spec : sensorReadSpec) {
            float voltage;
            bool success = GetVolt(spec.sensor, voltage, spec.numberOfSampleToAvgPerReadings, 0, TimeUnit::TIME_UNIT_MS);
            
            if (success) {
                spec.channelAvgReadingVoltageStorage += voltage;
                spec.numberOfSuccessfulReadings++;
            } else {
                overallSuccess = false;
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
    
    return overallSuccess;
}

bool AdcData::IsCommunicating(AdcChip adcChip) noexcept {
    if (!EnsureInitialized()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if any channels for this chip are responding
    for (uint8_t i = 0; i < registeredChannelCount_; ++i) {
        // This is a simplified check - in a real implementation,
        // you would map sensors to chips and test communication
        uint32_t dummy_count;
        BaseAdc::AdcErr result = adcTable_[i].adc.ReadChannelCount(adcTable_[i].channel, dummy_count, 1, 0);
        if (result == BaseAdc::AdcErr::ADC_SUCCESS) {
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
    BaseAdc::AdcErr result = adcInfo->adc.ReadChannelCount(adcInfo->channel, dummy_count, 1, 0);
    
    return (result == BaseAdc::AdcErr::ADC_SUCCESS);
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
