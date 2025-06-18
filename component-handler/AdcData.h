#ifndef COMPONENT_HANDLER_ADC_DATA_H_
#define COMPONENT_HANDLER_ADC_DATA_H_

#include <array>
#include <memory>
#include <atomic>
#include <mutex>

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "AdcMultiCountReading.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/BaseAdc.h"

/**
 * @file AdcData.h
 * @brief Comprehensive ADC data management for the HardFOC system.
 * 
 * This class provides a unified interface for all ADC operations in the system,
 * supporting both internal ESP32-C6 ADC and external ADC chips.
 */

/**
 * @brief Structure containing ADC channel information.
 */
struct AdcInfo {
    AdcInputSensor sensor;        ///< Sensor identifier
    BaseAdc& adc;                ///< Reference to the ADC driver
    uint8_t channel;             ///< Hardware channel number
    
    AdcInfo(AdcInputSensor s, BaseAdc& a, uint8_t ch) 
        : sensor(s), adc(a), channel(ch) {}
};

/**
 * @brief Structure for ADC reading specifications.
 */
struct AdcInputSensorReadSpec {
    AdcInputSensor sensor;
    uint8_t numberOfSampleToAvgPerReadings;
    
    uint32_t channelAvgReadingCountStorage;
    float channelAvgReadingVoltageStorage;
    uint8_t numberOfSuccessfulReadings;
    
    /// Constructor to initialize the struct
    AdcInputSensorReadSpec(AdcInputSensor sensorArg, uint8_t samplesPerReadings)
        : sensor(sensorArg),
          numberOfSampleToAvgPerReadings(samplesPerReadings),
          channelAvgReadingCountStorage(0U),
          channelAvgReadingVoltageStorage(0.0F),
          numberOfSuccessfulReadings(0U)
    {}
    
    void Reset() {
        channelAvgReadingCountStorage = 0U;
        channelAvgReadingVoltageStorage = 0.0F;
        numberOfSuccessfulReadings = 0U;
    }
};

/**
 * @class AdcData
 * @brief Thread-safe singleton class for ADC data management.
 * 
 * This class provides access to all ADC channels in the system through a unified interface.
 * It manages both internal ESP32-C6 ADC and external ADC chips.
 */
class AdcData {
public:
    /**
     * @brief Get the singleton instance.
     * @return Reference to the AdcData instance.
     */
    static AdcData& GetInstance() noexcept;
    
    /**
     * @brief Destructor.
     */
    ~AdcData() noexcept;
    
    // Delete copy constructor and assignment operator
    AdcData(const AdcData&) = delete;
    AdcData& operator=(const AdcData&) = delete;
    
    /**
     * @brief Ensure the ADC system is initialized.
     * @return true if initialization successful, false otherwise.
     */
    bool EnsureInitialized() noexcept;
    
    //=====================================================================//
    /// SINGLE SENSOR READERS
    //=====================================================================//
    
    /**
     * @brief Get ADC count from a sensor.
     * @param sensor The ADC sensor to read from.
     * @param count Reference to store the ADC count.
     * @param num_of_samples_to_avg Number of samples to average (default 1).
     * @param timeBetweenSamples Time between samples in milliseconds (default 0).
     * @param timeUnit Time unit for the delay (default milliseconds).
     * @return true if successful, false otherwise.
     */
    bool GetCount(AdcInputSensor sensor, uint32_t& count, 
                  uint8_t num_of_samples_to_avg = 1, 
                  uint32_t timeBetweenSamples = 0, 
                  TimeUnit timeUnit = TimeUnit::TIME_UNIT_MS) noexcept;
    
    /**
     * @brief Get ADC voltage from a sensor.
     * @param sensor The ADC sensor to read from.
     * @param adc_volts Reference to store the voltage.
     * @param num_of_samples_to_avg Number of samples to average (default 1).
     * @param timeBetweenSamples Time between samples in milliseconds (default 0).
     * @param timeUnit Time unit for the delay (default milliseconds).
     * @return true if successful, false otherwise.
     */
    bool GetVolt(AdcInputSensor sensor, float& adc_volts, 
                 uint8_t num_of_samples_to_avg = 1, 
                 uint32_t timeBetweenSamples = 0, 
                 TimeUnit timeUnit = TimeUnit::TIME_UNIT_MS) noexcept;
    
    /**
     * @brief Get both ADC count and voltage from a sensor.
     * @param sensor The ADC sensor to read from.
     * @param count Reference to store the ADC count.
     * @param adc_volts Reference to store the voltage.
     * @param num_of_samples_to_avg Number of samples to average (default 1).
     * @param timeBetweenSamples Time between samples in milliseconds (default 0).
     * @param timeUnit Time unit for the delay (default milliseconds).
     * @return true if successful, false otherwise.
     */
    bool GetCountAndVolt(AdcInputSensor sensor, uint32_t& count, float& adc_volts,
                         uint8_t num_of_samples_to_avg = 1, 
                         uint32_t timeBetweenSamples = 0,
                         TimeUnit timeUnit = TimeUnit::TIME_UNIT_MS) noexcept;
    
    //=====================================================================//
    /// MULTI SENSOR READERS
    //=====================================================================//
    
    /**
     * @brief Get multiple ADC counts.
     * @param sensorReadSpec Vector of sensor read specifications.
     * @param num_of_readings_to_avg Number of readings to average (default 1).
     * @param timeBetweenReadings Time between readings in milliseconds (default 0).
     * @param timeUnit Time unit for the delay (default milliseconds).
     * @return true if successful, false otherwise.
     */
    bool GetMultiCount(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                       uint8_t num_of_readings_to_avg = 1, 
                       uint32_t timeBetweenReadings = 0, 
                       TimeUnit timeUnit = TimeUnit::TIME_UNIT_MS) noexcept;
    
    /**
     * @brief Get multiple ADC voltages.
     * @param sensorReadSpec Vector of sensor read specifications.
     * @param num_of_readings_to_avg Number of readings to average (default 1).
     * @param timeBetweenReadings Time between readings in milliseconds (default 0).
     * @param timeUnit Time unit for the delay (default milliseconds).
     * @return true if successful, false otherwise.
     */
    bool GetMultiVolt(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                      uint8_t num_of_readings_to_avg = 1, 
                      uint32_t timeBetweenReadings = 0, 
                      TimeUnit timeUnit = TimeUnit::TIME_UNIT_MS) noexcept;
    
    //=====================================================================//
    /// STATUS AND HEALTH CHECKS
    //=====================================================================//
    
    /**
     * @brief Check if an ADC chip is communicating.
     * @param adcChip The ADC chip to check.
     * @return true if communicating, false otherwise.
     */
    bool IsCommunicating(AdcChip adcChip) noexcept;
    
    /**
     * @brief Check if an ADC input sensor is responding.
     * @param adcInputSensor The ADC input sensor to check.
     * @return true if responding, false otherwise.
     */
    bool IsResponding(AdcInputSensor adcInputSensor) noexcept;
    
    /**
     * @brief Register an ADC channel.
     * @param sensor The sensor identifier.
     * @param adc Reference to the ADC driver.
     * @param channel Hardware channel number.
     * @return true if registered successfully, false otherwise.
     */
    bool RegisterAdcChannel(AdcInputSensor sensor, BaseAdc& adc, uint8_t channel) noexcept;
    
    /**
     * @brief Get the total number of registered channels.
     * @return Number of registered channels.
     */
    uint8_t GetRegisteredChannelCount() const noexcept;
    
    /**
     * @brief Get channel information by sensor ID.
     * @param sensor The sensor identifier.
     * @return Pointer to AdcInfo or nullptr if not found.
     */
    const AdcInfo* GetChannelInfo(AdcInputSensor sensor) const noexcept;
    
private:
    /**
     * @brief Private constructor for singleton.
     */
    AdcData() noexcept;
    
    /**
     * @brief Initialize the ADC system.
     * @return true if successful, false otherwise.
     */
    bool Initialize() noexcept;
    
    /**
     * @brief Find ADC info for a sensor.
     * @param sensor The sensor to find.
     * @return Pointer to AdcInfo or nullptr if not found.
     */
    AdcInfo* FindAdcInfo(AdcInputSensor sensor) noexcept;
    
    /**
     * @brief Convert time unit and delay to milliseconds.
     * @param delay The delay value.
     * @param timeUnit The time unit.
     * @return Delay in milliseconds.
     */
    uint32_t ConvertToMilliseconds(uint32_t delay, TimeUnit timeUnit) const noexcept;
    
    /**
     * @brief Perform delay based on time unit.
     * @param delay The delay value.
     * @param timeUnit The time unit.
     */
    void PerformDelay(uint32_t delay, TimeUnit timeUnit) const noexcept;
    
    // Member variables
    mutable std::mutex mutex_;                      ///< Mutex for thread safety
    std::atomic<bool> initialized_;                 ///< Initialization status
    
    std::array<AdcInfo, static_cast<uint32_t>(AdcInputSensor::ADC_INPUT_COUNT)> adcTable_;
    uint8_t registeredChannelCount_;                ///< Number of registered channels
    
    // Constants for different ADC references
    static constexpr float esp32_internal_adc_vref_ = 3.3F;
    static constexpr float external_adc_vref_ = 2.5F;
};

#endif // COMPONENT_HANDLER_ADC_DATA_H_
