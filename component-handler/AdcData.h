#ifndef COMPONENT_HANDLER_ADC_DATA_H_
#define COMPONENT_HANDLER_ADC_DATA_H_

#include <array>
#include <memory>
#include <atomic>
#include <mutex>

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "AdcMultiCountReading.h"
#include "BaseAdc.h"

/**
 * @file AdcData.h
 * @brief Comprehensive ADC data management for the HardFOC system.
 * 
 * This class provides a unified interface for all ADC operations in the system,
 * supporting both internal ADC and external ADC chips.
 */

/**
 * @brief Structure containing ADC channel information.
 */
struct AdcInfo {
    AdcInputSensor sensor;       ///< Sensor identifier
    BaseAdc& adc;                ///< Reference to the ADC driver
    uint8_t channel;             ///< Hardware channel number
    bool isValid;                ///< Validity flag
    
    AdcInfo() = delete;  // Force explicit construction
    
    AdcInfo(AdcInputSensor s, BaseAdc& a, uint8_t ch) noexcept
        : sensor(s), adc(a), channel(ch), isValid(true) {}
    
    // Copy constructor
    AdcInfo(const AdcInfo& other) noexcept
        : sensor(other.sensor), adc(other.adc), channel(other.channel), isValid(other.isValid) {}
    
    // Assignment operator
    AdcInfo& operator=(const AdcInfo& other) noexcept {
        if (this != &other) {
            sensor = other.sensor;
            // Note: Cannot reassign reference, so we assume adc compatibility
            channel = other.channel;
            isValid = other.isValid;
        }
        return *this;
    }
    
    // Move operations (default is fine since we have a reference member)
    AdcInfo(AdcInfo&&) = delete;
    AdcInfo& operator=(AdcInfo&&) = delete;
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
    explicit AdcInputSensorReadSpec(AdcInputSensor sensorArg, uint8_t samplesPerReadings = 1) noexcept
        : sensor(sensorArg),
          numberOfSampleToAvgPerReadings(samplesPerReadings),
          channelAvgReadingCountStorage(0U),
          channelAvgReadingVoltageStorage(0.0F),
          numberOfSuccessfulReadings(0U)
    {
        // Validate input parameters
        if (samplesPerReadings == 0) {
            numberOfSampleToAvgPerReadings = 1;  // Safe default
        }
    }
    
    void Reset() noexcept {
        channelAvgReadingCountStorage = 0U;
        channelAvgReadingVoltageStorage = 0.0F;
        numberOfSuccessfulReadings = 0U;
    }
    
    [[nodiscard]] bool IsValid() const noexcept {
        return numberOfSampleToAvgPerReadings > 0;
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
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr GetCount(AdcInputSensor sensor, uint32_t& count, 
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
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr GetVolt(AdcInputSensor sensor, float& adc_volts, 
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
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr GetCountAndVolt(AdcInputSensor sensor, uint32_t& count, float& adc_volts,
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
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr GetMultiCount(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
                                         uint8_t num_of_readings_to_avg = 1, 
                                         uint32_t timeBetweenReadings = 0, 
                                         TimeUnit timeUnit = TimeUnit::TIME_UNIT_MS) noexcept;
    
    /**
     * @brief Get multiple ADC voltages.
     * @param sensorReadSpec Vector of sensor read specifications.
     * @param num_of_readings_to_avg Number of readings to average (default 1).
     * @param timeBetweenReadings Time between readings in milliseconds (default 0).
     * @param timeUnit Time unit for the delay (default milliseconds).
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr GetMultiVolt(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, 
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
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr RegisterAdcChannel(AdcInputSensor sensor, BaseAdc& adc, uint8_t channel) noexcept;
    
    /**
     * @brief Unregister an ADC channel.
     * @param sensor The sensor identifier to unregister.
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr UnregisterAdcChannel(AdcInputSensor sensor) noexcept;
    
    /**
     * @brief Unregister all ADC channels.
     * @return HfAdcErr error code.
     */
    [[nodiscard]] HfAdcErr UnregisterAllChannels() noexcept;
      /**
     * @brief Get the total number of registered channels.
     * @return Number of registered channels.
     */
    [[nodiscard]] uint8_t GetRegisteredChannelCount() const noexcept;
    
    /**
     * @brief Get channel information by sensor ID.
     * @param sensor The sensor identifier.
     * @return Pointer to AdcInfo or nullptr if not found.
     */
    [[nodiscard]] const AdcInfo* GetChannelInfo(AdcInputSensor sensor) const noexcept;
    
    /**
     * @brief Check if a sensor is registered.
     * @param sensor The sensor identifier to check.
     * @return true if registered, false otherwise.
     */
    [[nodiscard]] bool IsSensorRegistered(AdcInputSensor sensor) const noexcept;
    
    /**
     * @brief Get list of all registered sensors.
     * @return Vector of registered sensor IDs.
     */
    [[nodiscard]] std::vector<AdcInputSensor> GetRegisteredSensors() const noexcept;
    
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
     * @brief Validate input parameters for operations.
     * @param sensor The sensor to validate.
     * @param num_of_samples_to_avg Number of samples to validate.
     * @return HfAdcErr validation result.
     */
    [[nodiscard]] HfAdcErr ValidateParameters(AdcInputSensor sensor, 
                                              uint8_t num_of_samples_to_avg) const noexcept;
    
    /**
     * @brief Convert time unit and delay to milliseconds.
     * @param delay The delay value.
     * @param timeUnit The time unit.
     * @return Delay in milliseconds.
     */
    [[nodiscard]] uint32_t ConvertToMilliseconds(uint32_t delay, TimeUnit timeUnit) const noexcept;
    
    /**
     * @brief Perform delay based on time unit.
     * @param delay The delay value.
     * @param timeUnit The time unit.
     */
    void PerformDelay(uint32_t delay, TimeUnit timeUnit) const noexcept;
    
    // Member variables
    mutable std::mutex mutex_;                      ///< Mutex for thread safety
    std::atomic<bool> initialized_;                 ///< Initialization status
    
    // Use vector instead of fixed array for better flexibility
    std::vector<AdcInfo> adcTable_;                 ///< Registered ADC channels
    static constexpr uint8_t MAX_CHANNELS = static_cast<uint8_t>(AdcInputSensor::ADC_INPUT_COUNT);
};

#endif // COMPONENT_HANDLER_ADC_DATA_H_
