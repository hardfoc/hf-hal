/**
 * @file AdcManager.h
 * @brief Consolidated ADC management for the HardFOC system.
 * 
 * @details This class consolidates the functionality of AdcData and AdcHandler into
 *          a single, unified ADC manager. It provides comprehensive ADC operations
 *          across multiple sources (ESP32-C6 internal ADC, TMC9660 ADC) with thread-safe
 *          operation and modern error handling.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * 
 * Key Features:
 * - Thread-safe operation with mutex protection
 * - Multi-source ADC channel registration and management
 * - Simplified error handling using bool return values
 * - Raw and voltage reading capabilities
 * - Calibration and offset management
 * - Comprehensive health monitoring and diagnostics
 * - Singleton pattern for system-wide access
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 */

#ifndef COMPONENT_HANDLER_ADC_MANAGER_H_
#define COMPONENT_HANDLER_ADC_MANAGER_H_

#include "Result.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseAdc.h"
#include "Tmc9660MotorController.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <chrono>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class McuAdc;

//==============================================================================
// ADC DATA STRUCTURES
//==============================================================================

/**
 * @brief Structure representing a single ADC reading.
 */
struct AdcReading {
    uint32_t rawValue;              ///< Raw ADC count value
    float voltage;                  ///< Converted voltage value
    std::chrono::steady_clock::time_point timestamp;  ///< Reading timestamp
    ResultCode result;           ///< Reading result status
    
    /**
     * @brief Default constructor.
     */
    AdcReading() noexcept 
        : rawValue(0), voltage(0.0f), timestamp(std::chrono::steady_clock::now()),
          result(ResultCode::ERROR_ADC_READ_FAILED) {}
    
    /**
     * @brief Constructor with values.
     */
    AdcReading(uint32_t raw, float volt, ResultCode res = ResultCode::SUCCESS) noexcept
        : rawValue(raw), voltage(volt), timestamp(std::chrono::steady_clock::now()),
          result(res) {}
    
    /**
     * @brief Check if this reading is valid.
     */
    [[nodiscard]] bool IsValid() const noexcept {
        return IsSuccessResult(result);
    }
};

/**
 * @brief Structure for multi-sample ADC reading specifications.
 */
struct AdcSamplingSpec {
    AdcInputSensor sensor;          ///< Sensor/channel identifier
    uint8_t numberOfSamples;        ///< Number of samples to take
    uint16_t samplingIntervalMs;    ///< Interval between samples in ms
    bool enableFiltering;           ///< Enable digital filtering
    float filterWeight;             ///< Filter weight (0.0-1.0) for exponential filter
    
    /**
     * @brief Constructor with default values.
     */
    explicit AdcSamplingSpec(AdcInputSensor s, uint8_t samples = 1, 
                           uint16_t intervalMs = 0, bool filtering = false,
                           float weight = 0.1f) noexcept
        : sensor(s), numberOfSamples(samples), samplingIntervalMs(intervalMs),
          enableFiltering(filtering), filterWeight(weight) {
        // Validate parameters
        if (numberOfSamples == 0) numberOfSamples = 1;
        if (filterWeight < 0.0f || filterWeight > 1.0f) filterWeight = 0.1f;
    }
    
    /**
     * @brief Check if this specification is valid.
     */
    [[nodiscard]] bool IsValid() const noexcept {
        return numberOfSamples > 0 && filterWeight >= 0.0f && filterWeight <= 1.0f;
    }
};

/**
 * @brief Structure containing comprehensive ADC channel information.
 */
struct AdcChannelInfo {
    AdcInputSensor sensor;          ///< Functional sensor identifier
    std::unique_ptr<BaseAdc> adc;   ///< ADC driver instance
    uint8_t hardwareChannel;        ///< Hardware channel number
    std::string_view name;          ///< Human-readable name
    AdcChip source;                 ///< ADC source chip
    bool isRegistered;              ///< Registration status
    float referenceVoltage;         ///< Reference voltage for conversion
    uint32_t resolution;            ///< ADC resolution in bits
    
    // Calibration and conversion parameters
    float scaleFactor;              ///< Scaling factor for final conversion
    float offsetValue;              ///< Offset value for calibration
    float minVoltage;               ///< Minimum expected voltage
    float maxVoltage;               ///< Maximum expected voltage
    
    // Statistics and filtering
    AdcReading lastReading;         ///< Last successful reading
    float filteredValue;            ///< Filtered/averaged value
    uint32_t totalReadings;         ///< Total number of readings taken
    uint32_t successfulReadings;    ///< Number of successful readings
    
    /**
     * @brief Constructor for AdcChannelInfo.
     */
    AdcChannelInfo(AdcInputSensor s, std::unique_ptr<BaseAdc> a, uint8_t ch,
                   std::string_view n, AdcChip src, float refVolt = 3.3f,
                   uint32_t res = 12) noexcept
        : sensor(s), adc(std::move(a)), hardwareChannel(ch), name(n), source(src),
          isRegistered(true), referenceVoltage(refVolt), resolution(res),
          scaleFactor(1.0f), offsetValue(0.0f), minVoltage(0.0f),
          maxVoltage(refVolt), filteredValue(0.0f), totalReadings(0),
          successfulReadings(0) {}
    
    // Disable copy operations due to unique_ptr
    AdcChannelInfo(const AdcChannelInfo&) = delete;
    AdcChannelInfo& operator=(const AdcChannelInfo&) = delete;
    
    // Enable move operations
    AdcChannelInfo(AdcChannelInfo&&) = default;
    AdcChannelInfo& operator=(AdcChannelInfo&&) = default;
    
    /**
     * @brief Convert raw ADC value to voltage.
     */
    [[nodiscard]] float ConvertToVoltage(uint32_t rawValue) const noexcept {
        const float maxCount = static_cast<float>((1U << resolution) - 1);
        const float voltage = (static_cast<float>(rawValue) / maxCount) * referenceVoltage;
        return (voltage * scaleFactor) + offsetValue;
    }
    
    /**
     * @brief Update filtered value with new reading.
     */
    void UpdateFilteredValue(float newValue, float filterWeight = 0.1f) noexcept {
        if (totalReadings == 0) {
            filteredValue = newValue;
        } else {
            filteredValue = (filterWeight * newValue) + ((1.0f - filterWeight) * filteredValue);
        }
    }
    
    /**
     * @brief Check if reading is within expected range.
     */
    [[nodiscard]] bool IsReadingInRange(float voltage) const noexcept {
        return voltage >= minVoltage && voltage <= maxVoltage;
    }
};

/**
 * @brief Structure for batch ADC operation results.
 */
struct AdcBatchResult {
    std::vector<AdcInputSensor> sensors;        ///< Sensors read
    std::vector<AdcReading> readings;           ///< Individual readings
    std::vector<ResultCode> results;         ///< Individual results
    ResultCode overallResult;                ///< Overall operation result
    std::chrono::steady_clock::time_point timestamp;  ///< Batch timestamp
    
    /**
     * @brief Constructor.
     */
    AdcBatchResult() noexcept 
        : overallResult(ResultCode::ERROR_ADC_READ_FAILED),
          timestamp(std::chrono::steady_clock::now()) {}
    
    /**
     * @brief Check if all readings were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return IsSuccessResult(overallResult);
    }
    
    /**
     * @brief Get success rate as percentage.
     */
    [[nodiscard]] float GetSuccessRate() const noexcept {
        if (results.empty()) return 0.0f;
        
        size_t successCount = 0;
        for (const auto& result : results) {
            if (IsSuccessResult(result)) ++successCount;
        }
        
        return (static_cast<float>(successCount) / static_cast<float>(results.size())) * 100.0f;
    }
};

//==============================================================================
// MAIN ADC MANAGER CLASS
//==============================================================================

/**
 * @class AdcManager
 * @brief Consolidated ADC manager for the HardFOC system.
 * 
 * This class provides a unified interface for all ADC operations across
 * multiple hardware sources. It consolidates the functionality of the
 * previous AdcData and AdcHandler classes into a single, modern API.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations for statistics
 * 
 * Error Handling:
 * - All operations return bool with output parameters
 * - Comprehensive error codes via ResultCode enum
 * - Detailed error descriptions available
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple channels
 * - Advanced filtering and averaging
 * - Hardware-specific optimizations
 */
class AdcManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     * @return Reference to the ADC manager instance
     */
    static AdcManager& GetInstance() noexcept;
    
    /**
     * @brief Initialize the ADC manager system.
     * 
     * This method initializes the entire ADC system, including:
     * - ESP32-C6 internal ADC configuration
     * - TMC9660 ADC integration
     * - All channel registrations and calibrations
     * 
     * @param tmc9660Controller Reference to the TMC9660 controller
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool Initialize(Tmc9660MotorController& tmc9660Controller) noexcept;
    
    /**
     * @brief Shutdown the ADC manager system.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool Shutdown() noexcept;
    
    /**
     * @brief Check if the ADC system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    //==========================================================================
    // CHANNEL REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register an ADC channel with the system.
     * @param sensor Functional sensor identifier
     * @param referenceVoltage Reference voltage for conversion
     * @param calibrationScale Scaling factor for calibration
     * @param calibrationOffset Offset value for calibration
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool RegisterChannel(AdcInputSensor sensor,
                                       float referenceVoltage = 3.3f,
                                       float calibrationScale = 1.0f,
                                       float calibrationOffset = 0.0f) noexcept;
    
    /**
     * @brief Unregister an ADC channel from the system.
     * @param sensor Functional sensor identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool UnregisterChannel(AdcInputSensor sensor) noexcept;
    
    /**
     * @brief Check if an ADC channel is registered.
     * @param sensor Functional sensor identifier
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool IsChannelRegistered(AdcInputSensor sensor) const noexcept;
    
    /**
     * @brief Get information about a registered ADC channel.
     * @param sensor Functional sensor identifier
     * @return Result containing channel information or error
     */
    [[nodiscard]] bool GetChannelInfo(AdcInputSensor sensor, const AdcChannelInfo*& info) const noexcept;
    
    //==========================================================================
    // BASIC READING OPERATIONS
    //==========================================================================
    
    /**
     * @brief Read a single ADC channel.
     * @param sensor Functional sensor identifier
     * @return Result containing the reading or error
     */
    [[nodiscard]] bool ReadChannel(AdcInputSensor sensor, AdcReading& reading) noexcept;
    
    /**
     * @brief Read multiple samples from a channel and average them.
     * @param spec Sampling specification
     * @return Result containing the averaged reading or error
     */
    [[nodiscard]] bool ReadChannelWithSampling(const AdcSamplingSpec& spec, AdcReading& reading) noexcept;
    
    /**
     * @brief Read the filtered/averaged value for a channel.
     * @param sensor Functional sensor identifier
     * @return Result containing the filtered value or error
     */
    [[nodiscard]] bool ReadFilteredValue(AdcInputSensor sensor, float& value) noexcept;
    
    /**
     * @brief Read raw ADC count value (no conversion).
     * @param sensor Functional sensor identifier
     * @return Result containing the raw count or error
     */
    [[nodiscard]] bool ReadRawValue(AdcInputSensor sensor, uint32_t& value) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Read multiple ADC channels in a single operation.
     * @param sensors Vector of sensor identifiers to read
     * @return Result containing batch read results
     */
    [[nodiscard]] bool BatchRead(const std::vector<AdcInputSensor>& sensors, AdcBatchResult& result) noexcept;
    
    /**
     * @brief Read multiple channels with individual sampling specifications.
     * @param specs Vector of sampling specifications
     * @return Result containing batch read results
     */
    [[nodiscard]] bool BatchReadWithSampling(const std::vector<AdcSamplingSpec>& specs, AdcBatchResult& result) noexcept;
    
    /**
     * @brief Read all registered channels.
     * @return Result containing readings from all channels
     */
    [[nodiscard]] bool ReadAllChannels(AdcBatchResult& result) noexcept;
    
    //==========================================================================
    // CALIBRATION AND CONFIGURATION
    //==========================================================================
    
    /**
     * @brief Calibrate an ADC channel using known reference points.
     * @param sensor Functional sensor identifier
     * @param referenceVoltage Known reference voltage
     * @param measuredValue Measured ADC value at reference voltage
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool CalibrateChannel(AdcInputSensor sensor,
                                        float referenceVoltage,
                                        uint32_t measuredValue) noexcept;
    
    /**
     * @brief Set the voltage range for a channel (for validation).
     * @param sensor Functional sensor identifier
     * @param minVoltage Minimum expected voltage
     * @param maxVoltage Maximum expected voltage
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool SetChannelRange(AdcInputSensor sensor,
                                       float minVoltage,
                                       float maxVoltage) noexcept;
    
    /**
     * @brief Update filter settings for a channel.
     * @param sensor Functional sensor identifier
     * @param enableFiltering Enable/disable filtering
     * @param filterWeight Filter weight (0.0-1.0)
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool ConfigureFiltering(AdcInputSensor sensor,
                                          bool enableFiltering,
                                          float filterWeight = 0.1f) noexcept;
    
    //==========================================================================
    // SYSTEM INFORMATION AND DIAGNOSTICS
    //==========================================================================
    
    /**
     * @brief Get the total number of registered ADC channels.
     * @return Number of registered channels
     */
    [[nodiscard]] size_t GetRegisteredChannelCount() const noexcept;
    
    /**
     * @brief Get a list of all registered ADC channels.
     * @return Vector of registered sensor identifiers
     */
    [[nodiscard]] std::vector<AdcInputSensor> GetRegisteredSensors() const noexcept;
    
    /**
     * @brief Get system health information.
     * @return Result containing health information or error
     */
    [[nodiscard]] bool GetSystemHealth(std::string& health) const noexcept;
    
    /**
     * @brief Get detailed system statistics.
     * @return Result containing statistics or error
     */
    [[nodiscard]] bool GetSystemStatistics(std::string& stats) const noexcept;
    
    /**
     * @brief Reset all channel statistics and filters.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool ResetAllChannels() noexcept;
    
    /**
     * @brief Perform system self-test.
     * @return Result indicating test success or specific failures
     */
    [[nodiscard]] bool PerformSelfTest(std::string& result) noexcept;
    
    //==========================================================================
    // CONVENIENCE METHODS (NAME-BASED ACCESS)
    //==========================================================================
    
    /**
     * @brief Read an ADC channel using sensor name.
     * @param sensorName Human-readable sensor name
     * @return Result containing the reading or error
     */
    [[nodiscard]] bool ReadChannelByName(std::string_view sensorName, AdcReading& reading) noexcept;
    
    /**
     * @brief Read filtered value using sensor name.
     * @param sensorName Human-readable sensor name
     * @return Result containing the filtered value or error
     */
    [[nodiscard]] bool ReadFilteredValueByName(std::string_view sensorName, float& value) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    mutable std::mutex mutex_;                          ///< Thread safety mutex
    std::atomic<bool> isInitialized_{false};           ///< Initialization status
    
    // ADC storage and management
    std::unordered_map<AdcInputSensor, std::unique_ptr<AdcChannelInfo>> adcRegistry_;
    std::unordered_map<std::string_view, AdcInputSensor> nameToSensor_;
    
    // Hardware interfaces
    Tmc9660MotorController* tmc9660Controller_{nullptr};
    std::unique_ptr<McuAdc> mcuAdc_;
    
    // Statistics and diagnostics
    std::atomic<uint64_t> totalReadings_{0};
    std::atomic<uint64_t> successfulReadings_{0};
    std::atomic<uint64_t> failedReadings_{0};
    std::atomic<uint64_t> calibrationCount_{0};
    
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================
    
    /**
     * @brief Private constructor for singleton pattern.
     */
    AdcManager() = default;
    
    /**
     * @brief Initialize ESP32-C6 internal ADC.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool InitializeEsp32Adc() noexcept;
    
    /**
     * @brief Initialize TMC9660 ADC channels.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool InitializeTmc9660Adc() noexcept;
    
    /**
     * @brief Create and register all default ADC channels.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] bool RegisterDefaultChannels() noexcept;
    
    /**
     * @brief Find ADC channel info by sensor identifier.
     * @param sensor Sensor identifier
     * @return Pointer to channel info or nullptr if not found
     */
    [[nodiscard]] AdcChannelInfo* FindChannelInfo(AdcInputSensor sensor) noexcept;
    
    /**
     * @brief Find ADC channel info by sensor identifier (const version).
     * @param sensor Sensor identifier
     * @return Pointer to channel info or nullptr if not found
     */
    [[nodiscard]] const AdcChannelInfo* FindChannelInfo(AdcInputSensor sensor) const noexcept;
    
    /**
     * @brief Find sensor by name.
     * @param name Sensor name
     * @return Sensor identifier or invalid sensor if not found
     */
    [[nodiscard]] AdcInputSensor FindSensorByName(std::string_view name) const noexcept;
    
    /**
     * @brief Perform a single ADC reading with error handling.
     * @param channelInfo Channel information
     * @return ADC reading result
     */
    [[nodiscard]] AdcReading PerformSingleReading(AdcChannelInfo& channelInfo) noexcept;
    
    /**
     * @brief Update channel statistics after a reading.
     * @param channelInfo Channel information
     * @param reading The reading that was taken
     */
    void UpdateChannelStatistics(AdcChannelInfo& channelInfo, const AdcReading& reading) noexcept;
    
    /**
     * @brief Update global statistics.
     * @param success Whether the operation was successful
     */
    void UpdateGlobalStatistics(bool success) noexcept;
    
    // Disable copy and move for singleton    AdcManager(const AdcManager&) = delete;
    AdcManager& operator=(const AdcManager&) = delete;
    AdcManager(AdcManager&&) = delete;
    AdcManager& operator=(AdcManager&&) = delete;
};

//==============================================================================
// CONVENIENCE FUNCTIONS
//==============================================================================

/**
 * @brief Get the global ADC manager instance.
 * @return Reference to the ADC manager
 */
[[nodiscard]] inline AdcManager& GetAdcManager() noexcept {
    return AdcManager::GetInstance();
}

#endif // COMPONENT_HANDLER_ADC_MANAGER_H_
