/**
 * @file AdcManager.h
 * @brief Advanced ADC management system for the HardFOC platform.
 * 
 * @details This class provides a comprehensive ADC management system that integrates
 *          with the platform mapping system to automatically manage ADC channels from multiple
 *          hardware sources (ESP32-C6 internal ADC, TMC9660 ADC) based on functional sensor
 *          identifiers and hardware chip mappings.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features:
 * - Platform mapping integration for automatic channel discovery
 * - Multi-chip ADC management (ESP32-C6, TMC9660)
 * - Functional sensor abstraction with hardware-agnostic API
 * - Thread-safe operation with comprehensive error handling
 * - Automatic channel registration based on platform configuration
 * - Advanced diagnostics and health monitoring
 * - Batch operations for performance optimization
 * - Hardware resource validation and conflict detection
 * - String-based sensor identification for extensibility
 * - Smart sensor categorization and validation
 * - Complete BaseAdc function coverage through string-based routing
 * - Advanced sampling and filtering capabilities
 * - Calibration and offset management
 * 
 * Architecture:
 * - Uses string_view for sensor identification (extensible)
 * - Integrates with platform mapping for hardware mapping
 * - Supports all HardwareChip types defined in platform mapping
 * - Provides unified BaseAdc interface for all ADC operations
 * - Handler-based ADC creation for proper ownership
 * - Routes all BaseAdc functions through string-based API
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 * @note All sensor operations use string_view identifiers for maximum flexibility.
 */

#ifndef COMPONENT_HANDLER_ADC_MANAGER_H_
#define COMPONENT_HANDLER_ADC_MANAGER_H_

#include "Result.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseAdc.h"
#include "Tmc9660MotorController.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <chrono>
#include <optional>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class McuAdc;

//==============================================================================
// ADC ERROR HANDLING
//==============================================================================

/**
 * @brief Comprehensive ADC error codes for detailed error reporting.
 */
enum class hf_adc_err_t : uint8_t {
    ADC_SUCCESS = 0,                    ///< Operation successful
    ADC_ERR_NOT_INITIALIZED,            ///< ADC system not initialized
    ADC_ERR_ALREADY_INITIALIZED,        ///< ADC system already initialized
    ADC_ERR_INVALID_PARAMETER,          ///< Invalid parameter provided
    ADC_ERR_SENSOR_NOT_FOUND,           ///< Sensor not registered
    ADC_ERR_SENSOR_ALREADY_REGISTERED,  ///< Sensor already registered
    ADC_ERR_HARDWARE_FAULT,             ///< Hardware communication failure
    ADC_ERR_TIMEOUT,                    ///< Operation timeout
    ADC_ERR_OUT_OF_RANGE,               ///< Reading out of expected range
    ADC_ERR_CALIBRATION_FAILED,         ///< Calibration operation failed
    ADC_ERR_INSUFFICIENT_MEMORY,        ///< Insufficient memory
    ADC_ERR_COMMUNICATION_FAILURE,      ///< I2C/SPI communication error
    ADC_ERR_SAMPLING_ERROR,             ///< Sampling operation error
    ADC_ERR_FILTERING_ERROR,            ///< Filtering operation error
    ADC_ERR_BATCH_OPERATION_FAILED,     ///< Batch operation failed
    ADC_ERR_SYSTEM_OVERLOAD,            ///< System overload detected
    ADC_ERR_UNKNOWN                     ///< Unknown error
};

//==============================================================================
// ADC DATA STRUCTURES
//==============================================================================

/**
 * @brief Structure representing a single ADC reading with comprehensive metadata.
 */
struct AdcReading {
    uint32_t rawValue;                                      ///< Raw ADC count value
    float voltage;                                          ///< Converted voltage value
    float calibratedValue;                                  ///< Calibrated final value
    std::chrono::steady_clock::time_point timestamp;       ///< Reading timestamp
    hf_adc_err_t result;                                    ///< Reading result status
    uint8_t hardwareChannel;                                ///< Hardware channel used
    AdcChip sourceChip;                                     ///< Source ADC chip
    
    /**
     * @brief Default constructor.
     */
    AdcReading() noexcept 
        : rawValue(0), voltage(0.0f), calibratedValue(0.0f), 
          timestamp(std::chrono::steady_clock::now()),
          result(hf_adc_err_t::ADC_ERR_NOT_INITIALIZED), 
          hardwareChannel(0), sourceChip(AdcChip::UNKNOWN) {}
    
    /**
     * @brief Constructor with values.
     */
    AdcReading(uint32_t raw, float volt, float calibrated = 0.0f, 
               hf_adc_err_t res = hf_adc_err_t::ADC_SUCCESS,
               uint8_t channel = 0, AdcChip chip = AdcChip::UNKNOWN) noexcept
        : rawValue(raw), voltage(volt), calibratedValue(calibrated),
          timestamp(std::chrono::steady_clock::now()),
          result(res), hardwareChannel(channel), sourceChip(chip) {}
    
    /**
     * @brief Check if this reading is valid.
     */
    [[nodiscard]] bool IsValid() const noexcept {
        return result == hf_adc_err_t::ADC_SUCCESS;
    }
    
    /**
     * @brief Get age of reading in milliseconds.
     */
    [[nodiscard]] uint32_t GetAgeMs() const noexcept {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp);
        return static_cast<uint32_t>(duration.count());
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
    float maxAllowedVariance;       ///< Maximum allowed variance for sample validation
    
    /**
     * @brief Constructor with default values.
     */
    explicit AdcSamplingSpec(AdcInputSensor s, uint8_t samples = 1, 
                           uint16_t intervalMs = 0, bool filtering = false,
                           float weight = 0.1f, float variance = 0.1f) noexcept
        : sensor(s), numberOfSamples(samples), samplingIntervalMs(intervalMs),
          enableFiltering(filtering), filterWeight(weight), maxAllowedVariance(variance) {
        // Validate parameters
        if (numberOfSamples == 0) numberOfSamples = 1;
        if (filterWeight < 0.0f || filterWeight > 1.0f) filterWeight = 0.1f;
        if (maxAllowedVariance < 0.0f) maxAllowedVariance = 0.1f;
    }
    
    /**
     * @brief Check if this specification is valid.
     */
    [[nodiscard]] bool IsValid() const noexcept {
        return numberOfSamples > 0 && numberOfSamples <= 255 &&
               filterWeight >= 0.0f && filterWeight <= 1.0f &&
               maxAllowedVariance >= 0.0f;
    }
};

/**
 * @brief Structure containing comprehensive ADC channel information with platform mapping.
 */
struct AdcChannelInfo {
    std::string_view name;                      ///< Human-readable name (string_view to static data)
    AdcInputSensor sensor;                      ///< Functional sensor identifier
    std::unique_ptr<BaseAdc> adc;               ///< ADC driver instance (unique ownership)
    uint8_t hardwareChannel;                    ///< Hardware channel number
    AdcChip sourceChip;                         ///< ADC source chip
    HfAdcChipType hardware_chip;                ///< Hardware chip identifier from platform mapping
    bool isRegistered;                          ///< Registration status
    
    // Hardware configuration
    float referenceVoltage;                     ///< Reference voltage for conversion
    uint32_t resolution;                        ///< ADC resolution in bits
    uint32_t maxSampleRate;                     ///< Maximum sampling rate in Hz
    
    // Calibration and conversion parameters
    float scaleFactor;                          ///< Scaling factor for final conversion
    float offsetValue;                          ///< Offset value for calibration
    float minVoltage;                           ///< Minimum expected voltage
    float maxVoltage;                           ///< Maximum expected voltage
    
    // Statistics and filtering
    AdcReading lastReading;                     ///< Last successful reading
    float filteredValue;                        ///< Filtered/averaged value
    std::atomic<uint32_t> totalReadings;        ///< Total number of readings taken
    std::atomic<uint32_t> successfulReadings;   ///< Number of successful readings
    std::atomic<uint32_t> errorCount;           ///< Number of errors encountered
    std::atomic<uint64_t> lastAccessTime;       ///< Timestamp of last access
    
    // Filtering configuration
    bool filteringEnabled;                      ///< Is filtering enabled
    float filterWeight;                         ///< Current filter weight
    
    /**
     * @brief Constructor for AdcChannelInfo.
     */
    AdcChannelInfo(std::string_view n, AdcInputSensor s, std::unique_ptr<BaseAdc> a, 
                   uint8_t ch, AdcChip src, HfAdcChipType hw_chip,
                   float refVolt = 3.3f, uint32_t res = 12) noexcept
        : name(n), sensor(s), adc(std::move(a)), hardwareChannel(ch), sourceChip(src),
          hardware_chip(hw_chip), isRegistered(true), referenceVoltage(refVolt), 
          resolution(res), maxSampleRate(1000), scaleFactor(1.0f), offsetValue(0.0f), 
          minVoltage(0.0f), maxVoltage(refVolt), filteredValue(0.0f), 
          totalReadings(0), successfulReadings(0), errorCount(0), lastAccessTime(0),
          filteringEnabled(false), filterWeight(0.1f) {}
    
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
    void UpdateFilteredValue(float newValue, float weight = 0.0f) noexcept {
        if (weight <= 0.0f) weight = filterWeight;
        if (totalReadings.load() == 0) {
            filteredValue = newValue;
        } else {
            filteredValue = (weight * newValue) + ((1.0f - weight) * filteredValue);
        }
    }
    
    /**
     * @brief Check if reading is within expected range.
     */
    [[nodiscard]] bool IsReadingInRange(float voltage) const noexcept {
        return voltage >= minVoltage && voltage <= maxVoltage;
    }
    
    /**
     * @brief Get success rate as percentage.
     */
    [[nodiscard]] float GetSuccessRate() const noexcept {
        uint32_t total = totalReadings.load();
        if (total == 0) return 0.0f;
        uint32_t successful = successfulReadings.load();
        return (static_cast<float>(successful) / static_cast<float>(total)) * 100.0f;
    }
};

/**
 * @brief Structure for batch ADC operation specifications.
 */
struct AdcBatchOperation {
    std::vector<std::string_view> sensor_names;     ///< Sensor names to operate on
    std::vector<AdcSamplingSpec> sampling_specs;    ///< Individual sampling specifications
    bool use_individual_specs;                      ///< Use individual specs or common settings
    uint8_t common_samples;                         ///< Common number of samples (if not using individual specs)
    uint16_t common_interval_ms;                    ///< Common sampling interval (if not using individual specs)
    
    /**
     * @brief Constructor for simple batch read.
     */
    explicit AdcBatchOperation(std::vector<std::string_view> names) noexcept
        : sensor_names(std::move(names)), use_individual_specs(false),
          common_samples(1), common_interval_ms(0) {}
    
    /**
     * @brief Constructor for advanced batch read with individual specs.
     */
    explicit AdcBatchOperation(std::vector<AdcSamplingSpec> specs) noexcept
        : sampling_specs(std::move(specs)), use_individual_specs(true),
          common_samples(1), common_interval_ms(0) {
        // Extract sensor names from specs
        sensor_names.reserve(sampling_specs.size());
        for (const auto& spec : sampling_specs) {
            // Note: This requires sensor name lookup - will be handled in implementation
        }
    }
};

/**
 * @brief Structure for batch ADC operation results.
 */
struct AdcBatchResult {
    std::vector<std::string_view> sensor_names;         ///< Sensor names read
    std::vector<AdcReading> readings;                   ///< Individual readings
    std::vector<hf_adc_err_t> results;                  ///< Individual results
    hf_adc_err_t overall_result;                        ///< Overall operation result
    std::chrono::steady_clock::time_point timestamp;    ///< Batch timestamp
    uint32_t total_time_ms;                             ///< Total operation time
    
    /**
     * @brief Constructor.
     */
    AdcBatchResult() noexcept 
        : overall_result(hf_adc_err_t::ADC_ERR_NOT_INITIALIZED),
          timestamp(std::chrono::steady_clock::now()), total_time_ms(0) {}
    
    /**
     * @brief Check if all readings were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return overall_result == hf_adc_err_t::ADC_SUCCESS;
    }
    
    /**
     * @brief Get success rate as percentage.
     */
    [[nodiscard]] float GetSuccessRate() const noexcept {
        if (results.empty()) return 0.0f;
        
        size_t successCount = 0;
        for (const auto& result : results) {
            if (result == hf_adc_err_t::ADC_SUCCESS) ++successCount;
        }
        
        return (static_cast<float>(successCount) / static_cast<float>(results.size())) * 100.0f;
    }
};

/**
 * @brief Structure for ADC system diagnostics.
 */
struct AdcSystemDiagnostics {
    bool system_healthy;                                ///< Overall system health
    uint32_t total_channels_registered;                 ///< Total channels registered
    uint32_t channels_by_chip[static_cast<uint8_t>(AdcChip::TMC9660_ADC) + 1]; ///< Channels per chip
    uint32_t total_readings;                            ///< Total readings performed
    uint32_t successful_readings;                       ///< Successful readings
    uint32_t failed_readings;                           ///< Failed readings
    uint32_t communication_errors;                      ///< Communication errors
    uint32_t hardware_errors;                           ///< Hardware errors
    uint32_t calibration_errors;                        ///< Calibration errors
    uint64_t system_uptime_ms;                          ///< System uptime
    hf_adc_err_t last_error;                            ///< Last error encountered
    float average_success_rate;                         ///< Average success rate across all channels
    uint32_t total_calibrations;                        ///< Total calibrations performed
};

//==============================================================================
// MAIN ADC MANAGER CLASS
//==============================================================================

/**
 * @class AdcManager
 * @brief Advanced ADC management system for the HardFOC platform.
 * 
 * This class provides a comprehensive ADC management system that integrates
 * with the platform mapping system to automatically manage ADC channels from multiple
 * hardware sources. It uses string_view identifiers and hardware chip
 * mappings to provide a unified, hardware-agnostic API that routes all
 * BaseAdc functions through string-based sensor identification.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal RtosMutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - Core operations return hf_adc_err_t for detailed error codes
 * - Configuration operations return hf_adc_err_t for detailed error codes
 * - Comprehensive error codes via hf_adc_err_t enum
 * - Detailed error descriptions and diagnostics
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple channels
 * - Advanced filtering and averaging
 * - Lazy initialization of hardware resources
 * 
 * Platform Integration:
 * - Automatic channel discovery via platform mapping
 * - Hardware resource validation
 * - Conflict detection and resolution
 * - Multi-chip coordination
 * - Smart sensor categorization
 * 
 * Function Coverage:
 * - Complete BaseAdc function coverage through string-based routing
 * - All ADC operations available via string_view sensor names
 * - Consistent API design with proper camelCase naming
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
     * @brief Ensure the ADC manager system is initialized.
     * @return hf_adc_err_t::ADC_SUCCESS if initialization successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t EnsureInitialized() noexcept;
    
    /**
     * @brief Initialize the ADC manager system with TMC9660 controller.
     * @param tmc9660Controller Reference to the TMC9660 controller
     * @return hf_adc_err_t::ADC_SUCCESS if initialization successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t Initialize(Tmc9660MotorController& tmc9660Controller) noexcept;
    
    /**
     * @brief Shutdown the ADC manager system.
     * @return hf_adc_err_t::ADC_SUCCESS if shutdown successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t Shutdown() noexcept;
    
    /**
     * @brief Check if the ADC system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics and health information.
     * @param diagnostics Reference to store system diagnostics
     * @return hf_adc_err_t::ADC_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;
    
    //==========================================================================
    // CHANNEL REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register an ADC channel with the system using string_view identifier.
     * @param name Sensor name (must be static string or outlive the manager)
     * @param adc Unique pointer to ADC driver
     * @return hf_adc_err_t::ADC_SUCCESS if successful, error code otherwise
     * @note Sensor names must be static strings (string literals or static arrays)
     * @note Reserved prefixes (CORE_, COMM_, SYS_, INTERNAL_) are not allowed
     */
    [[nodiscard]] hf_adc_err_t RegisterChannel(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept;
    
    /**
     * @brief Register an ADC channel with complete configuration.
     * @param sensor Functional sensor identifier
     * @param name Human-readable name
     * @param referenceVoltage Reference voltage for conversion
     * @param calibrationScale Scaling factor for calibration
     * @param calibrationOffset Offset value for calibration
     * @return hf_adc_err_t::ADC_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t RegisterChannel(AdcInputSensor sensor,
                                               std::string_view name,
                                               float referenceVoltage = 3.3f,
                                               float calibrationScale = 1.0f,
                                               float calibrationOffset = 0.0f) noexcept;
    
    /**
     * @brief Get an ADC channel by name.
     * @param name Sensor name
     * @return Pointer to BaseAdc or nullptr if not found
     */
    [[nodiscard]] BaseAdc* Get(std::string_view name) noexcept;
    
    /**
     * @brief Check if an ADC channel is registered.
     * @param name Sensor name
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;
    
    /**
     * @brief Get count of registered channels.
     * @return Number of registered channels
     */
    [[nodiscard]] size_t Size() const noexcept;
    
    /**
     * @brief Log all registered ADC channels for debugging.
     */
    void LogAllRegisteredChannels() const noexcept;
    
    //==========================================================================
    // BASIC READING OPERATIONS (Complete BaseAdc Coverage)
    //==========================================================================
    
    /**
     * @brief Read a single ADC channel by name.
     * @param name Sensor name
     * @param reading Reference to store the reading
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadChannel(std::string_view name, AdcReading& reading) noexcept;
    
    /**
     * @brief Read a single ADC channel by sensor ID.
     * @param sensor Functional sensor identifier
     * @param reading Reference to store the reading
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadChannel(AdcInputSensor sensor, AdcReading& reading) noexcept;
    
    /**
     * @brief Read multiple samples from a channel and average them.
     * @param name Sensor name
     * @param spec Sampling specification
     * @param reading Reference to store the averaged reading
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadChannelWithSampling(std::string_view name, const AdcSamplingSpec& spec, AdcReading& reading) noexcept;
    
    /**
     * @brief Read the filtered/averaged value for a channel.
     * @param name Sensor name
     * @param value Reference to store the filtered value
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadFilteredValue(std::string_view name, float& value) noexcept;
    
    /**
     * @brief Read raw ADC count value (no conversion).
     * @param name Sensor name
     * @param value Reference to store the raw count
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadRawValue(std::string_view name, uint32_t& value) noexcept;
    
    /**
     * @brief Read voltage value with conversion.
     * @param name Sensor name
     * @param voltage Reference to store the voltage
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadVoltage(std::string_view name, float& voltage) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch read operations on multiple channels.
     * @param operation Batch operation specification
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] AdcBatchResult BatchRead(const AdcBatchOperation& operation) noexcept;
    
    /**
     * @brief Read multiple ADC channels by name.
     * @param sensor_names Vector of sensor names to read
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] AdcBatchResult BatchRead(const std::vector<std::string_view>& sensor_names) noexcept;
    
    /**
     * @brief Read all registered channels.
     * @return Batch operation results with readings from all channels
     */
    [[nodiscard]] AdcBatchResult ReadAllChannels() noexcept;
    
    /**
     * @brief Read multiple channels with individual sampling specifications.
     * @param specs Vector of sampling specifications
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] AdcBatchResult BatchReadWithSampling(const std::vector<AdcSamplingSpec>& specs) noexcept;
    
    //==========================================================================
    // CALIBRATION AND CONFIGURATION
    //==========================================================================
    
    /**
     * @brief Calibrate an ADC channel using known reference points.
     * @param name Sensor name
     * @param referenceVoltage Known reference voltage
     * @param measuredValue Measured ADC value at reference voltage
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t CalibrateChannel(std::string_view name,
                                                float referenceVoltage,
                                                uint32_t measuredValue) noexcept;
    
    /**
     * @brief Set the voltage range for a channel (for validation).
     * @param name Sensor name
     * @param minVoltage Minimum expected voltage
     * @param maxVoltage Maximum expected voltage
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t SetChannelRange(std::string_view name,
                                               float minVoltage,
                                               float maxVoltage) noexcept;
    
    /**
     * @brief Update filter settings for a channel.
     * @param name Sensor name
     * @param enableFiltering Enable/disable filtering
     * @param filterWeight Filter weight (0.0-1.0)
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ConfigureFiltering(std::string_view name,
                                                  bool enableFiltering,
                                                  float filterWeight = 0.1f) noexcept;
    
    /**
     * @brief Set reference voltage for a channel.
     * @param name Sensor name
     * @param referenceVoltage New reference voltage
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t SetReferenceVoltage(std::string_view name,
                                                   float referenceVoltage) noexcept;
    
    //==========================================================================
    // STATISTICS AND DIAGNOSTICS (Complete BaseAdc Coverage)
    //==========================================================================
    
    /**
     * @brief Get channel statistics.
     * @param name Sensor name
     * @param statistics Reference to store channel statistics
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetStatistics(std::string_view name, BaseAdc::AdcStatistics& statistics) const noexcept;
    
    /**
     * @brief Reset channel statistics.
     * @param name Sensor name
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ResetStatistics(std::string_view name) noexcept;
    
    /**
     * @brief Get system health information.
     * @param health_info Reference to store health information
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetSystemHealth(std::string& health_info) const noexcept;
    
    /**
     * @brief Get detailed system statistics.
     * @param stats Reference to store statistics
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetSystemStatistics(std::string& stats) const noexcept;
    
    /**
     * @brief Reset all channel statistics and filters.
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ResetAllChannels() noexcept;
    
    /**
     * @brief Perform system self-test.
     * @param result Reference to store test results
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t PerformSelfTest(std::string& result) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    /**
     * @brief Private method to perform actual initialization.
     * @return hf_adc_err_t initialization result
     */
    [[nodiscard]] hf_adc_err_t Initialize() noexcept;
    
    // ===============================
    // SYSTEM STATE
    // ===============================
    
    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> is_initialized_{false};
    
    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex mutex_;
    
    // ===============================
    // ADC STORAGE
    // ===============================
    
    /**
     * @brief String-based ADC registry with unique_ptr ownership.
     * Uses string_view keys pointing to static strings for embedded efficiency.
     * Protected by RtosMutex for thread-safe access.
     */
    std::unordered_map<std::string_view, std::unique_ptr<AdcChannelInfo>> adc_registry_;
    mutable RtosMutex registry_mutex_;  ///< RtosMutex for registry access
    
    /**
     * @brief Sensor ID to string name mapping for reverse lookup.
     */
    std::unordered_map<AdcInputSensor, std::string_view> sensor_to_name_;
    
    // ===============================
    // HARDWARE INTERFACES
    // ===============================
    
    /**
     * @brief TMC9660 Motor Controller reference for ADC access.
     */
    Tmc9660MotorController* tmc9660_controller_{nullptr};
    
    /**
     * @brief ESP32-C6 internal ADC handler (unique ownership).
     */
    std::unique_ptr<McuAdc> mcu_adc_;
    mutable RtosMutex mcu_adc_mutex_;  ///< Mutex for MCU ADC access
    
    // ===============================
    // SYSTEM STATISTICS
    // ===============================
    
    /**
     * @brief Total readings performed (atomic for thread safety).
     */
    std::atomic<uint64_t> total_readings_{0};
    
    /**
     * @brief Successful readings count (atomic for thread safety).
     */
    std::atomic<uint64_t> successful_readings_{0};
    
    /**
     * @brief Failed readings count (atomic for thread safety).
     */
    std::atomic<uint64_t> failed_readings_{0};
    
    /**
     * @brief Communication errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> communication_errors_{0};
    
    /**
     * @brief Hardware errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> hardware_errors_{0};
    
    /**
     * @brief Calibration operations count (atomic for thread safety).
     */
    std::atomic<uint64_t> calibration_count_{0};
    
    /**
     * @brief System start time for uptime calculations (atomic for thread safety).
     */
    std::atomic<uint64_t> system_start_time_{0};
    
    // ===============================
    // ERROR TRACKING
    // ===============================
    
    /**
     * @brief Thread-safe access to error tracking.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex error_mutex_;
    
    /**
     * @brief Last error encountered (for diagnostics only).
     */
    std::atomic<hf_adc_err_t> last_error_{hf_adc_err_t::ADC_SUCCESS};
    
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================
    
    /**
     * @brief Private constructor for singleton pattern.
     */
    AdcManager() = default;
    
    /**
     * @brief Private destructor.
     */
    ~AdcManager() = default;
    
    // Disable copy and move operations
    AdcManager(const AdcManager&) = delete;
    AdcManager& operator=(const AdcManager&) = delete;
    AdcManager(AdcManager&&) = delete;
    AdcManager& operator=(AdcManager&&) = delete;
    
    /**
     * @brief Initialize ESP32-C6 internal ADC.
     * @return hf_adc_err_t initialization result
     */
    [[nodiscard]] hf_adc_err_t InitializeEsp32Adc() noexcept;
    
    /**
     * @brief Initialize TMC9660 ADC channels.
     * @return hf_adc_err_t initialization result
     */
    [[nodiscard]] hf_adc_err_t InitializeTmc9660Adc() noexcept;
    
    /**
     * @brief Create and register all default ADC channels based on platform mapping.
     * @return hf_adc_err_t initialization result
     */
    [[nodiscard]] hf_adc_err_t RegisterDefaultChannels() noexcept;
    
    /**
     * @brief Find ADC channel info by sensor name.
     * @param name Sensor name
     * @return Pointer to channel info or nullptr if not found
     */
    [[nodiscard]] AdcChannelInfo* FindChannelInfo(std::string_view name) noexcept;
    
    /**
     * @brief Find ADC channel info by sensor name (const version).
     * @param name Sensor name
     * @return Pointer to channel info or nullptr if not found
     */
    [[nodiscard]] const AdcChannelInfo* FindChannelInfo(std::string_view name) const noexcept;
    
    /**
     * @brief Find ADC channel info by sensor ID.
     * @param sensor Sensor identifier
     * @return Pointer to channel info or nullptr if not found
     */
    [[nodiscard]] AdcChannelInfo* FindChannelInfo(AdcInputSensor sensor) noexcept;
    
    /**
     * @brief Find ADC channel info by sensor ID (const version).
     * @param sensor Sensor identifier
     * @return Pointer to channel info or nullptr if not found
     */
    [[nodiscard]] const AdcChannelInfo* FindChannelInfo(AdcInputSensor sensor) const noexcept;
    
    /**
     * @brief Perform a single ADC reading with comprehensive error handling.
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
    
    /**
     * @brief Update last error for diagnostics.
     * @param error_code Error code to record
     */
    void UpdateLastError(hf_adc_err_t error_code) noexcept;
    
    /**
     * @brief Validate sensor name using hf_adc_err_t error codes.
     * @param name Sensor name to validate
     * @return hf_adc_err_t::ADC_SUCCESS if valid, specific error otherwise
     */
    [[nodiscard]] static hf_adc_err_t ValidateSensorName(std::string_view name) noexcept;
    
    //==========================================================================
    // ADC CREATION METHODS (PRIVATE)
    //==========================================================================
    
    /**
     * @brief Create ESP32 ADC channel with proper configuration.
     * @param channel_id Physical channel number
     * @param reference_voltage Reference voltage
     * @param resolution ADC resolution in bits
     * @return Unique pointer to BaseAdc or nullptr if creation failed
     */
    [[nodiscard]] std::unique_ptr<BaseAdc> CreateEsp32AdcChannel(uint8_t channel_id,
                                                                float reference_voltage = 3.3f,
                                                                uint32_t resolution = 12) noexcept;
    
    /**
     * @brief Create TMC9660 ADC channel with proper configuration.
     * @param channel_id Physical channel number
     * @param device_index TMC9660 device index
     * @param reference_voltage Reference voltage
     * @param resolution ADC resolution in bits
     * @return Unique pointer to BaseAdc or nullptr if creation failed
     */
    [[nodiscard]] std::unique_ptr<BaseAdc> CreateTmc9660AdcChannel(uint8_t channel_id,
                                                                  uint8_t device_index = 0,
                                                                  float reference_voltage = 3.3f,
                                                                  uint32_t resolution = 12) noexcept;
};

//==============================================================================
// GLOBAL HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Get the ADC manager instance.
 * @return Reference to the ADC manager instance
 */
[[nodiscard]] inline AdcManager& GetAdcManager() noexcept {
    return AdcManager::GetInstance();
}

#endif // COMPONENT_HANDLER_ADC_MANAGER_H_
