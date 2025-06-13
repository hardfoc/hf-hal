

#ifndef HAL_COMPONENT_HANDLERS_ADCDATA_H_
#define HAL_COMPONENT_HANDLERS_ADCDATA_H_

#include <array>
#include <map>
#include <memory>
#include "UTILITIES/common/RtosCompat.h"

#include <HAL/component_handlers/HmiExtAdcDataServicer.h>
#include <HAL/component_handlers/AdcMultiCountReading.h>
#include <HAL/external_chip_drivers/ADS7952.h>
#include <HAL/external_chip_drivers/ADS7952_config_bits.h>
#include <HAL/internal_interface_drivers/BaseAdc.h>
#include <HAL/internal_interface_drivers/Esp32C6Adc.h>

#include <UTILITIES/common/Mutex.h>
#include <UTILITIES/common/CommonIDs.h>
#include <UTILITIES/common/MultiReadings.h>


/**
 * @brief Enumeration for ADC chips.
 */
enum class AdcChip : uint8_t
{
   // S5's INTERNAL ADC0
    ADC_S5_INTERNAL_ADC0_PWR_ELEC_BOARD = 0,

    // S5's INTERNAL ADC1
    ADC_S5_INTERNAL_ADC1_PWR_ELEC_BOARD,

    // PRESSURE SENSE BOARD ADS7952 ADC Chip
    ADC_ADS7952_PRESS_SENS_BOARD,

    // PWR ELEC BOARD ADS7952 ADC CHIP
    ADC_ADS7952_PWR_ELEC_BOARD,

    // EXTERNAL ADC SERVICER FOR THE HMI BOARD's ON BOARD ADC
    ADC_EXT_ADC_SERVICER_HMI_BOARD,
};

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
    {
    	/// No code at this time.
    }

    void Reset() {
		channelAvgReadingCountStorage = 0U;
		channelAvgReadingVoltageStorage = 0.0F;
		numberOfSuccessfulReadings = 0U;
    }
};

class AdcData
{
public:
	 /**
	  * This function creates and initializes the first instance of the AdcData.
	  * @return Pressure sensors
	  */
	static AdcData& GetInstance() noexcept;

	/**
	  *  Destructor is simple, it disables interrupts and closes the port.
	  */
	~AdcData() noexcept;

    //=====================================================================//
    /// SINGLE SENSOR READERS
    //=====================================================================//

    /**
     * Retrieves the last captured data from the specified ADC sensor, in ADC counts.
     *
     * @param sensor The ADC sensor from which data is requested.
     * @param count Reference to a variable where the ADC count will be stored.
     * @param num_of_samples_to_avg [OPTIONAL] Specifies the number of samples to be averaged. Default is 1.
     *                              If set to 0, the function will return the last saved data
     *                              for the channel (no polling will occur).
     *                              If set to a value greater than or equal to 1, the function will
     *                              poll for new data, average it, and return the result.
     * @param timeBetweenSamples    Time between consecutive samples (default is 0).
     * @param timeUnit              Time unit for timeBetweenSamples (default is TIME_UNIT_MS).
     *
     * @return True if the data retrieved is valid, and false if an error occurred during the last sensor data update.
     */
    bool GetCount(AdcInputSensor sensor, uint32_t& count, uint8_t num_of_samples_to_avg = 1, uint32_t timeBetweenSamples = 0, time_unit_t timeUnit = TIME_UNIT_MS) noexcept;

    /**
     * Retrieves the last captured data from the specified ADC sensor, in volts.
     *
     * @param sensor The ADC sensor from which data is requested.
     * @param adc_volts Reference to a variable where the read ADC voltage will be stored.
     * @param num_of_samples_to_avg [OPTIONAL] Specifies the number of samples to be averaged. Default is 1.
     *                              If set to 0, the function will return the last saved data
     *                              for the channel (no polling will occur).
     *                              If set to a value greater than or equal to 1, the function will
     *                              poll for new data, average it, and return the result.
     * @param timeBetweenSamples    Time between consecutive samples (default is 0).
     * @param timeUnit              Time unit for timeBetweenSamples (default is TIME_UNIT_MS).
     *
     * @return True if the data retrieved is valid, and false if an error occurred during the last sensor data update.
     */
    bool GetVolt(AdcInputSensor sensor, float& adc_volts, uint8_t num_of_samples_to_avg = 1, uint32_t timeBetweenSamples = 0, time_unit_t timeUnit = TIME_UNIT_MS) noexcept;

    //=====================================================================//
    /// MULTI-SENSORS READERS
    //=====================================================================//

    /**
     * Retrieves the last captured data from the specified ADC sensors, in ADC counts.
     *
     * @param sensorReadSpec The sensors, and the number of samples wanting to capture per readings,
     * 							combined with the variables that will store the readings.
     * @param num_of_readings_to_avg [OPTIONAL] Specifies the number of readings to be averaged. Default is 1.
     *                              If set to a value greater than or equal to 1, the function will
     *                              poll for new data, average it, and return the result.
     * @param timeBetweenReadings   Time between consecutive readings (default is 0).
     * 									(All specified sensors sampled together, then the "timeBetweenReadings" is waited
     * 									before looping back and re-sampling all sensors)
     * @param timeUnit              Time unit for timeBetweenSamples (default is TIME_UNIT_MS).
     *
     * @return True if the data retrieved is valid, and false if an error occurred during the last sensor data update.
     */
    bool GetMultiCount(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, uint8_t num_of_readings_to_avg = 1, uint32_t timeBetweenReadings = 0, time_unit_t timeUnit = TIME_UNIT_MS) noexcept;

    /**
     * @brief Retrieves the last captured data from the specified ADC sensors, in ADC counts.
     *
     * @tparam IdentifierType The type of the identifier for the ADC sensors.
     * @tparam DataType The type of the data to be retrieved.
     * @tparam MaxIdentifiersCount The maximum number of identifiers that can be handled.
     * @tparam ExtraType An extra type parameter that can be used for additional information.
     *
     * @param adcMultiCountReading A reference to a MultiReadings object that specifies the sensors,
     *                             the number of samples to capture per reading, and the variables
     *                             that will store the readings.
     * @param num_of_readings_to_avg [OPTIONAL] The number of readings to be averaged. Default is 1.
     *                                 If set to a value greater than or equal to 1, the function will
     *                                 poll for new data, average it, and return the result.
     * @param timeBetweenReadings [OPTIONAL] The time to wait between consecutive readings in units
     *                                       specified by `timeUnit`. Default is 0. All specified sensors
     *                                       are sampled together, then the function waits for
     *                                       `timeBetweenReadings` before looping back and re-sampling all sensors.
     * @param timeUnit [OPTIONAL] The unit of time for `timeBetweenReadings`. Default is TIME_UNIT_MS.
     *
     * @return True if the data retrieved is valid, and false if an error occurred during the last sensor data update.
     */
    template <uint8_t MaxIdentifiersCount>
    bool GetMultiCount(AdcMultiCountReading<MaxIdentifiersCount>& adcMultiCountReading, uint8_t num_of_readings_to_avg = 1, uint32_t timeBetweenReadings = 0, time_unit_t timeUnit = TIME_UNIT_MS) noexcept;


    /**
     * Retrieves the last captured data from the specified ADC sensor, in volts.
     *
     * @param sensorReadSpec The sensors, and the number of samples wanting to capture per readings,
     * 							combined with the variables that will store the readings.
     * @param num_of_samples_to_avg [OPTIONAL] Specifies the number of samples to be averaged. Default is 1.
     *                              If set to a value greater than or equal to 1, the function will
     *                              poll for new data, average it, and return the result.
     * @param timeBetweenReadings   Time between consecutive readings (default is 0).
     * 									(All specified sensors sampled together, then the "timeBetweenReadings" is waited
     * 									before looping back and re-sampling all sensors)
     * @param timeUnit              Time unit for timeBetweenSamples (default is TIME_UNIT_MS).
     *
     * @return True if the data retrieved is valid, and false if an error occurred during the last sensor data update.
     */
    bool GetMultiVolt(std::vector<AdcInputSensorReadSpec>& sensorReadSpec, uint8_t num_of_readings_to_avg = 1, uint32_t timeBetweenReadings = 0, time_unit_t timeUnit = TIME_UNIT_MS) noexcept;

    /**
     * @brief Retrieves the last captured voltage data from the specified ADC sensors.
     *
     * @tparam IdentifierType The type of the identifier for the ADC sensors.
     * @tparam DataType The type of the data to be retrieved.
     * @tparam MaxIdentifiersCount The maximum number of identifiers that can be handled.
     * @tparam ExtraType An extra type parameter that can be used for additional information.
     *
     * @param adcMultiVoltReading A reference to a MultiReadings object that specifies the sensors,
     *                            the number of samples to capture per reading, and the variables
     *                            that will store the readings.
     * @param num_of_readings_to_avg [OPTIONAL] The number of readings to be averaged. Default is 1.
     *                                 If set to a value greater than or equal to 1, the function will
     *                                 poll for new data, average it, and return the result.
     * @param timeBetweenReadings [OPTIONAL] The time to wait between consecutive readings in units
     *                                       specified by `timeUnit`. Default is 0. All specified sensors
     *                                       are sampled together, then the function waits for
     *                                       `timeBetweenReadings` before looping back and re-sampling all sensors.
     * @param timeUnit [OPTIONAL] The unit of time for `timeBetweenReadings`. Default is TIME_UNIT_MS.
     *
     * @return True if the data retrieved is valid, and false if an error occurred during the last sensor data update.
     */
    template <uint8_t MaxIdentifiersCount>
    bool GetMultiVolt(AdcMultiCountReading<MaxIdentifiersCount>& adcMultiVoltReading, uint8_t num_of_readings_to_avg = 1, uint32_t timeBetweenReadings = 0, time_unit_t timeUnit = TIME_UNIT_MS) noexcept;

    //=====================================================================//
    /// ADC CHIP AND SENSOR VERIFICATION FUNCTIONS
    //=====================================================================//

    /**
      * @brief  Retrieves communication status of the specified ADC chip
      * @param adcChip The ADC sensor from which data is requested.
      * @return True if the chip if responding to communications, false otherwise.
      */
	bool IsCommunicating( AdcChip adcChip ) noexcept;

    /**
	 * @brief  Retrieves communication status of the specified ADC chip
	 * @param adcChip The ADC sensor from which data is requested.
	 * @return True if the channel is responding and values aren't frozen, false otherwise.
	 */
    bool IsResponding( AdcInputSensor adcInputSensor) noexcept;

    //=====================================================================//
    //=====================================================================//

    /**
	  * @brief  This function checks if the class is initialized;
	  * @return true if initialized, false otherwise
	  */

	inline bool IsInitialized() const noexcept
	{
		return initialized;
	}

	/**
	 * @brief Initialize the  thread.
	 *
	 * This function initializes and creates the thread, setting up
	 * all necessary parameters. It must be called before the thread can be started.
	 *
	 * @return ssp_err_t - Error status, with SSP_SUCCESS indicating successful thread creation.
	 */
	bool EnsureInitialized() noexcept;


	// Constants are public for Unit Testing
	static constexpr float press_sense_ads7952_refp = 2.5F;
	static constexpr float press_sense_ads7952_va_1_and_2_v = 5.0F;
	static constexpr bool press_sense_ads7952_set_two_times_vref = true;
	static constexpr float pwr_elec_ads7952_refp = 2.5F;
	static constexpr float pwr_elec_ads7952_va_1_and_2_v = 5.0F;
	static constexpr bool  pwr_elec_ads7952_set_two_times_vref = true;  // CO2 pressure is 5 volts at full range

private:

	/**
	  * The constructor allocates a new instance of AdcData.
	   * @return n/a
	  */
	AdcData() noexcept;

	/**
	  * Copy constructor is deleted to avoid copying instances.
	  * @return n/a
	  */
	AdcData(const AdcData&) = delete;

	/**
	  *  Assignment operator is deleted to avoid copying instances.
	 */
	AdcData& operator =(const AdcData&) = delete;

	/**
	 * Initializes all ADCs.
	 * @return	true if successful, False if error initializing ADC
	 */
	bool Initialize() noexcept;

	/**
	 * Converts an ADC count to volts with the specified adc parameters.
	 *
	 * @param count	Channel's ADC count
	 * @param v_ref	ADC's reference voltage [Default = 3.3V]
	 * @param bit_resolution ADC's precision [Default = 12 Bit]
	 * @return	returns the converted count in volts.
	 */
	static float AdcCountToVolts(uint32_t count, float v_ref, int bitResolution)
	{
		return (static_cast<float>(count) / (static_cast<float>((1 << bitResolution) - 1))) * v_ref;
	}

        Mutex spiBusMutex; /// SPI bus0 mutex for thread safety between ADCs and EEPROMs

	std::atomic<bool> initialized;               /// ADC configured

	/// PRESSURE SENSOR BOARD ADC SENSOR [ADS7952]
	ADS7952 press_sense_ads7952;


	/// POWER ELECTRONICS BOARD ADC SENSOR [ADS7952]
	ADS7952 pwr_elec_ads7952;

	/// MICROCONTROLLERS INTERNAL ADC [S5]
	RenesasSynergyS5ADC mcu_internal_adc0; /**< Covers P000-P003, P008-009 */
	RenesasSynergyS5ADC mcu_internal_adc1; /**< Covers P004-P007 */
	static constexpr float s5_internal_adc_ext_refp = 3.3F;

	/// EXTERNAL BOARDS ADC CHANNELS
	HmiExtAdcDataServicer& hmi_external_adcs_servicer = HmiExtAdcDataServicer::GetInstance();

	struct AdcInfo
	{
		AdcInputSensor adcInputSensor;
		BaseAdc& adc;
		uint8_t channel_num;
	};
	std::array<AdcInfo, static_cast<uint32_t>(AdcInputSensor::ADC_INPUT_COUNT)> adcTable;

};

template <uint8_t MaxIdentifiersCount>
bool AdcData::GetMultiCount(AdcMultiCountReading<MaxIdentifiersCount>& adcMultiCountReading, uint8_t num_of_readings_to_avg, uint32_t timeBetweenReadings, time_unit_t timeUnit) noexcept
{
    float waitTimeBetweenSamplesMsec;
    TimeUnit unit;

    /// Make sure we have proper waiting time.
    switch(timeUnit) {
        case(TIME_UNIT_US): unit = TimeUnit::MICROSECONDS; break;
        case(TIME_UNIT_MS): unit = TimeUnit::MILLISECONDS; break;
        case(TIME_UNIT_S): unit = TimeUnit::SECONDS; break;
        default: unit = TimeUnit::MILLISECONDS; break;
    }

    ConvertTime((float)timeBetweenReadings, unit, waitTimeBetweenSamplesMsec, TimeUnit::MILLISECONDS);

    /// Cleanup struct in case it was modified before coming into this function
    adcMultiCountReading.Reset();

    uint32_t tempCountReading;
	num_of_readings_to_avg = CONSTRAIN(num_of_readings_to_avg, 1, 255);

    /// Read and average.
    for(int i = 0; i < num_of_readings_to_avg; i++)
    {
        for ( auto& reading  : adcMultiCountReading)
        {
            /// Read Channel
            if( GetCount(reading.identifier, tempCountReading, reading.numOfSamplesPerReading, 0U, timeUnit) )
            {
                reading.Append( tempCountReading );
            }
        }

        /// If a wait time specified between readings, wait
        if(timeBetweenReadings != 0U) {
            TxDelayMsec(static_cast<uint16_t>(waitTimeBetweenSamplesMsec));
        }
    }

    return true;
}



template <uint8_t MaxIdentifiersCount>
bool AdcData::GetMultiVolt(AdcMultiCountReading<MaxIdentifiersCount>& adcMultiVoltReading, uint8_t num_of_readings_to_avg, uint32_t timeBetweenReadings, time_unit_t timeUnit) noexcept
{
    float waitTimeBetweenSamplesMsec;
    TimeUnit unit;

    /// Make sure we have proper waiting time.
    switch(timeUnit) {
        case(TIME_UNIT_US): unit = TimeUnit::MICROSECONDS; break;
        case(TIME_UNIT_MS): unit = TimeUnit::MILLISECONDS; break;
        case(TIME_UNIT_S): unit = TimeUnit::SECONDS; break;
        default: unit = TimeUnit::MILLISECONDS; break;
    }

    ConvertTime((float)timeBetweenReadings, unit, waitTimeBetweenSamplesMsec, TimeUnit::MILLISECONDS);

    /// Cleanup struct in case it was modified before coming into this function
    adcMultiVoltReading.Reset();

    uint32_t tempCountReading;
	num_of_readings_to_avg = CONSTRAIN(num_of_readings_to_avg, 1, 255);

    /// Read and average.
    for(int i = 0; i < num_of_readings_to_avg; i++)
    {
        for ( auto& reading  : adcMultiVoltReading)
        {
            /// Read Channel
            if( GetCount(reading.identifier, tempCountReading, reading.numOfSamplesPerReading, 0U, timeUnit) )
            {
                reading.Append( tempCountReading );
            }
        }

        /// If a wait time specified between readings, wait
        if(timeBetweenReadings != 0U) {
            TxDelayMsec(static_cast<uint16_t>(waitTimeBetweenSamplesMsec));
        }
    }

    return true;
}



#endif /* HAL_COMPONENT_HANDLERS_ADCDATA_H_ */
