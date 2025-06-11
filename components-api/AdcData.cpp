

#include "stdbool.h"
#include <array>

#include <driver/spi_master.h>
#include <driver/adc.h>

#include <HAL/component_handlers/AdcData.h>
#include <HAL/component_handlers/ConsolePort.h>
#include <HAL/external_chip_drivers/ADS7952.h>
#include <HAL/internal_interface_drivers/BaseAdc.h>
#include <HAL/internal_interface_drivers/Esp32C6Adc.h>


#include "UTILITIES/common/TxUtility.h"
#include "UTILITIES/common/MutexGuard.h"
#include "UTILITIES/common/ThingsToString.h"

//==============================================================//
// VERBOSE??
//==============================================================//
static bool verbose = false;

//==============================================================//
// ADC-DATA CLASS
//==============================================================//

AdcData& AdcData::GetInstance() noexcept
{
	static AdcData adc_data_instance;
    return adc_data_instance;
}

AdcData::AdcData() noexcept:
    spiBusMutex("AdcSpiMutex"),
    initialized( false ),
    press_sense_ads7952(nullptr, press_sense_ads7952_refp, press_sense_ads7952_va_1_and_2_v, press_sense_ads7952_set_two_times_vref),
    pwr_elec_ads7952(nullptr, pwr_elec_ads7952_refp, pwr_elec_ads7952_va_1_and_2_v, pwr_elec_ads7952_set_two_times_vref),
    mcu_internal_adc0(ADC_UNIT_1, s5_internal_adc_ext_refp),
    mcu_internal_adc1(ADC_UNIT_1, s5_internal_adc_ext_refp),
	adcTable {{
				{ AdcInputSensor::ADC_PRESSURE_SENSE_DPS_0p18PSI, 				press_sense_ads7952,  0},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_DPS_2p32PSI, 				press_sense_ads7952,  1},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_PMS_5PSI, 					press_sense_ads7952,  2},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_PLS_15PSI, 				press_sense_ads7952,  3},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_PLS_150PSI, 				press_sense_ads7952,  4},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_RLS_30PSI,					press_sense_ads7952,  5},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_PWS_2_1PSI,				press_sense_ads7952,  6},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_PWS_1_1PSI,				press_sense_ads7952,  7},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_AMBIENT_TEMP_SENSE,		press_sense_ads7952,  8},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_EXTERNAL_INPUT_1,			press_sense_ads7952,  9},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_EXTERNAL_INPUT_2,			press_sense_ads7952, 10},
				{ AdcInputSensor::ADC_PRESSURE_SENSE_ADC_2p5V_VREF,				press_sense_ads7952, 11},

				{ AdcInputSensor::ADC_PWR_ELEC_THERMISTOR_HEATER,				pwr_elec_ads7952,     0},
				{ AdcInputSensor::ADC_PWR_ELEC_FLUID_SENSE_1,					pwr_elec_ads7952,     1},
				{ AdcInputSensor::ADC_PWR_ELEC_FLUID_SENSE_2,					pwr_elec_ads7952,     2},
				{ AdcInputSensor::ADC_PWR_ELEC_PRESS_SENSE_CO2_SUPPLY,			pwr_elec_ads7952,     3},
				{ AdcInputSensor::ADC_PWR_ELEC_AMBIENT_TEMP_SENSE,				pwr_elec_ads7952,     4},
				{ AdcInputSensor::ADC_PWR_ELEC_CDX_CURR_SENSE,					pwr_elec_ads7952,     5},
				{ AdcInputSensor::ADC_PWR_ELEC_ORV_CURR_SENSE,					pwr_elec_ads7952,     6},
				{ AdcInputSensor::ADC_PWR_ELEC_FAN_1_CURR_SENSE,				pwr_elec_ads7952,     7},
				{ AdcInputSensor::ADC_PWR_ELEC_FAN_2_CURR_SENSE,				pwr_elec_ads7952,     8},
				{ AdcInputSensor::ADC_PWR_ELEC_HEATER_CURR_SENSE,				pwr_elec_ads7952,     9},
				{ AdcInputSensor::ADC_PWR_ELEC_S5_A3V3_VREF,					pwr_elec_ads7952,    10},
				{ AdcInputSensor::ADC_PWR_ELEC_S5_CURR_SENSE_ADC_VREF,			pwr_elec_ads7952,    11},

				{ AdcInputSensor::ADC_INTERNAL_PRESS_SENSE_BOARD_EXT_ADC_VREF,	mcu_internal_adc0,     3},
				{ AdcInputSensor::ADC_INTERNAL_PWR_ELEC_BOARD_EXT_ADC_VREF,		mcu_internal_adc1,     2},
				{ AdcInputSensor::ADC_INTERNAL_S5_INT_ADC_EXT_VREF,				mcu_internal_adc1,     0},

				{ AdcInputSensor::ADC_HMI_ADC_VREF,								hmi_external_adcs_servicer,     0},
				{ AdcInputSensor::ADC_HMI_TEMPERATURE_SENSOR,					hmi_external_adcs_servicer,     1},
				{ AdcInputSensor::ADC_HMI_SPEAKER_CURR_SENSE,					hmi_external_adcs_servicer,     2},
				{ AdcInputSensor::ADC_HMI_EXTERNAL_INPUT_1,						hmi_external_adcs_servicer,     3},
				{ AdcInputSensor::ADC_HMI_EXTERNAL_INPUT_2,						hmi_external_adcs_servicer,     4},
				{ AdcInputSensor::ADC_HMI_EXTERNAL_INPUT_3,						hmi_external_adcs_servicer,     5},
				{ AdcInputSensor::ADC_HMI_EXTERNAL_INPUT_4,						hmi_external_adcs_servicer,     6},
				{ AdcInputSensor::ADC_HMI_EXTERNAL_INPUT_5,						hmi_external_adcs_servicer,     7}
			}}
{
	/// No code at this time
}

AdcData::~AdcData() noexcept{
	if(initialized)
	{
	// No code at this time
	}
}

/**
   * @brief Initialize the ADC
   *
   * This function creates the mutex the first time it is called and and initializes the ADC interface.
   * all necessary parameters. It must be called before the thread can be started.
   *
   * @return ssp_err_t - Error status, with SSP_SUCCESS indicating successful thread creation.
   */
bool AdcData::EnsureInitialized() noexcept
{
	if (!initialized)   // atomic variable..  no need to protect via mutex
	{
		 __disable_irq();

		initialized = Initialize();

		 __enable_irq();
	}

	return initialized;
}

/// Public functions ------------mutex protected------------------------------------------------------------------


/**
 * Initializes all ADCs.
 * @return	true if successful, False if error initializing ADC
 */
bool AdcData::Initialize() noexcept
{
	MutexGuard guard(spiBusMutex);

	bool adcsInitialized = false;

	/// INITIALIZE S5's INTERNAL ADCs 0 ON PWR ELEC BOARD.
	bool pwr_elec_s5_adc0_initialized = mcu_internal_adc0.EnsureInitialized();
	if(pwr_elec_s5_adc0_initialized == false)
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Failed to initialize S5's INTERNAL ADC 0."); }
		/// TODO: Error Handling
	}
	else
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Successfully initialized S5's INTERNAL ADC 0."); }
	}

	/// INITIALIZE S5's INTERNAL ADCs 1 ON PWR ELEC BOARD.
	bool pwr_elec_s5_adc1_initialized = mcu_internal_adc1.EnsureInitialized();
	if(pwr_elec_s5_adc1_initialized == false)
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Failed to initialize S5's INTERNAL ADC 1."); }
		/// TODO: Error Handling
	}
	else
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Successfully initialized S5's INTERNAL ADC 1."); }
	}

	/// INITIALIZE ADS7952 ON PRESS SENSE BOARD
	bool press_sense_ads7952_initialized = press_sense_ads7952.EnsureInitialized();
	if(press_sense_ads7952_initialized == false)
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Failed to initialize ADS7952 ADC Sensor on pressure sensor board."); }
		/// TODO: Error Handling
	}
	else
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Successfully initialized the ADS7952 ADC Sensor on pressure sensor board."); }
	}

	/// INITIALIZE ADS7952 ON PWR ELEC BOARD
	bool pwr_elec_ads7952_initialized = pwr_elec_ads7952.EnsureInitialized();
	if(pwr_elec_ads7952_initialized == false)
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Failed to initialize ADS7952 ADC Sensor on power electronics board."); }
		/// TODO: Error Handling
	}
	else
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Successfully initialized the ADS7952 ADC Sensor on power electronics board."); }
	}

	/// INITIALIZE HMI BOARD's ADC VALUE SERVICER SINGLETON CLASS.
	bool hmi_external_adcs_servicer_initialized = hmi_external_adcs_servicer.EnsureInitialized();
	if(hmi_external_adcs_servicer_initialized == false)
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Failed to initialize HMI External ADCs Servicer Singleton Class."); }
		/// TODO: Error Handling
	}
	else
	{
		if(verbose) { ConsolePort::GetInstance().Write( "AdcData::Initialize() - Successfully initialized the HMI External ADCs Servicer Singleton Class."); }
	}


	/// RETURN OUTCOME, WILL RETURN FALSE IF ANY ONE OF THEM FAILED INITIALIZATION
	adcsInitialized = (pwr_elec_s5_adc0_initialized &&
						pwr_elec_s5_adc1_initialized &&
						press_sense_ads7952_initialized &&
						pwr_elec_ads7952_initialized &&
						hmi_external_adcs_servicer_initialized);

	return adcsInitialized;
}

/**
 * Updates the specific ADC chip channels count value and automatically stores the timestamp at
 * which it was stored.
 *
 * @param sensor	ADC sensor to update ADC count to
 * @return	True if data polling and saving was successful, and False if there was an issue.
 */

bool AdcData::GetCount(AdcInputSensor sensor, uint32_t& count, uint8_t num_of_samples_to_avg, uint32_t timeBetweenSamples, time_unit_t timeUnit) noexcept
{
    if (EnsureInitialized() )
	{
    	BaseAdc::AdcErr err = BaseAdc::AdcErr::ADC_SUCCESS;
    	uint8_t channelNumber = 0;
    	{
			MutexGuard guard(spiBusMutex);

			auto FindPredicate = [&]( const AdcInfo& adcInfo ) { return ( adcInfo.adcInputSensor == sensor); };

			auto iterator = std::find_if( adcTable.begin(), adcTable.end(), FindPredicate );
			if( iterator )
			{
				float adc_v_reading = 0.0F;
				uint32_t adc_count_reading = 0;

				err = iterator->adc.ReadChannel(iterator->channel_num, adc_count_reading, adc_v_reading, num_of_samples_to_avg, timeBetweenSamples, timeUnit);
				if (err == BaseAdc::AdcErr::ADC_SUCCESS)
				{
					count = adc_count_reading;
					return true;
				}
				else
				{
					channelNumber = iterator->channel_num;
				}
			}

    	}
		if (err != BaseAdc::AdcErr::ADC_SUCCESS)
		{
			ConsolePort::Write("AdcData::GetCount(%s) - ReadChannel(%u) failed, reason: %s (%u).",
					AdcInputSensorToString(sensor), channelNumber, BaseAdc::AdcErrToString(err), err );
		}
	}
    return false;
}

bool AdcData::GetMultiCount(std::vector<AdcInputSensorReadSpec>& sensorsReadSpec, uint8_t num_of_readings_to_avg, uint32_t timeBetweenReadings, time_unit_t timeUnit) noexcept
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
	for (auto& sensorSpec : sensorsReadSpec) {
		sensorSpec.Reset();
	}

	uint32_t tempCountReading;
	num_of_readings_to_avg = CONSTRAIN(num_of_readings_to_avg, 1, 255);

	/// Read and average.
	for(int i = 0; i < num_of_readings_to_avg; i++) {
		for (auto& sensorSpec : sensorsReadSpec) {
			/// Read Channel
			bool status = GetCount(sensorSpec.sensor, tempCountReading, (sensorSpec.numberOfSampleToAvgPerReadings), 0U, timeUnit);

			/// If a successful reading
			if (status) {
		        /// Add to the number of successful readings
		        (sensorSpec.numberOfSuccessfulReadings)++;

		        /// Update the average count and voltage readings
		        (sensorSpec.channelAvgReadingCountStorage) += static_cast<uint32_t>(static_cast<float>((tempCountReading - (sensorSpec.channelAvgReadingCountStorage))) / static_cast<float>(sensorSpec.numberOfSuccessfulReadings));
			}
		}

		/// If a wait time specified between readings, wait
		if(timeBetweenReadings != 0U) {
			TxDelayMsec(static_cast<uint16_t>(waitTimeBetweenSamplesMsec));
		}
    }

    return true;
}


bool AdcData::GetVolt(AdcInputSensor sensor, float& adc_volts, uint8_t num_of_samples_to_avg, uint32_t timeBetweenSamples, time_unit_t timeUnit) noexcept
{
	if (EnsureInitialized() )
    {
		MutexGuard guard(spiBusMutex);

		auto FindPredicate = [&]( const AdcInfo& adcInfo ) { return ( adcInfo.adcInputSensor == sensor); };

		auto iterator = std::find_if( adcTable.begin(), adcTable.end(), FindPredicate );
		if( iterator )
		{

			float adc_v_reading = 0;
			uint32_t adc_count_reading = 0;

			// ReadChannel and check for errors
			BaseAdc::AdcErr err = iterator->adc.ReadChannel(iterator->channel_num, adc_count_reading, adc_v_reading, num_of_samples_to_avg, timeBetweenSamples, timeUnit);
			if (err == BaseAdc::AdcErr::ADC_SUCCESS)
			{

			//	if( sensor == AdcInputSensor::ADC_PWR_ELEC_THERMISTOR_HEATER)
			//	{
			//		ConsolePort::Write ("AdcData::GetVolt(%s) - Reading volts %.2f  counts: %u.",
			//						AdcInputSensorToString(sensor), adc_v_reading, adc_count_reading );
			//	}
				adc_volts = adc_v_reading;
				return true;
			}
			else
			{
				ConsolePort::Write ("AdcData::GetVolt(%s) - ReadChannel(%u) failed, reason: %s (%u).",
					AdcInputSensorToString(sensor), iterator->channel_num, BaseAdc::AdcErrToString(err), err );
			}
		}
    }
	return false;

}

bool AdcData::GetMultiVolt(std::vector<AdcInputSensorReadSpec>& sensorsReadSpec, uint8_t num_of_readings_to_avg, uint32_t timeBetweenReadings, time_unit_t timeUnit) noexcept
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
	for (auto& sensorSpec : sensorsReadSpec) {
		sensorSpec.Reset();
	}

	float tempVoltageReading;
	num_of_readings_to_avg = CONSTRAIN(num_of_readings_to_avg, 1, 255);

	/// Read and average.
	for(int i = 0; i < num_of_readings_to_avg; i++) {
		for (auto& sensorSpec : sensorsReadSpec) {
			/// Read Channel
			bool status = GetVolt(sensorSpec.sensor, tempVoltageReading, (sensorSpec.numberOfSampleToAvgPerReadings), 0U, timeUnit);

			/// If a successful reading
			if (status) {
		        /// Add to the number of successful readings
		        (sensorSpec.numberOfSuccessfulReadings)++;

		        /// Update the average voltage readings
		        (sensorSpec.channelAvgReadingVoltageStorage) += (tempVoltageReading - (sensorSpec.channelAvgReadingVoltageStorage)) / sensorSpec.numberOfSuccessfulReadings;
			}
		}

		/// If a wait time specified between readings, wait
		if(timeBetweenReadings != 0U) {
			TxDelayMsec(static_cast<uint16_t>(waitTimeBetweenSamplesMsec));
		}
    }

    return true;
}

bool AdcData::IsCommunicating(AdcChip adcChip) noexcept
{
    switch (adcChip)
    {
        case AdcChip::ADC_S5_INTERNAL_ADC0_PWR_ELEC_BOARD:
            return mcu_internal_adc0.EnsureInitialized();
            break;

        case AdcChip::ADC_S5_INTERNAL_ADC1_PWR_ELEC_BOARD:
            return mcu_internal_adc1.EnsureInitialized();
            break;

        case AdcChip::ADC_ADS7952_PRESS_SENS_BOARD:
            return press_sense_ads7952.EnsureInitialized();
            break;

        case AdcChip::ADC_ADS7952_PWR_ELEC_BOARD:
            return pwr_elec_ads7952.EnsureInitialized();
            break;

        default:
            return false;
            break;
    }
}

/**
  * @brief  Retrieves communication status of the specified ADC chip. IT is considered responsive if there is a change
  *     in consecutive reading values.
  * @param sensor The ADC sensor channel from which data is requested.
  * @return True if the channel is responding and values aren't frozen, false otherwise.
  */
bool AdcData::IsResponding( AdcInputSensor sensor) noexcept
{
	volatile bool channelPassing = EnsureInitialized();
	if (channelPassing )
	{
		MutexGuard guard(spiBusMutex);

		auto FindPredicate = [&]( const AdcInfo& adcInfo ) { return ( adcInfo.adcInputSensor == sensor); };

		auto iterator = std::find_if( adcTable.begin(), adcTable.end(), FindPredicate );
		if( iterator )
		{

			for ( uint32_t tries = 0; tries < 3 ; ++tries )
			{
				float adc_v_reading = 0;
				uint32_t adcCount = 0;

				static constexpr uint8_t numOfSamplesToAvg = 1;
				static constexpr uint32_t timeBetweenSamples = 0;
				static constexpr time_unit_t timeUnit = TIME_UNIT_MS;

				// Read channel multiple times, then check for changing values.

				static constexpr uint32_t SuccessiveCounts = 15;
				std::array<uint16_t, SuccessiveCounts> adcCounts = {};

				for( uint32_t reading = 0;  reading < SuccessiveCounts; ++reading )
				{
					auto err = iterator->adc.ReadChannel(iterator->channel_num, adcCount, adc_v_reading,
							numOfSamplesToAvg,timeBetweenSamples, timeUnit);
					if (err == BaseAdc::AdcErr::ADC_SUCCESS)
					{
						adcCounts[reading] = static_cast<uint16_t>(adcCount);
					}
					else
					{
						channelPassing = false;
						ConsolePort::Write ("AdcData::IsResponding(%s) - ReadChannel(%u) failed, reason: %s (%u).",
							AdcInputSensorToString(sensor), iterator->channel_num, BaseAdc::AdcErrToString(err), err);
					}
				}

				if( channelPassing)
				{
					if( std::equal( adcCounts.begin() + 1, adcCounts.end(), adcCounts.begin()))  // Same value
					{
						if( adcCounts[0] > 10U )  // TODO:   Re-evaluate whether this is acceptable.
						{
							channelPassing = false;
							ConsolePort::Write ("AdcData::IsResponding(%s) - Channel (%u) unchanging, counts: %u.",
								AdcInputSensorToString(sensor), iterator->channel_num, adcCounts[0]);

							TxDelayMsec(10);
						}
					}
					else  // Passing test, early return
					{
						return channelPassing;
					}

				}
			}
		}
	}
	return channelPassing;
}
