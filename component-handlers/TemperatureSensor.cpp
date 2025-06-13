
#include <component_handlers/AdcData.h>
#include <component_handlers/TemperatureSensor.h>
#include <external_chip_drivers/MCP9700.h>
#include <external_chip_drivers/NRG2104H3435.h>
#include "platform_compat.h"

#include "UTILITIES/common/TxUtility.h"
#include "UTILITIES/common/CommonIDs.h"


bool TemperatureSensor::GetTemp(temperature_sensor_id_t component, temp_unit_id_t unit, float& temp_value, uint8_t numOfSamplesToAvg)
{
    bool ret = false;
    float adc_v_reading = -1.0f;

    uint32_t time_between_samples_msec = 1;

    switch(component) {
        case TEMP_SENSE_ID_HEATER:
        {
            ret = AdcData::GetInstance().GetVolt(AdcInputSensor::ADC_PWR_ELEC_THERMISTOR_HEATER, adc_v_reading, numOfSamplesToAvg, time_between_samples_msec, TIME_UNIT_MS);
            if(ret) {
                heaterThermistor.CalculateTemp(adc_v_reading, temp_value, unit);
            }

            break;
        }
        case TEMP_SENSE_ID_POWER_ELECTRONICS_BOARD:
        {
            ret = AdcData::GetInstance().GetVolt(AdcInputSensor::ADC_PWR_ELEC_AMBIENT_TEMP_SENSE, adc_v_reading, numOfSamplesToAvg, time_between_samples_msec, TIME_UNIT_MS);
            if(ret) {
            	MCP9700::CalculateTemp(adc_v_reading, temp_value, unit);
            }

            break;
        }
        case TEMP_SENSE_ID_PRESSURE_SENSOR_BOARD:
        {
            ret = AdcData::GetInstance().GetVolt(AdcInputSensor::ADC_PRESSURE_SENSE_AMBIENT_TEMP_SENSE, adc_v_reading, numOfSamplesToAvg, time_between_samples_msec, TIME_UNIT_MS);
            if(ret) {
            	MCP9700::CalculateTemp(adc_v_reading, temp_value, unit);
            }

            break;
        }
        case TEMP_SENSE_ID_HMI_BOARD:
        {
        	/// Since this is an external boards ADC data supplied, averaging features may not be available.
            ret = AdcData::GetInstance().GetVolt(AdcInputSensor::ADC_HMI_TEMPERATURE_SENSOR, adc_v_reading, numOfSamplesToAvg, time_between_samples_msec, TIME_UNIT_MS);
            if(ret) {
            	TMP20::CalculateTemp(adc_v_reading, temp_value, unit);
            }

            break;
        }
        default:
        {
            temp_value = 0.0;
            return false;
        }
    }

    return ret;
}

bool TemperatureSensor::ReadAndSendTemperatureToConsole(const std::vector<TemperatureSampleSetting>& sampleSettings) {
    /// Vector to hold the temperature readings
    std::vector<TemperatureData> temperatures;

    /// Read each temperature based on the settings
    for (const auto& setting : sampleSettings) {
        float temp;

        /// Get the current timestamp.
        uint32_t timestamp = GetElapsedTimeMsec();

        /// Assume a function `GetTemperature` that reads the temperature sensor based on ID and returns in Celsius.
        if (!GetTemp(setting.id, TEMP_C, temp, setting.num_of_samples)) {
            return false;
        }

        /// Store the reading
        temperatures.emplace_back(setting.id, temp, timestamp);
    }

    /// Send the readings to the console
    return SendTemperatureToConsole(temperatures);
}

bool TemperatureSensor::SendTemperatureToConsole(const std::vector<TemperatureData>& temperatures) {
    sensorFrame tempSenseFrame;

    tempSenseFrame.component = TEMPERATURE_SENSORS_DATA;
    tempSenseFrame.operation = DATA;

    /// Create an instance of the union
    union FloatIntUnion punningValue;

    /// Set the unsigned int member to 0xFFFFFFFF
    punningValue.u = 0xFFFFFFFF;

    /// Initialize data array with NaNs
    for (int i = 0; i < MAX_FLOAT_DATA_FIELDS; i++) {
        tempSenseFrame.data[i] = punningValue.f;
    }

    /// Assuming MAX_FLOAT_DATA_FIELDS is a multiple of 2 and each pressure sensor takes 2 slots
    const int maxTemperatureSensors = MAX_FLOAT_DATA_FIELDS / 2;

    for (const auto& temp : temperatures) {
        if (temp.id < maxTemperatureSensors) {
            int baseIndex = temp.id * 2;
            tempSenseFrame.data[baseIndex] = temp.value;
            tempSenseFrame.data[baseIndex + 1] = static_cast<float>(temp.timestamp_ms);
        }
        /// Handle the case where the ID is out of range
    }

    /// Send sensor frame to the console GUI
    ConsolePort::GetInstance().SendToApp(tempSenseFrame);

    return true;
}




