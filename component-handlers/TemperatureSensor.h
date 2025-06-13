
#ifndef TEMPERATURE_SENSOR_H_
#define TEMPERATURE_SENSOR_H_

#include <vector>
#include "platform_compat.h"

#include <HAL/component_handlers/AdcData.h>
#include <HAL/external_chip_drivers/MCP9700.h>
#include <HAL/external_chip_drivers/NRG2104H3435.h>
#include <HAL/external_chip_drivers/NRG2104H3435.h>
#include <HAL/external_chip_drivers/TMP20.h>

#include "UTILITIES/common/CommonIDs.h"

    //=====================================//
    // TEMPERATURE DATA RELATED TO CONSOLE
    //=====================================//

struct TemperatureSampleSetting {
    temperature_sensor_id_t id;
    uint8_t num_of_samples;

    /** Constructor that takes all parameters */
    TemperatureSampleSetting(temperature_sensor_id_t temp_id, uint8_t samples)
        : id(temp_id), num_of_samples(samples) {}

    /** Constructor that only takes the id and sets default values for the other members */
    TemperatureSampleSetting(temperature_sensor_id_t temp_id)
        : id(temp_id), num_of_samples(1) {}  // Default to 1 sample
};

struct TemperatureData {
    temperature_sensor_id_t id;
    float value;
    uint32_t timestamp_ms;

    /** Constructor that takes all parameters */
    TemperatureData(temperature_sensor_id_t temp_id, float tempVal, uint32_t timeMsec)
        : id(temp_id), value(tempVal), timestamp_ms(timeMsec) {}

    /** Constructor that only takes the id and value and fetches the timestamp at creation */
    TemperatureData(temperature_sensor_id_t temp_id, float tempVal)
        : id(temp_id), value(tempVal), timestamp_ms(GetElapsedTimeMsec()) {}
};

    //===================================//
    //  TEMPERATURE SENSOR CLASS
    //===================================//

/**
 * @class TemperatureSensor
 * @brief Singleton class for temperature measurements.
 *
 * @details
 * The TemperatureSensor class represents a temperature sensor and provides functionality for temeprature measurements.
 * It employs lazy initialization, so the instance is initialized the first time it is used.
 */
class TemperatureSensor {
public:

    /**
     * @brief Returns the singleton instance of TemperatureSensor.
     * @return The singleton instance of TemperatureSensor.
     */
    static TemperatureSensor& GetInstance() {
        static TemperatureSensor instance;
        return instance;
    }

    /**
     * @brief Returns the temperature through the specified component in the specified unit.
     *
     * @param component The temperature sensor component.
     * @param unit The unit in which the temperature should be calculated.
     *
     * @return SSP error code:
     *     - SSP_SUCCESS: If the current value is successfully calculated.
     */
    bool GetTemp(temperature_sensor_id_t component, temp_unit_id_t unit, float& temp_value, uint8_t numOfSamplesToAvg = 1);

    /**
     * @brief Reads the temperature sensors based on provided settings and sends the readings to the console.
     *
     * This function processes a list of `TemperatureSampleSetting` structures, which provides
     * information regarding a specific temperature sensor ID and its associated sampling parameters.
     * It then captures and transmits the data for each temperature sensor to the standard output.
     *
     * @param sampleSettings A vector of `TemperatureSampleSetting` structures. Each structure denotes
     * a specific temperature sensor and its sampling parameters (number of samples).
     *
     * @note If a `TemperatureSampleSetting` structure does not specify the number of samples,
     * the function will employ the default value incorporated within the `TemperatureSampleSetting` structure.
     *
     * @return Returns `true` if all temperature readings were successfully dispatched to the console,
     * and `false` otherwise.
     *
     * @see TemperatureSampleSetting
     *
     * Example usage:
     * @code
     * ReadAndSendTemperatureToConsole({{HEATER_TEMP, 3}, {BOX_TEMP}});
     * @endcode
     *
     * The above example breakdown assumes default values for unspecified parameters.
     * @code
     * ID: HEATER_TEMP, Number of Samples: 3
     * ID: BOX_TEMP, Number of Samples: 1  // Default value for number of samples is 1
     * @endcode
     */
    bool ReadAndSendTemperatureToConsole(const std::vector<TemperatureSampleSetting>& sampleSettings);

    /**
     * @brief Dispatches a collection of temperature sensor data points to the console for real-time tracking.
     *
     * The `SendTemperatureToConsole` function processes a vector of `TemperatureData` structures.
     * Each structure in this vector holds an ID of a temperature sensor, its current value, and an
     * associated timestamp. This function outputs these data points to the console, enabling immediate
     * visualization and analysis.
     *
     * @param temperatures A vector incorporating `TemperatureData` structures. Each data point reflects a
     * reading from a temperature sensor at a particular timestamp. If the timestamp is not specified during
     * the creation, the structure will employ the timestamp when the `TemperatureData` object was constructed
     * with GetElapsedTimeMsec() function.
     *
     * @return Returns `true` if all temperature values were successfully printed to the console, `false` otherwise.
     *
     * @note The sequence of data in the `temperatures` vector is critical. Ensure data is systematically
     * arranged or organized in line with the prerequisites of the processing logic.
     *
     * @see TemperatureData
     *
     * Example usage:
     * @code
     * bool result = SendTemperatureToConsole({{HEATER_TEMP, 37.5f, 54321}, {BOX_TEMP, 22.5f}});
     * if (result) {
     *     ConsolePort::GetInstance().Write("Successfully dispatched temperature readings.");
     * }
     * @endcode
     */
    bool SendTemperatureToConsole(const std::vector<TemperatureData>& temperatures);

private:

    TemperatureSensor() = default;

    /**
     * @brief Deleted copy constructor to prevent copying of the instance.
     */
    TemperatureSensor(const TemperatureSensor&) = delete;

    /**
     * @brief Deleted assignment operator to prevent copying of the instance.
     */
    TemperatureSensor& operator=(const TemperatureSensor&) = delete;

    /**
      *  Destructor is simple, it disables interrupts and closes the port.
      */
    virtual ~TemperatureSensor() noexcept = default;

    NRG2104H3435 heaterThermistor;		/// Thermistor found on manifold's input CO2 heater section
 //   MCP9700	pwrElecBoardThermistor;		/// Thermistor IC for ambient temperature on back of power electronics board.
 //   MCP9700	pressSenseBoardThermistor;	/// Thermistor IC for ambient temperature on back of pressure sensor board.
   // TMP20 hmiBoardthermistor;			/// Thermistor IC for ambient temperature on back of HMI board.
};

#endif /* TEMPERATURE_SENSOR_H_ */
