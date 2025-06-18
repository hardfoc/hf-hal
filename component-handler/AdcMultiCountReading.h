#ifndef COMPONENT_HANDLER_ADC_MULTI_COUNT_READING_H_
#define COMPONENT_HANDLER_ADC_MULTI_COUNT_READING_H_

#include "MultiReadings.h"
#include "CommonIDs.h"
#include "ThingsToString.h"

/**
 * @file AdcMultiCountReading.h
 * @brief Specialization of MultiReadings for ADC count readings.
 */

/**
 * @class AdcMultiCountReading
 * @brief Specialization of MultiReadings for ADC count readings.
 * 
 * @tparam MaxSensorCount The maximum number of sensors that can be handled.
 */
template <uint8_t MaxSensorCount>
class AdcMultiCountReading : public MultiReadings<AdcInputSensor, uint32_t, MaxSensorCount, void>
{
public:
    /**
     * @brief Type alias for the base class.
     */
    using Base = MultiReadings<AdcInputSensor, uint32_t, MaxSensorCount, void>;

    /**
     * @brief Type alias for the Reading structure of the base class.
     */
    using Reading = typename Base::Reading;

    /**
     * @brief Default constructor.
     */
    AdcMultiCountReading() noexcept
        : Base(&AdcInputSensorToString)
    { }

    /**
     * @brief Constructor that takes an initializer list of pairs.
     * @param inputChannels The initializer list of pairs (sensor, samples).
     */
    AdcMultiCountReading(const std::initializer_list<std::pair<AdcInputSensor, uint8_t>>& inputChannels) noexcept
        : Base(inputChannels, &AdcInputSensorToString)
    { }

    /**
     * @brief Copy constructor.
     */
    AdcMultiCountReading(const AdcMultiCountReading& copy) noexcept = default;

    /**
     * @brief Copy assignment operator.
     */
    AdcMultiCountReading& operator=(const AdcMultiCountReading& copy) noexcept = default;

    /**
     * @brief Destructor.
     */
    ~AdcMultiCountReading() noexcept = default;
};

/**
 * @class AdcMultiVoltageReading
 * @brief Specialization of MultiReadings for ADC voltage readings.
 * 
 * @tparam MaxSensorCount The maximum number of sensors that can be handled.
 */
template <uint8_t MaxSensorCount>
class AdcMultiVoltageReading : public MultiReadings<AdcInputSensor, float, MaxSensorCount, void>
{
public:
    /**
     * @brief Type alias for the base class.
     */
    using Base = MultiReadings<AdcInputSensor, float, MaxSensorCount, void>;

    /**
     * @brief Type alias for the Reading structure of the base class.
     */
    using Reading = typename Base::Reading;

    /**
     * @brief Default constructor.
     */
    AdcMultiVoltageReading() noexcept
        : Base(&AdcInputSensorToString)
    { }

    /**
     * @brief Constructor that takes an initializer list of pairs.
     * @param inputChannels The initializer list of pairs (sensor, samples).
     */
    AdcMultiVoltageReading(const std::initializer_list<std::pair<AdcInputSensor, uint8_t>>& inputChannels) noexcept
        : Base(inputChannels, &AdcInputSensorToString)
    { }

    /**
     * @brief Copy constructor.
     */
    AdcMultiVoltageReading(const AdcMultiVoltageReading& copy) noexcept = default;

    /**
     * @brief Copy assignment operator.
     */
    AdcMultiVoltageReading& operator=(const AdcMultiVoltageReading& copy) noexcept = default;

    /**
     * @brief Destructor.
     */
    ~AdcMultiVoltageReading() noexcept = default;
};

#endif // COMPONENT_HANDLER_ADC_MULTI_COUNT_READING_H_
