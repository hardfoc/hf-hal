/**
 * @file FansHandler.cpp
 * @brief FansHandler class implementation
 * 
 * @section intro_sec Introduction
 * Contains the implementation of the "singleton-like" FansHandler class. FansHandler must be initialized
 * before use. It contains methods for handling the fans, such as enabling/disabling fans, setting their
 * duty cycle and frequency, and getting their speed, enabled status, duty cycle, and frequency.
 *
 * @note This class is thread and interrupt-safe.
 */

#include "UTILITIES/common/RtosCompat.h"
#include <array>
#include <algorithm>

#include <HAL/component_handlers/ConsolePort.h>
#include <HAL/component_handlers/FansHandler.h>
#include <HAL/internal_interface_drivers/DigitalInput.h>
#include <HAL/internal_interface_drivers/DigitalOutput.h>
#include <HAL/internal_interface_drivers/DigitalPWM.h>
#include <HAL/internal_interface_drivers/DigitalTachometer.h>


#include <UTILITIES/common/CommonIDs.h>
#include <UTILITIES/common/ThingsToString.h>
#include <UTILITIES/common/TxUtility.h>

//==============================================================//
/// VERBOSE??
//==============================================================//
static const bool verbose = false;

//==============================================================//
/// STRINGS
//==============================================================//

const char FansHandler::Fan::mutexBaseName[] = "FansHandlerMutex";

//==============================================================//
/// FAN DEFINITION
//==============================================================//

FansHandler::Fan::Fan(fan_id_t fanIdArg, DigitalOutput& fanEnableArg, DigitalPWM& fanPWMArg, DigitalTachometer& fanTachArg)
    : fanId(fanIdArg),
      fanEnable(fanEnableArg),
      fanPWM(fanPWMArg),
      fanTach(fanTachArg),
      mutex(mutexBaseName, FanIdToString(fanId)),
      fanInitialized(false)
{
    /// No code at this time
}

FansHandler::Fan::~Fan() {
    /// No code at this time
}

bool FansHandler::Fan::EnsureInitialized() noexcept
{
    if (!fanInitialized)
    {
        fanInitialized = FanInitialize();
    }
    return fanInitialized;
}

/**
 * @brief Initialize the peripheral.
 * @returns True if initialization is successful, false otherwise.
 */
bool FansHandler::Fan::FanInitialize() noexcept
{
    bool success = true;
    MutexGuard guard(mutex, MutexGuard::MaxInitializationTimeMsec);

    if (!fanTach.EnsureInitialized()) {
        WRITE_CONDITIONAL(verbose, "Fan %s(%d): Failed to enable power to fans !!!", FanIdToString(GetId()), GetId() );
        success = false;
    }

    if (fanPWM.SetDutyCycle(0) != ESP_OK) {
        WRITE_CONDITIONAL(verbose, "Fan %s(%d): Failed to initialize PWM output !!!", FanIdToString(GetId()), GetId() );
        success = false;
    }

    if (!fanEnable.SetActive()) {
        WRITE_CONDITIONAL(verbose, "Fan %s(%d): Failed to initialize low levels for Tach Input Capture !!!", FanIdToString(GetId()), GetId() );
        success = false;
    }

    return success;
}

//==============================================================//
/// FANS HANDLER CLASS
//==============================================================//

/**
 * @brief Constructor for FansHandler.
 * Initializes the FanStates array to a known state.
 */
FansHandler::FansHandler() noexcept :
    initialized(false),
    Fan1_Enable(FAN_1_EN, DigitalGpio::ActiveState::High, DigitalGpio::State::Inactive),
    Fan1_PWM((e_ioport_port_pin_t)FAN_1_SPEED_CTRL, &g_fans_speed_timer, DigitalPWM::component_channel_port_t::PORT_A),
    Fan1_Tach((e_ioport_port_pin_t)FAN_2_TACH_IN, g_fan_1_tach_input_capture, Fan1_PulsesPerRev),
    Fan2_Enable(FAN_2_EN, DigitalGpio::ActiveState::High, DigitalGpio::State::Inactive),
    Fan2_PWM((e_ioport_port_pin_t)FAN_2_SPEED_CTRL, &g_fans_speed_timer, DigitalPWM::component_channel_port_t::PORT_B),
    Fan2_Tach((e_ioport_port_pin_t)FAN_2_TACH_IN, g_fan_2_tach_input_capture, Fan2_PulsesPerRev),
    fans
    {
        Fan(FAN_1, Fan1_Enable, Fan1_PWM, Fan1_Tach),
        Fan(FAN_2, Fan2_Enable, Fan2_PWM, Fan2_Tach),
    }
{
    /// No code at this time
}

/**
 * @brief Destructor for FansHandler.
 * Shuts down the fans before the handler is destroyed.
 */
FansHandler::~FansHandler() noexcept
{
    for (auto& fan : fans) {
		fan.fanPWM.SetDutyCycle(0);
		fan.fanEnable.SetInactive();
		fan.fanTach.DisableCapture();
    }
}

bool FansHandler::Initialize() noexcept
{
    bool status = true;

    WRITE_CONDITIONAL(verbose, "FansHandler::Initialize() - Initializing FansHandler");

    for (auto& fan : fans) {
        /// Ensure each fans is initialized
        if (!fan.EnsureInitialized()) {
            WRITE_CONDITIONAL(verbose, "FansHandler::Initialize() - Fan %d: Failed to initialize top mutex", fan.GetId());
            status = false;
        }
        else {
            WRITE_CONDITIONAL(verbose, "FansHandler::Initialize() - Fan %d: Initialized", fan.GetId());
        }
    }

    return status;
}


bool FansHandler::Enable(fan_id_t fanId) noexcept
{
    bool ret = false;

    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

			// Set fan state to disabled
			ret = iterator->fanEnable.SetActive();

			if (verbose) {
				if(ret == true)
					ConsolePort::Write("FansHandler::Enable() - Enabled fan %d", fanId);
				else
					ConsolePort::Write("FansHandler::Enable() - Failed to enable fan %d", fanId);
			}
		}
    }
    return ret;
}

bool FansHandler::Disable(fan_id_t fanId) noexcept
{
    bool ret = false;
    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

			// Set fan state to disabled
			ret = iterator->fanEnable.SetInactive();

			// TODO: CHECK RPM FIRST BEFORE MOVING OUT

			if (verbose) {
				if((ret == true))
					ConsolePort::Write("FansHandler::Disable() - Disabled fan %d", fanId);
				else
					ConsolePort::Write("FansHandler::Disable() - Failed to Disable fan %d", fanId);
			}
		}
    }
    return ret;
}

bool FansHandler::SetDutyCycle(fan_id_t fanId, uint32_t dutyCycle) noexcept
{
    bool ret = false;

    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

                        esp_err_t err = iterator->fanPWM.SetDutyCycle(dutyCycle);
                        if(err == ESP_OK)
			{
				ret = true;
			}

			// TODO: Set the duty cycle for the fan using the corresponding DigitalPWM instance

			if((ret == true))
			{
				WRITE_CONDITIONAL(verbose, "FansHandler::SetDutyCycle() - Set duty cycle to %d for fan %d.", dutyCycle, fanId);
			}
			else
			{
				ConsolePort::Write("FansHandler::SetDutyCycle() - Failed to set the duty cycle to %d for fan %d.", dutyCycle, fanId);
			}

		}
    }
    return ret;
}

bool FansHandler::SetFrequency(fan_id_t fanId, uint32_t frequency) noexcept
{
    bool ret = false;

    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

			// Set frequency
                        esp_err_t err = iterator->fanPWM.SetTimerChannelFrequency(frequency);
                        if(err == ESP_OK)
			{
				ret = true;
			}

			// TODO: Set the frequency for the fan using the corresponding DigitalPWM instance

			if (verbose) {
				if((ret == true))
					ConsolePort::Write("FansHandler::SetFrequency() - Set frequency to %d for fan %d", frequency, fanId);
				else
					ConsolePort::Write("FansHandler::SetFrequency() - Failed to set frequency to %d for fan %d", frequency, fanId);
			}
		}
    }
    return ret;
}

float FansHandler::GetSpeedRPM(fan_id_t fanId)  noexcept
{
    float rpm = -1.0;
    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);


			rpm = static_cast<float>( iterator->fanTach.readRPM() );

			if (verbose) {
				if((rpm >= 0))
					ConsolePort::Write("FansHandler::GetSpeedRPM() - Fan %d speed is at RPM = %0.2f", fanId, rpm);
				else
					ConsolePort::Write("FansHandler::GetSpeedRPM() - Failed to get speed for fan %d", fanId);
			}

		}
    }
    return rpm;  // return an error code if the fan is not found or the mutex isn't created
}

bool FansHandler::IsEnabled(fan_id_t fanId) noexcept
{
    bool ret = false;
    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

			ret = iterator->fanEnable.IsActive();

			if (verbose) {
				if((ret == true))
					ConsolePort::Write("FansHandler::IsEnabled() - Checking enabled status for fan %d, result - %s", fanId, (ret?"ENABLED":"DISABLED"));
				else
					ConsolePort::Write("FansHandler::IsEnabled() - Failed when checking enabled status for fan %d", fanId);
			}

			return iterator->fanEnable.IsActive();
		}
    }
    return false;
}

uint32_t FansHandler::GetDutyCycle(fan_id_t fanId) noexcept
{
    uint32_t dutyCycle = 0;
    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

			dutyCycle = iterator->fanPWM.GetDutyCycle();

			if (verbose) {
				ConsolePort::Write("FansHandler::GetDutyCycle() - Getting duty cycle for fan %d - result : %d%", fanId, dutyCycle);
			}

		}
    }
    return dutyCycle;
}

uint32_t FansHandler::GetFrequency(fan_id_t fanId) noexcept
{
    uint32_t frequency = 0;
    if( EnsureInitialized() )
    {
		auto FindPredicate = [&](const Fan& entry) { return (entry.fanId == fanId); };

		auto iterator = std::find_if(fans.begin(), fans.end(), FindPredicate);
		if(iterator != fans.end())
		{
			MutexGuard guard(iterator->mutex);

			frequency = iterator->fanPWM.GetTimerChannelFrequency();

			if (verbose) {
				ConsolePort::Write("FansHandler::GetFrequency() - Getting frequency for fan %d - result %d", fanId, frequency);
			}

		}
    }
    return frequency;
}


