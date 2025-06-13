/**
 * @file FansHandler.h
 * @brief FansHandler class declaration
 * 
 * @section intro_sec Introduction
 * Contains the declaration of the "singleton-like" FansHandler class. FansHandler must be initialized
 * before use.   Base on empriracle data:
 *   Duty Cycle		 Fan Speed (RPM)
 * 		10				   0
 * 		15  		 	 600
 * 		25				1040
 * 		35				1410
 * 		45				1780
 * 		55				2155
 * 		65				2480
 * 		75				2823
 * 		85				3143
 * 		95				3386
 *
 *
 * @note This class is thread and interrupt-safe.
 */

#ifndef FANS_HANDLER_H
#define FANS_HANDLER_H

#include <array>

#include <HAL/internal_interface_drivers/DigitalInput.h>
#include <HAL/internal_interface_drivers/DigitalOutput.h>
#include <HAL/internal_interface_drivers/DigitalPWM.h>
#include <HAL/internal_interface_drivers/DigitalTachometer.h>

#include <UTILITIES/common/CommonIDs.h>
#include <UTILITIES/common/ThingsToString.h>

//==============================================================//
// FUNCTIONS
//==============================================================//
/**
 * @class FansHandler
 * @brief Singleton class for handling fans.
 */
class FansHandler {
    public:

        /**
         * @brief Get the singleton instance of FansHandler.
         * @return The singleton instance of FansHandler.
         */
        static FansHandler& GetInstance() noexcept {
            static FansHandler instance;
            return instance;
        }

        /**
         * @brief Copy constructor.
         * @details This constructor is deleted to prevent copying of instances.
         */
        FansHandler(const FansHandler&) = delete;

        /**
         * @brief Assignment operator.
         * @details This operator is deleted to prevent copying of instances.
         */
        void operator=(const FansHandler&) = delete;

        /**
		  * @brief  This function checks if the class is initialized;
		  * @return true if calibrated, false otherwise
		  */

		inline bool IsInitialized() const noexcept
		{
			return initialized;
		}

        /**
         * @brief Ensures the handler is initialized.
         * @return True if the handler is initialized, false otherwise.
         */
        bool EnsureInitialized() noexcept {
            if (!initialized) {
                initialized = Initialize();
            }
            return initialized;
        }

        /**
         * @brief Enables a fan.
         * @param fan The fan to be enabled.
         */
        bool Enable(fan_id_t fanId) noexcept;

        /**
         * @brief Disables a fan.
         * @param fan The fan to be disabled.
         */
        bool Disable(fan_id_t fanId) noexcept;

        /**
         * @brief Gets the enabled status of a fan.
         *
         * @param fan The fan to get the status for.
         *
         * @return The enabled status of the fan.
         */
        bool IsEnabled(fan_id_t fanId) noexcept;

        /**
         * @brief Sets the duty cycle of a fan.
         * @param fan The fan to set the duty cycle for.
         * @param dutyCycle The duty cycle to set.
         */
        bool SetDutyCycle(fan_id_t fanId, uint32_t dutyCycle) noexcept;

        /**
         * @brief Gets the duty cycle of a fan.
         *
         * @param fan The fan to get the duty cycle for.
         *
         * @return The duty cycle of the fan.
         */
        uint32_t GetDutyCycle(fan_id_t fanId) noexcept;

        /**
         * @brief Sets the frequency of a fan.
         * @param fan The fan to set the frequency for.
         * @param frequency The frequency to set.
         */
        bool SetFrequency(fan_id_t fanId, uint32_t frequency) noexcept;

        /**
         * @brief Gets the frequency of a fan.
         *
         * @param fan The fan to get the frequency for.
         *
         * @return The frequency of the fan.
         */
        uint32_t GetFrequency(fan_id_t fanId) noexcept;

        /**
         * @brief Gets the speed of a fan in RPM (Rotations Per Minute).
         * @param fan The fan to get the speed for.
         * @return The speed of the fan in RPM.
         */
        float GetSpeedRPM(fan_id_t fanId) noexcept;

    private:

        /**
         * @brief Private constructor for FansHandler.
         * This constructor initializes the fans handler with default values.
         */
        FansHandler() noexcept;

        /**
         * @brief Destructor for FansHandler.
         * Handles stopping the fans.
         */
        ~FansHandler() noexcept;

        /**
         * @brief Initializes the handler.
         * @return True if initialization is successful, false otherwise.
         */
        bool Initialize() noexcept;

        //==============================================================//
        /// FAN DEFINITION
        //==============================================================//

/**
		 * @brief Represents a fan with control and monitoring capabilities.
		 */
		struct Fan
		{
		public:
			/**
			 * @brief Unique identifier for the fan.
			 */
			fan_id_t fanId;

			/**
			 * @brief Reference to the digital output used to enable the fan.
			 */
			DigitalOutput& fanEnable;

			/**
			 * @brief Reference to the PWM control for the fan.
			 */
			DigitalPWM& fanPWM;

			/**
			 * @brief Reference to the tachometer for monitoring fan speed.
			 */
			DigitalTachometer& fanTach;

			/**
			 * @brief Mutex for thread-safe operations.
			 */
			Mutex mutex;


			/**
			 * @brief Parameterized Constructor.
			 * @param fanIdArg Unique identifier for the fan.
			 * @param fanEnableArg Reference to the digital output used to enable the fan.
			 * @param fanPWMArg Reference to the PWM control for the fan.
			 * @param fanTachArg Reference to the tachometer for monitoring fan speed.
			 */
			Fan(fan_id_t fanIdArg, DigitalOutput& fanEnableArg, DigitalPWM& fanPWMArg, DigitalTachometer& fanTachArg);

			/**
			 * @brief Destructor.
			 */
			~Fan();

			/**
			 * @brief Ensure class initialization.
			 * @details This function checks if the class is initialized, and if not, initializes it.
			 * @returns True if the class is now initialized, false otherwise.
			 */
			bool EnsureInitialized() noexcept;

			/**
			 * @brief Get the fan's unique identifier.
			 * @returns The fan's unique identifier.
			 */
			fan_id_t GetId() noexcept { return fanId; }

		private:

			/**
			 * @brief Initialize the peripheral.
			 * @returns True if initialization is successful, false otherwise.
			 */
			bool FanInitialize() noexcept;

			/**
			 * @brief Base name for the mutex.
			 */
	        static const char mutexBaseName[];

			/**
			 * @brief Indicates whether the fan has been initialized.
			 */
			bool fanInitialized;
		};

        //==============================================================//
        /// KEY VARIABLES
        //==============================================================//

        bool initialized;
        static constexpr uint8_t NumberOfFans = 2;

        DigitalOutput Fan1_Enable;      /**< Instance of DigitalOutput representing the Fan 1 enable signal. */
        DigitalPWM Fan1_PWM;            /**< Instance of DigitalPWM for controlling Fan 1 speed [duty cycle] and frequency. */
        DigitalTachometer Fan1_Tach;    /**< Instance of DigitalTachometer for capturing Fan 1 Tachometer Output. */
        static constexpr uint8_t Fan1_PulsesPerRev = 4;   /**< Fan 1 number of pulses per revolution. */


        DigitalOutput Fan2_Enable;      /**< Instance of DigitalOutput representing the Fan 1 enable signal. */
        DigitalPWM Fan2_PWM;            /**< Instance of DigitalPWM for controlling Fan 2 speed [duty cycle] and frequency. */
        DigitalTachometer Fan2_Tach;    /**< Instance of DigitalTachometer for capturing Fan 1 Tachometer Output. */
        static constexpr uint8_t Fan2_PulsesPerRev = 4;   /**< Fan 2 number of pulses per revolution. */

        std::array<Fan, NumberOfFans> fans;                       // Lookup table for fans
        static const std::array<fan_id_t, NumberOfFans> AllFans;    // List of all the fans
};

#endif  // FANS_HANDLER_H
