#ifndef COMPONENT_HANDLER_MOTOR_CONTROLLER_H_
#define COMPONENT_HANDLER_MOTOR_CONTROLLER_H_

#include "Tmc9660Handler.h"
#include <memory>

/**
 * @file MotorController.h
 * @brief Singleton class for managing the TMC9660 motor controller and its interfaces.
 *
 * This class provides a global singleton for the TMC9660 motor controller, including:
 * - Unified initialization and error handling
 * - Access to the Tmc9660Handler, driver, GPIO, and ADC
 * - Designed for single-device use (can be extended for multi-device)
 */
class MotorController {
public:
    /**
     * @brief Get the singleton instance.
     */
    static MotorController& GetInstance();

    /**
     * @brief Initialize the motor controller and TMC9660 device.
     * @return true if successful, false otherwise
     */
    bool Initialize();

    /**
     * @brief Get access to the Tmc9660Handler.
     */
    Tmc9660Handler& handler();

    /**
     * @brief Get access to the underlying TMC9660 driver.
     */
    TMC9660& driver();

    /**
     * @brief Get access to the GPIO wrapper for a given internal GPIO channel.
     */
    Tmc9660Handler::Gpio& gpio(uint8_t gpioNumber);

    /**
     * @brief Get access to the ADC wrapper.
     */
    Tmc9660Handler::Adc& adc();

private:
    MotorController();
    ~MotorController() = default;
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    std::unique_ptr<Tmc9660Handler> tmcHandler_;
};

#endif // COMPONENT_HANDLER_MOTOR_CONTROLLER_H_ 