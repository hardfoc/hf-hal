/**
 * @file BldcCanOpenController.h
 * @brief Simplified CANopen controller for a BLDC motor.
 */

#ifndef HAL_COMPONENT_HANDLERS_BLDC_CANOPEN_CONTROLLER_H_
#define HAL_COMPONENT_HANDLERS_BLDC_CANOPEN_CONTROLLER_H_

#include <HAL/internal_interface_drivers/FlexCan.h>
#include <cstdint>

/**
 * @class BldcCanOpenController
 * @brief Basic CANopen controller for a BLDC motor driver.
 */
class BldcCanOpenController {
public:
    /**
     * @brief Constructor.
     * @param bus CAN bus interface
     * @param nodeId Node ID of the motor driver
     */
    BldcCanOpenController(FlexCan& bus, uint8_t nodeId) noexcept;

    /**
     * @brief Initialize and start communication.
     */
    bool Initialize() noexcept;

    /**
     * @brief Start the motor via NMT command.
     */
    bool StartMotor() noexcept;

    /**
     * @brief Stop the motor via NMT command.
     */
    bool StopMotor() noexcept;

    /**
     * @brief Set the target velocity of the motor.
     * @param velocity Target velocity in manufacturer units
     */
    bool SetTargetVelocity(int32_t velocity) noexcept;

    /**
     * @brief Periodic processing of incoming frames.
     */
    bool Process() noexcept;

private:
    bool SendNmt(uint8_t command) noexcept;
    bool SendSdo(uint16_t index, uint8_t subIndex, uint32_t data, uint8_t size) noexcept;

    FlexCan& bus;
    uint8_t  node;
    bool     initialized;
};

#endif // HAL_COMPONENT_HANDLERS_BLDC_CANOPEN_CONTROLLER_H_
