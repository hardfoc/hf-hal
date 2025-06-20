#ifndef COMPONENT_HANDLER_TMC9660_CONTROLLER_H_
#define COMPONENT_HANDLER_TMC9660_CONTROLLER_H_

#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/BaseAdc.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/DigitalGpio.h"
#include "CommonIDs.h"
#include <cstdint>
#include <string_view>

/**
 * @file TMC9660Controller.h
 * @brief Motor controller class for TMC9660 with integrated ADC and GPIO functionality.
 * 
 * This class provides a unified interface for controlling the TMC9660 motor driver
 * and reading its integrated ADC channels for current sensing and diagnostics.
 */

/**
 * @class TMC9660Adc
 * @brief ADC interface for TMC9660 internal ADC channels.
 * 
 * The TMC9660 has 3 internal ADC channels for current sensing:
 * - Channel 0: Phase A current
 * - Channel 1: Phase B current  
 * - Channel 2: Phase C current
 */
class TMC9660Adc : public BaseAdc {
public:
    /**
     * @brief Constructor for TMC9660 ADC interface.
     * @param controller Reference to the parent TMC9660 controller.
     */
    explicit TMC9660Adc(class TMC9660Controller& controller) noexcept;

    /**
     * @brief Destructor.
     */
    ~TMC9660Adc() noexcept override = default;

    /**
     * @brief Initialize the TMC9660 ADC.
     * @return true if successful, false otherwise.
     */
    bool Initialize() noexcept override;

    /**
     * @brief Read ADC channel voltage.
     * @param channel_num ADC channel (0-2 for TMC9660).
     * @param channel_reading_v Output voltage reading.
     * @param numOfSamplesToAvg Number of samples to average.
     * @param timeBetweenSamples Delay between samples in ms.
     * @return ADC error status.
     */
    AdcErr ReadChannelV(uint8_t channel_num, float &channel_reading_v,
                        uint8_t numOfSamplesToAvg = 1,
                        uint32_t timeBetweenSamples = 0) noexcept override;

    /**
     * @brief Read ADC channel raw count.
     * @param channel_num ADC channel (0-2 for TMC9660).
     * @param channel_reading_count Output raw count.
     * @param numOfSamplesToAvg Number of samples to average.
     * @param timeBetweenSamples Delay between samples in ms.
     * @return ADC error status.
     */
    AdcErr ReadChannelCount(uint8_t channel_num, uint32_t &channel_reading_count,
                            uint8_t numOfSamplesToAvg = 1,
                            uint32_t timeBetweenSamples = 0) noexcept override;

    /**
     * @brief Read ADC channel count and voltage.
     * @param channel_num ADC channel (0-2 for TMC9660).
     * @param channel_reading_count Output raw count.
     * @param channel_reading_v Output voltage reading.
     * @param numOfSamplesToAvg Number of samples to average.
     * @param timeBetweenSamples Delay between samples in ms.
     * @return ADC error status.
     */
    AdcErr ReadChannel(uint8_t channel_num, uint32_t &channel_reading_count,
                       float &channel_reading_v, uint8_t numOfSamplesToAvg = 1,
                       uint32_t timeBetweenSamples = 0) noexcept override;

private:
    class TMC9660Controller& controller_;
};

/**
 * @class TMC9660Controller
 * @brief Comprehensive motor controller for TMC9660 with integrated peripherals.
 * 
 * This class manages the TMC9660 motor driver including:
 * - SPI communication for control and configuration
 * - Internal ADC for current sensing (3 channels)
 * - Motor control functions (enable, direction, step, etc.)
 * - Diagnostic and status monitoring
 */
class TMC9660Controller {
public:
    /**
     * @brief TMC9660 status structure.
     */
    struct Status {
        bool isInitialized;
        bool isEnabled;
        bool hasFault;
        int32_t position;
        int32_t velocity;
        float currentA;
        float currentB;
        float currentC;
        uint16_t temperature;
        uint16_t voltage;
    };

    /**
     * @brief Motor control modes.
     */
    enum class ControlMode : uint8_t {
        DISABLED = 0,
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        TORQUE_CONTROL,
        STEP_DIR_CONTROL
    };

    /**
     * @brief Constructor.
     */
    TMC9660Controller() noexcept;

    /**
     * @brief Destructor.
     */
    ~TMC9660Controller() noexcept = default;

    /**
     * @brief Initialize the TMC9660 controller.
     * @return true if successful, false otherwise.
     */
    bool Initialize() noexcept;

    /**
     * @brief Get the integrated ADC interface.
     * @return Reference to TMC9660 ADC.
     */
    TMC9660Adc& GetAdc() noexcept { return adc_; }

    /**
     * @brief Enable the motor.
     * @return true if successful, false otherwise.
     */
    bool EnableMotor() noexcept;

    /**
     * @brief Disable the motor.
     * @return true if successful, false otherwise.
     */
    bool DisableMotor() noexcept;

    /**
     * @brief Set motor control mode.
     * @param mode Control mode to set.
     * @return true if successful, false otherwise.
     */
    bool SetControlMode(ControlMode mode) noexcept;

    /**
     * @brief Set target position (position control mode).
     * @param position Target position in encoder counts.
     * @return true if successful, false otherwise.
     */
    bool SetTargetPosition(int32_t position) noexcept;

    /**
     * @brief Set target velocity (velocity control mode).
     * @param velocity Target velocity in counts/sec.
     * @return true if successful, false otherwise.
     */
    bool SetTargetVelocity(int32_t velocity) noexcept;

    /**
     * @brief Set target torque (torque control mode).
     * @param torque Target torque (normalized -1.0 to 1.0).
     * @return true if successful, false otherwise.
     */
    bool SetTargetTorque(float torque) noexcept;

    /**
     * @brief Read current motor status.
     * @return Motor status structure.
     */
    Status GetStatus() noexcept;

    /**
     * @brief Read phase currents from internal ADC.
     * @param currentA Output phase A current in Amps.
     * @param currentB Output phase B current in Amps.
     * @param currentC Output phase C current in Amps.
     * @return true if successful, false otherwise.
     */
    bool ReadPhaseCurrents(float& currentA, float& currentB, float& currentC) noexcept;

    /**
     * @brief Check if motor controller is responding.
     * @return true if responding, false otherwise.
     */
    bool IsResponding() noexcept;

    /**
     * @brief Run motor controller diagnostics.
     * @return true if all tests pass, false otherwise.
     */
    bool RunDiagnostics() noexcept;

    // Internal ADC access for TMC9660Adc class
    friend class TMC9660Adc;

private:
    /**
     * @brief Read TMC9660 internal ADC channel.
     * @param channel Channel number (0-2).
     * @param rawValue Output raw ADC value.
     * @return true if successful, false otherwise.
     */
    bool ReadInternalAdc(uint8_t channel, uint32_t& rawValue) noexcept;

    /**
     * @brief Convert raw ADC value to current in Amps.
     * @param rawValue Raw ADC value.
     * @param channel ADC channel (for calibration).
     * @return Current in Amps.
     */
    float ConvertRawToCurrent(uint32_t rawValue, uint8_t channel) noexcept;

    /**
     * @brief Send SPI command to TMC9660.
     * @param command Command to send.
     * @param data Data to send.
     * @return true if successful, false otherwise.
     */
    bool SendSpiCommand(uint8_t command, uint32_t data) noexcept;

    /**
     * @brief Read SPI data from TMC9660.
     * @param command Command to read.
     * @param data Output data received.
     * @return true if successful, false otherwise.
     */
    bool ReadSpiData(uint8_t command, uint32_t& data) noexcept;

    TMC9660Adc adc_;
    bool initialized_;
    ControlMode currentMode_;
    Status lastStatus_;
    
    // Calibration constants for current sensing
    static constexpr float CURRENT_SCALE_FACTOR = 0.001f;  // mA per LSB
    static constexpr float CURRENT_OFFSET = 1.65f;         // Offset voltage
};

#endif // COMPONENT_HANDLER_TMC9660_CONTROLLER_H_
