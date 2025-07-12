#ifndef COMPONENT_HANDLER_TMC9660_MOTOR_CONTROLLER_H_
#define COMPONENT_HANDLER_TMC9660_MOTOR_CONTROLLER_H_

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <functional>

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "AdcManager.h"
#include "GpioManager.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/TMC9660.h"
#include "base/BaseAdc.h"
#include "mcu/McuDigitalGpio.h"

/**
 * @file Tmc9660MotorController.h
 * @brief TMC9660 motor controller singleton class for the HardFOC system.
 * 
 * This class provides a unified interface for TMC9660 motor controller chips,
 * supporting both SPI and UART communication interfaces with proper GPIO control.
 */

/**
 * @brief Structure containing TMC9660 chip configuration.
 */
struct Tmc9660Config {
    Tmc9660ChipId chipId;                    ///< Unique chip identifier
    Tmc9660CommInterface primaryInterface;   ///< Primary communication interface
    Tmc9660CommInterface secondaryInterface; ///< Secondary interface (optional)
    uint8_t spiDeviceId;                     ///< SPI device ID (if using SPI)
    uint8_t uartPort;                        ///< UART port number (if using UART)
    uint32_t spiFrequency;                   ///< SPI frequency in Hz
    uint32_t uartBaudRate;                   ///< UART baud rate
    bool spiEnabled;                         ///< Whether SPI is currently enabled
    bool uartEnabled;                        ///< Whether UART is currently enabled
};

/**
 * @brief Structure containing TMC9660 status information.
 */
struct Tmc9660Status {
    bool isInitialized;                      ///< Whether chip is initialized
    bool isCommunicating;                    ///< Whether communication is active
    bool hasFault;                           ///< Whether chip has fault condition
    bool isDriverEnabled;                    ///< Whether driver output is enabled
    bool isAwake;                            ///< Whether chip is awake (not in sleep)
    uint32_t lastCommunicationTime;          ///< Last successful communication timestamp
    uint32_t faultCount;                     ///< Number of faults detected
    uint32_t communicationErrors;            ///< Number of communication errors
};

/**
 * @brief Structure for TMC9660 GPIO pin configuration.
 */
struct Tmc9660GpioConfig {
    GpioPin gpio17Pin;                       ///< TMC9660 GPIO17 pin mapping
    GpioPin gpio18Pin;                       ///< TMC9660 GPIO18 pin mapping
    GpioPin faultStatusPin;                  ///< Fault status input pin
    GpioPin driverEnablePin;                 ///< Driver enable output pin
    GpioPin resetControlPin;                 ///< Reset control output pin
    GpioPin spiCommEnablePin;                ///< SPI communication enable pin
    GpioPin wakeControlPin;                  ///< Wake control output pin
};

/**
 * @brief Structure for TMC9660 ADC channel configuration.
 */
struct Tmc9660AdcConfig {
    AdcInputSensor ain1Sensor;               ///< AIN1 ADC sensor mapping
    AdcInputSensor ain2Sensor;               ///< AIN2 ADC sensor mapping
    AdcInputSensor ain3Sensor;               ///< AIN3 ADC sensor mapping
};

/**
 * @class Tmc9660MotorController
 * @brief Thread-safe singleton class for TMC9660 motor controller management.
 * 
 * This class provides a unified interface for multiple TMC9660 chips in the system,
 * supporting both SPI and UART communication with proper GPIO control and ADC integration.
 */
class Tmc9660MotorController {
public:
    /**
     * @brief Maximum number of TMC9660 chips supported.
     */
    static constexpr uint8_t MAX_TMC9660_CHIPS = static_cast<uint8_t>(Tmc9660ChipId::TMC9660_CHIP_COUNT);

    /**
     * @brief Get the singleton instance.
     * @return Reference to the singleton instance.
     */
    static Tmc9660MotorController& GetInstance() noexcept;

    // Prevent copy and assignment
    Tmc9660MotorController(const Tmc9660MotorController&) = delete;
    Tmc9660MotorController& operator=(const Tmc9660MotorController&) = delete;

    /**
     * @brief Initialize the TMC9660 controller system.
     * @return true if initialization successful, false otherwise.
     */
    bool Initialize() noexcept;

    /**
     * @brief Ensure the system is initialized.
     * @return true if system is ready, false otherwise.
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Register a TMC9660 chip with the controller.
     * @param chipId The chip identifier.
     * @param config The chip configuration.
     * @param gpioConfig The GPIO pin configuration.
     * @param adcConfig The ADC channel configuration.
     * @return true if registration successful, false otherwise.
     */
    bool RegisterTmc9660Chip(Tmc9660ChipId chipId, 
                            const Tmc9660Config& config,
                            const Tmc9660GpioConfig& gpioConfig,
                            const Tmc9660AdcConfig& adcConfig) noexcept;

    /**
     * @brief Get the number of registered TMC9660 chips.
     * @return Number of registered chips.
     */
    uint8_t GetRegisteredChipCount() const noexcept;

    /**
     * @brief Check if a specific TMC9660 chip is registered.
     * @param chipId The chip identifier.
     * @return true if chip is registered, false otherwise.
     */
    bool IsChipRegistered(Tmc9660ChipId chipId) const noexcept;

    /**
     * @brief Get the status of a TMC9660 chip.
     * @param chipId The chip identifier.
     * @param status Reference to store the status information.
     * @return true if status retrieved successfully, false otherwise.
     */
    bool GetChipStatus(Tmc9660ChipId chipId, Tmc9660Status& status) noexcept;

    /**
     * @brief Enable SPI communication for a TMC9660 chip.
     * @param chipId The chip identifier.
     * @return true if enabled successfully, false otherwise.
     */
    bool EnableSpiCommunication(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Disable SPI communication for a TMC9660 chip.
     * @param chipId The chip identifier.
     * @return true if disabled successfully, false otherwise.
     */
    bool DisableSpiCommunication(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Switch communication interface for a TMC9660 chip.
     * @param chipId The chip identifier.
     * @param newInterface The new communication interface.
     * @return true if switched successfully, false otherwise.
     */
    bool SwitchCommunicationInterface(Tmc9660ChipId chipId, Tmc9660CommInterface newInterface) noexcept;

    /**
     * @brief Enable the driver output for a TMC9660 chip.
     * @param chipId The chip identifier.
     * @return true if enabled successfully, false otherwise.
     */
    bool EnableDriver(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Disable the driver output for a TMC9660 chip.
     * @param chipId The chip identifier.
     * @return true if disabled successfully, false otherwise.
     */
    bool DisableDriver(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Reset a TMC9660 chip.
     * @param chipId The chip identifier.
     * @return true if reset successfully, false otherwise.
     */
    bool ResetChip(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Wake up a TMC9660 chip from sleep mode.
     * @param chipId The chip identifier.
     * @return true if woken up successfully, false otherwise.
     */
    bool WakeUpChip(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Put a TMC9660 chip into sleep mode.
     * @param chipId The chip identifier.
     * @return true if put to sleep successfully, false otherwise.
     */
    bool SleepChip(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Check if a TMC9660 chip has a fault condition.
     * @param chipId The chip identifier.
     * @return true if chip has fault, false if no fault or chip not found.
     */
    bool HasFault(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Clear fault condition for a TMC9660 chip.
     * @param chipId The chip identifier.
     * @return true if fault cleared successfully, false otherwise.
     */
    bool ClearFault(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Read ADC value from a TMC9660 chip.
     * @param chipId The chip identifier.
     * @param adcChannel The ADC channel (1, 2, or 3).
     * @param value Reference to store the ADC value.
     * @return true if read successfully, false otherwise.
     */
    bool ReadAdcValue(Tmc9660ChipId chipId, uint8_t adcChannel, uint32_t& value) noexcept;

    /**
     * @brief Read ADC voltage from a TMC9660 chip.
     * @param chipId The chip identifier.
     * @param adcChannel The ADC channel (1, 2, or 3).
     * @param voltage Reference to store the voltage value.
     * @return true if read successfully, false otherwise.
     */
    bool ReadAdcVoltage(Tmc9660ChipId chipId, uint8_t adcChannel, float& voltage) noexcept;

    /**
     * @brief Set the state of a TMC9660 GPIO pin.
     * @param chipId The chip identifier.
     * @param gpioNumber The GPIO number (17 or 18).
     * @param active Whether to set the pin active or inactive.
     * @return true if set successfully, false otherwise.
     */
    bool SetGpioState(Tmc9660ChipId chipId, uint8_t gpioNumber, bool active) noexcept;

    /**
     * @brief Get the state of a TMC9660 GPIO pin.
     * @param chipId The chip identifier.
     * @param gpioNumber The GPIO number (17 or 18).
     * @return true if pin is active, false if inactive or error.
     */
    bool GetGpioState(Tmc9660ChipId chipId, uint8_t gpioNumber) noexcept;

    /**
     * @brief Test communication with all registered TMC9660 chips.
     * @return true if all chips respond, false otherwise.
     */
    bool TestCommunication() noexcept;

    /**
     * @brief Run system diagnostics on all TMC9660 chips.
     * @return true if all chips pass diagnostics, false otherwise.
     */
    bool RunDiagnostics() noexcept;

    /**
     * @brief Get overall system health status.
     * @return true if system is healthy, false if issues detected.
     */
    bool GetSystemHealth() noexcept;

    /**
     * @brief Print status information for all registered chips.
     */
    void PrintSystemStatus() noexcept;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    Tmc9660MotorController() noexcept;

    /**
     * @brief Destructor.
     */
    ~Tmc9660MotorController() noexcept;

    /**
     * @brief Structure containing TMC9660 chip instance data.
     */
    struct Tmc9660ChipInstance {
        Tmc9660ChipId chipId;
        Tmc9660Config config;
        Tmc9660GpioConfig gpioConfig;
        Tmc9660AdcConfig adcConfig;
        Tmc9660Status status;
        std::unique_ptr<TMC9660> tmcDriver;  ///< TMC9660 driver instance
        bool isRegistered;
    };

    /**
     * @brief Find a TMC9660 chip instance by ID.
     * @param chipId The chip identifier.
     * @return Pointer to chip instance or nullptr if not found.
     */
    Tmc9660ChipInstance* FindChipInstance(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Find a TMC9660 chip instance by ID (const version).
     * @param chipId The chip identifier.
     * @return Pointer to chip instance or nullptr if not found.
     */
    const Tmc9660ChipInstance* FindChipInstance(Tmc9660ChipId chipId) const noexcept;

    /**
     * @brief Initialize GPIO pins for a TMC9660 chip.
     * @param instance Reference to the chip instance.
     * @return true if initialized successfully, false otherwise.
     */
    bool InitializeChipGpio(Tmc9660ChipInstance& instance) noexcept;

    /**
     * @brief Initialize ADC channels for a TMC9660 chip.
     * @param instance Reference to the chip instance.
     * @return true if initialized successfully, false otherwise.
     */
    bool InitializeChipAdc(Tmc9660ChipInstance& instance) noexcept;

    /**
     * @brief Update the status of a TMC9660 chip.
     * @param instance Reference to the chip instance.
     * @return true if status updated successfully, false otherwise.
     */
    bool UpdateChipStatus(Tmc9660ChipInstance& instance) noexcept;

    /**
     * @brief Perform initial configuration of a TMC9660 chip.
     * @param instance Reference to the chip instance.
     * @return true if configured successfully, false otherwise.
     */
    bool ConfigureChip(Tmc9660ChipInstance& instance) noexcept;

    // Member variables
    std::atomic<bool> initialized_;                                    ///< Initialization flag
    std::mutex systemMutex_;                                          ///< System-wide mutex
    std::array<Tmc9660ChipInstance, MAX_TMC9660_CHIPS> chipInstances_; ///< Chip instances
    uint8_t registeredChipCount_;                                     ///< Number of registered chips
    
    // System references
    AdcManager* adcSystem_;                                           ///< ADC manager system reference
    GpioManager* gpioSystem_;                                         ///< GPIO manager system reference
};

/**
 * @brief Convenience function to get the TMC9660 controller instance.
 * @return Reference to the TMC9660 controller singleton.
 */
inline Tmc9660MotorController& GetTmc9660Controller() noexcept {
    return Tmc9660MotorController::GetInstance();
}

/**
 * @brief Initialize the primary TMC9660 chip with default configuration.
 * This function sets up TMC9660_CHIP_1 with the hardware configuration described
 * in the requirements.
 * @return true if initialization successful, false otherwise.
 */
bool InitializePrimaryTmc9660() noexcept;

/**
 * @brief Quick health check for all TMC9660 chips.
 * @return true if all chips are healthy, false otherwise.
 */
bool Tmc9660SystemHealthy() noexcept;

#endif // COMPONENT_HANDLER_TMC9660_MOTOR_CONTROLLER_H_
