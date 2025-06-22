#ifndef COMPONENT_HANDLER_TMC9660GPIO_H_
#define COMPONENT_HANDLER_TMC9660GPIO_H_

/**
 * @file Tmc9660Gpio.h
 * @brief TMC9660 GPIO integration wrapper for the unified GPIO system.
 *
 * This file provides wrapper classes to integrate TMC9660 GPIO pins with the
 * unified GPIO data handler system. The TMC9660 motor controller has GPIO
 * pins that can be controlled via SPI registers.
 */

#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/DigitalGpio.h"
#include "CommonIDs.h"
#include <memory>

// Forward declarations
class Tmc9660MotorController;

/**
 * @class Tmc9660GpioPin
 * @brief Represents a single TMC9660 GPIO pin as a DigitalGpio instance.
 *
 * This class wraps a TMC9660 GPIO pin and provides the standard DigitalGpio
 * interface for use with the unified GPIO system.
 */
class Tmc9660GpioPin : public DigitalGpio {
public:
    /**
     * @brief Constructor for TMC9660 GPIO pin.
     * @param controller Reference to the TMC9660 controller instance.
     * @param chipId The TMC9660 chip identifier.
     * @param gpioNumber The GPIO pin number on the TMC9660 (17 or 18).
     * @param activeState The active state (high or low).
     */
    Tmc9660GpioPin(Tmc9660MotorController& controller, 
                   Tmc9660ChipId chipId,
                   uint8_t gpioNumber,
                   ActiveState activeState) noexcept;

    /**
     * @brief Destructor.
     */
    virtual ~Tmc9660GpioPin() noexcept = default;

    // Prevent copy and assignment
    Tmc9660GpioPin(const Tmc9660GpioPin&) = delete;
    Tmc9660GpioPin& operator=(const Tmc9660GpioPin&) = delete;

    //==========================================================================
    // DigitalGpio Interface Implementation
    //==========================================================================

    /**
     * @brief Initialize the TMC9660 GPIO pin.
     * @return true if initialization successful, false otherwise.
     */
    bool Initialize() noexcept override;

    /**
     * @brief Set the pin to active state.
     * @return true if successful, false otherwise.
     */
    bool SetActive() noexcept override;

    /**
     * @brief Set the pin to inactive state.
     * @return true if successful, false otherwise.
     */
    bool SetInactive() noexcept override;

    /**
     * @brief Toggle the pin state.
     * @return true if successful, false otherwise.
     */
    bool Toggle() noexcept override;

    /**
     * @brief Check if the pin is in active state.
     * @return true if active, false otherwise.
     */
    bool IsActive() noexcept override;

    /**
     * @brief Get the current state of the pin.
     * @return Current pin state.
     */
    State GetState() noexcept override;

    //==========================================================================
    // New BaseGpio Interface Implementation
    //==========================================================================

    /**
     * @brief Get error-based pin state.
     * @param state Reference to store the pin state.
     * @return Error code indicating success or failure.
     */
    HfGpioErr GetStateWithError(State& state) noexcept override;

    /**
     * @brief Set pin state with error handling.
     * @param state The desired pin state.
     * @return Error code indicating success or failure.
     */
    HfGpioErr SetStateWithError(State state) noexcept override;

    /**
     * @brief Toggle pin state with error handling.
     * @return Error code indicating success or failure.
     */
    HfGpioErr ToggleWithError() noexcept override;

    /**
     * @brief Validate the pin configuration.
     * @return Error code indicating validation result.
     */
    HfGpioErr ValidatePin() noexcept override;

    /**
     * @brief Get the pin direction.
     * @return Pin direction (always output for TMC9660).
     */
    Direction GetDirection() const noexcept override;

    /**
     * @brief Get a description of this pin.
     * @return String description of the pin.
     */
    const char* GetDescription() const noexcept override;

    //==========================================================================
    // TMC9660-Specific Methods
    //==========================================================================

    /**
     * @brief Get the TMC9660 chip ID.
     * @return The chip identifier.
     */
    Tmc9660ChipId GetChipId() const noexcept { return chipId_; }

    /**
     * @brief Get the GPIO pin number on the TMC9660.
     * @return The GPIO pin number (17 or 18).
     */
    uint8_t GetGpioNumber() const noexcept { return gpioNumber_; }

    /**
     * @brief Check if this pin represents a valid TMC9660 GPIO.
     * @return true if valid, false otherwise.
     */
    bool IsValidTmc9660Gpio() const noexcept;

private:
    Tmc9660MotorController& controller_;  ///< Reference to TMC9660 controller
    Tmc9660ChipId chipId_;               ///< TMC9660 chip identifier
    uint8_t gpioNumber_;                 ///< GPIO pin number (17 or 18)
    mutable char description_[64];       ///< Pin description buffer
};

/**
 * @class Tmc9660GpioManager
 * @brief Manager for all TMC9660 GPIO pins in the system.
 *
 * This class manages the creation and registration of TMC9660 GPIO pins
 * with the unified GPIO system.
 */
class Tmc9660GpioManager {
public:
    /**
     * @brief Constructor.
     * @param controller Reference to the TMC9660 controller.
     */
    explicit Tmc9660GpioManager(Tmc9660MotorController& controller) noexcept;

    /**
     * @brief Destructor.
     */
    ~Tmc9660GpioManager() noexcept;

    // Prevent copy and assignment
    Tmc9660GpioManager(const Tmc9660GpioManager&) = delete;
    Tmc9660GpioManager& operator=(const Tmc9660GpioManager&) = delete;

    /**
     * @brief Initialize all TMC9660 GPIO pins.
     * @return true if all pins initialized successfully, false otherwise.
     */
    bool Initialize() noexcept;

    /**
     * @brief Register all TMC9660 GPIO pins with the GPIO data system.
     * @return true if all pins registered successfully, false otherwise.
     */
    bool RegisterAllPins() noexcept;

    /**
     * @brief Get a TMC9660 GPIO pin by chip and pin number.
     * @param chipId The TMC9660 chip identifier.
     * @param gpioNumber The GPIO pin number (17 or 18).
     * @return Pointer to the GPIO pin or nullptr if not found.
     */
    Tmc9660GpioPin* GetPin(Tmc9660ChipId chipId, uint8_t gpioNumber) noexcept;

    /**
     * @brief Get the number of registered TMC9660 GPIO pins.
     * @return Number of registered pins.
     */
    uint8_t GetRegisteredPinCount() const noexcept { return registeredPinCount_; }

private:
    /**
     * @brief Create GPIO pins for all TMC9660 chips.
     * @return true if successful, false otherwise.
     */
    bool CreateGpioPins() noexcept;

    /**
     * @brief Register a single TMC9660 chip's GPIO pins.
     * @param chipId The chip identifier.
     * @return true if successful, false otherwise.
     */
    bool RegisterChipPins(Tmc9660ChipId chipId) noexcept;

    /**
     * @brief Map TMC9660 chip and GPIO to GpioPin enum.
     * @param chipId The TMC9660 chip identifier.
     * @param gpioNumber The GPIO pin number.
     * @return Mapped GpioPin enum value.
     */
    GpioPin MapToGpioPin(Tmc9660ChipId chipId, uint8_t gpioNumber) noexcept;

    /**
     * @brief Generate a name for a TMC9660 GPIO pin.
     * @param chipId The chip identifier.
     * @param gpioNumber The GPIO pin number.
     * @param buffer Buffer to store the name.
     * @param bufferSize Size of the buffer.
     * @return Length of the generated name.
     */
    size_t GeneratePinName(Tmc9660ChipId chipId, uint8_t gpioNumber, 
                          char* buffer, size_t bufferSize) noexcept;

    static constexpr uint8_t MAX_TMC9660_CHIPS = 2;  ///< Maximum number of TMC9660 chips
    static constexpr uint8_t TMC9660_GPIOS_PER_CHIP = 2;  ///< GPIO pins per TMC9660 chip
    static constexpr uint8_t MAX_TMC9660_GPIOS = MAX_TMC9660_CHIPS * TMC9660_GPIOS_PER_CHIP;

    Tmc9660MotorController& controller_;            ///< Reference to TMC9660 controller
    std::unique_ptr<Tmc9660GpioPin> gpioPins_[MAX_TMC9660_GPIOS];  ///< GPIO pin instances
    uint8_t registeredPinCount_;                    ///< Number of registered pins
    bool initialized_;                              ///< Initialization status
};

#endif // COMPONENT_HANDLER_TMC9660GPIO_H_
