/**
 * @file Tmc9660Gpio.cpp
 * @brief Implementation of TMC9660 GPIO integration wrapper.
 *
 * This file implements the TMC9660 GPIO wrapper classes that integrate
 * TMC9660 GPIO pins with the unified GPIO data handler system.
 */

#include "Tmc9660Gpio.h"
#include "Tmc9660MotorController.h"
#include "GpioData.h"
#include "ThingsToString.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "Tmc9660Gpio";

//==============================================================================
// Tmc9660GpioPin Implementation
//==============================================================================

Tmc9660GpioPin::Tmc9660GpioPin(Tmc9660MotorController& controller, 
                               Tmc9660ChipId chipId,
                               uint8_t gpioNumber,
                               ActiveState activeState) noexcept
    : DigitalGpio(GPIO_NUM_NC, activeState), // TMC9660 GPIO doesn't use ESP32 pin
      controller_(controller),
      chipId_(chipId),
      gpioNumber_(gpioNumber)
{
    // Generate description for this pin
    snprintf(description_, sizeof(description_), 
             "TMC9660_%s_GPIO%u", 
             Tmc9660ChipIdToString(chipId_).data(),
             gpioNumber_);
}

bool Tmc9660GpioPin::Initialize() noexcept {
    if (IsInitialized()) {
        return true;
    }

    // Validate the GPIO number (TMC9660 has GPIO17 and GPIO18)
    if (gpioNumber_ != 17 && gpioNumber_ != 18) {
        return false;
    }

    // Validate the chip ID
    if (chipId_ >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        return false;
    }

    // The TMC9660 controller should already be initialized
    // We just mark this GPIO as initialized
    SetInitialized(true);
    return true;
}

bool Tmc9660GpioPin::SetActive() noexcept {
    if (!EnsureInitialized()) {
        return false;
    }

    return controller_.SetGpioState(chipId_, gpioNumber_, true);
}

bool Tmc9660GpioPin::SetInactive() noexcept {
    if (!EnsureInitialized()) {
        return false;
    }

    return controller_.SetGpioState(chipId_, gpioNumber_, false);
}

bool Tmc9660GpioPin::Toggle() noexcept {
    if (!EnsureInitialized()) {
        return false;
    }

    // Read current state and toggle it
    bool currentState = controller_.GetGpioState(chipId_, gpioNumber_);
    return controller_.SetGpioState(chipId_, gpioNumber_, !currentState);
}

bool Tmc9660GpioPin::IsActive() noexcept {
    if (!EnsureInitialized()) {
        return false;
    }

    bool gpioState = controller_.GetGpioState(chipId_, gpioNumber_);
    
    // Apply active state logic
    if (IsActiveHigh()) {
        return gpioState;
    } else {
        return !gpioState;
    }
}

DigitalGpio::State Tmc9660GpioPin::GetState() noexcept {
    if (IsActive()) {
        return State::Active;
    } else {
        return State::Inactive;
    }
}

HfGpioErr Tmc9660GpioPin::GetStateWithError(State& state) noexcept {
    HfGpioErr validation = ValidatePin();
    if (validation != HfGpioErr::GPIO_SUCCESS) {
        return validation;
    }

    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_NOT_INITIALIZED;
    }

    try {
        state = GetState();
        return HfGpioErr::GPIO_SUCCESS;
    } catch (...) {
        return HfGpioErr::GPIO_COMMUNICATION_ERROR;
    }
}

HfGpioErr Tmc9660GpioPin::SetStateWithError(State state) noexcept {
    HfGpioErr validation = ValidatePin();
    if (validation != HfGpioErr::GPIO_SUCCESS) {
        return validation;
    }

    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_NOT_INITIALIZED;
    }

    bool success = false;
    try {
        if (state == State::Active) {
            success = SetActive();
        } else {
            success = SetInactive();
        }
    } catch (...) {
        return HfGpioErr::GPIO_COMMUNICATION_ERROR;
    }

    return success ? HfGpioErr::GPIO_SUCCESS : HfGpioErr::GPIO_OPERATION_FAILED;
}

HfGpioErr Tmc9660GpioPin::ToggleWithError() noexcept {
    HfGpioErr validation = ValidatePin();
    if (validation != HfGpioErr::GPIO_SUCCESS) {
        return validation;
    }

    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_NOT_INITIALIZED;
    }

    try {
        bool success = Toggle();
        return success ? HfGpioErr::GPIO_SUCCESS : HfGpioErr::GPIO_OPERATION_FAILED;
    } catch (...) {
        return HfGpioErr::GPIO_COMMUNICATION_ERROR;
    }
}

HfGpioErr Tmc9660GpioPin::ValidatePin() noexcept {
    // Check if GPIO number is valid for TMC9660
    if (gpioNumber_ != 17 && gpioNumber_ != 18) {
        return HfGpioErr::GPIO_INVALID_PIN;
    }

    // Check if chip ID is valid
    if (chipId_ >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        return HfGpioErr::GPIO_INVALID_CHIP;
    }

    return HfGpioErr::GPIO_SUCCESS;
}

BaseGpio::Direction Tmc9660GpioPin::GetDirection() const noexcept {
    // TMC9660 GPIO pins are typically configured as outputs
    return Direction::Output;
}

const char* Tmc9660GpioPin::GetDescription() const noexcept {
    return description_;
}

bool Tmc9660GpioPin::IsValidTmc9660Gpio() const noexcept {
    return (gpioNumber_ == 17 || gpioNumber_ == 18) && 
           (chipId_ < Tmc9660ChipId::TMC9660_CHIP_COUNT);
}

//==============================================================================
// Tmc9660GpioManager Implementation
//==============================================================================

Tmc9660GpioManager::Tmc9660GpioManager(Tmc9660MotorController& controller) noexcept
    : controller_(controller),
      registeredPinCount_(0),
      initialized_(false)
{
    // Initialize array to nullptr
    for (uint8_t i = 0; i < MAX_TMC9660_GPIOS; ++i) {
        gpioPins_[i] = nullptr;
    }
}

Tmc9660GpioManager::~Tmc9660GpioManager() noexcept {
    // Smart pointers will automatically clean up
}

bool Tmc9660GpioManager::Initialize() noexcept {
    if (initialized_) {
        return true;
    }

    // Create all GPIO pin instances
    if (!CreateGpioPins()) {
        return false;
    }

    // Initialize all GPIO pins
    for (uint8_t i = 0; i < MAX_TMC9660_GPIOS; ++i) {
        if (gpioPins_[i] && !gpioPins_[i]->Initialize()) {
            return false;
        }
    }

    initialized_ = true;
    return true;
}

bool Tmc9660GpioManager::RegisterAllPins() noexcept {
    if (!initialized_ && !Initialize()) {
        return false;
    }

    // Register each chip's pins
    bool success = true;
    for (uint8_t chipIndex = 0; chipIndex < MAX_TMC9660_CHIPS; ++chipIndex) {
        Tmc9660ChipId chipId = static_cast<Tmc9660ChipId>(chipIndex);
        success &= RegisterChipPins(chipId);
    }

    return success;
}

Tmc9660GpioPin* Tmc9660GpioManager::GetPin(Tmc9660ChipId chipId, uint8_t gpioNumber) noexcept {
    if (chipId >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        return nullptr;
    }

    if (gpioNumber != 17 && gpioNumber != 18) {
        return nullptr;
    }

    // Calculate array index
    uint8_t chipIndex = static_cast<uint8_t>(chipId);
    uint8_t gpioIndex = (gpioNumber == 17) ? 0 : 1;  // GPIO17=0, GPIO18=1
    uint8_t arrayIndex = chipIndex * TMC9660_GPIOS_PER_CHIP + gpioIndex;

    if (arrayIndex < MAX_TMC9660_GPIOS) {
        return gpioPins_[arrayIndex].get();
    }

    return nullptr;
}

bool Tmc9660GpioManager::CreateGpioPins() noexcept {
    uint8_t arrayIndex = 0;

    // Create GPIO pins for each chip
    for (uint8_t chipIndex = 0; chipIndex < MAX_TMC9660_CHIPS; ++chipIndex) {
        Tmc9660ChipId chipId = static_cast<Tmc9660ChipId>(chipIndex);

        // Create GPIO17 and GPIO18 for this chip
        uint8_t gpioNumbers[] = {17, 18};
        for (uint8_t gpioIdx = 0; gpioIdx < 2; ++gpioIdx) {
            if (arrayIndex >= MAX_TMC9660_GPIOS) {
                return false;
            }

            try {
                gpioPins_[arrayIndex] = std::make_unique<Tmc9660GpioPin>(
                    controller_,
                    chipId,
                    gpioNumbers[gpioIdx],
                    ActiveState::High  // Default to high active
                );
                arrayIndex++;
            } catch (...) {
                return false;
            }
        }
    }

    return true;
}

bool Tmc9660GpioManager::RegisterChipPins(Tmc9660ChipId chipId) noexcept {
    GpioData& gpioData = GpioData::GetInstance();
    bool success = true;

    // Register GPIO17 and GPIO18 for this chip
    uint8_t gpioNumbers[] = {17, 18};
    for (uint8_t gpioIdx = 0; gpioIdx < 2; ++gpioIdx) {
        uint8_t gpioNumber = gpioNumbers[gpioIdx];
        Tmc9660GpioPin* pin = GetPin(chipId, gpioNumber);
        
        if (pin) {
            // Map to GpioPin enum
            GpioPin mappedPin = MapToGpioPin(chipId, gpioNumber);
            
            // Generate pin name
            char pinName[32];
            GeneratePinName(chipId, gpioNumber, pinName, sizeof(pinName));
            
            // Register with GPIO data system
            if (gpioData.RegisterGpioPin(mappedPin, *pin, pinName)) {
                registeredPinCount_++;
            } else {
                success = false;
            }
        } else {
            success = false;
        }
    }

    return success;
}

GpioPin Tmc9660GpioManager::MapToGpioPin(Tmc9660ChipId chipId, uint8_t gpioNumber) noexcept {
    // Map TMC9660 chips and GPIO numbers to GpioPin enum values
    if (chipId == Tmc9660ChipId::TMC9660_CHIP_1) {
        if (gpioNumber == 17) {
            return GpioPin::GPIO_TMC9660_CHIP1_GPIO17;
        } else if (gpioNumber == 18) {
            return GpioPin::GPIO_TMC9660_CHIP1_GPIO18;
        }
    } else if (chipId == Tmc9660ChipId::TMC9660_CHIP_2) {
        if (gpioNumber == 17) {
            return GpioPin::GPIO_TMC9660_CHIP2_GPIO17;
        } else if (gpioNumber == 18) {
            return GpioPin::GPIO_TMC9660_CHIP2_GPIO18;
        }
    }

    // Default fallback
    return GpioPin::GPIO_ESP32_PIN_0;
}

size_t Tmc9660GpioManager::GeneratePinName(Tmc9660ChipId chipId, uint8_t gpioNumber, 
                                          char* buffer, size_t bufferSize) noexcept {
    if (!buffer || bufferSize == 0) {
        return 0;
    }

    int result = snprintf(buffer, bufferSize, "tmc9660_%s_gpio%u",
                         Tmc9660ChipIdToString(chipId).data(), gpioNumber);
    
    if (result < 0 || static_cast<size_t>(result) >= bufferSize) {
        // Truncation occurred or error
        buffer[bufferSize - 1] = '\0';
        return bufferSize - 1;
    }

    return static_cast<size_t>(result);
}
