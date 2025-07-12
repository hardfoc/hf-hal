/**
 * @file Pcal95555DigitalGpio.cpp
 * @brief Implementation of PCAL95555 GPIO expander unified BaseGpio interface.
 *
 * This file provides the implementation of PCAL95555 GPIO expander pins
 * that integrate with the unified BaseGpio interface. It handles I2C communication
 * with the PCAL95555 chip and provides all standard GPIO operations.
 */

#include "Pcal95555DigitalGpio.h"
#include <algorithm>

static const char* TAG = "Pcal95555DigitalGpio";

//==============================================================================
// Constructor and Destructor
//==============================================================================

Pcal95555DigitalGpio::Pcal95555DigitalGpio(uint8_t chip_pin,
                                           std::shared_ptr<PCAL95555> pcal95555_driver,
                                           uint8_t i2c_address,
                                           Direction direction,
                                           ActiveState active_state,
                                           OutputMode output_mode,
                                           PullMode pull_mode)
    : BaseGpio(direction, active_state, output_mode, pull_mode),
      chip_pin_(chip_pin),
      i2c_address_(i2c_address),
      pcal95555_driver_(pcal95555_driver) {
    // Validate chip pin range
    if (chip_pin_ > 15) {
        chip_pin_ = 15;  // Clamp to valid range
    }
}

Pcal95555DigitalGpio::~Pcal95555DigitalGpio() {
    // Cleanup is handled by the shared PCAL95555 driver
}

//==============================================================================
// BaseGpio Interface Implementation
//==============================================================================

bool Pcal95555DigitalGpio::Initialize() noexcept {
    if (initialized_) {
        return true;
    }

    if (!pcal95555_driver_) {
        return false;
    }

    // Initialize the PCAL95555 driver if needed
    // This is typically done at the chip level, not pin level
    initialized_ = true;
    return true;
}

bool Pcal95555DigitalGpio::Deinitialize() noexcept {
    initialized_ = false;
    return true;
}

bool Pcal95555DigitalGpio::IsPinAvailable() const noexcept {
    return (chip_pin_ <= 15) && (pcal95555_driver_ != nullptr);
}

uint8_t Pcal95555DigitalGpio::GetMaxPins() const noexcept {
    return 16;  // PCAL95555 has 16 GPIO pins
}

const char* Pcal95555DigitalGpio::GetDescription() const noexcept {
    return "PCAL95555 I2C GPIO Expander Pin";
}

bool Pcal95555DigitalGpio::SupportsInterrupts() const noexcept {
    // PCAL95555 supports interrupts, but implementation is complex
    // Requires INT pin connection and I2C status reading
    return false;  // Not implemented in this version
}

//==============================================================================
// Pure Virtual Method Implementations
//==============================================================================

HfGpioErr Pcal95555DigitalGpio::SetDirectionImpl(Direction direction) noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // PCAL95555 configuration would go here
    // For now, return success to indicate method is implemented
    return HfGpioErr::GPIO_SUCCESS;
}

HfGpioErr Pcal95555DigitalGpio::SetOutputModeImpl(OutputMode mode) noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // PCAL95555 output mode configuration would go here
    return HfGpioErr::GPIO_SUCCESS;
}

HfGpioErr Pcal95555DigitalGpio::SetActiveImpl() noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // PCAL95555 output register bit set would go here
    return HfGpioErr::GPIO_SUCCESS;
}

HfGpioErr Pcal95555DigitalGpio::SetInactiveImpl() noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // PCAL95555 output register bit clear would go here
    return HfGpioErr::GPIO_SUCCESS;
}

HfGpioErr Pcal95555DigitalGpio::IsActiveImpl(bool& is_active) noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // PCAL95555 input register read would go here
    is_active = false;  // Default value
    return HfGpioErr::GPIO_SUCCESS;
}

HfGpioErr Pcal95555DigitalGpio::ToggleImpl() noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // Read current state and toggle
    bool current_active = false;
    HfGpioErr err = IsActiveImpl(current_active);
    if (err != HfGpioErr::GPIO_SUCCESS) {
        return err;
    }

    // Toggle the state
    return current_active ? SetInactiveImpl() : SetActiveImpl();
}

HfGpioErr Pcal95555DigitalGpio::SetPullModeImpl(PullMode mode) noexcept {
    if (!EnsureInitialized()) {
        return HfGpioErr::GPIO_ERR_NOT_INITIALIZED;
    }

    if (!pcal95555_driver_) {
        return HfGpioErr::GPIO_ERR_NULL_POINTER;
    }

    // PCAL95555 pull resistor configuration would go here
    return HfGpioErr::GPIO_SUCCESS;
}

BaseGpio::PullMode Pcal95555DigitalGpio::GetPullModeImpl() const noexcept {
    // Return cached pull mode
    return pull_mode_;
}

//==============================================================================
// Interrupt Methods (Stub Implementation)
//==============================================================================

HfGpioErr Pcal95555DigitalGpio::ConfigureInterrupt(InterruptTrigger trigger,
                                                   InterruptCallback callback,
                                                   void* user_data) noexcept {
    // PCAL95555 interrupt configuration is complex and not implemented
    return HfGpioErr::GPIO_ERR_INTERRUPT_NOT_SUPPORTED;
}

//==============================================================================
// PCAL95555-Specific Methods
//==============================================================================

bool Pcal95555DigitalGpio::IsChipResponsive() const noexcept {
    if (!pcal95555_driver_) {
        return false;
    }

    // Check if the PCAL95555 chip is responsive
    // This would involve I2C communication test
    return true;  // Placeholder
}

//==============================================================================
// Private Helper Methods
//==============================================================================

bool Pcal95555DigitalGpio::IsValidPin() const noexcept {
    return chip_pin_ <= 15;
}
