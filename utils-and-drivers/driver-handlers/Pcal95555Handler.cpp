#include "Pcal95555Handler.h"
#include <cstring>
#include <vector>

// ================= Pcal95555I2cAdapter =================
bool Pcal95555I2cAdapter::write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
    MutexLockGuard lock(i2c_mutex_);
    
    // Address validation: Ensures the PCAL95555 driver and BaseI2c device are consistent
    // This prevents accidental cross-device communication and provides type safety
    if (addr != i2c_device_.GetDeviceAddress()) {
        // Log error or handle mismatch - this indicates a configuration error
        return false;
    }
    
    // Create register write command: [register_address, data...]
    std::vector<uint8_t> command;
    command.reserve(1 + len);
    command.push_back(reg);
    command.insert(command.end(), data, data + len);
    
    // Use device-centric BaseI2c interface (no address parameter needed)
    return i2c_device_.Write(command.data(), command.size()) == hf_i2c_err_t::I2C_SUCCESS;
}

bool Pcal95555I2cAdapter::read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    MutexLockGuard lock(i2c_mutex_);
    
    // Address validation: Ensures the PCAL95555 driver and BaseI2c device are consistent
    if (addr != i2c_device_.GetDeviceAddress()) {
        return false;
    }
    
    // Use device-centric BaseI2c interface for register read
    return i2c_device_.WriteRead(&reg, 1, data, len) == hf_i2c_err_t::I2C_SUCCESS;
}

// ================= Pcal95555Handler =================
Pcal95555Handler::Pcal95555Handler(BaseI2c& i2c_device) noexcept
    : i2c_adapter_(std::make_unique<Pcal95555I2cAdapter>(i2c_device)),
      pcal95555_driver_(std::make_shared<PCAL95555>(i2c_adapter_.get(), i2c_device.GetDeviceAddress())) { }

hf_bool_t Pcal95555Handler::Initialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (initialized_) return true;
    initialized_ = pcal95555_driver_ && pcal95555_driver_->init();
    return initialized_;
}

hf_bool_t Pcal95555Handler::Deinitialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    initialized_ = false;
    return true;
}

hf_gpio_err_t Pcal95555Handler::SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT)
        return pcal95555_driver_->setPinDirection(pin, false); // false = output
    else
        return pcal95555_driver_->setPinDirection(pin, true); // true = input
}

hf_gpio_err_t Pcal95555Handler::SetOutput(hf_u8_t pin, hf_bool_t active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return pcal95555_driver_->writePin(pin, active);
}

hf_gpio_err_t Pcal95555Handler::ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->readPin(pin, value);
    if (ok) active = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::Toggle(hf_u8_t pin) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    if (!pcal95555_driver_->readPin(pin, value)) return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    return pcal95555_driver_->writePin(pin, !value);
}

hf_gpio_err_t Pcal95555Handler::SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    switch (pull_mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            return pcal95555_driver_->setPinPullUp(pin, false) && pcal95555_driver_->setPinPullDown(pin, false);
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            return pcal95555_driver_->setPinPullUp(pin, true) && pcal95555_driver_->setPinPullDown(pin, false);
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            return pcal95555_driver_->setPinPullUp(pin, false) && pcal95555_driver_->setPinPullDown(pin, true);
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            return pcal95555_driver_->setPinPullUp(pin, true) && pcal95555_driver_->setPinPullDown(pin, true);
        default:
            return hf_gpio_err_t::GPIO_ERR_INVALID_ARGUMENT;
    }
}

hf_gpio_err_t Pcal95555Handler::GetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t pullup = false, pulldown = false;
    hf_bool_t ok1 = pcal95555_driver_->getPinPullUp(pin, pullup);
    hf_bool_t ok2 = pcal95555_driver_->getPinPullDown(pin, pulldown);
    if (!ok1 || !ok2) return hf_gpio_err_t::GPIO_ERR_FAILURE;
    if (pullup && !pulldown) pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP;
    else if (!pullup && pulldown) pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
    else if (pullup && pulldown) pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN;
    else pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    return hf_gpio_err_t::GPIO_SUCCESS;
} 

// ===================== Pcal95555GpioPin Implementation ===================== //
Pcal95555GpioPin::Pcal95555GpioPin(
    hf_pin_num_t pin,
    std::shared_ptr<PCAL95555> driver,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode) noexcept
    : BaseGpio(pin, direction, active_state, output_mode, pull_mode),
      pin_(pin), driver_(std::move(driver)) {
    snprintf(description_, sizeof(description_), "PCAL95555_PIN_%d", static_cast<int>(pin_));
}

hf_bool_t Pcal95555GpioPin::Initialize() noexcept {
    MutexLockGuard lock(pin_mutex_);
    hf_bool_t ok = driver_ && driver_->setPinDirection(static_cast<hf_u8_t>(pin_), current_direction_ == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    if (ok) {
        switch (pull_mode_) {
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
                break;
        }
    }
    initialized_ = ok;
    return ok;
}

hf_bool_t Pcal95555GpioPin::Deinitialize() noexcept {
    MutexLockGuard lock(pin_mutex_);
    initialized_ = false;
    return true;
}

hf_bool_t Pcal95555GpioPin::IsPinAvailable() const noexcept {
    return driver_ && pin_ >= 0 && pin_ < 16;
}

const char* Pcal95555GpioPin::GetDescription() const noexcept {
    return description_;
}

hf_bool_t Pcal95555GpioPin::SupportsInterrupts() const noexcept {
    return true;
}

hf_gpio_err_t Pcal95555GpioPin::ConfigureInterrupt(hf_gpio_interrupt_trigger_t /*trigger*/, InterruptCallback /*callback*/, void* /*user_data*/) noexcept {
    return hf_gpio_err_t::GPIO_ERR_NOT_SUPPORTED;
}

hf_gpio_err_t Pcal95555GpioPin::SetPolarityInversion(hf_bool_t invert) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinPolarity(static_cast<hf_u8_t>(pin_), invert);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetPolarityInversion(hf_bool_t& invert) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinPolarity(static_cast<hf_u8_t>(pin_), value);
    if (ok) invert = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetInterruptMask(hf_bool_t mask) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinInterruptMask(static_cast<hf_u8_t>(pin_), mask);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptMask(hf_bool_t& mask) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinInterruptMask(static_cast<hf_u8_t>(pin_), value);
    if (ok) mask = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptStatus(hf_bool_t& status) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinInterruptStatus(static_cast<hf_u8_t>(pin_), value);
    if (ok) status = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetDirectionImpl(hf_gpio_direction_t direction) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinDirection(static_cast<hf_u8_t>(pin_), direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetOutputModeImpl(hf_gpio_output_mode_t /*mode*/) noexcept {
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555GpioPin::SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = true;
    switch (mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
            break;
    }
    pull_mode_ = mode;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_pull_mode_t Pcal95555GpioPin::GetPullModeImpl() const noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    hf_bool_t pullup = false, pulldown = false;
    driver_->getPinPullUp(static_cast<hf_u8_t>(pin_), pullup);
    driver_->getPinPullDown(static_cast<hf_u8_t>(pin_), pulldown);
    if (pullup && pulldown) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN;
    if (pullup) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP;
    if (pulldown) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

hf_gpio_err_t Pcal95555GpioPin::SetPinLevelImpl(hf_gpio_level_t level) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    hf_bool_t hardware_level = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    hf_bool_t ok = driver_->writePin(static_cast<hf_u8_t>(pin_), hardware_level);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetPinLevelImpl(hf_gpio_level_t& level) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    hf_bool_t value = false;
    hf_bool_t ok = driver_->readPin(static_cast<hf_u8_t>(pin_), value);
    if (ok) {
        level = value ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    hf_bool_t is_output = false;
    hf_bool_t ok = driver_->getPinDirection(static_cast<hf_u8_t>(pin_), is_output);
    if (ok) {
        direction = is_output ? hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT 
                             : hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    // PCAL95555 only supports push-pull output mode
    mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

// ===================== Handler Factory Method ===================== //
std::unique_ptr<BaseGpio> Pcal95555Handler::CreateGpioPin(
    hf_pin_num_t pin,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode) noexcept {
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return nullptr;
    return std::make_unique<Pcal95555GpioPin>(pin, pcal95555_driver_, direction, active_state, output_mode, pull_mode);
} 

// ===================== Pcal95555Handler Advanced Features ===================== //
hf_gpio_err_t Pcal95555Handler::SetPolarityInversion(hf_pin_num_t pin, hf_bool_t invert) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    return pcal95555_driver_->setPinPolarity(static_cast<hf_u8_t>(pin), invert);
}

hf_gpio_err_t Pcal95555Handler::GetPolarityInversion(hf_pin_num_t pin, hf_bool_t& invert) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->getPinPolarity(static_cast<hf_u8_t>(pin), value);
    if (ok) invert = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetInterruptMask(hf_pin_num_t pin, hf_bool_t mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    return pcal95555_driver_->setPinInterruptMask(static_cast<hf_u8_t>(pin), mask);
}

hf_gpio_err_t Pcal95555Handler::GetInterruptMask(hf_pin_num_t pin, hf_bool_t& mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->getPinInterruptMask(static_cast<hf_u8_t>(pin), value);
    if (ok) mask = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetInterruptStatus(hf_pin_num_t pin, hf_bool_t& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->getPinInterruptStatus(static_cast<hf_u8_t>(pin), value);
    if (ok) status = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool is_input = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    bool ok = pcal95555_driver_->setPinsDirection(pin_mask, is_input);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetOutputs(uint16_t pin_mask, hf_bool_t active) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->writePins(pin_mask, active);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = true;
    switch (pull_mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, false) && pcal95555_driver_->setPinsPullDown(pin_mask, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, true) && pcal95555_driver_->setPinsPullDown(pin_mask, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, false) && pcal95555_driver_->setPinsPullDown(pin_mask, true);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, true) && pcal95555_driver_->setPinsPullDown(pin_mask, true);
            break;
        default:
            ok = false;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptMasks(uint16_t& mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->getAllInterruptMasks(mask);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptStatus(uint16_t& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->getAllInterruptStatus(status);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_bool_t Pcal95555Handler::SoftwareReset() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!pcal95555_driver_) return false;
    return pcal95555_driver_->softwareReset();
}

hf_bool_t Pcal95555Handler::PowerDown() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!pcal95555_driver_) return false;
    return pcal95555_driver_->powerDown();
} 
