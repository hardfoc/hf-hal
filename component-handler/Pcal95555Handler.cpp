#include "Pcal95555Handler.h"
#include <cstring>

// ================= Pcal95555I2cAdapter =================
hf_bool_t Pcal95555I2cAdapter::write(hf_u8_t /*addr*/, hf_u8_t reg, const hf_u8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    return i2c_bus_.WriteRegister(i2c_address_, reg, data, len);
}

hf_bool_t Pcal95555I2cAdapter::read(hf_u8_t /*addr*/, hf_u8_t reg, hf_u8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    return i2c_bus_.ReadRegister(i2c_address_, reg, data, len);
}

// ================= Pcal95555Handler =================
Pcal95555Handler::Pcal95555Handler(BaseI2c& i2c_bus, hf_u8_t i2c_address) noexcept
    : i2c_adapter_(std::make_unique<Pcal95555I2cAdapter>(i2c_bus, i2c_address)),
      pcal95555_driver_(std::make_shared<PCAL95555>(i2c_adapter_.get())),
      i2c_address_(i2c_address) {}

hf_bool_t Pcal95555Handler::Initialize() noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (initialized_) return true;
    initialized_ = pcal95555_driver_ && pcal95555_driver_->init();
    return initialized_;
}

hf_bool_t Pcal95555Handler::Deinitialize() noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    initialized_ = false;
    return true;
}

hf_gpio_err_t Pcal95555Handler::SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT)
        return pcal95555_driver_->setPinDirection(pin, false); // false = output
    else
        return pcal95555_driver_->setPinDirection(pin, true); // true = input
}

hf_gpio_err_t Pcal95555Handler::SetOutput(hf_u8_t pin, hf_bool_t active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return pcal95555_driver_->writePin(pin, active);
}

hf_gpio_err_t Pcal95555Handler::ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->readPin(pin, value);
    if (ok) active = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::Toggle(hf_u8_t pin) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    if (!pcal95555_driver_->readPin(pin, value)) return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    return pcal95555_driver_->writePin(pin, !value);
}

hf_gpio_err_t Pcal95555Handler::SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    std::lock_guard<std::mutex> lock(handler_mutex_);
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
    std::lock_guard<std::mutex> lock(handler_mutex_);
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
    std::lock_guard<std::mutex> lock(pin_mutex_);
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
    std::lock_guard<std::mutex> lock(pin_mutex_);
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
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinPolarity(static_cast<hf_u8_t>(pin_), invert);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetPolarityInversion(hf_bool_t& invert) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinPolarity(static_cast<hf_u8_t>(pin_), value);
    if (ok) invert = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetInterruptMask(hf_bool_t mask) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinInterruptMask(static_cast<hf_u8_t>(pin_), mask);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptMask(hf_bool_t& mask) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinInterruptMask(static_cast<hf_u8_t>(pin_), value);
    if (ok) mask = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptStatus(hf_bool_t& status) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinInterruptStatus(static_cast<hf_u8_t>(pin_), value);
    if (ok) status = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetDirectionImpl(hf_gpio_direction_t direction) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinDirection(static_cast<hf_u8_t>(pin_), direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetOutputModeImpl(hf_gpio_output_mode_t /*mode*/) noexcept {
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555GpioPin::SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
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
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    hf_bool_t pullup = false, pulldown = false;
    driver_->getPinPullUp(static_cast<hf_u8_t>(pin_), pullup);
    driver_->getPinPullDown(static_cast<hf_u8_t>(pin_), pulldown);
    if (pullup && pulldown) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN;
    if (pullup) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP;
    if (pulldown) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

hf_gpio_err_t Pcal95555GpioPin::SetActiveImpl() noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->writePin(static_cast<hf_u8_t>(pin_), active_state_ == hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH);
    current_state_ = hf_gpio_state_t::HF_GPIO_STATE_ACTIVE;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetInactiveImpl() noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->writePin(static_cast<hf_u8_t>(pin_), active_state_ == hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
    current_state_ = hf_gpio_state_t::HF_GPIO_STATE_INACTIVE;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::IsActiveImpl(hf_bool_t& is_active) noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->readPin(static_cast<hf_u8_t>(pin_), value);
    if (ok) {
        is_active = (active_state_ == hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH) ? value : !value;
        current_state_ = is_active ? hf_gpio_state_t::HF_GPIO_STATE_ACTIVE : hf_gpio_state_t::HF_GPIO_STATE_INACTIVE;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::ToggleImpl() noexcept {
    std::lock_guard<std::mutex> lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    if (!driver_->readPin(static_cast<hf_u8_t>(pin_), value)) return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    hf_bool_t new_value = !value;
    hf_bool_t ok = driver_->writePin(static_cast<hf_u8_t>(pin_), new_value);
    current_state_ = new_value ? hf_gpio_state_t::HF_GPIO_STATE_ACTIVE : hf_gpio_state_t::HF_GPIO_STATE_INACTIVE;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
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
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    return pcal95555_driver_->setPinPolarity(static_cast<hf_u8_t>(pin), invert);
}

hf_gpio_err_t Pcal95555Handler::GetPolarityInversion(hf_pin_num_t pin, hf_bool_t& invert) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->getPinPolarity(static_cast<hf_u8_t>(pin), value);
    if (ok) invert = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetInterruptMask(hf_pin_num_t pin, hf_bool_t mask) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    return pcal95555_driver_->setPinInterruptMask(static_cast<hf_u8_t>(pin), mask);
}

hf_gpio_err_t Pcal95555Handler::GetInterruptMask(hf_pin_num_t pin, hf_bool_t& mask) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->getPinInterruptMask(static_cast<hf_u8_t>(pin), value);
    if (ok) mask = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetInterruptStatus(hf_pin_num_t pin, hf_bool_t& status) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    hf_bool_t value = false;
    hf_bool_t ok = pcal95555_driver_->getPinInterruptStatus(static_cast<hf_u8_t>(pin), value);
    if (ok) status = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool is_input = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    bool ok = pcal95555_driver_->setPinsDirection(pin_mask, is_input);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetOutputs(uint16_t pin_mask, hf_bool_t active) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->writePins(pin_mask, active);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
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
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->getAllInterruptMasks(mask);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptStatus(uint16_t& status) noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->getAllInterruptStatus(status);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_bool_t Pcal95555Handler::SoftwareReset() noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!pcal95555_driver_) return false;
    return pcal95555_driver_->softwareReset();
}

hf_bool_t Pcal95555Handler::PowerDown() noexcept {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!pcal95555_driver_) return false;
    return pcal95555_driver_->powerDown();
} 