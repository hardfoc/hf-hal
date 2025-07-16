#include "Tmc9660Handler.h"
#include <stdexcept>
#include <cstring>

// ==================== Tmc9660Handler Implementation ====================

Tmc9660Handler::Tmc9660Handler(TMC9660CommInterface& comm, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : tmc9660_(comm, address, bootCfg), bootCfg_(bootCfg) {
    gpioWrappers_[0] = std::make_unique<Gpio>(*this, 17);
    gpioWrappers_[1] = std::make_unique<Gpio>(*this, 18);
    adcWrapper_ = std::make_unique<Adc>(*this);
}

Tmc9660Handler::~Tmc9660Handler() = default;

bool Tmc9660Handler::Initialize() {
    return tmc9660_.bootloaderInit(bootCfg_) == TMC9660::BootloaderInitResult::Success;
}

Tmc9660Handler::Gpio& Tmc9660Handler::gpio(uint8_t gpioNumber) {
    if (gpioNumber == 17) return *gpioWrappers_[0];
    if (gpioNumber == 18) return *gpioWrappers_[1];
    throw std::out_of_range("TMC9660 only supports GPIO 17 and 18");
}

Tmc9660Handler::Adc& Tmc9660Handler::adc() {
    return *adcWrapper_;
}

// ==================== Gpio Wrapper Implementation ====================

Tmc9660Handler::Gpio::Gpio(Tmc9660Handler& parent, uint8_t gpioNumber)
    : BaseGpio(gpioNumber, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT),
      parent_(parent), gpioNumber_(gpioNumber) {
    std::snprintf(description_, sizeof(description_), "TMC9660 GPIO%u", gpioNumber_);
}

bool Tmc9660Handler::Gpio::Initialize() noexcept {
    // Set as output, no pull, active high
    return parent_.tmc9660_.gpio.setMode(gpioNumber_, true, false, true);
}

bool Tmc9660Handler::Gpio::Deinitialize() noexcept {
    // No hardware deinit needed
    return true;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetActive() noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (!parent_.tmc9660_.gpio.writePin(gpioNumber_, true))
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetInactive() noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (!parent_.tmc9660_.gpio.writePin(gpioNumber_, false))
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::Toggle() noexcept {
    bool is_active = false;
    hf_gpio_err_t err = IsActive(is_active);
    if (err != hf_gpio_err_t::GPIO_SUCCESS) return err;
    return is_active ? SetInactive() : SetActive();
}

hf_gpio_err_t Tmc9660Handler::Gpio::IsActive(bool& is_active) noexcept {
    hf_gpio_state_t state;
    hf_gpio_err_t err = GetState(state);
    if (err != hf_gpio_err_t::GPIO_SUCCESS) return err;
    is_active = (state == hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetState(hf_gpio_state_t& state) noexcept {
    bool value = false;
    if (!parent_.tmc9660_.gpio.readDigital(gpioNumber_, value))
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    state = value ? hf_gpio_state_t::HF_GPIO_STATE_ACTIVE : hf_gpio_state_t::HF_GPIO_STATE_INACTIVE;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetState(hf_gpio_state_t state) noexcept {
    if (state == hf_gpio_state_t::HF_GPIO_STATE_ACTIVE) return SetActive();
    if (state == hf_gpio_state_t::HF_GPIO_STATE_INACTIVE) return SetInactive();
    return hf_gpio_err_t::GPIO_ERR_INVALID_STATE;
}

hf_gpio_err_t Tmc9660Handler::Gpio::ValidatePin() noexcept {
    if (gpioNumber_ == 17 || gpioNumber_ == 18) return hf_gpio_err_t::GPIO_SUCCESS;
    return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
}

hf_gpio_direction_t Tmc9660Handler::Gpio::GetDirection() const noexcept {
    return hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT;
}

bool Tmc9660Handler::Gpio::IsPinAvailable() const noexcept {
    return (gpioNumber_ == 17 || gpioNumber_ == 18);
}

hf_u8_t Tmc9660Handler::Gpio::GetMaxPins() const noexcept {
    return 2;
}

const char* Tmc9660Handler::Gpio::GetDescription() const noexcept {
    return description_;
}

// ==================== Adc Wrapper Implementation ====================

Tmc9660Handler::Adc::Adc(Tmc9660Handler& parent) : parent_(parent) {}

bool Tmc9660Handler::Adc::Initialize() noexcept {
    // No hardware init needed for TMC9660 internal ADC
    return true;
}

bool Tmc9660Handler::Adc::Deinitialize() noexcept {
    // No hardware deinit needed
    return true;
}

hf_u8_t Tmc9660Handler::Adc::GetMaxChannels() const noexcept {
    return 4; // I0, I1, I2, I3
}

bool Tmc9660Handler::Adc::IsChannelAvailable(hf_channel_id_t channel_id) const noexcept {
    return (channel_id < 4);
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                               hf_u8_t numOfSamplesToAvg,
                                               hf_time_t timeBetweenSamples) noexcept {
    hf_u32_t raw = 0;
    hf_adc_err_t err = ReadChannelCount(channel_id, raw, numOfSamplesToAvg, timeBetweenSamples);
    if (err != hf_adc_err_t::ADC_SUCCESS) return err;
    // Example conversion: scale to voltage (adjust as needed for your hardware)
    channel_reading_v = static_cast<float>(raw) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                                   hf_u8_t numOfSamplesToAvg,
                                                   hf_time_t timeBetweenSamples) noexcept {
    if (channel_id > 3) return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    uint32_t value = 0;
    tmc9660::tmcl::Parameters param = static_cast<tmc9660::tmcl::Parameters>(
        static_cast<uint16_t>(tmc9660::tmcl::Parameters::ADC_I0) + channel_id);
    if (!parent_.tmc9660_.readParameter(param, value))
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    channel_reading_count = value;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                              float& channel_reading_v, hf_u8_t numOfSamplesToAvg,
                                              hf_time_t timeBetweenSamples) noexcept {
    hf_adc_err_t err = ReadChannelCount(channel_id, channel_reading_count, numOfSamplesToAvg, timeBetweenSamples);
    if (err != hf_adc_err_t::ADC_SUCCESS) return err;
    channel_reading_v = static_cast<float>(channel_reading_count) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
} 