#include "Tmc9660Handler.h"
#include <cstring>

// ==================== TMC9660 Communication Interface Implementations ====================

// SPI Communication Interface Implementation
Tmc9660SpiCommInterface::Tmc9660SpiCommInterface(BaseSpi& spi_interface) noexcept
    : spi_interface_(spi_interface) {}

bool Tmc9660SpiCommInterface::spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    // Ensure SPI is initialized
    if (!spi_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Use BaseSpi Transfer method - returns true on success
    return spi_interface_.Transfer(tx.data(), rx.data(), 8);
}

// UART Communication Interface Implementation  
Tmc9660UartCommInterface::Tmc9660UartCommInterface(BaseUart& uart_interface) noexcept
    : uart_interface_(uart_interface) {}

bool Tmc9660UartCommInterface::sendUartDatagram(const std::array<uint8_t, 9>& data) noexcept {
    // Ensure UART is initialized
    if (!uart_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Send the 9-byte datagram
    hf_uart_err_t result = uart_interface_.Write(data.data(), 9);
    return result == hf_uart_err_t::UART_SUCCESS;
}

bool Tmc9660UartCommInterface::receiveUartDatagram(std::array<uint8_t, 9>& data) noexcept {
    // Ensure UART is initialized
    if (!uart_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Read the 9-byte datagram with timeout
    hf_uart_err_t result = uart_interface_.Read(data.data(), 9, 1000); // 1 second timeout
    return result == hf_uart_err_t::UART_SUCCESS;
}

// ==================== Tmc9660Handler Implementation ====================

Tmc9660Handler::Tmc9660Handler(BaseSpi& spi_interface, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : spi_ref_(&spi_interface),
      uart_ref_(nullptr),
      active_comm_interface_(nullptr),
      tmc9660_(nullptr), // Will be created in Initialize()
      bootCfg_(bootCfg),
      device_address_(address) {
    // Interfaces will be created lazily in Initialize()
}

Tmc9660Handler::Tmc9660Handler(BaseUart& uart_interface, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : spi_ref_(nullptr),
      uart_ref_(&uart_interface),
      active_comm_interface_(nullptr),
      tmc9660_(nullptr), // Will be created in Initialize()
      bootCfg_(bootCfg),
      device_address_(address) {
    // Interfaces will be created lazily in Initialize()
}

Tmc9660Handler::Tmc9660Handler(BaseSpi& spi_interface, BaseUart& uart_interface, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : spi_ref_(&spi_interface),
      uart_ref_(&uart_interface),
      active_comm_interface_(nullptr),
      tmc9660_(nullptr), // Will be created in Initialize()
      bootCfg_(bootCfg),
      device_address_(address) {
    // Interfaces will be created lazily in Initialize()
    // SPI takes precedence when both are available
}

Tmc9660Handler::~Tmc9660Handler() = default;

TMC9660CommInterface* Tmc9660Handler::InitializeCommInterface() {
    // Prefer SPI if available, fallback to UART
    if (comm_interface_spi_) {
        return comm_interface_spi_.get();
    } else if (comm_interface_uart_) {
        return comm_interface_uart_.get();
    }
    return nullptr;
}

bool Tmc9660Handler::Initialize() {
    // Create communication interfaces only when needed
    if (spi_ref_ && !comm_interface_spi_) {
        comm_interface_spi_ = std::make_unique<Tmc9660SpiCommInterface>(*spi_ref_);
        if (!active_comm_interface_) {
            active_comm_interface_ = comm_interface_spi_.get();
        }
    }
    
    if (uart_ref_ && !comm_interface_uart_) {
        comm_interface_uart_ = std::make_unique<Tmc9660UartCommInterface>(*uart_ref_);
        if (!active_comm_interface_) {
            active_comm_interface_ = comm_interface_uart_.get();
        }
    }
    
    // SPI takes precedence if both are available
    if (spi_ref_ && comm_interface_spi_) {
        active_comm_interface_ = comm_interface_spi_.get();
    }
    
    if (!active_comm_interface_) {
        return false; // No valid communication interface
    }
    
    // Create TMC9660 driver instance with active interface
    if (!tmc9660_) {
        tmc9660_ = std::make_shared<TMC9660>(*active_comm_interface_, device_address_, bootCfg_);
    }
    
    // Create GPIO and ADC wrappers if not already created
    if (!gpioWrappers_[0]) {
        gpioWrappers_[0] = std::make_unique<Gpio>(*this, 17);
        gpioWrappers_[1] = std::make_unique<Gpio>(*this, 18);
        adcWrapper_ = std::make_unique<Adc>(*this);
    }
    
    return tmc9660_->bootloaderInit(bootCfg_) == TMC9660::BootloaderInitResult::Success;
}

CommMode Tmc9660Handler::GetCommMode() const noexcept {
    if (active_comm_interface_) {
        return active_comm_interface_->mode();
    }
    return CommMode::SPI; // Default fallback
}

bool Tmc9660Handler::SwitchCommInterface(CommMode mode) {
    switch (mode) {
        case CommMode::SPI:
            if (comm_interface_spi_) {
                active_comm_interface_ = comm_interface_spi_.get();
                return true;
            }
            break;
        case CommMode::UART:
            if (comm_interface_uart_) {
                active_comm_interface_ = comm_interface_uart_.get();
                return true;
            }
            break;
    }
    return false;
}

Tmc9660Handler::Gpio& Tmc9660Handler::gpio(uint8_t gpioNumber) {
    if (gpioNumber == 17) return *gpioWrappers_[0];
    if (gpioNumber == 18) return *gpioWrappers_[1];
    // Return first GPIO as fallback for invalid numbers (no exceptions)
    return *gpioWrappers_[0];
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
    if (!parent_.tmc9660_) {
        return false; // Handler not initialized
    }
    // Set as output, no pull, active high
    return parent_.tmc9660_->gpio.setMode(gpioNumber_, true, false, true);
}

bool Tmc9660Handler::Gpio::Deinitialize() noexcept {
    // No hardware deinit needed
    return true;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetActive() noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.setPin(gpioNumber_, true)) {
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILED;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetInactive() noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.setPin(gpioNumber_, false)) {
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILED;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::Toggle() noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    bool current_state;
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.readPin(gpioNumber_, current_state)) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILED;
    }
    
    if (!parent_.tmc9660_->gpio.setPin(gpioNumber_, !current_state)) {
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILED;
    }
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::IsActive(bool& is_active) noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.readPin(gpioNumber_, is_active)) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILED;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetState(hf_gpio_state_t& state) noexcept {
    bool is_active;
    hf_gpio_err_t result = IsActive(is_active);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        state = is_active ? hf_gpio_state_t::HF_GPIO_STATE_ACTIVE : hf_gpio_state_t::HF_GPIO_STATE_INACTIVE;
    }
    return result;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetState(hf_gpio_state_t state) noexcept {
    switch (state) {
        case hf_gpio_state_t::HF_GPIO_STATE_ACTIVE:
            return SetActive();
        case hf_gpio_state_t::HF_GPIO_STATE_INACTIVE:
            return SetInactive();
        default:
            return hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER;
    }
}

hf_gpio_err_t Tmc9660Handler::Gpio::ValidatePin() noexcept {
    return (gpioNumber_ == 17 || gpioNumber_ == 18) ? 
        hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
}

hf_gpio_direction_t Tmc9660Handler::Gpio::GetDirection() const noexcept {
    return hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT;
}

bool Tmc9660Handler::Gpio::IsPinAvailable() const noexcept {
    return gpioNumber_ == 17 || gpioNumber_ == 18;
}

hf_u8_t Tmc9660Handler::Gpio::GetMaxPins() const noexcept {
    return 2;
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
    return 4; // ADC_I0, ADC_I1, ADC_I2, ADC_I3
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
    
    // Convert raw ADC value to voltage (adjust scale factor as needed for your hardware)
    channel_reading_v = static_cast<float>(raw) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                                   hf_u8_t numOfSamplesToAvg,
                                                   hf_time_t timeBetweenSamples) noexcept {
    if (channel_id > 3) {
        return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    
    // Map channel ID to TMC9660 ADC parameter
    // Assuming ADC_I0, ADC_I1, ADC_I2, ADC_I3 are consecutive enum values
    uint32_t value = 0;
    tmc9660::tmcl::Parameters param = static_cast<tmc9660::tmcl::Parameters>(
        static_cast<uint16_t>(tmc9660::tmcl::Parameters::ADC_I0) + channel_id);
    
    if (!parent_.tmc9660_ || !parent_.tmc9660_->readParameter(param, value)) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    channel_reading_count = value;
    return hf_adc_err_t::ADC_SUCCESS;
} 