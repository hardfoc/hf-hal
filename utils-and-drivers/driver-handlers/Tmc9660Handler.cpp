#include "Tmc9660Handler.h"
#include <cstring>

/**
 * @brief Default TMC9660 bootloader configuration based on TMC9660-3PH-EVAL board settings.
 * 
 * This configuration is derived from the official TMC9660-3PH-EVAL evaluation board 
 * bootloader TOML file and provides a comprehensive setup for motor control applications.
 * 
 * Key configuration highlights:
 * - LDO Regulators: VEXT1=5.0V, VEXT2=3.3V with 3ms slope control
 * - Boot Mode: Parameter mode for flexible motor control via TMCL parameters
 * - UART: Auto16x baud rate detection, GPIO6/7 pins, device address 1
 * - External Clock: 16MHz crystal oscillator with PLL for stable 40MHz system clock
 * - SPI Flash: Enabled on SPI0 interface with 10MHz operation (GPIO11 SCK, GPIO12 CS)
 * - GPIO Configuration:
 *   - GPIO5: Analog input for sensor feedback
 *   - GPIO17: Digital input with pull-down resistor
 *   - GPIO18: Digital input with pull-down resistor
 * - Additional Features: Hall sensors (GPIO2/3/4), ABN encoders, watchdog enabled
 * 
 * This configuration ensures compatibility with the TMC9660-3PH-EVAL documentation
 * and reference designs, providing a solid foundation for motor control development.
 * 
 * @note Configuration matches TMC9660-3PH-EVKIT User Guide Rev 1 Table 3 specifications
 * @see TMC9660-3PH-EVKIT Evaluation Board User Guide for complete pin assignments
 */
// Static member initialization
const tmc9660::BootloaderConfig Tmc9660Handler::kDefaultBootConfig = {
    // LDO Configuration - Enable both LDOs per evaluation board settings
    {
        tmc9660::bootcfg::LDOVoltage::V5_0,      // vext1 - 5V output
        tmc9660::bootcfg::LDOVoltage::V3_3,      // vext2 - 3.3V output
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // slope_vext1 - 3ms slope
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // slope_vext2 - 3ms slope
        false                                    // ldo_short_fault - Disabled per config
    },
    
    // Boot Configuration - Parameter mode for parameter-based control
    {
        tmc9660::bootcfg::BootMode::Parameter,   // boot_mode - Parameter mode
        false,                                   // bl_ready_fault
        true,                                    // bl_exit_fault
        false,                                   // disable_selftest
        false,                                   // bl_config_fault
        false                                    // start_motor_control - Let application control
    },
    
    // UART Configuration - Standard settings for TMCL communication
    {
        1,                                          // device_address
        255,                                        // host_address (broadcast)
        false,                                      // disable_uart
        tmc9660::bootcfg::UartRxPin::GPIO7,        // rx_pin
        tmc9660::bootcfg::UartTxPin::GPIO6,        // tx_pin
        tmc9660::bootcfg::BaudRate::Auto16x        // baud_rate - Auto 16x oversampling
    },
    
    // RS485 Configuration - Disabled by default
    {
        false,                                      // enable_rs485
        tmc9660::bootcfg::RS485TxEnPin::None,      // txen_pin
        0,                                          // txen_pre_delay
        0                                           // txen_post_delay
    },
    
    // SPI Boot Configuration - Use interface 0 with standard pins
    {
        false,                                      // disable_spi
        tmc9660::bootcfg::SPIInterface::IFACE0,    // boot_spi_iface
        tmc9660::bootcfg::SPI0SckPin::GPIO6        // spi0_sck_pin
    },
    
    // SPI Flash Configuration - Enable per TMC9660-3PH-EVAL board settings
    {
        true,                                       // enable_flash - Enable SPI flash
        tmc9660::bootcfg::SPIInterface::IFACE0,    // flash_spi_iface - Use SPI0 block
        tmc9660::bootcfg::SPI0SckPin::GPIO11,      // spi0_sck_pin - GPIO11 per eval board
        12,                                         // cs_pin - GPIO12 chip select
        tmc9660::bootcfg::SPIFlashFreq::Div1       // freq_div - 10MHz frequency (Div1)
    },
    
    // I2C EEPROM Configuration - Disabled by default
    {
        false,                                      // enable_eeprom
        tmc9660::bootcfg::I2CSdaPin::GPIO5,        // sda_pin
        tmc9660::bootcfg::I2CSclPin::GPIO4,        // scl_pin
        0,                                          // address_bits
        tmc9660::bootcfg::I2CFreq::Freq100k        // freq_code
    },
    
    // Clock Configuration - Use external 16MHz crystal with PLL
    {
        tmc9660::bootcfg::ClockSource::External,   // use_external - External crystal
        tmc9660::bootcfg::ExtSourceType::Oscillator, // ext_source_type - Crystal oscillator
        tmc9660::bootcfg::XtalDrive::Freq16MHz,    // xtal_drive - 16MHz crystal
        false,                                      // xtal_boost
        tmc9660::bootcfg::SysClkSource::PLL,       // pll_selection - Use PLL
        14,                                         // rdiv - PLL reference divider for 16MHz
        tmc9660::bootcfg::SysClkDiv::Div1          // sysclk_div
    },
    
    // GPIO Configuration - Configure per TMC9660-3PH-EVAL board settings
    {
        0x00000000,  // outputMask - GPIO17 and GPIO18 as inputs (bits clear)
        0x00000000,  // directionMask - GPIO17 and GPIO18 configured as inputs
        0x00000000,  // pullUpMask - No pull-up resistors
        0x00060000,  // pullDownMask - GPIO17(bit17)=1, GPIO18(bit18)=1 pull-down
        0x00000020   // analogMask - GPIO5 as analog input (bit5=1)
    }
};

// ==================== TMC9660 Communication Interface Implementations ====================

// SPI Communication Interface Implementation
Tmc9660SpiCommInterface::Tmc9660SpiCommInterface(BaseSpi& spi_interface) noexcept
    : spi_interface_(spi_interface) {}

bool Tmc9660SpiCommInterface::spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    // Ensure SPI is initialized
    if (!spi_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Use the error-returning version and check for success
    hf_spi_err_t result = spi_interface_.Transfer(tx.data(), rx.data(), hf_u16_t(8), hf_u32_t(0));
    return result == hf_spi_err_t::SPI_SUCCESS;
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

hf_gpio_err_t Tmc9660Handler::Gpio::SetPinLevelImpl(hf_gpio_level_t level) noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    bool pin_high = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.writePin(gpioNumber_, pin_high)) {
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetPinLevelImpl(hf_gpio_level_t& level) noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    bool pin_state;
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.readDigital(gpioNumber_, pin_state)) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    }
    
    level = pin_state ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetDirectionImpl(hf_gpio_direction_t direction) noexcept {
    // TMC9660 GPIO pins are output-only, so we only support output direction
    if (direction != hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept {
    // TMC9660 GPIO pins are push-pull only
    if (mode != hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept {
    // TMC9660 GPIO pins don't support pull resistors
    if (mode != hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_pull_mode_t Tmc9660Handler::Gpio::GetPullModeImpl() const noexcept {
    // TMC9660 GPIO pins are always floating (no pull resistors)
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

bool Tmc9660Handler::Gpio::IsPinAvailable() const noexcept {
    return gpioNumber_ == 17 || gpioNumber_ == 18;
}

hf_u8_t Tmc9660Handler::Gpio::GetMaxPins() const noexcept {
    return 2;
}

const char* Tmc9660Handler::Gpio::GetDescription() const noexcept {
    return description_;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept {
    // TMC9660 GPIO pins are always output-only
    direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept {
    // TMC9660 GPIO pins are always push-pull
    mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL;
    return hf_gpio_err_t::GPIO_SUCCESS;
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

hf_adc_err_t Tmc9660Handler::Adc::ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                              float& channel_reading_v, hf_u8_t numOfSamplesToAvg,
                                              hf_time_t timeBetweenSamples) noexcept {
    // Read the raw count first
    hf_adc_err_t err = ReadChannelCount(channel_id, channel_reading_count, numOfSamplesToAvg, timeBetweenSamples);
    if (err != hf_adc_err_t::ADC_SUCCESS) {
        return err;
    }
    
    // Convert to voltage
    channel_reading_v = static_cast<float>(channel_reading_count) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
} 