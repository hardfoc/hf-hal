#include "CommChannelsManager.h"

// ESP32 comm interface includes
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspUart.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspCan.h"

// Board mapping includes (already included in header)

CommChannelsManager& CommChannelsManager::GetInstance() noexcept {
    static CommChannelsManager instance;
    return instance;
}

CommChannelsManager::CommChannelsManager() {
    //================ SPI =================//
    {
        hf_spi_bus_config_t spi_config = {};
        auto* miso_map = GetGpioMapping(HfFunctionalGpioPin::SPI0_MISO);
        auto* mosi_map = GetGpioMapping(HfFunctionalGpioPin::SPI0_MOSI);
        auto* sck_map  = GetGpioMapping(HfFunctionalGpioPin::SPI0_SCK);
        if (miso_map && mosi_map && sck_map) {
            spi_config.host = static_cast<hf_spi_host_device_t>(1); // SPI2_HOST
            spi_config.miso_pin = miso_map->physical_pin;
            spi_config.mosi_pin = mosi_map->physical_pin;
            spi_config.sclk_pin = sck_map->physical_pin;
            spi_config.clock_speed_hz = 10000000; // 10 MHz default
            spi_config.mode = 0; // SPI mode 0
            spi_config.bits_per_word = 8;
            spi_config.timeout_ms = 1000;
            // Create the bus
            spi_bus_ = std::make_unique<EspSpiBus>(spi_config);
            spi_bus_->Initialize();
            // Now create three devices for three CS pins
            std::vector<HfFunctionalGpioPin> cs_pins = {
                HfFunctionalGpioPin::SPI0_CS_TMC9660,
                HfFunctionalGpioPin::SPI0_CS_DEVICE2,
                HfFunctionalGpioPin::SPI0_CS_DEVICE3
            };
            for (auto cs_pin_enum : cs_pins) {
                auto* cs_map = GetGpioMapping(cs_pin_enum);
                if (cs_map) {
                    hf_spi_device_config_t dev_cfg = {};
                    dev_cfg.spics_io_num = cs_map->physical_pin;
                    dev_cfg.clock_speed_hz = spi_config.clock_speed_hz;
                    dev_cfg.mode = spi_config.mode;
                    dev_cfg.queue_size = 7;
                    dev_cfg.flags = 0;
                    dev_cfg.cs_ena_pretrans = 0;
                    dev_cfg.cs_ena_posttrans = 0;
                    dev_cfg.duty_cycle_pos = 128;
                    dev_cfg.input_delay_ns = 0;
                    dev_cfg.command_bits = 0;
                    dev_cfg.address_bits = 0;
                    dev_cfg.dummy_bits = 0;
                    dev_cfg.pre_cb = nullptr;
                    dev_cfg.post_cb = nullptr;
                    spi_devices_.emplace_back(spi_bus_->createDevice(dev_cfg));
                }
            }
        }
    }

    //================ I2C =================//
    {
        hf_i2c_master_bus_config_t i2c_config = {};
        auto* sda_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SDA);
        auto* scl_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SCL);
        if (sda_map && scl_map) {
            i2c_config.i2c_port = 0; // I2C_NUM_0
            i2c_config.sda_io_num = sda_map->physical_pin;
            i2c_config.scl_io_num = scl_map->physical_pin;
            i2c_config.enable_internal_pullup = sda_map->has_pullup && scl_map->has_pullup;
            // ... set other config fields as needed
            i2c_buses_.emplace_back(std::make_unique<EspI2c>(i2c_config));
        } else {
            // TODO: Log warning: I2C mapping missing, not instantiating I2C bus
        }
    }

    //================ UART =================//
    {
        hf_uart_config_t uart_config = {};
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::UART_TXD);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::UART_RXD);
        if (tx_map && rx_map) {
            uart_config.port_number = 0;
            uart_config.tx_pin = tx_map->physical_pin;
            uart_config.rx_pin = rx_map->physical_pin;
            uart_config.baud_rate = 115200;
            // ... set other config fields as needed
            uart_buses_.emplace_back(std::make_unique<EspUart>(uart_config));
        } else {
            // TODO: Log warning: UART mapping missing, not instantiating UART bus
        }
    }

    //================ CAN =================//
    {
        hf_esp_can_config_t can_config = {};
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_TX);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_RX);
        if (tx_map && rx_map) {
            can_config.controller_id = hf_can_controller_id_t::HF_CAN_CONTROLLER_0;
            can_config.tx_pin = tx_map->physical_pin;
            can_config.rx_pin = rx_map->physical_pin;
            can_config.baud_rate = 500000;
            // ... set other config fields as needed
            can_buses_.emplace_back(std::make_unique<EspCan>(can_config));
        } else {
            // TODO: Log warning: CAN mapping missing, not instantiating CAN bus
        }
    }
}

CommChannelsManager::~CommChannelsManager() = default;

//==================== Accessors ====================//

BaseSpi& CommChannelsManager::GetSpi(std::size_t which) noexcept {
    return *spi_devices_.at(which);
}
std::size_t CommChannelsManager::GetSpiCount() const noexcept {
    return spi_devices_.size();
}

BaseI2c& CommChannelsManager::GetI2c(std::size_t which) noexcept {
    return *i2c_buses_.at(which);
}
std::size_t CommChannelsManager::GetI2cCount() const noexcept {
    return i2c_buses_.size();
}

BaseUart& CommChannelsManager::GetUart(std::size_t which) noexcept {
    return *uart_buses_.at(which);
}
std::size_t CommChannelsManager::GetUartCount() const noexcept {
    return uart_buses_.size();
}

BaseCan& CommChannelsManager::GetCan(std::size_t which) noexcept {
    return *can_buses_.at(which);
}
std::size_t CommChannelsManager::GetCanCount() const noexcept {
    return can_buses_.size();
} 