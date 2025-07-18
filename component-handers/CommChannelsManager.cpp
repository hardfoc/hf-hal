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
    // Initialize vectors for managing multiple buses
    spi_device_indices_.reserve(4); // Reserve space for 4 SPI devices
    i2c_device_indices_.reserve(4); // Reserve space for 4 I2C devices
    uart_buses_.reserve(2);  // Reserve space for 2 UART buses
    can_buses_.reserve(2);   // Reserve space for 2 CAN buses
}

bool CommChannelsManager::Initialize() noexcept {

    //================ UART FOR TMC9660 TMCL COMMUNICATION =================//
    /*
    * # UART Configuration for TMC9660
    * - UART0 configured for TMC9660 TMCL protocol communication
    * - Baud rate: 115200, Format: 8N1 (8 data bits, no parity, 1 stop bit)
    * - Buffers: TX=256 bytes, RX=512 bytes (optimized for 9-byte TMCL frames)
    */
    {
        ESP_LOGI("CommChannelsManager", "Configuring UART for TMC9660 TMCL communication");
        
        hf_uart_config_t uart_config = {};
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::UART_TXD);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::UART_RXD);
        
        if (tx_map && rx_map) {
            // ESP-IDF v5.5 compliant UART configuration for TMC9660 TMCL protocol
            uart_config.port_number = 0;                                           // UART0 port
            uart_config.baud_rate = 115200;                                        // TMC9660 standard baud rate
            uart_config.data_bits = hf_uart_data_bits_t::HF_UART_DATA_8_BITS;      // 8 data bits per TMCL spec
            uart_config.parity = hf_uart_parity_t::HF_UART_PARITY_DISABLE;         // No parity for TMCL protocol
            uart_config.stop_bits = hf_uart_stop_bits_t::HF_UART_STOP_BITS_1;      // 1 stop bit
            uart_config.flow_control = hf_uart_flow_ctrl_t::HF_UART_HW_FLOWCTRL_DISABLE; // No flow control
            uart_config.tx_pin = tx_map->physical_pin;                             // TX: GPIO5 (UART_TXD)
            uart_config.rx_pin = rx_map->physical_pin;                             // RX: GPIO4 (UART_RXD)
            uart_config.rts_pin = HF_UART_IO_UNUSED;                               // RTS not used
            uart_config.cts_pin = HF_UART_IO_UNUSED;                               // CTS not used
            
            // Buffer configuration optimized for TMC9660 9-byte TMCL datagrams
            uart_config.tx_buffer_size = 256;                                      // TX buffer (~28 TMCL frames)
            uart_config.rx_buffer_size = 512;                                      // RX buffer (~56 TMCL frames)
            uart_config.event_queue_size = 20;                                     // Event queue for interrupts
            uart_config.operating_mode = hf_uart_operating_mode_t::HF_UART_MODE_INTERRUPT; // Interrupt-driven
            uart_config.timeout_ms = 100;                                          // Timeout for operations
            uart_config.enable_pattern_detection = false;                          // Disable for now
            uart_config.enable_wakeup = false;                                     // No wakeup needed
            uart_config.enable_loopback = false;                                   // No loopback testing
            
            // Create UART bus
            uart_buses_.emplace_back(std::make_unique<EspUart>(uart_config));
            
            ESP_LOGI("CommChannelsManager", "UART configured for TMC9660: 115200 8N1, TX=GPIO%d, RX=GPIO%d", 
                     tx_map->physical_pin, rx_map->physical_pin);
            ESP_LOGI("CommChannelsManager", "UART buffers: TX=%d bytes, RX=%d bytes (optimized for 9-byte TMCL frames)",
                     uart_config.tx_buffer_size, uart_config.rx_buffer_size);
        } else {
            ESP_LOGE("CommChannelsManager", "UART GPIO mapping missing - TMC9660 communication will not be available");
            ESP_LOGE("CommChannelsManager", "Required pins: UART_TXD=%s, UART_RXD=%s", 
                     tx_map ? "OK" : "MISSING", rx_map ? "OK" : "MISSING");
        }
    }

    //================ SPI2_HOST =================//
    {
        hf_spi_bus_config_t spi_config = {};

        // Get pin mappings for SPI
        // Note: This assumes SPI2_MISO, SPI2_MOSI, SPI2_SCK are defined in HfFunctionalGpioPin
        auto* miso_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MISO);
        auto* mosi_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MOSI);
        auto* sck_map  = GetGpioMapping(HfFunctionalGpioPin::SPI2_SCK);

        // If all required pins are mapped to a proper structure, create the SPI bus
        if (miso_map && mosi_map && sck_map) {
            spi_config.host = static_cast<hf_spi_host_device_t>(1); // SPI2_HOST
            spi_config.miso_pin = miso_map->physical_pin;
            spi_config.mosi_pin = mosi_map->physical_pin;
            spi_config.sclk_pin = sck_map->physical_pin;
            spi_config.dma_channel = 0; // Auto-select DMA channel
            spi_config.timeout_ms = 1000;
            spi_config.use_iomux = true; // Use IOMUX for better performance

            // Create the bus
            spi_bus_ = std::make_unique<EspSpiBus>(spi_config);
            spi_bus_->Initialize();

            // Now create devices for all configured CS pins
            for (auto cs_pin_enum : kSpiCsPins) {
                auto* cs_map = GetGpioMapping(cs_pin_enum);
                if (cs_map) {
                    hf_spi_device_config_t dev_cfg = {};
                    dev_cfg.cs_pin = cs_map->physical_pin;
                    dev_cfg.clock_speed_hz = 10 * 1000000; // 10 MHz max for both devices
                    
                    // Configure SPI mode based on device type
                    if (cs_pin_enum == HfFunctionalGpioPin::SPI2_CS_AS5047) {
                        // AS5047U: SPI Mode 1 (CPOL=0, CPHA=1) - samples on falling CLK edge
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_1;
                    } else if (cs_pin_enum == HfFunctionalGpioPin::SPI2_CS_TMC9660) {
                        // TMC9660: SPI Mode 3 (CPOL=1, CPHA=1) - idle high, sample on falling edge
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_3;
                    } else {
                        // Default mode for external GPIO devices
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_0;
                    }
                    
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
                    
                    // Create device and store its index
                    int device_index = spi_bus_->CreateDevice(dev_cfg);
                    if (device_index >= 0) {
                        spi_device_indices_.push_back(device_index);
                    }
                }
            }
        }
    }

    //================ I2C FOR BNO08X IMU AND PCAL95555 GPIO EXPANDER =================//
    {
        ESP_LOGI("CommChannelsManager", "Configuring I2C master bus for BNO08x IMU and PCAL95555 GPIO expander");
        
        hf_i2c_master_bus_config_t i2c_config = {};
        auto* sda_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SDA);
        auto* scl_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SCL);
        
        if (sda_map && scl_map) {
            // ESP-IDF v5.5+ I2C master bus configuration for BNO08x and PCAL95555
            i2c_config.i2c_port = I2C_NUM_0;                                               // I2C port 0 (ESP32C6 has 1 I2C port)
            i2c_config.sda_io_num = sda_map->physical_pin;                                 // SDA GPIO pin from board mapping
            i2c_config.scl_io_num = scl_map->physical_pin;                                 // SCL GPIO pin from board mapping
            i2c_config.enable_internal_pullup = sda_map->has_pullup && scl_map->has_pullup; // Use internal pullups if available
            i2c_config.clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT;        // Default clock source (APB)
            i2c_config.clk_flags = 0;                                                      // No additional clock flags
            i2c_config.glitch_ignore_cnt = hf_i2c_glitch_filter_t::HF_I2C_GLITCH_FILTER_7_CYCLES; // 7-cycle glitch filter for noise immunity
            i2c_config.trans_queue_depth = 8;                                              // Transaction queue depth for async operations
            i2c_config.intr_priority = 5;                                                  // Interrupt priority (0-7, 5=medium)
            i2c_config.flags = 0;                                                          // No additional configuration flags
            i2c_config.allow_pd = false;                                                   // Disable power down in sleep modes
            
            // Create I2C master bus instance
            i2c_bus_ = std::make_unique<EspI2cBus>(i2c_config);
            
            // Initialize the I2C bus
            if (i2c_bus_->Initialize()) {
                ESP_LOGI("CommChannelsManager", "I2C master bus initialized: SDA=GPIO%d, SCL=GPIO%d", 
                         sda_map->physical_pin, scl_map->physical_pin);
                ESP_LOGI("CommChannelsManager", "I2C configuration: 400kHz, 7-cycle glitch filter, queue depth=8");
                
                // Add BNO08x IMU device (standard I2C address 0x4A)
                {
                    hf_i2c_device_config_t bno08x_device = {};
                    bno08x_device.device_address = 0x4A;                                       // BNO08x standard I2C address
                    bno08x_device.dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;  // 7-bit addressing
                    bno08x_device.scl_speed_hz = 400000;                                       // 400kHz for fast I2C operation
                    bno08x_device.scl_wait_us = 0;                                             // No additional SCL wait time
                    bno08x_device.disable_ack_check = false;                                   // Enable ACK check
                    bno08x_device.flags = 0;                                                   // No device-specific flags
                    
                    int device_index = i2c_bus_->CreateDevice(bno08x_device);
                    if (device_index >= 0) {
                        i2c_device_indices_.push_back(device_index);
                        ESP_LOGI("CommChannelsManager", "BNO08x IMU device added to I2C bus: address=0x%02X, speed=400kHz", 
                                 bno08x_device.device_address);
                    } else {
                        ESP_LOGW("CommChannelsManager", "Failed to add BNO08x IMU device to I2C bus");
                    }
                }
                
                // Add PCAL95555 GPIO expander device (standard I2C address range 0x20-0x27)
                {
                    hf_i2c_device_config_t pcal95555_device = {};
                    pcal95555_device.device_address = 0x20;                                    // PCAL95555 base address (A0=A1=A2=0)
                    pcal95555_device.dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT; // 7-bit addressing
                    pcal95555_device.scl_speed_hz = 400000;                                    // 400kHz for fast I2C operation
                    pcal95555_device.scl_wait_us = 0;                                          // No additional SCL wait time
                    pcal95555_device.disable_ack_check = false;                                // Enable ACK check
                    pcal95555_device.flags = 0;                                                // No device-specific flags
                    
                    int device_index = i2c_bus_->CreateDevice(pcal95555_device);
                    if (device_index >= 0) {
                        i2c_device_indices_.push_back(device_index);
                        ESP_LOGI("CommChannelsManager", "PCAL95555 GPIO expander device added to I2C bus: address=0x%02X, speed=400kHz", 
                                 pcal95555_device.device_address);
                    } else {
                        ESP_LOGW("CommChannelsManager", "Failed to add PCAL95555 GPIO expander device to I2C bus");
                    }
                }
                
                ESP_LOGI("CommChannelsManager", "I2C bus ready for BNO08x IMU and PCAL95555 GPIO expander communication");
            } else {
                ESP_LOGE("CommChannelsManager", "Failed to initialize I2C master bus");
            }
        } else {
            ESP_LOGE("CommChannelsManager", "I2C GPIO mapping missing - I2C devices will not be available");
            ESP_LOGE("CommChannelsManager", "Required pins: I2C_SDA=%s, I2C_SCL=%s", 
                     sda_map ? "OK" : "MISSING", scl_map ? "OK" : "MISSING");
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

    //================ Initialization =================//

    // If no buses were created, return false
    if (!spi_bus_ && !i2c_bus_ && uart_buses_.empty() && can_buses_.empty()) {
        // No buses initialized, return false
        return false;
    }

    // Track if all buses were successfully initialized
    bool all_initialized = true;

    // Initialize SPI bus
    if (spi_bus_ && !spi_bus_->Initialize()) {
        all_initialized = false;
    }

    // Initialize I2C bus (already initialized in the configuration block above)
    // The I2C bus is initialized when created and devices are added
    for (auto& uart : uart_buses_) {
        if (!uart->EnsureInitialized()) {
            all_initialized = false;
        }
    }
    for (auto& can : can_buses_) {
        if (!can->EnsureInitialized()) {
            all_initialized = false;
        }
    }

    return all_initialized;
}

bool CommChannelsManager::Deinitialize() noexcept {
    if (!initialized_) {
        return true; // Already deinitialized
    }
    
    // If no buses were created, return false
    if (!spi_bus_ && !i2c_bus_ && uart_buses_.empty() && can_buses_.empty()) {
        // No buses initialized, return false
        return false;
    }

    // Track if all buses were successfully deinitialized
    bool all_deinitialized = true;

    // Deinitialize SPI bus
    if (spi_bus_ && !spi_bus_->Deinitialize()) {
        all_deinitialized = false;
    }

    // Deinitialize I2C bus
    if (i2c_bus_ && !i2c_bus_->Deinitialize()) {
        all_deinitialized = false;
    }
    for (auto& uart : uart_buses_) {
        if (!uart->EnsureDeinitialized()) {
            all_deinitialized = false;
        }
    }
    for (auto& can : can_buses_) {
        if (!can->EnsureDeinitialized()) {
            all_deinitialized = false;
        }
    }

    return all_deinitialized;
}

CommChannelsManager::~CommChannelsManager() = default;

//==================== Accessors ====================//

EspSpiBus& CommChannelsManager::GetSpiBus() noexcept {
    return *spi_bus_;
}

BaseSpi* CommChannelsManager::GetSpiDevice(int device_index) noexcept {
    if (!spi_bus_) return nullptr;
    return spi_bus_->GetDevice(device_index);
}

BaseSpi* CommChannelsManager::GetSpiDevice(SpiDeviceId device_id) noexcept {
    if (!spi_bus_) return nullptr;
    int device_index = static_cast<int>(device_id);
    return spi_bus_->GetDevice(device_index);
}

EspSpiDevice* CommChannelsManager::GetEspSpiDevice(int device_index) noexcept {
    if (!spi_bus_) return nullptr;
    return spi_bus_->GetEspDevice(device_index);
}

EspSpiDevice* CommChannelsManager::GetEspSpiDevice(SpiDeviceId device_id) noexcept {
    if (!spi_bus_) return nullptr;
    int device_index = static_cast<int>(device_id);
    return spi_bus_->GetEspDevice(device_index);
}

std::size_t CommChannelsManager::GetSpiDeviceCount() const noexcept {
    if (!spi_bus_) return 0;
    return spi_bus_->GetDeviceCount();
}

//==================== I2C Accessors ====================//

EspI2cBus& CommChannelsManager::GetI2cBus() noexcept {
    return *i2c_bus_;
}

BaseI2c* CommChannelsManager::GetI2cDevice(int device_index) noexcept {
    if (!i2c_bus_) return nullptr;
    return i2c_bus_->GetDevice(device_index);
}

BaseI2c* CommChannelsManager::GetI2cDevice(I2cDeviceId device_id) noexcept {
    if (!i2c_bus_) return nullptr;
    int device_index = static_cast<int>(device_id);
    return i2c_bus_->GetDevice(device_index);
}

EspI2cDevice* CommChannelsManager::GetEspI2cDevice(int device_index) noexcept {
    if (!i2c_bus_) return nullptr;
    return i2c_bus_->GetEspDevice(device_index);
}

EspI2cDevice* CommChannelsManager::GetEspI2cDevice(I2cDeviceId device_id) noexcept {
    if (!i2c_bus_) return nullptr;
    int device_index = static_cast<int>(device_id);
    return i2c_bus_->GetEspDevice(device_index);
}

std::size_t CommChannelsManager::GetI2cDeviceCount() const noexcept {
    if (!i2c_bus_) return 0;
    return i2c_bus_->GetDeviceCount();
}

//==================== Legacy I2C Accessors ====================//

BaseI2c& CommChannelsManager::GetI2c(std::size_t which) noexcept {
    // Legacy interface: return the first device for backward compatibility
    // TODO: Consider deprecating this method in favor of GetI2cDevice()
    if (!i2c_bus_ || which >= i2c_bus_->GetDeviceCount()) {
        static EspI2cDevice dummy_device;
        return dummy_device; // Return a dummy device to avoid null reference
    }
    return *i2c_bus_->GetDevice(which);
}

std::size_t CommChannelsManager::GetI2cCount() const noexcept {
    return i2c_bus_ ? i2c_bus_->GetDeviceCount() : 0;
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