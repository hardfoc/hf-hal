# CommChannelsManager - Communication Interfaces Management System

<div align="center">

![Component](https://img.shields.io/badge/component-CommChannelsManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Interfaces](https://img.shields.io/badge/interfaces-SPI%20|%20I2C%20|%20UART%20|%20CAN-orange.svg)

**Comprehensive communication interface management for the HardFOC platform**

</div>

## 📋 Overview

The `CommChannelsManager` is a singleton component handler that provides unified management of all communication interfaces on the HardFOC platform. It handles SPI, I2C, UART, and CAN interfaces with device-specific configuration and board-aware pin assignments.

### ✨ Key Features

- **📡 Multi-Protocol Support**: SPI, I2C, UART, CAN interfaces
- **🔗 Board-Aware Configuration**: Automatic pin mapping and device routing
- **🎯 Device-Specific Access**: Type-safe device enumeration and access
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **⚙️ Flexible Configuration**: Customizable interface parameters
- **🏥 Health Monitoring**: Interface status and diagnostics
- **🔧 Easy Integration**: Simple API for device access
- **🛡️ Error Handling**: Comprehensive error detection and recovery

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  CommChannelsManager                           │
├─────────────────────────────────────────────────────────────────┤
│  Device Enumeration │ Type-safe device identification         │
├─────────────────────────────────────────────────────────────────┤
│  Interface Access   │ SPI, I2C, UART, CAN interface access    │
├─────────────────────────────────────────────────────────────────┤
│  Pin Configuration  │ Board-specific pin and bus mapping      │
├─────────────────────────────────────────────────────────────────┤
│  ESP32 Drivers      │ ESP32-C6 hardware interface drivers     │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Interface Access

```cpp
#include "component-handlers/CommChannelsManager.h"

void comm_basic_example() {
    // Get singleton instance
    auto& comm = CommChannelsManager::GetInstance();
    
    // Initialize the manager
    if (!comm.EnsureInitialized()) {
        printf("Failed to initialize communication manager\n");
        return;
    }
    
    // Access SPI device
    auto* spi_device = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (spi_device) {
        printf("TMC9660 SPI device available\n");
    }
    
    // Access I2C device
    auto* i2c_device = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (i2c_device) {
        printf("BNO08x I2C device available\n");
    }
    
    // Access UART
    auto* uart = comm.GetUart(0);
    if (uart) {
        printf("UART0 interface available\n");
    }
    
    // Access CAN
    auto* can = comm.GetCan();
    if (can) {
        printf("CAN interface available\n");
    }
}
```

## 📖 API Reference

### Core Operations

#### Initialization and Management
```cpp
class CommChannelsManager {
public:
    // Singleton access
    static CommChannelsManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool EnsureDeinitialized() noexcept;
    bool IsInitialized() const noexcept;
    
private:
    bool Initialize() noexcept;
    void Deinitialize() noexcept;
};
```

#### Device Enumeration
```cpp
// SPI device identification
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0,  // TMC9660 motor controller (SPI Mode 3)
    AS5047U_POSITION_ENCODER = 1,  // AS5047U position encoder (SPI Mode 1)
    EXTERNAL_DEVICE_1 = 2,         // External device 1 (SPI Mode 0)
    EXTERNAL_DEVICE_2 = 3,         // External device 2 (SPI Mode 0)
    
    // Aliases
    MOTOR_CONTROLLER = TMC9660_MOTOR_CONTROLLER,
    POSITION_ENCODER = AS5047U_POSITION_ENCODER,
    TMC9660_SPI = TMC9660_MOTOR_CONTROLLER,
    
    SPI_DEVICE_COUNT
};

// I2C device identification
enum class I2cDeviceId : uint8_t {
    BNO08X_IMU = 0,              // BNO08x IMU sensor (address 0x4A or 0x4B)
    PCAL95555_GPIO_EXPANDER = 1, // PCAL95555 GPIO expander (address 0x20-0x27)
    
    // Aliases
    IMU = BNO08X_IMU,
    GPIO_EXPANDER = PCAL95555_GPIO_EXPANDER,
    
    I2C_DEVICE_COUNT
};
```

#### Interface Access
```cpp
// SPI interface access
BaseSpi* GetSpiBus() noexcept;                                    // Master bus access
BaseSpi* GetSpiDevice(SpiDeviceId device_id) noexcept;            // Device-specific access
EspSpiDevice* GetEspSpiDevice(SpiDeviceId device_id) noexcept;    // ESP32-specific access

// I2C interface access
BaseI2c* GetI2cBus() noexcept;                                    // Master bus access
BaseI2c* GetI2cDevice(I2cDeviceId device_id) noexcept;            // Device-specific access
EspI2cDevice* GetEspI2cDevice(I2cDeviceId device_id) noexcept;    // ESP32-specific access

// UART interface access
BaseUart* GetUart(uint8_t uart_num = 0) noexcept;                 // UART port access
EspUart* GetEspUart(uint8_t uart_num = 0) noexcept;               // ESP32-specific access

// CAN interface access
BaseCan* GetCan() noexcept;                                       // CAN interface access
EspCan* GetEspCan() noexcept;                                     // ESP32-specific access
```

#### Status and Diagnostics
```cpp
// Interface status
bool IsSpiInitialized() const noexcept;
bool IsI2cInitialized() const noexcept;
bool IsUartInitialized(uint8_t uart_num = 0) const noexcept;
bool IsCanInitialized() const noexcept;

// Device availability
bool IsSpiDeviceAvailable(SpiDeviceId device_id) const noexcept;
bool IsI2cDeviceAvailable(I2cDeviceId device_id) const noexcept;

// Statistics and health
uint32_t GetSpiTransactionCount() const noexcept;
uint32_t GetI2cTransactionCount() const noexcept;
uint32_t GetUartByteCount(uint8_t uart_num = 0) const noexcept;
uint32_t GetCanMessageCount() const noexcept;
```

## 🎯 Hardware Configuration

### Board Pin Mapping

The CommChannelsManager automatically configures pins based on the HardFOC board design:

```cpp
// SPI pin configuration (SPI2)
constexpr gpio_num_t SPI2_MOSI = GPIO_NUM_11;  // Master Out Slave In
constexpr gpio_num_t SPI2_MISO = GPIO_NUM_13;  // Master In Slave Out
constexpr gpio_num_t SPI2_SCLK = GPIO_NUM_12;  // Serial Clock

// SPI Chip Select pins
constexpr gpio_num_t SPI2_CS_TMC9660 = GPIO_NUM_10;      // TMC9660 motor controller
constexpr gpio_num_t SPI2_CS_AS5047U = GPIO_NUM_9;       // AS5047U position encoder
constexpr gpio_num_t EXT_GPIO_CS_1 = GPIO_NUM_8;         // External device 1
constexpr gpio_num_t EXT_GPIO_CS_2 = GPIO_NUM_7;         // External device 2

// I2C pin configuration (I2C0)
constexpr gpio_num_t I2C0_SDA = GPIO_NUM_6;   // Serial Data
constexpr gpio_num_t I2C0_SCL = GPIO_NUM_5;   // Serial Clock

// UART pin configurations
// UART0 (Debug/Programming)
constexpr gpio_num_t UART0_TX = GPIO_NUM_21;  // Transmit
constexpr gpio_num_t UART0_RX = GPIO_NUM_20;  // Receive

// UART1 (TMC9660 TMCL)
constexpr gpio_num_t UART1_TX = GPIO_NUM_4;   // Transmit to TMC9660
constexpr gpio_num_t UART1_RX = GPIO_NUM_3;   // Receive from TMC9660

// CAN pin configuration
constexpr gpio_num_t CAN_TX = GPIO_NUM_2;     // CAN Transmit
constexpr gpio_num_t CAN_RX = GPIO_NUM_1;     // CAN Receive
```

### Interface Specifications

| Interface | Pins | Speed | Devices | Description |
|-----------|------|-------|---------|-------------|
| **SPI2** | MOSI=11, MISO=13, SCLK=12 | Up to 10MHz | 4 devices | High-speed serial |
| **I2C0** | SDA=6, SCL=5 | Up to 400kHz | Multiple | Two-wire interface |
| **UART0** | TX=21, RX=20 | Up to 115200 | Debug/Programming | Console interface |
| **UART1** | TX=4, RX=3 | Up to 115200 | TMC9660 TMCL | Motor control |
| **CAN** | TX=2, RX=1 | Up to 1Mbps | Bus network | Vehicle communication |

## 🔧 Configuration

### SPI Configuration

```cpp
// SPI device configurations
struct SpiDeviceConfig {
    SpiDeviceId device_id;
    uint32_t clock_speed_hz;
    uint8_t mode;               // 0-3
    uint8_t bits_per_word;      // 8, 16, 32
    gpio_num_t cs_pin;
    bool cs_active_low;
    uint32_t cs_setup_time_ns;
    uint32_t cs_hold_time_ns;
};

// Predefined device configurations
static const SpiDeviceConfig kSpiDeviceConfigs[] = {
    // TMC9660 Motor Controller
    {
        .device_id = SpiDeviceId::TMC9660_MOTOR_CONTROLLER,
        .clock_speed_hz = 1000000,      // 1 MHz
        .mode = 3,                      // SPI Mode 3 (CPOL=1, CPHA=1)
        .bits_per_word = 8,             // 8-bit transfers
        .cs_pin = SPI2_CS_TMC9660,
        .cs_active_low = true,
        .cs_setup_time_ns = 100,
        .cs_hold_time_ns = 100
    },
    
    // AS5047U Position Encoder
    {
        .device_id = SpiDeviceId::AS5047U_POSITION_ENCODER,
        .clock_speed_hz = 10000000,     // 10 MHz
        .mode = 1,                      // SPI Mode 1 (CPOL=0, CPHA=1)
        .bits_per_word = 16,            // 16-bit transfers
        .cs_pin = SPI2_CS_AS5047U,
        .cs_active_low = true,
        .cs_setup_time_ns = 50,
        .cs_hold_time_ns = 50
    }
};
```

### I2C Configuration

```cpp
// I2C device configurations
struct I2cDeviceConfig {
    I2cDeviceId device_id;
    uint8_t device_address;
    uint32_t clock_speed_hz;
    uint32_t timeout_ms;
    bool enable_pullups;
};

// Predefined device configurations
static const I2cDeviceConfig kI2cDeviceConfigs[] = {
    // BNO08x IMU Sensor
    {
        .device_id = I2cDeviceId::BNO08X_IMU,
        .device_address = 0x4A,         // Primary address (0x4B alternative)
        .clock_speed_hz = 400000,       // 400 kHz fast mode
        .timeout_ms = 100,
        .enable_pullups = true
    },
    
    // PCAL95555 GPIO Expander
    {
        .device_id = I2cDeviceId::PCAL95555_GPIO_EXPANDER,
        .device_address = 0x20,         // Base address (0x20-0x27 range)
        .clock_speed_hz = 400000,       // 400 kHz fast mode
        .timeout_ms = 50,
        .enable_pullups = true
    }
};
```

### UART Configuration

```cpp
// UART configurations
struct UartConfig {
    uint8_t uart_num;
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uart_parity_t parity;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint32_t rx_buffer_size;
    uint32_t tx_buffer_size;
};

// Predefined UART configurations
static const UartConfig kUartConfigs[] = {
    // UART0 - Debug/Programming
    {
        .uart_num = 0,
        .baud_rate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_DISABLE,
        .tx_pin = UART0_TX,
        .rx_pin = UART0_RX,
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024
    },
    
    // UART1 - TMC9660 TMCL
    {
        .uart_num = 1,
        .baud_rate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_DISABLE,
        .tx_pin = UART1_TX,
        .rx_pin = UART1_RX,
        .rx_buffer_size = 512,
        .tx_buffer_size = 512
    }
};
```

## 📊 Examples

### SPI Device Communication

```cpp
#include "component-handlers/CommChannelsManager.h"

void spi_communication_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Get TMC9660 SPI device
    auto* tmc_spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!tmc_spi) {
        printf("TMC9660 SPI device not available\n");
        return;
    }
    
    printf("TMC9660 SPI communication test\n");
    
    // Test basic SPI communication
    std::array<uint8_t, 8> tx_data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    std::array<uint8_t, 8> rx_data = {0};
    
    if (tmc_spi->Transfer(tx_data.data(), rx_data.data(), 8)) {
        printf("SPI transfer successful\n");
        printf("Transmitted: ");
        for (auto byte : tx_data) {
            printf("0x%02X ", byte);
        }
        printf("\nReceived: ");
        for (auto byte : rx_data) {
            printf("0x%02X ", byte);
        }
        printf("\n");
    } else {
        printf("SPI transfer failed\n");
    }
    
    // Get AS5047U SPI device
    auto* encoder_spi = comm.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (encoder_spi) {
        printf("AS5047U SPI device available\n");
        
        // 16-bit transfer example
        uint16_t cmd = 0x3FFF;  // Read angle command
        uint16_t response = 0;
        
        if (encoder_spi->Transfer(reinterpret_cast<uint8_t*>(&cmd), 
                                 reinterpret_cast<uint8_t*>(&response), 2)) {
            printf("AS5047U angle read: 0x%04X\n", response);
        }
    }
}
```

### I2C Device Communication

```cpp
void i2c_communication_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Get BNO08x I2C device
    auto* imu_i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!imu_i2c) {
        printf("BNO08x I2C device not available\n");
        return;
    }
    
    printf("BNO08x I2C communication test\n");
    
    // Test I2C device presence
    if (imu_i2c->ProbeDevice()) {
        printf("BNO08x device detected\n");
        
        // Read chip ID (example)
        uint8_t chip_id = 0;
        if (imu_i2c->ReadRegister(0x00, &chip_id, 1)) {
            printf("BNO08x chip ID: 0x%02X\n", chip_id);
        }
    } else {
        printf("BNO08x device not responding\n");
    }
    
    // Get PCAL95555 I2C device
    auto* gpio_i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    if (gpio_i2c) {
        printf("PCAL95555 I2C device available\n");
        
        // Test device presence
        if (gpio_i2c->ProbeDevice()) {
            printf("PCAL95555 device detected\n");
            
            // Read input register (example)
            uint8_t input_reg = 0;
            if (gpio_i2c->ReadRegister(0x00, &input_reg, 1)) {
                printf("PCAL95555 input register: 0x%02X\n", input_reg);
            }
        } else {
            printf("PCAL95555 device not responding\n");
        }
    }
}
```

### UART Communication

```cpp
void uart_communication_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Get UART1 for TMC9660 TMCL communication
    auto* tmcl_uart = comm.GetUart(1);
    if (!tmcl_uart) {
        printf("TMC9660 UART not available\n");
        return;
    }
    
    printf("TMC9660 UART communication test\n");
    
    // TMCL command: Get version
    std::array<uint8_t, 9> tmcl_cmd = {
        0x01,  // Sync byte
        0x88,  // Get version command
        0x00,  // Type
        0x00,  // Motor/Bank
        0x00, 0x00, 0x00, 0x00,  // Value (4 bytes)
        0x89   // Checksum
    };
    
    // Send TMCL command
    if (tmcl_uart->Write(tmcl_cmd.data(), tmcl_cmd.size())) {
        printf("TMCL command sent\n");
        
        // Wait for response
        vTaskDelay(pdMS_TO_TICKS(100));
        
        std::array<uint8_t, 9> response = {0};
        size_t received = tmcl_uart->Read(response.data(), response.size());
        
        if (received == 9) {
            printf("TMCL response received: ");
            for (auto byte : response) {
                printf("0x%02X ", byte);
            }
            printf("\n");
            
            if (response[0] == 0x02) {  // Reply sync byte
                printf("TMCL communication successful\n");
            }
        } else {
            printf("TMCL response timeout\n");
        }
    } else {
        printf("TMCL command send failed\n");
    }
    
    // Get UART0 for debug output
    auto* debug_uart = comm.GetUart(0);
    if (debug_uart) {
        std::string debug_msg = "Debug message via UART0\n";
        debug_uart->Write(reinterpret_cast<const uint8_t*>(debug_msg.c_str()), 
                         debug_msg.length());
    }
}
```

### CAN Communication

```cpp
void can_communication_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Get CAN interface
    auto* can = comm.GetCan();
    if (!can) {
        printf("CAN interface not available\n");
        return;
    }
    
    printf("CAN communication test\n");
    
    // Send CAN message
    can_message_t tx_msg = {
        .identifier = 0x123,
        .flags = 0,  // Standard frame
        .data_length_code = 8,
        .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
    };
    
    if (can->Transmit(&tx_msg)) {
        printf("CAN message transmitted: ID=0x%03lX, DLC=%u\n", 
               tx_msg.identifier, tx_msg.data_length_code);
        
        printf("Data: ");
        for (int i = 0; i < tx_msg.data_length_code; i++) {
            printf("0x%02X ", tx_msg.data[i]);
        }
        printf("\n");
    } else {
        printf("CAN transmission failed\n");
    }
    
    // Check for received messages
    can_message_t rx_msg;
    if (can->Receive(&rx_msg, 100)) {  // 100ms timeout
        printf("CAN message received: ID=0x%03lX, DLC=%u\n",
               rx_msg.identifier, rx_msg.data_length_code);
        
        printf("Data: ");
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            printf("0x%02X ", rx_msg.data[i]);
        }
        printf("\n");
    } else {
        printf("No CAN messages received\n");
    }
}
```

### Communication Statistics and Diagnostics

```cpp
void communication_diagnostics_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    printf("Communication Interface Diagnostics\n");
    printf("===================================\n");
    
    // Check interface initialization status
    printf("Interface Status:\n");
    printf("  SPI: %s\n", comm.IsSpiInitialized() ? "INITIALIZED" : "NOT INITIALIZED");
    printf("  I2C: %s\n", comm.IsI2cInitialized() ? "INITIALIZED" : "NOT INITIALIZED");
    printf("  UART0: %s\n", comm.IsUartInitialized(0) ? "INITIALIZED" : "NOT INITIALIZED");
    printf("  UART1: %s\n", comm.IsUartInitialized(1) ? "INITIALIZED" : "NOT INITIALIZED");
    printf("  CAN: %s\n", comm.IsCanInitialized() ? "INITIALIZED" : "NOT INITIALIZED");
    
    // Check device availability
    printf("\nDevice Availability:\n");
    printf("  TMC9660 SPI: %s\n", 
           comm.IsSpiDeviceAvailable(SpiDeviceId::TMC9660_MOTOR_CONTROLLER) ? "AVAILABLE" : "NOT AVAILABLE");
    printf("  AS5047U SPI: %s\n", 
           comm.IsSpiDeviceAvailable(SpiDeviceId::AS5047U_POSITION_ENCODER) ? "AVAILABLE" : "NOT AVAILABLE");
    printf("  BNO08x I2C: %s\n", 
           comm.IsI2cDeviceAvailable(I2cDeviceId::BNO08X_IMU) ? "AVAILABLE" : "NOT AVAILABLE");
    printf("  PCAL95555 I2C: %s\n", 
           comm.IsI2cDeviceAvailable(I2cDeviceId::PCAL95555_GPIO_EXPANDER) ? "AVAILABLE" : "NOT AVAILABLE");
    
    // Communication statistics
    printf("\nCommunication Statistics:\n");
    printf("  SPI transactions: %lu\n", comm.GetSpiTransactionCount());
    printf("  I2C transactions: %lu\n", comm.GetI2cTransactionCount());
    printf("  UART0 bytes: %lu\n", comm.GetUartByteCount(0));
    printf("  UART1 bytes: %lu\n", comm.GetUartByteCount(1));
    printf("  CAN messages: %lu\n", comm.GetCanMessageCount());
    
    // Test all available devices
    printf("\nDevice Communication Tests:\n");
    
    // Test SPI devices
    for (int i = 0; i < static_cast<int>(SpiDeviceId::SPI_DEVICE_COUNT); i++) {
        auto device_id = static_cast<SpiDeviceId>(i);
        auto* spi_dev = comm.GetSpiDevice(device_id);
        
        if (spi_dev) {
            printf("  SPI Device %d: ", i);
            
            // Simple test transfer
            uint8_t test_byte = 0x00;
            uint8_t response = 0xFF;
            
            if (spi_dev->Transfer(&test_byte, &response, 1)) {
                printf("COMMUNICATION OK\n");
            } else {
                printf("COMMUNICATION FAILED\n");
            }
        }
    }
    
    // Test I2C devices
    for (int i = 0; i < static_cast<int>(I2cDeviceId::I2C_DEVICE_COUNT); i++) {
        auto device_id = static_cast<I2cDeviceId>(i);
        auto* i2c_dev = comm.GetI2cDevice(device_id);
        
        if (i2c_dev) {
            printf("  I2C Device %d: ", i);
            
            if (i2c_dev->ProbeDevice()) {
                printf("DEVICE DETECTED\n");
            } else {
                printf("DEVICE NOT RESPONDING\n");
            }
        }
    }
    
    printf("\nDiagnostics complete\n");
}
```

### Performance Benchmarking

```cpp
void communication_performance_benchmark() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    printf("Communication Performance Benchmark\n");
    printf("==================================\n");
    
    // SPI performance test
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (spi) {
        printf("\nSPI Performance (TMC9660):\n");
        
        constexpr int SPI_TEST_COUNT = 1000;
        std::array<uint8_t, 8> spi_data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
        std::array<uint8_t, 8> spi_response;
        
        auto start_time = esp_timer_get_time();
        
        for (int i = 0; i < SPI_TEST_COUNT; i++) {
            spi->Transfer(spi_data.data(), spi_response.data(), 8);
        }
        
        auto end_time = esp_timer_get_time();
        float avg_time = (end_time - start_time) / (float)SPI_TEST_COUNT;
        float throughput = (8.0f * SPI_TEST_COUNT * 1000000.0f) / (end_time - start_time);
        
        printf("  Average transfer time: %.2f µs\n", avg_time);
        printf("  Throughput: %.1f bytes/sec\n", throughput);
        printf("  Transfers per second: %.0f\n", 1000000.0f / avg_time);
    }
    
    // I2C performance test
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    if (i2c) {
        printf("\nI2C Performance (PCAL95555):\n");
        
        constexpr int I2C_TEST_COUNT = 1000;
        uint8_t i2c_data = 0x00;
        uint8_t i2c_response;
        
        auto start_time = esp_timer_get_time();
        
        for (int i = 0; i < I2C_TEST_COUNT; i++) {
            i2c->ReadRegister(0x00, &i2c_response, 1);
        }
        
        auto end_time = esp_timer_get_time();
        float avg_time = (end_time - start_time) / (float)I2C_TEST_COUNT;
        
        printf("  Average read time: %.2f µs\n", avg_time);
        printf("  Reads per second: %.0f\n", 1000000.0f / avg_time);
    }
    
    // UART performance test
    auto* uart = comm.GetUart(1);
    if (uart) {
        printf("\nUART Performance (UART1):\n");
        
        std::string test_string = "Performance test message\n";
        constexpr int UART_TEST_COUNT = 100;
        
        auto start_time = esp_timer_get_time();
        
        for (int i = 0; i < UART_TEST_COUNT; i++) {
            uart->Write(reinterpret_cast<const uint8_t*>(test_string.c_str()), 
                       test_string.length());
        }
        
        auto end_time = esp_timer_get_time();
        float total_bytes = test_string.length() * UART_TEST_COUNT;
        float throughput = (total_bytes * 1000000.0f) / (end_time - start_time);
        
        printf("  Total bytes transmitted: %.0f\n", total_bytes);
        printf("  Throughput: %.1f bytes/sec\n", throughput);
        printf("  Baud rate utilization: %.1f%%\n", (throughput * 10.0f) / 115200.0f);
    }
    
    printf("\nPerformance benchmark complete\n");
}
```

## 🔍 Advanced Usage

### Custom Device Configuration

```cpp
// Example of adding custom device configurations
class CustomCommManager : public CommChannelsManager {
public:
    // Add custom SPI device
    bool AddCustomSpiDevice(gpio_num_t cs_pin, uint32_t clock_hz, uint8_t mode) {
        // Implementation would add device to internal configuration
        return true;
    }
    
    // Add custom I2C device
    bool AddCustomI2cDevice(uint8_t address, uint32_t clock_hz) {
        // Implementation would add device to internal configuration
        return true;
    }
};
```

### Multi-Device Communication

```cpp
void multi_device_communication_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Simultaneous communication with multiple devices
    auto* tmc_spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    auto* encoder_spi = comm.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    auto* imu_i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    auto* gpio_i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    
    printf("Multi-device communication test\n");
    
    for (int i = 0; i < 100; i++) {
        // Read motor controller status
        if (tmc_spi) {
            std::array<uint8_t, 8> tmc_cmd = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            std::array<uint8_t, 8> tmc_response;
            tmc_spi->Transfer(tmc_cmd.data(), tmc_response.data(), 8);
        }
        
        // Read encoder position
        if (encoder_spi) {
            uint16_t encoder_cmd = 0x3FFF;
            uint16_t encoder_response;
            encoder_spi->Transfer(reinterpret_cast<uint8_t*>(&encoder_cmd),
                                 reinterpret_cast<uint8_t*>(&encoder_response), 2);
        }
        
        // Read IMU data
        if (imu_i2c) {
            uint8_t imu_data[6];
            imu_i2c->ReadRegister(0x1A, imu_data, 6);  // Example register
        }
        
        // Update GPIO expander
        if (gpio_i2c) {
            uint8_t gpio_state = (i % 2) ? 0xFF : 0x00;
            gpio_i2c->WriteRegister(0x01, &gpio_state, 1);  // Output register
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    printf("Multi-device communication complete\n");
}
```

## 🚨 Error Handling

### Comprehensive Error Management

```cpp
void comprehensive_error_handling() {
    auto& comm = CommChannelsManager::GetInstance();
    
    // Test initialization
    if (!comm.EnsureInitialized()) {
        printf("ERROR: Communication manager initialization failed\n");
        return;
    }
    
    printf("Communication error handling test\n");
    
    // Test each interface with error checking
    
    // SPI error handling
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (spi) {
        uint8_t test_data = 0x00;
        uint8_t response = 0xFF;
        
        for (int i = 0; i < 10; i++) {
            if (!spi->Transfer(&test_data, &response, 1)) {
                printf("ERROR: SPI transfer failed on attempt %d\n", i + 1);
            }
        }
    } else {
        printf("ERROR: SPI device not available\n");
    }
    
    // I2C error handling
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (i2c) {
        uint8_t reg_data;
        
        for (int i = 0; i < 10; i++) {
            if (!i2c->ReadRegister(0x00, &reg_data, 1)) {
                printf("ERROR: I2C read failed on attempt %d\n", i + 1);
            }
        }
    } else {
        printf("ERROR: I2C device not available\n");
    }
    
    // UART error handling
    auto* uart = comm.GetUart(1);
    if (uart) {
        std::string test_msg = "Test message\n";
        
        for (int i = 0; i < 10; i++) {
            size_t written = uart->Write(reinterpret_cast<const uint8_t*>(test_msg.c_str()),
                                        test_msg.length());
            if (written != test_msg.length()) {
                printf("ERROR: UART write incomplete on attempt %d (%zu/%zu bytes)\n", 
                       i + 1, written, test_msg.length());
            }
        }
    } else {
        printf("ERROR: UART device not available\n");
    }
    
    // Interface recovery test
    printf("Testing interface recovery...\n");
    
    // Deinitialize and reinitialize
    if (comm.EnsureDeinitialized()) {
        printf("Communication manager deinitialized\n");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        if (comm.EnsureInitialized()) {
            printf("Communication manager reinitialized successfully\n");
        } else {
            printf("ERROR: Failed to reinitialize communication manager\n");
        }
    } else {
        printf("ERROR: Failed to deinitialize communication manager\n");
    }
    
    printf("Error handling test complete\n");
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver
- **[Hardware Setup Guide](../hardware/HARDWARE_SETUP.md)** - Hardware configuration

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*