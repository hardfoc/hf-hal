/**
 * @file comm_bus_example.cpp
 * @brief Example demonstrating the new communication bus abstractions
 * 
 * This example shows how to use the new I2C, SPI, and UART abstractions
 * in both modern and legacy compatibility modes.
 */

#include "mcu/McuI2cBus.h"
#include "mcu/McuSpiBus.h"
#include "mcu/McuUartDriver.h"

// Legacy compatibility includes (for demonstration)
#include "drivers/I2cBus.h"
#include "drivers/SpiBus.h"
#include "drivers/UartDriver.h"

#include <iostream>

void modern_i2c_example() {
    std::cout << "\n=== Modern I2C Example ===\n";
    
    // Configure I2C bus
    I2cBusConfig i2c_config;
    i2c_config.port = 0;
    i2c_config.sda_pin = 21;
    i2c_config.scl_pin = 22;
    i2c_config.clock_speed_hz = 400000;  // 400kHz fast mode
    i2c_config.enable_pullups = true;
    i2c_config.timeout_ms = 1000;
    
    // Create MCU I2C bus instance
    McuI2cBus i2c_bus(i2c_config);
    
    // Initialize and use
    if (i2c_bus.EnsureInitialized()) {
        std::cout << "I2C bus initialized successfully\n";
        
        // Write data to device at address 0x48
        uint8_t write_data[] = {0x01, 0x02, 0x03};
        HfI2cErr result = i2c_bus.Write(0x48, write_data, sizeof(write_data));
        
        if (result == HfI2cErr::I2C_SUCCESS) {
            std::cout << "Write successful\n";
        } else {
            std::cout << "Write failed: " << static_cast<int>(result) << "\n";
        }
        
        // Read data from device
        uint8_t read_data[4];
        result = i2c_bus.Read(0x48, read_data, sizeof(read_data));
        
        if (result == HfI2cErr::I2C_SUCCESS) {
            std::cout << "Read successful\n";
        } else {
            std::cout << "Read failed: " << static_cast<int>(result) << "\n";
        }
        
        // Write then read (register access)
        uint8_t reg_addr = 0x10;
        uint8_t reg_data[2];
        result = i2c_bus.WriteRead(0x48, &reg_addr, 1, reg_data, sizeof(reg_data));
        
        if (result == HfI2cErr::I2C_SUCCESS) {
            std::cout << "Register read successful\n";
        }
        
        // Scan for devices
        uint8_t found_addresses[10];
        uint8_t device_count = i2c_bus.ScanBus(found_addresses, sizeof(found_addresses));
        std::cout << "Found " << static_cast<int>(device_count) << " I2C devices\n";
    } else {
        std::cout << "I2C initialization failed\n";
    }
}

void modern_spi_example() {
    std::cout << "\n=== Modern SPI Example ===\n";
    
    // Configure SPI bus
    SpiBusConfig spi_config;
    spi_config.host = 1;
    spi_config.mosi_pin = 23;
    spi_config.miso_pin = 19;
    spi_config.sclk_pin = 18;
    spi_config.cs_pin = 5;
    spi_config.clock_speed_hz = 1000000;  // 1MHz
    spi_config.mode = 0;  // Mode 0 (CPOL=0, CPHA=0)
    spi_config.bits_per_word = 8;
    spi_config.cs_active_low = true;
    
    // Create MCU SPI bus instance
    McuSpiBus spi_bus(spi_config);
    
    // Initialize and use
    if (spi_bus.EnsureInitialized()) {
        std::cout << "SPI bus initialized successfully\n";
        
        // Full-duplex transfer
        uint8_t tx_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
        uint8_t rx_data[4];
        
        HfSpiErr result = spi_bus.Transfer(tx_data, rx_data, sizeof(tx_data));
        
        if (result == HfSpiErr::SPI_SUCCESS) {
            std::cout << "SPI transfer successful\n";
        } else {
            std::cout << "SPI transfer failed: " << static_cast<int>(result) << "\n";
        }
        
        // Write-only transfer
        result = spi_bus.Write(tx_data, sizeof(tx_data));
        
        if (result == HfSpiErr::SPI_SUCCESS) {
            std::cout << "SPI write successful\n";
        }
        
        // Read-only transfer
        result = spi_bus.Read(rx_data, sizeof(rx_data));
        
        if (result == HfSpiErr::SPI_SUCCESS) {
            std::cout << "SPI read successful\n";
        }
        
        // Manual CS control for multiple transfers
        spi_bus.SetChipSelect(true);
        spi_bus.Write(tx_data, 2);
        spi_bus.Read(rx_data, 2);
        spi_bus.SetChipSelect(false);
        
        std::cout << "Manual CS control completed\n";
    } else {
        std::cout << "SPI initialization failed\n";
    }
}

void modern_uart_example() {
    std::cout << "\n=== Modern UART Example ===\n";
    
    // Configure UART
    UartConfig uart_config;
    uart_config.baud_rate = 115200;
    uart_config.data_bits = 8;
    uart_config.parity = 0;  // No parity
    uart_config.stop_bits = 1;
    uart_config.use_hardware_flow_control = false;
    uart_config.tx_pin = 1;
    uart_config.rx_pin = 3;
    uart_config.tx_buffer_size = 512;
    uart_config.rx_buffer_size = 512;
    uart_config.timeout_ms = 1000;
    
    // Create MCU UART driver instance
    McuUartDriver uart(2, uart_config);  // UART port 2
    
    // Initialize and use
    if (uart.EnsureInitialized()) {
        std::cout << "UART initialized successfully\n";
        
        // Write string
        const char* message = "Hello, UART!\n";
        HfUartErr result = uart.Write(reinterpret_cast<const uint8_t*>(message), 
                                     strlen(message));
        
        if (result == HfUartErr::UART_SUCCESS) {
            std::cout << "UART write successful\n";
        } else {
            std::cout << "UART write failed: " << static_cast<int>(result) << "\n";
        }
        
        // Printf-style output
        int chars_written = uart.Printf("Temperature: %d°C, Status: %s\n", 25, "OK");
        if (chars_written > 0) {
            std::cout << "Printf wrote " << chars_written << " characters\n";
        }
        
        // Check for available data
        uint16_t available = uart.BytesAvailable();
        if (available > 0) {
            std::cout << available << " bytes available for reading\n";
            
            // Read available data
            uint8_t read_buffer[100];
            result = uart.Read(read_buffer, std::min(available, static_cast<uint16_t>(sizeof(read_buffer))));
            
            if (result == HfUartErr::UART_SUCCESS) {
                std::cout << "UART read successful\n";
            }
        }
        
        // Read a line of text
        char line_buffer[80];
        uint16_t line_length = uart.ReadLine(line_buffer, sizeof(line_buffer), 5000);
        if (line_length > 0) {
            std::cout << "Read line: " << line_buffer << "\n";
        }
    } else {
        std::cout << "UART initialization failed\n";
    }
}

void legacy_compatibility_example() {
    std::cout << "\n=== Legacy Compatibility Example ===\n";
    
    // These now use the new implementations under the hood
    // but maintain the old API for backward compatibility
    
    // Legacy I2C usage
    I2cBusConfig i2c_config;
    i2c_config.port = 0;
    i2c_config.sda_pin = 21;
    i2c_config.scl_pin = 22;
    i2c_config.clock_speed_hz = 100000;
    
    I2cBus legacy_i2c(i2c_config);  // Actually creates McuI2cBus
    if (legacy_i2c.Open()) {
        std::cout << "Legacy I2C opened successfully\n";
        
        // Old boolean API still works
        if (legacy_i2c.WriteByte(0x48, 0x42)) {
            std::cout << "Legacy I2C write successful\n";
        }
        
        uint8_t data;
        if (legacy_i2c.ReadByte(0x48, data)) {
            std::cout << "Legacy I2C read successful, data: 0x" << std::hex << static_cast<int>(data) << "\n";
        }
        
        legacy_i2c.Close();
    }
    
    // Legacy SPI usage
    SpiBusConfig spi_config;
    spi_config.host = 1;
    spi_config.mosi_pin = 23;
    spi_config.miso_pin = 19;
    spi_config.sclk_pin = 18;
    spi_config.clock_speed_hz = 1000000;
    
    SpiBus legacy_spi(spi_config);  // Actually creates McuSpiBus
    if (legacy_spi.Open()) {
        std::cout << "Legacy SPI opened successfully\n";
        
        uint8_t tx_data[] = {0x01, 0x02};
        uint8_t rx_data[2];
        
        if (legacy_spi.Transfer(tx_data, rx_data, sizeof(tx_data))) {
            std::cout << "Legacy SPI transfer successful\n";
        }
        
        legacy_spi.Close();
    }
    
    // Legacy UART usage
    UartConfig uart_config;
    uart_config.baud_rate = 115200;
    uart_config.tx_pin = 1;
    uart_config.rx_pin = 3;
    
    UartDriver legacy_uart(2, uart_config);  // Actually creates McuUartDriver
    if (legacy_uart.Open()) {
        std::cout << "Legacy UART opened successfully\n";
        
        if (legacy_uart.WriteString("Legacy UART test\n")) {
            std::cout << "Legacy UART write successful\n";
        }
        
        // Old Printf API still works
        legacy_uart.Printf("Counter: %d\n", 123);
        
        legacy_uart.Close();
    }
}

int main() {
    std::cout << "Communication Bus Abstraction Examples\n";
    std::cout << "=====================================\n";
    
    // Demonstrate modern API usage
    modern_i2c_example();
    modern_spi_example();
    modern_uart_example();
    
    // Demonstrate legacy compatibility
    legacy_compatibility_example();
    
    std::cout << "\nAll examples completed!\n";
    return 0;
}
