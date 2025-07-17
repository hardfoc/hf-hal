/**
 * @file EnumeratedSpiDeviceExample.cpp
 * @brief Example demonstrating enumeration-based SPI device access.
 *
 * This example shows how to use the SpiDeviceId enumeration and convenience
 * methods to access specific SPI devices in a type-safe and intuitive way.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "component-handers/CommChannelsManager.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
#include <cstdint>
#include <array>

/**
 * @brief Example: Access devices using enumeration-based approach.
 */
void ExampleEnumerationBasedAccess() {
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    if (!comm_mgr.EnsureInitialized()) {
        // Handle initialization failure
        return;
    }

    // Method 1: Using SpiDeviceId enumeration
    BaseSpi* motor_controller = comm_mgr.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    BaseSpi* position_encoder = comm_mgr.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    BaseSpi* external_dev1 = comm_mgr.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_1);
    BaseSpi* external_dev2 = comm_mgr.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_2);

    // Method 2: Using convenience methods (recommended for clarity)
    BaseSpi* motor = comm_mgr.GetMotorController();
    BaseSpi* encoder = comm_mgr.GetPositionEncoder();
    BaseSpi* ext1 = comm_mgr.GetExternalDevice1();
    BaseSpi* ext2 = comm_mgr.GetExternalDevice2();

    // Method 3: Using aliases for common devices
    BaseSpi* motor_alias = comm_mgr.GetSpiDevice(SpiDeviceId::MOTOR_CONTROLLER);
    BaseSpi* encoder_alias = comm_mgr.GetSpiDevice(SpiDeviceId::POSITION_ENCODER);

    // Check if devices are available before using
    if (motor_controller) {
        // TMC9660 motor controller communication
        // This device uses SPI Mode 3 (configured automatically)
        std::array<uint8_t, 4> tmc_tx_data = {0x00, 0x00, 0x00, 0x00}; // Example register read
        std::array<uint8_t, 4> tmc_rx_data = {};
        
        auto result = motor_controller->Transfer(
            tmc_tx_data.data(), 
            tmc_rx_data.data(), 
            tmc_tx_data.size()
        );
        
        if (result == hf_spi_err_t::HF_SPI_OK) {
            // Process TMC9660 response
        }
    }

    if (position_encoder) {
        // AS5047U position encoder communication
        // This device uses SPI Mode 1 (configured automatically)
        std::array<uint8_t, 2> as5047_cmd = {0xFF, 0xFF}; // Example angle read command
        std::array<uint8_t, 2> as5047_response = {};
        
        auto result = position_encoder->Transfer(
            as5047_cmd.data(), 
            as5047_response.data(), 
            as5047_cmd.size()
        );
        
        if (result == hf_spi_err_t::HF_SPI_OK) {
            // Extract 14-bit angle from response
            uint16_t angle_raw = ((as5047_response[0] & 0x3F) << 8) | as5047_response[1];
            // Convert to degrees, handle parity, etc.
        }
    }
}

/**
 * @brief Example: Access ESP-specific device features.
 */
void ExampleEspSpecificAccess() {
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    if (!comm_mgr.EnsureInitialized()) {
        return;
    }

    // Get ESP-specific device interfaces for advanced features
    EspSpiDevice* esp_motor = comm_mgr.GetEspMotorController();
    EspSpiDevice* esp_encoder = comm_mgr.GetEspPositionEncoder();

    if (esp_motor) {
        // Use ESP-specific features like bus acquisition
        auto acquire_result = esp_motor->AcquireBus(1000); // 1 second timeout
        if (acquire_result == hf_spi_err_t::HF_SPI_OK) {
            
            // Perform multiple transactions without bus overhead
            std::array<uint8_t, 4> cmd1 = {0x01, 0x00, 0x00, 0x00};
            std::array<uint8_t, 4> resp1 = {};
            esp_motor->Transfer(cmd1.data(), resp1.data(), cmd1.size());
            
            std::array<uint8_t, 4> cmd2 = {0x02, 0x00, 0x00, 0x00};
            std::array<uint8_t, 4> resp2 = {};
            esp_motor->Transfer(cmd2.data(), resp2.data(), cmd2.size());
            
            // Release the bus
            esp_motor->ReleaseBus();
        }

        // Get actual clock frequency for diagnostics
        uint32_t actual_freq = 0;
        auto freq_result = esp_motor->GetActualClockFrequency(actual_freq);
        if (freq_result == hf_spi_err_t::HF_SPI_OK) {
            // Log or use actual frequency information
        }
    }
}

/**
 * @brief Example: Iterate through all devices by enumeration.
 */
void ExampleIterateDevices() {
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    if (!comm_mgr.EnsureInitialized()) {
        return;
    }

    // Iterate through all defined device types
    for (int i = 0; i < static_cast<int>(SpiDeviceId::SPI_DEVICE_COUNT); ++i) {
        SpiDeviceId device_id = static_cast<SpiDeviceId>(i);
        BaseSpi* device = comm_mgr.GetSpiDevice(device_id);
        
        if (device) {
            // Device is available, perform operations
            switch (device_id) {
                case SpiDeviceId::TMC9660_MOTOR_CONTROLLER:
                    // Handle TMC9660-specific operations
                    break;
                    
                case SpiDeviceId::AS5047U_POSITION_ENCODER:
                    // Handle AS5047U-specific operations
                    break;
                    
                case SpiDeviceId::EXTERNAL_DEVICE_1:
                case SpiDeviceId::EXTERNAL_DEVICE_2:
                    // Handle external device operations
                    break;
                    
                default:
                    break;
            }
        }
    }
}

/**
 * @brief Example: Device-specific configuration and usage patterns.
 */
void ExampleDeviceSpecificUsage() {
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    if (!comm_mgr.EnsureInitialized()) {
        return;
    }

    // TMC9660 Motor Controller Example
    // - Uses SPI Mode 3 (CPOL=1, CPHA=1)
    // - 32-bit register access
    // - Typical frequency: 1-10 MHz
    if (auto* tmc = comm_mgr.GetMotorController()) {
        // Read GCONF register (0x00)
        std::array<uint8_t, 5> tmc_read_cmd = {0x00, 0x00, 0x00, 0x00, 0x00};
        std::array<uint8_t, 5> tmc_read_resp = {};
        
        tmc->Transfer(tmc_read_cmd.data(), tmc_read_resp.data(), 5);
        
        // Write GCONF register
        std::array<uint8_t, 5> tmc_write_cmd = {0x80, 0x00, 0x00, 0x00, 0x01}; // Set some bit
        tmc->Transfer(tmc_write_cmd.data(), nullptr, 5);
    }

    // AS5047U Position Encoder Example
    // - Uses SPI Mode 1 (CPOL=0, CPHA=1)
    // - 16-bit commands and responses
    // - Typical frequency: 1-10 MHz
    if (auto* encoder = comm_mgr.GetPositionEncoder()) {
        // Read angle register (0x3FFF with parity)
        std::array<uint8_t, 2> angle_cmd = {0x3F, 0xFF};
        std::array<uint8_t, 2> angle_resp = {};
        
        encoder->Transfer(angle_cmd.data(), angle_resp.data(), 2);
        
        // Extract 14-bit angle value
        uint16_t raw_angle = ((angle_resp[0] & 0x3F) << 8) | angle_resp[1];
        float angle_degrees = (raw_angle * 360.0f) / 16384.0f; // 14-bit resolution
    }
}

int main() {
    // Demonstrate different access patterns
    ExampleEnumerationBasedAccess();
    ExampleEspSpecificAccess();
    ExampleIterateDevices();
    ExampleDeviceSpecificUsage();
    
    return 0;
}
