/**
 * @file SpiUsageExample.cpp
 * @brief Example demonstrating the improved SPI structure with reference access and ESP-specific API.
 * 
 * This example shows how to use the CommChannelsManager and EspSpiBus 
 * with the new improved device management structure, including reference access
 * and ESP-specific functionality.
 */

#include "CommChannelsManager.h"

// Example class that holds a reference to an SPI device
class MotorController {
private:
    BaseSpi& spi_device_;
    
public:
    explicit MotorController(BaseSpi& spi_device) : spi_device_(spi_device) {}
    
    bool SendCommand(uint8_t command, uint16_t value) {
        uint8_t tx_data[3] = {command, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
        uint8_t rx_data[3];
        
        auto result = spi_device_.Transfer(tx_data, rx_data, 3);
        return result == hf_spi_err_t::SPI_SUCCESS;
    }
};

// Example class that needs ESP-specific features
class AdvancedSpiController {
private:
    EspSpiDevice& esp_spi_device_;
    
public:
    explicit AdvancedSpiController(EspSpiDevice& esp_device) : esp_spi_device_(esp_device) {}
    
    bool ConfigureAdvanced() {
        // Access ESP-specific features
        spi_device_handle_t handle = esp_spi_device_.GetHandle();
        const auto& config = esp_spi_device_.GetConfig();
        
        // Get actual clock frequency
        uint32_t actual_freq;
        if (esp_spi_device_.GetActualClockFrequency(actual_freq) == hf_spi_err_t::SPI_SUCCESS) {
            // Use actual frequency for timing calculations
            printf("Actual SPI frequency: %lu Hz\n", actual_freq);
        }
        
        // Acquire bus for exclusive access during critical operations
        if (esp_spi_device_.AcquireBus(1000) == hf_spi_err_t::SPI_SUCCESS) {
            // Perform multiple transactions without bus switching overhead
            uint8_t cmd1[] = {0x01, 0x02};
            uint8_t cmd2[] = {0x03, 0x04};
            
            esp_spi_device_.Transfer(cmd1, nullptr, 2);
            esp_spi_device_.Transfer(cmd2, nullptr, 2);
            
            // Release bus when done
            esp_spi_device_.ReleaseBus();
        }
        
        return true;
    }
};

void example_spi_usage() {
    // Get the singleton instance
    CommChannelsManager& comm = CommChannelsManager::GetInstance();
    
    // Initialize all communication channels
    if (!comm.EnsureInitialized()) {
        printf("Failed to initialize communication channels\n");
        return;
    }
    
    //==========================================================
    // APPROACH 1: Pointer access (safe, can check for nullptr)
    //==========================================================
    
    BaseSpi* tmc9660_device = comm.GetSpiDevice(0);
    if (tmc9660_device) {
        uint8_t tx_data[] = {0x01, 0x02, 0x03};
        uint8_t rx_data[3];
        
        auto result = tmc9660_device->Transfer(tx_data, rx_data, 3);
        if (result == hf_spi_err_t::SPI_SUCCESS) {
            printf("TMC9660 communication successful\n");
        }
    } else {
        printf("TMC9660 device not found\n");
    }
    
    //==========================================================
    // APPROACH 2: Simple device access (now using regular GetSpiDevice)
    //==========================================================
    
    // Get device as pointer - no exceptions thrown
    BaseSpi* encoder_device = comm.GetSpiDevice(2); // AS5047 encoder
    if (encoder_device) {
        // Create a motor controller that holds a reference
        MotorController motor_ctrl(*encoder_device);
        
        // Use the motor controller
        if (motor_ctrl.SendCommand(0x3F, 0xFFFF)) {
            printf("Motor controller command sent successfully\n");
        }
        
    } else {
        printf("Device index 2 not found or invalid\n");
    }
    
    //==========================================================
    // APPROACH 3: ESP-specific API access
    //==========================================================
    
    // Get ESP-specific device for advanced features - no exceptions
    EspSpiDevice* esp_device = comm.GetEspSpiDevice(0);
    if (esp_device) {
        // Create controller that uses ESP-specific features
        AdvancedSpiController advanced_ctrl(*esp_device);
        advanced_ctrl.ConfigureAdvanced();
        
        // Direct ESP-specific operations
        uint32_t actual_freq;
        if (esp_device->GetActualClockFrequency(actual_freq) == hf_spi_err_t::SPI_SUCCESS) {
            printf("Device 0 actual frequency: %lu Hz\n", actual_freq);
        }
        
        // Access configuration
        const auto& config = esp_device->GetConfig();
        printf("Device CS pin: %d\n", config.cs_pin);
        printf("Device mode: %d\n", static_cast<int>(config.mode));
        
    } else {
        printf("ESP device access error: Device not found\n");
    }
    
    //==========================================================
    // APPROACH 4: Bus-level management
    //==========================================================
    
    EspSpiBus& spi_bus = comm.GetSpiBus();
    
    // Check device count
    printf("Total SPI devices: %zu\n", spi_bus.GetDeviceCount());
    
    // Iterate through all devices
    for (int i = 0; i < static_cast<int>(spi_bus.GetDeviceCount()); ++i) {
        BaseSpi* device = spi_bus.GetDevice(i);
        if (device && device->IsInitialized()) {
            printf("Device %d is initialized and ready\n", i);
        }
    }
    
    // Create a new device dynamically
    hf_spi_device_config_t new_config = {};
    new_config.cs_pin = 25;
    new_config.clock_speed_hz = 1000000; // 1 MHz
    new_config.mode = hf_spi_mode_t::HF_SPI_MODE_0;
    new_config.queue_size = 3;
    
    int new_device_index = spi_bus.CreateDevice(new_config);
    if (new_device_index >= 0) {
        printf("Created new device at index %d\n", new_device_index);
        
        // Use the new device
        BaseSpi* new_device = spi_bus.GetDevice(new_device_index);
        if (new_device) {
            uint8_t test_data[] = {0xAA, 0x55};
            uint8_t response[2];
            new_device->Transfer(test_data, response, 2);
            
            // Access ESP-specific features of the new device
            EspSpiDevice* esp_new_device = spi_bus.GetEspDevice(new_device_index);
            if (esp_new_device) {
                spi_device_handle_t handle = esp_new_device->GetHandle();
                printf("New device handle: %p\n", handle);
            }
            
        } else {
            printf("Error using new device: Device not found\n");
        }
    }
}

void example_device_specific_usage() {
    CommChannelsManager& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    //==========================================================
    // TMC9660 Motor Controller (Device Index 0)
    //==========================================================
    EspSpiDevice* tmc9660 = comm.GetEspSpiDevice(0);
    if (tmc9660) {
        // TMC9660 needs specific timing - use ESP features
        uint32_t actual_freq;
        tmc9660->GetActualClockFrequency(actual_freq);
        printf("TMC9660 actual SPI frequency: %lu Hz\n", actual_freq);
        
        // Read TMC9660 version
        uint8_t read_version[] = {0x00, 0x00, 0x00, 0x00}; // Read register 0
        uint8_t version_response[4];
        
        if (tmc9660->Transfer(read_version, version_response, 4) == hf_spi_err_t::SPI_SUCCESS) {
            uint32_t version = (version_response[1] << 16) | (version_response[2] << 8) | version_response[3];
            printf("TMC9660 version: 0x%06lX\n", version);
        }
        
    } else {
        printf("TMC9660 error: Device not found\n");
    }
    
    //==========================================================
    // AS5047 Position Encoder (Device Index 2)
    //==========================================================
    BaseSpi* as5047 = comm.GetSpiDevice(2);
    if (as5047) {
        // Read angle from AS5047
        uint8_t read_angle[] = {0x3F, 0xFF}; // Read angle command
        uint8_t angle_response[2];
        
        if (as5047->Transfer(read_angle, angle_response, 2) == hf_spi_err_t::SPI_SUCCESS) {
            uint16_t raw_angle = (angle_response[0] << 8) | angle_response[1];
            raw_angle &= 0x3FFF; // 14-bit angle
            float angle_degrees = (raw_angle * 360.0f) / 16384.0f;
            printf("AS5047 angle: %.2f degrees\n", angle_degrees);
        }
        
    } else {
        printf("AS5047 error: Device not found\n");
    }
}

// Example of setting up class members with SPI references
class SystemController {
private:
    BaseSpi* motor_spi_;
    BaseSpi* encoder_spi_;
    EspSpiDevice* advanced_spi_;
    
public:
    SystemController(BaseSpi* motor, BaseSpi* encoder, EspSpiDevice* advanced)
        : motor_spi_(motor), encoder_spi_(encoder), advanced_spi_(advanced) {
    }
    
    void RunControlLoop() {
        if (!motor_spi_ || !encoder_spi_ || !advanced_spi_) {
            printf("Invalid SPI device pointers\n");
            return;
        }
        
        // Read encoder position
        uint8_t encoder_cmd[] = {0x3F, 0xFF};
        uint8_t encoder_data[2];
        encoder_spi_->Transfer(encoder_cmd, encoder_data, 2);
        
        // Send motor command based on position
        uint8_t motor_cmd[] = {0x01, 0x02, 0x03};
        motor_spi_->Transfer(motor_cmd, nullptr, 3);
        
        // Use advanced ESP features for diagnostics
        uint32_t freq;
        if (advanced_spi_->GetActualClockFrequency(freq) == hf_spi_err_t::SPI_SUCCESS) {
            // Log frequency for diagnostics
        }
    }
};

void example_system_integration() {
    CommChannelsManager& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Get devices - check for nullptr
    BaseSpi* motor_device = comm.GetSpiDevice(0);      // TMC9660 motor controller
    BaseSpi* encoder_device = comm.GetSpiDevice(2);     // AS5047 encoder
    EspSpiDevice* advanced_device = comm.GetEspSpiDevice(1); // Advanced device for diagnostics
    
    if (motor_device && encoder_device && advanced_device) {
        // Create system controller with pointers to devices
        SystemController system(motor_device, encoder_device, advanced_device);
        
        // Run control loop
        system.RunControlLoop();
        
    } else {
        printf("System integration error: One or more devices not found\n");
    }
}
