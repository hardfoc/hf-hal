/**
 * @file advanced_comm_example.cpp
 * @brief Advanced example demonstrating communication bus coordination.
 *
 * This example shows how to use multiple communication buses (I2C, SPI, UART)
 * together in a real-world application scenario, demonstrating:
 * - Multi-sensor data acquisition
 * - Inter-device communication
 * - Error handling and recovery
 * - Thread-safe operations
 * - Performance optimization
 */

#include "mcu/All.h"  // Includes all MCU implementations
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <memory>

/**
 * @brief Example application class using multiple communication buses
 */
class MultiSensorSystem {
public:
    MultiSensorSystem();
    ~MultiSensorSystem() = default;
    
    bool Initialize();
    void Run();
    void Shutdown();
    
    // Individual sensor interfaces
    bool ReadTemperatureSensor();      // I2C
    bool ConfigureAccelerometer();     // SPI
    bool SendDataToHost();            // UART
    
    // System-wide operations
    void DataAcquisitionLoop();
    void CommunicationLoop();
    
private:
    // Communication bus instances
    std::unique_ptr<McuI2cBus> i2cBus_;
    std::unique_ptr<McuSpiBus> spiBus_;
    std::unique_ptr<McuUartDriver> uartDriver_;
    
    // System state
    std::atomic<bool> running_;
    std::atomic<float> temperature_;
    std::atomic<int16_t> accelerometer_[3];  // X, Y, Z
    
    // Thread management
    std::thread dataThread_;
    std::thread commThread_;
    
    // Configuration
    static constexpr uint8_t TEMP_SENSOR_ADDR = 0x48;  // TMP102 temperature sensor
    static constexpr uint8_t ACCEL_REGISTER = 0x32;    // Accelerometer data register
    static constexpr uint32_t SAMPLE_RATE_MS = 100;    // 10Hz sampling
};

MultiSensorSystem::MultiSensorSystem() : running_(false), temperature_(0.0f) {
    accelerometer_[0] = accelerometer_[1] = accelerometer_[2] = 0;
}

bool MultiSensorSystem::Initialize() {
    std::cout << "Initializing Multi-Sensor System..." << std::endl;
    
    // Configure I2C for temperature sensor
    I2cBusConfig i2cConfig;
    i2cConfig.port = 0;
    i2cConfig.sda_pin = 21;
    i2cConfig.scl_pin = 22;
    i2cConfig.clock_speed_hz = 400000;  // 400kHz fast mode
    i2cConfig.enable_internal_pullups = true;
    i2cConfig.timeout_ms = 1000;
    
    i2cBus_ = std::make_unique<McuI2cBus>(i2cConfig);
    if (!i2cBus_->EnsureInitialized()) {
        std::cerr << "Failed to initialize I2C bus" << std::endl;
        return false;
    }
    
    // Configure SPI for accelerometer
    SpiBusConfig spiConfig;
    spiConfig.mosi_pin = 23;
    spiConfig.miso_pin = 19;
    spiConfig.sclk_pin = 18;
    spiConfig.cs_pin = 5;
    spiConfig.clock_speed_hz = 1000000;  // 1MHz
    spiConfig.mode = 0;  // CPOL=0, CPHA=0
    spiConfig.word_size = 8;
    spiConfig.auto_cs = true;
    
    spiBus_ = std::make_unique<McuSpiBus>(spiConfig);
    if (!spiBus_->EnsureInitialized()) {
        std::cerr << "Failed to initialize SPI bus" << std::endl;
        return false;
    }
    
    // Configure UART for host communication
    UartConfig uartConfig;
    uartConfig.uart_port = 2;
    uartConfig.tx_pin = 17;
    uartConfig.rx_pin = 16;
    uartConfig.baud_rate = 115200;
    uartConfig.data_bits = 8;
    uartConfig.parity = UartParity::UART_PARITY_NONE;
    uartConfig.stop_bits = 1;
    uartConfig.flow_control = UartFlowControl::UART_FLOW_CONTROL_NONE;
    
    uartDriver_ = std::make_unique<McuUartDriver>(uartConfig);
    if (!uartDriver_->EnsureInitialized()) {
        std::cerr << "Failed to initialize UART" << std::endl;
        return false;
    }
    
    // Initialize sensors
    if (!ConfigureAccelerometer()) {
        std::cerr << "Failed to configure accelerometer" << std::endl;
        return false;
    }
    
    std::cout << "Multi-Sensor System initialized successfully" << std::endl;
    return true;
}

void MultiSensorSystem::Run() {
    running_ = true;
    
    // Start data acquisition thread
    dataThread_ = std::thread(&MultiSensorSystem::DataAcquisitionLoop, this);
    
    // Start communication thread
    commThread_ = std::thread(&MultiSensorSystem::CommunicationLoop, this);
    
    std::cout << "Multi-Sensor System started" << std::endl;
    
    // Send initial status message
    uartDriver_->Printf("Multi-Sensor System Online\n");
    uartDriver_->Printf("Sampling Rate: %d Hz\n", 1000 / SAMPLE_RATE_MS);
    uartDriver_->Printf("I2C: Temperature Sensor @ 0x%02X\n", TEMP_SENSOR_ADDR);
    uartDriver_->Printf("SPI: Accelerometer\n");
    uartDriver_->Printf("UART: Host Communication @ %d baud\n", 
                       uartDriver_->GetConfig().baud_rate);
}

void MultiSensorSystem::Shutdown() {
    std::cout << "Shutting down Multi-Sensor System..." << std::endl;
    
    running_ = false;
    
    // Wait for threads to complete
    if (dataThread_.joinable()) {
        dataThread_.join();
    }
    if (commThread_.joinable()) {
        commThread_.join();
    }
    
    // Send shutdown message
    if (uartDriver_) {
        uartDriver_->Printf("Multi-Sensor System Offline\n");
    }
    
    std::cout << "Multi-Sensor System shut down complete" << std::endl;
}

bool MultiSensorSystem::ReadTemperatureSensor() {
    // Read temperature from TMP102 sensor via I2C
    uint8_t tempData[2];
    
    HfI2cErr result = i2cBus_->ReadRegister(TEMP_SENSOR_ADDR, 0x00, tempData, 2);
    
    if (result == HfI2cErr::I2C_SUCCESS) {
        // Convert TMP102 data to temperature
        int16_t rawTemp = (tempData[0] << 4) | (tempData[1] >> 4);
        if (rawTemp > 0x7FF) {
            rawTemp |= 0xF000;  // Sign extend
        }
        
        float temperature = rawTemp * 0.0625f;  // TMP102 resolution
        temperature_ = temperature;
        
        return true;
    }
    else if (result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND) {
        // Sensor not connected - use simulated data for demo
        static float simTemp = 25.0f;
        simTemp += (rand() % 100 - 50) * 0.01f;  // ±0.5°C variation
        temperature_ = simTemp;
        return true;
    }
    
    return false;
}

bool MultiSensorSystem::ConfigureAccelerometer() {
    // Configure accelerometer via SPI
    uint8_t configCmd[] = {
        0x20,  // CTRL_REG1 address
        0x27   // 50Hz, XYZ enabled
    };
    
    HfSpiErr result = spiBus_->WriteOnly(configCmd, sizeof(configCmd));
    
    if (result == HfSpiErr::SPI_SUCCESS) {
        std::cout << "Accelerometer configured successfully" << std::endl;
        return true;
    }
    else if (result == HfSpiErr::SPI_ERR_DEVICE_NOT_RESPONDING) {
        std::cout << "Accelerometer not connected - using simulated data" << std::endl;
        return true;  // Continue with simulated data
    }
    
    std::cerr << "Failed to configure accelerometer: " 
              << static_cast<int>(result) << std::endl;
    return false;
}

bool MultiSensorSystem::SendDataToHost() {
    // Send sensor data to host via UART in JSON format
    float temp = temperature_.load();
    int16_t accel_x = accelerometer_[0].load();
    int16_t accel_y = accelerometer_[1].load();
    int16_t accel_z = accelerometer_[2].load();
    
    HfUartErr result = uartDriver_->Printf(
        "{\"temp\":%.2f,\"accel\":[%d,%d,%d],\"ts\":%lld}\n",
        temp, accel_x, accel_y, accel_z,
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count()
    );
    
    return (result == HfUartErr::UART_SUCCESS);
}

void MultiSensorSystem::DataAcquisitionLoop() {
    std::cout << "Data acquisition thread started" << std::endl;
    
    while (running_) {
        auto startTime = std::chrono::steady_clock::now();
        
        // Read temperature sensor (I2C)
        if (!ReadTemperatureSensor()) {
            std::cerr << "Failed to read temperature sensor" << std::endl;
        }
        
        // Read accelerometer data (SPI)
        uint8_t accelCmd = 0x80 | ACCEL_REGISTER;  // Read command with auto-increment
        uint8_t accelData[6];
        
        HfSpiErr spiResult = spiBus_->WriteRead(&accelCmd, 1, accelData, 6);
        
        if (spiResult == HfSpiErr::SPI_SUCCESS) {
            // Convert to 16-bit signed values
            accelerometer_[0] = (accelData[1] << 8) | accelData[0];  // X
            accelerometer_[1] = (accelData[3] << 8) | accelData[2];  // Y
            accelerometer_[2] = (accelData[5] << 8) | accelData[4];  // Z
        }
        else if (spiResult == HfSpiErr::SPI_ERR_DEVICE_NOT_RESPONDING) {
            // Generate simulated accelerometer data
            static int16_t simAccel[3] = {100, -200, 1000};  // Gravity + noise
            for (int i = 0; i < 3; ++i) {
                simAccel[i] += (rand() % 200 - 100);  // Add noise
                accelerometer_[i] = simAccel[i];
            }
        }
        
        // Maintain sampling rate
        auto endTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime);
        
        if (elapsed.count() < SAMPLE_RATE_MS) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(SAMPLE_RATE_MS - elapsed.count()));
        }
    }
    
    std::cout << "Data acquisition thread stopped" << std::endl;
}

void MultiSensorSystem::CommunicationLoop() {
    std::cout << "Communication thread started" << std::endl;
    
    while (running_) {
        // Send data to host
        if (!SendDataToHost()) {
            std::cerr << "Failed to send data to host" << std::endl;
        }
        
        // Check for incoming commands
        uint8_t rxBuffer[64];
        size_t bytesRead = 0;
        
        HfUartErr result = uartDriver_->Read(rxBuffer, sizeof(rxBuffer) - 1, 
                                           bytesRead, 10);  // 10ms timeout
        
        if (result == HfUartErr::UART_SUCCESS && bytesRead > 0) {
            rxBuffer[bytesRead] = '\0';  // Null terminate
            
            // Simple command processing
            std::string command(reinterpret_cast<char*>(rxBuffer));
            
            if (command.find("status") != std::string::npos) {
                uartDriver_->Printf("System Status: Running\n");
                uartDriver_->Printf("Temperature: %.2f°C\n", temperature_.load());
                uartDriver_->Printf("Accelerometer: [%d, %d, %d]\n",
                                  accelerometer_[0].load(),
                                  accelerometer_[1].load(),
                                  accelerometer_[2].load());
            }
            else if (command.find("stop") != std::string::npos) {
                uartDriver_->Printf("Stopping system...\n");
                running_ = false;
            }
        }
        
        // Communication loop runs at 10Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Communication thread stopped" << std::endl;
}

//=============================================================================
// Main Application
//=============================================================================

int main() {
    std::cout << "=== HardFOC Multi-Sensor System Example ===" << std::endl;
    std::cout << "Demonstrating I2C, SPI, and UART coordination" << std::endl;
    std::cout << std::endl;
    
    MultiSensorSystem system;
    
    // Initialize the system
    if (!system.Initialize()) {
        std::cerr << "Failed to initialize system" << std::endl;
        return 1;
    }
    
    // Run the system
    system.Run();
    
    // Run for 30 seconds
    std::cout << "System running for 30 seconds..." << std::endl;
    std::cout << "Send 'status' via UART for system status" << std::endl;
    std::cout << "Send 'stop' via UART to stop early" << std::endl;
    
    std::this_thread::sleep_for(std::chrono::seconds(30));
    
    // Shutdown
    system.Shutdown();
    
    std::cout << "Example completed successfully" << std::endl;
    return 0;
}
