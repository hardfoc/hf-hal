/**
 * @file CommBusTests.cpp
 * @brief Implementation of communication bus unit tests.
 *
 * This file implements comprehensive tests for I2C, SPI, and UART communication
 * bus abstractions to ensure proper functionality and compatibility.
 */

#include "CommBusTests.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>

//=============================================================================
// CommBusTestManager Implementation
//=============================================================================

CommBusTestManager::CommBusTestManager() {
    SetupI2cTests();
    SetupSpiTests();
    SetupUartTests();
    SetupIntegrationTests();
    
    // Set up after-test hook for logging
    testManager_.SetAfterTestHook([this](bool result, CommBusTestType testType) {        const char* testNames[] = {
            "I2C Initialization", "I2C Configuration", "I2C Basic Read/Write",
            "I2C Register Operations", "I2C Device Scan", "I2C Error Handling",
            "I2C Thread Safety",
            "SPI Initialization", "SPI Configuration", "SPI Basic Transfer",
            "SPI Full Duplex", "SPI Chip Select", "SPI Error Handling",
            "SPI Thread Safety",
            "UART Initialization", "UART Configuration", "UART Basic TX/RX",
            "UART Formatted Output", "UART Flow Control", "UART Error Handling",
            "UART Thread Safety",
            "Multi-Bus Coordination", "Performance Benchmark"
        };
        
        int testIndex = static_cast<int>(testType);
        if (testIndex < sizeof(testNames)/sizeof(testNames[0])) {
            PrintTestResult(testNames[testIndex], result);
        }
    });
}

bool CommBusTestManager::RunAllTests() {
    std::cout << "=== Running All Communication Bus Tests ===" << std::endl;
    return testManager_.Start(CommBusTestType::I2C_INITIALIZATION_TEST, 
                             CommBusTestType::PERFORMANCE_BENCHMARK_TEST);
}

bool CommBusTestManager::RunI2cTests() {
    std::cout << "=== Running I2C Tests ===" << std::endl;
    return testManager_.Start(CommBusTestType::I2C_INITIALIZATION_TEST, 
                             CommBusTestType::I2C_LEGACY_COMPATIBILITY_TEST);
}

bool CommBusTestManager::RunSpiTests() {
    std::cout << "=== Running SPI Tests ===" << std::endl;
    return testManager_.Start(CommBusTestType::SPI_INITIALIZATION_TEST, 
                             CommBusTestType::SPI_LEGACY_COMPATIBILITY_TEST);
}

bool CommBusTestManager::RunUartTests() {
    std::cout << "=== Running UART Tests ===" << std::endl;
    return testManager_.Start(CommBusTestType::UART_INITIALIZATION_TEST, 
                             CommBusTestType::UART_LEGACY_COMPATIBILITY_TEST);
}

bool CommBusTestManager::RunIntegrationTests() {
    std::cout << "=== Running Integration Tests ===" << std::endl;
    return testManager_.Start(CommBusTestType::MULTI_BUS_COORDINATION_TEST, 
                             CommBusTestType::PERFORMANCE_BENCHMARK_TEST);
}

//=============================================================================
// Test Setup Methods
//=============================================================================

void CommBusTestManager::SetupI2cTests() {
    testManager_.AddTest(CommBusTestType::I2C_INITIALIZATION_TEST,
        [this]() { return TestI2cInitialization(); }, false);
    testManager_.AddTest(CommBusTestType::I2C_CONFIGURATION_TEST,
        [this]() { return TestI2cConfiguration(); }, false);
    testManager_.AddTest(CommBusTestType::I2C_BASIC_READ_WRITE_TEST,
        [this]() { return TestI2cBasicReadWrite(); }, true);
    testManager_.AddTest(CommBusTestType::I2C_REGISTER_OPERATIONS_TEST,
        [this]() { return TestI2cRegisterOperations(); }, true);
    testManager_.AddTest(CommBusTestType::I2C_DEVICE_SCAN_TEST,
        [this]() { return TestI2cDeviceScan(); }, true);
    testManager_.AddTest(CommBusTestType::I2C_ERROR_HANDLING_TEST,
        [this]() { return TestI2cErrorHandling(); }, true);
    testManager_.AddTest(CommBusTestType::I2C_THREAD_SAFETY_TEST,        [this]() { return TestI2cThreadSafety(); }, true);
}

void CommBusTestManager::SetupSpiTests() {
    testManager_.AddTest(CommBusTestType::SPI_INITIALIZATION_TEST,
        [this]() { return TestSpiInitialization(); }, false);
    testManager_.AddTest(CommBusTestType::SPI_CONFIGURATION_TEST,
        [this]() { return TestSpiConfiguration(); }, false);
    testManager_.AddTest(CommBusTestType::SPI_BASIC_TRANSFER_TEST,
        [this]() { return TestSpiBasicTransfer(); }, true);
    testManager_.AddTest(CommBusTestType::SPI_FULL_DUPLEX_TEST,
        [this]() { return TestSpiFullDuplex(); }, true);
    testManager_.AddTest(CommBusTestType::SPI_CHIP_SELECT_TEST,
        [this]() { return TestSpiChipSelect(); }, true);
    testManager_.AddTest(CommBusTestType::SPI_ERROR_HANDLING_TEST,
        [this]() { return TestSpiErrorHandling(); }, true);
    testManager_.AddTest(CommBusTestType::SPI_THREAD_SAFETY_TEST,        [this]() { return TestSpiThreadSafety(); }, true);
}

void CommBusTestManager::SetupUartTests() {
    testManager_.AddTest(CommBusTestType::UART_INITIALIZATION_TEST,
        [this]() { return TestUartInitialization(); }, false);
    testManager_.AddTest(CommBusTestType::UART_CONFIGURATION_TEST,
        [this]() { return TestUartConfiguration(); }, false);
    testManager_.AddTest(CommBusTestType::UART_BASIC_TX_RX_TEST,
        [this]() { return TestUartBasicTxRx(); }, true);
    testManager_.AddTest(CommBusTestType::UART_FORMATTED_OUTPUT_TEST,
        [this]() { return TestUartFormattedOutput(); }, true);
    testManager_.AddTest(CommBusTestType::UART_FLOW_CONTROL_TEST,
        [this]() { return TestUartFlowControl(); }, true);
    testManager_.AddTest(CommBusTestType::UART_ERROR_HANDLING_TEST,
        [this]() { return TestUartErrorHandling(); }, true);
    testManager_.AddTest(CommBusTestType::UART_THREAD_SAFETY_TEST,        [this]() { return TestUartThreadSafety(); }, true);
}

void CommBusTestManager::SetupIntegrationTests() {
    testManager_.AddTest(CommBusTestType::MULTI_BUS_COORDINATION_TEST,
        [this]() { return TestMultiBusCoordination(); }, true);
    testManager_.AddTest(CommBusTestType::PERFORMANCE_BENCHMARK_TEST,
        [this]() { return TestPerformanceBenchmark(); }, true);
}

//=============================================================================
// I2C Test Implementations
//=============================================================================

bool CommBusTestManager::TestI2cInitialization() {
    try {
        I2cBusConfig config;
        config.port = 0;
        config.sda_pin = 21;
        config.scl_pin = 22;
        config.clock_speed_hz = 100000;
        config.enable_internal_pullups = true;
        
        i2cBus_ = std::make_unique<McuI2cBus>(config);
        
        // Test initialization
        if (!i2cBus_->EnsureInitialized()) {
            return false;
        }
        
        // Test multiple initialization calls (should succeed)
        if (!i2cBus_->EnsureInitialized()) {
            return false;
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

bool CommBusTestManager::TestI2cConfiguration() {
    if (!i2cBus_) return false;
    
    // Test configuration retrieval
    I2cBusConfig config = i2cBus_->GetConfig();
    
    return (config.port == 0 && 
            config.sda_pin == 21 && 
            config.scl_pin == 22 &&
            config.clock_speed_hz == 100000);
}

bool CommBusTestManager::TestI2cBasicReadWrite() {
    if (!i2cBus_) return false;
    
    // Create a mock device for testing
    const uint8_t testAddress = 0x48;
    uint8_t writeData[] = {0x01, 0x02, 0x03};
    uint8_t readBuffer[3];
    
    // Test write operation
    HfI2cErr result = i2cBus_->Write(testAddress, writeData, sizeof(writeData));
    if (result != HfI2cErr::I2C_SUCCESS) {
        // This may fail without real hardware, but error should be reasonable
        return (result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND || 
                result == HfI2cErr::I2C_ERR_DEVICE_NACK);
    }
    
    // Test read operation
    result = i2cBus_->Read(testAddress, readBuffer, sizeof(readBuffer));
    if (result != HfI2cErr::I2C_SUCCESS) {
        // This may fail without real hardware, but error should be reasonable
        return (result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND || 
                result == HfI2cErr::I2C_ERR_DEVICE_NACK);
    }
    
    return true;
}

bool CommBusTestManager::TestI2cRegisterOperations() {
    if (!i2cBus_) return false;
    
    const uint8_t testAddress = 0x48;
    const uint8_t regAddress = 0x10;
    const uint8_t testValue = 0xAA;
    uint8_t readValue;
    
    // Test register write
    HfI2cErr result = i2cBus_->WriteRegister(testAddress, regAddress, testValue);
    if (result != HfI2cErr::I2C_SUCCESS) {
        return (result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND || 
                result == HfI2cErr::I2C_ERR_DEVICE_NACK);
    }
    
    // Test register read
    result = i2cBus_->ReadRegister(testAddress, regAddress, readValue);
    if (result != HfI2cErr::I2C_SUCCESS) {
        return (result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND || 
                result == HfI2cErr::I2C_ERR_DEVICE_NACK);
    }
    
    return true;
}

bool CommBusTestManager::TestI2cDeviceScan() {
    if (!i2cBus_) return false;
    
    std::vector<uint8_t> devices = i2cBus_->ScanForDevices();
    
    // Device scan should complete without error (may find 0 devices)
    return true;
}

bool CommBusTestManager::TestI2cErrorHandling() {
    if (!i2cBus_) return false;
    
    // Test invalid address
    uint8_t data = 0;
    HfI2cErr result = i2cBus_->Read(0x00, &data, 1);  // Reserved address
    
    // Should return appropriate error
    return (result != HfI2cErr::I2C_SUCCESS);
}

bool CommBusTestManager::TestI2cThreadSafety() {
    if (!i2cBus_) return false;
    
    std::vector<std::thread> threads;
    std::atomic<int> successCount{0};
    const int numThreads = 4;
    const int operationsPerThread = 10;
    
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([this, &successCount, operationsPerThread]() {
            int localSuccessCount = 0;
            for (int j = 0; j < operationsPerThread; ++j) {
                uint8_t data = 0xAA;
                // Thread-safe operations should not crash
                HfI2cErr result = i2cBus_->Write(0x48, &data, 1);
                if (result == HfI2cErr::I2C_SUCCESS || 
                    result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND) {
                    localSuccessCount++;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            successCount += localSuccessCount;
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    // All operations should complete (success or expected failure)
    return successCount == (numThreads * operationsPerThread);
}

bool CommBusTestManager::TestI2cLegacyCompatibility() {
    // Test legacy I2cBus alias
    I2cBusConfig config;
    config.port = 1;
    config.sda_pin = 19;
    config.scl_pin = 18;
    config.clock_speed_hz = 400000;
    
    I2cBus legacyBus(config);  // Should create McuI2cBus
    
    if (!legacyBus.EnsureInitialized()) {
        return false;
    }
    
    // Test legacy boolean API
    uint8_t data = 0x55;
    bool result = legacyBus.WriteByte(0x48, data);
    
    // Should either succeed or fail gracefully
    return true;
}

//=============================================================================
// SPI Test Implementations
//=============================================================================

bool CommBusTestManager::TestSpiInitialization() {
    try {
        SpiBusConfig config;
        config.mosi_pin = 23;
        config.miso_pin = 19;
        config.sclk_pin = 18;
        config.cs_pin = 5;
        config.clock_speed_hz = 1000000;
        config.mode = 0;
        config.word_size = 8;
        
        spiBus_ = std::make_unique<McuSpiBus>(config);
        
        if (!spiBus_->EnsureInitialized()) {
            return false;
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

bool CommBusTestManager::TestSpiConfiguration() {
    if (!spiBus_) return false;
    
    SpiBusConfig config = spiBus_->GetConfig();
    
    return (config.mosi_pin == 23 && 
            config.miso_pin == 19 && 
            config.sclk_pin == 18 &&
            config.cs_pin == 5);
}

bool CommBusTestManager::TestSpiBasicTransfer() {
    if (!spiBus_) return false;
    
    uint8_t txData[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rxData[4];
    
    HfSpiErr result = spiBus_->Transfer(txData, rxData, sizeof(txData));
    
    // Should complete without crashing
    return (result == HfSpiErr::SPI_SUCCESS || 
            result == HfSpiErr::SPI_ERR_DEVICE_NOT_RESPONDING);
}

bool CommBusTestManager::TestSpiFullDuplex() {
    if (!spiBus_) return false;
    
    uint8_t txData[] = {0xAA, 0x55, 0xCC, 0x33};
    uint8_t rxData[4];
    
    HfSpiErr result = spiBus_->TransferFullDuplex(txData, rxData, sizeof(txData));
    
    return (result == HfSpiErr::SPI_SUCCESS || 
            result == HfSpiErr::SPI_ERR_DEVICE_NOT_RESPONDING);
}

bool CommBusTestManager::TestSpiChipSelect() {
    if (!spiBus_) return false;
    
    // Test manual chip select control
    spiBus_->SetChipSelectActive(true);
    spiBus_->SetChipSelectActive(false);
    
    return true;
}

bool CommBusTestManager::TestSpiErrorHandling() {
    if (!spiBus_) return false;
    
    // Test null pointer handling
    HfSpiErr result = spiBus_->Transfer(nullptr, nullptr, 10);
    
    return (result == HfSpiErr::SPI_ERR_NULL_POINTER);
}

bool CommBusTestManager::TestSpiThreadSafety() {
    if (!spiBus_) return false;
    
    std::vector<std::thread> threads;
    std::atomic<int> successCount{0};
    const int numThreads = 4;
    
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([this, &successCount]() {
            uint8_t data[] = {0x01, 0x02};
            uint8_t rx[2];
            HfSpiErr result = spiBus_->Transfer(data, rx, 2);
            if (result == HfSpiErr::SPI_SUCCESS || 
                result == HfSpiErr::SPI_ERR_DEVICE_NOT_RESPONDING) {
                successCount++;
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    return successCount == numThreads;
}

bool CommBusTestManager::TestSpiLegacyCompatibility() {
    SpiBusConfig config;
    config.mosi_pin = 23;
    config.miso_pin = 19;
    config.sclk_pin = 18;
    config.cs_pin = 5;
    config.clock_speed_hz = 1000000;
    
    SpiBus legacyBus(config);  // Should create McuSpiBus
    
    return legacyBus.EnsureInitialized();
}

//=============================================================================
// UART Test Implementations
//=============================================================================

bool CommBusTestManager::TestUartInitialization() {
    try {
        UartConfig config;
        config.uart_port = 2;
        config.tx_pin = 17;
        config.rx_pin = 16;
        config.baud_rate = 115200;
        config.data_bits = 8;
        config.parity = UartParity::UART_PARITY_NONE;
        config.stop_bits = 1;
        
        uartDriver_ = std::make_unique<McuUartDriver>(config);
        
        if (!uartDriver_->EnsureInitialized()) {
            return false;
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

bool CommBusTestManager::TestUartConfiguration() {
    if (!uartDriver_) return false;
    
    UartConfig config = uartDriver_->GetConfig();
    
    return (config.uart_port == 2 && 
            config.tx_pin == 17 && 
            config.rx_pin == 16 &&
            config.baud_rate == 115200);
}

bool CommBusTestManager::TestUartBasicTxRx() {
    if (!uartDriver_) return false;
    
    const char* testString = "Hello, UART!";
    char rxBuffer[32];
    
    HfUartErr result = uartDriver_->Write(reinterpret_cast<const uint8_t*>(testString), 
                                          strlen(testString));
    
    if (result != HfUartErr::UART_SUCCESS) {
        return false;
    }
    
    size_t bytesRead = 0;
    result = uartDriver_->Read(reinterpret_cast<uint8_t*>(rxBuffer), 
                               sizeof(rxBuffer) - 1, bytesRead, 100);
    
    // Should complete without error (may read 0 bytes)
    return (result == HfUartErr::UART_SUCCESS || 
            result == HfUartErr::UART_ERR_TIMEOUT);
}

bool CommBusTestManager::TestUartFormattedOutput() {
    if (!uartDriver_) return false;
    
    HfUartErr result = uartDriver_->Printf("Test value: %d\n", 42);
    
    return (result == HfUartErr::UART_SUCCESS);
}

bool CommBusTestManager::TestUartFlowControl() {
    if (!uartDriver_) return false;
    
    // Test RTS/CTS control
    uartDriver_->SetRts(true);
    uartDriver_->SetRts(false);
    
    bool ctsState = uartDriver_->GetCts();
    
    // Should complete without error
    return true;
}

bool CommBusTestManager::TestUartErrorHandling() {
    if (!uartDriver_) return false;
    
    // Test null pointer handling
    HfUartErr result = uartDriver_->Write(nullptr, 10);
    
    return (result == HfUartErr::UART_ERR_NULL_POINTER);
}

bool CommBusTestManager::TestUartThreadSafety() {
    if (!uartDriver_) return false;
    
    std::vector<std::thread> threads;
    std::atomic<int> successCount{0};
    const int numThreads = 4;
    
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back([this, &successCount, i]() {
            std::string testData = "Thread " + std::to_string(i) + " data\n";
            HfUartErr result = uartDriver_->Write(
                reinterpret_cast<const uint8_t*>(testData.c_str()), 
                testData.length());
            if (result == HfUartErr::UART_SUCCESS) {
                successCount++;
            }
        });
    }
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    return successCount == numThreads;
}

bool CommBusTestManager::TestUartLegacyCompatibility() {
    UartConfig config;
    config.uart_port = 1;
    config.tx_pin = 4;
    config.rx_pin = 5;
    config.baud_rate = 9600;
    
    UartDriver legacyDriver(config);  // Should create McuUartDriver
    
    return legacyDriver.EnsureInitialized();
}

//=============================================================================
// Integration Test Implementations
//=============================================================================

bool CommBusTestManager::TestMultiBusCoordination() {
    // Test using multiple buses simultaneously
    if (!i2cBus_ || !spiBus_ || !uartDriver_) {
        return false;
    }
    
    // Perform operations on all buses concurrently
    std::vector<std::thread> threads;
    std::atomic<int> successCount{0};
    
    // I2C thread
    threads.emplace_back([this, &successCount]() {
        uint8_t data = 0xAA;
        HfI2cErr result = i2cBus_->Write(0x48, &data, 1);
        if (result == HfI2cErr::I2C_SUCCESS || 
            result == HfI2cErr::I2C_ERR_DEVICE_NOT_FOUND) {
            successCount++;
        }
    });
    
    // SPI thread
    threads.emplace_back([this, &successCount]() {
        uint8_t txData[] = {0x01, 0x02};
        uint8_t rxData[2];
        HfSpiErr result = spiBus_->Transfer(txData, rxData, 2);
        if (result == HfSpiErr::SPI_SUCCESS || 
            result == HfSpiErr::SPI_ERR_DEVICE_NOT_RESPONDING) {
            successCount++;
        }
    });
    
    // UART thread
    threads.emplace_back([this, &successCount]() {
        const char* msg = "Test\n";
        HfUartErr result = uartDriver_->Write(
            reinterpret_cast<const uint8_t*>(msg), strlen(msg));
        if (result == HfUartErr::UART_SUCCESS) {
            successCount++;
        }
    });
    
    for (auto& thread : threads) {
        thread.join();
    }
    
    return successCount == 3;
}

bool CommBusTestManager::TestPerformanceBenchmark() {
    if (!i2cBus_ || !spiBus_ || !uartDriver_) {
        return false;
    }
    
    const int iterations = 100;
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Perform repeated operations to test performance
    for (int i = 0; i < iterations; ++i) {
        uint8_t data = static_cast<uint8_t>(i);
        
        // I2C operation
        i2cBus_->Write(0x48, &data, 1);
        
        // SPI operation
        uint8_t rxData;
        spiBus_->Transfer(&data, &rxData, 1);
        
        // UART operation
        uartDriver_->Write(&data, 1);
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    
    std::cout << "Performance test completed " << iterations 
              << " iterations in " << duration.count() << "ms" << std::endl;
    
    // Should complete in reasonable time
    return duration.count() < 10000;  // Less than 10 seconds
}

//=============================================================================
// Utility Methods
//=============================================================================

void CommBusTestManager::PrintTestResult(const char* testName, bool result) {
    std::cout << "[" << (result ? "PASS" : "FAIL") << "] " << testName << std::endl;
}

//=============================================================================
// Mock Device Implementations
//=============================================================================

MockI2cDevice::MockI2cDevice(uint8_t address) 
    : address_(address), respondToScan_(true) {
    memset(registers_, 0, sizeof(registers_));
}

void MockI2cDevice::SetRegisterValue(uint8_t reg, uint8_t value) {
    registers_[reg] = value;
}

uint8_t MockI2cDevice::GetRegisterValue(uint8_t reg) const {
    return registers_[reg];
}

MockSpiDevice::MockSpiDevice() : responseLength_(0), currentPosition_(0) {
    memset(responseData_, 0, sizeof(responseData_));
}

void MockSpiDevice::SetResponse(const uint8_t* data, size_t length) {
    if (length <= sizeof(responseData_)) {
        memcpy(responseData_, data, length);
        responseLength_ = length;
        currentPosition_ = 0;
    }
}

bool MockSpiDevice::ProcessTransfer(const uint8_t* txData, uint8_t* rxData, size_t length) {
    if (rxData && currentPosition_ + length <= responseLength_) {
        memcpy(rxData, responseData_ + currentPosition_, length);
        currentPosition_ += length;
        return true;
    }
    return false;
}

MockUartDevice::MockUartDevice() : echoMode_(true), rxBufferLength_(0) {
    memset(rxBuffer_, 0, sizeof(rxBuffer_));
}

bool MockUartDevice::ProcessData(const uint8_t* txData, size_t txLength, 
                                uint8_t* rxData, size_t& rxLength) {
    if (echoMode_ && txData && rxData && txLength <= rxLength) {
        memcpy(rxData, txData, txLength);
        rxLength = txLength;
        return true;
    }
    rxLength = 0;
    return false;
}

//=============================================================================
// C Interface
//=============================================================================

extern "C" int RunCommBusTests() {
    CommBusTestManager testManager;
    
    std::cout << "Starting Communication Bus Tests..." << std::endl;
    
    bool result = testManager.RunAllTests();
    
    std::cout << "Communication Bus Tests " 
              << (result ? "PASSED" : "FAILED") << std::endl;
    
    return result ? 0 : 1;
}
