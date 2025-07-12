/**
 * @file CommBusTests.h
 * @brief Unit tests for communication bus implementations (I2C, SPI, UART).
 *
 * This file provides comprehensive unit tests for the new layered communication
 * bus architecture including BaseI2cBus, BaseSpiBus, BaseUartDriver and their
 * MCU implementations.
 */

#ifndef COMM_BUS_TESTS_H
#define COMM_BUS_TESTS_H

#include "TestManager.h"
#include "base/BaseI2cBus.h"
#include "base/BaseSpiBus.h"
#include "base/BaseUartDriver.h"
#include "mcu/McuI2cBus.h"
#include "mcu/McuSpiBus.h"
#include "mcu/McuUartDriver.h"
#include "mcu/McuI2cBus.h"
#include "mcu/McuSpiBus.h"
#include "mcu/McuUartDriver.h"
#include <memory>

/**
 * @brief Enumeration of all communication bus tests
 */
enum class CommBusTestType {
    // I2C Tests
    I2C_INITIALIZATION_TEST,
    I2C_CONFIGURATION_TEST,
    I2C_BASIC_READ_WRITE_TEST,
    I2C_REGISTER_OPERATIONS_TEST,    I2C_DEVICE_SCAN_TEST,
    I2C_ERROR_HANDLING_TEST,
    I2C_THREAD_SAFETY_TEST,
    
    // SPI Tests
    SPI_INITIALIZATION_TEST,
    SPI_CONFIGURATION_TEST,
    SPI_BASIC_TRANSFER_TEST,
    SPI_FULL_DUPLEX_TEST,
    SPI_CHIP_SELECT_TEST,    SPI_ERROR_HANDLING_TEST,
    SPI_THREAD_SAFETY_TEST,
    
    // UART Tests
    UART_INITIALIZATION_TEST,
    UART_CONFIGURATION_TEST,
    UART_BASIC_TX_RX_TEST,
    UART_FORMATTED_OUTPUT_TEST,
    UART_FLOW_CONTROL_TEST,    UART_ERROR_HANDLING_TEST,
    UART_THREAD_SAFETY_TEST,
    
    // Integration Tests
    MULTI_BUS_COORDINATION_TEST,
    PERFORMANCE_BENCHMARK_TEST,
    
    MAX_TEST_TYPE
};

/**
 * @brief Communication bus test manager class
 * 
 * This class orchestrates all communication bus tests using the TestManager
 * framework to ensure comprehensive testing of the new architecture.
 */
class CommBusTestManager {
public:
    /**
     * @brief Constructor - sets up all test cases
     */
    CommBusTestManager();
    
    /**
     * @brief Run all communication bus tests
     * @return true if all tests pass, false otherwise
     */
    bool RunAllTests();
    
    /**
     * @brief Run only I2C tests
     * @return true if all I2C tests pass, false otherwise
     */
    bool RunI2cTests();
    
    /**
     * @brief Run only SPI tests
     * @return true if all SPI tests pass, false otherwise
     */
    bool RunSpiTests();
    
    /**
     * @brief Run only UART tests
     * @return true if all UART tests pass, false otherwise
     */
    bool RunUartTests();
    
    /**
     * @brief Run integration tests
     * @return true if all integration tests pass, false otherwise
     */
    bool RunIntegrationTests();

private:
    TestManager<CommBusTestType, CommBusTestType::MAX_TEST_TYPE> testManager_;
    
    // Test setup helpers
    void SetupI2cTests();
    void SetupSpiTests();
    void SetupUartTests();
    void SetupIntegrationTests();
    
    // I2C Test Methods
    bool TestI2cInitialization();
    bool TestI2cConfiguration();
    bool TestI2cBasicReadWrite();
    bool TestI2cRegisterOperations();
    bool TestI2cDeviceScan();    bool TestI2cErrorHandling();
    bool TestI2cThreadSafety();
    
    // SPI Test Methods
    bool TestSpiInitialization();
    bool TestSpiConfiguration();
    bool TestSpiBasicTransfer();
    bool TestSpiFullDuplex();
    bool TestSpiChipSelect();    bool TestSpiErrorHandling();
    bool TestSpiThreadSafety();
    
    // UART Test Methods
    bool TestUartInitialization();
    bool TestUartConfiguration();
    bool TestUartBasicTxRx();
    bool TestUartFormattedOutput();
    bool TestUartFlowControl();    bool TestUartErrorHandling();
    bool TestUartThreadSafety();
    
    // Integration Test Methods
    bool TestMultiBusCoordination();
    bool TestPerformanceBenchmark();
    
    // Test utilities
    void PrintTestResult(const char* testName, bool result);
    bool CreateMockI2cDevice(uint8_t address);
    bool CreateMockSpiDevice();
    bool CreateMockUartDevice();
    
    // Test instances
    std::unique_ptr<McuI2cBus> i2cBus_;
    std::unique_ptr<McuSpiBus> spiBus_;
    std::unique_ptr<McuUartDriver> uartDriver_;
};

/**
 * @brief Mock I2C device for testing
 * 
 * This class simulates an I2C device for testing purposes,
 * allowing verification of I2C communication without real hardware.
 */
class MockI2cDevice {
public:
    MockI2cDevice(uint8_t address);
    
    bool RespondToScan() const { return respondToScan_; }
    void SetRespondToScan(bool respond) { respondToScan_ = respond; }
    
    uint8_t GetAddress() const { return address_; }
    
    // Mock register read/write
    void SetRegisterValue(uint8_t reg, uint8_t value);
    uint8_t GetRegisterValue(uint8_t reg) const;
    
private:
    uint8_t address_;
    bool respondToScan_;
    uint8_t registers_[256];  // Mock register space
};

/**
 * @brief Mock SPI device for testing
 */
class MockSpiDevice {
public:
    MockSpiDevice();
    
    // Mock SPI responses
    void SetResponse(const uint8_t* data, size_t length);
    bool ProcessTransfer(const uint8_t* txData, uint8_t* rxData, size_t length);
    
private:
    uint8_t responseData_[1024];
    size_t responseLength_;
    size_t currentPosition_;
};

/**
 * @brief Mock UART device for testing
 */
class MockUartDevice {
public:
    MockUartDevice();
    
    // Mock UART echo functionality
    bool ProcessData(const uint8_t* txData, size_t txLength, uint8_t* rxData, size_t& rxLength);
    void SetEchoMode(bool enabled) { echoMode_ = enabled; }
    
private:
    bool echoMode_;
    uint8_t rxBuffer_[1024];
    size_t rxBufferLength_;
};

// Global test function declarations
extern "C" {
    /**
     * @brief Entry point for communication bus tests
     * @return 0 if all tests pass, non-zero otherwise
     */
    int RunCommBusTests();
}

#endif // COMM_BUS_TESTS_H
