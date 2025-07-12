/**
 * @file test_runner.cpp
 * @brief Simple test runner for communication bus tests.
 *
 * This file provides a standalone test runner that can be compiled and
 * executed to verify the communication bus implementations.
 */

#include "CommBusTests.h"
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << "=== HardFOC Communication Bus Test Suite ===" << std::endl;
    std::cout << "Testing I2C, SPI, and UART abstraction layers..." << std::endl;
    std::cout << std::endl;
    
    CommBusTestManager testManager;
    bool allTestsPassed = true;
    
    // Parse command line arguments for selective testing
    if (argc > 1) {
        std::string testType = argv[1];
        
        if (testType == "i2c") {
            std::cout << "Running I2C tests only..." << std::endl;
            allTestsPassed = testManager.RunI2cTests();
        }
        else if (testType == "spi") {
            std::cout << "Running SPI tests only..." << std::endl;
            allTestsPassed = testManager.RunSpiTests();
        }
        else if (testType == "uart") {
            std::cout << "Running UART tests only..." << std::endl;
            allTestsPassed = testManager.RunUartTests();
        }
        else if (testType == "integration") {
            std::cout << "Running integration tests only..." << std::endl;
            allTestsPassed = testManager.RunIntegrationTests();
        }
        else {
            std::cout << "Unknown test type: " << testType << std::endl;
            std::cout << "Available options: i2c, spi, uart, integration" << std::endl;
            return 1;
        }
    }
    else {
        // Run all tests
        allTestsPassed = testManager.RunAllTests();
    }
    
    std::cout << std::endl;
    std::cout << "=== Test Summary ===" << std::endl;
    std::cout << "Overall Result: " << (allTestsPassed ? "PASSED" : "FAILED") << std::endl;
    
    if (allTestsPassed) {
        std::cout << "All communication bus tests passed successfully!" << std::endl;
        std::cout << "The new abstraction layer is working correctly." << std::endl;
    } else {
        std::cout << "Some tests failed. Please review the output above." << std::endl;
        std::cout << "Note: Some failures may be expected without real hardware." << std::endl;
    }
    
    return allTestsPassed ? 0 : 1;
}
