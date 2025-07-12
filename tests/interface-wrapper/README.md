# Communication Bus Test Suite

This directory contains comprehensive unit tests for the HardFOC communication bus abstraction layer, including I2C, SPI, and UART implementations.

## Overview

The test suite verifies:
- **Abstract Base Interfaces**: Proper inheritance and virtual function dispatch
- **MCU Implementations**: Platform-specific functionality (ESP32, STM32, etc.)
- **Legacy Compatibility**: Backward compatibility with existing code
- **Thread Safety**: Concurrent access protection
- **Error Handling**: Comprehensive error reporting and recovery
- **Performance**: Basic performance benchmarks

## Test Structure

### Test Categories

1. **I2C Tests** (`CommBusTestType::I2C_*`)
   - Initialization and configuration
   - Basic read/write operations
   - Register-based communications
   - Device scanning
   - Error handling and recovery
   - Thread safety verification
   - Legacy API compatibility

2. **SPI Tests** (`CommBusTestType::SPI_*`)
   - Bus initialization and setup
   - Basic data transfers
   - Full-duplex communication
   - Chip select control
   - Error handling
   - Thread safety
   - Legacy compatibility

3. **UART Tests** (`CommBusTestType::UART_*`)
   - Port initialization
   - Basic TX/RX operations
   - Formatted output (printf-style)
   - Flow control (RTS/CTS)
   - Error handling
   - Thread safety
   - Legacy compatibility

4. **Integration Tests**
   - Multi-bus coordination
   - Performance benchmarks
   - Resource sharing verification

### Mock Devices

The test suite includes mock device implementations for testing without real hardware:

- **MockI2cDevice**: Simulates I2C peripheral responses
- **MockSpiDevice**: Provides SPI transfer responses
- **MockUartDevice**: Echo-mode UART simulation

## Building and Running

### Standalone Build (Development/CI)

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build the tests
make

# Run all tests
./hf_comm_bus_tests

# Run specific test categories
./hf_comm_bus_tests i2c      # I2C tests only
./hf_comm_bus_tests spi      # SPI tests only
./hf_comm_bus_tests uart     # UART tests only
./hf_comm_bus_tests integration  # Integration tests only
```

### ESP-IDF Build

```bash
# In your ESP-IDF project directory
idf.py build

# Flash and run (tests will execute on startup)
idf.py flash monitor
```

### CMake Test Integration

```bash
# Build and run via CTest
make test

# Or run specific CMake targets
make run_comm_bus_tests
make run_i2c_tests
make run_spi_tests
make run_uart_tests
make run_integration_tests
```

## Test Results

### Expected Behavior

**Without Real Hardware:**
- Initialization tests should PASS
- Configuration tests should PASS
- Communication tests may FAIL with expected errors:
  - I2C: `I2C_ERR_DEVICE_NOT_FOUND` or `I2C_ERR_DEVICE_NACK`
  - SPI: `SPI_ERR_DEVICE_NOT_RESPONDING`
  - UART: `UART_ERR_TIMEOUT` for read operations
- Thread safety tests should PASS
- Legacy compatibility tests should PASS
- Integration tests should PASS (with expected communication failures)

**With Real Hardware:**
- All tests should PASS when appropriate devices are connected
- Communication tests will succeed with actual device responses
- Performance benchmarks will provide realistic timing data

### Interpreting Results

```
[PASS] I2C Initialization          ✓ Bus setup successful
[PASS] I2C Configuration          ✓ Config validation successful  
[FAIL] I2C Basic Read/Write       ✓ Expected without hardware
[PASS] I2C Error Handling         ✓ Proper error reporting
[PASS] I2C Thread Safety          ✓ Concurrent access protected
[PASS] I2C Legacy Compatibility   ✓ Backward compatibility maintained
```

## Adding New Tests

### 1. Add Test Enum

```cpp
enum class CommBusTestType {
    // ... existing tests ...
    YOUR_NEW_TEST,
    MAX_TEST_TYPE
};
```

### 2. Implement Test Method

```cpp
bool CommBusTestManager::TestYourNewFeature() {
    // Your test implementation
    return true;  // or false for failure
}
```

### 3. Register Test

```cpp
void CommBusTestManager::SetupYourTests() {
    testManager_.AddTest(CommBusTestType::YOUR_NEW_TEST,
        [this]() { return TestYourNewFeature(); }, 
        true);  // continueOnFail
}
```

### 4. Update Test Names Array

```cpp
const char* testNames[] = {
    // ... existing names ...
    "Your New Test",
};
```

## Test Configuration

### Hardware Configuration

For hardware-in-the-loop testing, configure pin assignments in test methods:

```cpp
// I2C Configuration
I2cBusConfig i2cConfig;
i2cConfig.port = 0;
i2cConfig.sda_pin = 21;    // Adjust for your hardware
i2cConfig.scl_pin = 22;    // Adjust for your hardware
i2cConfig.clock_speed_hz = 400000;

// SPI Configuration  
SpiBusConfig spiConfig;
spiConfig.mosi_pin = 23;   // Adjust for your hardware
spiConfig.miso_pin = 19;   // Adjust for your hardware
spiConfig.sclk_pin = 18;   // Adjust for your hardware
spiConfig.cs_pin = 5;      // Adjust for your hardware

// UART Configuration
UartConfig uartConfig;
uartConfig.uart_port = 2;
uartConfig.tx_pin = 17;    // Adjust for your hardware
uartConfig.rx_pin = 16;    // Adjust for your hardware
uartConfig.baud_rate = 115200;
```

### Mock Device Configuration

Configure mock devices for specific test scenarios:

```cpp
// Configure mock I2C device
MockI2cDevice mockDevice(0x48);
mockDevice.SetRegisterValue(0x00, 0xAA);
mockDevice.SetRespondToScan(true);

// Configure mock SPI device
MockSpiDevice mockSpi;
uint8_t response[] = {0x01, 0x02, 0x03, 0x04};
mockSpi.SetResponse(response, sizeof(response));

// Configure mock UART device
MockUartDevice mockUart;
mockUart.SetEchoMode(true);
```

## Continuous Integration

The test suite is designed for CI/CD integration:

```yaml
# Example GitHub Actions workflow
- name: Build and Test Communication Bus
  run: |
    mkdir build && cd build
    cmake ..
    make
    ./hf_comm_bus_tests
```

## Troubleshooting

### Common Issues

1. **Compilation Errors**
   - Ensure all required headers are included
   - Check CMake configuration for missing dependencies
   - Verify C++17 standard support

2. **Test Failures**
   - Review hardware connections if using real devices
   - Check pin assignments in test configuration
   - Verify MCU platform support is implemented

3. **Platform-Specific Issues**
   - ESP32: Ensure ESP-IDF is properly configured
   - STM32: Implement platform-specific MCU classes
   - Other platforms: Add platform support as needed

### Debug Output

Enable verbose test output by modifying `PrintTestResult()`:

```cpp
void CommBusTestManager::PrintTestResult(const char* testName, bool result) {
    std::cout << "[" << (result ? "PASS" : "FAIL") << "] " << testName;
    if (!result) {
        std::cout << " - Check hardware connections and configuration";
    }
    std::cout << std::endl;
}
```

## Contributing

When adding new communication bus features:

1. **Add Tests First**: Write tests for new functionality
2. **Test All Platforms**: Ensure cross-platform compatibility
3. **Update Documentation**: Keep this README and code comments current
4. **Verify Integration**: Run full test suite before committing
5. **Add Performance Tests**: Include benchmarks for new features

## Files

- `CommBusTests.h` - Test class declarations and mock devices
- `CommBusTests.cpp` - Test implementations
- `test_runner.cpp` - Standalone test runner
- `CMakeLists.txt` - Build configuration
- `README.md` - This documentation
