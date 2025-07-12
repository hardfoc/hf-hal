# HardFOC Test Suite

This directory contains the consolidated test suite for the HardFOC Hardware Abstraction Layer (HAL).

## Directory Structure

```
tests/
├── CMakeLists.txt                    # Master test build configuration
├── component-handler/                # Component handler layer tests
│   ├── GpioSystemIntegrationTest.h   # GPIO system integration tests
│   └── GpioSystemIntegrationTest.cpp
├── interface-wrapper/                # Internal interface wrapper tests
│   ├── CMakeLists.txt                # Interface wrapper test build config
│   ├── README.md                     # Interface wrapper test documentation
│   ├── CommBusTests.h                # Communication bus tests
│   ├── CommBusTests.cpp
│   └── test_runner.cpp               # Test runner implementation
└── external-drivers/                 # External driver tests
    └── (Future external driver tests will go here)
```

## Running Tests

### Build All Tests
```bash
cd hf-hal/tests
mkdir build && cd build
cmake ..
make
```

### Run Specific Test Suites

#### Component Handler Tests
```bash
cd component-handler
# Run GPIO system integration tests
./gpio_system_integration_test
```

#### Interface Wrapper Tests
```bash
cd interface-wrapper
# Run communication bus tests
./comm_bus_tests
```

## Test Categories

### 1. Component Handler Tests
- **GPIO System Integration Tests**: Comprehensive testing of the consolidated GPIO management system
- **ADC System Integration Tests**: Testing of the consolidated ADC management system
- **Manager Lifecycle Tests**: Testing initialization, configuration, and cleanup

### 2. Interface Wrapper Tests
- **Communication Bus Tests**: Testing I2C, SPI, UART, and CAN bus implementations
- **Base Class Tests**: Testing abstract base class functionality
- **MCU-specific Implementation Tests**: Testing ESP32-C6 specific implementations

### 3. External Driver Tests
- **Driver Integration Tests**: Testing external hardware driver integration
- **Communication Protocol Tests**: Testing device-specific communication protocols

## Test Guidelines

### Adding New Tests
1. Choose the appropriate test category directory
2. Follow existing naming conventions
3. Include both header and implementation files
4. Update the relevant CMakeLists.txt file
5. Document test cases and expected outcomes

### Test Naming Convention
- Files: `[ComponentName]Test.h` and `[ComponentName]Test.cpp`
- Test classes: `[ComponentName]Test`
- Test methods: `test_[specific_functionality]`

### Test Requirements
- All tests must be self-contained and not depend on external hardware unless specifically marked as integration tests
- Use mocking for hardware dependencies where appropriate
- Include both positive and negative test cases
- Verify error handling and edge cases

## Contributing

When adding new tests:
1. Ensure tests are comprehensive and cover both success and failure scenarios
2. Follow the existing code style and documentation standards
3. Update this README if adding new test categories or significant functionality
4. Ensure all tests pass before submitting changes

## Test Environment

- **Target Platform**: ESP32-C6
- **Build System**: CMake
- **Test Framework**: Custom (can be extended to use Google Test or similar)
- **Mocking**: Custom mocking framework for hardware abstraction

---

*This test suite provides comprehensive coverage of the HardFOC HAL system to ensure reliability and maintainability.*
