# HardFOC PWM Architecture Documentation

## Overview

The HardFOC PWM system provides a modern, extensible, and thread-safe architecture for PWM generation across different hardware platforms. The system is designed with portability in mind, supporting both MCU-based PWM peripherals and external PWM controller ICs.

## Architecture

### Core Components

1. **BasePwm** - Abstract base class defining the PWM interface
2. **McuPwm** - ESP32C6-specific PWM implementation
3. **SfPwm** - Thread-safe wrapper for any BasePwm implementation
4. **External PWM Drivers** - Support for external PWM ICs (e.g., PCA9685)

### Design Principles

- **Portability**: Abstract interface allows easy porting to different MCUs
- **Extensibility**: Support for both MCU and external IC-based PWM
- **Thread Safety**: Thread-safe wrappers for multi-threaded applications
- **Feature Rich**: Advanced features like fade, complementary PWM, and dead time
- **Error Handling**: Comprehensive error reporting and handling

## Class Hierarchy

```
BasePwm (Abstract Interface)
├── McuPwm (ESP32C6 Implementation)
├── Pca9685Pwm (External IC Implementation)
└── SfPwm (Thread-Safe Wrapper)
```

## Features

### Basic PWM Operations
- Channel configuration with frequency, duty cycle, and resolution
- Start/stop individual channels
- Multi-channel operations
- Real-time duty cycle and frequency adjustment

### Advanced Features (ESP32C6)
- **Phase Control**: Adjust phase relationship between channels
- **Fade Operations**: Hardware-accelerated smooth transitions
- **Complementary PWM**: For H-bridge motor control with dead time
- **Dead Time Control**: Prevent shoot-through in power electronics
- **Callback System**: Event-driven programming with interrupts

### Thread Safety
- Mutex-protected operations
- Configurable timeout for lock acquisition
- Safe concurrent access from multiple threads

## Usage Examples

### Basic MCU PWM Usage

```cpp
#include "McuPwm.h"

// Create MCU PWM controller
auto pwm = std::make_unique<McuPwm>();

if (pwm->Initialize() == HfPwmErr::PWM_SUCCESS) {
    // Configure PWM channel
    PwmChannelConfig config{};
    config.output_pin = GPIO_NUM_2;
    config.frequency_hz = 1000;
    config.resolution_bits = 12;
    config.initial_duty_cycle = 0.5f;
    
    pwm->ConfigureChannel(0, config);
    pwm->Start(0);
    
    // Adjust duty cycle
    pwm->SetDutyCycle(0, 0.75f);
}
```

### Thread-Safe PWM Usage

```cpp
#include "SfPwm.h"
#include "McuPwm.h"

// Create thread-safe PWM wrapper
SfPwm sf_pwm(std::make_unique<McuPwm>());

if (sf_pwm.Initialize() == HfPwmErr::PWM_SUCCESS) {
    // Configure and use PWM from multiple threads
    // All operations are automatically thread-safe
    
    std::thread t1([&sf_pwm]() {
        sf_pwm.SetDutyCycle(0, 0.3f);
    });
    
    std::thread t2([&sf_pwm]() {
        sf_pwm.SetDutyCycle(1, 0.7f);
    });
    
    t1.join();
    t2.join();
}
```

### External PWM IC Usage

```cpp
#include "Pca9685Pwm.h"
#include "McuI2c.h"

// Create PCA9685 PWM controller
auto i2c = std::make_unique<McuI2c>();
auto pca9685 = std::make_unique<Pca9685Pwm>(std::move(i2c), 0x40);

if (pca9685->Initialize() == HfPwmErr::PWM_SUCCESS) {
    // Configure servo channel
    PwmChannelConfig config{};
    config.frequency_hz = 50; // 50Hz for servo
    config.initial_duty_cycle = 0.075f; // 1.5ms pulse
    
    pca9685->ConfigureChannel(0, config);
    pca9685->Start(0);
}
```

### Advanced ESP32C6 Features

```cpp
#include "McuPwm.h"

auto pwm = std::make_unique<McuPwm>();
pwm->Initialize();

// Complementary PWM with dead time
PwmComplementaryConfig comp_config{};
comp_config.dead_time_ns = 1000; // 1μs dead time
comp_config.enable_complementary = true;

pwm->ConfigureComplementary(0, 1, comp_config);

// Fade configuration
PwmFadeConfig fade_config{};
fade_config.target_duty_cycle = 1.0f;
fade_config.fade_time_ms = 2000;
fade_config.fade_mode = PwmFadeMode::LINEAR;

pwm->ConfigureFade(0, fade_config);
pwm->StartFade(0);
```

## Error Handling

The PWM system uses a comprehensive error enumeration:

```cpp
enum class HfPwmErr {
    PWM_SUCCESS = 0,
    PWM_INVALID_ARGUMENT,
    PWM_INVALID_CHANNEL,
    PWM_INVALID_FREQUENCY,
    PWM_INVALID_DUTY_CYCLE,
    PWM_HARDWARE_ERROR,
    PWM_NOT_INITIALIZED,
    PWM_ALREADY_INITIALIZED,
    PWM_TIMEOUT,
    PWM_NOT_SUPPORTED,
    PWM_RESOURCE_BUSY,
    PWM_INSUFFICIENT_RESOURCES
};
```

## Configuration Structures

### PwmChannelConfig
```cpp
struct PwmChannelConfig {
    gpio_num_t output_pin;
    uint32_t frequency_hz;
    uint8_t resolution_bits;
    float initial_duty_cycle;
    uint8_t timer_id;
    uint8_t channel_id;
    bool invert_output;
    bool idle_level_high;
};
```

### PwmFadeConfig
```cpp
struct PwmFadeConfig {
    float target_duty_cycle;
    uint32_t fade_time_ms;
    PwmFadeMode fade_mode;
    PwmCallback fade_complete_callback;
    void* user_data;
};
```

### PwmComplementaryConfig
```cpp
struct PwmComplementaryConfig {
    uint16_t dead_time_ns;
    bool enable_complementary;
    PwmDeadTimeMode dead_time_mode;
};
```

## Platform Support

### ESP32C6 (Current)
- Full feature support including advanced capabilities
- Hardware-accelerated fade operations
- Complementary PWM with dead time control
- Up to 8 PWM channels
- 14-bit resolution support

### Future Platforms
The abstract interface allows easy porting to:
- STM32 series MCUs
- Nordic nRF series
- Raspberry Pi Pico
- Other ESP32 variants

## External PWM IC Support

### PCA9685 (Implemented)
- 16-channel 12-bit PWM controller
- I2C interface (up to 1MHz)
- Configurable frequency (24Hz to 1526Hz)
- Servo and LED control applications

### Future External ICs
- TLC5940 (16-channel LED driver)
- PWM expansion modules
- Custom PWM controllers

## Thread Safety

The `SfPwm` class provides thread-safe access to any `BasePwm` implementation:

- **Mutex Protection**: All operations are protected by mutexes
- **Timeout Control**: Configurable timeout for lock acquisition
- **Error Handling**: Timeout errors are properly reported
- **Performance**: Minimal locking overhead

## Building and Integration

### CMake Integration

Add to your CMakeLists.txt:

```cmake
# Include PWM system
target_sources(your_target PRIVATE
    "path/to/McuPwm.cpp"
    "path/to/SfPwm.cpp"
    # Add external drivers as needed
    "path/to/Pca9685Pwm.cpp"
)

target_include_directories(your_target PRIVATE
    "path/to/inc"
)
```

### ESP-IDF Component

The PWM system is integrated as an ESP-IDF component with proper dependencies.

## Testing

Comprehensive testing includes:
- Unit tests for each PWM implementation
- Integration tests with hardware
- Thread safety stress tests
- Performance benchmarks

## Best Practices

1. **Use SfPwm for multi-threaded applications**
2. **Configure channels before starting PWM**
3. **Handle errors appropriately**
4. **Use appropriate resolution for your application**
5. **Consider power consumption in portable applications**
6. **Implement proper shutdown procedures**

## Migration Guide

### From Legacy PWM System

1. Replace direct register access with PWM interface calls
2. Update configuration structures
3. Add error handling
4. Use thread-safe wrappers where needed

### Example Migration

```cpp
// Legacy (direct register access)
ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_12_BIT,
    .freq_hz = 1000,
};
ledc_timer_config(&timer_conf);

// Modern (PWM interface)
PwmChannelConfig config{};
config.frequency_hz = 1000;
config.resolution_bits = 12;
config.output_pin = GPIO_NUM_2;

auto pwm = std::make_unique<McuPwm>();
pwm->Initialize();
pwm->ConfigureChannel(0, config);
```

## Performance Considerations

- **MCU PWM**: Zero-copy hardware operations, minimal CPU overhead
- **External PWM**: I2C communication overhead, suitable for lower frequency updates
- **Thread-Safe**: Minimal mutex overhead, lock-free read operations where possible

## Troubleshooting

### Common Issues

1. **PWM_HARDWARE_ERROR**: Check pin configuration and hardware connections
2. **PWM_INVALID_FREQUENCY**: Verify frequency is within supported range
3. **PWM_TIMEOUT**: Increase mutex timeout or reduce thread contention
4. **PWM_RESOURCE_BUSY**: Ensure proper channel allocation

### Debug Features

- Comprehensive error reporting
- Channel state tracking
- Hardware capability queries

## Future Enhancements

1. **DMA Support**: For high-frequency pattern generation
2. **Pattern Generation**: Complex waveform generation
3. **Synchronization**: Multi-channel synchronization
4. **Power Management**: Dynamic frequency scaling integration
5. **Real-time Control**: RT-thread integration for deterministic timing

## Conclusion

The HardFOC PWM architecture provides a robust, extensible, and portable foundation for PWM generation in embedded systems. The combination of abstract interfaces, concrete implementations, and thread-safe wrappers ensures that the system can meet the needs of both simple and complex applications while maintaining code quality and maintainability.
