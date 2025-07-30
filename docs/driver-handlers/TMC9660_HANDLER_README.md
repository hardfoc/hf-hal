# Tmc9660Handler - Motor Controller Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Tmc9660Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)
![Interface](https://img.shields.io/badge/interface-SPI%20|%20UART-green.svg)

**Unified handler for TMC9660 motor controller with GPIO and ADC integration**

</div>

## 📋 Overview

The `Tmc9660Handler` is a unified handler for TMC9660 motor controller that provides a modern, comprehensive interface for motor control, GPIO expansion, and ADC functionality. It supports both SPI and UART communication interfaces, offers BaseGpio and BaseAdc compatible wrappers, and provides direct access to the TMC9660 driver for advanced motor control operations.

### ✨ Key Features

- **🚗 Motor Control**: Advanced motor control with FOC, stepper, and BLDC support
- **🔌 GPIO Expansion**: 18 internal GPIO pins with BaseGpio compatibility
- **📊 ADC Integration**: 12 ADC channels with BaseAdc compatibility
- **🌡️ Temperature Monitoring**: Internal temperature sensor support
- **📡 Multi-Interface**: SPI and UART communication support
- **🛡️ Thread-Safe**: Concurrent access from multiple tasks
- **⚡ High Performance**: Optimized motor control algorithms
- **🏥 Health Monitoring**: Comprehensive diagnostics and error handling

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   Tmc9660Handler                               │
├─────────────────────────────────────────────────────────────────┤
│  Motor Control    │ FOC, stepper, BLDC motor control          │
├─────────────────────────────────────────────────────────────────┤
│  GPIO Expansion   │ 18 internal GPIO pins with BaseGpio       │
├─────────────────────────────────────────────────────────────────┤
│  ADC Integration  │ 12 ADC channels with BaseAdc              │
├─────────────────────────────────────────────────────────────────┤
│  TMC9660 Driver   │ Low-level motor controller interface       │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Motor Control

```cpp
#include "utils-and-drivers/driver-handlers/Tmc9660Handler.h"
#include "component-handlers/CommChannelsManager.h"

void tmc9660_basic_example() {
    // Get SPI interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) {
        printf("SPI interface not available\n");
        return;
    }
    
    // Create TMC9660 handler
    Tmc9660Handler handler(*spi);
    
    // Initialize handler
    if (!handler.Initialize()) {
        printf("Failed to initialize TMC9660\n");
        return;
    }
    
    // Access motor control
    auto& driver = handler.driver();
    printf("TMC9660 motor controller ready\n");
    
    // Access GPIO
    auto& gpio = handler.gpio(17);  // GPIO pin 17
    gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    gpio.SetActive(true);
    
    // Access ADC
    auto& adc = handler.adc();
    float voltage;
    if (adc.ReadChannelV(0, voltage) == hf_adc_err_t::HF_ADC_ERR_NONE) {
        printf("ADC channel 0: %.2fV\n", voltage);
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class Tmc9660Handler {
public:
    // SPI constructor
    explicit Tmc9660Handler(BaseSpi& spi_interface,
                           uint8_t address = 0,
                           const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig) noexcept;

    // UART constructor
    explicit Tmc9660Handler(BaseUart& uart_interface,
                           uint8_t address = 0,
                           const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig) noexcept;

    // Initialization
    bool Initialize();
    bool IsDriverReady() const noexcept;
};
```

#### Motor Control Access
```cpp
// Direct driver access
TMC9660& driver() noexcept;
const TMC9660& driver() const noexcept;

// Communication mode
CommMode GetCommMode() const noexcept;
bool HasSpiInterface() const noexcept;
bool HasUartInterface() const noexcept;
bool SwitchCommInterface(CommMode mode);

// Bootloader configuration
const tmc9660::BootloaderConfig& bootConfig() const noexcept;
```

#### GPIO Integration
```cpp
// GPIO wrapper class
class Gpio : public BaseGpio {
public:
    // BaseGpio interface implementation
    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    bool IsPinAvailable() const noexcept override;
    hf_u8_t GetMaxPins() const noexcept override;
    const char* GetDescription() const noexcept override;
    
    // Protected implementation methods
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;
    hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;
    hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;
    hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;
    hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;
    hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;
};

// GPIO access
Gpio& gpio(uint8_t gpioNumber);
```

#### ADC Integration
```cpp
// ADC wrapper class
class Adc : public BaseAdc {
public:
    // BaseAdc interface implementation
    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    hf_u8_t GetMaxChannels() const noexcept override;
    bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;
    hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                             hf_u8_t numOfSamplesToAvg = 1,
                             hf_time_t timeBetweenSamples = 0) noexcept override;
    hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                 hf_u8_t numOfSamplesToAvg = 1,
                                 hf_time_t timeBetweenSamples = 0) noexcept override;
    hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                            float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                            hf_time_t timeBetweenSamples = 0) noexcept override;
    hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                     hf_u32_t* readings, float* voltages) noexcept override;
    hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept override;
    hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept override;
    hf_adc_err_t ResetStatistics() noexcept override;
    hf_adc_err_t ResetDiagnostics() noexcept override;

    // TMC9660-specific channel reading methods
    hf_adc_err_t ReadAinChannel(uint8_t ain_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadCurrentSenseChannel(uint8_t current_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadVoltageChannel(uint8_t voltage_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadTemperatureChannel(uint8_t temp_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadMotorDataChannel(uint8_t motor_channel, hf_u32_t& raw_value, float& voltage) noexcept;
};

// ADC access
Adc& adc();
```

#### Temperature Monitoring
```cpp
// Temperature wrapper class
class Temperature : public BaseTemperature {
public:
    // BaseTemperature interface implementation
    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    hf_temp_err_t ReadTemperature(float& temperature_celsius) noexcept override;
    hf_temp_err_t GetStatistics(hf_temp_statistics_t& statistics) noexcept override;
    hf_temp_err_t GetDiagnostics(hf_temp_diagnostics_t& diagnostics) noexcept override;
    hf_temp_err_t ResetStatistics() noexcept override;
    hf_temp_err_t ResetDiagnostics() noexcept override;
    hf_u32_t GetCapabilities() const noexcept override;
};

// Temperature access
Temperature& temperature();
```

## 🎯 Hardware Support

### TMC9660 Features

- **Motor Control**: FOC, stepper, and BLDC motor support
- **GPIO Expansion**: 18 internal GPIO pins (0-17)
- **ADC Channels**: 12 ADC channels for analog measurements
- **Communication**: SPI and UART interfaces
- **Temperature Monitoring**: Internal temperature sensor
- **Current Sensing**: Multiple current sense channels
- **Voltage Monitoring**: Supply and driver voltage monitoring
- **Motor Data**: Current, velocity, and position feedback

### GPIO Pin Mapping

The TMC9660 provides 18 internal GPIO pins (0-17) that can be configured as inputs or outputs with pull-up/pull-down resistors and interrupt support.

### ADC Channel Types

- **AIN Channels (0-3)**: External analog inputs
- **Current Sense Channels (I0-I3)**: Motor current monitoring
- **Voltage Channels**: Supply and driver voltage monitoring
- **Temperature Channels**: Chip and external temperature sensors
- **Motor Data Channels**: Current, velocity, and position feedback

## 📊 Examples

### Basic Motor Control

```cpp
void basic_motor_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    // Access motor driver
    auto& driver = handler.driver();
    
    // Configure motor parameters
    driver.setMotorType(tmc9660::tmcl::MotorType::BLDC);
    driver.setCommutationMode(tmc9660::tmcl::CommutationMode::SIX_STEP);
    
    // Set motor parameters
    driver.setMotorPolePairs(4);
    driver.setMaxCurrent(2.0f);  // 2A
    driver.setMaxVelocity(1000.0f);  // 1000 RPM
    
    // Enable motor
    driver.enableMotor();
    
    // Set target velocity
    driver.setTargetVelocity(500.0f);  // 500 RPM
    
    printf("Motor control initialized\n");
}
```

### GPIO Usage

```cpp
void gpio_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    // Configure GPIO pins
    auto& gpio0 = handler.gpio(0);
    auto& gpio1 = handler.gpio(1);
    
    // Set up outputs
    gpio0.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    gpio0.SetOutputMode(hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
    gpio0.SetActive(true);
    
    // Set up inputs with pull-up
    gpio1.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    gpio1.SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULLUP);
    
    // Read input state
    hf_gpio_level_t level;
    if (gpio1.GetPinLevel(level) == hf_gpio_err_t::HF_GPIO_ERR_NONE) {
        printf("GPIO 1 level: %s\n", level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH ? "HIGH" : "LOW");
    }
    
    // Toggle output
    gpio0.Toggle();
}
```

### ADC Measurements

```cpp
void adc_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    auto& adc = handler.adc();
    
    // Read external analog input
    float ain_voltage;
    if (adc.ReadChannelV(0, ain_voltage) == hf_adc_err_t::HF_ADC_ERR_NONE) {
        printf("AIN0 voltage: %.2fV\n", ain_voltage);
    }
    
    // Read motor current
    float current_voltage;
    if (adc.ReadCurrentSenseChannel(0, current_voltage) == hf_adc_err_t::HF_ADC_ERR_NONE) {
        printf("Motor current voltage: %.2fV\n", current_voltage);
    }
    
    // Read supply voltage
    float supply_voltage;
    if (adc.ReadVoltageChannel(0, supply_voltage) == hf_adc_err_t::HF_ADC_ERR_NONE) {
        printf("Supply voltage: %.2fV\n", supply_voltage);
    }
    
    // Read chip temperature
    float temp_voltage;
    if (adc.ReadTemperatureChannel(0, temp_voltage) == hf_adc_err_t::HF_ADC_ERR_NONE) {
        printf("Chip temperature voltage: %.2fV\n", temp_voltage);
    }
}
```

### Temperature Monitoring

```cpp
void temperature_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    auto& temp = handler.temperature();
    
    // Read temperature
    float temperature;
    if (temp.ReadTemperature(temperature) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Chip temperature: %.1f°C\n", temperature);
    }
    
    // Get temperature statistics
    hf_temp_statistics_t stats;
    if (temp.GetStatistics(stats) == hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("Temperature statistics:\n");
        printf("  Min: %.1f°C\n", stats.min_temperature_celsius);
        printf("  Max: %.1f°C\n", stats.max_temperature_celsius);
        printf("  Average: %.1f°C\n", stats.average_temperature_celsius);
        printf("  Readings: %u\n", stats.total_readings);
    }
}
```

### Advanced Motor Control

```cpp
void advanced_motor_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    auto& driver = handler.driver();
    auto& adc = handler.adc();
    
    // Configure for FOC control
    driver.setMotorType(tmc9660::tmcl::MotorType::BLDC);
    driver.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC);
    
    // Set motor parameters
    driver.setMotorPolePairs(4);
    driver.setMaxCurrent(3.0f);
    driver.setMaxVelocity(2000.0f);
    
    // Configure PID parameters
    driver.setVelocityP(100.0f);
    driver.setVelocityI(10.0f);
    driver.setVelocityD(1.0f);
    
    // Enable motor
    driver.enableMotor();
    
    // Velocity control loop
    for (int i = 0; i < 100; i++) {
        // Set target velocity (ramp up)
        float target_velocity = 500.0f * (i / 100.0f);
        driver.setTargetVelocity(target_velocity);
        
        // Read motor feedback
        float current_voltage;
        if (adc.ReadMotorDataChannel(0, current_voltage) == hf_adc_err_t::HF_ADC_ERR_NONE) {
            printf("Target: %.0f RPM, Current: %.2fV\n", target_velocity, current_voltage);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz control loop
    }
    
    // Stop motor
    driver.setTargetVelocity(0.0f);
    driver.disableMotor();
}
```

### Communication Interface Switching

```cpp
void interface_switching_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    auto* uart = nullptr; // Get UART interface if available
    
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    printf("Initial communication mode: %s\n", 
           handler.GetCommMode() == CommMode::SPI ? "SPI" : "UART");
    
    // Check available interfaces
    if (handler.HasSpiInterface()) {
        printf("SPI interface available\n");
    }
    
    if (handler.HasUartInterface()) {
        printf("UART interface available\n");
    }
    
    // Switch to UART if available
    if (handler.HasUartInterface()) {
        if (handler.SwitchCommInterface(CommMode::UART)) {
            printf("Switched to UART communication\n");
        } else {
            printf("Failed to switch to UART\n");
        }
    }
    
    // Switch back to SPI
    if (handler.SwitchCommInterface(CommMode::SPI)) {
        printf("Switched back to SPI communication\n");
    }
}
```

### Error Handling

```cpp
void error_handling_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    
    // Check initialization
    if (!handler.Initialize()) {
        printf("ERROR: Failed to initialize TMC9660\n");
        return;
    }
    
    // Check driver readiness
    if (!handler.IsDriverReady()) {
        printf("ERROR: TMC9660 driver not ready\n");
        return;
    }
    
    // Safe GPIO operations
    auto& gpio = handler.gpio(0);
    hf_gpio_err_t result = gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    if (result != hf_gpio_err_t::HF_GPIO_ERR_NONE) {
        printf("ERROR: Failed to set GPIO direction: %d\n", static_cast<int>(result));
        return;
    }
    
    // Safe ADC operations
    auto& adc = handler.adc();
    float voltage;
    hf_adc_err_t adc_result = adc.ReadChannelV(0, voltage);
    if (adc_result != hf_adc_err_t::HF_ADC_ERR_NONE) {
        printf("ERROR: Failed to read ADC: %d\n", static_cast<int>(adc_result));
        return;
    }
    
    // Safe temperature operations
    auto& temp = handler.temperature();
    float temperature;
    hf_temp_err_t temp_result = temp.ReadTemperature(temperature);
    if (temp_result != hf_temp_err_t::HF_TEMP_ERR_NONE) {
        printf("ERROR: Failed to read temperature: %d\n", static_cast<int>(temp_result));
        return;
    }
    
    printf("All operations successful\n");
}
```

## 🔍 Advanced Usage

### Multi-Interface Configuration

```cpp
void multi_interface_config() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    auto* uart = nullptr; // Get UART interface if available
    
    if (!spi) return;
    
    // Create handler with both interfaces
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    // Configure bootloader settings
    tmc9660::BootloaderConfig custom_config = handler.bootConfig();
    custom_config.motor_type = tmc9660::tmcl::MotorType::BLDC;
    custom_config.max_current = 5.0f;
    custom_config.max_velocity = 3000.0f;
    
    // Apply custom configuration
    auto& driver = handler.driver();
    driver.setMotorType(custom_config.motor_type);
    driver.setMaxCurrent(custom_config.max_current);
    driver.setMaxVelocity(custom_config.max_velocity);
    
    printf("Custom configuration applied\n");
}
```

### Integrated System Example

```cpp
void integrated_system_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) return;
    
    Tmc9660Handler handler(*spi);
    if (!handler.Initialize()) return;
    
    // Access all subsystems
    auto& driver = handler.driver();
    auto& gpio = handler.gpio(0);
    auto& adc = handler.adc();
    auto& temp = handler.temperature();
    
    // Configure system
    gpio.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    driver.setMotorType(tmc9660::tmcl::MotorType::BLDC);
    driver.enableMotor();
    
    // System monitoring loop
    for (int i = 0; i < 100; i++) {
        // Read all sensors
        float motor_current, supply_voltage, temperature;
        
        adc.ReadCurrentSenseChannel(0, motor_current);
        adc.ReadVoltageChannel(0, supply_voltage);
        temp.ReadTemperature(temperature);
        
        // Control logic
        if (temperature > 80.0f) {
            gpio.SetActive(true);  // Turn on cooling fan
            driver.setMaxCurrent(1.0f);  // Reduce current
        } else {
            gpio.SetActive(false);  // Turn off cooling fan
            driver.setMaxCurrent(3.0f);  // Normal current
        }
        
        // Monitor supply voltage
        if (supply_voltage < 10.0f) {
            driver.disableMotor();  // Low voltage protection
        }
        
        printf("Temp: %.1f°C, Current: %.2fA, Voltage: %.1fV\n", 
               temperature, motor_current, supply_voltage);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Cleanup
    driver.disableMotor();
    gpio.SetActive(false);
}
```

## 📚 See Also

- **[MotorController Documentation](../component-handlers/MOTOR_CONTROLLER_README.md)** - Motor management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[GpioManager Documentation](../component-handlers/GPIO_MANAGER_README.md)** - GPIO management system
- **[AdcManager Documentation](../component-handlers/ADC_MANAGER_README.md)** - ADC management system

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*