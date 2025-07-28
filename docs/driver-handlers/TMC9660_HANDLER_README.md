# Tmc9660Handler - Motor Controller Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Tmc9660Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)
![Interface](https://img.shields.io/badge/interface-SPI%20|%20UART-green.svg)

**Unified driver handler for TMC9660 motor controllers with GPIO and ADC integration**

</div>

## 📋 Overview

The `Tmc9660Handler` is a comprehensive driver handler that provides unified access to TMC9660 motor controller functionality. It integrates motor control, GPIO operations, and ADC readings in a single interface, supporting both SPI and UART communication protocols.

### ✨ Key Features

- **🎛️ Complete Motor Control**: Stepper and BLDC motor support
- **🔌 Dual Communication**: SPI and UART interfaces
- **📍 GPIO Integration**: 8 configurable GPIO pins
- **📊 ADC Support**: 3 analog input channels
- **🛡️ Safety Features**: Fault detection and protection
- **⚙️ Flexible Configuration**: Customizable bootloader settings
- **🏥 Health Monitoring**: Communication and status monitoring
- **🔧 Easy Integration**: BaseGpio and BaseAdc compatible wrappers

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Tmc9660Handler                             │
├─────────────────────────────────────────────────────────────────┤
│  Communication      │ SPI/UART interface abstraction          │
├─────────────────────────────────────────────────────────────────┤
│  TMC9660 Driver     │ Core motor controller driver            │
├─────────────────────────────────────────────────────────────────┤
│  GPIO Wrapper       │ BaseGpio compatible GPIO interface      │
├─────────────────────────────────────────────────────────────────┤
│  ADC Wrapper        │ BaseAdc compatible ADC interface        │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### SPI-Based TMC9660

```cpp
#include "utils-and-drivers/driver-handlers/Tmc9660Handler.h"
#include "component-handlers/CommChannelsManager.h"

void spi_tmc9660_example() {
    // Get SPI interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi_device = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi_device) return;
    
    // Create TMC9660 handler with SPI
    Tmc9660Handler handler(*spi_device, 0x00);  // Address 0x00
    
    // Initialize handler
    if (handler.Initialize()) {
        auto tmc = handler.GetTmc9660Driver();
        
        // Configure motor
        tmc->SetMaxCurrent(1000);     // 1A
        tmc->SetVelocityLimit(1000);  // 1000 RPM
        tmc->EnableMotor(true);
        
        printf("TMC9660 ready via SPI\n");
    }
}
```

### UART-Based TMC9660

```cpp
void uart_tmc9660_example() {
    // Get UART interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* uart = comm.GetUart(0);  // UART0
    if (!uart) return;
    
    // Create TMC9660 handler with UART
    Tmc9660Handler handler(*uart, 0x01);  // Address 0x01
    
    // Initialize handler
    if (handler.Initialize()) {
        auto tmc = handler.GetTmc9660Driver();
        
        // Configure motor for UART operation
        tmc->SetMaxCurrent(800);
        tmc->SetVelocityLimit(500);
        tmc->EnableMotor(true);
        
        printf("TMC9660 ready via UART\n");
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
    Tmc9660Handler(BaseSpi& spi_interface, uint8_t address,
                   const tmc9660::BootloaderConfig& bootCfg = kDefaultBootConfig);
    
    // UART constructor  
    Tmc9660Handler(BaseUart& uart_interface, uint8_t address,
                   const tmc9660::BootloaderConfig& bootCfg = kDefaultBootConfig);
    
    // Dual interface constructor
    Tmc9660Handler(BaseSpi& spi_interface, BaseUart& uart_interface, uint8_t address,
                   const tmc9660::BootloaderConfig& bootCfg = kDefaultBootConfig);
    
    // Initialization
    bool Initialize() noexcept;
    bool IsInitialized() const noexcept;
    void Deinitialize() noexcept;
    
    // Communication testing
    bool TestCommunication() noexcept;
};
```

#### Driver Access
```cpp
// Core TMC9660 driver
std::shared_ptr<TMC9660> GetTmc9660Driver() noexcept;

// GPIO wrapper (BaseGpio compatible)
std::shared_ptr<BaseGpio> GetGpioWrapper() noexcept;

// ADC wrapper (BaseAdc compatible)  
std::shared_ptr<BaseAdc> GetAdcWrapper() noexcept;

// Communication interfaces
BaseSpi* GetSpiInterface() noexcept;
BaseUart* GetUartInterface() noexcept;
```

#### Configuration
```cpp
// Default bootloader configuration
static const tmc9660::BootloaderConfig kDefaultBootConfig;

// Device information
uint8_t GetDeviceAddress() const noexcept;
bool HasSpiInterface() const noexcept;
bool HasUartInterface() const noexcept;
```

## 🎯 Hardware Features

### TMC9660 Motor Controller Capabilities

| Feature | Range/Options | Description |
|---------|---------------|-------------|
| **Motor Types** | Stepper, BLDC | Configurable motor type support |
| **Current Control** | 0-2000mA | Precise current regulation |
| **Velocity Control** | 0-10000 RPM | High-speed operation |
| **Position Control** | 32-bit | Absolute/relative positioning |
| **Microsteps** | 1-256 | Configurable resolution |
| **PWM Frequency** | 15-100 kHz | Adjustable switching frequency |
| **GPIO Pins** | 8 digital I/O | Configurable input/output |
| **ADC Channels** | 3 × 12-bit | Analog feedback inputs |

### Communication Interfaces

```cpp
// SPI Configuration (TMC9660 as slave)
struct SpiConfig {
    uint32_t max_clock_hz = 1000000;     // Max 1MHz
    uint8_t mode = 3;                    // SPI Mode 3 (CPOL=1, CPHA=1)
    uint8_t bits_per_word = 8;           // 8-bit transfers
    uint8_t transfer_size = 8;           // 8-byte datagrams
};

// UART Configuration (TMCL protocol)
struct UartConfig {
    uint32_t baud_rate = 115200;         // Default baud rate
    uint8_t data_bits = 8;               // 8 data bits
    uint8_t stop_bits = 1;               // 1 stop bit
    uart_parity_t parity = UART_PARITY_DISABLE;  // No parity
    uint8_t datagram_size = 9;           // 9-byte TMCL datagrams
};
```

## 🔧 Configuration

### Bootloader Configuration

```cpp
// Default configuration
static const tmc9660::BootloaderConfig kDefaultBootConfig = {
    // Motor configuration
    .motor_type = tmc9660::MotorType::STEPPER,
    .current_limit_ma = 1000,
    .microstep_resolution = 256,
    
    // Motion limits
    .velocity_limit_rpm = 1000,
    .acceleration_limit_rpm_per_s = 1000,
    .position_limit_steps = 0,  // No limit
    
    // Protection features
    .enable_stallguard = true,
    .stallguard_threshold = 100,
    .enable_coolstep = true,
    .coolstep_threshold = 500,
    
    // PWM configuration
    .enable_spreadcycle = true,
    .enable_stealth_chop = false,
    .pwm_frequency_khz = 35,
    .pwm_amplitude = 255,
    
    // Safety features
    .enable_overtemperature_protection = true,
    .enable_overcurrent_protection = true,
    .thermal_limit_celsius = 100
};
```

### Custom Configuration Examples

```cpp
// High-performance stepper configuration
tmc9660::BootloaderConfig high_perf_stepper = {
    .motor_type = tmc9660::MotorType::STEPPER,
    .current_limit_ma = 1500,
    .microstep_resolution = 512,  // Higher resolution
    .velocity_limit_rpm = 3000,   // Higher speed
    .acceleration_limit_rpm_per_s = 5000,
    .enable_spreadcycle = true,   // Better high-speed performance
    .enable_stealth_chop = false,
    .pwm_frequency_khz = 70       // Higher PWM frequency
};

// BLDC motor configuration
tmc9660::BootloaderConfig bldc_config = {
    .motor_type = tmc9660::MotorType::BLDC,
    .current_limit_ma = 1200,
    .velocity_limit_rpm = 5000,
    .acceleration_limit_rpm_per_s = 10000,
    .enable_stallguard = false,   // Not applicable for BLDC
    .enable_coolstep = false,
    .enable_spreadcycle = false,
    .enable_stealth_chop = true,  // Quiet operation
    .pwm_frequency_khz = 50
};
```

## 📊 Examples

### Basic Motor Control

```cpp
#include "utils-and-drivers/driver-handlers/Tmc9660Handler.h"

void basic_motor_control() {
    // Setup communication (assuming SPI)
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    
    // Create and initialize handler
    Tmc9660Handler handler(*spi, 0x00);
    if (!handler.Initialize()) {
        printf("Failed to initialize TMC9660\n");
        return;
    }
    
    auto tmc = handler.GetTmc9660Driver();
    
    // Basic motor setup
    tmc->SetMotorType(tmc9660::MotorType::STEPPER);
    tmc->SetMaxCurrent(800);           // 800mA
    tmc->SetMicrostepResolution(256);  // 256 microsteps/step
    tmc->SetVelocityLimit(1000);       // 1000 RPM
    tmc->SetAccelerationLimit(2000);   // 2000 RPM/s
    
    // Enable motor
    tmc->EnableMotor(true);
    
    // Execute movements
    printf("Executing motor movements...\n");
    
    // Move to absolute position
    tmc->MoveToPosition(1000);
    while (tmc->IsMoving()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("Reached position: %ld\n", tmc->GetActualPosition());
    
    // Move relative
    tmc->MoveRelative(-500);
    while (tmc->IsMoving()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("Final position: %ld\n", tmc->GetActualPosition());
    
    // Disable motor
    tmc->EnableMotor(false);
}
```

### Velocity Control Mode

```cpp
void velocity_control_example() {
    // Setup handler (abbreviated)
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto tmc = handler.GetTmc9660Driver();
    
    // Configure for velocity control
    tmc->SetControlMode(tmc9660::ControlMode::VELOCITY);
    tmc->SetMaxCurrent(1000);
    tmc->SetAccelerationLimit(3000);
    
    tmc->EnableMotor(true);
    
    // Velocity ramp test
    printf("Velocity control test:\n");
    
    // Accelerate
    for (int rpm = 0; rpm <= 2000; rpm += 200) {
        printf("Setting velocity: %d RPM\n", rpm);
        tmc->SetTargetVelocity(rpm);
        
        // Wait for velocity to stabilize
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        int32_t actual = tmc->GetActualVelocity();
        printf("  Actual: %ld RPM\n", actual);
    }
    
    // Decelerate
    for (int rpm = 2000; rpm >= 0; rpm -= 200) {
        tmc->SetTargetVelocity(rpm);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    tmc->EnableMotor(false);
}
```

### GPIO Operations

```cpp
void gpio_operations_example() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    
    // Get GPIO wrapper
    auto gpio = handler.GetGpioWrapper();
    if (!gpio) {
        printf("GPIO wrapper not available\n");
        return;
    }
    
    // Configure GPIO pins
    gpio->ConfigurePin(0, false);  // Pin 0 as output
    gpio->ConfigurePin(1, false);  // Pin 1 as output
    gpio->ConfigurePin(2, true);   // Pin 2 as input
    gpio->ConfigurePin(3, true);   // Pin 3 as input
    
    printf("TMC9660 GPIO operations:\n");
    
    for (int i = 0; i < 10; i++) {
        // Toggle output pins
        gpio->SetPin(0, i % 2);
        gpio->SetPin(1, (i + 1) % 2);
        
        // Read input pins
        bool input2 = gpio->GetPin(2);
        bool input3 = gpio->GetPin(3);
        
        printf("Outputs: %d %d, Inputs: %s %s\n",
               i % 2, (i + 1) % 2,
               input2 ? "HIGH" : "LOW",
               input3 ? "HIGH" : "LOW");
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### ADC Monitoring

```cpp
void adc_monitoring_example() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    
    // Get ADC wrapper
    auto adc = handler.GetAdcWrapper();
    if (!adc) {
        printf("ADC wrapper not available\n");
        return;
    }
    
    printf("TMC9660 ADC monitoring:\n");
    
    for (int i = 0; i < 50; i++) {
        // Read all ADC channels
        uint16_t ain1 = adc->ReadRaw(0);  // AIN1
        uint16_t ain2 = adc->ReadRaw(1);  // AIN2
        uint16_t ain3 = adc->ReadRaw(2);  // AIN3
        
        // Convert to voltages (assuming 3.3V reference)
        float volt1 = (ain1 * 3.3f) / 4096.0f;
        float volt2 = (ain2 * 3.3f) / 4096.0f;
        float volt3 = (ain3 * 3.3f) / 4096.0f;
        
        printf("ADC: AIN1=%.3fV, AIN2=%.3fV, AIN3=%.3fV\n",
               volt1, volt2, volt3);
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

### Fault Detection and Handling

```cpp
void fault_detection_example() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto tmc = handler.GetTmc9660Driver();
    
    // Enable fault detection features
    tmc->EnableStallguard(true);
    tmc->SetStallguardThreshold(100);
    tmc->EnableOvertemperatureProtection(true);
    tmc->EnableOvercurrentProtection(true);
    
    // Configure motor for operation
    tmc->SetMaxCurrent(1000);
    tmc->SetVelocityLimit(500);
    tmc->EnableMotor(true);
    
    printf("Fault detection active. Starting motor...\n");
    
    // Start continuous rotation
    tmc->SetTargetVelocity(300);
    
    // Monitor for faults
    bool fault_detected = false;
    for (int i = 0; i < 1000 && !fault_detected; i++) {
        auto status = tmc->GetDriverStatus();
        
        // Check for various fault conditions
        if (status.stall_detected) {
            printf("FAULT: Stall detected at step %d\n", i);
            fault_detected = true;
        }
        
        if (status.overtemperature_warning) {
            printf("WARNING: High temperature detected\n");
        }
        
        if (status.overtemperature_error) {
            printf("FAULT: Overtemperature shutdown\n");
            fault_detected = true;
        }
        
        if (status.overcurrent_detected) {
            printf("FAULT: Overcurrent detected\n");
            fault_detected = true;
        }
        
        if (status.communication_error) {
            printf("FAULT: Communication error\n");
            fault_detected = true;
        }
        
        // Print periodic status
        if (i % 100 == 0) {
            printf("Status: Pos=%ld, Vel=%ld, Current=%dmA, Temp=%d°C\n",
                   tmc->GetActualPosition(),
                   tmc->GetActualVelocity(),
                   status.actual_current_ma,
                   status.temperature_celsius);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Handle fault condition
    if (fault_detected) {
        printf("Fault detected, stopping motor\n");
        tmc->SetTargetVelocity(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        tmc->EnableMotor(false);
    } else {
        printf("No faults detected during test\n");
        tmc->EnableMotor(false);
    }
}
```

### Dual Interface Communication

```cpp
void dual_interface_example() {
    // Get both SPI and UART interfaces
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    auto* uart = comm.GetUart(0);
    
    if (!spi || !uart) {
        printf("Required interfaces not available\n");
        return;
    }
    
    // Create handler with both interfaces
    Tmc9660Handler handler(*spi, *uart, 0x00);
    
    if (!handler.Initialize()) {
        printf("Failed to initialize dual-interface TMC9660\n");
        return;
    }
    
    printf("TMC9660 initialized with dual interface support\n");
    printf("SPI interface: %s\n", handler.HasSpiInterface() ? "Available" : "Not available");
    printf("UART interface: %s\n", handler.HasUartInterface() ? "Available" : "Not available");
    
    auto tmc = handler.GetTmc9660Driver();
    
    // Test communication on both interfaces
    printf("Testing SPI communication...\n");
    bool spi_ok = handler.TestCommunication();
    printf("SPI test: %s\n", spi_ok ? "PASS" : "FAIL");
    
    // Note: Testing UART would require switching the device to UART mode
    // which is typically done through hardware configuration
    
    // Use the device normally
    tmc->SetMaxCurrent(800);
    tmc->EnableMotor(true);
    printf("Motor control via dual interface ready\n");
}
```

### Performance Benchmarking

```cpp
void performance_benchmark() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto tmc = handler.GetTmc9660Driver();
    
    printf("TMC9660 Performance Benchmark\n");
    printf("=============================\n");
    
    // Test communication speed
    auto start_time = esp_timer_get_time();
    constexpr int NUM_READS = 1000;
    
    for (int i = 0; i < NUM_READS; i++) {
        tmc->GetActualPosition();
    }
    
    auto end_time = esp_timer_get_time();
    float avg_time_us = (end_time - start_time) / (float)NUM_READS;
    float reads_per_sec = 1000000.0f / avg_time_us;
    
    printf("Register read performance:\n");
    printf("  Average time: %.2f µs\n", avg_time_us);
    printf("  Reads per second: %.0f\n", reads_per_sec);
    
    // Test GPIO performance
    auto gpio = handler.GetGpioWrapper();
    if (gpio) {
        gpio->ConfigurePin(0, false);  // Output pin
        
        start_time = esp_timer_get_time();
        constexpr int NUM_TOGGLES = 1000;
        
        for (int i = 0; i < NUM_TOGGLES; i++) {
            gpio->SetPin(0, i % 2);
        }
        
        end_time = esp_timer_get_time();
        avg_time_us = (end_time - start_time) / (float)NUM_TOGGLES;
        float toggles_per_sec = 1000000.0f / avg_time_us;
        
        printf("GPIO toggle performance:\n");
        printf("  Average time: %.2f µs\n", avg_time_us);
        printf("  Toggles per second: %.0f\n", toggles_per_sec);
    }
    
    // Test ADC performance
    auto adc = handler.GetAdcWrapper();
    if (adc) {
        start_time = esp_timer_get_time();
        constexpr int NUM_ADC_READS = 1000;
        
        for (int i = 0; i < NUM_ADC_READS; i++) {
            adc->ReadRaw(0);
        }
        
        end_time = esp_timer_get_time();
        avg_time_us = (end_time - start_time) / (float)NUM_ADC_READS;
        float adc_reads_per_sec = 1000000.0f / avg_time_us;
        
        printf("ADC read performance:\n");
        printf("  Average time: %.2f µs\n", avg_time_us);
        printf("  Reads per second: %.0f\n", adc_reads_per_sec);
    }
}
```

## 🔍 Advanced Usage

### Custom Communication Interface

```cpp
// Example of creating a custom SPI communication interface
class CustomTmc9660SpiInterface : public SPITMC9660CommInterface {
public:
    CustomTmc9660SpiInterface(BaseSpi& spi) : spi_(spi) {}
    
    bool spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept override {
        // Custom SPI transfer implementation with enhanced error handling
        if (!spi_.IsInitialized()) {
            return false;
        }
        
        // Add retry logic
        constexpr int MAX_RETRIES = 3;
        for (int retry = 0; retry < MAX_RETRIES; retry++) {
            if (spi_.Transfer(tx.data(), rx.data(), 8)) {
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(1));  // Brief delay before retry
        }
        
        return false;
    }
    
private:
    BaseSpi& spi_;
};
```

### Motor Profiling and Tuning

```cpp
void motor_profiling() {
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto tmc = handler.GetTmc9660Driver();
    
    printf("Motor Profiling and Tuning\n");
    printf("==========================\n");
    
    // Test different current settings
    std::array<uint16_t, 5> currents = {500, 750, 1000, 1250, 1500};
    
    for (auto current : currents) {
        printf("\nTesting current: %umA\n", current);
        tmc->SetMaxCurrent(current);
        tmc->EnableMotor(true);
        
        // Measure acceleration performance
        auto start_time = esp_timer_get_time();
        tmc->SetTargetVelocity(1000);
        
        // Wait for target velocity
        while (std::abs(tmc->GetActualVelocity() - 1000) > 50) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        auto end_time = esp_timer_get_time();
        float accel_time_ms = (end_time - start_time) / 1000.0f;
        
        printf("  Acceleration time to 1000 RPM: %.1f ms\n", accel_time_ms);
        
        // Stop motor
        tmc->SetTargetVelocity(0);
        vTaskDelay(pdMS_TO_TICKS(500));
        tmc->EnableMotor(false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Test different microstep resolutions
    std::array<uint16_t, 4> microsteps = {64, 128, 256, 512};
    
    tmc->SetMaxCurrent(1000);  // Use optimal current from above
    
    for (auto usteps : microsteps) {
        printf("\nTesting microsteps: %u\n", usteps);
        tmc->SetMicrostepResolution(usteps);
        tmc->EnableMotor(true);
        
        // Measure positioning accuracy
        tmc->MoveToPosition(0);  // Home position
        while (tmc->IsMoving()) vTaskDelay(pdMS_TO_TICKS(10));
        
        // Move to test position
        int32_t target_pos = 1000;
        tmc->MoveToPosition(target_pos);
        while (tmc->IsMoving()) vTaskDelay(pdMS_TO_TICKS(10));
        
        int32_t actual_pos = tmc->GetActualPosition();
        int32_t error = actual_pos - target_pos;
        
        printf("  Position accuracy: target=%ld, actual=%ld, error=%ld\n",
               target_pos, actual_pos, error);
        
        tmc->EnableMotor(false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 🚨 Error Handling

### Comprehensive Error Management

```cpp
void comprehensive_error_handling() {
    // Initialize with error checking
    auto& comm = CommChannelsManager::GetInstance();
    if (!comm.EnsureInitialized()) {
        printf("ERROR: Failed to initialize communication manager\n");
        return;
    }
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi) {
        printf("ERROR: SPI device not available\n");
        return;
    }
    
    // Test communication interface first
    if (!spi->IsInitialized()) {
        printf("ERROR: SPI interface not initialized\n");
        return;
    }
    
    // Create handler with error handling
    try {
        Tmc9660Handler handler(*spi, 0x00);
        
        // Test communication before full initialization
        if (!handler.TestCommunication()) {
            printf("ERROR: TMC9660 communication test failed\n");
            return;
        }
        
        // Initialize handler
        if (!handler.Initialize()) {
            printf("ERROR: TMC9660 handler initialization failed\n");
            return;
        }
        
        auto tmc = handler.GetTmc9660Driver();
        if (!tmc) {
            printf("ERROR: TMC9660 driver not available\n");
            return;
        }
        
        // Validate basic functionality
        uint32_t version = tmc->GetChipVersion();
        if (version == 0 || version == 0xFFFFFFFF) {
            printf("ERROR: Invalid chip version: 0x%08lX\n", version);
            return;
        }
        
        printf("TMC9660 initialized successfully (version: 0x%08lX)\n", version);
        
        // Configure with error checking
        if (!tmc->SetMaxCurrent(1000)) {
            printf("ERROR: Failed to set current limit\n");
            return;
        }
        
        // Test motor enable
        if (!tmc->EnableMotor(true)) {
            printf("ERROR: Failed to enable motor\n");
            return;
        }
        
        // Monitor for runtime errors
        printf("Running with error monitoring...\n");
        for (int i = 0; i < 100; i++) {
            auto status = tmc->GetDriverStatus();
            
            if (status.communication_error) {
                printf("ERROR: Communication error at iteration %d\n", i);
                break;
            }
            
            if (status.overtemperature_error) {
                printf("ERROR: Overtemperature error at iteration %d\n", i);
                tmc->EnableMotor(false);
                break;
            }
            
            if (status.overcurrent_detected) {
                printf("ERROR: Overcurrent detected at iteration %d\n", i);
                tmc->EnableMotor(false);
                break;
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        tmc->EnableMotor(false);
        printf("Operation completed successfully\n");
        
    } catch (const std::exception& e) {
        printf("EXCEPTION: %s\n", e.what());
    }
}
```

## 📚 See Also

- **[MotorController Documentation](../component-handlers/MOTOR_CONTROLLER_README.md)** - Motor controller manager
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[TMC9660 Datasheet](https://www.trinamic.com/products/integrated-circuits/details/tmc9660/)** - Official hardware documentation
- **[SPI Interface Guide](../hardware/SPI_INTERFACE_GUIDE.md)** - SPI communication setup
- **[UART Interface Guide](../hardware/UART_INTERFACE_GUIDE.md)** - UART communication setup

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*