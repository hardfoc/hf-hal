# Tmc9660Handler - Advanced Motor Controller Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Tmc9660Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)
![Interface](https://img.shields.io/badge/interface-SPI%20|%20UART-green.svg)
![Features](https://img.shields.io/badge/features-Motor%20|%20GPIO%20|%20ADC%20|%20Scripting-purple.svg)

**Comprehensive driver handler for TMC9660 motor controllers with advanced FOC control, GPIO expansion, and ADC integration**

</div>

## 📋 Overview

The `Tmc9660Handler` is a sophisticated driver handler that provides unified access to the TMC9660 motor controller's extensive capabilities. It integrates advanced motor control (DC, BLDC, Stepper), GPIO operations, ADC readings, and TMCL scripting in a single interface, supporting both SPI and UART communication protocols.

### ✨ Key Features

- **🎛️ Advanced Motor Control**: DC, BLDC/PMSM, and Stepper motor support with FOC
- **🔌 Dual Communication**: SPI (1MHz) and UART (up to 1Mbps) interfaces
- **📍 GPIO Expansion**: 18 configurable GPIO pins with interrupt support
- **📊 Advanced ADC System**: 4 current sense + 4 analog + voltage/temperature channels
- **🛡️ Comprehensive Protection**: I²t monitoring, thermal protection, fault detection
- **⚙️ TMCL Scripting**: Programmable motor control with debugging support
- **🏥 Advanced Telemetry**: Real-time status, diagnostics, and performance monitoring
- **🔧 Flexible Integration**: BaseGpio and BaseAdc compatible wrappers

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Tmc9660Handler                             │
├─────────────────────────────────────────────────────────────────┤
│  Communication Layer    │ SPI/UART interface abstraction      │
├─────────────────────────────────────────────────────────────────┤
│  TMC9660 Core Driver    │ Advanced motor control library      │
├─────────────────────────────────────────────────────────────────┤
│  GPIO Wrapper          │ BaseGpio compatible (18 pins)        │
├─────────────────────────────────────────────────────────────────┤
│  ADC Wrapper           │ BaseAdc compatible (12 channels)     │
├─────────────────────────────────────────────────────────────────┤
│  TMCL Scripting        │ Programmable control engine          │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### SPI-Based TMC9660 Initialization

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
        auto& driver = handler.driver();
        
        // Configure bootloader for parameter mode
        tmc9660::BootloaderConfig cfg{};
        cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        cfg.boot.start_motor_control = true;
        
        if (driver.bootloaderInit(&cfg) == TMC9660::BootloaderInitResult::Success) {
            // Configure motor
            driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC, 4);
            driver.motorConfig.setMaxTorqueCurrent(1000);  // 1A
            driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_ENCODER);
            
            printf("TMC9660 ready via SPI\n");
        }
    }
}
```

### UART-Based TMC9660 Initialization

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
        auto& driver = handler.driver();
        
        // Configure for UART operation
        tmc9660::BootloaderConfig cfg{};
        cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        cfg.uart.device_address = 0x01;
        cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
        
        if (driver.bootloaderInit(&cfg) == TMC9660::BootloaderInitResult::Success) {
            driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER);
            driver.motorConfig.setMaxTorqueCurrent(800);
            
            printf("TMC9660 ready via UART\n");
        }
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
    
    // Driver access
    TMC9660& driver() noexcept;
    Gpio& gpio(uint8_t gpioNumber);
    Adc& adc();
};
```

#### GPIO Wrapper (BaseGpio Compatible)
```cpp
class Gpio : public BaseGpio {
public:
    // Pin configuration
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;
    hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;
    hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;
    
    // GPIO information
    bool IsPinAvailable() const noexcept override;
    hf_u8_t GetMaxPins() const noexcept override;
    const char* GetDescription() const noexcept override;
};
```

#### ADC Wrapper (BaseAdc Compatible)
```cpp
class Adc : public BaseAdc {
public:
    // Channel operations
    bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;
    hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& voltage,
                             hf_u8_t samples = 1, hf_time_t interval = 0) noexcept override;
    hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& count,
                                 hf_u8_t samples = 1, hf_time_t interval = 0) noexcept override;
    
    // TMC9660-specific channel reading
    hf_adc_err_t ReadAinChannel(uint8_t ain_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadCurrentSenseChannel(uint8_t current_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadVoltageChannel(uint8_t voltage_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    hf_adc_err_t ReadTemperatureChannel(uint8_t temp_channel, hf_u32_t& raw_value, float& voltage) noexcept;
    
    // Statistics and diagnostics
    hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept override;
    hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept override;
};
```

## 🎯 Hardware Features

### TMC9660 Advanced Capabilities

| Feature | Specifications | Description |
|---------|----------------|-------------|
| **Motor Types** | DC, BLDC/PMSM, Stepper | Full motor type support with FOC |
| **Current Control** | 0-2000mA per phase | Precise current regulation with I²t protection |
| **Velocity Control** | 0-100,000 RPM | High-speed operation with closed-loop control |
| **Position Control** | 32-bit absolute/relative | High-precision positioning |
| **FOC Control** | Field-Oriented Control | Advanced motor control algorithms |
| **PWM Frequency** | 10-100 kHz | Configurable switching frequency |
| **GPIO Pins** | 18 digital I/O | Configurable with interrupt support |
| **ADC Channels** | 12 total channels | Current sense, analog, voltage, temperature |
| **Communication** | SPI (1MHz) / UART (1Mbps) | High-speed interfaces |
| **TMCL Scripting** | Programmable control | Custom motor control programs |

### Advanced ADC System

```cpp
// ADC Channel Types
enum class AdcChannelType {
    // Current Sense Channels (I0-I3)
    CURRENT_SENSE_I0 = 0x1000,
    CURRENT_SENSE_I1 = 0x1001,
    CURRENT_SENSE_I2 = 0x1002,
    CURRENT_SENSE_I3 = 0x1003,
    
    // External Analog Inputs (AIN0-AIN3)
    ANALOG_INPUT_AIN0 = 0x2000,
    ANALOG_INPUT_AIN1 = 0x2001,
    ANALOG_INPUT_AIN2 = 0x2002,
    ANALOG_INPUT_AIN3 = 0x2003,
    
    // Voltage Monitoring
    VOLTAGE_SUPPLY = 0x3000,
    VOLTAGE_DRIVER = 0x3001,
    
    // Temperature Sensing
    TEMPERATURE_CHIP = 0x4000,
    TEMPERATURE_EXTERNAL = 0x4001
};

// ADC Configuration
struct AdcConfig {
    tmc9660::tmcl::AdcShuntType shunt_type;      // Internal/External shunt
    tmc9660::tmcl::CsaGain gain_012;             // Gain for I0/I1/I2
    tmc9660::tmcl::CsaGain gain_3;               // Gain for I3
    tmc9660::tmcl::CsaFilter filter_012;         // Filter for I0/I1/I2
    tmc9660::tmcl::CsaFilter filter_3;           // Filter for I3
    uint16_t scaling_factor;                      // Current scaling (1-65535)
    uint16_t individual_scales[4];                // Per-channel scaling
    tmc9660::tmcl::AdcInversion inversion[4];    // Per-channel inversion
    int16_t offsets[4];                          // Per-channel offsets
};
```

### GPIO System

```cpp
// GPIO Configuration
struct GpioConfig {
    uint32_t output_mask;           // Output pin mask
    uint32_t direction_mask;        // Direction mask (1=output)
    uint32_t pull_up_mask;          // Pull-up enable mask
    uint32_t pull_down_mask;        // Pull-down enable mask
    uint32_t analog_mask;           // Analog input mask
};

// GPIO Pin Functions
enum class GpioFunction {
    DIGITAL_INPUT = 0,
    DIGITAL_OUTPUT = 1,
    ANALOG_INPUT = 2,
    PWM_OUTPUT = 3,
    INTERRUPT_INPUT = 4,
    UART_TX = 5,
    UART_RX = 6,
    SPI_SCK = 7,
    SPI_MOSI = 8,
    SPI_MISO = 9,
    I2C_SDA = 10,
    I2C_SCL = 11
};
```

## 🔧 Configuration

### Default Bootloader Configuration

```cpp
// Comprehensive default configuration
static const tmc9660::BootloaderConfig kDefaultBootConfig = {
    // LDO Configuration
    {
        tmc9660::bootcfg::LDOVoltage::V5_0,      // VEXT1: 5V output
        tmc9660::bootcfg::LDOVoltage::V3_3,      // VEXT2: 3.3V output
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // 3ms slope control
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // 3ms slope control
        false                                    // LDO short fault disabled
    },
    
    // Boot Configuration
    {
        tmc9660::bootcfg::BootMode::Parameter,   // Parameter mode for TMCL
        false,                                   // Bootloader ready fault disabled
        true,                                    // Bootloader exit fault enabled
        false,                                   // Self-test enabled
        false,                                   // Config fault disabled
        false                                    // Don't auto-start motor control
    },
    
    // UART Configuration
    {
        1,                                          // Device address
        255,                                        // Host address (broadcast)
        false,                                      // UART enabled
        tmc9660::bootcfg::UartRxPin::GPIO7,        // RX pin
        tmc9660::bootcfg::UartTxPin::GPIO6,        // TX pin
        tmc9660::bootcfg::BaudRate::Auto16x        // Auto baud detection
    },
    
    // SPI Configuration
    {
        false,                                      // SPI enabled
        tmc9660::bootcfg::SPIInterface::IFACE0,    // SPI interface 0
        tmc9660::bootcfg::SPI0SckPin::GPIO6        // SCK pin
    },
    
    // Clock Configuration
    {
        tmc9660::bootcfg::ClockSource::External,        // External clock
        tmc9660::bootcfg::ExtSourceType::Oscillator,    // Crystal oscillator
        tmc9660::bootcfg::XtalDrive::Freq16MHz,         // 16MHz crystal
        false,                                          // No boost
        tmc9660::bootcfg::SysClkSource::PLL,            // Use PLL
        14,                                             // R-divider
        tmc9660::bootcfg::SysClkDiv::Div1               // No system clock division
    }
};
```

### Motor Configuration Examples

```cpp
// BLDC Motor Configuration
void configure_bldc_motor(TMC9660& driver) {
    // Motor type and parameters
    driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC, 4);  // 4 pole pairs
    driver.motorConfig.setDirection(tmc9660::tmcl::MotorDirection::FORWARD);
    driver.motorConfig.setPWMFrequency(50000);  // 50kHz PWM
    driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_ENCODER);
    driver.motorConfig.setOutputVoltageLimit(8000);
    driver.motorConfig.setMaxTorqueCurrent(1200);  // 1.2A
    driver.motorConfig.setMaxFluxCurrent(800);     // 0.8A
    driver.motorConfig.setPWMSwitchingScheme(tmc9660::tmcl::PwmSwitchingScheme::SVPWM);
    driver.motorConfig.setIdleMotorPWMBehavior(tmc9660::tmcl::IdleMotorPwmBehavior::PWM_OFF_WHEN_MOTOR_IDLE);
}

// Stepper Motor Configuration
void configure_stepper_motor(TMC9660& driver) {
    // Motor type and parameters
    driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER, 1);
    driver.motorConfig.setDirection(tmc9660::tmcl::MotorDirection::FORWARD);
    driver.motorConfig.setPWMFrequency(35000);  // 35kHz PWM
    driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE);
    driver.motorConfig.setOutputVoltageLimit(6000);
    driver.motorConfig.setMaxTorqueCurrent(1000);  // 1A
    driver.motorConfig.setPWMSwitchingScheme(tmc9660::tmcl::PwmSwitchingScheme::STANDARD);
    driver.motorConfig.setIdleMotorPWMBehavior(tmc9660::tmcl::IdleMotorPwmBehavior::PWM_ON_WHEN_MOTOR_IDLE);
}
```

## 📊 Examples

### Advanced Motor Control with FOC

```cpp
void advanced_foc_control() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    // Initialize bootloader
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    driver.bootloaderInit(&cfg);
    
    // Configure BLDC motor with FOC
    configure_bldc_motor(driver);
    
    // Configure current sensing
    driver.currentSensing.setShuntType(tmc9660::tmcl::AdcShuntType::INTERNAL);
    driver.currentSensing.setCSAGain(tmc9660::tmcl::CsaGain::GAIN_20, tmc9660::tmcl::CsaGain::GAIN_20);
    driver.currentSensing.setScalingFactor(1000);
    driver.currentSensing.calibrateOffsets(true, 2000);
    
    // Configure protection
    driver.protection.configureI2t(100, 2.0f, 1000, 1.0f);  // Two-window I²t protection
    driver.protection.configureTemperature(80.0f, 100.0f);  // Temperature protection
    driver.protection.setOvercurrentEnabled(true);
    
    // Enable motor
    driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_ENCODER);
    
    printf("Advanced FOC control ready\n");
    
    // Velocity control example
    for (int rpm = 0; rpm <= 3000; rpm += 500) {
        driver.sendCommand(tmc9660::tmcl::Op::SAP, 3, 0, rpm);  // Set target velocity
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        uint32_t actual_rpm;
        driver.sendCommand(tmc9660::tmcl::Op::GAP, 3, 0, actual_rpm);  // Get actual velocity
        printf("Target: %d RPM, Actual: %lu RPM\n", rpm, actual_rpm);
    }
}
```

### GPIO Operations with Interrupts

```cpp
void gpio_advanced_operations() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    // Configure GPIO pins
    driver.gpio.setMode(0, true, true, true);   // Output with pull-up
    driver.gpio.setMode(1, true, true, true);   // Output with pull-up
    driver.gpio.setMode(2, false, true, true);  // Input with pull-up
    driver.gpio.setMode(3, false, true, true);  // Input with pull-up
    
    // Configure interrupts
    driver.globals.setInputTrigger(2, tmc9660::tmcl::TriggerTransition::RISING);
    driver.globals.setInputTrigger(3, tmc9660::tmcl::TriggerTransition::FALLING);
    
    printf("GPIO operations with interrupts:\n");
    
    for (int i = 0; i < 20; i++) {
        // Toggle outputs
        driver.gpio.writePin(0, i % 2);
        driver.gpio.writePin(1, (i + 1) % 2);
        
        // Read inputs
        bool input2, input3;
        driver.gpio.readDigital(2, input2);
        driver.gpio.readDigital(3, input3);
        
        // Read analog input (if configured)
        uint16_t analog_value;
        if (driver.gpio.readAnalog(5, analog_value)) {
            float voltage = (analog_value * 3.3f) / 65535.0f;
            printf("Outputs: %d %d, Inputs: %s %s, Analog: %.3fV\n",
                   i % 2, (i + 1) % 2,
                   input2 ? "HIGH" : "LOW",
                   input3 ? "HIGH" : "LOW",
                   voltage);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### Advanced ADC Monitoring

```cpp
void advanced_adc_monitoring() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    // Configure ADC system
    driver.currentSensing.setShuntType(tmc9660::tmcl::AdcShuntType::INTERNAL);
    driver.currentSensing.setCSAGain(tmc9660::tmcl::CsaGain::GAIN_20, tmc9660::tmcl::CsaGain::GAIN_20);
    driver.currentSensing.setCSAFilter(tmc9660::tmcl::CsaFilter::FILTER_1, tmc9660::tmcl::CsaFilter::FILTER_1);
    driver.currentSensing.setScalingFactor(1000);
    driver.currentSensing.calibrateOffsets(true, 1000);
    
    printf("Advanced ADC monitoring:\n");
    
    for (int i = 0; i < 100; i++) {
        // Read current sense channels
        int16_t current_raw[4];
        driver.currentSensing.readScaledAndOffset(current_raw[0], current_raw[1], 
                                                 current_raw[2], current_raw[3]);
        
        // Read analog inputs
        uint16_t analog_values[4];
        for (int j = 0; j < 4; j++) {
            driver.gpio.readAnalog(j, analog_values[j]);
        }
        
        // Read voltage and temperature
        float supply_voltage = driver.telemetry.getSupplyVoltage();
        float chip_temp = driver.telemetry.getChipTemperature();
        uint16_t ext_temp = driver.telemetry.getExternalTemperature();
        
        // Convert currents to mA
        float currents_ma[4];
        for (int j = 0; j < 4; j++) {
            currents_ma[j] = (current_raw[j] * 1000.0f) / driver.currentSensing.getScalingFactor();
        }
        
        printf("Currents: I0=%.1fmA I1=%.1fmA I2=%.1fmA I3=%.1fmA\n",
               currents_ma[0], currents_ma[1], currents_ma[2], currents_ma[3]);
        printf("Analog: AIN0=%u AIN1=%u AIN2=%u AIN3=%u\n",
               analog_values[0], analog_values[1], analog_values[2], analog_values[3]);
        printf("System: Vsupply=%.2fV, Tchip=%.1f°C, Text=%u\n",
               supply_voltage, chip_temp, ext_temp);
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

### TMCL Scripting Example

```cpp
void tmcl_scripting_example() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    // Create a simple TMCL script for motor control
    std::vector<uint32_t> script = {
        // Set motor to velocity mode
        0x05000001,  // SAP 1, 0, 1 (velocity mode)
        
        // Set target velocity to 1000 RPM
        0x05000003,  // SAP 3, 0, 1000 (target velocity)
        
        // Wait for velocity to reach target
        0x1B000000,  // WAIT 0, 0, 1000 (wait 1000ms)
        
        // Set velocity to 0
        0x05000003,  // SAP 3, 0, 0 (target velocity = 0)
        
        // Stop program
        0x1C000000   // STOP
    };
    
    // Upload script to TMC9660
    if (driver.script.upload(script)) {
        printf("TMCL script uploaded successfully\n");
        
        // Start script execution
        if (driver.script.start(0)) {
            printf("Script execution started\n");
            
            // Monitor script execution
            uint32_t status;
            while (driver.script.getStatus(status)) {
                if (status == 0) {  // Script completed
                    printf("Script execution completed\n");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}
```

### Comprehensive Fault Detection

```cpp
void comprehensive_fault_detection() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    // Configure comprehensive protection
    driver.protection.configureI2t(100, 2.0f, 1000, 1.0f);
    driver.protection.configureTemperature(80.0f, 100.0f);
    driver.protection.configureVoltage(600, 200);  // 60V over, 20V under
    driver.protection.setOvercurrentEnabled(true);
    
    // Configure fault handling
    driver.faultHandling.setDriveFaultBehaviour(tmc9660::tmcl::DriveFaultBehaviour::DISABLE_DRIVER);
    driver.faultHandling.setGdrvRetryBehaviour(tmc9660::tmcl::GdrvRetryBehaviour::RETRY_3_TIMES);
    
    printf("Comprehensive fault detection active\n");
    
    // Monitor for faults
    for (int i = 0; i < 1000; i++) {
        // Check general status flags
        uint32_t status_flags;
        if (driver.telemetry.getGeneralStatusFlags(status_flags)) {
            if (status_flags & 0x00000001) printf("WARNING: Config loaded\n");
            if (status_flags & 0x00000002) printf("WARNING: Motor enabled\n");
            if (status_flags & 0x00000004) printf("WARNING: Motor running\n");
        }
        
        // Check error flags
        uint32_t error_flags;
        if (driver.telemetry.getGeneralErrorFlags(error_flags)) {
            if (error_flags & 0x00000001) {
                printf("ERROR: Overtemperature detected!\n");
                driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
                break;
            }
            if (error_flags & 0x00000002) {
                printf("ERROR: Overcurrent detected!\n");
                driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
                break;
            }
            if (error_flags & 0x00000004) {
                printf("ERROR: Undervoltage detected!\n");
                driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
                break;
            }
        }
        
        // Check gate driver errors
        uint32_t gdrv_errors;
        if (driver.telemetry.getGateDriverErrorFlags(gdrv_errors)) {
            if (gdrv_errors != 0) {
                printf("ERROR: Gate driver fault detected: 0x%08lX\n", gdrv_errors);
                driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
                break;
            }
        }
        
        // Check ADC status
        uint32_t adc_status;
        if (driver.telemetry.getADCStatusFlags(adc_status)) {
            if (adc_status != 0) {
                printf("WARNING: ADC clipping detected: 0x%08lX\n", adc_status);
            }
        }
        
        // Print periodic status
        if (i % 100 == 0) {
            float supply_v = driver.telemetry.getSupplyVoltage();
            float chip_temp = driver.telemetry.getChipTemperature();
            int16_t motor_current = driver.telemetry.getMotorCurrent();
            
            printf("Status: V=%.2fV, T=%.1f°C, I=%dmA\n",
                   supply_v, chip_temp, motor_current);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Clear error flags
    driver.telemetry.clearGeneralErrorFlags(0xFFFFFFFF);
    driver.telemetry.clearGateDriverErrorFlags(0xFFFFFFFF);
    driver.telemetry.clearADCStatusFlags(0xFFFFFFFF);
}
```

### Performance Benchmarking

```cpp
void performance_benchmark() {
    // Setup handler
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    printf("TMC9660 Performance Benchmark\n");
    printf("=============================\n");
    
    // Test parameter read/write performance
    auto start_time = esp_timer_get_time();
    constexpr int NUM_OPERATIONS = 1000;
    
    for (int i = 0; i < NUM_OPERATIONS; i++) {
        uint32_t value;
        driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_POSITION, value);
    }
    
    auto end_time = esp_timer_get_time();
    float avg_time_us = (end_time - start_time) / (float)NUM_OPERATIONS;
    float ops_per_sec = 1000000.0f / avg_time_us;
    
    printf("Parameter read performance:\n");
    printf("  Average time: %.2f µs\n", avg_time_us);
    printf("  Operations per second: %.0f\n", ops_per_sec);
    
    // Test GPIO performance
    start_time = esp_timer_get_time();
    for (int i = 0; i < NUM_OPERATIONS; i++) {
        driver.gpio.writePin(0, i % 2);
    }
    end_time = esp_timer_get_time();
    avg_time_us = (end_time - start_time) / (float)NUM_OPERATIONS;
    ops_per_sec = 1000000.0f / avg_time_us;
    
    printf("GPIO write performance:\n");
    printf("  Average time: %.2f µs\n", avg_time_us);
    printf("  Operations per second: %.0f\n", ops_per_sec);
    
    // Test ADC performance
    start_time = esp_timer_get_time();
    for (int i = 0; i < NUM_OPERATIONS; i++) {
        int16_t adc_values[4];
        driver.currentSensing.readScaledAndOffset(adc_values[0], adc_values[1], 
                                                 adc_values[2], adc_values[3]);
    }
    end_time = esp_timer_get_time();
    avg_time_us = (end_time - start_time) / (float)NUM_OPERATIONS;
    ops_per_sec = 1000000.0f / avg_time_us;
    
    printf("ADC read performance:\n");
    printf("  Average time: %.2f µs\n", avg_time_us);
    printf("  Operations per second: %.0f\n", ops_per_sec);
    
    // Test telemetry performance
    start_time = esp_timer_get_time();
    for (int i = 0; i < NUM_OPERATIONS; i++) {
        float supply_v = driver.telemetry.getSupplyVoltage();
        float chip_temp = driver.telemetry.getChipTemperature();
        int16_t motor_current = driver.telemetry.getMotorCurrent();
    }
    end_time = esp_timer_get_time();
    avg_time_us = (end_time - start_time) / (float)NUM_OPERATIONS;
    ops_per_sec = 1000000.0f / avg_time_us;
    
    printf("Telemetry read performance:\n");
    printf("  Average time: %.2f µs\n", avg_time_us);
    printf("  Operations per second: %.0f\n", ops_per_sec);
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
        
        // Add retry logic with exponential backoff
        constexpr int MAX_RETRIES = 5;
        for (int retry = 0; retry < MAX_RETRIES; retry++) {
            if (spi_.Transfer(tx.data(), rx.data(), 8)) {
                // Verify checksum
                uint8_t calculated_checksum = 0;
                for (int i = 0; i < 7; i++) {
                    calculated_checksum += rx[i];
                }
                
                if (calculated_checksum == rx[7]) {
                    return true;
                }
            }
            
            // Exponential backoff
            vTaskDelay(pdMS_TO_TICKS(1 << retry));
        }
        
        return false;
    }
    
private:
    BaseSpi& spi_;
};
```

### Advanced Motor Profiling

```cpp
void advanced_motor_profiling() {
    Tmc9660Handler handler(*spi, 0x00);
    handler.Initialize();
    auto& driver = handler.driver();
    
    printf("Advanced Motor Profiling\n");
    printf("========================\n");
    
    // Test different motor configurations
    struct MotorTest {
        tmc9660::tmcl::MotorType type;
        uint8_t pole_pairs;
        uint16_t current_ma;
        uint32_t pwm_freq_hz;
        const char* description;
    };
    
    std::vector<MotorTest> tests = {
        {tmc9660::tmcl::MotorType::BLDC, 4, 1000, 50000, "BLDC 4-pole 1A 50kHz"},
        {tmc9660::tmcl::MotorType::BLDC, 4, 1500, 70000, "BLDC 4-pole 1.5A 70kHz"},
        {tmc9660::tmcl::MotorType::STEPPER, 1, 800, 35000, "Stepper 1A 35kHz"},
        {tmc9660::tmcl::MotorType::STEPPER, 1, 1200, 50000, "Stepper 1.2A 50kHz"}
    };
    
    for (const auto& test : tests) {
        printf("\nTesting: %s\n", test.description);
        
        // Configure motor
        driver.motorConfig.setType(test.type, test.pole_pairs);
        driver.motorConfig.setMaxTorqueCurrent(test.current_ma);
        driver.motorConfig.setPWMFrequency(test.pwm_freq_hz);
        
        // Enable motor
        driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE);
        
        // Measure acceleration performance
        auto start_time = esp_timer_get_time();
        driver.sendCommand(tmc9660::tmcl::Op::SAP, 3, 0, 1000);  // Set target velocity
        
        // Wait for velocity to reach target
        uint32_t actual_velocity;
        int timeout_count = 0;
        while (timeout_count < 1000) {
            driver.sendCommand(tmc9660::tmcl::Op::GAP, 3, 0, actual_velocity);
            if (actual_velocity >= 950) break;  // Within 5% of target
            vTaskDelay(pdMS_TO_TICKS(1));
            timeout_count++;
        }
        
        auto end_time = esp_timer_get_time();
        float accel_time_ms = (end_time - start_time) / 1000.0f;
        
        printf("  Acceleration time to 1000 RPM: %.1f ms\n", accel_time_ms);
        printf("  Final velocity: %lu RPM\n", actual_velocity);
        
        // Measure current consumption
        int16_t motor_current = driver.telemetry.getMotorCurrent();
        printf("  Current consumption: %d mA\n", motor_current);
        
        // Stop motor
        driver.sendCommand(tmc9660::tmcl::Op::SAP, 3, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
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
    
    auto& driver = handler.driver();
    
    // Initialize bootloader with error checking
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    auto boot_result = driver.bootloaderInit(&cfg);
    if (boot_result != TMC9660::BootloaderInitResult::Success) {
        printf("ERROR: Bootloader initialization failed: %d\n", static_cast<int>(boot_result));
        return;
    }
    
    // Validate basic functionality
    uint32_t chip_id;
    if (!driver.sendCommand(tmc9660::tmcl::Op::GetInfo, 0, 0, chip_id)) {
        printf("ERROR: Failed to read chip ID\n");
        return;
    }
    
    printf("TMC9660 initialized successfully (ID: 0x%08lX)\n", chip_id);
    
    // Configure with error checking
    if (!driver.motorConfig.setMaxTorqueCurrent(1000)) {
        printf("ERROR: Failed to set current limit\n");
        return;
    }
    
    // Test motor enable
    if (!driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE)) {
        printf("ERROR: Failed to enable motor\n");
        return;
    }
    
    // Monitor for runtime errors
    printf("Running with comprehensive error monitoring...\n");
    for (int i = 0; i < 100; i++) {
        // Check communication status
        if (!handler.TestCommunication()) {
            printf("ERROR: Communication lost at iteration %d\n", i);
            break;
        }
        
        // Check for hardware faults
        uint32_t error_flags;
        if (driver.telemetry.getGeneralErrorFlags(error_flags)) {
            if (error_flags != 0) {
                printf("ERROR: Hardware fault detected: 0x%08lX at iteration %d\n", error_flags, i);
                driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
                break;
            }
        }
        
        // Check temperature
        float temp = driver.telemetry.getChipTemperature();
        if (temp > 90.0f) {
            printf("ERROR: High temperature detected: %.1f°C at iteration %d\n", temp, i);
            driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
    printf("Operation completed successfully\n");
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