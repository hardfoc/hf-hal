# HardFOC Vortex V1 Base Interface Reference

## 📋 Overview

This reference guide provides detailed information about the BaseGpio and BaseAdc interfaces that are used throughout the HardFOC Vortex V1 system. Understanding these interfaces is crucial for optimal cached access performance, as they represent the actual hardware abstraction layer functions available when you cache component pointers.

## 🔧 BaseGpio Interface Reference

The BaseGpio interface provides standardized access to GPIO functionality across different hardware sources (ESP32-C6, PCAL95555, TMC9660). This interface is implemented by:

- **PCAL95555Handler::Pcal95555GpioPin** - For PCAL95555 GPIO expander pins
- **Tmc9660Handler::Gpio** - For TMC9660 motor controller GPIO pins
- **ESP32 GPIO drivers** - For native ESP32-C6 GPIO pins

### Core Interface Methods

#### Initialization and Information
```cpp
// Lifecycle management
bool Initialize() noexcept;
bool Deinitialize() noexcept;

// Hardware information
bool IsPinAvailable() const noexcept;
hf_u8_t GetMaxPins() const noexcept;
const char* GetDescription() const noexcept;

// Feature support
bool SupportsInterrupts() const noexcept;
```

#### Pin State Control (Core Performance Functions)
```cpp
// Direct pin level control (lowest level - fastest)
hf_gpio_err_t SetPinLevel(hf_gpio_level_t level) noexcept;      // ~10-50ns
hf_gpio_err_t GetPinLevel(hf_gpio_level_t& level) noexcept;     // ~15-60ns

// High-level convenience functions (built on SetPinLevel/GetPinLevel)
hf_gpio_err_t SetActive() noexcept;                             // ~15-70ns
hf_gpio_err_t SetInactive() noexcept;                           // ~15-70ns
hf_gpio_err_t Toggle() noexcept;                                // ~30-120ns
hf_gpio_err_t Read(bool& state) noexcept;                       // ~20-80ns
hf_gpio_err_t IsActive(bool& active) noexcept;                  // ~20-80ns
```

#### Pin Configuration
```cpp
// Direction control
hf_gpio_err_t SetDirection(hf_gpio_direction_t direction) noexcept;
hf_gpio_err_t GetDirection(hf_gpio_direction_t& direction) const noexcept;

// Pull resistor control
hf_gpio_err_t SetPullMode(hf_gpio_pull_mode_t mode) noexcept;
hf_gpio_pull_mode_t GetPullMode() const noexcept;

// Output mode control
hf_gpio_err_t SetOutputMode(hf_gpio_output_mode_t mode) noexcept;
hf_gpio_err_t GetOutputMode(hf_gpio_output_mode_t& mode) const noexcept;
```

#### Interrupt Support
```cpp
// Interrupt configuration
hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                InterruptCallback callback = nullptr,
                                void* user_data = nullptr) noexcept;

hf_gpio_err_t EnableInterrupt() noexcept;
hf_gpio_err_t DisableInterrupt() noexcept;
```

#### Statistics and Diagnostics
```cpp
// Performance monitoring
hf_gpio_err_t GetStatistics(PinStatistics& statistics) noexcept;
hf_gpio_err_t ResetStatistics() noexcept;
```

### Hardware-Specific Extensions

#### PCAL95555 Advanced Features
The PCAL95555 GPIO implementation provides additional features beyond the base interface:

```cpp
// Polarity inversion control
hf_gpio_err_t SetPolarityInversion(hf_bool_t invert) noexcept;
hf_gpio_err_t GetPolarityInversion(hf_bool_t& invert) noexcept;

// Interrupt masking
hf_gpio_err_t SetInterruptMask(hf_bool_t mask) noexcept;
hf_gpio_err_t GetInterruptMask(hf_bool_t& mask) noexcept;
hf_gpio_err_t GetInterruptStatus(hf_bool_t& status) noexcept;
```

### BaseGpio Performance Characteristics

| Function | PCAL95555 | TMC9660 | ESP32 | Use Case |
|----------|-----------|---------|-------|----------|
| **SetPinLevel()** | ~30-80ns | ~20-60ns | ~10-40ns | Real-time control |
| **GetPinLevel()** | ~40-100ns | ~25-70ns | ~15-50ns | State monitoring |
| **SetActive()** | ~35-90ns | ~25-70ns | ~15-50ns | Convenience |
| **Toggle()** | ~70-180ns | ~50-140ns | ~30-100ns | Pulse generation |
| **SetDirection()** | ~100-250ns | ~80-200ns | ~50-150ns | Configuration |

## 📊 BaseAdc Interface Reference

The BaseAdc interface provides standardized access to ADC functionality. This interface is implemented by:

- **Tmc9660Handler::Adc** - For TMC9660 motor controller ADC channels
- **ESP32 ADC drivers** - For native ESP32-C6 ADC channels

### Core Interface Methods

#### Initialization and Information
```cpp
// Lifecycle management
bool Initialize() noexcept;
bool Deinitialize() noexcept;

// Hardware information
hf_u8_t GetMaxChannels() const noexcept;
bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept;
```

#### ADC Reading Functions (Core Performance Functions)
```cpp
// Voltage reading (most common - optimized)
hf_adc_err_t ReadVoltage(float& voltage,                        // ~20-100ns
                        hf_u8_t numOfSamplesToAvg = 1,
                        hf_time_t timeBetweenSamples = 0) noexcept;

// Raw count reading (fastest - no conversion)
hf_adc_err_t ReadCount(hf_u32_t& count,                         // ~15-80ns
                      hf_u8_t numOfSamplesToAvg = 1,
                      hf_time_t timeBetweenSamples = 0) noexcept;

// Combined reading (voltage + raw in one call)
hf_adc_err_t ReadChannel(hf_u32_t& raw_value, float& voltage,   // ~25-120ns
                        hf_u8_t numOfSamplesToAvg = 1,
                        hf_time_t timeBetweenSamples = 0) noexcept;

// Multi-channel reading (batch operation)
hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids,
                                 hf_u8_t num_channels,
                                 hf_u32_t* raw_values,
                                 float* voltages) noexcept;
```

#### Statistics and Diagnostics
```cpp
// Performance monitoring
hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept;
hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept;
hf_adc_err_t ResetStatistics() noexcept;
hf_adc_err_t ResetDiagnostics() noexcept;
```

### TMC9660 ADC Specialized Functions

The TMC9660 ADC implementation provides hardware-specific channel reading methods:

```cpp
// External analog input channels (AIN0-AIN3)
hf_adc_err_t ReadAinChannel(uint8_t ain_channel,                // ~20-100ns
                           hf_u32_t& raw_value, float& voltage) noexcept;

// Current sensing channels (I0-I3)
hf_adc_err_t ReadCurrentSenseChannel(uint8_t current_channel,   // ~25-120ns
                                    hf_u32_t& raw_value, float& voltage) noexcept;

// Voltage monitoring channels (supply, driver)
hf_adc_err_t ReadVoltageChannel(uint8_t voltage_channel,        // ~30-140ns
                               hf_u32_t& raw_value, float& voltage) noexcept;

// Temperature channels (chip, external)
hf_adc_err_t ReadTemperatureChannel(uint8_t temp_channel,       // ~35-160ns
                                   hf_u32_t& raw_value, float& voltage) noexcept;

// Motor data channels (current, velocity, position)
hf_adc_err_t ReadMotorDataChannel(uint8_t motor_channel,        // ~40-180ns
                                 hf_u32_t& raw_value, float& voltage) noexcept;
```

### BaseAdc Performance Characteristics

| Function | TMC9660 | ESP32 | Use Case |
|----------|---------|-------|----------|
| **ReadVoltage()** | ~20-100ns | ~30-120ns | Real-time feedback |
| **ReadCount()** | ~15-80ns | ~25-100ns | Fastest reading |
| **ReadChannel()** | ~25-120ns | ~35-140ns | Combined data |
| **ReadAinChannel()** | ~20-100ns | N/A | External inputs |
| **ReadCurrentSenseChannel()** | ~25-120ns | N/A | Motor current |
| **ReadMotorDataChannel()** | ~40-180ns | N/A | Motor feedback |

## 🚀 Cached Access Implementation Patterns

### High-Performance GPIO Control
```cpp
class HighPerformanceGpioControl {
private:
    std::shared_ptr<BaseGpio> enable_pin_;
    std::shared_ptr<BaseGpio> direction_pin_;
    std::shared_ptr<BaseGpio> step_pin_;
    
public:
    bool Initialize() {
        // Cache GPIO pointers once
        enable_pin_ = vortex.gpio.Get("MOTOR_ENABLE");
        direction_pin_ = vortex.gpio.Get("MOTOR_DIRECTION");
        step_pin_ = vortex.gpio.Get("MOTOR_STEP");
        
        return enable_pin_ && direction_pin_ && step_pin_;
    }
    
    void HighFrequencyControl() {
        // Ultra-fast pin operations using BaseGpio interface
        while (running_) {
            // Direct BaseGpio calls - maximum performance
            enable_pin_->SetActive();                               // ~15-50ns
            direction_pin_->SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);  // ~10-40ns
            
            // Generate step pulse
            step_pin_->SetActive();                                 // ~15-50ns
            DelayNanoseconds(1000);  // 1µs pulse
            step_pin_->SetInactive();                               // ~15-50ns
            
            DelayMicroseconds(100);  // 10kHz step rate
        }
    }
};
```

### High-Performance ADC Sampling
```cpp
class HighPerformanceAdcSampling {
private:
    BaseAdc* current_sensor_;
    BaseAdc* velocity_sensor_;
    BaseAdc* position_sensor_;
    
public:
    bool Initialize() {
        // Cache ADC pointers once
        current_sensor_ = vortex.adc.Get("TMC9660_CURRENT_I0");
        velocity_sensor_ = vortex.adc.Get("TMC9660_MOTOR_VELOCITY");
        position_sensor_ = vortex.adc.Get("TMC9660_MOTOR_POSITION");
        
        return current_sensor_ && velocity_sensor_ && position_sensor_;
    }
    
    void HighFrequencySampling() {
        // Ultra-fast ADC sampling using BaseAdc interface
        std::array<float, 3> readings;
        
        while (sampling_) {
            // Direct BaseAdc calls - maximum performance
            current_sensor_->ReadVoltage(readings[0]);      // ~20-100ns
            velocity_sensor_->ReadVoltage(readings[1]);     // ~20-100ns
            position_sensor_->ReadVoltage(readings[2]);     // ~20-100ns
            
            // Process readings immediately
            ProcessControlLoop(readings);
            
            DelayMicroseconds(100);  // 10kHz sampling rate
        }
    }
};
```

## 📚 Interface Design Philosophy

### BaseGpio Design
- **Hardware Agnostic**: Same interface across ESP32, PCAL95555, and TMC9660
- **Performance Optimized**: Direct hardware access through implementation-specific methods
- **Feature Rich**: Supports interrupts, configuration, and diagnostics
- **Thread Safe**: All operations are thread-safe with internal locking

### BaseAdc Design
- **Unified Interface**: Common ADC operations across different hardware
- **TMC9660 Optimized**: Specialized functions for motor control applications
- **Batch Capable**: Multi-channel reading for efficiency
- **Measurement Focused**: Built-in statistics and diagnostics

### Performance Optimization Strategy
1. **Cache Interface Pointers**: Store BaseGpio*/BaseAdc* for direct access
2. **Use Core Functions**: Prefer SetPinLevel/GetPinLevel and ReadVoltage/ReadCount
3. **Validate Once**: Check cached pointers during initialization, not in loops
4. **Batch When Possible**: Use multi-channel/multi-pin operations
5. **Monitor Performance**: Use built-in statistics for optimization

This base interface reference enables developers to make informed decisions about when and how to use cached access for maximum performance while maintaining the hardware abstraction benefits of the unified API design.