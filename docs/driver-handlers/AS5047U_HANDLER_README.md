# As5047uHandler - Position Encoder Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-As5047uHandler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-AS5047U-orange.svg)
![Interface](https://img.shields.io/badge/interface-SPI-green.svg)

**High-precision magnetic position encoder driver handler with advanced features**

</div>

## 📋 Overview

The `As5047uHandler` is a specialized driver handler for the AS5047U magnetic position encoder. It provides high-precision 14-bit angular position sensing with advanced features like automatic gain control, magnetic field diagnostics, and programmable zero position.

### ✨ Key Features

- **📐 High Precision**: 14-bit resolution (0.022° accuracy)
- **🔄 360° Sensing**: Absolute position measurement
- **⚡ High Speed**: Up to 28,000 RPM operation
- **📡 SPI Interface**: Fast digital communication
- **🛡️ Built-in Diagnostics**: Magnetic field strength monitoring
- **⚙️ Programmable Features**: Zero position, rotation direction
- **🔍 Advanced Filtering**: Hysteresis and filtering options
- **🏥 Health Monitoring**: Error detection and status reporting

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    As5047uHandler                              │
├─────────────────────────────────────────────────────────────────┤
│  Position Interface │ Angle, velocity, and acceleration       │
├─────────────────────────────────────────────────────────────────┤
│  SPI Communication  │ 16-bit register access with CRC         │
├─────────────────────────────────────────────────────────────────┤
│  Diagnostic System  │ Magnetic field and error monitoring     │
├─────────────────────────────────────────────────────────────────┤
│  AS5047U Driver     │ Low-level register and calibration      │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Position Reading

```cpp
#include "utils-and-drivers/driver-handlers/As5047uHandler.h"
#include "component-handlers/CommChannelsManager.h"

void as5047u_basic_example() {
    // Get SPI interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (!spi) {
        printf("SPI interface not available\n");
        return;
    }
    
    // Create AS5047U handler
    As5047uHandler encoder(*spi);
    
    // Initialize encoder
    if (!encoder.Initialize()) {
        printf("Failed to initialize AS5047U\n");
        return;
    }
    
    printf("AS5047U Position Encoder ready\n");
    
    // Read position continuously
    for (int i = 0; i < 100; i++) {
        float angle_deg = encoder.GetAngleDegrees();
        uint16_t raw_count = encoder.GetRawPosition();
        
        printf("Position: %.2f° (raw: %u)\n", angle_deg, raw_count);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class As5047uHandler {
public:
    // Constructor
    As5047uHandler(BaseSpi& spi_interface);
    
    // Initialization
    bool Initialize() noexcept;
    bool IsInitialized() const noexcept;
    void Deinitialize() noexcept;
    
    // Communication testing
    bool TestCommunication() noexcept;
};
```

#### Position Reading
```cpp
// Raw position data
uint16_t GetRawPosition() noexcept;
uint16_t GetRawPositionWithStatus() noexcept;

// Angle conversion
float GetAngleDegrees() noexcept;
float GetAngleRadians() noexcept;

// Velocity calculation (requires multiple samples)
float GetVelocityRpm() noexcept;
float GetVelocityRadPerSec() noexcept;

// Multi-turn support
int32_t GetAbsolutePosition() noexcept;  // Accumulated position
void ResetPosition() noexcept;
```

#### Configuration and Calibration
```cpp
// Zero position setting
bool SetZeroPosition() noexcept;
bool SetZeroPosition(uint16_t zero_count) noexcept;
uint16_t GetZeroPosition() const noexcept;

// Rotation direction
bool SetRotationDirection(RotationDirection direction) noexcept;
RotationDirection GetRotationDirection() const noexcept;

// Hysteresis and filtering
bool SetHysteresis(HysteresisLevel level) noexcept;
bool SetFastFilterThreshold(uint8_t threshold) noexcept;
bool SetSlowFilterThreshold(uint8_t threshold) noexcept;
```

#### Diagnostics and Status
```cpp
// Error flags
struct ErrorFlags {
    bool parity_error;
    bool invalid_command;
    bool framing_error;
    bool voltage_error;
    bool magnetic_field_too_strong;
    bool magnetic_field_too_weak;
    bool cordic_overflow;
    bool offset_compensation_ready;
};

ErrorFlags GetErrorFlags() noexcept;
bool HasErrors() noexcept;

// Magnetic field diagnostics
uint8_t GetMagneticFieldStrength() noexcept;
bool IsMagneticFieldOk() noexcept;

// Automatic gain control
uint8_t GetAutomaticGainControl() noexcept;
```

### Configuration Enums

```cpp
// Rotation direction
enum class RotationDirection {
    CLOCKWISE = 0,
    COUNTER_CLOCKWISE = 1
};

// Hysteresis levels
enum class HysteresisLevel {
    OFF = 0,
    LSB_1 = 1,
    LSB_2 = 2,
    LSB_3 = 3
};

// Power modes
enum class PowerMode {
    NORMAL = 0,
    LOW_POWER_1 = 1,
    LOW_POWER_2 = 2,
    LOW_POWER_3 = 3
};
```

## 🎯 Hardware Features

### AS5047U Capabilities

| Feature | Specification | Description |
|---------|---------------|-------------|
| **Resolution** | 14-bit (16384 counts) | 0.022° per LSB |
| **Accuracy** | ±0.05° (typ) | High precision measurement |
| **Speed** | Up to 28,000 RPM | High-speed operation |
| **Supply Voltage** | 4.5V - 5.5V | Single supply operation |
| **SPI Interface** | Up to 10 MHz | Fast digital communication |
| **Update Rate** | Up to 28 kSPS | High-frequency sampling |
| **Temperature Range** | -40°C to +150°C | Industrial temperature range |
| **Package** | TSSOP14 | Compact surface mount |

### Register Map Overview

```cpp
// AS5047U Register Addresses
enum class As5047uRegister : uint16_t {
    // Volatile registers
    NOP = 0x0000,
    ERRFL = 0x0001,      // Error flags
    PROG = 0x0003,       // Programming register
    DIAAGC = 0x3FFC,     // Diagnostics and AGC
    MAG = 0x3FFD,        // Magnetic field strength
    ANGLEUNC = 0x3FFE,   // Angle without compensation
    ANGLECOM = 0x3FFF,   // Angle with compensation
    
    // Non-volatile registers  
    ZPOSM = 0x0016,      // Zero position MSB
    ZPOSL = 0x0017,      // Zero position LSB
    SETTINGS1 = 0x0018,  // Settings register 1
    SETTINGS2 = 0x0019,  // Settings register 2
};
```

## 🔧 Configuration

### SPI Communication Setup

```cpp
// AS5047U SPI configuration
struct As5047uSpiConfig {
    uint32_t max_clock_hz = 10000000;   // 10 MHz max
    uint8_t mode = 1;                   // SPI Mode 1 (CPOL=0, CPHA=1)
    uint8_t bits_per_word = 16;         // 16-bit transfers
    bool use_cs_control = true;         // Manual CS control
    uint32_t cs_delay_us = 1;           // CS setup/hold time
};
```

### Initialization Configuration

```cpp
// Initialization options
struct As5047uConfig {
    RotationDirection rotation_dir = RotationDirection::CLOCKWISE;
    HysteresisLevel hysteresis = HysteresisLevel::LSB_1;
    uint8_t fast_filter_threshold = 3;
    uint8_t slow_filter_threshold = 16;
    PowerMode power_mode = PowerMode::NORMAL;
    bool enable_uvw_abi = false;        // UVW/ABI output pins
    bool enable_pwm_output = false;     // PWM output
    uint16_t zero_position = 0;         // Custom zero position
};
```

## 📊 Examples

### Basic Position Monitoring

```cpp
#include "utils-and-drivers/driver-handlers/As5047uHandler.h"

void position_monitoring_example() {
    // Setup SPI communication
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    auto* spi = comm.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    
    // Create encoder handler
    As5047uHandler encoder(*spi);
    
    if (!encoder.Initialize()) {
        printf("AS5047U initialization failed\n");
        return;
    }
    
    // Check for magnetic field
    if (!encoder.IsMagneticFieldOk()) {
        printf("WARNING: Magnetic field not optimal\n");
        printf("Field strength: %u\n", encoder.GetMagneticFieldStrength());
    }
    
    printf("AS5047U position monitoring started\n");
    
    // Continuous position monitoring
    for (int i = 0; i < 1000; i++) {
        // Read position data
        float angle = encoder.GetAngleDegrees();
        uint16_t raw = encoder.GetRawPosition();
        
        // Check for errors
        if (encoder.HasErrors()) {
            auto errors = encoder.GetErrorFlags();
            printf("ERROR: ");
            if (errors.magnetic_field_too_weak) printf("MagWeak ");
            if (errors.magnetic_field_too_strong) printf("MagStrong ");
            if (errors.parity_error) printf("Parity ");
            printf("\n");
        }
        
        // Display position
        printf("Angle: %7.2f° (raw: %5u)\n", angle, raw);
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### Velocity Measurement

```cpp
void velocity_measurement_example() {
    As5047uHandler encoder(*spi);
    encoder.Initialize();
    
    printf("AS5047U velocity measurement demo\n");
    
    // Variables for velocity calculation
    float previous_angle = encoder.GetAngleDegrees();
    uint32_t previous_time = esp_timer_get_time();
    
    for (int i = 0; i < 500; i++) {
        // Get current position and time
        float current_angle = encoder.GetAngleDegrees();
        uint32_t current_time = esp_timer_get_time();
        
        // Calculate time difference
        float dt_sec = (current_time - previous_time) / 1000000.0f;
        
        // Calculate angular difference (handle wrap-around)
        float angle_diff = current_angle - previous_angle;
        if (angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } else if (angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
        
        // Calculate velocity
        float velocity_deg_per_sec = angle_diff / dt_sec;
        float velocity_rpm = velocity_deg_per_sec * 60.0f / 360.0f;
        
        // Display results
        printf("Position: %7.2f°, Velocity: %8.1f°/s (%6.1f RPM)\n",
               current_angle, velocity_deg_per_sec, velocity_rpm);
        
        // Update previous values
        previous_angle = current_angle;
        previous_time = current_time;
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz sampling
    }
}
```

### Multi-Turn Position Tracking

```cpp
void multi_turn_tracking_example() {
    As5047uHandler encoder(*spi);
    encoder.Initialize();
    
    printf("AS5047U multi-turn position tracking\n");
    
    // Reset absolute position counter
    encoder.ResetPosition();
    
    int32_t total_rotations = 0;
    float previous_angle = encoder.GetAngleDegrees();
    
    printf("Rotate the encoder to test multi-turn tracking...\n");
    
    for (int i = 0; i < 2000; i++) {
        float current_angle = encoder.GetAngleDegrees();
        
        // Detect full rotations
        float angle_diff = current_angle - previous_angle;
        if (angle_diff > 180.0f) {
            // Crossed 0° going backwards
            total_rotations--;
            printf("Full rotation backwards (total: %ld)\n", total_rotations);
        } else if (angle_diff < -180.0f) {
            // Crossed 0° going forwards
            total_rotations++;
            printf("Full rotation forwards (total: %ld)\n", total_rotations);
        }
        
        // Calculate absolute position
        float absolute_angle = (total_rotations * 360.0f) + current_angle;
        int32_t absolute_position = encoder.GetAbsolutePosition();
        
        printf("Current: %6.1f°, Absolute: %8.1f°, Turns: %ld\n",
               current_angle, absolute_angle, total_rotations);
        
        previous_angle = current_angle;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Calibration and Zero Setting

```cpp
void calibration_example() {
    As5047uHandler encoder(*spi);
    encoder.Initialize();
    
    printf("AS5047U calibration and zero setting\n");
    
    // Read current position
    float current_angle = encoder.GetAngleDegrees();
    uint16_t current_raw = encoder.GetRawPosition();
    
    printf("Current position: %.2f° (raw: %u)\n", current_angle, current_raw);
    
    // Set current position as zero
    printf("Setting current position as zero reference...\n");
    if (encoder.SetZeroPosition()) {
        printf("Zero position set successfully\n");
    } else {
        printf("Failed to set zero position\n");
        return;
    }
    
    // Verify zero setting
    vTaskDelay(pdMS_TO_TICKS(100));
    float new_angle = encoder.GetAngleDegrees();
    printf("New angle after zero set: %.2f°\n", new_angle);
    
    // Test rotation direction setting
    printf("\nTesting rotation direction...\n");
    
    // Set clockwise direction
    encoder.SetRotationDirection(RotationDirection::CLOCKWISE);
    printf("Direction set to CLOCKWISE\n");
    
    for (int i = 0; i < 10; i++) {
        printf("CW - Position: %.2f°\n", encoder.GetAngleDegrees());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Set counter-clockwise direction
    encoder.SetRotationDirection(RotationDirection::COUNTER_CLOCKWISE);
    printf("Direction set to COUNTER_CLOCKWISE\n");
    
    for (int i = 0; i < 10; i++) {
        printf("CCW - Position: %.2f°\n", encoder.GetAngleDegrees());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Restore clockwise direction
    encoder.SetRotationDirection(RotationDirection::CLOCKWISE);
    printf("Direction restored to CLOCKWISE\n");
}
```

### Diagnostic Monitoring

```cpp
void diagnostic_monitoring_example() {
    As5047uHandler encoder(*spi);
    encoder.Initialize();
    
    printf("AS5047U diagnostic monitoring\n");
    printf("==============================\n");
    
    for (int i = 0; i < 200; i++) {
        // Read position and diagnostics
        float angle = encoder.GetAngleDegrees();
        uint8_t mag_field = encoder.GetMagneticFieldStrength();
        uint8_t agc = encoder.GetAutomaticGainControl();
        
        // Check error flags
        auto errors = encoder.GetErrorFlags();
        
        // Display diagnostic information
        printf("Pos: %6.1f°, MagField: %3u, AGC: %3u", angle, mag_field, agc);
        
        // Display error status
        if (encoder.HasErrors()) {
            printf(" [ERRORS:");
            if (errors.magnetic_field_too_weak) printf(" MagWeak");
            if (errors.magnetic_field_too_strong) printf(" MagStrong");
            if (errors.parity_error) printf(" Parity");
            if (errors.invalid_command) printf(" InvCmd");
            if (errors.framing_error) printf(" Frame");
            if (errors.voltage_error) printf(" Voltage");
            if (errors.cordic_overflow) printf(" CORDIC");
            printf("]");
        } else {
            printf(" [OK]");
        }
        
        // Magnetic field status
        if (mag_field < 50) {
            printf(" [MAG: WEAK]");
        } else if (mag_field > 200) {
            printf(" [MAG: STRONG]");
        } else {
            printf(" [MAG: OK]");
        }
        
        printf("\n");
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Advanced Filtering Configuration

```cpp
void filtering_configuration_example() {
    As5047uHandler encoder(*spi);
    encoder.Initialize();
    
    printf("AS5047U filtering configuration test\n");
    
    // Test different hysteresis levels
    HysteresisLevel hysteresis_levels[] = {
        HysteresisLevel::OFF,
        HysteresisLevel::LSB_1,
        HysteresisLevel::LSB_2,
        HysteresisLevel::LSB_3
    };
    
    const char* hysteresis_names[] = {
        "OFF", "1 LSB", "2 LSB", "3 LSB"
    };
    
    for (int h = 0; h < 4; h++) {
        printf("\nTesting hysteresis level: %s\n", hysteresis_names[h]);
        encoder.SetHysteresis(hysteresis_levels[h]);
        
        // Collect stability data
        float sum = 0.0f;
        float min_val = 360.0f;
        float max_val = 0.0f;
        constexpr int samples = 100;
        
        for (int i = 0; i < samples; i++) {
            float angle = encoder.GetAngleDegrees();
            sum += angle;
            min_val = std::min(min_val, angle);
            max_val = std::max(max_val, angle);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        float average = sum / samples;
        float range = max_val - min_val;
        
        printf("  Average: %.3f°\n", average);
        printf("  Range: %.3f°\n", range);
        printf("  Stability: %s\n", (range < 0.1f) ? "EXCELLENT" : 
                                   (range < 0.5f) ? "GOOD" : "POOR");
    }
    
    // Test filter thresholds
    printf("\nTesting filter thresholds:\n");
    
    uint8_t fast_thresholds[] = {1, 3, 6, 9};
    uint8_t slow_thresholds[] = {8, 16, 32, 48};
    
    for (int t = 0; t < 4; t++) {
        printf("Fast: %u, Slow: %u\n", fast_thresholds[t], slow_thresholds[t]);
        
        encoder.SetFastFilterThreshold(fast_thresholds[t]);
        encoder.SetSlowFilterThreshold(slow_thresholds[t]);
        
        // Measure response time (simplified test)
        auto start_time = esp_timer_get_time();
        float start_angle = encoder.GetAngleDegrees();
        
        // Wait for significant change (would need actual rotation)
        for (int i = 0; i < 50; i++) {
            float current_angle = encoder.GetAngleDegrees();
            if (std::abs(current_angle - start_angle) > 1.0f) {
                auto response_time = esp_timer_get_time() - start_time;
                printf("  Response time: %llu µs\n", response_time);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    // Restore default settings
    encoder.SetHysteresis(HysteresisLevel::LSB_1);
    encoder.SetFastFilterThreshold(3);
    encoder.SetSlowFilterThreshold(16);
    
    printf("Filter configuration test complete\n");
}
```

### Performance Benchmarking

```cpp
void performance_benchmark() {
    As5047uHandler encoder(*spi);
    encoder.Initialize();
    
    printf("AS5047U Performance Benchmark\n");
    printf("=============================\n");
    
    // Test read speed
    auto start_time = esp_timer_get_time();
    constexpr int READ_COUNT = 10000;
    
    for (int i = 0; i < READ_COUNT; i++) {
        encoder.GetRawPosition();
    }
    
    auto end_time = esp_timer_get_time();
    float avg_read_time = (end_time - start_time) / (float)READ_COUNT;
    float reads_per_second = 1000000.0f / avg_read_time;
    
    printf("Raw position reads:\n");
    printf("  Average time: %.2f µs\n", avg_read_time);
    printf("  Reads per second: %.0f\n", reads_per_second);
    printf("  Max theoretical RPM: %.0f\n", reads_per_second * 60.0f / 16384.0f);
    
    // Test angle conversion performance
    start_time = esp_timer_get_time();
    
    for (int i = 0; i < READ_COUNT; i++) {
        encoder.GetAngleDegrees();
    }
    
    end_time = esp_timer_get_time();
    float avg_angle_time = (end_time - start_time) / (float)READ_COUNT;
    float angles_per_second = 1000000.0f / avg_angle_time;
    
    printf("Angle conversion:\n");
    printf("  Average time: %.2f µs\n", avg_angle_time);
    printf("  Conversions per second: %.0f\n", angles_per_second);
    
    // Test diagnostic read performance
    start_time = esp_timer_get_time();
    constexpr int DIAG_COUNT = 1000;
    
    for (int i = 0; i < DIAG_COUNT; i++) {
        encoder.GetErrorFlags();
        encoder.GetMagneticFieldStrength();
    }
    
    end_time = esp_timer_get_time();
    float avg_diag_time = (end_time - start_time) / (float)DIAG_COUNT;
    
    printf("Diagnostic reads:\n");
    printf("  Average time: %.2f µs\n", avg_diag_time);
    
    // Memory usage
    printf("Memory usage:\n");
    printf("  Handler size: %zu bytes\n", sizeof(As5047uHandler));
    
    printf("Performance benchmark complete\n");
}
```

## 🔍 Advanced Usage

### Custom SPI Interface

```cpp
// Example of creating a custom SPI interface with enhanced timing
class HighSpeedAs5047uHandler : public As5047uHandler {
public:
    HighSpeedAs5047uHandler(BaseSpi& spi) : As5047uHandler(spi) {}
    
    bool Initialize() noexcept override {
        // Configure SPI for maximum speed
        if (!As5047uHandler::Initialize()) {
            return false;
        }
        
        // Additional high-speed optimizations
        ConfigureForHighSpeed();
        return true;
    }
    
    float GetAngleDegreesOptimized() noexcept {
        // Optimized angle reading with reduced overhead
        uint16_t raw = GetRawPositionFast();
        return ConvertRawToAngle(raw);
    }
    
private:
    void ConfigureForHighSpeed() {
        // Reduce filter thresholds for faster response
        SetFastFilterThreshold(1);
        SetSlowFilterThreshold(4);
        
        // Minimize hysteresis
        SetHysteresis(HysteresisLevel::OFF);
    }
    
    uint16_t GetRawPositionFast() noexcept {
        // Direct register access without error checking for speed
        return ReadRegisterDirect(As5047uRegister::ANGLECOM);
    }
    
    float ConvertRawToAngle(uint16_t raw) noexcept {
        // Optimized conversion
        return (raw & 0x3FFF) * (360.0f / 16384.0f);
    }
};
```

### Motion Control Integration

```cpp
// Example of integrating AS5047U with motor control
class EncoderFeedbackController {
public:
    EncoderFeedbackController(As5047uHandler& encoder, MotorController& motor)
        : encoder_(encoder), motor_(motor), target_angle_(0.0f) {}
    
    bool Initialize() {
        if (!encoder_.Initialize()) {
            return false;
        }
        
        // Set encoder as zero reference
        encoder_.SetZeroPosition();
        
        // Configure control parameters
        kp_ = 2.0f;  // Proportional gain
        ki_ = 0.1f;  // Integral gain
        kd_ = 0.05f; // Derivative gain
        
        return true;
    }
    
    void SetTargetAngle(float angle_degrees) {
        target_angle_ = angle_degrees;
    }
    
    void Update() {
        // Read current position
        float current_angle = encoder_.GetAngleDegrees();
        
        // Calculate error
        float error = target_angle_ - current_angle;
        
        // Handle wrap-around
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;
        
        // PID control calculation
        float dt = 0.001f;  // 1ms update rate
        
        integral_error_ += error * dt;
        float derivative_error = (error - previous_error_) / dt;
        
        float control_output = (kp_ * error) + (ki_ * integral_error_) + (kd_ * derivative_error);
        
        // Convert to motor velocity command
        float velocity_rpm = control_output * 10.0f;  // Scale factor
        
        // Apply to motor
        auto* handler = motor_.handler(0);
        if (handler) {
            auto tmc = handler->GetTmc9660Driver();
            tmc->SetTargetVelocity(static_cast<int32_t>(velocity_rpm));
        }
        
        previous_error_ = error;
    }
    
    float GetPositionError() const {
        return target_angle_ - encoder_.GetAngleDegrees();
    }
    
private:
    As5047uHandler& encoder_;
    MotorController& motor_;
    float target_angle_;
    float kp_, ki_, kd_;
    float integral_error_ = 0.0f;
    float previous_error_ = 0.0f;
};
```

## 🚨 Error Handling

### Comprehensive Error Management

```cpp
void comprehensive_error_handling() {
    auto& comm = CommChannelsManager::GetInstance();
    if (!comm.EnsureInitialized()) {
        printf("ERROR: Communication manager failed\n");
        return;
    }
    
    auto* spi = comm.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (!spi) {
        printf("ERROR: SPI device not available\n");
        return;
    }
    
    if (!spi->IsInitialized()) {
        printf("ERROR: SPI interface not initialized\n");
        return;
    }
    
    As5047uHandler encoder(*spi);
    
    // Test communication first
    if (!encoder.TestCommunication()) {
        printf("ERROR: AS5047U not responding\n");
        return;
    }
    
    // Initialize with error checking
    if (!encoder.Initialize()) {
        printf("ERROR: AS5047U initialization failed\n");
        return;
    }
    
    printf("AS5047U initialized successfully\n");
    
    // Monitor for errors during operation
    uint32_t error_count = 0;
    uint32_t total_reads = 0;
    
    for (int i = 0; i < 1000; i++) {
        total_reads++;
        
        // Read position with error checking
        float angle = encoder.GetAngleDegrees();
        
        // Check for hardware errors
        if (encoder.HasErrors()) {
            error_count++;
            auto errors = encoder.GetErrorFlags();
            
            printf("ERROR %u (%.1f%% rate):", error_count, 
                   100.0f * error_count / total_reads);
            
            if (errors.magnetic_field_too_weak) {
                printf(" Magnetic field too weak");
            }
            if (errors.magnetic_field_too_strong) {
                printf(" Magnetic field too strong");
            }
            if (errors.parity_error) {
                printf(" Parity error");
            }
            if (errors.voltage_error) {
                printf(" Voltage error");
            }
            
            printf("\n");
            
            // Too many errors indicate hardware problem
            if (error_count > 50) {
                printf("CRITICAL: Too many errors, check hardware\n");
                break;
            }
        }
        
        // Check magnetic field quality
        if (!encoder.IsMagneticFieldOk()) {
            uint8_t field_strength = encoder.GetMagneticFieldStrength();
            printf("WARNING: Magnetic field issue (strength: %u)\n", field_strength);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float error_rate = 100.0f * error_count / total_reads;
    printf("Operation completed: %u errors in %u reads (%.2f%%)\n", 
           error_count, total_reads, error_rate);
    
    if (error_rate < 1.0f) {
        printf("System operating normally\n");
    } else if (error_rate < 5.0f) {
        printf("Some errors detected, check mounting and magnetic field\n");
    } else {
        printf("High error rate, hardware service required\n");
    }
}
```

## 📚 See Also

- **[MotorController Documentation](../component-handlers/MOTOR_CONTROLLER_README.md)** - Motor control integration
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - SPI interface setup
- **[AS5047U Datasheet](https://ams.com/as5047u)** - Official hardware documentation
- **[SPI Interface Guide](../hardware/SPI_INTERFACE_GUIDE.md)** - SPI communication setup
- **[Position Control Guide](../tutorials/POSITION_CONTROL_GUIDE.md)** - Position control applications

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*