# Pcal95555Handler - GPIO Expander Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Pcal95555Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-PCAL95555-orange.svg)
![Interface](https://img.shields.io/badge/interface-I2C-green.svg)

**High-performance driver handler for PCAL95555 16-bit I2C GPIO expanders**

</div>

## 📋 Overview

The `Pcal95555Handler` is a comprehensive driver handler that provides advanced GPIO expansion capabilities through the PCAL95555 16-bit I2C GPIO expander. It offers full BaseGpio compatibility, interrupt support, and advanced features like agile I/O configuration and programmable output drive strength.

### ✨ Key Features

- **🔌 16-Bit GPIO Expansion**: Full 16-pin GPIO expansion per chip
- **📡 I2C Interface**: Flexible I2C communication with configurable addressing
- **🔔 Interrupt Support**: Hardware interrupt capabilities with edge detection
- **⚡ High Performance**: Optimized batch operations and caching
- **🔧 BaseGpio Compatible**: Drop-in replacement for standard GPIO interfaces
- **🛡️ Safety Features**: Input voltage tolerance and ESD protection
- **📊 Advanced Configuration**: Programmable drive strength and slew rate
- **🏥 Health Monitoring**: Communication status and error detection

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   Pcal95555Handler                             │
├─────────────────────────────────────────────────────────────────┤
│  BaseGpio Interface │ Standard GPIO operations compatibility    │
├─────────────────────────────────────────────────────────────────┤
│  I2C Communication  │ Efficient register access and caching    │
├─────────────────────────────────────────────────────────────────┤
│  Interrupt System   │ Hardware interrupt handling              │
├─────────────────────────────────────────────────────────────────┤
│  PCAL95555 Driver   │ Low-level device register control        │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic GPIO Operations

```cpp
#include "utils-and-drivers/driver-handlers/Pcal95555Handler.h"
#include "component-handlers/CommChannelsManager.h"

void pcal95555_basic_example() {
    // Get I2C interface
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    if (!i2c) {
        printf("I2C interface not available\n");
        return;
    }
    
    // Create PCAL95555 handler
    Pcal95555Handler handler(*i2c, 0x20);  // Address 0x20
    
    // Initialize handler
    if (!handler.Initialize()) {
        printf("Failed to initialize PCAL95555\n");
        return;
    }
    
    // Configure pins
    handler.ConfigurePin(0, false);  // Pin 0 as output
    handler.ConfigurePin(1, true);   // Pin 1 as input
    
    // Basic operations
    handler.SetPin(0, true);         // Set pin 0 high
    bool state = handler.GetPin(1);  // Read pin 1
    
    printf("PCAL95555 GPIO operations complete\n");
    printf("Pin 1 state: %s\n", state ? "HIGH" : "LOW");
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class Pcal95555Handler {
public:
    // Constructor
    Pcal95555Handler(BaseI2c& i2c_interface, uint8_t device_address);
    
    // Initialization
    bool Initialize() noexcept;
    bool IsInitialized() const noexcept;
    void Deinitialize() noexcept;
    
    // Communication testing
    bool TestCommunication() noexcept;
};
```

#### BaseGpio Interface Implementation
```cpp
// Pin configuration
bool ConfigurePin(uint8_t pin, bool is_input, bool enable_pullup = false) noexcept override;

// Basic pin operations
bool SetPin(uint8_t pin, bool state) noexcept override;
bool GetPin(uint8_t pin) noexcept override;
bool TogglePin(uint8_t pin) noexcept override;

// Batch operations
bool SetMultiplePins(uint16_t pin_mask, uint16_t values) noexcept;
uint16_t GetMultiplePins(uint16_t pin_mask) noexcept;

// Pin state queries
bool IsPinConfigured(uint8_t pin) const noexcept override;
bool IsPinInput(uint8_t pin) const noexcept override;
bool IsPinOutput(uint8_t pin) const noexcept override;
```

#### Advanced Features
```cpp
// Interrupt configuration
using GpioInterruptCallback = std::function<void(uint8_t pin, bool state)>;

bool SetInterrupt(uint8_t pin, GpioInterruptCallback callback,
                  InterruptType type = InterruptType::BOTH_EDGES) noexcept;
bool RemoveInterrupt(uint8_t pin) noexcept;
bool EnableInterrupts(bool enable) noexcept;

// Advanced pin configuration
bool SetDriveStrength(uint8_t pin, DriveStrength strength) noexcept;
bool SetSlewRate(uint8_t pin, SlewRate rate) noexcept;
bool SetInputLatch(uint8_t pin, bool enable) noexcept;

// Device information
uint8_t GetDeviceAddress() const noexcept;
uint16_t GetChipId() const noexcept;
```

### Configuration Enums

```cpp
// Drive strength options
enum class DriveStrength {
    LOW_25_PERCENT = 0,     // 25% drive strength
    MEDIUM_50_PERCENT = 1,  // 50% drive strength  
    HIGH_75_PERCENT = 2,    // 75% drive strength
    FULL_100_PERCENT = 3    // 100% drive strength
};

// Slew rate control
enum class SlewRate {
    SLOW = 0,    // Slow slew rate (reduced EMI)
    FAST = 1     // Fast slew rate (faster switching)
};

// Interrupt types
enum class InterruptType {
    RISING_EDGE = 0,
    FALLING_EDGE = 1,
    BOTH_EDGES = 2,
    LOW_LEVEL = 3,
    HIGH_LEVEL = 4
};
```

## 🎯 Hardware Features

### PCAL95555 Capabilities

| Feature | Specification | Description |
|---------|---------------|-------------|
| **GPIO Pins** | 16 pins | Bidirectional I/O pins |
| **Operating Voltage** | 1.65V - 5.5V | Wide voltage range |
| **I2C Interface** | Standard/Fast mode | Up to 400kHz |
| **Address Range** | 0x20 - 0x27 | 8 possible addresses |
| **Current Capability** | 25mA per pin | High current drive |
| **Input Voltage** | 5.5V tolerant | Overvoltage protection |
| **Interrupt Support** | Hardware interrupt | Single interrupt pin |
| **ESD Protection** | ±2kV HBM | Robust protection |

### Pin Configuration Matrix

```cpp
// Pin numbering (0-15)
// Port 0: Pins 0-7
// Port 1: Pins 8-15

// Example pin assignments
enum class Pcal95555Pins {
    // Port 0 (0-7)
    USER_LED_1 = 0,
    USER_LED_2 = 1,
    USER_BUTTON_1 = 2,
    USER_BUTTON_2 = 3,
    RELAY_CONTROL_1 = 4,
    RELAY_CONTROL_2 = 5,
    STATUS_LED = 6,
    SPARE_IO_1 = 7,
    
    // Port 1 (8-15)  
    MOTOR_ENABLE = 8,
    MOTOR_DIRECTION = 9,
    LIMIT_SWITCH_1 = 10,
    LIMIT_SWITCH_2 = 11,
    ENCODER_A = 12,
    ENCODER_B = 13,
    FAULT_INPUT = 14,
    SPARE_IO_2 = 15
};
```

## 🔧 Configuration

### Device Addressing

```cpp
// PCAL95555 I2C address configuration
// Base address: 0x20
// Address pins A2, A1, A0 set the lower 3 bits

constexpr uint8_t PCAL95555_BASE_ADDRESS = 0x20;

// Address options (A2=0, A1=0, A0=X)
constexpr uint8_t PCAL95555_ADDR_0 = 0x20;  // A2=0, A1=0, A0=0
constexpr uint8_t PCAL95555_ADDR_1 = 0x21;  // A2=0, A1=0, A0=1
constexpr uint8_t PCAL95555_ADDR_2 = 0x22;  // A2=0, A1=1, A0=0
constexpr uint8_t PCAL95555_ADDR_3 = 0x23;  // A2=0, A1=1, A0=1
constexpr uint8_t PCAL95555_ADDR_4 = 0x24;  // A2=1, A1=0, A0=0
constexpr uint8_t PCAL95555_ADDR_5 = 0x25;  // A2=1, A1=0, A0=1
constexpr uint8_t PCAL95555_ADDR_6 = 0x26;  // A2=1, A1=1, A0=0
constexpr uint8_t PCAL95555_ADDR_7 = 0x27;  // A2=1, A1=1, A0=1
```

### Initialization Configuration

```cpp
// Initialization options
struct Pcal95555Config {
    uint8_t device_address = 0x20;
    uint32_t i2c_timeout_ms = 100;
    bool enable_interrupts = true;
    uint8_t interrupt_pin = GPIO_NUM_NC;
    DriveStrength default_drive_strength = DriveStrength::MEDIUM_50_PERCENT;
    SlewRate default_slew_rate = SlewRate::FAST;
    bool enable_input_latch = false;
    bool auto_increment_registers = true;
};
```

## 📊 Examples

### Basic GPIO Control

```cpp
#include "utils-and-drivers/driver-handlers/Pcal95555Handler.h"

void basic_gpio_control() {
    // Setup I2C communication
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    
    // Create handler
    Pcal95555Handler handler(*i2c, 0x20);
    
    if (!handler.Initialize()) {
        printf("PCAL95555 initialization failed\n");
        return;
    }
    
    // Configure pins for LED control
    handler.ConfigurePin(0, false);  // LED 1 (output)
    handler.ConfigurePin(1, false);  // LED 2 (output)
    handler.ConfigurePin(2, false);  // LED 3 (output)
    
    // Configure pins for button inputs
    handler.ConfigurePin(8, true, true);   // Button 1 (input with pullup)
    handler.ConfigurePin(9, true, true);   // Button 2 (input with pullup)
    
    printf("PCAL95555 basic GPIO control demo\n");
    
    // LED pattern demo
    for (int i = 0; i < 10; i++) {
        // Set LEDs in sequence
        handler.SetPin(0, i % 3 == 0);
        handler.SetPin(1, i % 3 == 1);
        handler.SetPin(2, i % 3 == 2);
        
        // Read button states
        bool btn1 = !handler.GetPin(8);  // Active low
        bool btn2 = !handler.GetPin(9);  // Active low
        
        printf("LEDs: %d%d%d, Buttons: %s %s\n",
               i % 3 == 0 ? 1 : 0,
               i % 3 == 1 ? 1 : 0, 
               i % 3 == 2 ? 1 : 0,
               btn1 ? "PRESSED" : "RELEASED",
               btn2 ? "PRESSED" : "RELEASED");
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Turn off all LEDs
    handler.SetPin(0, false);
    handler.SetPin(1, false);
    handler.SetPin(2, false);
}
```

### Batch Operations

```cpp
void batch_operations_example() {
    Pcal95555Handler handler(*i2c, 0x20);
    handler.Initialize();
    
    // Configure all pins as outputs
    for (int pin = 0; pin < 16; pin++) {
        handler.ConfigurePin(pin, false);
    }
    
    printf("PCAL95555 batch operations demo\n");
    
    // Pattern 1: All pins high
    handler.SetMultiplePins(0xFFFF, 0xFFFF);
    printf("All pins HIGH\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Pattern 2: All pins low
    handler.SetMultiplePins(0xFFFF, 0x0000);
    printf("All pins LOW\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Pattern 3: Alternating pattern
    handler.SetMultiplePins(0xFFFF, 0xAAAA);  // 1010101010101010
    printf("Alternating pattern 1\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Pattern 4: Opposite alternating
    handler.SetMultiplePins(0xFFFF, 0x5555);  // 0101010101010101
    printf("Alternating pattern 2\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Pattern 5: Walking bit
    for (int bit = 0; bit < 16; bit++) {
        uint16_t pattern = 1 << bit;
        handler.SetMultiplePins(0xFFFF, pattern);
        printf("Walking bit position %d\n", bit);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Clear all pins
    handler.SetMultiplePins(0xFFFF, 0x0000);
    printf("Batch operations complete\n");
}
```

### Interrupt Handling

```cpp
void interrupt_handling_example() {
    Pcal95555Handler handler(*i2c, 0x20);
    handler.Initialize();
    
    // Configure interrupt pins
    handler.ConfigurePin(10, true, true);  // Limit switch 1
    handler.ConfigurePin(11, true, true);  // Limit switch 2
    handler.ConfigurePin(14, true, true);  // Fault input
    
    // Setup interrupt callbacks
    auto limit_switch_callback = [](uint8_t pin, bool state) {
        printf("Limit switch %d: %s\n", pin - 9, state ? "RELEASED" : "ACTIVATED");
    };
    
    auto fault_callback = [](uint8_t pin, bool state) {
        if (!state) {  // Active low fault
            printf("FAULT detected on pin %d!\n", pin);
        } else {
            printf("Fault cleared on pin %d\n", pin);
        }
    };
    
    // Register interrupt handlers
    handler.SetInterrupt(10, limit_switch_callback, InterruptType::BOTH_EDGES);
    handler.SetInterrupt(11, limit_switch_callback, InterruptType::BOTH_EDGES);
    handler.SetInterrupt(14, fault_callback, InterruptType::BOTH_EDGES);
    
    // Enable interrupt system
    handler.EnableInterrupts(true);
    
    printf("Interrupt monitoring active. Trigger inputs...\n");
    
    // Monitor for 30 seconds
    for (int i = 0; i < 300; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Cleanup
    handler.RemoveInterrupt(10);
    handler.RemoveInterrupt(11);
    handler.RemoveInterrupt(14);
    handler.EnableInterrupts(false);
    
    printf("Interrupt monitoring stopped\n");
}
```

### Advanced Configuration

```cpp
void advanced_configuration_example() {
    Pcal95555Handler handler(*i2c, 0x20);
    handler.Initialize();
    
    printf("PCAL95555 advanced configuration demo\n");
    
    // Configure high-current outputs with different drive strengths
    handler.ConfigurePin(0, false);  // Motor enable
    handler.ConfigurePin(1, false);  // Relay control 1
    handler.ConfigurePin(2, false);  // Relay control 2
    
    // Set maximum drive strength for high-current loads
    handler.SetDriveStrength(0, DriveStrength::FULL_100_PERCENT);
    handler.SetDriveStrength(1, DriveStrength::HIGH_75_PERCENT);
    handler.SetDriveStrength(2, DriveStrength::HIGH_75_PERCENT);
    
    // Configure fast slew rate for quick switching
    handler.SetSlewRate(0, SlewRate::FAST);
    handler.SetSlewRate(1, SlewRate::FAST);
    handler.SetSlewRate(2, SlewRate::FAST);
    
    // Configure sensitive inputs with input latch
    handler.ConfigurePin(8, true, true);   // Encoder A
    handler.ConfigurePin(9, true, true);   // Encoder B
    
    // Enable input latch for stable readings
    handler.SetInputLatch(8, true);
    handler.SetInputLatch(9, true);
    
    // Configure low-noise outputs with slow slew rate
    handler.ConfigurePin(6, false);  // Status LED
    handler.SetDriveStrength(6, DriveStrength::LOW_25_PERCENT);
    handler.SetSlewRate(6, SlewRate::SLOW);
    
    printf("Advanced configuration applied:\n");
    printf("- High-current outputs: Full/75%% drive strength, fast slew\n");
    printf("- Sensitive inputs: Input latch enabled\n");
    printf("- Status LED: Low drive strength, slow slew (low noise)\n");
    
    // Test high-current outputs
    printf("Testing high-current outputs...\n");
    handler.SetPin(0, true);   // Motor enable
    vTaskDelay(pdMS_TO_TICKS(500));
    handler.SetPin(1, true);   // Relay 1
    vTaskDelay(pdMS_TO_TICKS(500));
    handler.SetPin(2, true);   // Relay 2
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Status LED indication
    for (int i = 0; i < 5; i++) {
        handler.SetPin(6, true);
        vTaskDelay(pdMS_TO_TICKS(200));
        handler.SetPin(6, false);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Turn off all outputs
    handler.SetPin(0, false);
    handler.SetPin(1, false);
    handler.SetPin(2, false);
    handler.SetPin(6, false);
    
    printf("Advanced configuration demo complete\n");
}
```

### Multi-Device Chain

```cpp
void multi_device_chain_example() {
    // Setup multiple PCAL95555 devices
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    
    // Create handlers for multiple devices
    Pcal95555Handler device1(*i2c, 0x20);  // Address 0x20
    Pcal95555Handler device2(*i2c, 0x21);  // Address 0x21
    Pcal95555Handler device3(*i2c, 0x22);  // Address 0x22
    
    // Initialize all devices
    if (!device1.Initialize() || !device2.Initialize() || !device3.Initialize()) {
        printf("Failed to initialize PCAL95555 devices\n");
        return;
    }
    
    printf("Multi-device PCAL95555 chain initialized\n");
    printf("Device 1 (0x20): %s\n", device1.TestCommunication() ? "OK" : "FAIL");
    printf("Device 2 (0x21): %s\n", device2.TestCommunication() ? "OK" : "FAIL");
    printf("Device 3 (0x22): %s\n", device3.TestCommunication() ? "OK" : "FAIL");
    
    // Configure all pins as outputs
    for (int pin = 0; pin < 16; pin++) {
        device1.ConfigurePin(pin, false);
        device2.ConfigurePin(pin, false);
        device3.ConfigurePin(pin, false);
    }
    
    // Synchronized LED chase across all devices
    printf("Running synchronized LED chase...\n");
    
    for (int cycle = 0; cycle < 3; cycle++) {
        // Device 1 chase
        for (int pin = 0; pin < 16; pin++) {
            device1.SetMultiplePins(0xFFFF, 1 << pin);
            device2.SetMultiplePins(0xFFFF, 0x0000);
            device3.SetMultiplePins(0xFFFF, 0x0000);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Device 2 chase
        for (int pin = 0; pin < 16; pin++) {
            device1.SetMultiplePins(0xFFFF, 0x0000);
            device2.SetMultiplePins(0xFFFF, 1 << pin);
            device3.SetMultiplePins(0xFFFF, 0x0000);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Device 3 chase
        for (int pin = 0; pin < 16; pin++) {
            device1.SetMultiplePins(0xFFFF, 0x0000);
            device2.SetMultiplePins(0xFFFF, 0x0000);
            device3.SetMultiplePins(0xFFFF, 1 << pin);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    // Clear all devices
    device1.SetMultiplePins(0xFFFF, 0x0000);
    device2.SetMultiplePins(0xFFFF, 0x0000);
    device3.SetMultiplePins(0xFFFF, 0x0000);
    
    printf("Multi-device chain demo complete\n");
    printf("Total GPIO pins available: 48 (3 × 16)\n");
}
```

### Performance Benchmarking

```cpp
void performance_benchmark() {
    Pcal95555Handler handler(*i2c, 0x20);
    handler.Initialize();
    
    printf("PCAL95555 Performance Benchmark\n");
    printf("===============================\n");
    
    // Configure pin 0 as output for testing
    handler.ConfigurePin(0, false);
    
    // Test single pin operations
    auto start_time = esp_timer_get_time();
    constexpr int SINGLE_OPS = 1000;
    
    for (int i = 0; i < SINGLE_OPS; i++) {
        handler.SetPin(0, i % 2);
    }
    
    auto end_time = esp_timer_get_time();
    float single_op_time = (end_time - start_time) / (float)SINGLE_OPS;
    float single_ops_per_sec = 1000000.0f / single_op_time;
    
    printf("Single pin operations:\n");
    printf("  Average time: %.2f µs\n", single_op_time);
    printf("  Operations per second: %.0f\n", single_ops_per_sec);
    
    // Test batch operations
    start_time = esp_timer_get_time();
    constexpr int BATCH_OPS = 1000;
    
    for (int i = 0; i < BATCH_OPS; i++) {
        handler.SetMultiplePins(0xFFFF, i & 0xFFFF);
    }
    
    end_time = esp_timer_get_time();
    float batch_op_time = (end_time - start_time) / (float)BATCH_OPS;
    float batch_ops_per_sec = 1000000.0f / batch_op_time;
    float pins_per_batch = 16.0f;
    float effective_pin_ops_per_sec = batch_ops_per_sec * pins_per_batch;
    
    printf("Batch operations (16 pins):\n");
    printf("  Average time: %.2f µs\n", batch_op_time);
    printf("  Batch operations per second: %.0f\n", batch_ops_per_sec);
    printf("  Effective pin ops per second: %.0f\n", effective_pin_ops_per_sec);
    printf("  Speedup factor: %.1fx\n", effective_pin_ops_per_sec / single_ops_per_sec);
    
    // Test read operations
    start_time = esp_timer_get_time();
    constexpr int READ_OPS = 1000;
    
    for (int i = 0; i < READ_OPS; i++) {
        handler.GetPin(0);
    }
    
    end_time = esp_timer_get_time();
    float read_op_time = (end_time - start_time) / (float)READ_OPS;
    float read_ops_per_sec = 1000000.0f / read_op_time;
    
    printf("Read operations:\n");
    printf("  Average time: %.2f µs\n", read_op_time);
    printf("  Reads per second: %.0f\n", read_ops_per_sec);
    
    // Test batch read operations
    start_time = esp_timer_get_time();
    constexpr int BATCH_READS = 1000;
    
    for (int i = 0; i < BATCH_READS; i++) {
        handler.GetMultiplePins(0xFFFF);
    }
    
    end_time = esp_timer_get_time();
    float batch_read_time = (end_time - start_time) / (float)BATCH_READS;
    float batch_reads_per_sec = 1000000.0f / batch_read_time;
    float effective_pin_reads_per_sec = batch_reads_per_sec * pins_per_batch;
    
    printf("Batch reads (16 pins):\n");
    printf("  Average time: %.2f µs\n", batch_read_time);
    printf("  Batch reads per second: %.0f\n", batch_reads_per_sec);
    printf("  Effective pin reads per second: %.0f\n", effective_pin_reads_per_sec);
    printf("  Read speedup factor: %.1fx\n", effective_pin_reads_per_sec / read_ops_per_sec);
}
```

## 🔍 Advanced Usage

### Custom I2C Interface

```cpp
// Example of creating a custom I2C interface with enhanced error handling
class EnhancedPcal95555Handler : public Pcal95555Handler {
public:
    EnhancedPcal95555Handler(BaseI2c& i2c, uint8_t addr) 
        : Pcal95555Handler(i2c, addr), retry_count_(0) {}
    
    bool Initialize() noexcept override {
        // Add retry logic to initialization
        constexpr int MAX_RETRIES = 3;
        
        for (int retry = 0; retry < MAX_RETRIES; retry++) {
            if (Pcal95555Handler::Initialize()) {
                printf("PCAL95555 initialized on retry %d\n", retry);
                return true;
            }
            
            printf("Initialization attempt %d failed, retrying...\n", retry + 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        printf("PCAL95555 initialization failed after %d retries\n", MAX_RETRIES);
        return false;
    }
    
    bool TestCommunication() noexcept override {
        // Enhanced communication test with detailed reporting
        bool result = Pcal95555Handler::TestCommunication();
        
        if (result) {
            printf("PCAL95555 communication OK (retries: %u)\n", retry_count_);
            retry_count_ = 0;  // Reset retry counter on success
        } else {
            retry_count_++;
            printf("PCAL95555 communication failed (retry %u)\n", retry_count_);
        }
        
        return result;
    }
    
private:
    uint32_t retry_count_;
};
```

### State Machine Integration

```cpp
// Example of integrating PCAL95555 with a state machine
class GpioStateMachine {
public:
    enum class State {
        IDLE,
        STARTUP,
        RUNNING,
        FAULT,
        SHUTDOWN
    };
    
    GpioStateMachine(Pcal95555Handler& handler) : handler_(handler), current_state_(State::IDLE) {
        // Configure pins for state machine
        handler_.ConfigurePin(MOTOR_ENABLE_PIN, false);    // Motor enable
        handler_.ConfigurePin(STATUS_LED_PIN, false);      // Status LED
        handler_.ConfigurePin(FAULT_LED_PIN, false);       // Fault LED
        handler_.ConfigurePin(ESTOP_INPUT_PIN, true, true); // E-stop input
        handler_.ConfigurePin(START_BUTTON_PIN, true, true); // Start button
        
        // Setup interrupt for emergency stop
        handler_.SetInterrupt(ESTOP_INPUT_PIN, 
            [this](uint8_t pin, bool state) {
                if (!state) {  // E-stop activated (active low)
                    SetState(State::FAULT);
                }
            }, InterruptType::FALLING_EDGE);
    }
    
    void Update() {
        switch (current_state_) {
            case State::IDLE:
                HandleIdleState();
                break;
            case State::STARTUP:
                HandleStartupState();
                break;
            case State::RUNNING:
                HandleRunningState();
                break;
            case State::FAULT:
                HandleFaultState();
                break;
            case State::SHUTDOWN:
                HandleShutdownState();
                break;
        }
    }
    
private:
    static constexpr uint8_t MOTOR_ENABLE_PIN = 0;
    static constexpr uint8_t STATUS_LED_PIN = 1;
    static constexpr uint8_t FAULT_LED_PIN = 2;
    static constexpr uint8_t ESTOP_INPUT_PIN = 8;
    static constexpr uint8_t START_BUTTON_PIN = 9;
    
    Pcal95555Handler& handler_;
    State current_state_;
    uint32_t state_timer_;
    
    void SetState(State new_state) {
        printf("State change: %d -> %d\n", static_cast<int>(current_state_), static_cast<int>(new_state));
        current_state_ = new_state;
        state_timer_ = esp_timer_get_time() / 1000;  // Reset timer
        
        // Update outputs based on state
        UpdateOutputs();
    }
    
    void UpdateOutputs() {
        switch (current_state_) {
            case State::IDLE:
                handler_.SetPin(MOTOR_ENABLE_PIN, false);
                handler_.SetPin(STATUS_LED_PIN, false);
                handler_.SetPin(FAULT_LED_PIN, false);
                break;
                
            case State::STARTUP:
                handler_.SetPin(MOTOR_ENABLE_PIN, false);
                handler_.SetPin(STATUS_LED_PIN, true);   // Solid status LED
                handler_.SetPin(FAULT_LED_PIN, false);
                break;
                
            case State::RUNNING:
                handler_.SetPin(MOTOR_ENABLE_PIN, true);
                // Blink status LED in RUNNING state (handled in HandleRunningState)
                handler_.SetPin(FAULT_LED_PIN, false);
                break;
                
            case State::FAULT:
                handler_.SetPin(MOTOR_ENABLE_PIN, false);
                handler_.SetPin(STATUS_LED_PIN, false);
                handler_.SetPin(FAULT_LED_PIN, true);    // Solid fault LED
                break;
                
            case State::SHUTDOWN:
                handler_.SetPin(MOTOR_ENABLE_PIN, false);
                handler_.SetPin(STATUS_LED_PIN, false);
                handler_.SetPin(FAULT_LED_PIN, false);
                break;
        }
    }
    
    void HandleIdleState() {
        // Check for start button press
        if (!handler_.GetPin(START_BUTTON_PIN)) {  // Active low
            SetState(State::STARTUP);
        }
    }
    
    void HandleStartupState() {
        uint32_t elapsed = (esp_timer_get_time() / 1000) - state_timer_;
        
        if (elapsed > 2000) {  // 2 second startup delay
            SetState(State::RUNNING);
        }
    }
    
    void HandleRunningState() {
        uint32_t elapsed = (esp_timer_get_time() / 1000) - state_timer_;
        
        // Blink status LED every 500ms in running state
        bool led_state = (elapsed % 1000) < 500;
        handler_.SetPin(STATUS_LED_PIN, led_state);
        
        // Check for stop button
        if (!handler_.GetPin(START_BUTTON_PIN)) {  // Same button for start/stop
            SetState(State::SHUTDOWN);
        }
    }
    
    void HandleFaultState() {
        // Stay in fault state until manual reset
        // Could add automatic fault recovery logic here
    }
    
    void HandleShutdownState() {
        uint32_t elapsed = (esp_timer_get_time() / 1000) - state_timer_;
        
        if (elapsed > 1000) {  // 1 second shutdown delay
            SetState(State::IDLE);
        }
    }
};
```

## 🚨 Error Handling

### Comprehensive Error Management

```cpp
void comprehensive_error_handling() {
    auto& comm = CommChannelsManager::GetInstance();
    if (!comm.EnsureInitialized()) {
        printf("ERROR: Communication manager initialization failed\n");
        return;
    }
    
    auto* i2c = comm.GetI2cDevice(I2cDeviceId::PCAL95555_GPIO_EXPANDER);
    if (!i2c) {
        printf("ERROR: I2C device not available\n");
        return;
    }
    
    // Test I2C interface
    if (!i2c->IsInitialized()) {
        printf("ERROR: I2C interface not initialized\n");
        return;
    }
    
    // Create handler with error checking
    try {
        Pcal95555Handler handler(*i2c, 0x20);
        
        // Test communication before initialization
        if (!handler.TestCommunication()) {
            printf("ERROR: PCAL95555 not responding at address 0x20\n");
            
            // Try alternative addresses
            for (uint8_t addr = 0x21; addr <= 0x27; addr++) {
                Pcal95555Handler test_handler(*i2c, addr);
                if (test_handler.TestCommunication()) {
                    printf("FOUND: PCAL95555 responding at address 0x%02X\n", addr);
                    break;
                }
            }
            return;
        }
        
        // Initialize with error checking
        if (!handler.Initialize()) {
            printf("ERROR: PCAL95555 initialization failed\n");
            return;
        }
        
        // Verify chip ID
        uint16_t chip_id = handler.GetChipId();
        if (chip_id != 0x9555) {  // Expected PCAL95555 chip ID
            printf("WARNING: Unexpected chip ID: 0x%04X (expected 0x9555)\n", chip_id);
        }
        
        printf("PCAL95555 initialized successfully (ID: 0x%04X)\n", chip_id);
        
        // Test basic functionality
        handler.ConfigurePin(0, false);  // Configure as output
        
        // Test pin operations with error checking
        if (!handler.SetPin(0, true)) {
            printf("ERROR: Failed to set pin 0\n");
            return;
        }
        
        // Monitor communication health
        printf("Running with communication monitoring...\n");
        uint32_t error_count = 0;
        
        for (int i = 0; i < 100; i++) {
            if (!handler.TestCommunication()) {
                error_count++;
                printf("WARNING: Communication error %u at iteration %d\n", error_count, i);
                
                if (error_count > 5) {
                    printf("ERROR: Too many communication errors, aborting\n");
                    break;
                }
            }
            
            // Toggle pin to test functionality
            handler.TogglePin(0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        if (error_count == 0) {
            printf("Operation completed successfully with no errors\n");
        } else {
            printf("Operation completed with %u communication errors\n", error_count);
        }
        
    } catch (const std::exception& e) {
        printf("EXCEPTION: %s\n", e.what());
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](../component-handlers/GPIO_MANAGER_README.md)** - GPIO management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[PCAL95555 Datasheet](https://www.nxp.com/docs/en/data-sheet/PCAL95555.pdf)** - Official hardware documentation
- **[I2C Interface Guide](../hardware/I2C_INTERFACE_GUIDE.md)** - I2C communication setup
- **[Hardware Setup Guide](../hardware/HARDWARE_SETUP.md)** - Hardware configuration

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*