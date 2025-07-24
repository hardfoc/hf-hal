# ImuManager GPIO Interrupt Support Implementation

## Overview

I've successfully implemented GPIO interrupt support for the ImuManager class, allowing the BNO08x IMU to operate in interrupt-driven mode using the PCAL_IMU_INT pin through the GpioManager system.

## Key Features Added

### 1. **GPIO Interrupt Integration**
- **Pin Used**: `PCAL_IMU_INT` (PCAL95555 IO4, active low)
- **Integration**: Uses GpioManager to access the interrupt pin
- **Configuration**: Automatic setup during ImuManager initialization
- **Thread Safety**: All interrupt operations protected by RtosMutex

### 2. **New ImuManager Methods**

```cpp
// Interrupt configuration and control
bool ConfigureInterrupt(std::function<void()> callback = nullptr) noexcept;
bool EnableInterrupt() noexcept;
bool DisableInterrupt() noexcept;
bool IsInterruptEnabled() const noexcept;

// Interrupt-driven task synchronization
bool WaitForInterrupt(uint32_t timeout_ms = 0) noexcept;
uint32_t GetInterruptCount() const noexcept;
```

### 3. **Automatic Initialization**
- **GpioManager Integration**: Automatically gets reference to GpioManager during initialization
- **Pin Registration**: Registers PCAL_IMU_INT as input with pull-up resistor
- **Graceful Fallback**: If GPIO interrupt not available, falls back to polling mode
- **Non-blocking**: GPIO setup failure doesn't prevent IMU initialization

### 4. **Interrupt Handler Implementation**
- **ISR Safety**: Minimal interrupt handler for embedded safety
- **Semaphore Signaling**: Uses FreeRTOS binary semaphore for task synchronization
- **Counter Tracking**: Atomic interrupt counter for monitoring
- **User Callbacks**: Optional user callback execution in ISR context

## Usage Patterns

### Basic Interrupt Setup
```cpp
auto& imu_mgr = ImuManager::GetInstance();
imu_mgr.Initialize();  // Automatically sets up GPIO interrupt

// Configure interrupt with optional callback
bool configured = imu_mgr.ConfigureInterrupt([]() {
    // Minimal ISR callback - keep it simple!
});

if (configured && imu_mgr.EnableInterrupt()) {
    // Interrupt is ready for use
}
```

### Task-Based Interrupt Processing
```cpp
void imu_task(void* pvParameters) {
    auto& imu_mgr = ImuManager::GetInstance();
    
    while (running) {
        if (imu_mgr.WaitForInterrupt(1000)) {  // 1 second timeout
            // Interrupt occurred - process sensor data
            auto* handler = imu_mgr.GetBno08xHandler();
            if (handler) {
                handler->Update();  // Process new sensor data
            }
        }
    }
}
```

### Polling Fallback
```cpp
auto& imu_mgr = ImuManager::GetInstance();
if (!imu_mgr.IsInterruptEnabled()) {
    // Use polling mode
    while (running) {
        auto* handler = imu_mgr.GetBno08xHandler();
        if (handler) {
            handler->Update();
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz polling
    }
}
```

## Technical Implementation Details

### Pin Configuration
- **Functional Pin**: `HfFunctionalGpioPin::PCAL_IMU_INT`
- **GpioManager Type**: `HardFOC::FunctionalGpioPin` (with static_cast conversion)
- **Direction**: Input with pull-up resistor
- **Trigger**: Falling edge (BNO08x INT is active low)

### Memory Management
- **FreeRTOS Semaphore**: Binary semaphore for WaitForInterrupt()
- **Atomic Counter**: Thread-safe interrupt counting
- **RAII Cleanup**: Automatic resource cleanup in destructor

### Error Handling
- **Graceful Degradation**: Works without GPIO manager or interrupt pin
- **Comprehensive Logging**: ESP-IDF logging for debugging
- **Return Value Validation**: All methods return success/failure status

## Examples Created

### 1. **Enhanced Bno08xHandlerImuExample.cpp**
- Shows basic interrupt setup and usage
- Demonstrates both polling and interrupt modes
- Includes error handling patterns

### 2. **ImuInterruptTaskExample.cpp**
- Complete FreeRTOS task-based example
- Shows proper task creation and cleanup
- Demonstrates WaitForInterrupt() usage
- Includes 30-second demonstration run

## Benefits

### 1. **Power Efficiency**
- Tasks can sleep until data is available
- No continuous polling required
- Reduced CPU usage

### 2. **Responsive Processing**
- Immediate notification when new sensor data arrives
- Minimal latency between data ready and processing
- Deterministic response times

### 3. **Architectural Consistency**
- Follows same patterns as MotorController and other managers
- Integrates seamlessly with existing GPIO architecture
- Maintains exception-free design principles

### 4. **Backward Compatibility**
- Existing polling-mode code continues to work
- Optional interrupt functionality
- Graceful fallback to polling if GPIO not available

## Hardware Requirements

- **BNO08x IMU**: Connected to I2C bus
- **PCAL95555 GPIO Expander**: For interrupt pin access
- **Hardware Connection**: BNO08x INT pin to PCAL95555 IO4
- **Board Support**: Vortex V1 board with PCAL_IMU_INT pin mapping

## Future Enhancements

1. **Multiple IMU Support**: Extend to support multiple BNO08x devices with separate interrupt pins
2. **Interrupt Priority**: Configurable interrupt priority levels
3. **Advanced Filtering**: Hardware interrupt filtering and debouncing
4. **Power Management**: Integration with ESP32 power management for deep sleep support

---

This implementation provides a complete, production-ready GPIO interrupt system for the BNO08x IMU that integrates seamlessly with the existing HardFOC architecture while maintaining the high standards of safety, efficiency, and usability established by the other device managers.
