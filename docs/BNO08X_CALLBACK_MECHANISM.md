# BNO08x Callback Mechanism - ESP32 Implementation with EspGpio

## Overview

The BNO08x IMU operates with a **callback-based data delivery system**. This document explains how callbacks work and the new **EspGpio-based interrupt support** in the ImuManager.

## How BNO08x Callbacks Work

### 1. **Polling Mode (Recommended for Most Applications)**

```cpp
// Setup
auto& imu_mgr = ImuManager::GetInstance();
if (imu_mgr.EnsureInitialized()) {
    BNO085& bno08x = imu_mgr.GetBno08x();
    bno08x.setCallback([](const SensorEvent& event) {
        ESP_LOGI("IMU", "New sensor data: %d", (int)event.sensor);
    });
    bno08x.enableSensor(BNO085Sensor::RotationVector, 50); // 20Hz

    // Main loop
    while (true) {
        bno08x.update();  // ← This triggers callbacks when new data is available
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz polling rate
    }
}
```

### 2. **Interrupt Mode with EspGpio (New & Improved)**

```cpp
// Hardware: BNO08x INT pin → ESP32 GPIO (via board mapping)
// Software: Clean API using ImuManager + EspGpio

auto& imu_mgr = ImuManager::GetInstance();
if (imu_mgr.EnsureInitialized()) {
    BNO085& bno08x = imu_mgr.GetBno08x();
    
    // Configure interrupt with optional ISR callback
    if (imu_mgr.ConfigureInterrupt([](){ 
        // Optional callback executed in ISR context - keep minimal
    })) {
        imu_mgr.EnableInterrupt();
        ESP_LOGI("IMU", "Interrupt mode enabled");
        
        // Wait for interrupts and process data
        while (true) {
            // Wait for interrupt notification, then:
            bno08x.update();  // ← Still need this call!
        }
    } else {
        ESP_LOGW("IMU", "Interrupt not available, using polling");
        // Fallback to polling mode
    }
}
```

## Key Advantages of EspGpio Integration

### **Why Use ImuManager + EspGpio?**

✅ **Clean API**: Simple `ConfigureInterrupt()` / `EnableInterrupt()` calls  
✅ **Automatic GPIO Management**: No manual ESP-IDF GPIO configuration  
✅ **Board Abstraction**: Uses functional pin mapping system  
✅ **Error Handling**: Comprehensive error checking and logging  
✅ **Fallback Support**: Gracefully falls back to polling if interrupt unavailable  
✅ **Thread Safety**: All operations are thread-safe  
✅ **Resource Management**: Automatic cleanup in destructor  

### **Comparison: Raw GPIO vs EspGpio**

```cpp
// ❌ Raw ESP-IDF GPIO (old way)
gpio_config_t io_conf = {};
io_conf.intr_type = GPIO_INTR_NEGEDGE;
io_conf.mode = GPIO_MODE_INPUT;
io_conf.pin_bit_mask = (1ULL << 20);
gpio_config(&io_conf);
gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
gpio_isr_handler_add(GPIO_NUM_20, gpio_isr_handler, NULL);

// ✅ EspGpio + ImuManager (new way)
auto& imu_mgr = ImuManager::GetInstance();
if (imu_mgr.ConfigureInterrupt() && imu_mgr.EnableInterrupt()) {
    // Interrupt ready!
}
```

## Callback Execution Context

```cpp
void MySensorCallback(const SensorEvent& event) {
    // ✅ Safe operations in callback:
    ESP_LOGI("IMU", "Sensor data received");
    memcpy(&global_sensor_data, &event, sizeof(event));
    xQueueSend(data_queue, &event, 0);
    
    // ❌ Avoid in callback:
    vTaskDelay(100);           // Blocking operations
    printf("Long message");    // Slow I/O operations
    malloc(1024);             // Large memory allocations
}
```

**Important:** The callback is **NOT an interrupt handler**. It's a regular function call, so:
- ✅ You can use ESP_LOG, FreeRTOS functions, etc.
- ✅ Safe to access global variables
- ❌ Keep it fast - avoid blocking operations

## Hardware Connections

### **Minimum Required (Polling Mode):**
```
BNO08x → ESP32
VCC    → 3.3V
GND    → GND
SDA    → GPIO18 (I2C_SDA)
SCL    → GPIO19 (I2C_SCL)
```

### **Optional for Interrupt Mode:**
```
BNO08x → ESP32
INT    → GPIO20 (any available GPIO)
RST    → GPIO25 (optional, for hardware reset)
```

## Example Sensor Rates

| Sensor Type | Typical Rate | Update() Call Rate |
|-------------|--------------|-------------------|
| Rotation Vector | 20-100 Hz | 100 Hz |
| Accelerometer | 10-200 Hz | 100 Hz |
| Gyroscope | 10-200 Hz | 100 Hz |
| Magnetometer | 1-100 Hz | 100 Hz |
| Tap Detector | Event-based | 50 Hz |

**Rule of Thumb:** Call `update()` at 2-5x your fastest sensor rate.

## Troubleshooting

### **"My callback never gets called"**
```cpp
// Check this sequence:
bno08x.setCallback(MyCallback);           // 1. Set callback first
bno08x.enableSensor(sensor_type, rate);   // 2. Enable sensor
while (true) {
    bno08x.update();                      // 3. Call update() regularly
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

### **"Getting too much data"**
```cpp
// Filter high-frequency data in callback:
void FilteredCallback(const SensorEvent& event) {
    static uint32_t count = 0;
    count++;
    
    if (count % 10 == 0) {  // Only process every 10th sample
        ProcessSensorData(event);
    }
}
```

### **"I2C communication errors"**
- Check I2C wiring and pull-up resistors
- Verify BNO08x I2C address (usually 0x4A)
- Ensure power supply is stable (3.3V)
- Check if other I2C devices conflict

## Summary

1. **Use polling mode** - it's simpler and works great
2. **Call `bno08x.update()` regularly** - this is what triggers callbacks  
3. **No interrupt pin needed** - but you can add it for optimization
4. **Callbacks are normal functions** - not interrupt handlers
5. **Keep callbacks fast** - avoid blocking operations

The BNO08x driver handles all the complex I2C communication internally. Your job is just to call `update()` and process the callback data!
