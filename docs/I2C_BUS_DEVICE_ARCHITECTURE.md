# I2C Bus-Device Architecture Migration Guide

## Overview

The ESP32 I2C implementation has been restructured to follow the modern ESP-IDF v5.5+ bus-device model, providing a clean separation between bus management and device handling. This new architecture matches the existing SPI implementation pattern and provides a consistent API across all communication channels.

## Key Changes

### Architecture

**Before (Old EspI2c):**
- Single class handling both bus and device operations
- Limited to one device per bus instance
- No clear separation between bus and device configuration
- Direct ESP-IDF driver calls mixed with abstraction

**After (New EspI2cBus/EspI2cDevice):**
- Clean separation: `EspI2cBus` manages the bus, `EspI2cDevice` represents individual devices
- Multiple devices per bus with individual configurations
- Follows ESP-IDF v5.5+ bus-device model using `i2c_new_master_bus()` and `i2c_master_bus_add_device()`
- Consistent with SPI implementation pattern

### API Changes

#### CommChannelsManager New Methods

**Bus Access:**
```cpp
EspI2cBus& GetI2cBus();  // Direct bus access
```

**Device Access by Index:**
```cpp
BaseI2c* GetI2cDevice(int device_index);
EspI2cDevice* GetEspI2cDevice(int device_index);
std::size_t GetI2cDeviceCount();
```

**Device Access by Enumeration (Type-Safe):**
```cpp
BaseI2c* GetI2cDevice(I2cDeviceId device_id);
EspI2cDevice* GetEspI2cDevice(I2cDeviceId device_id);
```

**Convenience Accessors:**
```cpp
BaseI2c* GetImu();                    // BNO08x IMU
BaseI2c* GetGpioExpander();           // PCAL95555 GPIO expander
EspI2cDevice* GetEspImu();            // ESP-specific IMU interface
EspI2cDevice* GetEspGpioExpander();   // ESP-specific GPIO expander interface
```

#### Device Enumeration

```cpp
enum class I2cDeviceId : uint8_t {
    BNO08X_IMU = 0,              // BNO08x IMU sensor (address 0x4A)
    PCAL9555_GPIO_EXPANDER = 1,  // PCAL9555 GPIO expander (address 0x20)
    EXTERNAL_DEVICE_1 = 2,       // External I2C device 1
    EXTERNAL_DEVICE_2 = 3,       // External I2C device 2
    
    // Aliases for common usage
    IMU = BNO08X_IMU,
    GPIO_EXPANDER = PCAL9555_GPIO_EXPANDER,
    
    I2C_DEVICE_COUNT             // Total number of I2C devices
};
```

## Usage Examples

### Basic Device Access

```cpp
// Get the communication manager
auto& comm_mgr = CommChannelsManager::GetInstance();
comm_mgr.EnsureInitialized();

// Method 1: Type-safe enumerated access (recommended)
BaseI2c* imu = comm_mgr.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
if (imu) {
    // Use the IMU device
    uint8_t data;
    imu->ReadRegister(0x00, data);
}

// Method 2: Convenience accessor
BaseI2c* imu_convenience = comm_mgr.GetImu();
if (imu_convenience) {
    // Same device as above, more convenient
}

// Method 3: Index-based access
BaseI2c* device0 = comm_mgr.GetI2cDevice(0);
if (device0) {
    // Access first device on the bus
}
```

### Advanced Bus Management

```cpp
// Get direct bus access for advanced operations
auto& i2c_bus = comm_mgr.GetI2cBus();

// Check device count
std::size_t device_count = i2c_bus.GetDeviceCount();

// Get ESP-specific device for advanced features
EspI2cDevice* esp_imu = comm_mgr.GetEspImu();
if (esp_imu) {
    // Access ESP-specific features like advanced timeouts
}
```

### Legacy Compatibility

```cpp
// Old API (deprecated but still supported)
BaseI2c& legacy_i2c = comm_mgr.GetI2c(0);  // Returns first device
std::size_t count = comm_mgr.GetI2cCount();

// New API (recommended)
BaseI2c* new_i2c = comm_mgr.GetI2cDevice(0);
std::size_t new_count = comm_mgr.GetI2cDeviceCount();
```

## Configuration

### Device Configuration

Each I2C device can have its own configuration:

```cpp
hf_i2c_device_config_t device_config = {
    .device_address = 0x4A,                                    // Device I2C address
    .dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT,  // 7-bit addressing
    .scl_speed_hz = 400000,                                    // 400kHz operation
    .scl_wait_us = 0,                                          // No additional wait time
    .disable_ack_check = false,                                // Enable ACK checking
    .flags = 0                                                 // No additional flags
};
```

### Bus Configuration

Single bus configuration shared by all devices:

```cpp
hf_i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,                                     // I2C port
    .sda_io_num = sda_pin,                                     // SDA GPIO pin
    .scl_io_num = scl_pin,                                     // SCL GPIO pin
    .enable_internal_pullup = true,                            // Enable internal pullups
    .clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT, // Default clock
    .glitch_ignore_cnt = hf_i2c_glitch_filter_t::HF_I2C_GLITCH_FILTER_7_CYCLES, // Glitch filter
    .trans_queue_depth = 8,                                    // Transaction queue depth
    .intr_priority = 5,                                        // Interrupt priority
    .flags = 0,                                                // No additional flags
    .allow_pd = false                                          // No power down in sleep
};
```

## Migration Guide

### For Application Code

1. **Update includes:** No changes needed, continue using `CommChannelsManager.h`

2. **Update device access:**
   ```cpp
   // Old way
   BaseI2c& i2c = comm_mgr.GetI2c(0);
   
   // New way (recommended)
   BaseI2c* imu = comm_mgr.GetImu();
   // or
   BaseI2c* imu = comm_mgr.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
   ```

3. **Handle null pointers:** New API returns pointers that can be null
   ```cpp
   BaseI2c* device = comm_mgr.GetImu();
   if (device) {
       // Use device
   } else {
       // Handle error
   }
   ```

### For Driver Code

1. **Update to new architecture:** If you were using `EspI2c` directly, update to use `EspI2cBus`/`EspI2cDevice`

2. **Device-specific configuration:** Each device can now have its own configuration

3. **Error handling:** New architecture provides better error handling and reporting

## Benefits

### 1. **Scalability**
- Multiple devices per bus with individual configurations
- Easy to add new devices without changing existing code

### 2. **Type Safety**
- Enumerated device access prevents errors
- Compile-time checking of device types

### 3. **Consistency**
- Same pattern as SPI implementation
- Consistent API across all communication channels

### 4. **Modern ESP-IDF**
- Uses ESP-IDF v5.5+ bus-device model
- Better performance and resource management

### 5. **Maintainability**
- Clear separation of concerns
- Easier to debug and maintain

## Hardware Support

Currently configured devices:
- **BNO08x IMU:** I2C address 0x4A, 400kHz operation
- **PCAL95555 GPIO Expander:** I2C address 0x20, 400kHz operation
- **External Device Slots:** 2 additional device slots available

## Files Modified/Created

### New Files
- `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c_NEW.h`
- `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspI2c_NEW.cpp`
- `examples/I2cBusDeviceExample.cpp`
- `tests/component-handler/I2cBusDeviceTest.cpp`

### Modified Files
- `component-handlers/CommChannelsManager.h`
- `component-handlers/CommChannelsManager.cpp`
- `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/utils/EspTypes_I2C.h`
- `component-handlers/CMakeLists.txt`

### Legacy Files (Deprecated)
- `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h`
- `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspI2c.cpp`

## Testing

### Example Code
See `examples/I2cBusDeviceExample.cpp` for comprehensive usage examples.

### Unit Tests
See `tests/component-handler/I2cBusDeviceTest.cpp` for unit tests covering:
- Initialization and configuration
- Device access patterns
- API consistency
- Error handling
- Legacy compatibility

### Integration Tests
Existing examples like `I2cBno08xExample.cpp` continue to work through the ImuManager layer.

## Future Enhancements

1. **Additional Devices:** Easy to add more I2C devices by extending the enumeration
2. **Advanced Features:** Per-device timeout configuration, interrupt handling
3. **Power Management:** Integration with ESP32 power management features
4. **Diagnostics:** Built-in I2C bus scanning and device detection

## Troubleshooting

### Common Issues

1. **Device not found:** Check I2C address and wiring
2. **Initialization failed:** Verify GPIO pin configuration
3. **Communication errors:** Check pull-up resistors and bus speed
4. **Null pointer access:** Always check device pointer before use

### Debug Tips

1. Enable I2C logging: `esp_log_level_set("i2c", ESP_LOG_DEBUG)`
2. Check device count: `comm_mgr.GetI2cDeviceCount()`
3. Verify bus initialization: `comm_mgr.IsInitialized()`
4. Test with simple read/write operations first

This new architecture provides a solid foundation for scalable I2C communication in the HardFOC system while maintaining backward compatibility and following modern ESP-IDF best practices.
