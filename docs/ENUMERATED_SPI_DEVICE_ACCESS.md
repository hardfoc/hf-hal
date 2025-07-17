# Enumeration-Based SPI Device Access

This document describes the new enumeration-based SPI device access system in CommChannelsManager, which provides a type-safe and intuitive way to access specific SPI devices by their functional purpose.

## Overview

Instead of using numeric indices to access SPI devices, you can now use meaningful enumeration values and convenience methods. This approach provides:

- **Type Safety**: Compile-time checking of device identifiers
- **Code Clarity**: Self-documenting device access
- **Maintainability**: Changes to device order don't break existing code
- **IntelliSense Support**: Better autocomplete and documentation

## SpiDeviceId Enumeration

```cpp
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0,  ///< TMC9660 motor controller (SPI Mode 3)
    AS5047U_POSITION_ENCODER = 1,  ///< AS5047U position encoder (SPI Mode 1)
    EXTERNAL_DEVICE_1 = 2,         ///< External device 1 (SPI Mode 0)
    EXTERNAL_DEVICE_2 = 3,         ///< External device 2 (SPI Mode 0)
    
    // Aliases for common usage
    MOTOR_CONTROLLER = TMC9660_MOTOR_CONTROLLER,
    POSITION_ENCODER = AS5047U_POSITION_ENCODER,
    
    SPI_DEVICE_COUNT  ///< Total number of SPI devices
};
```

## Device Mapping

The enumeration values map directly to the CS pin configuration:

| Device ID | CS Pin | Physical Pin | SPI Mode | Purpose |
|-----------|--------|--------------|----------|---------|
| `TMC9660_MOTOR_CONTROLLER` | `SPI2_CS_TMC9660` | IO18 | Mode 3 | Motor controller |
| `AS5047U_POSITION_ENCODER` | `SPI2_CS_AS5047` | IO20 | Mode 1 | Position encoder |
| `EXTERNAL_DEVICE_1` | `EXT_GPIO_CS_1` | IO19 | Mode 0 | External device |
| `EXTERNAL_DEVICE_2` | `EXT_GPIO_CS_2` | IO8 | Mode 0 | External device |

## Access Methods

### 1. Enumeration-Based Access

```cpp
auto& comm_mgr = CommChannelsManager::GetInstance();

// Access devices using SpiDeviceId enumeration
BaseSpi* motor = comm_mgr.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
BaseSpi* encoder = comm_mgr.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
EspSpiDevice* esp_motor = comm_mgr.GetEspSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
```

### 2. Convenience Methods (Recommended)

```cpp
// Convenient named accessors
BaseSpi* motor = comm_mgr.GetMotorController();
BaseSpi* encoder = comm_mgr.GetPositionEncoder();
BaseSpi* ext1 = comm_mgr.GetExternalDevice1();
BaseSpi* ext2 = comm_mgr.GetExternalDevice2();

// ESP-specific interfaces
EspSpiDevice* esp_motor = comm_mgr.GetEspMotorController();
EspSpiDevice* esp_encoder = comm_mgr.GetEspPositionEncoder();
```

### 3. Traditional Index-Based Access (Still Supported)

```cpp
// Legacy numeric access still works
BaseSpi* motor = comm_mgr.GetSpiDevice(0); // TMC9660
BaseSpi* encoder = comm_mgr.GetSpiDevice(1); // AS5047U
```

## Usage Examples

### TMC9660 Motor Controller

```cpp
if (auto* motor = comm_mgr.GetMotorController()) {
    // TMC9660 uses SPI Mode 3 and 32-bit register access
    std::array<uint8_t, 5> read_gconf = {0x00, 0x00, 0x00, 0x00, 0x00};
    std::array<uint8_t, 5> response = {};
    
    auto result = motor->Transfer(read_gconf.data(), response.data(), 5);
    if (result == hf_spi_err_t::HF_SPI_OK) {
        // Process TMC9660 GCONF register value
        uint32_t gconf_value = (response[1] << 24) | (response[2] << 16) | 
                              (response[3] << 8) | response[4];
    }
}
```

### AS5047U Position Encoder

```cpp
if (auto* encoder = comm_mgr.GetPositionEncoder()) {
    // AS5047U uses SPI Mode 1 and 16-bit commands
    std::array<uint8_t, 2> read_angle = {0x3F, 0xFF}; // Angle register with parity
    std::array<uint8_t, 2> angle_data = {};
    
    auto result = encoder->Transfer(read_angle.data(), angle_data.data(), 2);
    if (result == hf_spi_err_t::HF_SPI_OK) {
        // Extract 14-bit angle (ignore parity bit)
        uint16_t raw_angle = ((angle_data[0] & 0x3F) << 8) | angle_data[1];
        float angle_degrees = (raw_angle * 360.0f) / 16384.0f;
    }
}
```

### ESP-Specific Features

```cpp
if (auto* esp_motor = comm_mgr.GetEspMotorController()) {
    // Use ESP-specific bus acquisition for multiple transactions
    if (esp_motor->AcquireBus(1000) == hf_spi_err_t::HF_SPI_OK) {
        // Perform multiple operations without bus overhead
        // ... multiple transfers ...
        esp_motor->ReleaseBus();
    }
    
    // Get actual clock frequency for diagnostics
    uint32_t actual_freq = 0;
    if (esp_motor->GetActualClockFrequency(actual_freq) == hf_spi_err_t::HF_SPI_OK) {
        // Log or validate clock frequency
    }
}
```

### Device Iteration

```cpp
// Iterate through all defined devices
for (int i = 0; i < static_cast<int>(SpiDeviceId::SPI_DEVICE_COUNT); ++i) {
    SpiDeviceId device_id = static_cast<SpiDeviceId>(i);
    if (auto* device = comm_mgr.GetSpiDevice(device_id)) {
        // Device is available
        switch (device_id) {
            case SpiDeviceId::TMC9660_MOTOR_CONTROLLER:
                // Handle motor controller
                break;
            case SpiDeviceId::AS5047U_POSITION_ENCODER:
                // Handle position encoder
                break;
            // ... other cases
        }
    }
}
```

## Error Handling

All access methods return `nullptr` if:
- The CommChannelsManager is not initialized
- The SPI bus is not available
- The requested device index/ID is invalid
- Device creation failed during initialization

Always check for `nullptr` before using the returned device pointers:

```cpp
auto* device = comm_mgr.GetMotorController();
if (device) {
    // Safe to use device
    device->Transfer(/* ... */);
} else {
    // Handle device not available
}
```

## Device-Specific SPI Modes

The system automatically configures the correct SPI mode for each device:

- **TMC9660**: Mode 3 (CPOL=1, CPHA=1) - Industrial motor controller standard
- **AS5047U**: Mode 1 (CPOL=0, CPHA=1) - Position encoder requirement
- **External Devices**: Mode 0 (CPOL=0, CPHA=0) - Default for general purposes

## Migration from Index-Based Access

If you have existing code using numeric indices:

```cpp
// Old approach
BaseSpi* motor = comm_mgr.GetSpiDevice(0);
BaseSpi* encoder = comm_mgr.GetSpiDevice(1);

// New approach (recommended)
BaseSpi* motor = comm_mgr.GetMotorController();
BaseSpi* encoder = comm_mgr.GetPositionEncoder();

// Or using enums
BaseSpi* motor = comm_mgr.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
BaseSpi* encoder = comm_mgr.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
```

## Benefits

1. **Maintainability**: Device order changes don't break code
2. **Readability**: Clear intent with named accessors
3. **Type Safety**: Compile-time validation of device IDs
4. **Documentation**: Self-documenting device purposes
5. **IDE Support**: Better autocomplete and IntelliSense
6. **Backward Compatibility**: Existing index-based code continues to work

## Thread Safety

All access methods are thread-safe and can be called from multiple tasks simultaneously. The underlying EspSpiBus and EspSpiDevice implementations use RTOS mutexes for protection.
