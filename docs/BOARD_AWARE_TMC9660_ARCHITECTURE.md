# Board-Aware TMC9660 MotorController Architecture

## Overview

The MotorController has been redesigned to provide **board-aware TMC9660 device management**. This means the controller understands the physical board layout and automatically manages device creation, initialization, and deletion based on the hardware configuration.

## Board Configuration

### Hardware Layout
The system supports up to **4 SPI device slots** with predefined purposes:

| Index | CS Pin | Device Type | Management | Description |
|-------|--------|-------------|------------|-------------|
| **0** | `SPI2_CS_TMC9660` | **Onboard TMC9660** | **Required** | Main motor controller (always present) |
| **1** | `SPI2_CS_AS5047` | **AS5047U Encoder** | N/A | Position encoder (not used for TMC9660) |
| **2** | `EXT_GPIO_CS_1` | **External TMC9660** | **Dynamic** | External motor controller 1 (optional) |
| **3** | `EXT_GPIO_CS_2` | **External TMC9660** | **Dynamic** | External motor controller 2 (optional) |

### Device Management Rules

1. **Onboard TMC9660 (Index 0)**:
   - ✅ **Always created** during `Initialize()`
   - ❌ **Cannot be deleted** (permanent system component)
   - 🔒 **Automatically configured** with board-specific SPI interface

2. **External TMC9660 Devices (Indices 2-3)**:
   - ✅ **Dynamic creation** via `CreateExternalDevice()`
   - ✅ **Dynamic deletion** via `DeleteExternalDevice()`
   - 🔧 **User-configurable** SPI or UART interfaces
   - 🎯 **Specific CS line assignment** (no arbitrary placement)

## New API Methods

### Device Creation and Management

```cpp
class MotorController {
public:
    // Constants for board-aware indexing
    static constexpr uint8_t ONBOARD_TMC9660_INDEX = 0;
    static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 2;  
    static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 3;
    
    // Initialization (creates onboard device automatically)
    bool Initialize();
    
    // External device creation (SPI)
    bool CreateExternalDevice(uint8_t csDeviceIndex, 
                            BaseSpi& spiInterface, 
                            uint8_t address,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr);
    
    // External device creation (UART)
    bool CreateExternalDevice(uint8_t csDeviceIndex,
                            BaseUart& uartInterface,
                            uint8_t address, 
                            const tmc9660::BootloaderConfig* bootCfg = nullptr);
    
    // External device deletion
    bool DeleteExternalDevice(uint8_t csDeviceIndex);
    
    // Device validation and enumeration
    bool IsDeviceValid(uint8_t deviceIndex) const noexcept;
    bool IsExternalSlotAvailable(uint8_t csDeviceIndex) const noexcept;
    std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;
};
```

## Usage Patterns

### 1. Basic System Initialization

```cpp
auto& motorController = MotorController::GetInstance();

// Initialize system (automatically creates onboard TMC9660)
bool success = motorController.Initialize();
if (success) {
    // Onboard TMC9660 is now available at index 0
    auto& onboardGpio = motorController.gpio(17, MotorController::ONBOARD_TMC9660_INDEX);
    auto& onboardAdc = motorController.adc(MotorController::ONBOARD_TMC9660_INDEX);
}
```

### 2. External Device Creation

```cpp
auto& commManager = CommChannelsManager::GetInstance();

// Create external TMC9660 on EXT_GPIO_CS_1 (index 2)
BaseSpi* spiExt1 = commManager.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_1);
bool created = motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX,
    *spiExt1,
    0x02  // Device address
);

if (created) {
    // External device is now available at index 2
    auto& extGpio = motorController.gpio(18, MotorController::EXTERNAL_DEVICE_1_INDEX);
    auto driver = motorController.driver(MotorController::EXTERNAL_DEVICE_1_INDEX);
}
```

### 3. Device Enumeration and Validation

```cpp
// Check which devices are active
auto activeDevices = motorController.GetActiveDeviceIndices();
for (auto idx : activeDevices) {
    if (motorController.IsDeviceValid(idx)) {
        auto& handler = motorController.handler(idx);
        // Use handler...
    }
}

// Check specific device availability
bool onboardReady = motorController.IsDeviceValid(MotorController::ONBOARD_TMC9660_INDEX);
bool ext1Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
```

### 4. Safe Device Deletion

```cpp
// Delete external device when no longer needed
if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
    bool deleted = motorController.DeleteExternalDevice(MotorController::EXTERNAL_DEVICE_1_INDEX);
    if (deleted) {
        // Device successfully removed, slot is now available
        bool slotAvailable = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
    }
}

// Attempting to delete onboard device (will fail)
bool cannotDelete = motorController.DeleteExternalDevice(MotorController::ONBOARD_TMC9660_INDEX);
// cannotDelete == false (onboard device cannot be deleted)
```

## Backward Compatibility

### Legacy RegisterDevice Method
The old `RegisterDevice()` method is still supported but deprecated:

```cpp
// Legacy method (still works, but not recommended)
auto handler = std::make_unique<Tmc9660Handler>(spi, address);
uint8_t index = motorController.RegisterDevice(std::move(handler));

// New method (preferred)
bool created = motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX, spi, address);
```

### Default Parameter Behavior
All access methods default to the onboard device for backward compatibility:

```cpp
// These are equivalent
auto& gpio17_old = motorController.gpio(17);  // Defaults to index 0
auto& gpio17_new = motorController.gpio(17, MotorController::ONBOARD_TMC9660_INDEX);

auto& adc_old = motorController.adc();  // Defaults to index 0  
auto& adc_new = motorController.adc(MotorController::ONBOARD_TMC9660_INDEX);
```

## Error Handling

### Device Creation Errors
```cpp
bool created = motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX, spi, address);

if (!created) {
    // Possible reasons:
    // - Invalid CS device index (not 2 or 3)
    // - Device slot already occupied
    // - Invalid SPI/UART interface
    // - Handler creation failed
}
```

### Device Access Errors
```cpp
try {
    auto& handler = motorController.handler(deviceIndex);
    auto driver = motorController.driver(deviceIndex);
} catch (const std::out_of_range& e) {
    // Invalid or inactive device index
} catch (const std::runtime_error& e) {
    // Device not initialized or driver unavailable
}
```

### Validation Before Access
```cpp
// Always validate before accessing
if (motorController.IsDeviceValid(deviceIndex)) {
    try {
        auto& handler = motorController.handler(deviceIndex);
        // Safe to use handler
    } catch (const std::exception& e) {
        // Handle unexpected errors
    }
}
```

## Thread Safety

- ✅ All methods are **thread-safe** with mutex protection
- ✅ Device creation/deletion is **atomic**
- ✅ Device access is **protected** against concurrent modifications
- ✅ Safe for **multi-threaded applications**

## Migration Guide

### From Generic Multi-Device to Board-Aware

**Old approach (generic):**
```cpp
// Register devices without board awareness
auto device0 = std::make_unique<Tmc9660Handler>(spi1, 0x01);
auto device1 = std::make_unique<Tmc9660Handler>(spi2, 0x02);
uint8_t idx0 = motorController.RegisterDevice(std::move(device0));
uint8_t idx1 = motorController.RegisterDevice(std::move(device1));
```

**New approach (board-aware):**
```cpp
// Initialize system (creates onboard device automatically)
motorController.Initialize();

// Create external devices on specific CS lines
motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX, spi2, 0x02);

// Access devices by board-defined indices
auto& onboardGpio = motorController.gpio(17, MotorController::ONBOARD_TMC9660_INDEX);
auto& externalGpio = motorController.gpio(18, MotorController::EXTERNAL_DEVICE_1_INDEX);
```

## Benefits

### 1. **Board Layout Awareness**
- 🎯 Predefined CS pin assignments match hardware
- 🔒 Prevents invalid device placement
- 📍 Clear mapping between indices and physical connections

### 2. **Simplified Device Management**
- 🚀 Automatic onboard device creation
- 🔧 Easy external device addition/removal
- 🎛️ No manual CS pin configuration required

### 3. **Enhanced Safety**
- 🛡️ Cannot accidentally delete onboard device
- ✅ Validation before device operations
- 🔍 Clear error reporting for invalid operations

### 4. **Development Efficiency**
- 📚 Self-documenting device indices
- 🎯 Board-specific constants for clear code
- 🔄 Easy device enumeration and status checking

### 5. **Production Flexibility**
- 🔌 Dynamic external device support
- 🏭 Runtime device configuration
- 📈 Scalable for different board variants

## Best Practices

1. **Always call Initialize() first** to create the onboard device
2. **Use board-defined constants** (`ONBOARD_TMC9660_INDEX`, `EXTERNAL_DEVICE_1_INDEX`, etc.)
3. **Validate device availability** before creation/access operations
4. **Handle exceptions** when accessing devices
5. **Check initialization status** after device creation
6. **Use CreateExternalDevice()** instead of legacy RegisterDevice()
7. **Clean up external devices** when no longer needed

This board-aware architecture provides a robust, safe, and efficient foundation for TMC9660 device management while maintaining backward compatibility and supporting the specific hardware layout of your board.
