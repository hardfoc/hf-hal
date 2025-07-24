# Enhanced MotorController: Board-Aware TMC9660 Device Management

## Implementation Summary

The MotorController has been completely redesigned to provide **board-aware TMC9660 device management** that understands the physical hardware layout and provides specialized handling for onboard vs. external devices.

## Key Features Implemented

### 1. **Board Layout Awareness**
- **Predefined device indices** corresponding to actual board CS lines
- **Index 0**: Onboard TMC9660 (SPI2_CS_TMC9660) - Always required, cannot be deleted
- **Index 2**: External Device 1 (EXT_GPIO_CS_1) - Dynamic creation/deletion  
- **Index 3**: External Device 2 (EXT_GPIO_CS_2) - Dynamic creation/deletion
- **Index 1**: Reserved for AS5047U encoder (not used for TMC9660)

### 2. **Device Management API**
```cpp
class MotorController {
public:
    // Board-aware constants
    static constexpr uint8_t ONBOARD_TMC9660_INDEX = 0;
    static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 2;  
    static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 3;
    
    // Core management methods
    bool Initialize();                                    // Initialize all active devices
    bool CreateExternalDevice(uint8_t csIndex, ...);     // Create external devices  
    bool DeleteExternalDevice(uint8_t csIndex);          // Delete external devices
    
    // Device validation and enumeration
    bool IsDeviceValid(uint8_t index) const;             // Check device existence
    bool IsExternalSlotAvailable(uint8_t csIndex) const; // Check slot availability
    std::vector<uint8_t> GetActiveDeviceIndices() const; // List active devices
    
    // Device access (defaults to onboard device for compatibility)
    Tmc9660Handler& handler(uint8_t index = ONBOARD_TMC9660_INDEX);
    std::shared_ptr<TMC9660> driver(uint8_t index = ONBOARD_TMC9660_INDEX);
    Tmc9660Handler::Gpio& gpio(uint8_t gpioNum, uint8_t index = ONBOARD_TMC9660_INDEX);
    Tmc9660Handler::Adc& adc(uint8_t index = ONBOARD_TMC9660_INDEX);
};
```

### 3. **Device Creation Patterns**

#### **Onboard Device (Manual Registration)**
```cpp
auto& motorController = MotorController::GetInstance();

// Create onboard device manually (index 0)
auto onboardDevice = std::make_unique<Tmc9660Handler>(spiInterface, 0x01);
uint8_t index = motorController.RegisterDevice(std::move(onboardDevice));
// index should be 0 for onboard device

motorController.Initialize(); // Initialize all devices
```

#### **External Device Creation**
```cpp
// Create external device on specific CS line
bool created = motorController.CreateExternalDevice(
    MotorController::EXTERNAL_DEVICE_1_INDEX,  // CS line 2 (EXT_GPIO_CS_1)
    spiInterface,                              // SPI or UART interface
    0x02                                       // Device address
);

// Device is automatically initialized if system is already initialized
```

### 4. **Safe Device Access**
```cpp
// Always validate before access
if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
    try {
        auto& gpio18 = motorController.gpio(18, MotorController::EXTERNAL_DEVICE_1_INDEX);
        auto& adc = motorController.adc(MotorController::EXTERNAL_DEVICE_1_INDEX);
        auto driver = motorController.driver(MotorController::EXTERNAL_DEVICE_1_INDEX);
    } catch (const std::exception& e) {
        // Handle device access errors
    }
}
```

### 5. **Device Enumeration**
```cpp
// Get all active devices
auto activeDevices = motorController.GetActiveDeviceIndices();
uint8_t deviceCount = motorController.GetDeviceCount();

// Check specific slots
bool onboardActive = motorController.IsDeviceValid(MotorController::ONBOARD_TMC9660_INDEX);
bool ext1Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
```

### 6. **External Device Deletion**
```cpp
// Delete external device when no longer needed
bool deleted = motorController.DeleteExternalDevice(MotorController::EXTERNAL_DEVICE_1_INDEX);
if (deleted) {
    // Device removed, slot now available for new device
    bool available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
}

// Cannot delete onboard device
bool cannotDelete = motorController.DeleteExternalDevice(MotorController::ONBOARD_TMC9660_INDEX);
// Returns false - onboard device cannot be deleted
```

## Architecture Benefits

### 1. **Hardware Compliance**
- ✅ **Matches physical board layout** with predefined CS pin assignments
- 🔒 **Prevents invalid device placement** on wrong CS lines
- 📍 **Clear mapping** between software indices and hardware connections

### 2. **Development Safety**
- 🛡️ **Cannot accidentally delete onboard device** (system protection)
- ✅ **Validation before device operations** (prevents crashes)
- 🎯 **Board-specific constants** for self-documenting code
- 🔍 **Clear error reporting** for invalid operations

### 3. **Operational Flexibility**
- 🔌 **Dynamic external device support** (plug-and-play)
- 🏭 **Runtime device configuration** (no recompilation needed)
- 🔄 **Hot-swappable external devices** (create/delete as needed)
- 📈 **Scalable for board variants** (easy to extend)

### 4. **Backward Compatibility**
- 🔄 **Legacy RegisterDevice() still works** (deprecated but functional)
- 🎯 **Default parameters access onboard device** (old code still works)
- 📚 **Gradual migration path** (no forced breaking changes)

## Implementation Details

### **Thread Safety**
- All methods are **thread-safe** with `std::mutex` protection
- Device creation/deletion is **atomic**
- Safe for **multi-threaded applications**

### **Error Handling**
- **Exception-based** error reporting for device access
- **Return value validation** for device creation/deletion
- **Validation methods** for safe operation

### **Memory Management**
- **RAII-compliant** with `std::unique_ptr` ownership
- **Automatic cleanup** when devices are deleted
- **No memory leaks** in device lifecycle

### **State Tracking**
```cpp
// Internal state arrays (thread-safe access)
std::array<std::unique_ptr<Tmc9660Handler>, MAX_TMC9660_DEVICES> tmcHandlers_;
std::array<bool, MAX_TMC9660_DEVICES> deviceInitialized_;  // Initialization status
std::array<bool, MAX_TMC9660_DEVICES> deviceActive_;       // Device presence
bool onboardDeviceCreated_;  // Onboard device creation tracking
bool systemInitialized_;     // System initialization status
```

## Usage Recommendations

### **For New Projects**
1. Use `CreateExternalDevice()` for external TMC9660 devices
2. Use board-defined constants (`ONBOARD_TMC9660_INDEX`, etc.)
3. Always validate device existence before access
4. Handle exceptions in device access code
5. Use device enumeration for dynamic discovery

### **For Existing Projects**
1. Legacy `RegisterDevice()` continues to work
2. Gradually migrate to board-aware constants
3. Add validation checks for enhanced safety
4. Consider using external device deletion for resource management

### **Best Practices**
1. **Always call Initialize()** after device registration
2. **Check IsDeviceValid()** before device access
3. **Use try-catch blocks** for device operations  
4. **Store device indices** returned by creation methods
5. **Clean up external devices** when no longer needed
6. **Use board constants** instead of magic numbers

## Files Modified

### **Core Implementation**
- `MotorController.h` - Enhanced header with board-aware API
- `MotorController.cpp` - Complete implementation rewrite

### **Documentation**
- `BOARD_AWARE_TMC9660_ARCHITECTURE.md` - Comprehensive architecture guide
- `SimpleBoardAwareTmc9660Example.cpp` - Working usage example

### **Examples**
- `BoardAwareTmc9660Example.cpp` - Advanced usage with CommChannelsManager
- `SimpleBoardAwareTmc9660Example.cpp` - Basic usage without dependencies

## Migration Path

**Phase 1**: Use existing code as-is (backward compatible)
**Phase 2**: Add device validation for enhanced safety  
**Phase 3**: Migrate to board-aware constants for clarity
**Phase 4**: Use CreateExternalDevice() for new external devices
**Phase 5**: Implement dynamic device management as needed

This implementation provides a **robust, safe, and flexible** foundation for TMC9660 device management while respecting the physical board layout and maintaining full backward compatibility.
