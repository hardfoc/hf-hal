# **CommChannelsManager Architecture Documentation**

## **📋 Table of Contents**

1. [Overview](#overview)
2. [Architecture Design](#architecture-design)
3. [Bus Model](#bus-model)
4. [Device Management](#device-management)
5. [Component Handler Integration](#component-handler-integration)
6. [Pin Configuration System](#pin-configuration-system)
7. [External Device Registration](#external-device-registration)
8. [Error Handling](#error-handling)
9. [Thread Safety](#thread-safety)
10. [Usage Examples](#usage-examples)
11. [Best Practices](#best-practices)
12. [Future Extensibility](#future-extensibility)

---

## **🎯 Overview**

The `CommChannelsManager` is the **central communication hub** of the HardFOC system, providing a unified interface for managing all communication channels (SPI, I2C, UART, CAN) across the platform. It implements a sophisticated **multi-bus, multi-device architecture** that supports both built-in and external devices.

### **🔑 Key Features**

- **🔄 Multi-Bus Support**: Built-in buses (Index 0) and external buses (Index 10+)
- **🔧 Runtime Device Management**: Create, register, and remove devices at runtime
- **🎯 Device ID Abstraction**: Type-safe access to known devices via enums
- **🔗 Pin-to-Interface Mapping**: Functional pin names map to physical hardware
- **🛡️ Exception-Free Design**: All operations return `bool` or error codes
- **🔒 Thread Safety**: Protected by `RtosMutex` for multi-threaded access

---

## **🏗️ Architecture Design**

### **📊 System Layers**

```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                        │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ ImuManager  │ │MotorController│ │ GpioManager │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                 COMMCHANNELSMANAGER LAYER                   │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ Bus Index 0 │ │ Bus Index 10+│ │ Custom Buses│           │
│  │ (Built-in)  │ │ (External)  │ │ (User Reg.) │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    INTERFACE ABSTRACTION                    │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │   BaseSpi   │ │   BaseI2c   │ │   BaseUart  │           │
│  │   BaseCan   │ │             │ │             │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   MCU IMPLEMENTATION LAYER                  │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │  EspSpiBus  │ │  EspI2cBus  │ │  EspUart    │           │
│  │ EspSpiDevice│ │ EspI2cDevice│ │  EspCan     │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    PIN CONFIGURATION LAYER                  │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │         hf_functional_pin_config_vortex_v1.hpp         │ │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐       │ │
│  │  │ GPIO Pins   │ │ SPI Pins    │ │ I2C Pins    │       │ │
│  │  │ ADC Channels│ │ UART Pins   │ │ CAN Pins    │       │ │
│  │  └─────────────┘ └─────────────┘ └─────────────┘       │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### **🔧 Singleton Pattern**

```cpp
class CommChannelsManager {
public:
    static CommChannelsManager& GetInstance() noexcept;
    
    // Non-copyable, non-movable
    CommChannelsManager(const CommChannelsManager&) = delete;
    CommChannelsManager& operator=(const CommChannelsManager&) = delete;
    CommChannelsManager(CommChannelsManager&&) = delete;
    CommChannelsManager& operator=(CommChannelsManager&&) = delete;
};
```

---

## **🚌 Bus Model**

### **📊 Bus Indexing Scheme**

The system uses a **hierarchical bus indexing scheme**:

```cpp
// Built-in Buses (Index 0)
Bus 0: ESP32 SPI Bus (spi_bus_)
  ├── Device 0: TMC9660_MOTOR_CONTROLLER (CS: SPI2_CS_TMC9660)
  ├── Device 1: AS5047U_POSITION_ENCODER (CS: SPI2_CS_AS5047)
  ├── Device 2: EXTERNAL_GPIO_CS_1 (CS: EXT_GPIO_CS_1)
  └── Device 3: EXTERNAL_GPIO_CS_2 (CS: EXT_GPIO_CS_2)

Bus 0: ESP32 I2C Bus (i2c_bus_)
  ├── Device 0: BNO08X_IMU (Address: 0x4A)
  └── Device 1: PCAL9555_GPIO_EXPANDER (Address: 0x20)

// External Buses (Index 10+)
Bus 10+: User-registered custom buses
  ├── Custom SPI devices (custom_spi_devices_)
  └── Custom I2C devices (custom_i2c_devices_)
```

### **🔗 Device Storage Structures**

```cpp
// Built-in Device Tracking
std::vector<int> spi_device_indices_;           // Built-in SPI device indices
std::vector<int> i2c_device_indices_;           // Built-in I2C device indices
std::vector<uint8_t> i2c_device_addresses_;     // Built-in I2C addresses

// External Device Tracking
std::map<int, std::shared_ptr<BaseSpi>> custom_spi_devices_;    // device_index -> BaseSpi
std::map<int, std::shared_ptr<BaseI2c>> custom_i2c_devices_;    // device_index -> BaseI2c

// Bus Association Mapping
std::map<int, uint8_t> spi_device_to_bus_mapping_;    // device_index -> bus_index
std::map<int, uint8_t> i2c_device_to_bus_mapping_;    // device_index -> bus_index

// External Bus Tracking
std::set<uint8_t> external_spi_buses_;    // External SPI bus indices (10+)
std::set<uint8_t> external_i2c_buses_;    // External I2C bus indices (10+)
```

---

## **🔧 Device Management**

### **🎯 Device ID Enums**

```cpp
// SPI Device IDs (map to Bus 0, device indices 0-3)
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0,  // Maps to Bus 0, Device 0
    AS5047U_POSITION_ENCODER = 1,  // Maps to Bus 0, Device 1
    EXTERNAL_GPIO_CS_1 = 2,        // Maps to Bus 0, Device 2
    EXTERNAL_GPIO_CS_2 = 3,        // Maps to Bus 0, Device 3
    
    // Aliases for common usage
    MOTOR_CONTROLLER = TMC9660_MOTOR_CONTROLLER,
    POSITION_ENCODER = AS5047U_POSITION_ENCODER,
    TMC9660_SPI = TMC9660_MOTOR_CONTROLLER,
};

// I2C Device IDs (map to Bus 0, device indices 0-1)
enum class I2cDeviceId : uint8_t {
    BNO08X_IMU = 0,              // Maps to Bus 0, Device 0 (Address 0x4A)
    PCAL9555_GPIO_EXPANDER = 1,  // Maps to Bus 0, Device 1 (Address 0x20)
    
    // Aliases for common usage
    IMU = BNO08X_IMU,
    GPIO_EXPANDER = PCAL9555_GPIO_EXPANDER,
};
```

### **🔍 Device Access Patterns**

```cpp
// Method 1: Device ID Access (Recommended for known devices)
BaseSpi* spi = comm_manager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
BaseI2c* i2c = comm_manager.GetI2cDevice(I2cDeviceId::BNO08X_IMU);

// Method 2: Bus-Aware Access (For external or runtime devices)
BaseSpi* spi = comm_manager.GetSpiDevice(0, 0);  // Bus 0, Device 0
BaseI2c* i2c = comm_manager.GetI2cDevice(0, 0);  // Bus 0, Device 0

// Method 3: External Bus Access
BaseSpi* custom_spi = comm_manager.GetSpiDevice(10, device_index);
BaseI2c* custom_i2c = comm_manager.GetI2cDevice(10, device_index);
```

---

## **🤖 Component Handler Integration**

### **🔄 Integration Patterns**

Component handlers use `CommChannelsManager` to access communication interfaces for their device-specific handlers:

```cpp
// Pattern: Get Interface → Create Handler → Manage Lifecycle
class ComponentHandler {
private:
    CommChannelsManager* comm_manager_;
    std::unique_ptr<DeviceHandler> device_handler_;
    
public:
    bool Initialize() {
        // 1. Get communication interface
        BaseSpi* spi_interface = comm_manager_->GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
        
        // 2. Create device-specific handler
        device_handler_ = std::make_unique<DeviceHandler>(*spi_interface, config);
        
        // 3. Initialize the handler
        return device_handler_->Initialize();
    }
};
```

### **📋 Component Handler Examples**

#### **ImuManager Integration**

```cpp
bool ImuManager::InitializeOnboardBno08xDevice() {
    // 1. Get I2C device from CommChannelsManager
    BaseI2c* imu_device = comm_manager_->GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!imu_device) {
        return false;
    }
    
    // 2. Create BNO08x handler with I2C interface
    bno08x_handlers_[ONBOARD_IMU_INDEX] = std::make_unique<Bno08xHandler>(*imu_device, config);
    
    // 3. Initialize the handler
    Bno08xError init_result = bno08x_handlers_[ONBOARD_IMU_INDEX]->Initialize();
    return init_result == Bno08xError::SUCCESS;
}
```

#### **MotorController Integration**

```cpp
bool MotorController::Initialize() {
    // 1. Get SPI device from CommChannelsManager
    BaseSpi* spiInterface = commManager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spiInterface) {
        return false;
    }
    
    // 2. Create TMC9660 handler with SPI interface
    auto onboardHandler = std::make_unique<Tmc9660Handler>(*spiInterface, 0x01, &kDefaultBootConfig);
    
    // 3. Store handler and mark as active
    tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
    deviceActive_[ONBOARD_TMC9660_INDEX] = true;
    
    return true;
}
```

#### **GpioManager Integration**

```cpp
bool GpioManager::EnsurePcal95555Handler() {
    // 1. Get I2C device from CommChannelsManager
    auto i2c_device = comm_manager.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c_device) {
        return false;
    }
    
    // 2. Create PCAL95555 handler with I2C interface
    pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_device);
    
    // 3. Initialize the handler
    return pcal95555_handler_->Initialize();
}
```

---

## **🔗 Pin Configuration System**

### **📌 Functional Pin Mapping**

The system uses a **XMACRO-based configuration** in `hf_functional_pin_config_vortex_v1.hpp`:

```cpp
// Example: SPI2_CS_TMC9660 pin definition
X(SPI2_CS_TMC9660, "COMM_SPI2_CS_TMC9660", PIN_CATEGORY_COMM, ESP32_INTERNAL, 18, 0, true, true, true, true, 40)

// Breakdown:
// - SPI2_CS_TMC9660: Functional pin enum
// - "COMM_SPI2_CS_TMC9660": Human-readable name
// - PIN_CATEGORY_COMM: Pin category (communication)
// - ESP32_INTERNAL: Chip type
// - 18: Physical GPIO pin number
// - 0: Unit number
// - true: Inverted logic
// - true: Has pull resistor
// - true: Pull-up
// - true: Push-pull output
// - 40: Max current (mA)
```

### **🎯 Pin Categories**

```cpp
enum class HfPinCategory : uint8_t {
    PIN_CATEGORY_CORE = 0,    // System/core pins (skip GPIO registration)
    PIN_CATEGORY_COMM = 1,    // Communication pins (skip GPIO registration)  
    PIN_CATEGORY_GPIO = 2,    // Available for GPIO operations
    PIN_CATEGORY_USER = 3     // User-defined pins (always register)
};
```

### **🔧 Pin-to-Interface Mapping Flow**

```cpp
// 1. Pin Configuration → Physical Pin Number
auto* cs_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_CS_TMC9660);
// Returns: physical_pin = 18

// 2. Physical Pin → Device Configuration
hf_spi_device_config_t dev_cfg = {};
dev_cfg.cs_pin = cs_map->physical_pin;  // 18
dev_cfg.clock_speed_hz = 10 * 1000000;  // 10 MHz
dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_3;  // TMC9660 mode

// 3. Device Configuration → ESP32 Device
int device_index = spi_bus_->CreateDevice(dev_cfg);

// 4. Device Index → Bus Mapping
spi_device_to_bus_mapping_[device_index] = 0;  // Bus 0
```

---

## **🔧 External Device Registration**

### **📝 Custom Device Registration Patterns**

```cpp
// Pattern 1: Register Custom SPI Device
class MyCustomSpiDevice : public BaseSpi {
    // Implementation of BaseSpi interface
    hf_spi_err_t Transfer(const hf_u8_t* tx_data, hf_u8_t* rx_data, hf_u16_t length, hf_u32_t timeout_ms = 0) noexcept override {
        // Custom SPI implementation
        return hf_spi_err_t::SPI_SUCCESS;
    }
    
    bool Initialize() noexcept override {
        // Custom initialization
        return true;
    }
    
    bool Deinitialize() noexcept override {
        // Custom cleanup
        return true;
    }
};

// Register with CommChannelsManager
auto& comm_manager = CommChannelsManager::GetInstance();
int device_index = comm_manager.RegisterCustomSpiDevice(
    std::make_shared<MyCustomSpiDevice>(),  // Custom device
    -1,                                     // Auto-assign device index
    10                                      // Bus index 10 (external)
);

// Access the custom device
BaseSpi* custom_spi = comm_manager.GetSpiDevice(10, device_index);
```

### **🔗 Runtime I2C Device Creation**

```cpp
// Create I2C device at runtime
auto& comm_manager = CommChannelsManager::GetInstance();

// Create device at address 0x48 with 400kHz speed
int device_index = comm_manager.CreateI2cDevice(0x48, 400000);

if (device_index >= 0) {
    // Get the I2C interface for the new device
    BaseI2c* i2c_interface = comm_manager.GetI2cDevice(0, device_index);
    
    // Use the interface with your device driver
    MyDeviceDriver driver(*i2c_interface);
    driver.Initialize();
    
    // Clean up when done
    comm_manager.RemoveI2cDevice(device_index);
}
```

### **🔄 Device Lifecycle Management**

```cpp
// 1. Device Creation
int device_index = comm_manager.CreateI2cDevice(0x48, 400000);

// 2. Device Access
BaseI2c* device = comm_manager.GetI2cDevice(0, device_index);

// 3. Device Usage
MyDeviceDriver driver(*device);
driver.Initialize();
driver.PerformOperations();

// 4. Device Cleanup
comm_manager.RemoveI2cDevice(device_index);
```

---

## **🛡️ Error Handling**

### **✅ Exception-Free Design Patterns**

```cpp
// Method 1: Boolean Return with Pointer Output
bool GetMcuI2cBus(uint8_t bus_index, EspI2cBus*& bus) noexcept {
    if (bus_index == 0 && i2c_bus_) {
        bus = i2c_bus_.get();
        return true;
    }
    bus = nullptr;
    return false;
}

// Method 2: Integer Return with Error Codes
int CreateI2cDevice(uint8_t device_address, uint32_t speed_hz) noexcept {
    if (!i2c_bus_) {
        return -1;  // Error: bus not available
    }
    
    if (HasI2cDeviceAtAddress(0, device_address)) {
        return GetI2cDeviceIndexByAddress(device_address);  // Return existing device
    }
    
    // Success: return device index
    return device_index;
}

// Method 3: Null Pointer Return
BaseSpi* GetSpiDevice(SpiDeviceId device_id) noexcept {
    switch (device_id) {
        case SpiDeviceId::TMC9660_MOTOR_CONTROLLER:
            return GetSpiDevice(0, static_cast<int>(SpiDeviceId::TMC9660_MOTOR_CONTROLLER));
        default:
            return nullptr;  // Error: device not found
    }
}
```

### **🔍 Error Handling Best Practices**

```cpp
// Always check return values
BaseSpi* spi = comm_manager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
if (!spi) {
    // Handle error: device not available
    return false;
}

// Check device creation results
int device_index = comm_manager.CreateI2cDevice(0x48, 400000);
if (device_index < 0) {
    // Handle error: device creation failed
    return false;
}

// Use RAII for device cleanup
class DeviceWrapper {
private:
    CommChannelsManager& comm_manager_;
    int device_index_;
    
public:
    DeviceWrapper(CommChannelsManager& cm, int index) : comm_manager_(cm), device_index_(index) {}
    
    ~DeviceWrapper() {
        if (device_index_ >= 0) {
            comm_manager_.RemoveI2cDevice(device_index_);
        }
    }
    
    BaseI2c* GetDevice() {
        return comm_manager_.GetI2cDevice(0, device_index_);
    }
};
```

---

## **🔒 Thread Safety**

### **🛡️ Mutex Protection**

```cpp
class CommChannelsManager {
private:
    mutable RtosMutex mutex_;  // Thread safety
    
public:
    BaseSpi* GetSpiDevice(uint8_t bus_index, int device_index) noexcept {
        // All operations are protected by mutex
        // Implementation details handle locking
        return GetSpiDeviceImpl(bus_index, device_index);
    }
};
```

### **🔒 Thread Safety Guarantees**

- **✅ Read Operations**: Thread-safe access to device interfaces
- **✅ Write Operations**: Thread-safe device creation/removal
- **✅ Concurrent Access**: Multiple threads can access different devices
- **✅ Atomic Operations**: Device state changes are atomic

---

## **📋 Usage Examples**

### **🎯 Basic Usage**

```cpp
// 1. Get CommChannelsManager instance
auto& comm_manager = CommChannelsManager::GetInstance();

// 2. Ensure initialization
if (!comm_manager.EnsureInitialized()) {
    // Handle initialization failure
    return false;
}

// 3. Access built-in devices
BaseSpi* tmc9660_spi = comm_manager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
BaseI2c* bno08x_i2c = comm_manager.GetI2cDevice(I2cDeviceId::BNO08X_IMU);

// 4. Use devices with handlers
Tmc9660Handler tmc9660_handler(*tmc9660_spi, 0x01, &config);
Bno08xHandler bno08x_handler(*bno08x_i2c, config);
```

### **🔧 Advanced Usage**

```cpp
// 1. Create runtime I2C device
int device_index = comm_manager.CreateI2cDevice(0x48, 400000);
if (device_index >= 0) {
    BaseI2c* custom_i2c = comm_manager.GetI2cDevice(0, device_index);
    
    // 2. Register custom SPI device
    auto custom_spi = std::make_shared<MyCustomSpiDevice>();
    int spi_index = comm_manager.RegisterCustomSpiDevice(custom_spi, -1, 10);
    
    // 3. Access external bus devices
    BaseSpi* external_spi = comm_manager.GetSpiDevice(10, spi_index);
    
    // 4. Clean up
    comm_manager.RemoveI2cDevice(device_index);
    comm_manager.RemoveSpiDevice(spi_index);
}
```

### **🤖 Component Handler Integration**

```cpp
class MyComponentHandler {
private:
    CommChannelsManager& comm_manager_;
    std::unique_ptr<MyDeviceHandler> device_handler_;
    
public:
    MyComponentHandler() : comm_manager_(CommChannelsManager::GetInstance()) {}
    
    bool Initialize() {
        // Get communication interface
        BaseSpi* spi_interface = comm_manager_.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
        if (!spi_interface) {
            return false;
        }
        
        // Create device handler
        device_handler_ = std::make_unique<MyDeviceHandler>(*spi_interface, config);
        
        // Initialize handler
        return device_handler_->Initialize();
    }
    
    bool PerformOperation() {
        if (!device_handler_) {
            return false;
        }
        return device_handler_->PerformOperation();
    }
};
```

---

## **📚 Best Practices**

### **🎯 Device Access**

1. **Use Device IDs for Known Devices**: Prefer `SpiDeviceId`/`I2cDeviceId` over numeric indices
2. **Check Return Values**: Always verify device access success
3. **Use RAII**: Implement proper cleanup for runtime-created devices
4. **Prefer Bus-Aware Access**: Use `GetDevice(bus_index, device_index)` for external devices

### **🔧 Device Registration**

1. **Register Early**: Register custom devices during system initialization
2. **Use Meaningful Indices**: Choose device indices that make sense for your application
3. **Clean Up Properly**: Always remove devices when no longer needed
4. **Check Conflicts**: Verify device addresses don't conflict before creation

### **🛡️ Error Handling**

1. **Check Initialization**: Always ensure `CommChannelsManager` is initialized
2. **Validate Parameters**: Check device addresses and indices before use
3. **Handle Failures Gracefully**: Implement proper error recovery
4. **Log Errors**: Use the Logger system for debugging

### **🔒 Thread Safety**

1. **Don't Hold Locks**: Keep critical sections short
2. **Use RAII**: Let destructors handle cleanup
3. **Avoid Deadlocks**: Be careful with nested mutex acquisitions
4. **Test Concurrent Access**: Verify thread safety in your application

---

## **🚀 Future Extensibility**

### **🔧 Planned Enhancements**

1. **Multi-Bus Support**: Support for multiple built-in buses (SPI1, SPI2, I2C1, I2C2)
2. **Dynamic Bus Creation**: Runtime creation of new buses
3. **Device Hot-Plugging**: Support for hot-plugging devices
4. **Advanced Error Recovery**: Automatic retry and recovery mechanisms
5. **Performance Monitoring**: Built-in performance metrics and profiling

### **🔗 Extension Points**

1. **Custom Bus Types**: Support for custom bus implementations
2. **Device Discovery**: Automatic device discovery on buses
3. **Configuration Persistence**: Save/restore device configurations
4. **Remote Device Support**: Network-based device access
5. **Advanced Diagnostics**: Comprehensive system diagnostics

### **📈 Scalability Considerations**

1. **Memory Efficiency**: Optimize memory usage for large numbers of devices
2. **Performance**: Minimize overhead for high-frequency operations
3. **Flexibility**: Support for various device types and configurations
4. **Maintainability**: Keep the architecture clean and extensible

---

## **📖 Conclusion**

The `CommChannelsManager` provides a **robust, flexible, and extensible** foundation for communication management in the HardFOC system. Its multi-bus architecture, runtime device management, and exception-free design make it suitable for both simple and complex applications.

Key benefits:
- **🔧 Flexibility**: Support for built-in and external devices
- **🎯 Simplicity**: Easy-to-use API with type-safe device access
- **🛡️ Reliability**: Exception-free design with comprehensive error handling
- **🔒 Safety**: Thread-safe operations with proper synchronization
- **🚀 Extensibility**: Support for custom devices and future enhancements

The architecture successfully balances **simplicity of use** with **power and flexibility**, making it an excellent foundation for embedded communication management. 