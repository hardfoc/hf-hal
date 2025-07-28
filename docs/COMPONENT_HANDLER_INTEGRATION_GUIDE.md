# **Component Handler Integration Guide**

## **📋 Table of Contents**

1. [Overview](#overview)
2. [Integration Architecture](#integration-architecture)
3. [Component Handler Patterns](#component-handler-patterns)
4. [ImuManager Integration](#imumanager-integration)
5. [MotorController Integration](#motorcontroller-integration)
6. [GpioManager Integration](#gpiomanager-integration)
7. [AdcManager Integration](#adcmanager-integration)
8. [Custom Component Handler](#custom-component-handler)
9. [Best Practices](#best-practices)
10. [Troubleshooting](#troubleshooting)

---

## **🎯 Overview**

Component handlers are **high-level managers** that provide domain-specific functionality by coordinating multiple device handlers and communication interfaces. They use `CommChannelsManager` to access the underlying communication buses and create device-specific handlers.

### **🔑 Key Concepts**

- **Component Handler**: High-level manager (ImuManager, MotorController, GpioManager, AdcManager)
- **Device Handler**: Device-specific implementation (Bno08xHandler, Tmc9660Handler, Pcal95555Handler)
- **Communication Interface**: Abstract interface (BaseSpi, BaseI2c, BaseUart, BaseCan)
- **CommChannelsManager**: Central communication hub that manages all interfaces

---

## **🏗️ Integration Architecture**

### **📊 Component Handler Architecture**

```
┌─────────────────────────────────────────────────────────────┐
│                    COMPONENT HANDLER                        │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ Device      │ │ Device      │ │ Device      │           │
│  │ Handler 1   │ │ Handler 2   │ │ Handler N   │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
│           │               │               │                │
│           └───────────────┼───────────────┘                │
│                           │                                │
└───────────────────────────┼────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                 COMMCHANNELSMANAGER                         │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ BaseSpi     │ │ BaseI2c     │ │ BaseUart    │           │
│  │ Interface   │ │ Interface   │ │ Interface   │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                           │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │ ESP32 SPI   │ │ ESP32 I2C   │ │ ESP32 UART  │           │
│  │ Bus         │ │ Bus         │ │ Bus         │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
```

### **🔄 Integration Pattern**

```cpp
// Standard Integration Pattern
class ComponentHandler {
private:
    CommChannelsManager& comm_manager_;
    std::unique_ptr<DeviceHandler> device_handler_;
    
public:
    ComponentHandler() : comm_manager_(CommChannelsManager::GetInstance()) {}
    
    bool Initialize() {
        // 1. Get communication interface from CommChannelsManager
        BaseSpi* spi_interface = comm_manager_.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
        if (!spi_interface) {
            return false;
        }
        
        // 2. Create device-specific handler with the interface
        device_handler_ = std::make_unique<DeviceHandler>(*spi_interface, config);
        
        // 3. Initialize the device handler
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

## **🔧 Component Handler Patterns**

### **🎯 Pattern 1: Single Device Handler**

```cpp
class SingleDeviceComponent {
private:
    CommChannelsManager& comm_manager_;
    std::unique_ptr<DeviceHandler> device_handler_;
    
public:
    bool Initialize() {
        // Get interface and create handler
        BaseSpi* interface = comm_manager_.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
        if (!interface) return false;
        
        device_handler_ = std::make_unique<DeviceHandler>(*interface, config);
        return device_handler_->Initialize();
    }
};
```

### **🔧 Pattern 2: Multiple Device Handlers**

```cpp
class MultiDeviceComponent {
private:
    CommChannelsManager& comm_manager_;
    std::array<std::unique_ptr<DeviceHandler>, MAX_DEVICES> device_handlers_;
    std::array<bool, MAX_DEVICES> device_active_;
    
public:
    bool Initialize() {
        // Initialize multiple devices
        for (size_t i = 0; i < MAX_DEVICES; ++i) {
            if (device_active_[i]) {
                BaseSpi* interface = comm_manager_.GetSpiDevice(0, i);
                if (!interface) continue;
                
                device_handlers_[i] = std::make_unique<DeviceHandler>(*interface, config);
                device_handlers_[i]->Initialize();
            }
        }
        return true;
    }
};
```

### **🔄 Pattern 3: Runtime Device Creation**

```cpp
class RuntimeDeviceComponent {
private:
    CommChannelsManager& comm_manager_;
    std::map<int, std::unique_ptr<DeviceHandler>> device_handlers_;
    
public:
    int CreateDevice(uint8_t address) {
        // Create runtime device
        int device_index = comm_manager_.CreateI2cDevice(address, 400000);
        if (device_index < 0) return -1;
        
        // Get interface and create handler
        BaseI2c* interface = comm_manager_.GetI2cDevice(0, device_index);
        if (!interface) return -1;
        
        device_handlers_[device_index] = std::make_unique<DeviceHandler>(*interface, config);
        device_handlers_[device_index]->Initialize();
        
        return device_index;
    }
    
    void RemoveDevice(int device_index) {
        device_handlers_.erase(device_index);
        comm_manager_.RemoveI2cDevice(device_index);
    }
};
```

---

## **🤖 ImuManager Integration**

### **📋 Overview**

`ImuManager` manages BNO08x IMU devices, supporting both onboard and external devices with I2C and SPI interfaces.

### **🔧 Architecture**

```cpp
class ImuManager {
private:
    CommChannelsManager* comm_manager_;
    std::array<std::unique_ptr<Bno08xHandler>, MAX_IMU_DEVICES> bno08x_handlers_;
    std::array<bool, MAX_IMU_DEVICES> device_active_;
    std::array<bool, MAX_IMU_DEVICES> device_initialized_;
    std::array<int, MAX_IMU_DEVICES> i2c_device_indices_;  // Track I2C devices for cleanup
};
```

### **🔄 Integration Flow**

#### **1. Onboard Device Initialization**

```cpp
bool ImuManager::InitializeOnboardBno08xDevice() {
    // 1. Get I2C device from CommChannelsManager
    BaseI2c* imu_device = comm_manager_->GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!imu_device) {
        Logger::GetInstance().Error("ImuManager", "BNO08x IMU device not available");
        return false;
    }
    
    // 2. Create BNO08x handler with I2C interface
    Bno08xConfig config = Bno08xHandler::GetDefaultConfig();
    bno08x_handlers_[ONBOARD_IMU_INDEX] = std::make_unique<Bno08xHandler>(*imu_device, config);
    
    // 3. Initialize the handler
    Bno08xError init_result = bno08x_handlers_[ONBOARD_IMU_INDEX]->Initialize();
    if (init_result != Bno08xError::SUCCESS) {
        Logger::GetInstance().Error("ImuManager", "Failed to initialize BNO08x handler");
        bno08x_handlers_[ONBOARD_IMU_INDEX].reset();
        return false;
    }
    
    // 4. Mark as active and initialized
    device_active_[ONBOARD_IMU_INDEX] = true;
    device_initialized_[ONBOARD_IMU_INDEX] = true;
    
    return true;
}
```

#### **2. External Device Creation**

```cpp
bool ImuManager::CreateExternalBno08xDevice(uint8_t deviceIndex, uint8_t i2c_address, uint32_t i2c_speed_hz) {
    // 1. Create I2C device in CommChannelsManager
    int i2c_device_index = comm_manager_->CreateI2cDevice(i2c_address, i2c_speed_hz);
    if (i2c_device_index < 0) {
        Logger::GetInstance().Error("ImuManager", "Failed to create I2C device");
        return false;
    }
    
    // 2. Get I2C interface for the new device
    BaseI2c* i2c_interface = comm_manager_->GetI2cDevice(0, i2c_device_index);
    if (!i2c_interface) {
        Logger::GetInstance().Error("ImuManager", "Failed to get I2C interface");
        comm_manager_->RemoveI2cDevice(i2c_device_index);
        return false;
    }
    
    // 3. Create BNO08x handler with the interface
    Bno08xConfig config = Bno08xHandler::GetDefaultConfig();
    auto external_handler = std::make_unique<Bno08xHandler>(*i2c_interface, config);
    
    // 4. Initialize the handler
    Bno08xError init_result = external_handler->Initialize();
    if (init_result != Bno08xError::SUCCESS) {
        Logger::GetInstance().Error("ImuManager", "Failed to initialize external BNO08x handler");
        comm_manager_->RemoveI2cDevice(i2c_device_index);
        return false;
    }
    
    // 5. Store handler and track I2C device index
    bno08x_handlers_[deviceIndex] = std::move(external_handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = true;
    i2c_device_indices_[deviceIndex] = i2c_device_index;  // For cleanup
    
    return true;
}
```

#### **3. Device Cleanup**

```cpp
bool ImuManager::DeleteExternalDevice(uint8_t deviceIndex) {
    // 1. Get stored I2C device index
    int i2c_device_index = i2c_device_indices_[deviceIndex];
    
    // 2. Remove BNO08x handler
    bno08x_handlers_[deviceIndex].reset();
    device_active_[deviceIndex] = false;
    device_initialized_[deviceIndex] = false;
    
    // 3. Remove I2C device from CommChannelsManager
    if (i2c_device_index >= 0) {
        comm_manager_->RemoveI2cDevice(i2c_device_index);
        i2c_device_indices_[deviceIndex] = -1;
    }
    
    return true;
}
```

---

## **⚙️ MotorController Integration**

### **📋 Overview**

`MotorController` manages TMC9660 motor controller devices, supporting both onboard and external devices with SPI and UART interfaces.

### **🔧 Architecture**

```cpp
class MotorController {
private:
    std::array<std::unique_ptr<Tmc9660Handler>, MAX_TMC9660_DEVICES> tmcHandlers_;
    std::array<bool, MAX_TMC9660_DEVICES> deviceActive_;
    std::array<bool, MAX_TMC9660_DEVICES> deviceInitialized_;
    bool onboardDeviceCreated_;
};
```

### **🔄 Integration Flow**

#### **1. Onboard Device Initialization**

```cpp
bool MotorController::Initialize() {
    // 1. Get SPI device from CommChannelsManager
    BaseSpi* spiInterface = commManager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spiInterface) {
        return false;
    }
    
    // 2. Create TMC9660 handler with SPI interface
    auto onboardHandler = std::make_unique<Tmc9660Handler>(
        *spiInterface, 
        0x01,  // Default TMC9660 address
        &Tmc9660Handler::kDefaultBootConfig
    );
    
    // 3. Store handler and mark as active
    tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
    deviceActive_[ONBOARD_TMC9660_INDEX] = true;
    deviceInitialized_[ONBOARD_TMC9660_INDEX] = false;
    
    // 4. Initialize all active devices
    bool allSuccess = true;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i]) {
            deviceInitialized_[i] = tmcHandlers_[i]->Initialize();
            if (!deviceInitialized_[i]) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}
```

#### **2. External Device Creation**

```cpp
bool MotorController::CreateExternalDevice(uint8_t csDeviceIndex, SpiDeviceId spiDeviceId, uint8_t address) {
    // 1. Get SPI device from CommChannelsManager
    BaseSpi* spiInterface = commManager.GetSpiDevice(spiDeviceId);
    if (!spiInterface) {
        return false;
    }
    
    // 2. Create TMC9660 handler with SPI interface
    auto externalHandler = std::make_unique<Tmc9660Handler>(
        *spiInterface,
        address,
        &Tmc9660Handler::kDefaultBootConfig
    );
    
    // 3. Store handler and mark as active
    tmcHandlers_[csDeviceIndex] = std::move(externalHandler);
    deviceActive_[csDeviceIndex] = true;
    deviceInitialized_[csDeviceIndex] = false;
    
    // 4. Initialize if system is already initialized
    if (IsInitialized()) {
        deviceInitialized_[csDeviceIndex] = tmcHandlers_[csDeviceIndex]->Initialize();
    }
    
    return true;
}
```

---

## **🔌 GpioManager Integration**

### **📋 Overview**

`GpioManager` manages GPIO pins from multiple sources (ESP32, PCAL95555, TMC9660), using `CommChannelsManager` to access I2C interfaces for expander chips.

### **🔧 Architecture**

```cpp
class GpioManager {
private:
    std::unordered_map<std::string_view, std::shared_ptr<BaseGpio>> gpio_registry_;
    std::unique_ptr<Pcal95555Handler> pcal95555_handler_;
    MotorController* motor_controller_;  // For TMC9660 GPIO access
};
```

### **🔄 Integration Flow**

#### **1. PCAL95555 Handler Creation**

```cpp
bool GpioManager::EnsurePcal95555Handler() {
    // 1. Get I2C device from CommChannelsManager
    auto i2c_device = comm_manager.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c_device) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
        return false;
    }
    
    // 2. Create PCAL95555 handler with I2C interface
    pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_device);
    
    // 3. Initialize the handler
    if (!pcal95555_handler_->Initialize()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
        pcal95555_handler_.reset();
        return false;
    }
    
    return true;
}
```

#### **2. TMC9660 GPIO Access**

```cpp
Tmc9660Handler* GpioManager::GetTmc9660Handler(uint8_t device_index) {
    // Get MotorController instance if not already cached
    if (!motor_controller_) {
        motor_controller_ = &MotorController::GetInstance();
    }
    
    // Access TMC9660 handler through MotorController
    return motor_controller_->handler(device_index);
}
```

#### **3. GPIO Pin Creation**

```cpp
std::shared_ptr<BaseGpio> GpioManager::CreatePcal95555GpioPin(hf_u8_t pin_id, hf_u8_t unit_number, bool is_inverted, 
                                                             bool has_pull, bool pull_is_up, bool is_push_pull,
                                                             hf_u32_t max_current_ma) noexcept {
    // 1. Ensure PCAL95555 handler is available
    if (!EnsurePcal95555Handler()) {
        return nullptr;
    }
    
    // 2. Create GPIO pin using PCAL95555 handler
    return pcal95555_handler_->CreateGpioPin(pin_id, is_inverted, has_pull, pull_is_up, is_push_pull, max_current_ma);
}
```

---

## **📊 AdcManager Integration**

### **📋 Overview**

`AdcManager` manages ADC channels from multiple sources (ESP32, TMC9660), using `MotorController` to access TMC9660 ADC channels.

### **🔧 Architecture**

```cpp
class AdcManager {
private:
    std::unordered_map<std::string_view, std::shared_ptr<BaseAdc>> adc_registry_;
    MotorController* motor_controller_;  // For TMC9660 ADC access
};
```

### **🔄 Integration Flow**

#### **1. TMC9660 ADC Channel Registration**

```cpp
hf_adc_err_t AdcManager::RegisterPlatformChannels() noexcept {
    // 1. Get MotorController for TMC9660 access
    if (!motor_controller_) {
        motor_controller_ = &MotorController::GetInstance();
    }
    
    // 2. Iterate through platform ADC channel mappings
    for (size_t i = 0; i < HF_ADC_MAPPING_SIZE; ++i) {
        const auto& mapping = HF_ADC_MAPPING[i];
        
        // 3. Handle TMC9660 ADC channels
        if (mapping.chip_type == static_cast<uint8_t>(HfAdcChipType::TMC9660_CONTROLLER)) {
            // 4. Get TMC9660 handler
            Tmc9660Handler* tmc9660_handler = motor_controller_->handler(mapping.unit_number);
            if (!tmc9660_handler) {
                continue;
            }
            
            // 5. Create TMC9660 ADC wrapper
            auto tmc9660_wrapper = std::make_unique<Tmc9660AdcWrapper>(
                *tmc9660_handler, 
                mapping.physical_channel,
                mapping.resolution_bits,
                mapping.max_voltage_mv,
                mapping.voltage_divider
            );
            
            // 6. Register the ADC channel
            hf_adc_err_t result = RegisterChannel(mapping.name, std::move(tmc9660_wrapper));
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManager", "Registered TMC9660 channel %s", mapping.name);
            }
        }
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}
```

---

## **🔧 Custom Component Handler**

### **📋 Template for New Component Handlers**

```cpp
class MyCustomComponent {
private:
    CommChannelsManager& comm_manager_;
    std::unique_ptr<MyDeviceHandler> device_handler_;
    bool initialized_;
    
public:
    MyCustomComponent() 
        : comm_manager_(CommChannelsManager::GetInstance()), initialized_(false) {}
    
    bool Initialize() {
        if (initialized_) {
            return true;
        }
        
        // 1. Ensure CommChannelsManager is initialized
        if (!comm_manager_.EnsureInitialized()) {
            return false;
        }
        
        // 2. Get communication interface
        BaseSpi* spi_interface = comm_manager_.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
        if (!spi_interface) {
            return false;
        }
        
        // 3. Create device handler
        device_handler_ = std::make_unique<MyDeviceHandler>(*spi_interface, config);
        
        // 4. Initialize device handler
        if (!device_handler_->Initialize()) {
            device_handler_.reset();
            return false;
        }
        
        initialized_ = true;
        return true;
    }
    
    bool PerformOperation() {
        if (!initialized_ || !device_handler_) {
            return false;
        }
        return device_handler_->PerformOperation();
    }
    
    void Shutdown() {
        if (device_handler_) {
            device_handler_->Deinitialize();
            device_handler_.reset();
        }
        initialized_ = false;
    }
    
    ~MyCustomComponent() {
        Shutdown();
    }
};
```

---

## **📚 Best Practices**

### **🎯 Initialization Patterns**

1. **Lazy Initialization**: Use `EnsureInitialized()` for CommChannelsManager
2. **Error Checking**: Always check interface availability before creating handlers
3. **Resource Management**: Use RAII for device handler lifecycle
4. **Cleanup**: Implement proper shutdown procedures

### **🔧 Interface Access**

1. **Use Device IDs**: Prefer `SpiDeviceId`/`I2cDeviceId` over numeric indices
2. **Check Return Values**: Always verify interface access success
3. **Handle Failures**: Implement graceful error handling
4. **Log Operations**: Use Logger for debugging and monitoring

### **🔄 Device Lifecycle**

1. **Creation**: Create handlers during initialization
2. **Initialization**: Initialize handlers after creation
3. **Usage**: Access handlers through component methods
4. **Cleanup**: Properly deinitialize and cleanup resources

### **🛡️ Error Handling**

1. **Validate Parameters**: Check all input parameters
2. **Handle Failures**: Implement proper error recovery
3. **Resource Cleanup**: Clean up resources on failure
4. **Log Errors**: Use Logger for error reporting

---

## **🔍 Troubleshooting**

### **❌ Common Issues**

#### **1. Interface Not Available**

```cpp
// Problem: GetSpiDevice returns nullptr
BaseSpi* spi = comm_manager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
if (!spi) {
    // Solution: Check CommChannelsManager initialization
    if (!comm_manager.EnsureInitialized()) {
        // Handle initialization failure
        return false;
    }
    
    // Check if device is properly configured
    // Verify pin configuration in board mapping
}
```

#### **2. Device Creation Failure**

```cpp
// Problem: CreateI2cDevice returns -1
int device_index = comm_manager.CreateI2cDevice(0x48, 400000);
if (device_index < 0) {
    // Solution: Check for address conflicts
    if (comm_manager.HasI2cDeviceAtAddress(0, 0x48)) {
        // Device already exists at this address
        device_index = comm_manager.GetI2cDeviceIndexByAddress(0x48);
    } else {
        // Handle creation failure
        return false;
    }
}
```

#### **3. Handler Initialization Failure**

```cpp
// Problem: Device handler initialization fails
auto handler = std::make_unique<MyDeviceHandler>(*interface, config);
if (!handler->Initialize()) {
    // Solution: Check interface configuration
    // Verify device is present and responding
    // Check power and connections
    return false;
}
```

### **🔧 Debugging Tips**

1. **Enable Logging**: Use Logger for detailed operation tracking
2. **Check Initialization**: Verify CommChannelsManager is initialized
3. **Validate Configuration**: Check pin and device configurations
4. **Test Interfaces**: Test communication interfaces independently
5. **Monitor Resources**: Check for resource leaks and cleanup

---

## **📖 Conclusion**

Component handlers provide **high-level, domain-specific functionality** by coordinating device handlers and communication interfaces. The integration with `CommChannelsManager` provides:

- **🔧 Flexibility**: Support for multiple device types and interfaces
- **🎯 Simplicity**: Clean separation of concerns
- **🛡️ Reliability**: Proper error handling and resource management
- **🔒 Safety**: Thread-safe operations and proper cleanup
- **🚀 Extensibility**: Easy to add new component handlers

By following the established patterns and best practices, you can create robust, maintainable component handlers that integrate seamlessly with the HardFOC system architecture. 