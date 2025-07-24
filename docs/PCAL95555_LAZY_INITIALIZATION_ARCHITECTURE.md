/**
 * @file PCAL95555_LAZY_INITIALIZATION_ARCHITECTURE.md
 * @brief Lazy Initialization Pattern Implementation for PCAL95555Handler
 * @date 2025-01-24
 * @author HardFOC Team
 */

# PCAL95555Handler: Lazy Initialization Architecture

## **Problem with Eager Initialization**

The original implementation created driver objects in the constructor:

### **❌ Eager Initialization (Original)**
```cpp
class Pcal95555Handler {
public:
    Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin) noexcept
        : i2c_adapter_(std::make_unique<Pcal95555I2cAdapter>(i2c_device)),     // Created in constructor
          pcal95555_driver_(std::make_shared<PCAL95555>(i2c_adapter_.get(), 
                                                        i2c_device.GetDeviceAddress())), // Created in constructor
          interrupt_pin_(interrupt_pin) {
        // Objects created regardless of whether they'll be used
    }
};
```

### **Problems:**
1. **Resource Waste**: Objects created even if handler is never used
2. **Constructor Failure**: Constructor can fail due to I2C/driver initialization
3. **Inconsistent Pattern**: Other handlers (MotorController, ImuManager) use lazy initialization
4. **Testing Difficulties**: Hard to test without working hardware
5. **Error Handling**: Constructor can't return error codes

## **✅ Solution: Lazy Initialization Pattern**

### **Lightweight Constructor (Stores References Only)**
```cpp
class Pcal95555Handler {
public:
    Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin = nullptr) noexcept
        : i2c_device_(i2c_device),           // Store reference only
          i2c_adapter_(nullptr),             // Created during initialization
          pcal95555_driver_(nullptr),        // Created during initialization
          initialized_(false),
          interrupt_pin_(interrupt_pin) {
        // Lightweight constructor - no I2C communication or object creation
        pin_registry_.fill(nullptr);
    }
};
```

### **EnsureInitialized() Pattern**
```cpp
bool EnsureInitialized() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (initialized_) {
        return true;  // Already initialized
    }
    
    return Initialize() == hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Initialize() noexcept {
    // Create adapter and driver on-demand
    if (!i2c_adapter_) {
        i2c_adapter_ = std::make_unique<Pcal95555I2cAdapter>(i2c_device_);
    }
    
    if (!pcal95555_driver_) {
        pcal95555_driver_ = std::make_shared<PCAL95555>(i2c_adapter_.get(), 
                                                       i2c_device_.GetDeviceAddress());
    }
    
    // Initialize hardware
    if (!pcal95555_driver_->init()) {
        return hf_gpio_err_t::GPIO_ERR_INITIALIZATION_FAILED;
    }
    
    initialized_ = true;
    return hf_gpio_err_t::GPIO_SUCCESS;
}
```

## **Consistency with Other Handlers**

### **MotorController Pattern**
```cpp
class MotorController {
public:
    static MotorController& GetInstance();
    
    bool EnsureInitialized() noexcept {
        MutexLockGuard lock(deviceMutex_);
        if (initialized_) return true;
        return Initialize();
    }
    
private:
    bool Initialize();  // Creates TMC9660 handlers on-demand
    bool initialized_ = false;
};
```

### **ImuManager Pattern**
```cpp
class ImuManager {
public:
    static ImuManager& GetInstance() noexcept;
    
    bool Initialize() noexcept {
        MutexLockGuard lock(manager_mutex_);
        if (initialized_) return true;
        
        // Create BNO08x handler on-demand
        bool bno08x_ok = InitializeBno08xHandler();
        initialized_ = bno08x_ok;
        return initialized_;
    }
    
private:
    bool initialized_ = false;
};
```

### **PCAL95555Handler Pattern (New)**
```cpp
class Pcal95555Handler {
public:
    explicit Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin = nullptr) noexcept;
    
    bool EnsureInitialized() noexcept {
        MutexLockGuard lock(handler_mutex_);
        if (initialized_) return true;
        return Initialize() == hf_gpio_err_t::GPIO_SUCCESS;
    }
    
private:
    hf_gpio_err_t Initialize() noexcept;  // Creates adapter/driver on-demand
    bool initialized_ = false;
};
```

## **Usage Pattern Changes**

### **Before (Eager Initialization)**
```cpp
// Constructor could fail internally, but no way to detect
Pcal95555Handler handler(i2c_device, interrupt_pin);

// Hope it works
auto pin = handler.CreateGpioPin(5);
```

### **After (Lazy Initialization)**
```cpp
// Lightweight constructor always succeeds
Pcal95555Handler handler(i2c_device, interrupt_pin);

// Initialization happens on first use
if (handler.EnsureInitialized()) {
    auto pin = handler.CreateGpioPin(5);  // This calls EnsureInitialized() internally
}

// Or explicit initialization
if (handler.Initialize() == hf_gpio_err_t::GPIO_SUCCESS) {
    // Use handler
}
```

## **API Method Updates**

All public methods now call `EnsureInitialized()` before accessing the driver:

### **GPIO Operations**
```cpp
hf_gpio_err_t SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    if (!EnsureInitialized()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;  // Lazy init
    
    MutexLockGuard lock(handler_mutex_);
    return pcal95555_driver_->setPinDirection(pin, direction == HF_GPIO_DIRECTION_INPUT) 
           ? hf_gpio_err_t::GPIO_SUCCESS 
           : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}
```

### **Pin Creation**
```cpp
std::shared_ptr<BaseGpio> CreateGpioPin(...) noexcept {
    if (pin >= 16) return nullptr;
    
    if (!EnsureInitialized()) {  // Lazy init before creating pins
        return nullptr;
    }
    
    // Rest of pin creation logic
}
```

## **Memory Management**

### **Lazy Creation**
```cpp
// Objects created only when needed
if (!i2c_adapter_) {
    i2c_adapter_ = std::make_unique<Pcal95555I2cAdapter>(i2c_device_);
}

if (!pcal95555_driver_) {
    pcal95555_driver_ = std::make_shared<PCAL95555>(i2c_adapter_.get(), address);
}
```

### **Proper Cleanup**
```cpp
hf_gpio_err_t Deinitialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_SUCCESS;
    
    // Clear pin registry
    {
        MutexLockGuard pin_lock(pin_registry_mutex_);
        pin_registry_.fill(nullptr);
    }
    
    // Clean up lazily-created objects
    pcal95555_driver_.reset();  // Release driver
    i2c_adapter_.reset();       // Release adapter
    
    initialized_ = false;
    return hf_gpio_err_t::GPIO_SUCCESS;
}
```

## **Benefits of Lazy Initialization**

### **1. Consistent Architecture**
- **Pattern Alignment**: Matches MotorController and ImuManager patterns
- **Predictable API**: Same initialization approach across all handlers
- **Error Handling**: Consistent error reporting and recovery

### **2. Resource Efficiency**
- **On-Demand Creation**: Objects created only when actually needed
- **Memory Savings**: No wasted resources for unused handlers
- **Clean Teardown**: Proper resource cleanup on deinitialization

### **3. Better Error Handling**
- **Graceful Failures**: Constructor never fails, initialization can be retried
- **Error Propagation**: Clear error codes from initialization attempts
- **Recovery Options**: Failed initialization doesn't break the object

### **4. Testing and Development**
- **Mock-Friendly**: Easy to test without hardware (constructor always succeeds)
- **Incremental Development**: Can create handler before I2C device is ready
- **Debug Support**: Clear initialization state tracking

### **5. Thread Safety**
- **Atomic Initialization**: EnsureInitialized() is thread-safe
- **Consistent State**: No race conditions during object creation
- **Proper Locking**: Handler mutex protects initialization state

## **Migration Impact**

### **No Breaking Changes**
- User API remains exactly the same
- All existing code continues to work
- Internal implementation changes only

### **Improved Reliability**
- Constructor never fails due to I2C issues
- Better error reporting from initialization
- Consistent behavior across all handlers

## **Example Usage**

### **Simple Usage**
```cpp
auto& gpio_mgr = GpioManager::GetInstance();
auto i2c_device = gpio_mgr.GetI2cDevice(0x20);
auto interrupt_pin = gpio_mgr.GetGpio(HfFunctionalGpioPin::I2C_PCAL95555_INT);

// Lightweight constructor
Pcal95555Handler handler(*i2c_device, interrupt_pin);

// Lazy initialization on first use
auto pin5 = handler.CreateGpioPin(5);  // Initializes handler automatically
if (pin5) {
    pin5->SetDirection(HF_GPIO_DIRECTION_OUTPUT);
    pin5->SetPinLevel(HF_GPIO_LEVEL_HIGH);
}
```

### **Explicit Initialization**
```cpp
Pcal95555Handler handler(*i2c_device, interrupt_pin);

// Explicit initialization with error checking
if (handler.EnsureInitialized()) {
    // Handler is ready to use
    auto pins = handler.GetCreatedPins();
    // ... use handler
} else {
    // Handle initialization failure
    ESP_LOGE("App", "Failed to initialize PCAL95555 handler");
}
```

## **Conclusion**

The lazy initialization pattern provides:

✅ **Architectural Consistency**: Matches MotorController and ImuManager patterns  
✅ **Resource Efficiency**: Objects created only when needed  
✅ **Better Error Handling**: Clear initialization failure reporting  
✅ **Thread Safety**: Atomic initialization with proper locking  
✅ **Testing Support**: Mock-friendly constructor that never fails  
✅ **Clean Lifecycle**: Proper resource creation and cleanup  

This pattern makes PCAL95555Handler a first-class citizen in the HardFOC handler ecosystem, following established conventions while providing robust, efficient GPIO expansion capabilities.
