/**
 * @file PCAL95555_PIN_REGISTRY_ARCHITECTURE.md
 * @brief Improved PCAL95555Handler Architecture with Pin Registry Management
 * @date 2025-01-24
 * @author HardFOC Team
 */

# PCAL95555Handler Pin Registry Architecture

## **Problem Solved**

The original implementation had several critical flaws:

1. **No Pin Tracking**: Handler didn't track which pins were already created
2. **Resource Waste**: Creating temporary `Pcal95555GpioPin` objects in interrupt handlers
3. **Multiple Pin Creation**: Users could create multiple objects for the same physical pin
4. **Poor Resource Management**: No way to get shared access to existing pins

## **Solution: Pin Registry Pattern**

### **Key Architecture Changes**

#### **1. Pin Registry Management**
```cpp
// Private members in Pcal95555Handler
std::array<std::shared_ptr<Pcal95555GpioPin>, 16> pin_registry_;  ///< Registry of created pins
mutable RtosMutex pin_registry_mutex_;  ///< Thread safety for pin registry
```

#### **2. Shared Ownership Model**
- **Before**: `std::unique_ptr<BaseGpio> CreateGpioPin(...)`
- **After**: `std::shared_ptr<BaseGpio> CreateGpioPin(..., bool allow_existing = true)`

#### **3. Pin Creation Logic**
```cpp
std::shared_ptr<BaseGpio> CreateGpioPin(pin, ..., bool allow_existing = true) {
    MutexLockGuard pin_lock(pin_registry_mutex_);
    
    // Check if pin already exists
    if (pin_registry_[pin] != nullptr) {
        if (allow_existing) {
            return pin_registry_[pin];  // Return existing pin
        } else {
            return nullptr;  // Fail if not allowing existing
        }
    }
    
    // Create new pin and store in registry
    auto new_pin = std::make_shared<Pcal95555GpioPin>(...);
    if (new_pin && new_pin->Initialize()) {
        pin_registry_[pin] = new_pin;
        return new_pin;
    }
    
    return nullptr;
}
```

#### **4. Optimized Interrupt Processing**
```cpp
void ProcessInterrupts() noexcept {
    uint16_t status = pcal95555_driver_->getInterruptStatus();
    
    for (int pin = 0; pin < 16; pin++) {
        if ((status & (1 << pin)) && pin_callbacks_[pin]) {
            // Use existing pin from registry (no object creation in ISR)
            BaseGpio* gpio_ptr = nullptr;
            {
                MutexLockGuard pin_lock(pin_registry_mutex_);
                if (pin_registry_[pin]) {
                    gpio_ptr = pin_registry_[pin].get();
                }
            }
            
            // Call callback directly with existing pin or nullptr
            pin_callbacks_[pin](gpio_ptr, pin_triggers_[pin], pin_user_data_[pin]); 
        }
    }
}
```

## **New API Methods**

### **Pin Registry Access**
```cpp
// Get existing GPIO pin by number
std::shared_ptr<BaseGpio> GetGpioPin(hf_pin_num_t pin) noexcept;

// Check if a pin is already created
bool IsPinCreated(hf_pin_num_t pin) const noexcept;

// Get list of all created pin numbers
std::vector<hf_pin_num_t> GetCreatedPins() const noexcept;
```

### **Enhanced Pin Creation**
```cpp
// Create or get existing pin (default behavior)
auto pin = handler.CreateGpioPin(5);  // Returns existing if already created

// Force creation failure if pin exists
auto pin = handler.CreateGpioPin(5, direction, active_state, output_mode, pull_mode, false);
```

## **Benefits of New Architecture**

### **1. Resource Efficiency**
- **No Temporary Objects**: Interrupt handlers use existing pins from registry
- **Shared Ownership**: Multiple users can share the same pin object safely
- **Controlled Creation**: Prevents duplicate pin objects for same physical pin

### **2. Better Resource Management**
- **Registry Tracking**: Handler knows exactly which pins are created
- **Proper Cleanup**: `Deinitialize()` releases all pin registry entries
- **Memory Safety**: Shared pointers handle lifetime management automatically

### **3. Interrupt Performance**
- **Zero Allocation in ISR**: No `std::make_unique` calls during interrupt processing
- **Direct Callback**: Use existing pin objects or pass nullptr if no pin created
- **Thread Safety**: Proper locking around pin registry access

### **4. User-Friendly API**
- **Get Existing Pins**: Users can retrieve previously created pins
- **Pin Discovery**: List all created pins for debugging/management
- **Flexible Creation**: Allow or prevent duplicate pin creation as needed

## **Usage Examples**

### **Basic Pin Creation**
```cpp
auto& handler = Pcal95555Handler(i2c_device, interrupt_pin);
handler.Initialize();

// Create a pin (or get existing)
auto pin5 = handler.CreateGpioPin(5, HF_GPIO_DIRECTION_OUTPUT);

// Later, get the same pin from another part of code
auto same_pin5 = handler.GetGpioPin(5);  // Returns same shared_ptr

// Check if pin exists
if (handler.IsPinCreated(5)) {
    // Pin exists, safe to use
}
```

### **Interrupt Setup with Registry**
```cpp
// Create pin first
auto input_pin = handler.CreateGpioPin(3, HF_GPIO_DIRECTION_INPUT);

// Configure interrupt on the created pin
input_pin->ConfigureInterrupt(
    HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
    [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
        // gpio parameter will be the same pin object created above
        // No temporary objects created during interrupt processing
    },
    nullptr
);
```

### **Resource Management**
```cpp
// Get all created pins for cleanup or debugging
auto created_pins = handler.GetCreatedPins();
for (auto pin_num : created_pins) {
    auto pin = handler.GetGpioPin(pin_num);
    // Do something with each created pin
}

// Cleanup - this releases all pins in registry
handler.Deinitialize();
```

## **Thread Safety**

### **Multiple Mutexes for Fine-Grained Locking**
- `handler_mutex_`: Protects handler initialization state
- `pin_registry_mutex_`: Protects pin registry operations
- `interrupt_mutex_`: Protects interrupt configuration arrays

### **Deadlock Prevention**
- Mutexes acquired in consistent order
- Short lock durations in interrupt context
- Separate locks for different concerns

## **Comparison with Other Handlers**

This architecture follows the successful patterns used in:

### **MotorController**
- Device registry with array-based storage
- Shared pointer management for device access
- Thread-safe creation and access methods

### **ImuManager**
- Singleton pattern with resource tracking
- Proper initialization and cleanup lifecycle
- Exception-free design with error handling

### **BNO08xHandler**
- Shared pointer model for driver management
- Registry-based access patterns
- Clean separation of concerns

## **Migration Guide**

### **Old Code**
```cpp
auto pin = handler.CreateGpioPin(5);  // Returns unique_ptr
// No way to get existing pins
// Temporary objects created in interrupts
```

### **New Code**
```cpp
auto pin = handler.CreateGpioPin(5);  // Returns shared_ptr
auto same_pin = handler.GetGpioPin(5);  // Get existing pin
// Interrupts use existing pins from registry
```

### **Breaking Changes**
1. `CreateGpioPin()` now returns `shared_ptr` instead of `unique_ptr`
2. Added optional `allow_existing` parameter to `CreateGpioPin()`
3. `Deinitialize()` now clears pin registry (invalidates existing pins)

## **Conclusion**

The pin registry architecture solves all identified problems:

✅ **Pin Tracking**: Complete registry of all created pins  
✅ **Resource Efficiency**: No temporary objects in interrupt handlers  
✅ **Controlled Creation**: Prevents or allows duplicate pin creation  
✅ **Shared Access**: Multiple users can safely share pin objects  
✅ **Performance**: Zero allocation in interrupt processing  
✅ **Thread Safety**: Proper locking for all registry operations  

This design makes the PCAL95555Handler a robust, efficient, and user-friendly component that follows established patterns from successful handlers in the HardFOC system.
