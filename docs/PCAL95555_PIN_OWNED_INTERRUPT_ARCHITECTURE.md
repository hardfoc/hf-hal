/**
 * @file PCAL95555_PIN_OWNED_INTERRUPT_ARCHITECTURE.md
 * @brief Pin-Owned Interrupt Data Architecture for PCAL95555Handler
 * @date 2025-01-24
 * @author HardFOC Team
 */

# PCAL95555Handler: Pin-Owned Interrupt Architecture

## **Problem with Handler-Owned Interrupt Data**

The initial registry implementation still had a design flaw:

### **❌ Handler-Stored Interrupt Data**
```cpp
// Handler stored interrupt data in arrays
std::array<InterruptCallback, 16> pin_callbacks_;       
std::array<void*, 16> pin_user_data_;                   
std::array<hf_gpio_interrupt_trigger_t, 16> pin_triggers_;

// Pin just delegates to handler
hf_gpio_err_t ConfigureInterrupt(...) {
    return parent_handler_->RegisterPinInterrupt(pin_, trigger, callback, user_data);
}
```

### **Problems:**
1. **Data Duplication**: Handler has arrays for data that belongs to pins
2. **Indirection**: Pin delegates to handler, handler stores data separately
3. **Coupling**: Handler needs to track individual pin interrupt state
4. **Inconsistency**: Pin objects exist but don't own their interrupt data
5. **Index Coupling**: Arrays indexed by pin number create tight coupling

## **✅ Solution: Pin-Owned Interrupt Data**

### **Pin Objects Own Their Interrupt Data**
```cpp
class Pcal95555GpioPin : public BaseGpio {
private:
    // Pin owns its interrupt configuration
    InterruptCallback interrupt_callback_ = nullptr;   
    void* interrupt_user_data_ = nullptr;             
    hf_gpio_interrupt_trigger_t interrupt_trigger_ = HF_GPIO_INTERRUPT_TRIGGER_NONE; 
    bool interrupt_enabled_ = false;                  
    
    friend class Pcal95555Handler;  // Handler can access for interrupt processing
};
```

### **Handler Queries Pins Directly**
```cpp
void ProcessInterrupts() noexcept {
    uint16_t status = pcal95555_driver_->getInterruptStatus();
    
    for (int pin = 0; pin < 16; pin++) {
        if (status & (1 << pin)) {
            // Get pin from registry
            auto gpio_pin = pin_registry_[pin];
            
            // Pin owns and provides its interrupt data
            if (gpio_pin && gpio_pin->interrupt_enabled_ && gpio_pin->interrupt_callback_) {
                gpio_pin->interrupt_callback_(
                    gpio_pin.get(), 
                    gpio_pin->interrupt_trigger_, 
                    gpio_pin->interrupt_user_data_
                );
            }
        }
    }
}
```

## **Architecture Benefits**

### **1. Proper Encapsulation**
- **Pin Owns Data**: Each pin object owns its interrupt configuration
- **Clean Separation**: Handler doesn't store pin-specific data
- **Single Responsibility**: Handler manages hardware, pins manage their state

### **2. Reduced Coupling**
- **No Index Arrays**: No `pin_callbacks_[16]` arrays indexed by pin number
- **Direct Access**: Handler directly queries pin objects for their data
- **Flexible Pins**: Pins can exist without interrupt configuration

### **3. Better Resource Management**
- **Pin Lifecycle**: Interrupt data lifetime tied to pin object lifetime
- **Automatic Cleanup**: When pin is destroyed, interrupt data is automatically cleaned up
- **No Orphaned Data**: No possibility of stale interrupt data in handler arrays

### **4. Improved Thread Safety**
- **Pin-Level Locking**: Each pin protects its own interrupt data with `pin_mutex_`
- **Fine-Grained Locking**: No single lock for all interrupt operations
- **Reduced Contention**: Multiple pins can configure interrupts concurrently

## **API Changes**

### **Pin Configuration (No Change to User)**
```cpp
auto pin = handler.CreateGpioPin(5);
pin->ConfigureInterrupt(
    HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
    [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
        // Callback receives the actual pin object
    },
    nullptr
);
```

### **Handler Registration (Internal Change)**
```cpp
// OLD: Handler stores interrupt data in arrays
hf_gpio_err_t RegisterPinInterrupt(pin, trigger, callback, user_data) {
    pin_callbacks_[pin] = callback;      // Handler stores data
    pin_user_data_[pin] = user_data;
    pin_triggers_[pin] = trigger;
}

// NEW: Handler stores data in pin object
hf_gpio_err_t RegisterPinInterrupt(pin, trigger, callback, user_data) {
    auto gpio_pin = pin_registry_[pin];  // Get pin from registry
    if (!gpio_pin) return GPIO_ERR_PIN_NOT_FOUND;
    
    // Pin stores its own data
    gpio_pin->interrupt_callback_ = callback;
    gpio_pin->interrupt_user_data_ = user_data;
    gpio_pin->interrupt_trigger_ = trigger;
    gpio_pin->interrupt_enabled_ = true;
}
```

## **Memory Layout Comparison**

### **❌ Old: Handler-Owned Arrays**
```
Pcal95555Handler:
├── pin_callbacks_[16]     // 16 * 8 bytes = 128 bytes
├── pin_user_data_[16]     // 16 * 8 bytes = 128 bytes  
├── pin_triggers_[16]      // 16 * 4 bytes = 64 bytes
└── pin_registry_[16]      // 16 * 8 bytes = 128 bytes
                           // Total: 448 bytes

Each Pcal95555GpioPin:
└── (no interrupt data)    // Pin doesn't own its interrupt state
```

### **✅ New: Pin-Owned Data**
```
Pcal95555Handler:
└── pin_registry_[16]      // 16 * 8 bytes = 128 bytes
                           // Total: 128 bytes (320 bytes saved!)

Each Pcal95555GpioPin:
├── interrupt_callback_    // 8 bytes
├── interrupt_user_data_   // 8 bytes
├── interrupt_trigger_     // 4 bytes
└── interrupt_enabled_     // 1 byte
                           // Total: 21 bytes per pin (only when pin created)
```

### **Memory Efficiency:**
- **Handler Size**: 320 bytes smaller (no interrupt arrays)
- **Pin Size**: 21 bytes larger per pin (only when pins are created)
- **Total Memory**: Much more efficient when few pins are created
- **Cache Locality**: Interrupt data co-located with pin data

## **Error Handling Improvements**

### **Requirement: Pin Must Exist**
```cpp
hf_gpio_err_t RegisterPinInterrupt(...) {
    auto gpio_pin = pin_registry_[pin];
    if (!gpio_pin) {
        return GPIO_ERR_PIN_NOT_FOUND;  // Pin must be created first
    }
    // Configure interrupt on existing pin
}
```

This prevents interrupt configuration on non-existent pins, improving API safety.

## **Comparison with Other Patterns**

### **Similar to BaseGpio Pattern**
```cpp
class BaseGpio {
private:
    hf_gpio_direction_t current_direction_;     // Pin owns its direction
    hf_gpio_active_state_t current_active_state_; // Pin owns its polarity
    // ...
};
```

### **Consistent with Object-Oriented Principles**
- **Encapsulation**: Pin owns its state data
- **Single Responsibility**: Each class has clear responsibilities
- **Data Locality**: Related data stored together

## **Performance Analysis**

### **Interrupt Processing Performance**
```cpp
// OLD: Array lookup + indirection
void ProcessInterrupts() {
    for (int pin = 0; pin < 16; pin++) {
        if ((status & (1 << pin)) && pin_callbacks_[pin]) {  // Array lookup
            pin_callbacks_[pin](...);  // Function pointer call
        }
    }
}

// NEW: Direct pin access
void ProcessInterrupts() {
    for (int pin = 0; pin < 16; pin++) {
        if (status & (1 << pin)) {
            auto gpio_pin = pin_registry_[pin];  // Single array lookup
            if (gpio_pin && gpio_pin->interrupt_enabled_) {  // Direct member access
                gpio_pin->interrupt_callback_(...);  // Function pointer call
            }
        }
    }
}
```

**Performance**: Essentially identical, but new version has better cache locality.

## **Migration Impact**

### **Breaking Changes: None**
- User API remains exactly the same
- Internal implementation changes only
- All existing code continues to work

### **Internal Changes**
- Handler no longer stores interrupt arrays
- Pin objects store their own interrupt data
- `friend class Pcal95555Handler` allows handler access to pin data

## **Conclusion**

Pin-owned interrupt data provides:

✅ **Better Encapsulation**: Pins own their interrupt state  
✅ **Reduced Memory**: No fixed-size arrays in handler  
✅ **Improved Locality**: Interrupt data co-located with pin data  
✅ **Cleaner Design**: Handler doesn't store pin-specific data  
✅ **Better Safety**: Must create pin before configuring interrupt  
✅ **Flexible Threading**: Pin-level mutex granularity  

This architecture follows object-oriented principles and provides a cleaner, more efficient implementation while maintaining the same user-facing API.
