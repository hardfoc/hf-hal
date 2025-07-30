# Hardware Filtering Implementation Report

## 📋 Overview

This report documents the implementation of hardware filtering in the GPIO Manager and ADC Manager to ensure that only pins and channels that are actually available on the current hardware configuration are registered during initialization. This prevents attempts to access non-existent hardware resources and provides clear logging about what is being registered or skipped.

## 🎯 Current Hardware Configuration

Based on the pin configuration in `hf_functional_pin_config_vortex_v1.hpp`, the current Vortex V1 board has the following hardware configuration:

### **GPIO Hardware Configuration**
| Chip Type | Unit | Bank | Status | Description |
|-----------|------|------|--------|-------------|
| ESP32 Internal | 0 | 0 | ✅ Available | Main microcontroller GPIO pins |
| PCAL95555 Expander | 0 | 0 | ✅ Available | Single GPIO expander on board |
| TMC9660 Controller | 0 | 0 | ✅ Available | Single motor controller on board |

### **ADC Hardware Configuration**
| Chip Type | Unit | ADC Unit | Status | Description |
|-----------|------|----------|--------|-------------|
| ESP32 Internal | 0 | 0 | ❌ Disabled | Currently not used (as per user requirements) |
| TMC9660 Controller | 0 | 0 | ✅ Available | Single motor controller ADC |

## 🔧 Implementation Details

### **1. GPIO Manager Hardware Filtering**

#### **Location**: `component-handlers/GpioManager.cpp` lines 640-680

#### **Filtering Logic**:
```cpp
// Filter pins based on current hardware configuration
// Only register pins that are actually available on the current board
bool should_register = false;

switch (static_cast<HfGpioChipType>(mapping.chip_type)) {
    case HfGpioChipType::ESP32_INTERNAL:
        // ESP32 internal: only unit 0, bank 0
        should_register = (mapping.chip_unit == 0 && mapping.gpio_bank == 0);
        break;
        
    case HfGpioChipType::PCAL95555_EXPANDER:
        // PCAL95555: only unit 0, bank 0 (single expander on board)
        should_register = (mapping.chip_unit == 0 && mapping.gpio_bank == 0);
        break;
        
    case HfGpioChipType::TMC9660_CONTROLLER:
        // TMC9660: only unit 0, bank 0 (single controller on board)
        should_register = (mapping.chip_unit == 0 && mapping.gpio_bank == 0);
        break;
        
    default:
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER);
        continue;
}
```

#### **Logging Implementation**:
```cpp
if (!should_register) {
    Logger::GetInstance().Info("GpioManager", "Skipping GPIO %.*s (chip_unit=%d, bank=%d) - not available on current hardware", 
                       static_cast<int>(mapping.name.length()), mapping.name.data(), 
                       mapping.chip_unit, mapping.gpio_bank);
    continue;
}

// ... GPIO creation ...

if (gpio) {
    RegisterGpio(mapping.name, std::move(gpio));
    Logger::GetInstance().Info("GpioManager", "Registered GPIO %.*s (chip_unit=%d, bank=%d)", 
                       static_cast<int>(mapping.name.length()), mapping.name.data(), 
                       mapping.chip_unit, mapping.gpio_bank);
} else {
    Logger::GetInstance().Error("GpioManager", "Failed to create GPIO %.*s (chip_unit=%d, bank=%d)", 
                    static_cast<int>(mapping.name.length()), mapping.name.data(), 
                    mapping.chip_unit, mapping.gpio_bank);
}
```

### **2. ADC Manager Hardware Filtering**

#### **Location**: `component-handlers/AdcManager.cpp` lines 995-1030

#### **Filtering Logic**:
```cpp
// Filter channels based on current hardware configuration
// Only register channels that are actually available on the current board
bool should_register = false;

switch (static_cast<HfAdcChipType>(mapping.chip_type)) {
    case HfAdcChipType::ESP32_INTERNAL:
        // ESP32 internal: currently not used (as per user requirements)
        should_register = false;
        break;
        
    case HfAdcChipType::TMC9660_CONTROLLER:
        // TMC9660: only unit 0, ADC unit 0 (single controller on board)
        should_register = (mapping.chip_unit == 0 && mapping.adc_unit == 0);
        break;
        
    default:
        Logger::GetInstance().Error("AdcManager", "Unknown chip type %d for channel %.*s", 
                       mapping.chip_type, static_cast<int>(channel_name.length()), channel_name.data());
        continue;
}
```

#### **Logging Implementation**:
```cpp
if (!should_register) {
    Logger::GetInstance().Info("AdcManager", "Skipping ADC channel %.*s (chip_unit=%d, adc_unit=%d) - not available on current hardware", 
                       static_cast<int>(channel_name.length()), channel_name.data(), 
                       mapping.chip_unit, mapping.adc_unit);
    continue;
}

// ... ADC wrapper creation ...

if (tmc9660_wrapper) {
    hf_adc_err_t result = RegisterChannel(channel_name, std::move(tmc9660_wrapper));
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Info("AdcManager", "Registered TMC9660 channel %.*s (chip_unit=%d, adc_unit=%d)", 
                       static_cast<int>(channel_name.length()), channel_name.data(), 
                       mapping.chip_unit, mapping.adc_unit);
        registered_count++;
    }
}
```

## 📊 Expected Registration Results

### **GPIO Pins Expected to Register**

#### **ESP32 Internal GPIOs (Unit 0, Bank 0)**:
- `GPIO_EXT_GPIO_CS_1` - External GPIO CS 1
- `GPIO_EXT_GPIO_CS_2` - External GPIO CS 2

#### **PCAL95555 GPIOs (Unit 0, Bank 0)**:
- `GPIO_PCAL_FAULT_STATUS` - Fault status pin
- `GPIO_PCAL_DRV_EN` - Driver enable pin
- `GPIO_PCAL_RST_CTRL` - Reset control pin
- `GPIO_PCAL_PWR_GOOD` - Power good pin
- `GPIO_PCAL_CAN_HS_STB_OP` - CAN high-speed standby pin
- `GPIO_PCAL_IMU_BOOT` - IMU boot pin
- `GPIO_PCAL_IMU_INT` - IMU interrupt pin
- `GPIO_PCAL_IMU_RST` - IMU reset pin
- `GPIO_PCAL_SPI_COMM_EN` - SPI communication enable pin
- `GPIO_PCAL_WAKE_CTRL` - Wake control pin

#### **TMC9660 GPIOs (Unit 0, Bank 0)**:
- `GPIO_TMC_GPIO17` - TMC9660 GPIO 17
- `GPIO_TMC_GPIO18` - TMC9660 GPIO 18

### **ADC Channels Expected to Register**

#### **TMC9660 ADC Channels (Unit 0, ADC Unit 0)**:
- `ADC_TMC9660_AIN0` - ADC Input 0 (Reserved)
- `ADC_TMC9660_AIN1` - ADC Input 1 (Reserved)
- `ADC_TMC9660_AIN2` - ADC Input 2 (Reserved)
- `ADC_TMC9660_AIN3` - ADC Input 3 (Temperature Sensor)
- `TMC9660_CURRENT_I0` - Current Sense I0
- `TMC9660_CURRENT_I1` - Current Sense I1
- `TMC9660_CURRENT_I2` - Current Sense I2
- `TMC9660_CURRENT_I3` - Current Sense I3
- `TMC9660_SUPPLY_VOLTAGE` - Supply Voltage
- `TMC9660_DRIVER_VOLTAGE` - Driver Voltage
- `TMC9660_CHIP_TEMPERATURE` - Chip Temperature
- `TMC9660_EXTERNAL_TEMPERATURE` - External Temperature
- `TMC9660_MOTOR_CURRENT` - Motor Current
- `TMC9660_MOTOR_VELOCITY` - Motor Velocity
- `TMC9660_MOTOR_POSITION` - Motor Position

## 🚫 Pins/Channels Expected to be Skipped

### **GPIO Pins Skipped**:
- Any pins with `chip_unit != 0` (future multi-device support)
- Any pins with `gpio_bank != 0` (future multi-bank support)
- CORE and COMM category pins (filtered by category)

### **ADC Channels Skipped**:
- ESP32 internal ADC channels (filtered by chip type)
- Any channels with `chip_unit != 0` (future multi-device support)
- Any channels with `adc_unit != 0` (future multi-ADC support)

## 🔄 Future Extensibility

### **Multi-Device Support**
The filtering logic is designed to be easily extensible for future hardware configurations:

```cpp
// Example: Future multi-device configuration
case HfGpioChipType::PCAL95555_EXPANDER:
    // Support multiple PCAL95555 expanders
    should_register = (mapping.chip_unit < MAX_PCAL95555_DEVICES && mapping.gpio_bank == 0);
    break;
```

### **Multi-Bank Support**
```cpp
// Example: Future multi-bank configuration
case HfGpioChipType::ESP32_INTERNAL:
    // Support multiple GPIO banks
    should_register = (mapping.chip_unit == 0 && mapping.gpio_bank < MAX_ESP32_GPIO_BANKS);
    break;
```

### **Configuration-Driven Filtering**
Future enhancements could include:
- Configuration file for hardware setup
- Runtime hardware detection
- Dynamic filtering based on detected hardware

## 📈 Benefits

### **1. Hardware Safety**
- ✅ Prevents attempts to access non-existent hardware
- ✅ Avoids runtime errors from invalid hardware access
- ✅ Ensures system stability

### **2. Clear Logging**
- ✅ Detailed information about what is being registered
- ✅ Clear indication of what is being skipped and why
- ✅ Easy debugging of hardware configuration issues

### **3. Future-Proof Design**
- ✅ Easy to extend for new hardware configurations
- ✅ Maintains backward compatibility
- ✅ Supports gradual hardware expansion

### **4. Performance Optimization**
- ✅ Only creates handlers for available hardware
- ✅ Reduces memory usage
- ✅ Faster initialization

## 🎯 Configuration Summary

### **Current Hardware Limits**:
```cpp
// GPIO Configuration
#define MAX_ESP32_UNITS 1
#define MAX_ESP32_GPIO_BANKS 1
#define MAX_PCAL95555_UNITS 1
#define MAX_PCAL95555_GPIO_BANKS 1
#define MAX_TMC9660_UNITS 1
#define MAX_TMC9660_GPIO_BANKS 1

// ADC Configuration
#define MAX_ESP32_ADC_UNITS 0  // Currently disabled
#define MAX_TMC9660_UNITS 1
#define MAX_TMC9660_ADC_UNITS 1
```

### **Filtering Rules**:
1. **ESP32 Internal**: Only unit 0, bank 0
2. **PCAL95555**: Only unit 0, bank 0
3. **TMC9660 GPIO**: Only unit 0, bank 0
4. **TMC9660 ADC**: Only unit 0, ADC unit 0
5. **ESP32 ADC**: Currently disabled

## ✅ Verification

### **Expected Log Output**:
```
[INFO] GpioManager: Registered GPIO GPIO_EXT_GPIO_CS_1 (chip_unit=0, bank=0)
[INFO] GpioManager: Registered GPIO GPIO_EXT_GPIO_CS_2 (chip_unit=0, bank=0)
[INFO] GpioManager: Registered GPIO GPIO_PCAL_FAULT_STATUS (chip_unit=0, bank=0)
[INFO] GpioManager: Registered GPIO GPIO_PCAL_DRV_EN (chip_unit=0, bank=0)
...
[INFO] AdcManager: Registered TMC9660 channel ADC_TMC9660_AIN3 (chip_unit=0, adc_unit=0)
[INFO] AdcManager: Registered TMC9660 channel TMC9660_CURRENT_I0 (chip_unit=0, adc_unit=0)
...
```

### **Expected Skip Messages**:
```
[INFO] GpioManager: Skipping GPIO GPIO_FUTURE_DEVICE (chip_unit=1, bank=0) - not available on current hardware
[INFO] AdcManager: Skipping ADC channel ADC_FUTURE_CHANNEL (chip_unit=1, adc_unit=0) - not available on current hardware
```

## 🎯 Conclusion

The hardware filtering implementation ensures that the GPIO Manager and ADC Manager only register pins and channels that are actually available on the current Vortex V1 hardware configuration. This provides:

- **Hardware Safety**: Prevents invalid hardware access
- **Clear Logging**: Detailed information about registration process
- **Future Extensibility**: Easy to extend for new hardware
- **Performance Optimization**: Only creates necessary handlers

The implementation is robust, well-documented, and ready for production use while maintaining flexibility for future hardware expansions.

---

*This report documents the complete implementation of hardware filtering in the GPIO Manager and ADC Manager to ensure only available hardware resources are registered.* 