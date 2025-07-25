# GPIO Manager Refactor Summary

## Overview

The GPIO Manager has been completely refactored to provide a modern, extensible, and comprehensive GPIO management system. This refactor introduces string-based pin identification, complete BaseGpio function coverage, smart pin categorization, and proper electrical configuration handling.

## Key Features

### 1. String-Based Pin Identification
- **Extensible**: Use any string identifier for GPIO pins
- **Runtime Registration**: Register custom GPIOs dynamically
- **No Enum Limitations**: Break free from compile-time enum constraints
- **Plugin Support**: Perfect for user plugins and external configurations

### 2. Complete BaseGpio Function Coverage
- **Full API Routing**: Every BaseGpio function available through string-based API
- **Consistent Interface**: Unified access to all GPIO operations
- **Error Handling**: Comprehensive error codes and diagnostics
- **Thread Safety**: All operations are thread-safe

### 3. Smart Pin Categorization
- **CORE**: System/reserved pins (XTAL, BOOT_SEL, JTAG) - skipped by GpioManager
- **COMM**: Communication pins (SPI, I2C, UART) - left for peripheral drivers
- **GPIO**: Available for GPIO operations - automatically registered
- **USER**: User-defined pins - always registered

### 4. Proper Electrical Configuration Handling
- **Primitive Configuration Fields**: Pin mapping uses only primitive types (bool, uint8_t, uint32_t)
- **Enum-Based Configuration**: Uses local enum classes for chip types and pin categories
- **Pull Resistor Configuration**: `has_pull` and `pull_is_up` fields for proper pull configuration
- **Output Mode Configuration**: `is_push_pull` field for push-pull vs open-drain output
- **Logic Inversion**: `is_inverted` field for proper active state handling
- **Current Limits**: `max_current_ma` field for electrical safety
- **Independent Design**: Pin configuration file has no external dependencies

### 5. Multi-Device Support
- **Unit/Device Numbers**: Support for multiple PCAL95555 units and TMC9660 devices
- **PCAL95555 Multi-Unit**: Each PCAL95555 unit can have different pin configurations
- **TMC9660 Multi-Device**: Support for onboard (device 0) and external devices (devices 2-3)
- **Device-Aware Creation**: GPIO creation methods accept unit/device numbers
- **Handler Routing**: Automatic routing to correct device handler based on unit/device number

### 6. Handler-Aware GPIO Creation
- **Proper Ownership**: Handlers own their GPIO instances
- **Factory Methods**: Use handler factory methods for GPIO creation
- **Lazy Initialization**: Handlers initialized only when needed
- **Multi-Chip Support**: ESP32, PCAL95555, TMC9660 integration

### 7. Advanced Features
- **Batch Operations**: Optimized multi-pin operations
- **Interrupt Support**: Full interrupt configuration and handling
- **Statistics & Diagnostics**: Comprehensive monitoring and health checks
- **Error Tracking**: Detailed error messages and diagnostics
- **Thread Safety**: RtosMutex-based protection for embedded systems

## Architecture

### Handler Ownership Hierarchy

The system follows a proper ownership hierarchy to prevent duplicate handlers and ensure clean architecture:

- **MotorController**: Sole owner of all `Tmc9660Handler` instances (`unique_ptr`)
- **GpioManager**: Consumer that gets access via raw pointers from MotorController
- **PCAL95555**: Owned and managed by `GpioManager` directly (`unique_ptr`)
- **ESP32**: Direct hardware access, no handler needed

**Ownership Model:**
- **MotorController** creates and owns `Tmc9660Handler` instances with `unique_ptr`
- **GpioManager** gets access via `MotorController::handler()` which returns raw pointers
- **Handler lifetime** is tied to MotorController lifecycle
- **No shared ownership** - clear single responsibility

This ensures:
- **Single Source of Truth**: Only one `Tmc9660Handler` per device
- **Proper Dependencies**: GpioManager depends on MotorController, not vice versa
- **Clean Separation**: Each manager owns its specific hardware handlers
- **No Duplicate Handlers**: Prevents resource conflicts and initialization issues
- **Clear Lifecycle**: Predictable destruction when MotorController is destroyed
- **No Reference Counting**: Efficient resource management without shared_ptr overhead

### Pin Configuration Independence

The pin configuration file (`hf_functional_pin_config_vortex_v1.hpp`) is now completely independent of external types and uses local enum classes:

```cpp
// Local enum classes for type safety
enum class HfPinCategory : uint8_t {
    PIN_CATEGORY_CORE = 0,    ///< System/core pins (skip GPIO registration)
    PIN_CATEGORY_COMM = 1,    ///< Communication pins (skip GPIO registration)  
    PIN_CATEGORY_GPIO = 2,    ///< Available for GPIO operations
    PIN_CATEGORY_USER = 3     ///< User-defined pins (always register)
};

enum class HfGpioChipType : uint8_t {
    ESP32_INTERNAL = 0,
    PCAL95555_EXPANDER,
    TMC9660_CONTROLLER
};

enum class HfAdcChipType : uint8_t {
    TMC9660_CONTROLLER = 0
};
```

```cpp
struct HfGpioMapping {
    uint8_t functional_pin;    ///< Functional pin identifier (enum value)
    uint8_t chip_type;         ///< Hardware chip identifier (enum value)
    uint8_t physical_pin;      ///< Physical pin number on the chip
    uint8_t unit_number;       ///< Unit/device number (0=first unit, 1=second unit, etc.)
    bool is_inverted;          ///< Whether pin logic is inverted
    bool has_pull;             ///< Whether pin has any pull resistor
    bool pull_is_up;           ///< If has_pull=true: true=pull-up, false=pull-down
    bool is_push_pull;         ///< Output mode: true=push-pull, false=open-drain
    uint32_t max_current_ma;   ///< Maximum current in milliamps
    uint8_t category;          ///< Pin category for registration control (enum value)
    std::string_view name;     ///< Human-readable pin name (self-documenting)
};
```

### XMACRO Configuration

The XMACRO now includes all electrical configuration fields:

```cpp
#define HF_FUNCTIONAL_GPIO_PIN_LIST(X) \
    X(ENUM_NAME, STRING_NAME, CATEGORY, CHIP_TYPE, PHYSICAL_PIN, UNIT_NUMBER, INVERTED, HAS_PULL, PULL_IS_UP, IS_PUSH_PULL, MAX_CURRENT_MA)

// Example usage with enum values:
X(XTAL_32K_P, "CORE_XTAL_32K_P", PIN_CATEGORY_CORE, ESP32_INTERNAL, 0, 0, false, false, false, true, 0)
X(PCAL_GPIO17, "GPIO_PCAL_GPIO17", PIN_CATEGORY_GPIO, PCAL95555_EXPANDER, 0, 0, false, false, false, true, 25)
```

### Manager Translation

The GpioManager translates primitive configuration values to driver enums:

```cpp
// Configure pull mode based on primitive values
if (has_pull) {
    hf_gpio_pull_mode_t pull_mode = pull_is_up ? 
        hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP : 
        hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
    gpio->SetPullMode(pull_mode);
} else {
    gpio->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
}

// Configure output mode based on primitive value
hf_gpio_output_mode_t output_mode = is_push_pull ? 
    hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL : 
    hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN;
gpio->SetOutputMode(output_mode);

// Multi-device support in CreateTmc9660GpioPin
Tmc9660Handler* handler = GetTmc9660Handler(device_index);
auto gpio = handler->CreateGpioPin(pin_id);

// Multi-unit support in CreatePcal95555GpioPin
auto gpio = pcal95555_handler_->CreateGpioPin(pin_id, unit_number);
```

## API Reference

### Basic Operations

```cpp
// Set pin state
bool Set(std::string_view name, bool value) noexcept;
bool SetActive(std::string_view name) noexcept;
bool SetInactive(std::string_view name) noexcept;

// Read pin state
bool Read(std::string_view name, bool& state) noexcept;
bool IsActive(std::string_view name, bool& active) noexcept;

// Toggle pin
bool Toggle(std::string_view name) noexcept;
```

### Pin Configuration

```cpp
// Direction configuration
hf_gpio_err_t SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept;
bool GetDirection(std::string_view name, hf_gpio_direction_t& direction) const noexcept;

// Pull resistor configuration
hf_gpio_err_t SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept;
bool GetPullMode(std::string_view name, hf_gpio_pull_mode_t& pull_mode) const noexcept;

// Output mode configuration
hf_gpio_err_t SetOutputMode(std::string_view name, hf_gpio_output_mode_t output_mode) noexcept;
bool GetOutputMode(std::string_view name, hf_gpio_output_mode_t& output_mode) const noexcept;
```

### Interrupt Support

```cpp
// Interrupt configuration
hf_gpio_err_t ConfigureInterrupt(std::string_view name, hf_gpio_interrupt_trigger_t trigger,
                                BaseGpio::InterruptCallback callback, void* user_data = nullptr) noexcept;
hf_gpio_err_t EnableInterrupt(std::string_view name) noexcept;
hf_gpio_err_t DisableInterrupt(std::string_view name) noexcept;
bool SupportsInterrupts(std::string_view name) const noexcept;
```

### Batch Operations

```cpp
// Batch operations for performance
GpioBatchResult BatchWrite(const GpioBatchOperation& operation) noexcept;
GpioBatchResult BatchRead(const std::vector<std::string_view>& pin_names) noexcept;
GpioBatchResult SetMultipleActive(const std::vector<std::string_view>& pin_names) noexcept;
GpioBatchResult SetMultipleInactive(const std::vector<std::string_view>& pin_names) noexcept;
```

### Registration and Management

```cpp
// GPIO registration
bool RegisterGpio(std::string_view name, std::shared_ptr<BaseGpio> gpio) noexcept;
std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;
bool Contains(std::string_view name) const noexcept;
size_t Size() const noexcept;
```

### Diagnostics and Health

```cpp
// System diagnostics
bool GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
bool GetSystemHealth(std::string& health_info) const noexcept;

// Pin statistics
bool GetStatistics(std::string_view name, BaseGpio::PinStatistics& statistics) const noexcept;
bool ResetStatistics(std::string_view name) noexcept;

// System operations
bool ResetAllPins() noexcept;
```

## Usage Examples

### Basic GPIO Operations

```cpp
auto& gpio_manager = GpioManager::GetInstance();

// Set pin to active
if (gpio_manager.SetActive("GPIO_WS2812_LED_DAT")) {
    std::cout << "LED activated" << std::endl;
}

// Read pin state
bool state;
if (gpio_manager.Read("GPIO_EXT_GPIO_CS_1", state)) {
    std::cout << "CS1 state: " << (state ? "ACTIVE" : "INACTIVE") << std::endl;
}

// Toggle pin
gpio_manager.Toggle("GPIO_WS2812_LED_DAT");
```

### Pin Configuration

```cpp
// Configure pin direction and electrical characteristics
gpio_manager.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
gpio_manager.SetPullMode("GPIO_EXT_GPIO_CS_1", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
gpio_manager.SetOutputMode("GPIO_EXT_GPIO_CS_1", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);

// Read back configuration
hf_gpio_direction_t direction;
hf_gpio_pull_mode_t pull_mode;
hf_gpio_output_mode_t output_mode;

gpio_manager.GetDirection("GPIO_EXT_GPIO_CS_1", direction);
gpio_manager.GetPullMode("GPIO_EXT_GPIO_CS_1", pull_mode);
gpio_manager.GetOutputMode("GPIO_EXT_GPIO_CS_1", output_mode);
```

### Interrupt Handling

```cpp
// Configure interrupt callback
auto interrupt_callback = [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
    std::cout << "Interrupt triggered!" << std::endl;
};

// Configure and enable interrupt
gpio_manager.ConfigureInterrupt("GPIO_PCAL_IMU_INT", 
                               hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
                               interrupt_callback);
gpio_manager.EnableInterrupt("GPIO_PCAL_IMU_INT");
```

### Batch Operations

```cpp
// Batch read multiple pins
std::vector<std::string_view> pins = {"GPIO_WS2812_LED_DAT", "GPIO_EXT_GPIO_CS_1"};
auto read_result = gpio_manager.BatchRead(pins);

if (read_result.AllSuccessful()) {
    for (size_t i = 0; i < read_result.pin_names.size(); ++i) {
        std::cout << read_result.pin_names[i] << ": " 
                  << (read_result.states[i] ? "ACTIVE" : "INACTIVE") << std::endl;
    }
}

// Batch write multiple pins
std::vector<bool> states = {true, false};
GpioBatchOperation write_operation(pins, states);
auto write_result = gpio_manager.BatchWrite(write_operation);
```

### User-Defined GPIO Registration

```cpp
// Create custom ESP32 GPIO
auto custom_gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(25));

// Configure the GPIO
custom_gpio->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
custom_gpio->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
custom_gpio->SetOutputMode(hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
custom_gpio->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH);

if (custom_gpio->Initialize()) {
    // Register with string name
    if (gpio_manager.RegisterGpio("USER_CUSTOM_LED", custom_gpio)) {
        // Use the custom GPIO
        gpio_manager.SetActive("USER_CUSTOM_LED");
    } else {
        std::cout << "Failed to register custom GPIO" << std::endl;
    }
}
```

### TMC9660 GPIO Access (via MotorController)

```cpp
// TMC9660 GPIOs are automatically created during GpioManager initialization
// The GpioManager gets handler access through MotorController

// Access TMC9660 GPIO pins through GpioManager (which uses MotorController internally)
if (gpio_manager.Set("GPIO_TMC9660_GPIO17", true)) {
    std::cout << "Set TMC9660 GPIO17 to active" << std::endl;
}

// The MotorController owns the Tmc9660Handler, GpioManager just accesses it
auto& motor_controller = MotorController::GetInstance();
auto tmc_handler = motor_controller.handler(0); // Get onboard TMC9660 handler
if (tmc_handler) {
    // Direct access to TMC9660 handler for advanced operations
    auto gpio = tmc_handler->gpio(17); // Get GPIO17 wrapper
    gpio.setDirection(true); // Set as output
}
```

### Multi-Device Support Examples

```cpp
// PCAL95555 Multi-Unit Support
// Each PCAL95555 unit can have different pin configurations
// Unit 0: Main PCAL95555 on I2C bus
// Unit 1: Secondary PCAL95555 on same I2C bus (different address)

// Access pins from different PCAL95555 units
if (gpio_manager.Set("GPIO_PCAL_GPIO17", true)) {  // Unit 0 (default)
    std::cout << "Set PCAL95555 Unit 0 GPIO17 to active" << std::endl;
}

// TMC9660 Multi-Device Support
// Device 0: Onboard TMC9660 (SPI2_CS_TMC9660)
// Device 2: External TMC9660 (EXT_GPIO_CS_1)
// Device 3: External TMC9660 (EXT_GPIO_CS_2)

// Access GPIOs from different TMC9660 devices
if (gpio_manager.Set("GPIO_TMC9660_GPIO17", true)) {  // Device 0 (onboard)
    std::cout << "Set onboard TMC9660 GPIO17 to active" << std::endl;
}

// Direct access to specific devices through MotorController
auto& motor_controller = MotorController::GetInstance();
auto onboard_handler = motor_controller.handler(0);  // Onboard device
auto external_handler = motor_controller.handler(2);  // External device 1

if (onboard_handler && external_handler) {
    // Both devices are available
    auto onboard_gpio = onboard_handler->gpio(17);
    auto external_gpio = external_handler->gpio(18);
    
    onboard_gpio.setDirection(true);   // Set onboard GPIO17 as output
    external_gpio.setDirection(false); // Set external GPIO18 as input
}
```

### System Diagnostics

```cpp
// Get system diagnostics
GpioSystemDiagnostics diagnostics;
if (gpio_manager.GetSystemDiagnostics(diagnostics)) {
    std::cout << "System Healthy: " << (diagnostics.system_healthy ? "YES" : "NO") << std::endl;
    std::cout << "Total Pins: " << diagnostics.total_pins_registered << std::endl;
    std::cout << "Total Operations: " << diagnostics.total_operations << std::endl;
    std::cout << "Successful Operations: " << diagnostics.successful_operations << std::endl;
    std::cout << "Failed Operations: " << diagnostics.failed_operations << std::endl;
}

// Get pin statistics
BaseGpio::PinStatistics stats;
if (gpio_manager.GetStatistics("GPIO_WS2812_LED_DAT", stats)) {
    std::cout << "Total Operations: " << stats.totalOperations << std::endl;
    std::cout << "State Changes: " << stats.stateChanges << std::endl;
}
```

## Migration Guide

### From Enum-Based to String-Based

**Old (Enum-based):**
```cpp
// Old enum-based approach
enum class GpioID { LED_PIN, BUTTON_PIN, CS_PIN };
gpio_manager.set(GpioID::LED_PIN, true);
```

**New (String-based):**
```cpp
// New string-based approach
gpio_manager.Set("GPIO_WS2812_LED_DAT", true);
gpio_manager.Set("USER_CUSTOM_BUTTON", false);
```

### From Snake_case to CamelCase

**Old (snake_case):**
```cpp
gpio_manager.set_active("GPIO_LED");
gpio_manager.read_state("GPIO_BUTTON", state);
gpio_manager.set_direction("GPIO_CS", direction);
```

**New (camelCase):**
```cpp
gpio_manager.SetActive("GPIO_LED");
gpio_manager.Read("GPIO_BUTTON", state);
gpio_manager.SetDirection("GPIO_CS", direction);
```

### Electrical Configuration

**Old (limited configuration):**
```cpp
// Limited pull-up configuration
gpio_manager.set_pullup("GPIO_PIN", true);
```

**New (comprehensive electrical configuration):**
```cpp
// Full electrical configuration
gpio_manager.SetPullMode("GPIO_PIN", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
gpio_manager.SetOutputMode("GPIO_PIN", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
gpio_manager.SetDirection("GPIO_PIN", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
```

## Benefits

### 1. Extensibility
- **Runtime Registration**: Add GPIOs without recompiling
- **Plugin Support**: Perfect for user plugins and external configurations
- **Dynamic Configuration**: Load pin configurations from files or network

### 2. Maintainability
- **Single Source of Truth**: XMACRO generates all pin data
- **Primitive Configuration**: No external type dependencies
- **Clear Separation**: Pin definition independent of driver implementation

### 3. Performance
- **Batch Operations**: Optimized multi-pin operations
- **String View**: Efficient string handling without allocations
- **Lazy Initialization**: Handlers initialized only when needed

### 4. Safety
- **Thread Safety**: All operations protected by mutexes
- **Error Handling**: Comprehensive error codes and diagnostics
- **Validation**: Pin name validation and reserved prefix protection

### 5. Electrical Safety
- **Proper Configuration**: All pins configured with correct electrical characteristics
- **Current Limits**: Maximum current specifications for safety
- **Pull Resistors**: Proper pull-up/pull-down configuration
- **Output Modes**: Correct push-pull vs open-drain configuration

## Conclusion

The GPIO Manager refactor provides a modern, extensible, and comprehensive GPIO management system that:

1. **Eliminates enum limitations** with string-based identification
2. **Provides complete BaseGpio coverage** through unified API
3. **Implements smart pin categorization** for proper resource management
4. **Ensures proper electrical configuration** with primitive configuration fields
5. **Maintains independence** from external type dependencies
6. **Offers advanced features** like batch operations and diagnostics
7. **Guarantees thread safety** for embedded systems
8. **Supports user extensibility** through dynamic registration

This refactor transforms the GPIO system from a limited enum-based approach to a powerful, extensible platform that can handle complex embedded applications while maintaining safety, performance, and maintainability. 