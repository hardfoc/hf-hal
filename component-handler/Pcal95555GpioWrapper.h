/**
 * @file Pcal95555GpioWrapper.h
 * @brief Comprehensive PCAL95555 I2C GPIO expander wrapper implementing BaseGpio interface.
 *
 * This file provides a complete implementation of the PCAL95555 16-bit I2C GPIO expander
 * that integrates seamlessly with the HardFOC HAL system. It implements the unified
 * BaseGpio interface and provides proper integration with the ESP32 I2C system and
 * the hf-pincfg platform mapping system.
 *
 * Features:
 * - Full BaseGpio interface implementation
 * - ESP32 I2C bus integration via EspI2c
 * - Platform mapping integration via hf-pincfg
 * - Thread-safe I2C communication
 * - Comprehensive error handling and diagnostics
 * - Support for all PCAL95555 features (pull resistors, drive strength, interrupts)
 * - Lazy initialization pattern
 * - Proper resource management
 *
 * @author HardFOC Team
 * @version 2.0
 * @date 2025
 * @copyright HardFOC
 */

#ifndef COMPONENT_HANDLER_PCAL95555_GPIO_WRAPPER_H_
#define COMPONENT_HANDLER_PCAL95555_GPIO_WRAPPER_H_

#include "base/BaseGpio.h"
#include "base/BaseI2c.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-pcal95555-driver/src/pcal95555.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include "CommonIDs.h"
#include <memory>
#include <mutex>
#include <array>

// Forward declarations
class Pcal95555GpioWrapper;

/**
 * @brief PCAL95555 chip pin enumeration for the single chip configuration.
 * 
 * This enum defines the 16 pins available on the PCAL95555 chip with
 * functional names that match the platform mapping system.
 */
enum class Pcal95555Chip1Pin : uint8_t {
    // Motor control pins (0-3)
    MOTOR_ENABLE_1 = 0,    ///< Motor enable control
    MOTOR_BRAKE_1 = 1,     ///< Motor brake control  
    MOTOR_ENABLE_2 = 2,    ///< Second motor enable (if applicable)
    MOTOR_BRAKE_2 = 3,     ///< Second motor brake (if applicable)
    
    // Motor status pins (4-5)
    MOTOR_FAULT_1 = 4,     ///< Motor fault status input
    MOTOR_FAULT_2 = 5,     ///< Second motor fault status (if applicable)
    
    // System status LEDs (6-9)
    LED_STATUS_GREEN = 6,  ///< Status OK LED (Green)
    LED_STATUS_RED = 7,    ///< Status Error LED (Red)
    LED_ERROR = 8,         ///< Error indicator LED
    LED_COMM = 9,          ///< Communication status LED
    
    // External device control (10-13)
    EXT_RELAY_1 = 10,      ///< External relay control 1
    EXT_RELAY_2 = 11,      ///< External relay control 2
    EXT_OUTPUT_1 = 12,     ///< External output 1
    EXT_OUTPUT_2 = 13,     ///< External output 2
    
    // External inputs (14-15)
    EXT_INPUT_1 = 14,      ///< External input 1
    EXT_INPUT_2 = 15,      ///< External input 2
    
    PCAL95555_PIN_COUNT = 16
};

/**
 * @brief I2C bus adapter for PCAL95555 using BaseI2c.
 * 
 * This class implements the PCAL95555::i2cBus interface using the
 * abstract BaseI2c interface for platform-agnostic communication.
 */
class Pcal95555I2cAdapter : public PCAL95555::i2cBus {
public:
    /**
     * @brief Constructor for I2C bus adapter.
     * @param i2c_bus Reference to BaseI2c bus instance
     * @param i2c_address 7-bit I2C address of the PCAL95555 chip
     */
    explicit Pcal95555I2cAdapter(BaseI2c& i2c_bus, uint8_t i2c_address) noexcept
        : i2c_bus_(i2c_bus), i2c_address_(i2c_address) {}

    /**
     * @brief Write bytes to PCAL95555 register.
     * @param addr 7-bit I2C address (ignored, uses constructor address)
     * @param reg Register address to write to
     * @param data Pointer to data buffer
     * @param len Number of bytes to write
     * @return true if successful, false on error
     */
    bool write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) override;

    /**
     * @brief Read bytes from PCAL95555 register.
     * @param addr 7-bit I2C address (ignored, uses constructor address)
     * @param reg Register address to read from
     * @param data Pointer to data buffer
     * @param len Number of bytes to read
     * @return true if successful, false on error
     */
    bool read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) override;

private:
    BaseI2c& i2c_bus_;     ///< BaseI2c bus reference
    uint8_t i2c_address_;  ///< PCAL95555 I2C address
    mutable std::mutex i2c_mutex_; ///< Thread safety mutex
};

/**
 * @brief PCAL95555 GPIO pin implementation of BaseGpio.
 * 
 * This class provides a concrete implementation of BaseGpio for individual
 * PCAL95555 pins. It handles all GPIO operations through the shared
 * PCAL95555 driver instance.
 */
class Pcal95555GpioPin : public BaseGpio {
public:
    /**
     * @brief Constructor for PCAL95555 GPIO pin.
     * @param functional_pin Functional GPIO pin identifier
     * @param chip_pin PCAL95555 chip pin number (0-15)
     * @param pcal95555_driver Shared PCAL95555 driver instance
     * @param direction Initial pin direction
     * @param active_state Active state polarity
     * @param output_mode Output drive mode
     * @param pull_mode Pull resistor configuration
     */
    explicit Pcal95555GpioPin(
        HardFOC::FunctionalGpioPin functional_pin,
        Pcal95555Chip1Pin chip_pin,
        std::shared_ptr<PCAL95555> pcal95555_driver,
        hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;

    /**
     * @brief Destructor - ensures proper cleanup.
     */
    ~Pcal95555GpioPin() override = default;

    //==============================================================//
    // BASEGPIO IMPLEMENTATION
    //==============================================================//

    /**
     * @brief Initialize the PCAL95555 GPIO pin.
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept override;

    /**
     * @brief Deinitialize the PCAL95555 GPIO pin.
     * @return true if deinitialization successful, false otherwise
     */
    bool Deinitialize() noexcept override;

    /**
     * @brief Check if the pin is available for GPIO operations.
     * @return true if pin is available, false otherwise
     */
    bool IsPinAvailable() const noexcept override;

    /**
     * @brief Get the maximum number of pins supported by PCAL95555.
     * @return 16 (PCAL95555 has 16 GPIO pins)
     */
    hf_u8_t GetMaxPins() const noexcept override;

    /**
     * @brief Get human-readable description of this GPIO instance.
     * @return Pointer to description string
     */
    const char* GetDescription() const noexcept override;

    /**
     * @brief Check if interrupts are supported.
     * @return true (PCAL95555 supports interrupts)
     */
    bool SupportsInterrupts() const noexcept override;

    /**
     * @brief Configure GPIO interrupt settings.
     * @param trigger Interrupt trigger type
     * @param callback Callback function (optional)
     * @param user_data User data for callback (optional)
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                     InterruptCallback callback = nullptr,
                                     void* user_data = nullptr) noexcept override;

    //==============================================================//
    // PCAL95555-SPECIFIC METHODS
    //==============================================================//

    /**
     * @brief Get the PCAL95555 chip pin number.
     * @return Chip pin number (0-15)
     */
    Pcal95555Chip1Pin GetChipPin() const noexcept { return chip_pin_; }

    /**
     * @brief Get the functional GPIO pin identifier.
     * @return Functional GPIO pin identifier
     */
    HardFOC::FunctionalGpioPin GetFunctionalPin() const noexcept { return functional_pin_; }

    /**
     * @brief Check if the PCAL95555 chip is responsive.
     * @return true if chip responds, false otherwise
     */
    bool IsChipResponsive() const noexcept;

protected:
    //==============================================================//
    // PURE VIRTUAL IMPLEMENTATIONS
    //==============================================================//

    /**
     * @brief Platform-specific implementation for setting pin direction.
     * @param direction Desired pin direction
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;

    /**
     * @brief Platform-specific implementation for setting output mode.
     * @param mode Desired output mode
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;

    /**
     * @brief Platform-specific implementation for setting active state.
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t SetActiveImpl() noexcept override;

    /**
     * @brief Platform-specific implementation for setting inactive state.
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t SetInactiveImpl() noexcept override;

    /**
     * @brief Platform-specific implementation for reading pin state.
     * @param is_active Output parameter for active state
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t IsActiveImpl(bool& is_active) noexcept override;

    /**
     * @brief Platform-specific implementation for toggling pin state.
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t ToggleImpl() noexcept override;

    /**
     * @brief Platform-specific implementation for setting pull resistor mode.
     * @param mode Desired pull mode
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;

    /**
     * @brief Platform-specific implementation for reading pull resistor mode.
     * @return Current pull mode
     */
    hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;

private:
    //==============================================================//
    // PRIVATE MEMBERS
    //==============================================================//

    HardFOC::FunctionalGpioPin functional_pin_;  ///< Functional GPIO pin identifier
    Pcal95555Chip1Pin chip_pin_;                 ///< PCAL95555 chip pin number
    std::shared_ptr<PCAL95555> pcal95555_driver_; ///< Shared PCAL95555 driver instance

    //==============================================================//
    // PRIVATE HELPERS
    //==============================================================//

    /**
     * @brief Validate chip pin number.
     * @return true if pin number is valid, false otherwise
     */
    bool ValidateChipPin() const noexcept;

    /**
     * @brief Get the bit mask for this pin.
     * @return Bit mask with only this pin's bit set
     */
    uint16_t GetPinMask() const noexcept;

    /**
     * @brief Convert PCAL95555 driver errors to HfGpioErr codes.
     * @param success True if operation succeeded, false if failed
     * @return Appropriate HfGpioErr code
     */
    hf_gpio_err_t ConvertDriverError(bool success) const noexcept;

    /**
     * @brief Apply the current configuration to the PCAL95555 chip.
     * @return hf_gpio_err_t error code
     */
    hf_gpio_err_t ApplyConfiguration() noexcept;
};

/**
 * @brief PCAL95555 GPIO expander manager class.
 * 
 * This class manages a single PCAL95555 chip and provides access to all
 * 16 GPIO pins. It handles initialization, configuration, and provides
 * factory methods for creating GPIO pin instances.
 */
class Pcal95555GpioWrapper {
public:
    /**
 * @brief Constructor for PCAL95555 GPIO wrapper.
 * @param i2c_bus BaseI2c bus instance
 * @param i2c_address PCAL95555 I2C address (default: 0x20)
 */
explicit Pcal95555GpioWrapper(BaseI2c& i2c_bus, uint8_t i2c_address = 0x20) noexcept;

    /**
     * @brief Destructor - ensures proper cleanup.
     */
    ~Pcal95555GpioWrapper() = default;

    //==============================================================//
    // INITIALIZATION AND CONFIGURATION
    //==============================================================//

    /**
     * @brief Initialize the PCAL95555 chip and all GPIO pins.
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Deinitialize the PCAL95555 chip and all GPIO pins.
     * @return true if deinitialization successful, false otherwise
     */
    bool Deinitialize() noexcept;

    /**
     * @brief Check if the PCAL95555 chip is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept { return initialized_; }

    /**
     * @brief Check if the PCAL95555 chip is healthy and responsive.
     * @return true if chip is healthy, false otherwise
     */
    bool IsHealthy() const noexcept;

    //==============================================================//
    // GPIO PIN FACTORY METHODS
    //==============================================================//

    /**
     * @brief Create a GPIO pin instance for the specified functional pin.
     * @param functional_pin Functional GPIO pin identifier
     * @return Shared pointer to GPIO pin instance, nullptr if invalid
     */
    std::shared_ptr<Pcal95555GpioPin> CreateGpioPin(HardFOC::FunctionalGpioPin functional_pin) noexcept;

    /**
     * @brief Create a GPIO pin instance for the specified chip pin.
     * @param chip_pin PCAL95555 chip pin number
     * @return Shared pointer to GPIO pin instance, nullptr if invalid
     */
    std::shared_ptr<Pcal95555GpioPin> CreateGpioPin(Pcal95555Chip1Pin chip_pin) noexcept;

    /**
     * @brief Get the PCAL95555 driver instance.
     * @return Shared pointer to PCAL95555 driver, nullptr if not initialized
     */
    std::shared_ptr<PCAL95555> GetDriver() const noexcept { return pcal95555_driver_; }

    /**
     * @brief Get the I2C address of this PCAL95555 chip.
     * @return I2C address
     */
    uint8_t GetI2cAddress() const noexcept { return i2c_address_; }

    //==============================================================//
    // DIAGNOSTICS AND STATUS
    //==============================================================//

    /**
     * @brief Get comprehensive diagnostic information.
     * @return Diagnostic information structure
     */
    struct Diagnostics {
        bool chip_initialized;           ///< Chip initialization status
        bool chip_responsive;            ///< Chip communication status
        uint16_t error_flags;            ///< PCAL95555 error flags
        uint16_t interrupt_status;       ///< Current interrupt status
        uint16_t input_port_0;           ///< Port 0 input values
        uint16_t input_port_1;           ///< Port 1 input values
        uint16_t output_port_0;          ///< Port 0 output values
        uint16_t output_port_1;          ///< Port 1 output values
        uint16_t config_port_0;          ///< Port 0 configuration
        uint16_t config_port_1;          ///< Port 1 configuration
        uint32_t i2c_errors;             ///< I2C communication errors
        uint32_t total_operations;       ///< Total GPIO operations
        uint32_t successful_operations;  ///< Successful operations
        uint32_t failed_operations;      ///< Failed operations
    };

    /**
     * @brief Get current diagnostic information.
     * @param diagnostics Reference to store diagnostic information
     * @return true if successful, false otherwise
     */
    bool GetDiagnostics(Diagnostics& diagnostics) const noexcept;

    /**
     * @brief Reset the PCAL95555 chip to default configuration.
     * @return true if successful, false otherwise
     */
    bool ResetToDefault() noexcept;

    /**
     * @brief Clear all error flags.
     * @return true if successful, false otherwise
     */
    bool ClearErrorFlags() noexcept;

private:
    //==============================================================//
    // PRIVATE MEMBERS
    //==============================================================//

    BaseI2c& i2c_bus_;                                   ///< BaseI2c bus reference
    uint8_t i2c_address_;                                ///< PCAL95555 I2C address
    std::unique_ptr<Pcal95555I2cAdapter> i2c_adapter_;   ///< I2C bus adapter
    std::shared_ptr<PCAL95555> pcal95555_driver_;        ///< PCAL95555 driver instance
    bool initialized_;                                   ///< Initialization status
    mutable std::mutex wrapper_mutex_;                   ///< Thread safety mutex

    //==============================================================//
    // PRIVATE HELPERS
    //==============================================================//

    /**
     * @brief Map functional GPIO pin to chip pin.
     * @param functional_pin Functional GPIO pin identifier
     * @return PCAL95555 chip pin number, or invalid value if not mapped
     */
    Pcal95555Chip1Pin MapFunctionalToChipPin(HardFOC::FunctionalGpioPin functional_pin) const noexcept;

    /**
     * @brief Get default configuration for a functional pin.
     * @param functional_pin Functional GPIO pin identifier
     * @param direction Output parameter for default direction
     * @param active_state Output parameter for default active state
     * @param pull_mode Output parameter for default pull mode
     * @return true if pin is mapped, false otherwise
     */
    bool GetDefaultPinConfig(HardFOC::FunctionalGpioPin functional_pin,
                            hf_gpio_direction_t& direction,
                            hf_gpio_active_state_t& active_state,
                            hf_gpio_pull_mode_t& pull_mode) const noexcept;

    /**
     * @brief Configure default pin settings based on platform mapping.
     * @return true if successful, false otherwise
     */
    bool ConfigureDefaultPinSettings() noexcept;
};

//==============================================================//
// GLOBAL FACTORY FUNCTIONS
//==============================================================//

/**
 * @brief Create a PCAL95555 GPIO wrapper instance.
 * @param i2c_bus BaseI2c bus instance
 * @param i2c_address PCAL95555 I2C address (default: 0x20)
 * @return Shared pointer to PCAL95555 GPIO wrapper
 */
std::shared_ptr<Pcal95555GpioWrapper> CreatePcal95555GpioWrapper(BaseI2c& i2c_bus, uint8_t i2c_address = 0x20) noexcept;

/**
 * @brief Get the default PCAL95555 I2C address.
 * @return Default I2C address (0x20)
 */
constexpr uint8_t GetDefaultPcal95555I2cAddress() noexcept { return 0x20; }

/**
 * @brief Get the number of pins available on PCAL95555.
 * @return Number of pins (16)
 */
constexpr uint8_t GetPcal95555PinCount() noexcept { return 16; }

#endif // COMPONENT_HANDLER_PCAL95555_GPIO_WRAPPER_H_ 