/**
 * @file GpioManager.h
 * @brief Advanced GPIO management system for the HardFOC platform.
 * 
 * @details This class provides a comprehensive GPIO management system that integrates
 *          with the platform mapping system to automatically manage GPIOs from multiple
 *          hardware sources (ESP32-C6, PCAL95555, TMC9660) based on functional pin
 *          identifiers and hardware chip mappings.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features:
 * - Platform mapping integration for automatic pin discovery
 * - Multi-chip GPIO management (ESP32, PCAL95555, TMC9660)
 * - Functional pin abstraction with hardware-agnostic API
 * - Thread-safe operation with comprehensive error handling
 * - Automatic pin registration based on platform configuration
 * - Advanced diagnostics and health monitoring
 * - Batch operations for performance optimization
 * - Hardware resource validation and conflict detection
 * 
 * Architecture:
 * - Uses HardFOC::FunctionalGpioPin for pin identification
 * - Integrates with HardFOC::GpioPlatformMapping for hardware mapping
 * - Supports all HardwareChip types defined in platform mapping
 * - Provides unified BaseGpio interface for all pin operations
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 * @note All pin operations use functional pin identifiers, not hardware-specific pin numbers.
 */

#ifndef COMPONENT_HANDLER_GPIO_MANAGER_H_
#define COMPONENT_HANDLER_GPIO_MANAGER_H_

#include "Result.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseGpio.h"
#include "SfI2cBus.h"
#include "Tmc9660MotorController.h"
#include "Pcal95555Handler.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/McuDigitalGpio.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <optional>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Pcal95555Handler;

//==============================================================================
// GPIO INFORMATION STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive GPIO pin information with platform mapping.
 */
struct GpioInfo {
    HardFOC::FunctionalGpioPin functional_pin;  ///< Functional pin identifier
    std::unique_ptr<BaseGpio> gpio_driver;      ///< GPIO driver instance
    std::string_view name;                      ///< Human-readable name
    HardFOC::HardwareChip hardware_chip;        ///< Hardware chip identifier
    uint8_t hardware_pin_id;                    ///< Hardware pin ID within the chip
    bool is_registered;                         ///< Registration status
    bool is_input;                              ///< Pin direction (true = input, false = output)
    bool current_state;                         ///< Last known pin state
    uint32_t access_count;                      ///< Number of times accessed
    uint32_t error_count;                       ///< Number of errors encountered
    uint64_t last_access_time;                  ///< Timestamp of last access
    
    /**
     * @brief Constructor for GpioInfo.
     */
    GpioInfo(HardFOC::FunctionalGpioPin fp, std::unique_ptr<BaseGpio> driver, 
             std::string_view n, HardFOC::HardwareChip chip, uint8_t pin_id) noexcept
        : functional_pin(fp), gpio_driver(std::move(driver)), name(n), 
          hardware_chip(chip), hardware_pin_id(pin_id), is_registered(true),
          is_input(false), current_state(false), access_count(0), 
          error_count(0), last_access_time(0) {}
    
    // Disable copy operations due to unique_ptr
    GpioInfo(const GpioInfo&) = delete;
    GpioInfo& operator=(const GpioInfo&) = delete;
    
    // Enable move operations
    GpioInfo(GpioInfo&&) = default;
    GpioInfo& operator=(GpioInfo&&) = default;
};

/**
 * @brief Structure for GPIO batch operation specifications.
 */
struct GpioBatchOperation {
    std::vector<HardFOC::FunctionalGpioPin> pins;  ///< Functional pins to operate on
    std::vector<bool> states;                       ///< Desired states (for write operations)
    bool is_write_operation;                        ///< true for write, false for read
    
    /**
     * @brief Constructor for write operations.
     */
    GpioBatchOperation(std::vector<HardFOC::FunctionalGpioPin> p, std::vector<bool> s) noexcept
        : pins(std::move(p)), states(std::move(s)), is_write_operation(true) {}
    
    /**
     * @brief Constructor for read operations.
     */
    explicit GpioBatchOperation(std::vector<HardFOC::FunctionalGpioPin> p) noexcept
        : pins(std::move(p)), is_write_operation(false) {}
};

/**
 * @brief Structure for GPIO batch operation results.
 */
struct GpioBatchResult {
    std::vector<HardFOC::FunctionalGpioPin> pins;  ///< Pins operated on
    std::vector<bool> states;                       ///< Resulting states
    std::vector<ResultCode> results;                ///< Individual operation results
    ResultCode overall_result;                      ///< Overall operation result
    
    /**
     * @brief Check if all operations were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return IsSuccessResult(overall_result);
    }
};

/**
 * @brief Structure for GPIO system diagnostics.
 */
struct GpioSystemDiagnostics {
    bool system_healthy;                           ///< Overall system health
    uint32_t total_pins_registered;                ///< Total pins registered
    uint32_t pins_by_chip[static_cast<uint8_t>(HardFOC::HardwareChip::HARDWARE_CHIP_COUNT)]; ///< Pins per chip
    uint32_t total_operations;                     ///< Total operations performed
    uint32_t successful_operations;                ///< Successful operations
    uint32_t failed_operations;                    ///< Failed operations
    uint32_t communication_errors;                 ///< Communication errors
    uint32_t hardware_errors;                      ///< Hardware errors
    uint64_t system_uptime_ms;                     ///< System uptime
    std::vector<std::string> error_messages;       ///< Recent error messages
};

//==============================================================================
// MAIN GPIO MANAGER CLASS
//==============================================================================

/**
 * @class GpioManager
 * @brief Advanced GPIO management system for the HardFOC platform.
 * 
 * This class provides a comprehensive GPIO management system that integrates
 * with the platform mapping system to automatically manage GPIOs from multiple
 * hardware sources. It uses functional pin identifiers and hardware chip
 * mappings to provide a unified, hardware-agnostic API.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - All operations return Result<T> types
 * - Comprehensive error codes via ResultCode enum
 * - Detailed error descriptions and diagnostics
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple pins
 * - Cached state information
 * - Lazy initialization of hardware resources
 * 
 * Platform Integration:
 * - Automatic pin discovery via platform mapping
 * - Hardware resource validation
 * - Conflict detection and resolution
 * - Multi-chip coordination
 */
class GpioManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     * @return Reference to the GPIO manager instance
     */
    static GpioManager& GetInstance() noexcept;
    
    /**
     * @brief Initialize the GPIO manager system with platform mapping integration.
     * 
     * This method initializes the entire GPIO system, including:
     * - Platform mapping validation and pin discovery
     * - ESP32-C6 native GPIO configuration
     * - PCAL95555 I2C expander setup and pin registration
     * - TMC9660 GPIO integration
     * - Automatic pin registration based on platform configuration
     * - Hardware resource validation and conflict detection
     * 
     * @param i2cBus Reference to the I2C bus for PCAL95555 communication
     * @param tmc9660Controller Reference to the TMC9660 controller
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> Initialize(SfI2cBus& i2cBus, 
                                         Tmc9660MotorController& tmc9660Controller) noexcept;
    
    /**
     * @brief Shutdown the GPIO manager system.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> Shutdown() noexcept;
    
    /**
     * @brief Check if the GPIO system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics and health information.
     * @return Result containing system diagnostics
     */
    [[nodiscard]] Result<GpioSystemDiagnostics> GetSystemDiagnostics() const noexcept;
    
    //==========================================================================
    // PLATFORM MAPPING INTEGRATION
    //==========================================================================
    
    /**
     * @brief Check if a functional pin is available on this platform.
     * @param pin Functional pin identifier
     * @return true if pin is available, false otherwise
     */
    [[nodiscard]] bool IsPinAvailable(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Get hardware resource information for a functional pin.
     * @param pin Functional pin identifier
     * @return Result containing hardware resource information
     */
    [[nodiscard]] Result<const HardFOC::GpioHardwareResource*> GetPinHardwareResource(
        HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Register all available pins from platform mapping.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> RegisterAllPlatformPins() noexcept;
    
    /**
     * @brief Get list of all available functional pins.
     * @return Vector of available functional pin identifiers
     */
    [[nodiscard]] std::vector<HardFOC::FunctionalGpioPin> GetAvailablePins() const noexcept;
    
    //==========================================================================
    // PIN REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register a GPIO pin with the system using functional pin identifier.
     * @param pin Functional pin identifier
     * @param direction Pin direction (true = input, false = output)
     * @param initial_state Initial state for output pins
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> RegisterPin(HardFOC::FunctionalGpioPin pin, bool direction, 
                                          bool initial_state = false) noexcept;
    
    /**
     * @brief Unregister a GPIO pin from the system.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> UnregisterPin(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Check if a GPIO pin is registered.
     * @param pin Functional pin identifier
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool IsPinRegistered(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Get information about a registered GPIO pin.
     * @param pin Functional pin identifier
     * @return Result containing pin information or error
     */
    [[nodiscard]] Result<const GpioInfo*> GetPinInfo(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Get count of registered pins.
     * @return Number of registered pins
     */
    [[nodiscard]] size_t GetRegisteredPinCount() const noexcept;
    
    /**
     * @brief Get list of all registered pins.
     * @return Vector of registered functional pin identifiers
     */
    [[nodiscard]] std::vector<HardFOC::FunctionalGpioPin> GetRegisteredPins() const noexcept;
    
    //==========================================================================
    // BASIC PIN OPERATIONS
    //==========================================================================
    
    /**
     * @brief Set a GPIO pin to active state.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetActive(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Set a GPIO pin to inactive state.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetInactive(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Set a GPIO pin to a specific state.
     * @param pin Functional pin identifier
     * @param state Desired state (true = active, false = inactive)
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetState(HardFOC::FunctionalGpioPin pin, bool state) noexcept;
    
    /**
     * @brief Toggle a GPIO pin state.
     * @param pin Functional pin identifier
     * @return Result containing new state or error
     */
    [[nodiscard]] Result<bool> Toggle(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Read the current state of a GPIO pin.
     * @param pin Functional pin identifier
     * @return Result containing pin state or error
     */
    [[nodiscard]] Result<bool> ReadState(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Check if a GPIO pin is currently active.
     * @param pin Functional pin identifier
     * @return Result containing active status or error
     */
    [[nodiscard]] Result<bool> IsActive(HardFOC::FunctionalGpioPin pin) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch write operations on multiple pins.
     * @param operation Batch operation specification
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> BatchWrite(const GpioBatchOperation& operation) noexcept;
    
    /**
     * @brief Perform batch read operations on multiple pins.
     * @param pins Vector of functional pin identifiers to read
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> BatchRead(
        const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to active state.
     * @param pins Vector of functional pin identifiers
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> SetMultipleActive(
        const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pins Vector of functional pin identifiers
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> SetMultipleInactive(
        const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept;
    
    //==========================================================================
    // PIN CONFIGURATION
    //==========================================================================
    
    /**
     * @brief Configure pin direction.
     * @param pin Functional pin identifier
     * @param direction Pin direction (true = input, false = output)
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetPinDirection(HardFOC::FunctionalGpioPin pin, bool direction) noexcept;
    
    /**
     * @brief Configure pin pull-up/pull-down.
     * @param pin Functional pin identifier
     * @param pull_mode Pull mode configuration
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetPinPullMode(HardFOC::FunctionalGpioPin pin, 
                                             hf_gpio_pull_mode_t pull_mode) noexcept;
    
    /**
     * @brief Configure pin output mode.
     * @param pin Functional pin identifier
     * @param output_mode Output mode configuration
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetPinOutputMode(HardFOC::FunctionalGpioPin pin,
                                               hf_gpio_output_mode_t output_mode) noexcept;
    
    /**
     * @brief Get pin direction.
     * @param pin Functional pin identifier
     * @return Result containing pin direction or error
     */
    [[nodiscard]] Result<bool> GetPinDirection(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Get pin pull mode.
     * @param pin Functional pin identifier
     * @return Result containing pull mode or error
     */
    [[nodiscard]] Result<hf_gpio_pull_mode_t> GetPinPullMode(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Get pin output mode.
     * @param pin Functional pin identifier
     * @return Result containing output mode or error
     */
    [[nodiscard]] Result<hf_gpio_output_mode_t> GetPinOutputMode(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    //==========================================================================
    // INTERRUPT SUPPORT
    //==========================================================================
    
    /**
     * @brief Configure interrupt for a pin.
     * @param pin Functional pin identifier
     * @param trigger Interrupt trigger type
     * @param callback Interrupt callback function
     * @param user_data User data for callback
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> ConfigureInterrupt(HardFOC::FunctionalGpioPin pin,
                                                 BaseGpio::InterruptTrigger trigger,
                                                 BaseGpio::InterruptCallback callback,
                                                 void* user_data = nullptr) noexcept;
    
    /**
     * @brief Enable interrupt for a pin.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> EnableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Disable interrupt for a pin.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> DisableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Check if pin supports interrupts.
     * @param pin Functional pin identifier
     * @return true if interrupts are supported, false otherwise
     */
    [[nodiscard]] bool SupportsInterrupts(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    //==========================================================================
    // SYSTEM INFORMATION
    //==========================================================================
    
    /**
     * @brief Get system health information.
     * @return Result containing health status string
     */
    [[nodiscard]] Result<std::string> GetSystemHealth() const noexcept;
    
    /**
     * @brief Reset all output pins to inactive state.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> ResetAllPins() noexcept;
    
    /**
     * @brief Get pin statistics.
     * @param pin Functional pin identifier
     * @return Result containing pin statistics or error
     */
    [[nodiscard]] Result<BaseGpio::PinStatistics> GetPinStatistics(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Clear pin statistics.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> ClearPinStatistics(HardFOC::FunctionalGpioPin pin) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    // System state
    std::atomic<bool> is_initialized_{false};
    mutable std::mutex mutex_;
    
    // Hardware interfaces
    SfI2cBus* i2c_bus_{nullptr};
    Tmc9660MotorController* tmc9660_controller_{nullptr};
    std::shared_ptr<Pcal95555Handler> pcal95555_handler_;
    
    // Pin registry
    std::unordered_map<HardFOC::FunctionalGpioPin, std::unique_ptr<GpioInfo>> pin_registry_;
    
    // Statistics
    std::atomic<uint32_t> total_operations_{0};
    std::atomic<uint32_t> successful_operations_{0};
    std::atomic<uint32_t> failed_operations_{0};
    std::atomic<uint32_t> communication_errors_{0};
    std::atomic<uint32_t> hardware_errors_{0};
    std::atomic<uint64_t> system_start_time_{0};
    
    // Error tracking
    mutable std::mutex error_mutex_;
    std::vector<std::string> recent_errors_;
    static constexpr size_t MAX_ERROR_MESSAGES = 10;
    
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================
    
    /**
     * @brief Private constructor for singleton pattern.
     */
    GpioManager() = default;
    
    /**
     * @brief Private destructor.
     */
    ~GpioManager() = default;
    
    // Disable copy and move operations
    GpioManager(const GpioManager&) = delete;
    GpioManager& operator=(const GpioManager&) = delete;
    GpioManager(GpioManager&&) = delete;
    GpioManager& operator=(GpioManager&&) = delete;
    
    /**
     * @brief Initialize ESP32-C6 native GPIO system.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> InitializeEsp32Gpio() noexcept;
    
    /**
     * @brief Initialize PCAL95555 GPIO wrapper.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> InitializePcal95555GpioWrapper() noexcept;
    
    /**
     * @brief Initialize TMC9660 GPIO system.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> InitializeTmc9660Gpio() noexcept;
    
    /**
     * @brief Create GPIO driver for a functional pin.
     * @param pin Functional pin identifier
     * @param direction Pin direction
     * @return Result containing GPIO driver or error
     */
    [[nodiscard]] Result<std::unique_ptr<BaseGpio>> CreateGpioDriver(
        HardFOC::FunctionalGpioPin pin, bool direction) noexcept;
    
    /**
     * @brief Create ESP32 GPIO driver.
     * @param pin_id Hardware pin ID
     * @param direction Pin direction
     * @return Result containing GPIO driver or error
     */
    [[nodiscard]] Result<std::unique_ptr<BaseGpio>> CreateEsp32GpioDriver(
        uint8_t pin_id, bool direction) noexcept;
    
    /**
     * @brief Create PCAL95555 GPIO driver.
     * @param pin_id Hardware pin ID
     * @param direction Pin direction
     * @return Result containing GPIO driver or error
     */
    [[nodiscard]] Result<std::unique_ptr<BaseGpio>> CreatePcal95555GpioDriver(
        uint8_t pin_id, bool direction) noexcept;
    
    /**
     * @brief Create TMC9660 GPIO driver.
     * @param pin_id Hardware pin ID
     * @param direction Pin direction
     * @return Result containing GPIO driver or error
     */
    [[nodiscard]] Result<std::unique_ptr<BaseGpio>> CreateTmc9660GpioDriver(
        uint8_t pin_id, bool direction) noexcept;
    
    /**
     * @brief Find GPIO info by functional pin.
     * @param pin Functional pin identifier
     * @return Pointer to GPIO info or nullptr if not found
     */
    [[nodiscard]] GpioInfo* FindGpioInfo(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Find GPIO info by functional pin (const version).
     * @param pin Functional pin identifier
     * @return Pointer to GPIO info or nullptr if not found
     */
    [[nodiscard]] const GpioInfo* FindGpioInfo(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Update operation statistics.
     * @param success Whether the operation was successful
     */
    void UpdateStatistics(bool success) noexcept;
    
    /**
     * @brief Add error message to recent errors list.
     * @param error_message Error message to add
     */
    void AddErrorMessage(const std::string& error_message) noexcept;
    
    /**
     * @brief Get pin name from functional pin identifier.
     * @param pin Functional pin identifier
     * @return Pin name string
     */
    [[nodiscard]] std::string GetPinName(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Validate hardware resource for pin creation.
     * @param resource Hardware resource descriptor
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> ValidateHardwareResource(
        const HardFOC::GpioHardwareResource* resource) const noexcept;
    
    /**
     * @brief Check for hardware resource conflicts.
     * @param resource Hardware resource descriptor
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> CheckHardwareConflicts(
        const HardFOC::GpioHardwareResource* resource) const noexcept;
};

//==============================================================================
// GLOBAL HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Get the GPIO manager instance.
 * @return Reference to the GPIO manager instance
 */
[[nodiscard]] inline GpioManager& GetGpioManager() noexcept {
    return GpioManager::GetInstance();
}

#endif // COMPONENT_HANDLER_GPIO_MANAGER_H_
