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

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseGpio.h"
#include "SfI2cBus.h"
#include "Tmc9660MotorController.h"
#include "Pcal95555Handler.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspGpio.h"

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
 * - Core operations return simple bool values for embedded system efficiency
 * - Configuration operations return hf_gpio_err_t for detailed error codes
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
     * @brief Ensure the GPIO manager system is initialized.
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool EnsureInitialized() noexcept;
    
    /**
     * @brief Shutdown the GPIO manager system.
     * @return true if shutdown successful, false otherwise
     */
    [[nodiscard]] bool Shutdown() noexcept;
    
    /**
     * @brief Check if the GPIO system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics and health information.
     * @param diagnostics Reference to store system diagnostics
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
    
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
     * @param resource Reference to store hardware resource pointer
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetPinHardwareResource(HardFOC::FunctionalGpioPin pin, 
                                            const HardFOC::GpioHardwareResource*& resource) const noexcept;
    
    /**
     * @brief Register all available pins from platform mapping.
     * @return true if registration successful, false otherwise
     */
    [[nodiscard]] bool RegisterAllPlatformPins() noexcept;
    
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
     * @return true if registration successful, false otherwise
     */
    [[nodiscard]] bool RegisterPin(HardFOC::FunctionalGpioPin pin, bool direction, 
                                  bool initial_state = false) noexcept;
    
    /**
     * @brief Unregister a GPIO pin from the system.
     * @param pin Functional pin identifier
     * @return true if unregistration successful, false otherwise
     */
    [[nodiscard]] bool UnregisterPin(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Check if a GPIO pin is registered.
     * @param pin Functional pin identifier
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool IsPinRegistered(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Get information about a registered GPIO pin.
     * @param pin Functional pin identifier
     * @param info Reference to store pin information pointer
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetPinInfo(HardFOC::FunctionalGpioPin pin, const GpioInfo*& info) const noexcept;
    
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
    // MODERN SHARED_PTR PIN ACCESS
    //==========================================================================
    
    /**
     * @brief Get a shared pointer to a GPIO pin if it exists.
     * @param pin Functional pin identifier
     * @return Shared pointer to BaseGpio or nullptr if pin not found/created
     * @note Does NOT create pin if not found. Use CreateSharedPin() to create new pins.
     * @note Thread-safe with RtosMutex protection at pin level.
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> GetPin(HfFunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Create a GPIO pin with specific configuration and return shared pointer.
     * @param pin Functional pin identifier
     * @param direction Pin direction (input/output)
     * @param active_state Active polarity (high/low)
     * @param pull_mode Pull resistor configuration
     * @return Shared pointer to BaseGpio or nullptr if creation failed
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> CreateSharedPin(
        HfFunctionalGpioPin pin,
        hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;
    
    /**
     * @brief Get multiple GPIO pins as shared pointers in one call.
     * @param pins Vector of functional pin identifiers
     * @return Vector of shared pointers (nullptr for failed pins)
     */
    [[nodiscard]] std::vector<std::shared_ptr<BaseGpio>> GetPins(
        const std::vector<HfFunctionalGpioPin>& pins) noexcept;
    
    //==========================================================================
    // BASIC PIN OPERATIONS
    //==========================================================================
    
    /**
     * @brief Set a GPIO pin to active state.
     * @param pin Functional pin identifier
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool SetActive(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Set a GPIO pin to inactive state.
     * @param pin Functional pin identifier
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool SetInactive(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Set a GPIO pin to a specific state.
     * @param pin Functional pin identifier
     * @param state Desired state (true = active, false = inactive)
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool SetState(HardFOC::FunctionalGpioPin pin, bool state) noexcept;
    
    /**
     * @brief Toggle a GPIO pin state.
     * @param pin Functional pin identifier
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool Toggle(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Read the current state of a GPIO pin.
     * @param pin Functional pin identifier
     * @param state Reference to store the read state
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool ReadState(HardFOC::FunctionalGpioPin pin, bool& state) noexcept;
    
    /**
     * @brief Check if a GPIO pin is in active state.
     * @param pin Functional pin identifier
     * @param active Reference to store the active state
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool IsActive(HardFOC::FunctionalGpioPin pin, bool& active) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch write operations on multiple pins.
     * @param operation Batch operation specification
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult BatchWrite(const GpioBatchOperation& operation) noexcept;
    
    /**
     * @brief Perform batch read operations on multiple pins.
     * @param pins Vector of functional pin identifiers to read
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult BatchRead(
        const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to active state.
     * @param pins Vector of functional pin identifiers
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult SetMultipleActive(
        const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pins Vector of functional pin identifiers
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult SetMultipleInactive(
        const std::vector<HardFOC::FunctionalGpioPin>& pins) noexcept;
    
    //==========================================================================
    // PIN CONFIGURATION
    //==========================================================================
    
    /**
     * @brief Configure pin direction.
     * @param pin Functional pin identifier
     * @param direction Pin direction (true = input, false = output)
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t SetPinDirection(HardFOC::FunctionalGpioPin pin, bool direction) noexcept;
    
    /**
     * @brief Configure pin pull-up/pull-down.
     * @param pin Functional pin identifier
     * @param pull_mode Pull mode configuration
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t SetPinPullMode(HardFOC::FunctionalGpioPin pin, 
                                             hf_gpio_pull_mode_t pull_mode) noexcept;
    
    /**
     * @brief Configure pin output mode.
     * @param pin Functional pin identifier
     * @param output_mode Output mode configuration
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t SetPinOutputMode(HardFOC::FunctionalGpioPin pin,
                                               hf_gpio_output_mode_t output_mode) noexcept;
    
    /**
     * @brief Get pin direction.
     * @param pin Functional pin identifier
     * @param direction Reference to store direction (true = input, false = output)
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetPinDirection(HardFOC::FunctionalGpioPin pin, bool& direction) const noexcept;
    
    /**
     * @brief Get pin pull mode.
     * @param pin Functional pin identifier
     * @param pull_mode Reference to store pull mode
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetPinPullMode(HardFOC::FunctionalGpioPin pin, hf_gpio_pull_mode_t& pull_mode) const noexcept;
    
    /**
     * @brief Get pin output mode.
     * @param pin Functional pin identifier
     * @param output_mode Reference to store output mode
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetPinOutputMode(HardFOC::FunctionalGpioPin pin, hf_gpio_output_mode_t& output_mode) const noexcept;
    
    //==========================================================================
    // INTERRUPT SUPPORT
    //==========================================================================
    
    /**
     * @brief Configure interrupt for a pin.
     * @param pin Functional pin identifier
     * @param trigger Interrupt trigger type
     * @param callback Interrupt callback function
     * @param user_data User data for callback
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t ConfigureInterrupt(HardFOC::FunctionalGpioPin pin,
                                                   BaseGpio::InterruptTrigger trigger,
                                                   BaseGpio::InterruptCallback callback,
                                                   void* user_data = nullptr) noexcept;
    
    /**
     * @brief Enable interrupt for a pin.
     * @param pin Functional pin identifier
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t EnableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept;
    
    /**
     * @brief Disable interrupt for a pin.
     * @param pin Functional pin identifier
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t DisableInterrupt(HardFOC::FunctionalGpioPin pin) noexcept;
    
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
     * @param health_info Reference to store health information string
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetSystemHealth(std::string& health_info) const noexcept;
    
    /**
     * @brief Reset all output pins to inactive state.
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool ResetAllPins() noexcept;
    
    /**
     * @brief Get pin statistics.
     * @param pin Functional pin identifier
     * @param statistics Reference to store pin statistics
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool GetPinStatistics(HardFOC::FunctionalGpioPin pin, BaseGpio::PinStatistics& statistics) const noexcept;
    
    /**
     * @brief Clear pin statistics.
     * @param pin Functional pin identifier
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool ClearPinStatistics(HardFOC::FunctionalGpioPin pin) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    /**
     * @brief Initialize the GPIO manager system with platform pin registration.
     * 
     * This private method performs the actual initialization including:
     * - CommChannelsManager dependency validation
     * - All platform pins registration and validation
     * - Lazy initialization of hardware handlers
     * - Thread-safe initialization with RtosMutex
     * 
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool Initialize() noexcept;
    
    // ===============================
    // SYSTEM STATE
    // ===============================
    
    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> is_initialized_{false};
    
    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex mutex_;
    
    // ===============================
    // HARDWARE INTERFACES (REMOVED - USE COMMCHANNELSMANAGER)
    // ===============================
    
    // Pin registry with RtosMutex protection
    std::unordered_map<HardFOC::FunctionalGpioPin, std::unique_ptr<GpioInfo>> pin_registry_;
    
    /**
     * @brief Modern shared pointer pin registry for safe shared access.
     * Allows multiple components to safely share the same GPIO pin.
     * Protected by RtosMutex for thread-safe access.
     */
    std::unordered_map<HfFunctionalGpioPin, std::shared_ptr<BaseGpio>> shared_pin_registry_;
    mutable RtosMutex shared_pin_mutex_;  ///< RtosMutex for shared pin registry access
    
    // ===============================
    // GPIO HANDLERS (LAZY INITIALIZATION)
    // ===============================
    
    /**
     * @brief PCAL95555 GPIO expander handler (lazy initialized when first accessed).
     * Single instance manages all PCAL95555 pins to prevent duplicate controllers.
     * Uses CommChannelsManager for I2C access.
     */
    std::unique_ptr<Pcal95555Handler> pcal95555_handler_;
    mutable RtosMutex pcal_handler_mutex_;  ///< Mutex for PCAL95555 handler initialization
    
    /**
     * @brief Hardware interrupt pin for PCAL95555 (ESP32 GPIO connected to PCAL95555 INT line).
     * This pin receives hardware interrupts from the PCAL95555 when any enabled pin changes state.
     * Lazy initialized with PCAL95555 handler.
     */
    std::unique_ptr<BaseGpio> pcal_interrupt_pin_;
    
    /**
     * @brief TMC9660 GPIO handler (lazy initialized when first accessed).  
     * Single instance manages all TMC9660 pins to prevent duplicate controllers.
     * Uses CommChannelsManager for SPI/UART access.
     */
    std::unique_ptr<Tmc9660Handler> tmc9660_handler_;
    mutable RtosMutex tmc_handler_mutex_;   ///< Mutex for TMC9660 handler initialization
    
    // ===============================
    // SYSTEM STATISTICS
    // ===============================
    
    /**
     * @brief Total operations performed (atomic for thread safety).
     */
    std::atomic<uint32_t> total_operations_{0};
    
    /**
     * @brief Successful operations count (atomic for thread safety).
     */
    std::atomic<uint32_t> successful_operations_{0};
    
    /**
     * @brief Failed operations count (atomic for thread safety).
     */
    std::atomic<uint32_t> failed_operations_{0};
    
    /**
     * @brief Communication errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> communication_errors_{0};
    
    /**
     * @brief Hardware errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> hardware_errors_{0};
    
    /**
     * @brief System start time for uptime calculations (atomic for thread safety).
     */
    std::atomic<uint64_t> system_start_time_{0};
    
    // ===============================
    // ERROR TRACKING
    // ===============================
    
    /**
     * @brief Last error code for diagnostics (atomic for thread safety).
     */
    std::atomic<hf_gpio_err_t> last_error_{hf_gpio_err_t::GPIO_SUCCESS};
    
    /**
     * @brief Thread-safe access to error message storage.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex error_mutex_;
    
    /**
     * @brief Recent error messages for diagnostics.
     * Limited to MAX_ERROR_MESSAGES for memory management.
     */
    std::vector<std::string> recent_errors_;
    
    /**
     * @brief Maximum number of error messages to retain.
     * Prevents unbounded memory growth in long-running embedded systems.
     */
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
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool InitializeEsp32Gpio() noexcept;
    
    /**
     * @brief Initialize PCAL95555 GPIO wrapper.
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool InitializePcal95555GpioWrapper() noexcept;
    
    /**
     * @brief Initialize TMC9660 GPIO system.
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool InitializeTmc9660Gpio() noexcept;
    
    /**
     * @brief Create GPIO pin from functional pin.
     * @param pin Functional pin identifier
     * @param direction Pin direction (true = input, false = output)
     * @param driver Reference to store created driver
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool CreateGpioPin(
        HardFOC::FunctionalGpioPin pin, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept;
    
    /**
     * @brief Create ESP32 GPIO pin.
     * @param pin_id Hardware pin ID
     * @param direction Pin direction (true = input, false = output)
     * @param driver Reference to store created driver
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool CreateEsp32GpioPin(
        uint8_t pin_id, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept;
    
    /**
     * @brief Create PCAL95555 GPIO pin.
     * @param pin_id Hardware pin ID
     * @param direction Pin direction (true = input, false = output)
     * @param driver Reference to store created driver
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool CreatePcal95555GpioPin(
        uint8_t pin_id, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept;
    
    /**
     * @brief Create TMC9660 GPIO pin.
     * @param pin_id Hardware pin ID
     * @param direction Pin direction (true = input, false = output)
     * @param driver Reference to store created driver
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool CreateTmc9660GpioPin(
        uint8_t pin_id, bool direction, std::unique_ptr<BaseGpio>& driver) noexcept;
    
    /**
     * @brief Ensure PCAL95555 handler is initialized (lazy initialization).
     * @return true if successful, false otherwise
     * @note Thread-safe with RtosMutex protection
     */
    [[nodiscard]] bool EnsurePcal95555Handler() noexcept;
    
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
     * @brief Update last error for diagnostics.
     * @param error_code Error code to record
     */
    void UpdateLastError(hf_gpio_err_t error_code) noexcept;
    
    /**
     * @brief Get pin name from functional pin identifier.
     * @param pin Functional pin identifier
     * @return Pin name string
     */
    [[nodiscard]] std::string GetPinName(HardFOC::FunctionalGpioPin pin) const noexcept;
    
    /**
     * @brief Validate hardware resource for pin creation.
     * @param resource Hardware resource descriptor
     * @return true if valid, false otherwise
     */
    [[nodiscard]] bool ValidateHardwareResource(
        const HardFOC::GpioHardwareResource* resource) const noexcept;
    
    /**
     * @brief Check for hardware resource conflicts.
     * @param resource Hardware resource descriptor
     * @return true if no conflicts, false otherwise
     */
    [[nodiscard]] bool CheckHardwareConflicts(
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
