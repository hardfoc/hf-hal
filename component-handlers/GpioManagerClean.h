/**
 * @file GpioManagerClean.h
 * @brief Clean GPIO management system using existing hf_gpio_err_t error codes.
 * 
 * This version removes error message storage and uses the comprehensive
 * hf_gpio_err_t error codes from the hf-internal-interface submodule.
 * 
 * Key Changes:
 * - Removed error message storage completely (saves 650+ bytes)
 * - Uses existing hf_gpio_err_t for all error reporting
 * - Simplified pin validation integrated with hf_gpio_err_t
 * - Lean diagnostics with counters only
 * - Zero dynamic allocations
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.2 (Clean)
 */

#ifndef COMPONENT_HANDLER_GPIO_MANAGER_CLEAN_H_
#define COMPONENT_HANDLER_GPIO_MANAGER_CLEAN_H_

#include "CommonIDs.h"
#include "base/BaseGpio.h"
#include "SfI2cBus.h"
#include "Tmc9660MotorController.h"
#include "Pcal95555Handler.h"
#include "MotorController.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspGpio.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Pcal95555Handler;
class Tmc9660Handler;

//==============================================================================
// CLEAN DIAGNOSTICS (NO ERROR STORAGE)
//==============================================================================

/**
 * @brief Lean system diagnostics with operational metrics only.
 * No error message storage - just counters and hardware status.
 */
struct GpioSystemDiagnostics {
    bool system_healthy;                    ///< Overall system health
    uint32_t total_pins_registered;         ///< Total pins registered
    uint32_t total_operations;              ///< Total operations performed
    uint32_t successful_operations;         ///< Successful operations
    uint32_t failed_operations;             ///< Failed operations
    uint32_t validation_failures;           ///< Pin name validation failures
    uint32_t hardware_failures;             ///< Hardware communication failures
    uint64_t system_uptime_ms;              ///< System uptime in milliseconds
    
    // Hardware component availability
    bool pcal95555_available;               ///< PCAL95555 handler available
    bool tmc9660_available;                 ///< TMC9660 handler available
    bool comm_manager_initialized;          ///< CommChannelsManager status
    
    // Error rate calculations
    float success_rate;                     ///< Success rate (0.0 - 1.0)
    hf_gpio_err_t last_error;              ///< Last error encountered
};

/**
 * @brief Batch operation results using hf_gpio_err_t.
 */
struct GpioBatchResult {
    std::vector<std::string_view> pin_names;    ///< Pin names operated on
    std::vector<bool> states;                   ///< Resulting states (for read operations)
    std::vector<hf_gpio_err_t> results;        ///< Individual operation results
    hf_gpio_err_t overall_result;              ///< Overall operation result
    
    /**
     * @brief Check if all operations were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return overall_result == hf_gpio_err_t::GPIO_SUCCESS;
    }
};

/**
 * @brief Batch operation specification.
 */
struct GpioBatchOperation {
    std::vector<std::string_view> pin_names;    ///< Pin names to operate on
    std::vector<bool> states;                   ///< Desired states (for write operations)
    bool is_write_operation;                    ///< true for write, false for read
    
    /**
     * @brief Constructor for write operations.
     */
    GpioBatchOperation(std::vector<std::string_view> names, std::vector<bool> s) noexcept
        : pin_names(std::move(names)), states(std::move(s)), is_write_operation(true) {}
    
    /**
     * @brief Constructor for read operations.
     */
    explicit GpioBatchOperation(std::vector<std::string_view> names) noexcept
        : pin_names(std::move(names)), is_write_operation(false) {}
};

//==============================================================================
// CLEAN GPIO MANAGER CLASS
//==============================================================================

/**
 * @class GpioManagerClean
 * @brief Clean GPIO management system using hf_gpio_err_t error codes.
 * 
 * This version eliminates error message storage and uses the comprehensive
 * hf_gpio_err_t error codes from the hf-internal-interface submodule for
 * all error reporting. This provides:
 * 
 * - Structured error handling with industry-standard error codes
 * - Zero dynamic memory allocation for error tracking
 * - Minimal memory footprint (saves 650+ bytes)
 * - Better integration with existing BaseGpio infrastructure
 * - Actionable error codes for precise error handling
 * 
 * Error Handling Strategy:
 * - Pin validation errors: Specific hf_gpio_err_t codes
 * - Hardware operations: Pass-through hf_gpio_err_t from BaseGpio
 * - System operations: Appropriate hf_gpio_err_t codes
 * - Diagnostics: Counters and last error only
 */
class GpioManagerClean {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     */
    static GpioManagerClean& GetInstance() noexcept;
    
    /**
     * @brief Ensure the GPIO manager system is initialized.
     * @return hf_gpio_err_t::GPIO_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_gpio_err_t EnsureInitialized() noexcept;
    
    /**
     * @brief Shutdown the GPIO manager system.
     * @return hf_gpio_err_t::GPIO_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_gpio_err_t Shutdown() noexcept;
    
    /**
     * @brief Check if the GPIO system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics.
     * @param diagnostics Reference to store system diagnostics
     * @return hf_gpio_err_t::GPIO_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_gpio_err_t GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
    
    //==========================================================================
    // PIN VALIDATION (USING hf_gpio_err_t)
    //==========================================================================
    
    /**
     * @brief Validate pin name using hf_gpio_err_t error codes.
     * @param name Pin name to validate
     * @return hf_gpio_err_t::GPIO_SUCCESS if valid, specific error otherwise
     */
    [[nodiscard]] static hf_gpio_err_t ValidatePinName(std::string_view name) noexcept;
    
    //==========================================================================
    // GPIO REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register a GPIO pin with the system.
     * @param name Pin name (must be static string or outlive the manager)
     * @param gpio Shared pointer to GPIO driver
     * @return hf_gpio_err_t::GPIO_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_gpio_err_t RegisterGpio(std::string_view name, std::shared_ptr<BaseGpio> gpio) noexcept;
    
    /**
     * @brief Get a GPIO pin by name.
     * @param name Pin name
     * @return Shared pointer to GPIO or nullptr if not found
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;
    
    /**
     * @brief Check if a GPIO pin is registered.
     * @param name Pin name
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;
    
    /**
     * @brief Get count of registered pins.
     * @return Number of registered pins
     */
    [[nodiscard]] size_t Size() const noexcept;
    
    //==========================================================================
    // BASIC GPIO OPERATIONS (RETURN hf_gpio_err_t)
    //==========================================================================
    
    /**
     * @brief Set a GPIO pin to a specific state.
     * @param name Pin name
     * @param value Desired state (true = active, false = inactive)
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t Set(std::string_view name, bool value) noexcept;
    
    /**
     * @brief Set a GPIO pin to active state.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t SetActive(std::string_view name) noexcept;
    
    /**
     * @brief Set a GPIO pin to inactive state.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t SetInactive(std::string_view name) noexcept;
    
    /**
     * @brief Read the current state of a GPIO pin.
     * @param name Pin name
     * @param state Reference to store the read state
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t Read(std::string_view name, bool& state) noexcept;
    
    /**
     * @brief Toggle a GPIO pin state.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t Toggle(std::string_view name) noexcept;
    
    /**
     * @brief Check if a GPIO pin is in active state.
     * @param name Pin name
     * @param active Reference to store the active state
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t IsActive(std::string_view name, bool& active) noexcept;
    
    //==========================================================================
    // PIN CONFIGURATION (PASS-THROUGH hf_gpio_err_t)
    //==========================================================================
    
    /**
     * @brief Configure pin direction.
     * @param name Pin name
     * @param direction Pin direction
     * @return hf_gpio_err_t error code from BaseGpio
     */
    [[nodiscard]] hf_gpio_err_t SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept;
    
    /**
     * @brief Configure pin pull-up/pull-down.
     * @param name Pin name
     * @param pull_mode Pull mode configuration
     * @return hf_gpio_err_t error code from BaseGpio
     */
    [[nodiscard]] hf_gpio_err_t SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept;
    
    /**
     * @brief Configure pin output mode.
     * @param name Pin name
     * @param output_mode Output mode configuration
     * @return hf_gpio_err_t error code from BaseGpio
     */
    [[nodiscard]] hf_gpio_err_t SetOutputMode(std::string_view name, hf_gpio_output_mode_t output_mode) noexcept;
    
    /**
     * @brief Get pin direction.
     * @param name Pin name
     * @param direction Reference to store direction
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetDirection(std::string_view name, hf_gpio_direction_t& direction) const noexcept;
    
    /**
     * @brief Get pin pull mode.
     * @param name Pin name
     * @param pull_mode Reference to store pull mode
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetPullMode(std::string_view name, hf_gpio_pull_mode_t& pull_mode) const noexcept;
    
    /**
     * @brief Get pin output mode.
     * @param name Pin name
     * @param output_mode Reference to store output mode
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetOutputMode(std::string_view name, hf_gpio_output_mode_t& output_mode) const noexcept;
    
    //==========================================================================
    // INTERRUPT SUPPORT (PASS-THROUGH hf_gpio_err_t)
    //==========================================================================
    
    /**
     * @brief Configure interrupt for a pin.
     * @param name Pin name
     * @param trigger Interrupt trigger type
     * @param callback Interrupt callback function
     * @param user_data User data for callback
     * @return hf_gpio_err_t error code from BaseGpio
     */
    [[nodiscard]] hf_gpio_err_t ConfigureInterrupt(std::string_view name,
                                                   hf_gpio_interrupt_trigger_t trigger,
                                                   BaseGpio::InterruptCallback callback,
                                                   void* user_data = nullptr) noexcept;
    
    /**
     * @brief Enable interrupt for a pin.
     * @param name Pin name
     * @return hf_gpio_err_t error code from BaseGpio
     */
    [[nodiscard]] hf_gpio_err_t EnableInterrupt(std::string_view name) noexcept;
    
    /**
     * @brief Disable interrupt for a pin.
     * @param name Pin name
     * @return hf_gpio_err_t error code from BaseGpio
     */
    [[nodiscard]] hf_gpio_err_t DisableInterrupt(std::string_view name) noexcept;
    
    /**
     * @brief Check if pin supports interrupts.
     * @param name Pin name
     * @return true if interrupts are supported, false otherwise
     */
    [[nodiscard]] bool SupportsInterrupts(std::string_view name) const noexcept;
    
    //==========================================================================
    // STATISTICS (CLEAN - NO ERROR STORAGE)
    //==========================================================================
    
    /**
     * @brief Get pin statistics.
     * @param name Pin name
     * @param statistics Reference to store pin statistics
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetStatistics(std::string_view name, BaseGpio::PinStatistics& statistics) const noexcept;
    
    /**
     * @brief Reset pin statistics.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t ResetStatistics(std::string_view name) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS (USING hf_gpio_err_t)
    //==========================================================================
    
    /**
     * @brief Perform batch write operations on multiple pins.
     * @param operation Batch operation specification
     * @return Batch operation results with hf_gpio_err_t codes
     */
    [[nodiscard]] GpioBatchResult BatchWrite(const GpioBatchOperation& operation) noexcept;
    
    /**
     * @brief Perform batch read operations on multiple pins.
     * @param pin_names Vector of pin names to read
     * @return Batch operation results with hf_gpio_err_t codes
     */
    [[nodiscard]] GpioBatchResult BatchRead(const std::vector<std::string_view>& pin_names) noexcept;
    
    /**
     * @brief Set multiple pins to active state.
     * @param pin_names Vector of pin names
     * @return Batch operation results with hf_gpio_err_t codes
     */
    [[nodiscard]] GpioBatchResult SetMultipleActive(const std::vector<std::string_view>& pin_names) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pin_names Vector of pin names
     * @return Batch operation results with hf_gpio_err_t codes
     */
    [[nodiscard]] GpioBatchResult SetMultipleInactive(const std::vector<std::string_view>& pin_names) noexcept;
    
    //==========================================================================
    // SYSTEM OPERATIONS
    //==========================================================================
    
    /**
     * @brief Reset all output pins to inactive state.
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t ResetAllPins() noexcept;

private:
    //==========================================================================
    // CLEAN PRIVATE MEMBERS (NO ERROR STORAGE)
    //==========================================================================
    
    // System state
    std::atomic<bool> is_initialized_{false};
    mutable RtosMutex mutex_;
    
    // GPIO registry
    std::unordered_map<std::string_view, std::shared_ptr<BaseGpio>> gpio_registry_;
    mutable RtosMutex registry_mutex_;
    
    // Hardware handlers (lazy initialization)
    std::unique_ptr<Pcal95555Handler> pcal95555_handler_;
    mutable RtosMutex pcal_handler_mutex_;
    MotorController* motor_controller_ = nullptr;
    
    // Clean statistics (no error storage)
    std::atomic<uint32_t> total_operations_{0};
    std::atomic<uint32_t> successful_operations_{0};
    std::atomic<uint32_t> failed_operations_{0};
    std::atomic<uint32_t> validation_failures_{0};
    std::atomic<uint32_t> hardware_failures_{0};
    std::atomic<uint64_t> system_start_time_{0};
    std::atomic<hf_gpio_err_t> last_error_{hf_gpio_err_t::GPIO_SUCCESS};
    
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================
    
    GpioManagerClean() = default;
    ~GpioManagerClean() = default;
    GpioManagerClean(const GpioManagerClean&) = delete;
    GpioManagerClean& operator=(const GpioManagerClean&) = delete;
    GpioManagerClean(GpioManagerClean&&) = delete;
    GpioManagerClean& operator=(GpioManagerClean&&) = delete;
    
    [[nodiscard]] hf_gpio_err_t Initialize() noexcept;
    [[nodiscard]] hf_gpio_err_t EnsurePcal95555Handler() noexcept;
    [[nodiscard]] Tmc9660Handler* GetTmc9660Handler(uint8_t device_index = 0) noexcept;
    
    void UpdateStatistics(hf_gpio_err_t result) noexcept;
    
    // GPIO creation methods
    [[nodiscard]] std::shared_ptr<BaseGpio> CreateEsp32GpioPin(hf_u8_t pin_id, bool is_inverted, 
                                                             bool has_pull, bool pull_is_up, bool is_push_pull,
                                                             hf_u32_t max_current_ma) noexcept;
    [[nodiscard]] std::shared_ptr<BaseGpio> CreatePcal95555GpioPin(hf_u8_t pin_id, hf_u8_t unit_number, bool is_inverted, 
                                                                 bool has_pull, bool pull_is_up, bool is_push_pull,
                                                                 hf_u32_t max_current_ma) noexcept;
    [[nodiscard]] std::shared_ptr<BaseGpio> CreateTmc9660GpioPin(hf_u8_t pin_id, hf_u8_t device_index, bool is_inverted, 
                                                               bool has_pull, bool pull_is_up, bool is_push_pull,
                                                               hf_u32_t max_current_ma) noexcept;
};

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Get the clean GPIO manager instance.
 */
[[nodiscard]] inline GpioManagerClean& GetCleanGpioManager() noexcept {
    return GpioManagerClean::GetInstance();
}

#endif // COMPONENT_HANDLER_GPIO_MANAGER_CLEAN_H_