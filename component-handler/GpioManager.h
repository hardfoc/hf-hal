/**
 * @file GpioManager.h
 * @brief Consolidated GPIO management for the HardFOC system.
 * 
 * @details This class consolidates the functionality of GpioData and GpioHandler into
 *          a single, unified GPIO manager. It provides comprehensive GPIO operations
 *          across multiple sources (ESP32-C6, PCAL95555, TMC9660) with thread-safe
 *          operation and modern error handling.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * 
 * Key Features:
 * - Thread-safe operation with mutex protection
 * - Multi-source GPIO registration and management
 * - Unified error handling with Result<T> types
 * - Comprehensive health monitoring and diagnostics
 * - Singleton pattern for system-wide access
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 */

#ifndef COMPONENT_HANDLER_GPIO_MANAGER_H_
#define COMPONENT_HANDLER_GPIO_MANAGER_H_

#include "Result.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseGpio.h"
#include "SfI2cBus.h"
#include "Tmc9660MotorController.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
 * - Performance-optimized batch operations
 * - Hardware-agnostic functional pin identifiers
 * - Clean, modern C++ API
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Pcal95555Gpio;

//==============================================================================
// GPIO INFORMATION STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive GPIO pin information.
 */
struct GpioInfo {
    GpioPin pin;                    ///< Functional pin identifier
    std::unique_ptr<BaseGpio> gpio;  ///< GPIO driver instance
    std::string_view name;          ///< Human-readable name
    GpioChip source;               ///< GPIO source chip
    bool isRegistered;             ///< Registration status
    bool isInput;                  ///< Pin direction (true = input, false = output)
    bool currentState;             ///< Last known pin state
    uint32_t accessCount;          ///< Number of times accessed
    
    /**
     * @brief Constructor for GpioInfo.
     */
    GpioInfo(GpioPin p, std::unique_ptr<BaseGpio> g, std::string_view n, GpioChip s) noexcept
        : pin(p), gpio(std::move(g)), name(n), source(s), isRegistered(true),
          isInput(false), currentState(false), accessCount(0) {}
    
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
    std::vector<GpioPin> pins;      ///< Pins to operate on
    std::vector<bool> states;       ///< Desired states (for write operations)
    bool isWriteOperation;          ///< true for write, false for read
    
    /**
     * @brief Constructor for write operations.
     */
    GpioBatchOperation(std::vector<GpioPin> p, std::vector<bool> s) noexcept
        : pins(std::move(p)), states(std::move(s)), isWriteOperation(true) {}
    
    /**
     * @brief Constructor for read operations.
     */
    explicit GpioBatchOperation(std::vector<GpioPin> p) noexcept
        : pins(std::move(p)), isWriteOperation(false) {}
};

/**
 * @brief Structure for GPIO batch operation results.
 */
struct GpioBatchResult {
    std::vector<GpioPin> pins;          ///< Pins operated on
    std::vector<bool> states;           ///< Resulting states
    std::vector<ResultCode> results; ///< Individual operation results
    ResultCode overallResult;        ///< Overall operation result
    
    /**
     * @brief Check if all operations were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return IsSuccessResult(overallResult);
    }
};

//==============================================================================
// MAIN GPIO MANAGER CLASS
//==============================================================================

/**
 * @class GpioManager
 * @brief Consolidated GPIO manager for the HardFOC system.
 * 
 * This class provides a unified interface for all GPIO operations across
 * multiple hardware sources. It consolidates the functionality of the
 * previous GpioData and GpioHandler classes into a single, modern API.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - All operations return Result<T> types
 * - Comprehensive error codes via ResultCode enum
 * - Detailed error descriptions available
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple pins
 * - Cached state information
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
     * @brief Initialize the GPIO manager system.
     * 
     * This method initializes the entire GPIO system, including:
     * - ESP32-C6 native GPIO configuration
     * - PCAL95555 I2C expander setup
     * - TMC9660 GPIO integration
     * - All pin registrations and mappings
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
    
    //==========================================================================
    // PIN REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register a GPIO pin with the system.
     * @param pin Functional pin identifier
     * @param direction Pin direction (true = input, false = output)
     * @param initialState Initial state for output pins
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> RegisterPin(GpioPin pin, bool direction, 
                                          bool initialState = false) noexcept;
    
    /**
     * @brief Unregister a GPIO pin from the system.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> UnregisterPin(GpioPin pin) noexcept;
    
    /**
     * @brief Check if a GPIO pin is registered.
     * @param pin Functional pin identifier
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool IsPinRegistered(GpioPin pin) const noexcept;
    
    /**
     * @brief Get information about a registered GPIO pin.
     * @param pin Functional pin identifier
     * @return Result containing pin information or error
     */
    [[nodiscard]] Result<const GpioInfo*> GetPinInfo(GpioPin pin) const noexcept;
    
    //==========================================================================
    // BASIC PIN OPERATIONS
    //==========================================================================
    
    /**
     * @brief Set a GPIO pin to active state.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetActive(GpioPin pin) noexcept;
    
    /**
     * @brief Set a GPIO pin to inactive state.
     * @param pin Functional pin identifier
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetInactive(GpioPin pin) noexcept;
    
    /**
     * @brief Set a GPIO pin to a specific state.
     * @param pin Functional pin identifier
     * @param state Desired state (true = active, false = inactive)
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetState(GpioPin pin, bool state) noexcept;
    
    /**
     * @brief Toggle a GPIO pin state.
     * @param pin Functional pin identifier
     * @return Result containing the new state or error
     */
    [[nodiscard]] Result<bool> Toggle(GpioPin pin) noexcept;
    
    /**
     * @brief Read the current state of a GPIO pin.
     * @param pin Functional pin identifier
     * @return Result containing the pin state or error
     */
    [[nodiscard]] Result<bool> ReadState(GpioPin pin) noexcept;
    
    /**
     * @brief Check if a GPIO pin is currently active.
     * @param pin Functional pin identifier
     * @return Result containing active status or error
     */
    [[nodiscard]] Result<bool> IsActive(GpioPin pin) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch GPIO write operations.
     * @param operation Batch operation specification
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> BatchWrite(const GpioBatchOperation& operation) noexcept;
    
    /**
     * @brief Perform batch GPIO read operations.
     * @param pins Vector of pins to read
     * @return Result containing batch read results
     */
    [[nodiscard]] Result<GpioBatchResult> BatchRead(const std::vector<GpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to active state.
     * @param pins Vector of pins to activate
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> SetMultipleActive(const std::vector<GpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pins Vector of pins to deactivate
     * @return Result containing batch operation results
     */
    [[nodiscard]] Result<GpioBatchResult> SetMultipleInactive(const std::vector<GpioPin>& pins) noexcept;
    
    //==========================================================================
    // SYSTEM INFORMATION AND DIAGNOSTICS
    //==========================================================================
    
    /**
     * @brief Get the total number of registered GPIO pins.
     * @return Number of registered pins
     */
    [[nodiscard]] size_t GetRegisteredPinCount() const noexcept;
    
    /**
     * @brief Get a list of all registered GPIO pins.
     * @return Vector of registered pin identifiers
     */
    [[nodiscard]] std::vector<GpioPin> GetRegisteredPins() const noexcept;
    
    /**
     * @brief Get system health information.
     * @return Result containing health information or error
     */
    [[nodiscard]] Result<std::string> GetSystemHealth() const noexcept;
    
    /**
     * @brief Get detailed system statistics.
     * @return Result containing statistics or error
     */
    [[nodiscard]] Result<std::string> GetSystemStatistics() const noexcept;
    
    /**
     * @brief Reset all GPIO pins to their default states.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> ResetAllPins() noexcept;
    
    /**
     * @brief Perform system self-test.
     * @return Result indicating test success or specific failures
     */
    [[nodiscard]] Result<std::string> PerformSelfTest() noexcept;
    
    //==========================================================================
    // CONVENIENCE METHODS (NAME-BASED ACCESS)
    //==========================================================================
    
    /**
     * @brief Set a GPIO pin to active state using pin name.
     * @param pinName Human-readable pin name
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetActiveByName(std::string_view pinName) noexcept;
    
    /**
     * @brief Set a GPIO pin to inactive state using pin name.
     * @param pinName Human-readable pin name
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> SetInactiveByName(std::string_view pinName) noexcept;
    
    /**
     * @brief Read the current state of a GPIO pin using pin name.
     * @param pinName Human-readable pin name
     * @return Result containing the pin state or error
     */
    [[nodiscard]] Result<bool> ReadStateByName(std::string_view pinName) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    mutable std::mutex mutex_;                          ///< Thread safety mutex
    std::atomic<bool> isInitialized_{false};           ///< Initialization status
    
    // GPIO storage and management
    std::unordered_map<GpioPin, std::unique_ptr<GpioInfo>> gpioRegistry_;
    std::unordered_map<std::string_view, GpioPin> nameToPin_;
    
    // Hardware interfaces
    SfI2cBus* i2cBus_{nullptr};
    Tmc9660MotorController* tmc9660Controller_{nullptr};
    std::unique_ptr<Pcal95555Gpio> pcal95555Gpio_;
    
    // Statistics and diagnostics
    std::atomic<uint64_t> totalOperations_{0};
    std::atomic<uint64_t> successfulOperations_{0};
    std::atomic<uint64_t> failedOperations_{0};
    
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================
    
    /**
     * @brief Private constructor for singleton pattern.
     */
    GpioManager() = default;
    
    /**
     * @brief Initialize ESP32-C6 native GPIO pins.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> InitializeEsp32Gpio() noexcept;
    
    /**
     * @brief Initialize PCAL95555 GPIO expander.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> InitializePcal95555Gpio() noexcept;
    
    /**
     * @brief Initialize TMC9660 GPIO pins.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> InitializeTmc9660Gpio() noexcept;
    
    /**
     * @brief Create and register all default GPIO pins.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] Result<void> RegisterDefaultPins() noexcept;
    
    /**
     * @brief Find GPIO info by pin identifier.
     * @param pin Pin identifier
     * @return Pointer to GPIO info or nullptr if not found
     */
    [[nodiscard]] GpioInfo* FindGpioInfo(GpioPin pin) noexcept;
    
    /**
     * @brief Find GPIO info by pin identifier (const version).
     * @param pin Pin identifier
     * @return Pointer to GPIO info or nullptr if not found
     */
    [[nodiscard]] const GpioInfo* FindGpioInfo(GpioPin pin) const noexcept;
    
    /**
     * @brief Find GPIO pin by name.
     * @param name Pin name
     * @return Pin identifier or invalid pin if not found
     */
    [[nodiscard]] GpioPin FindPinByName(std::string_view name) const noexcept;
    
    /**
     * @brief Update internal statistics.
     * @param success Whether the operation was successful
     */
    void UpdateStatistics(bool success) noexcept;
    
    // Disable copy and move for singleton    GpioManager(const GpioManager&) = delete;
    GpioManager& operator=(const GpioManager&) = delete;
    GpioManager(GpioManager&&) = delete;
    GpioManager& operator=(GpioManager&&) = delete;
};

//==============================================================================
// CONVENIENCE FUNCTIONS
//==============================================================================

/**
 * @brief Get the global GPIO manager instance.
 * @return Reference to the GPIO manager
 */
[[nodiscard]] inline GpioManager& GetGpioManager() noexcept {
    return GpioManager::GetInstance();
}

#endif // COMPONENT_HANDLER_GPIO_MANAGER_H_
