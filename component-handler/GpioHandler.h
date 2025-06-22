#ifndef COMPONENT_HANDLER_GPIO_HANDLER_H_
#define COMPONENT_HANDLER_GPIO_HANDLER_H_

#include "GpioData.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "DigitalGpio.h"
#include "SfI2cBus.h"
#include "Tmc9660MotorController.h"
#include <string_view>
#include <memory>

/**
 * @file GpioHandler.h
 * @brief Main external interface for GPIO operations in the HardFOC system.
 * 
 * This class provides the primary interface for applications to interact with
 * GPIO pins. It acts as a high-level wrapper around the GpioData system,
 * providing convenient methods for common GPIO operations.
 * 
 * Key Features:
 * - Simple, intuitive API for GPIO operations
 * - Automatic error handling and logging
 * - Support for both pin IDs and names
 * - Batch operations for multiple pins
 * - Real-time system health monitoring
 * 
 * Architecture:
 * - GpioData: Handles all registration, setup, and low-level operations
 * - GpioHandler: Provides the main external interface for applications
 */

/**
 * @class GpioHandler
 * @brief Main external interface for GPIO operations.
 * 
 * This class provides the primary interface for applications to interact with
 * GPIO pins across all supported sources (ESP32-C6, PCAL95555, TMC9660).
 * 
 * All GPIO registration and setup is handled by GpioData, while GpioHandler
 * provides the convenient application interface.
 */
class GpioHandler {
public:
    /**
     * @brief Get the singleton instance.
     * @return Reference to the GpioHandler instance.
     */
    static GpioHandler& GetInstance() noexcept;

    /**
     * @brief Initialize the GPIO handler system.
     * 
     * This method initializes the entire GPIO system, including:
     * - ESP32-C6 native GPIO configuration
     * - PCAL95555 I2C expander setup
     * - TMC9660 GPIO integration
     * - All pin registrations and mappings
     * 
     * @param i2cBus Reference to the I2C bus for PCAL95555 communication
     * @param tmc9660Controller Reference to the TMC9660 controller
     * @return true if initialization successful, false otherwise
     */
    bool Initialize(SfI2cBus& i2cBus, Tmc9660MotorController& tmc9660Controller) noexcept;

    /**
     * @brief Check if the GPIO system is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept;

    //==========================================================================
    // Pin Control Methods
    //==========================================================================

    /**
     * @brief Set a GPIO pin to active state.
     * @param pin GPIO pin identifier
     * @return true if successful, false otherwise
     */
    bool SetActive(GpioPin pin) noexcept;

    /**
     * @brief Set a GPIO pin to inactive state.
     * @param pin GPIO pin identifier
     * @return true if successful, false otherwise
     */
    bool SetInactive(GpioPin pin) noexcept;

    /**
     * @brief Toggle a GPIO pin state.
     * @param pin GPIO pin identifier
     * @return true if successful, false otherwise
     */
    bool Toggle(GpioPin pin) noexcept;

    /**
     * @brief Check if a GPIO pin is active.
     * @param pin GPIO pin identifier
     * @return true if active, false if inactive or error
     */
    bool IsActive(GpioPin pin) noexcept;

    /**
     * @brief Get the current state of a GPIO pin.
     * @param pin GPIO pin identifier
     * @return Current pin state (Active/Inactive)
     */
    DigitalGpio::State GetState(GpioPin pin) noexcept;

    //==========================================================================
    // Name-based Pin Control Methods
    //==========================================================================

    /**
     * @brief Set a GPIO pin to active state by name.
     * @param name GPIO pin name
     * @return true if successful, false otherwise
     */
    bool SetActiveByName(std::string_view name) noexcept;

    /**
     * @brief Set a GPIO pin to inactive state by name.
     * @param name GPIO pin name
     * @return true if successful, false otherwise
     */
    bool SetInactiveByName(std::string_view name) noexcept;

    /**
     * @brief Toggle a GPIO pin state by name.
     * @param name GPIO pin name
     * @return true if successful, false otherwise
     */
    bool ToggleByName(std::string_view name) noexcept;

    /**
     * @brief Check if a GPIO pin is active by name.
     * @param name GPIO pin name
     * @return true if active, false if inactive or error
     */
    bool IsActiveByName(std::string_view name) noexcept;

    //==========================================================================
    // Batch Operations
    //==========================================================================

    /**
     * @brief Set multiple GPIO pins to active state.
     * @param pins Array of GPIO pin identifiers
     * @param count Number of pins in the array
     * @return Number of pins successfully set
     */
    uint8_t SetMultipleActive(const GpioPin* pins, uint8_t count) noexcept;

    /**
     * @brief Set multiple GPIO pins to inactive state.
     * @param pins Array of GPIO pin identifiers
     * @param count Number of pins in the array
     * @return Number of pins successfully set
     */
    uint8_t SetMultipleInactive(const GpioPin* pins, uint8_t count) noexcept;

    /**
     * @brief Read multiple GPIO pins at once.
     * @param specs Array of GpioReadSpec structures
     * @param count Number of pins to read
     * @return Number of pins successfully read
     */
    uint8_t ReadMultiplePins(GpioReadSpec* specs, uint8_t count) noexcept;

    //==========================================================================
    // System Information Methods
    //==========================================================================

    /**
     * @brief Get information about a registered GPIO pin.
     * @param pin GPIO pin identifier
     * @return Pointer to GpioInfo structure, or nullptr if not found
     */
    const GpioInfo* GetPinInfo(GpioPin pin) const noexcept;

    /**
     * @brief Get information about a registered GPIO pin by name.
     * @param name GPIO pin name
     * @return Pointer to GpioInfo structure, or nullptr if not found
     */
    const GpioInfo* GetPinInfoByName(std::string_view name) const noexcept;

    /**
     * @brief Get the total number of registered GPIO pins.
     * @return Number of registered pins
     */
    uint8_t GetRegisteredPinCount() const noexcept;

    /**
     * @brief Check if a GPIO source is communicating properly.
     * @param source GPIO source to check
     * @return true if communicating, false otherwise
     */
    bool IsSourceCommunicating(GpioChip source) const noexcept;

    /**
     * @brief Get overall system health status.
     * @return System health information
     */
    GpioSystemHealth GetSystemHealth() const noexcept;

    //==========================================================================
    // Advanced Operations
    //==========================================================================

    /**
     * @brief Perform a GPIO pin blink operation.
     * @param pin GPIO pin identifier
     * @param onTimeMs Time in milliseconds to stay active
     * @param offTimeMs Time in milliseconds to stay inactive
     * @param cycles Number of blink cycles (0 = infinite)
     * @return true if blink started successfully, false otherwise
     */
    bool BlinkPin(GpioPin pin, uint32_t onTimeMs, uint32_t offTimeMs, uint8_t cycles = 1) noexcept;

    /**
     * @brief Stop any ongoing blink operation on a pin.
     * @param pin GPIO pin identifier
     * @return true if stopped successfully, false otherwise
     */
    bool StopBlink(GpioPin pin) noexcept;

    /**
     * @brief Perform system diagnostics and health check.
     * @return true if all systems healthy, false if issues detected
     */
    bool RunDiagnostics() noexcept;    /**
     * @brief Reset all GPIO pins to their default states.
     * @return true if successful, false otherwise
     */
    bool ResetAllPins() noexcept;

    //==========================================================================
    // Health Monitoring
    //==========================================================================

    /**
     * @brief Check if ESP32-C6 GPIO system is healthy.
     * @return true if ESP32 GPIO is healthy, false otherwise
     */
    bool IsEsp32GpioHealthy() const noexcept;

    /**
     * @brief Check if PCAL95555 expander is healthy.
     * @return true if PCAL95555 is healthy, false otherwise
     */
    bool IsPcal95555Healthy() const noexcept;

    /**
     * @brief Check if TMC9660 GPIO system is healthy.
     * @return true if TMC9660 GPIO is healthy, false otherwise
     */
    bool IsTmc9660GpioHealthy() const noexcept;

    /**
     * @brief Check if the entire GPIO system is healthy.
     * @return true if all subsystems are healthy, false otherwise
     */
    bool IsSystemHealthy() const noexcept;

    // Delete copy constructor and assignment operator
    GpioHandler(const GpioHandler&) = delete;
    GpioHandler& operator=(const GpioHandler&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    GpioHandler() noexcept = default;

    /**
     * @brief Private destructor.
     */
    ~GpioHandler() noexcept = default;

    /**
     * @brief Get reference to the underlying GpioData system.
     * @return Reference to GpioData instance
     */
    GpioData& GetGpioData() noexcept;

    /**
     * @brief Initialize ESP32-C6 GPIO pins with safety validation.
     * @return true if successful, false otherwise.
     */
    bool InitializeEsp32Gpio() noexcept;
    
    /**
     * @brief Initialize PCAL95555 GPIO expander chips.
     * @param i2cBus Reference to I2C bus.
     * @return true if successful, false otherwise.
     */
    bool InitializePcal95555Gpio(SfI2cBus& i2cBus) noexcept;
    
    /**
     * @brief Initialize TMC9660 GPIO pins.
     * @param tmc9660Controller Reference to TMC9660 controller.
     * @return true if successful, false otherwise.
     */
    bool InitializeTmc9660Gpio(Tmc9660MotorController& tmc9660Controller) noexcept;
    
    /**
     * @brief Create ESP32-C6 GPIO instances for safe pins.
     * @return true if successful, false otherwise.
     */
    bool CreateEsp32GpioInstances() noexcept;
    
    /**
     * @brief Register safe ESP32-C6 pins with the GPIO system.
     * @return true if successful, false otherwise.
     */
    bool RegisterSafeEsp32Pins() noexcept;

    // Initialization and system state
    bool initialized_ = false;  ///< Initialization status
    
    // Source-specific health tracking
    bool esp32GpioHealthy_ = false;      ///< ESP32 GPIO system health status
    bool pcal95555Healthy_ = false;      ///< PCAL95555 expander health status
    bool tmc9660GpioHealthy_ = false;    ///< TMC9660 GPIO system health status
    
    // External system references (managed during initialization)
    SfI2cBus* i2cBus_ = nullptr;
    Tmc9660MotorController* tmc9660Controller_ = nullptr;
    
    // GPIO chip managers
    std::unique_ptr<Pcal95555Chip> pcal95555Chip_;
    std::unique_ptr<Tmc9660GpioManager> tmc9660Manager_;
    
    // ESP32-C6 GPIO instances storage
    std::vector<std::unique_ptr<DigitalGpio>> esp32GpioInstances_;
};

#endif // COMPONENT_HANDLER_GPIO_HANDLER_H_
