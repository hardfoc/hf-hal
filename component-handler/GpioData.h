#ifndef COMPONENT_HANDLER_GPIO_DATA_H_
#define COMPONENT_HANDLER_GPIO_DATA_H_

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "DigitalGpio.h"

/**
 * @file GpioData.h
 * @brief Comprehensive, multi-source GPIO data management for the HardFOC system.
 *  * This class provides a unified interface for all GPIO operations in the system,
 * supporting multiple GPIO sources:
 * - ESP32-C6 native GPIO pins
 * - PCAL95555 I2C GPIO expander (single chip support)
 * - TMC9660 motor controller GPIO pins
 * - Future expandable GPIO sources
 *
 * Key Features:
 * - Thread-safe operation with mutex protection
 * - Multi-source GPIO registration and management
 * - Comprehensive error handling and diagnostics
 * - Performance-optimized batch operations
 */

/**
 * @brief Structure containing GPIO pin information from any source.
 */
struct GpioInfo {
    GpioPin pin;                 ///< Pin identifier from CommonIDs
    DigitalGpio& gpio;          ///< Reference to the GPIO driver
    std::string_view name;      ///< Human-readable name
    GpioChip source;            ///< GPIO source (ESP32, PCAL95555, TMC9660, etc.)
    bool isRegistered;          ///< Registration status
    
    /**
     * @brief Constructor for GPIO info.
     * @param p Pin identifier.
     * @param g GPIO driver reference.
     * @param n Human-readable name.
     * @param src GPIO source chip.
     */
    GpioInfo(GpioPin p, DigitalGpio& g, std::string_view n, GpioChip src = GpioChip::GPIO_ESP32_CHIP) 
        : pin(p), gpio(g), name(n), source(src), isRegistered(true) {}
};

/**
 * @brief Structure for GPIO reading specifications with state tracking.
 */
struct GpioReadSpec {
    GpioPin pin;                ///< Pin to read
    bool lastState;             ///< Previous state
    bool currentState;          ///< Current state
    bool stateChanged;          ///< State change flag
    uint32_t changeCount;       ///< Number of state changes
    uint32_t lastChangeTime;    ///< Timestamp of last change
    
    /**
     * @brief Constructor.
     * @param p Pin identifier.
     */
    GpioReadSpec(GpioPin p) 
        : pin(p), lastState(false), currentState(false), stateChanged(false),
          changeCount(0), lastChangeTime(0) {}
    
    /**
     * @brief Update the state and detect changes.
     * @param newState The new pin state.
     * @param timestamp Current timestamp.
     */
    void UpdateState(bool newState, uint32_t timestamp = 0) {
        lastState = currentState;
        currentState = newState;
        stateChanged = (lastState != currentState);
        if (stateChanged) {
            changeCount++;
            lastChangeTime = timestamp;
        }
    }
    
    /**
     * @brief Reset the state tracking.
     */
    void Reset() {
        lastState = false;
        currentState = false;
        stateChanged = false;
        changeCount = 0;
        lastChangeTime = 0;
    }
};

/**
 * @brief GPIO system health status.
 */
struct GpioSystemHealth {
    uint32_t totalRegisteredPins;      ///< Total registered pins
    uint32_t totalCommunicationErrors; ///< Total communication errors
    uint32_t lastHealthCheckTime;      ///< Last health check timestamp
    
    /**
     * @brief Check if the system has basic health indicators.
     * @return true if basic health indicators are met.
     */
    bool IsBasicallyHealthy() const {
        return totalRegisteredPins > 0;
    }
};

/**
 * @class GpioData
 * @brief Thread-safe singleton class for comprehensive GPIO data management.
 * 
 * This class provides centralized access to all GPIO pins in the system through
 * a unified interface, supporting multiple GPIO sources with automatic safety
 * validation and error handling.
 */
class GpioData {
public:    

    /**
     * @brief Get the singleton instance.
     * @return Reference to the GpioData instance.
     */
    static GpioData& GetInstance() noexcept;    
    
    /**
     * @brief Destructor.
     */
    ~GpioData() noexcept;    // Delete copy constructor and assignment operator
    GpioData(const GpioData&) = delete;
    GpioData& operator=(const GpioData&) = delete;
    
    //=====================================================================//
    /// SYSTEM HEALTH MANAGEMENT (GENERIC)
    //=====================================================================//
    
    /**
     * @brief Increment communication error count.
     */
    void IncrementCommunicationErrors() noexcept;

    //=====================================================================//
    /// GPIO REGISTRATION AND MANAGEMENT
    //=====================================================================//
    
    /**
     * @brief Register a GPIO pin from any source.
     * @param pin The pin identifier.
     * @param gpio Reference to the GPIO driver.
     * @param name Human-readable name for the pin.
     * @param source GPIO source chip (defaults to ESP32).
     * @return true if registered successfully, false otherwise.
     */
    bool RegisterGpioPin(GpioPin pin, DigitalGpio& gpio, std::string_view name,                        GpioChip source = GpioChip::GPIO_ESP32_CHIP) noexcept;
    
    /**
     * @brief Get the total number of registered pins.
     * @return Number of registered pins.
     */
    uint8_t GetRegisteredPinCount() const noexcept;
    
    /**
     * @brief Get the number of pins registered for a specific source.
     * @param source The GPIO source chip.
     * @return Number of pins from this source.
     */
    uint8_t GetRegisteredPinCount(GpioChip source) const noexcept;
    
    /**
     * @brief Get pin information by pin ID.
     * @param pin The pin identifier.
     * @return Pointer to GpioInfo or nullptr if not found.
     */
    const GpioInfo* GetPinInfo(GpioPin pin) const noexcept;
    
    /**
     * @brief Get pin by name.
     * @param name The pin name to search for.
     * @return Pointer to GpioInfo or nullptr if not found.
     */
    const GpioInfo* GetPinByName(std::string_view name) const noexcept;
    
    /**
     * @brief Get all pins from a specific source.
     * @param source The GPIO source chip.
     * @param pins Vector to store the pin information.
     * @return Number of pins found.
     */
    uint8_t GetPinsBySource(GpioChip source, std::vector<const GpioInfo*>& pins) const noexcept;

    //=====================================================================//
    /// SINGLE PIN OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Set a pin to active state.
     * @param pin The pin identifier.
     * @return true if successful, false otherwise.
     */
    bool SetActive(GpioPin pin) noexcept;
    
    /**
     * @brief Set a pin to inactive state.
     * @param pin The pin identifier.
     * @return true if successful, false otherwise.
     */
    bool SetInactive(GpioPin pin) noexcept;
    
    /**
     * @brief Toggle a pin state.
     * @param pin The pin identifier.
     * @return true if successful, false otherwise.
     */
    bool Toggle(GpioPin pin) noexcept;
    
    /**
     * @brief Check if a pin is active.
     * @param pin The pin identifier.
     * @return true if active, false otherwise.
     */
    bool IsActive(GpioPin pin) noexcept;

    //=====================================================================//
    /// NAME-BASED OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Set a pin state by name.
     * @param name The pin name.
     * @param active Desired active state.
     * @return true if successful, false otherwise.
     */
    bool SetPinByName(std::string_view name, bool active) noexcept;
    
    /**
     * @brief Get a pin state by name.
     * @param name The pin name.
     * @return true if active, false otherwise.
     */
    bool GetPinByName(std::string_view name) noexcept;
    
    /**
     * @brief Toggle a pin by name.
     * @param name The pin name.
     * @return true if successful, false otherwise.
     */
    bool TogglePinByName(std::string_view name) noexcept;

    //=====================================================================//
    /// MULTI-PIN OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Set multiple pins to active state.
     * @param pins Vector of pin identifiers.
     * @return true if all successful, false otherwise.
     */
    bool SetMultipleActive(const std::vector<GpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pins Vector of pin identifiers.
     * @return true if all successful, false otherwise.
     */
    bool SetMultipleInactive(const std::vector<GpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple GPIO pins to the same state.
     * @param pins Array of GPIO pin identifiers.
     * @param count Number of pins in the array.
     * @param active true to set active, false to set inactive.
     * @return Number of pins successfully set.
     */
    uint8_t SetMultiplePins(const GpioPin* pins, uint8_t count, bool active) noexcept;
    
    /**
     * @brief Read multiple GPIO pins at once.
     * @param specs Array of GpioReadSpec structures.
     * @param count Number of pins to read.
     * @return Number of pins successfully read.
     */
    uint8_t ReadMultiplePins(GpioReadSpec* specs, uint8_t count) noexcept;
    
    /**
     * @brief Read multiple pins with state change detection.
     * @param readSpecs Vector of GPIO read specifications.
     * @return true if all reads successful, false otherwise.
     */
    bool ReadMultiplePins(std::vector<GpioReadSpec>& readSpecs) noexcept;
    
    /**
     * @brief Set a pattern across multiple pins.
     * @param pins Vector of pin identifiers.
     * @param pattern Bit pattern to set (LSB = first pin).
     * @return true if successful, false otherwise.
     */
    bool SetPinPattern(const std::vector<GpioPin>& pins, uint32_t pattern) noexcept;
    
    /**
     * @brief Get a pattern from multiple pins.
     * @param pins Vector of pin identifiers.
     * @param pattern Reference to store the bit pattern.
     * @return true if successful, false otherwise.
     */
    bool GetPinPattern(const std::vector<GpioPin>& pins, uint32_t& pattern) noexcept;

    //=====================================================================//
    /// ADVANCED OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Generate a pulse on a pin.
     * @param pin The pin identifier.
     * @param duration_ms Pulse duration in milliseconds.
     * @return true if successful, false otherwise.
     */
    bool PulsePin(GpioPin pin, uint32_t duration_ms) noexcept;
    
    /**
     * @brief Blink a pin with specified timing.
     * @param pin The pin identifier.
     * @param on_time_ms On time in milliseconds.
     * @param off_time_ms Off time in milliseconds.
     * @param cycles Number of blink cycles.
     * @return true if successful, false otherwise.
     */
    bool BlinkPin(GpioPin pin, uint32_t on_time_ms, uint32_t off_time_ms, uint8_t cycles) noexcept;
    
    /**
     * @brief Perform a GPIO pin blink operation (simplified).
     * @param pin GPIO pin identifier.
     * @param onTimeMs Time in milliseconds to stay active.
     * @param offTimeMs Time in milliseconds to stay inactive.
     * @param cycles Number of blink cycles.
     * @return true if blink completed successfully, false otherwise.
     */
    bool BlinkPin(GpioPin pin, uint32_t onTimeMs, uint32_t offTimeMs, uint8_t cycles = 1) noexcept;
    
    /**
     * @brief Stop any ongoing blink operation on a pin (simplified).
     * @param pin GPIO pin identifier.
     * @return true if stopped successfully, false otherwise.
     */
    bool StopBlink(GpioPin pin) noexcept;
    
    /**
     * @brief Set status LEDs for system indication.
     * @param status_led Status LED pin.
     * @param error_led Error LED pin.
     * @param comm_led Communication LED pin.
     * @param status_on Status LED state.
     * @param error_on Error LED state.
     * @param comm_on Communication LED state.
     * @return true if successful, false otherwise.
     */
    bool SetLedStatus(GpioPin status_led, GpioPin error_led, GpioPin comm_led,
                      bool status_on, bool error_on, bool comm_on) noexcept;

    //=====================================================================//
    /// SYSTEM HEALTH AND DIAGNOSTICS
    //=====================================================================//
    
    /**
     * @brief Check if a GPIO source is communicating properly.
     * @param gpioChip The GPIO source to check.
     * @return true if communicating, false otherwise.
     */
    bool IsCommunicating(GpioChip gpioChip) noexcept;
    
    /**
     * @brief Check if a specific pin is responding.
     * @param pin The pin identifier.
     * @return true if responding, false otherwise.
     */
    bool IsResponding(GpioPin pin) noexcept;
    
    /**
     * @brief Run comprehensive GPIO system test.
     * @return true if all tests pass, false otherwise.
     */
    bool RunGpioTest() noexcept;
    
    /**
     * @brief Perform system diagnostics and health check.
     * @return true if all systems healthy, false if issues detected.
     */
    bool RunDiagnostics() noexcept;
    
    /**
     * @brief Reset all GPIO pins to their default states.
     * @return true if successful, false otherwise.
     */
    bool ResetAllPins() noexcept;
    
    /**
     * @brief Get overall system health status.
     * @return System health structure.
     */
    GpioSystemHealth GetSystemHealth() noexcept;
    
    /**
     * @brief Update system health status.
     * @return true if system is healthy, false otherwise.
     */
    bool UpdateSystemHealth() noexcept;

private:    /**
     * @brief Private constructor for singleton pattern.
     */
    GpioData() noexcept;
      /**
     * @brief Find GPIO info by pin (non-const version).
     * @param pin The pin identifier.
     * @return Pointer to GpioInfo or nullptr if not found.
     */
    GpioInfo* FindGpioInfo(GpioPin pin) noexcept;
    
    /**
     * @brief Find GPIO info by name (non-const version).
     * @param name The pin name.
     * @return Pointer to GpioInfo or nullptr if not found.
     */
    GpioInfo* FindGpioInfoByName(std::string_view name) noexcept;    // Thread safety
    mutable std::mutex mutex_;                    ///< Mutex for thread safety
    
    // GPIO storage
    std::array<GpioInfo, static_cast<uint32_t>(GpioPin::GPIO_PIN_COUNT)> gpioTable_;  ///< GPIO pin table
    uint8_t registeredPinCount_;                  ///< Number of registered pins
    
    // System health tracking
    GpioSystemHealth systemHealth_;               ///< System health status
    uint32_t lastHealthUpdate_;                   ///< Last health update time
    uint32_t communicationErrorCount_;            ///< Total communication errors
};

#endif // COMPONENT_HANDLER_GPIO_DATA_H_
