#ifndef COMPONENT_HANDLER_GPIO_DATA_H_
#define COMPONENT_HANDLER_GPIO_DATA_H_

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>

#include "../CommonIDs.h"
#include "../ThingsToString.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/DigitalGpio.h"

/**
 * @file GpioData.h
 * @brief Comprehensive GPIO data management for the HardFOC system.
 * 
 * This class provides a unified interface for all GPIO operations in the system,
 * supporting both internal ESP32-C6 GPIO and external GPIO expanders.
 */

/**
 * @brief Structure containing GPIO pin information.
 */
struct GpioInfo {
    GpioPin pin;                 ///< Pin identifier
    DigitalGpio& gpio;          ///< Reference to the GPIO driver
    std::string_view name;      ///< Human-readable name
    
    GpioInfo(GpioPin p, DigitalGpio& g, std::string_view n) 
        : pin(p), gpio(g), name(n) {}
};

/**
 * @brief Structure for GPIO reading specifications.
 */
struct GpioReadSpec {
    GpioPin pin;
    bool lastState;
    bool currentState;
    bool stateChanged;
    
    GpioReadSpec(GpioPin p) 
        : pin(p), lastState(false), currentState(false), stateChanged(false) {}
    
    void UpdateState(bool newState) {
        lastState = currentState;
        currentState = newState;
        stateChanged = (lastState != currentState);
    }
    
    void Reset() {
        lastState = false;
        currentState = false;
        stateChanged = false;
    }
};

/**
 * @class GpioData
 * @brief Thread-safe singleton class for GPIO data management.
 * 
 * This class provides access to all GPIO pins in the system through a unified interface.
 * It manages both internal ESP32-C6 GPIO and external GPIO expanders like PCAL95555.
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
    ~GpioData() noexcept;
    
    // Delete copy constructor and assignment operator
    GpioData(const GpioData&) = delete;
    GpioData& operator=(const GpioData&) = delete;
    
    /**
     * @brief Ensure the GPIO system is initialized.
     * @return true if initialization successful, false otherwise.
     */
    bool EnsureInitialized() noexcept;
    
    //=====================================================================//
    /// GPIO REGISTRATION AND MANAGEMENT
    //=====================================================================//
    
    /**
     * @brief Register a GPIO pin.
     * @param pin The pin identifier.
     * @param gpio Reference to the GPIO driver.
     * @param name Human-readable name for the pin.
     * @return true if registered successfully, false otherwise.
     */
    bool RegisterGpioPin(GpioPin pin, DigitalGpio& gpio, std::string_view name) noexcept;
    
    /**
     * @brief Get the total number of registered pins.
     * @return Number of registered pins.
     */
    uint8_t GetRegisteredPinCount() const noexcept;
    
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
    
    //=====================================================================//
    /// SINGLE PIN OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Set a pin to active state.
     * @param pin The pin to set active.
     * @return true if successful, false otherwise.
     */
    bool SetActive(GpioPin pin) noexcept;
    
    /**
     * @brief Set a pin to inactive state.
     * @param pin The pin to set inactive.
     * @return true if successful, false otherwise.
     */
    bool SetInactive(GpioPin pin) noexcept;
    
    /**
     * @brief Toggle a pin state.
     * @param pin The pin to toggle.
     * @return true if successful, false otherwise.
     */
    bool Toggle(GpioPin pin) noexcept;
    
    /**
     * @brief Check if a pin is active.
     * @param pin The pin to check.
     * @return true if active, false if inactive or error.
     */
    bool IsActive(GpioPin pin) noexcept;
    
    /**
     * @brief Set pin state by name.
     * @param name The pin name.
     * @param active true to set active, false to set inactive.
     * @return true if successful, false otherwise.
     */
    bool SetPinByName(std::string_view name, bool active) noexcept;
    
    /**
     * @brief Get pin state by name.
     * @param name The pin name.
     * @return true if active, false if inactive or not found.
     */
    bool GetPinByName(std::string_view name) noexcept;
    
    /**
     * @brief Toggle pin by name.
     * @param name The pin name.
     * @return true if successful, false otherwise.
     */
    bool TogglePinByName(std::string_view name) noexcept;
    
    //=====================================================================//
    /// MULTI PIN OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Set multiple pins to active state.
     * @param pins Vector of pins to set active.
     * @return true if all successful, false if any failed.
     */
    bool SetMultipleActive(const std::vector<GpioPin>& pins) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pins Vector of pins to set inactive.
     * @return true if all successful, false if any failed.
     */
    bool SetMultipleInactive(const std::vector<GpioPin>& pins) noexcept;
    
    /**
     * @brief Read multiple pins state.
     * @param readSpecs Vector of GPIO read specifications.
     * @return true if all successful, false if any failed.
     */
    bool ReadMultiplePins(std::vector<GpioReadSpec>& readSpecs) noexcept;
    
    /**
     * @brief Set pin pattern from a bitmask.
     * @param pins Vector of pins to set.
     * @param pattern Bitmask pattern (LSB = first pin).
     * @return true if successful, false otherwise.
     */
    bool SetPinPattern(const std::vector<GpioPin>& pins, uint32_t pattern) noexcept;
    
    /**
     * @brief Get pin pattern as a bitmask.
     * @param pins Vector of pins to read.
     * @param pattern Reference to store the bitmask.
     * @return true if successful, false otherwise.
     */
    bool GetPinPattern(const std::vector<GpioPin>& pins, uint32_t& pattern) noexcept;
    
    //=====================================================================//
    /// SPECIALIZED OPERATIONS
    //=====================================================================//
    
    /**
     * @brief Pulse a pin (set active, delay, set inactive).
     * @param pin The pin to pulse.
     * @param duration_ms Pulse duration in milliseconds.
     * @return true if successful, false otherwise.
     */
    bool PulsePin(GpioPin pin, uint32_t duration_ms) noexcept;
    
    /**
     * @brief Blink a pin with specified on/off times.
     * @param pin The pin to blink.
     * @param on_time_ms Time to stay on in milliseconds.
     * @param off_time_ms Time to stay off in milliseconds.
     * @param cycles Number of blink cycles.
     * @return true if successful, false otherwise.
     */
    bool BlinkPin(GpioPin pin, uint32_t on_time_ms, uint32_t off_time_ms, uint8_t cycles) noexcept;
    
    /**
     * @brief Set LED status pattern.
     * @param status_led The status LED pin.
     * @param error_led The error LED pin.
     * @param comm_led The communication LED pin.
     * @param status_on Status LED state.
     * @param error_on Error LED state.
     * @param comm_on Communication LED state.
     * @return true if successful, false otherwise.
     */
    bool SetLedStatus(GpioPin status_led, GpioPin error_led, GpioPin comm_led,
                      bool status_on, bool error_on, bool comm_on) noexcept;
    
    //=====================================================================//
    /// STATUS AND HEALTH CHECKS
    //=====================================================================//
    
    /**
     * @brief Check if a GPIO chip is communicating.
     * @param gpioChip The GPIO chip to check.
     * @return true if communicating, false otherwise.
     */
    bool IsCommunicating(GpioChip gpioChip) noexcept;
    
    /**
     * @brief Check if a GPIO pin is responding.
     * @param pin The GPIO pin to check.
     * @return true if responding, false otherwise.
     */
    bool IsResponding(GpioPin pin) noexcept;
    
    /**
     * @brief Run a comprehensive GPIO test.
     * @return true if all tests pass, false otherwise.
     */
    bool RunGpioTest() noexcept;
    
    /**
     * @brief Get GPIO system health status.
     * @return true if system is healthy, false otherwise.
     */
    bool GetSystemHealth() noexcept;
    
private:
    /**
     * @brief Private constructor for singleton.
     */
    GpioData() noexcept;
    
    /**
     * @brief Initialize the GPIO system.
     * @return true if successful, false otherwise.
     */
    bool Initialize() noexcept;
    
    /**
     * @brief Find GPIO info for a pin.
     * @param pin The pin to find.
     * @return Pointer to GpioInfo or nullptr if not found.
     */
    GpioInfo* FindGpioInfo(GpioPin pin) noexcept;
    
    /**
     * @brief Find GPIO info by name.
     * @param name The pin name to find.
     * @return Pointer to GpioInfo or nullptr if not found.
     */
    GpioInfo* FindGpioInfoByName(std::string_view name) noexcept;
    
    // Member variables
    mutable std::mutex mutex_;                      ///< Mutex for thread safety
    std::atomic<bool> initialized_;                 ///< Initialization status
    
    std::array<GpioInfo, static_cast<uint32_t>(GpioPin::GPIO_PIN_COUNT)> gpioTable_;
    uint8_t registeredPinCount_;                    ///< Number of registered pins
};

#endif // COMPONENT_HANDLER_GPIO_DATA_H_
