#ifndef COMPONENT_HANDLER_SYSTEM_INIT_H_
#define COMPONENT_HANDLER_SYSTEM_INIT_H_

#include "AdcData.h"
#include "GpioHandler.h"
#include "../CommonIDs.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_gpio_config.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_ext_pins_enum.hpp"

/**
 * @file SystemInit.h
 * @brief System initialization for ADC and GPIO subsystems.
 * 
 * This file provides functions to initialize and configure all ADC and GPIO
 * components in the HardFOC system.
 */

/**
 * @class SystemInit
 * @brief System initialization and configuration class.
 */
class SystemInit {
public:
    /**
     * @brief Initialize the complete ADC and GPIO system.
     * @return true if initialization successful, false otherwise.
     */
    static bool InitializeSystem() noexcept;
    
    /**
     * @brief Initialize the ADC subsystem.
     * @return true if initialization successful, false otherwise.
     */
    static bool InitializeAdcSystem() noexcept;
    
    /**
     * @brief Initialize the GPIO subsystem.
     * @return true if initialization successful, false otherwise.
     */
    static bool InitializeGpioSystem() noexcept;
    
    /**
     * @brief Register all ESP32-C6 native GPIO pins.
     * @return true if all pins registered successfully, false otherwise.
     */
    static bool RegisterNativeGpioPins() noexcept;
    
    /**
     * @brief Register all PCAL95555 GPIO expander pins.
     * @return true if all pins registered successfully, false otherwise.
     */
    static bool RegisterExpanderGpioPins() noexcept;
    
    /**
     * @brief Register all ESP32-C6 internal ADC channels.
     * @return true if all channels registered successfully, false otherwise.
     */
    static bool RegisterInternalAdcChannels() noexcept;
    
    /**
     * @brief Register all external ADC channels.
     * @return true if all channels registered successfully, false otherwise.
     */
    static bool RegisterExternalAdcChannels() noexcept;
    
    /**
     * @brief Run system self-test.
     * @return true if all tests pass, false otherwise.
     */
    static bool RunSystemSelfTest() noexcept;
    
    /**
     * @brief Get system health status.
     * @return true if system is healthy, false otherwise.
     */
    static bool GetSystemHealth() noexcept;
    
    /**
     * @brief Print system status information.
     */
    static void PrintSystemStatus() noexcept;
    
private:
    /**
     * @brief Create and register ESP32 native GPIO pins.
     * @return true if successful, false otherwise.
     */
    static bool CreateNativeGpioPins() noexcept;
    
    /**
     * @brief Create and register PCAL95555 GPIO pins.
     * @return true if successful, false otherwise.
     */
    static bool CreateExpanderGpioPins() noexcept;
    
    /**
     * @brief Create and register internal ADC channels.
     * @return true if successful, false otherwise.
     */
    static bool CreateInternalAdcChannels() noexcept;
    
    /**
     * @brief Create and register external ADC channels.
     * @return true if successful, false otherwise.
     */
    static bool CreateExternalAdcChannels() noexcept;
    
    /**
     * @brief Map functional pin names to hardware pins.
     * @return true if successful, false otherwise.
     */
    static bool MapFunctionalPins() noexcept;
    
    /**
     * @brief Map functional ADC channels to hardware channels.
     * @return true if successful, false otherwise.
     */
    static bool MapFunctionalAdcChannels() noexcept;
    
    /**
     * @brief Initialize I2C bus for GPIO expanders.
     * @return true if successful, false otherwise.
     */
    static bool InitializeI2cBus() noexcept;
};

/**
 * @brief Initialize the complete HardFOC system.
 * This is the main initialization function that should be called during system startup.
 * @return true if initialization successful, false otherwise.
 */
bool InitializeHardFocSystem() noexcept;

/**
 * @brief Get the current system status.
 * @return true if system is operational, false otherwise.
 */
bool GetHardFocSystemStatus() noexcept;

/**
 * @brief Run a comprehensive system test.
 * @return true if all tests pass, false otherwise.
 */
bool RunHardFocSystemTest() noexcept;

#endif // COMPONENT_HANDLER_SYSTEM_INIT_H_
