#ifndef COMPONENT_HANDLER_ALL_H_
#define COMPONENT_HANDLER_ALL_H_

/**
 * @file All.h
 * @brief Consolidated header for the modernized HardFOC component handler system.
 * 
 * This header provides convenient access to all core component handler classes
 * for hardware-agnostic GPIO and ADC operations using the new unified manager
 * architecture.
 * 
 * Key Features:
 * - Unified manager classes (GPIO, ADC)
 * - Modern Result<T> error handling
 * - X-macro based error codes
 * - Hardware-agnostic functional identifiers
 * - Thread-safe operations
 * - Comprehensive diagnostics
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

//==============================================================================
// CORE SYSTEM INCLUDES
//==============================================================================

// Modern unified error handling and result types
#include "Result.h"

// Platform mapping and functional identifiers
#include "../utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"
#include "CommonIDs.h"

// New consolidated manager classes
#include "GpioManager.h"
#include "AdcManager.h"

// Hardware integration classes (still needed for initialization)
#include "Tmc9660MotorController.h"
#include "TMC9660Controller.h"
#include "Pcal95555Gpio.h"

// Utility classes
#include "ThingsToString.h"

//==============================================================================
// LEGACY COMPATIBILITY INCLUDES
//==============================================================================

/**
 * @brief Legacy includes for backward compatibility.
 * 
 * @deprecated These classes are replaced by the new manager classes.
 * Include only for transitional code that hasn't been migrated yet.
 * 
 * Migration Guide:
 * - GpioData + GpioHandler -> GpioManager
 * - AdcData + AdcHandler -> AdcManager
 */

//==============================================================================
// CONVENIENCE NAMESPACE
//==============================================================================

/**
 * @brief Convenience namespace containing all component handler functionality.
 * 
 * This namespace provides easy access to the new manager classes and
 * common operations without needing to remember specific class names.
 */
namespace HardFocComponentHandler {
    
    //==========================================================================
    // MODERN TYPE ALIASES
    //==========================================================================
    
    /**
     * @brief Modern GPIO manager type alias.
     */
    using GpioManager = ::GpioManager;
    
    /**
     * @brief Modern ADC manager type alias.
     */
    using AdcManager = ::AdcManager;
    
    /**
     * @brief Result type alias for convenience.
     */
    template<typename T = void>
    using Result = ::Result<T>;
    
    /**
     * @brief Error code type alias.
     */
    using ErrorCode = ResultCode;
    
    /**
     * @brief GPIO pin identifier type.
     */
    using GpioPin = ::GpioPin;
    
    /**
     * @brief ADC sensor identifier type.
     */
    using AdcSensor = ::AdcInputSensor;
    
    //==========================================================================
    // CONVENIENCE FUNCTIONS
    //==========================================================================
    
    /**
     * @brief Get the global GPIO manager instance.
     * @return Reference to the GPIO manager
     */    [[nodiscard]] inline GpioManager& GetGpio() noexcept {
        return ::GpioManager::GetInstance();
    }
    
    /**
     * @brief Get the global ADC manager instance.
     * @return Reference to the ADC manager
     */    [[nodiscard]] inline AdcManager& GetAdc() noexcept {
        return ::AdcManager::GetInstance();
    }
    
    /**
     * @brief Initialize the entire component handler system.
     * 
     * This function initializes both GPIO and ADC managers with all
     * required hardware interfaces.
     * 
     * @param i2cBus Reference to I2C bus for GPIO expander
     * @param tmc9660Controller Reference to TMC9660 controller
     * @return Result indicating success or specific error
     */
    [[nodiscard]] inline Result<void> Initialize(SfI2cBus& i2cBus, 
                                                Tmc9660MotorController& tmc9660Controller) noexcept {
        // Initialize GPIO manager
        auto gpioResult = GetGpio().Initialize(i2cBus, tmc9660Controller);
        if (gpioResult.IsError()) {
            return gpioResult;
        }
        
        // Initialize ADC manager
        auto adcResult = GetAdc().Initialize(tmc9660Controller);
        if (adcResult.IsError()) {
            return adcResult;
        }
        
        return HARDFOC_SUCCESS();
    }
    
    /**
     * @brief Shutdown the entire component handler system.
     * @return Result indicating success or specific error
     */
    [[nodiscard]] inline Result<void> Shutdown() noexcept {
        // Shutdown managers in reverse order
        auto adcResult = GetAdc().Shutdown();
        auto gpioResult = GetGpio().Shutdown();
        
        // Return the first error encountered, or success if both succeed
        if (adcResult.IsError()) return adcResult;
        if (gpioResult.IsError()) return gpioResult;
        
        return HARDFOC_SUCCESS();
    }
    
    /**
     * @brief Check if the component handler system is fully initialized.
     * @return true if both GPIO and ADC managers are initialized
     */
    [[nodiscard]] inline bool IsInitialized() noexcept {
        return GetGpio().IsInitialized() && GetAdc().IsInitialized();
    }
    
    /**
     * @brief Get comprehensive system health information.
     * @return Result containing health information or error
     */
    [[nodiscard]] inline Result<std::string> GetSystemHealth() noexcept {
        auto gpioHealth = GetGpio().GetSystemHealth();
        auto adcHealth = GetAdc().GetSystemHealth();
        
        if (gpioHealth.IsError()) return gpioHealth;
        if (adcHealth.IsError()) return adcHealth;
        
        std::string combinedHealth = "=== HardFOC Component Handler System Health ===\n\n";
        combinedHealth += "GPIO System:\n" + gpioHealth.GetValue() + "\n\n";
        combinedHealth += "ADC System:\n" + adcHealth.GetValue() + "\n";
        
        return Result<std::string>(std::move(combinedHealth));
    }
    
    /**
     * @brief Perform comprehensive system self-test.
     * @return Result containing test results or error
     */
    [[nodiscard]] inline Result<std::string> PerformSelfTest() noexcept {
        auto gpioTest = GetGpio().PerformSelfTest();
        auto adcTest = GetAdc().PerformSelfTest();
        
        std::string combinedResults = "=== HardFOC Component Handler Self-Test Results ===\n\n";
        
        if (gpioTest.IsSuccess()) {
            combinedResults += "GPIO Test: PASSED\n" + gpioTest.GetValue() + "\n\n";
        } else {
            combinedResults += "GPIO Test: FAILED\nError: " + 
                              std::string(GetResultDescription(gpioTest.GetResult())) + "\n\n";
        }
        
        if (adcTest.IsSuccess()) {
            combinedResults += "ADC Test: PASSED\n" + adcTest.GetValue() + "\n";
        } else {
            combinedResults += "ADC Test: FAILED\nError: " + 
                              std::string(GetResultDescription(adcTest.GetResult())) + "\n";
        }
        
        // Determine overall result
        ErrorCode overallResult = ErrorCode::SUCCESS;
        if (gpioTest.IsError() || adcTest.IsError()) {
            overallResult = ErrorCode::ERROR_SYSTEM_FAULT;
        }
        
        return Result<std::string>(overallResult == ErrorCode::SUCCESS ? 
                                  Result<std::string>(std::move(combinedResults)) :
                                  Result<std::string>(overallResult));
    }    
} // namespace HardFocComponentHandler

//==============================================================================
// GLOBAL CONVENIENCE ALIASES
//==============================================================================

/**
 * @brief Global alias for the component handler namespace.
 */
namespace HFCH = HardFocComponentHandler;

#endif // COMPONENT_HANDLER_ALL_H_