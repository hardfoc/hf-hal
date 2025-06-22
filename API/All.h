#ifndef COMPONENT_HANDLER_ALL_H_
#define COMPONENT_HANDLER_ALL_H_

/**
 * @file All.h
 * @brief Master include file for the HardFOC component handler system.
 * 
 * This file includes all the necessary headers for the comprehensive
 * ADC and multi-source GPIO management system. Include this single header 
 * to get access to all functionality including ESP32-C6, PCAL95555, and 
 * TMC9660 GPIO sources.
 */

// Core data types and enums
#include "../component-handler/CommonIDs.h"
#include "../component-handler/ThingsToString.h"

// Multi-reading support
#include "../component-handler/MultiReadings.h"
#include "../component-handler/AdcMultiCountReading.h"

// Core data management systems
#include "../component-handler/AdcData.h"
#include "../component-handler/GpioData.h"
#include "../component-handler/GpioHandler.h"// Comprehensive multi-source GPIO system

// Motor controller systems
#include "../component-handler/Tmc9660MotorController.h"
#include "../component-handler/Tmc9660Gpio.h"

// GPIO expander support
#include "../component-handler/Pcal95555Gpio.h"

// ESP32-C6 GPIO support
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/Esp32C6Gpio.h"

// System initialization and configuration
#include "../component-handler/SystemInit.h"
#include "../component-handler/HardFocIntegration.h"

// Testing and validation
#include "../component-handler/GpioSystemIntegrationTest.h"

// Hardware configuration with safety validation
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_gpio_config.hpp"
#include "../component-handler/hf_ext_pins_enum.hpp"

// Legacy compatibility wrappers (deprecated but maintained)
#include "../component-handler/AdcHandler.h"
#include "../component-handler/GpioHandler.h"

/**
 * @brief Namespace containing all HardFOC component handler functionality.
 */
namespace HardFocComponentHandler {
    // Type aliases for easy access
    using AdcSystem = AdcData;
    using GpioSystem = GpioDataNew;
    using MotorController = Tmc9660MotorController;
    using SystemInitializer = SystemInit;
    using Integration = HardFocIntegration;
      // Commonly used types
    using AdcSensor = AdcInputSensor;
    using GpioPin = ::GpioPin;
    using AdcChip = ::AdcChip;
    using GpioChip = ::GpioChip;
    using TimeUnit = ::TimeUnit;
    using Tmc9660ChipId = ::Tmc9660ChipId;
    using Tmc9660CommInterface = ::Tmc9660CommInterface;
    
    // Multi-reading types
    template<uint8_t N>
    using AdcMultiCount = AdcMultiCountReading<N>;
    
    template<uint8_t N>
    using AdcMultiVoltage = AdcMultiVoltageReading<N>;
    
    // Common multi-reading instances
    using AdcMultiCount8 = AdcMultiCountReading<8>;
    using AdcMultiCount16 = AdcMultiCountReading<16>;
    using AdcMultiVoltage8 = AdcMultiVoltageReading<8>;
    using AdcMultiVoltage16 = AdcMultiVoltageReading<16>;
    
    /**
     * @brief Quick initialization function for the entire system.
     * @return true if successful, false otherwise.
     */
    inline bool Initialize() noexcept {
        return InitializeHardFocForMain();
    }
    
    /**
     * @brief Quick health check function.
     * @return true if system is healthy, false otherwise.
     */
    inline bool IsHealthy() noexcept {
        return GetQuickHealthStatus();
    }
      /**
     * @brief Quick initialization function for the entire system.
     * @return true if successful, false otherwise.
     */
    inline bool Initialize() noexcept {
        return SystemInit::InitializeSystem() && InitializePrimaryTmc9660();
    }
    
    /**
     * @brief Quick health check function.
     * @return true if system is healthy, false otherwise.
     */
    inline bool IsHealthy() noexcept {
        return GetHardFocSystemStatus() && Tmc9660SystemHealthy();
    }
    
    /**
     * @brief Quick periodic maintenance function.
     */
    inline void Maintain() noexcept {
        RunPeriodicSystemMaintenance();
        GetTmc9660Controller().TestCommunication();
    }
    
    /**
     * @brief Get the ADC system instance.
     * @return Reference to the ADC system.
     */
    inline AdcSystem& GetAdcSystem() noexcept {
        return AdcData::GetInstance();
    }    /**
     * @brief Get the GPIO system instance.
     * @return Reference to the GPIO system.
     */
    inline GpioSystem& GetGpioSystem() noexcept {
        return GpioDataNew::GetInstance();
    }
    
    /**
     * @brief Get the motor controller system instance.
     * @return Reference to the motor controller system.
     */
    inline MotorController& GetMotorController() noexcept {
        return Tmc9660MotorController::GetInstance();
    }
    
    /**
     * @brief Demo all system functionality.
     */
    inline void RunDemo() noexcept {
        HardFocIntegration::DemoAdcUsage();
        HardFocIntegration::DemoGpioUsage();
        HardFocIntegration::DemoMultiChannelAdc();
        HardFocIntegration::DemoMultiPinGpio();
        HardFocIntegration::RunSystemDiagnostics();
    }
    
} // namespace HardFocComponentHandler

/**
 * @brief Convenient macro for initializing the HardFOC system.
 * Use this in your main function.
 */
#define HARDFOC_INIT() HardFocComponentHandler::Initialize()

/**
 * @brief Convenient macro for periodic system maintenance.
 * Use this in your main loop.
 */
#define HARDFOC_MAINTAIN() HardFocComponentHandler::Maintain()

/**
 * @brief Convenient macro for quick health check.
 * Returns true if system is healthy.
 */
#define HARDFOC_HEALTHY() HardFocComponentHandler::IsHealthy()

#endif // COMPONENT_HANDLER_ALL_H_
