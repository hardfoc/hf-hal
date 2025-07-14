/**
 * @file All.h
 * @brief Comprehensive header file for the HardFOC Component Handler system.
 * 
 * @details This header provides access to all major components of the HardFOC
 *          system, including the advanced GPIO management system with platform
 *          mapping integration, ADC management, TMC9660 motor controller,
 *          and PCAL95555 GPIO expander support.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Components:
 * - Advanced GPIO Manager with platform mapping integration
 * - ADC Manager for multi-chip ADC support
 * - TMC9660 Motor Controller with comprehensive GPIO/ADC integration
 * - PCAL95555 GPIO Expander wrapper with BaseGpio interface
 * - System initialization and configuration
 * - Error handling and result management
 * - Thread-safe utilities and RTOS integration
 * 
 * Architecture:
 * - Platform mapping integration for hardware-agnostic pin management
 * - Multi-chip GPIO support (ESP32, PCAL95555, TMC9660)
 * - Functional pin abstraction with hardware resource mapping
 * - Comprehensive error handling with Result<T> types
 * - Thread-safe operation with modern C++ patterns
 * 
 * @note This header provides the complete interface for the HardFOC HAL system.
 * @note All components use functional pin identifiers and platform mapping.
 */

#ifndef COMPONENT_HANDLER_ALL_H_
#define COMPONENT_HANDLER_ALL_H_

//==============================================================================
// CORE SYSTEM COMPONENTS
//==============================================================================

/**
 * @brief Core result and error handling system.
 * 
 * Provides the Result<T> template class for comprehensive error handling
 * across the entire HardFOC system.
 */
#include "Result.h"

/**
 * @brief Common identifiers and type definitions.
 * 
 * Provides hardware-agnostic functional identifiers for GPIO pins and ADC
 * channels, along with platform mapping integration.
 */
#include "CommonIDs.h"

/**
 * @brief System initialization and configuration.
 * 
 * Provides system-wide initialization and configuration management.
 */
#include "SystemInit.h"

/**
 * @brief Utility functions for string conversion and formatting.
 * 
 * Provides utilities for converting various types to strings and
 * formatting output for debugging and logging.
 */
#include "ThingsToString.h"

//==============================================================================
// ADVANCED GPIO MANAGEMENT SYSTEM
//==============================================================================

/**
 * @brief Advanced GPIO management system with platform mapping integration.
 * 
 * This is the main GPIO management system that integrates with the platform
 * mapping system to automatically manage GPIOs from multiple hardware sources
 * (ESP32-C6, PCAL95555, TMC9660) based on functional pin identifiers and
 * hardware chip mappings.
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
 * - Interrupt support for compatible pins
 * - Comprehensive pin configuration and statistics
 * 
 * Usage:
 * @code
 * // Get the GPIO manager instance
 * auto& gpio_manager = GpioManager::GetInstance();
 * 
 * // Initialize with hardware interfaces
 * auto result = gpio_manager.Initialize(i2c_bus, tmc9660_controller);
 * if (result.IsError()) {
 *     // Handle initialization error
 * }
 * 
 * // Use functional pin identifiers
 * auto set_result = gpio_manager.SetActive(HardFOC::FunctionalGpioPin::MOTOR_ENABLE);
 * auto read_result = gpio_manager.ReadState(HardFOC::FunctionalGpioPin::MOTOR_FAULT_STATUS);
 * 
 * // Batch operations for performance
 * std::vector<HardFOC::FunctionalGpioPin> pins = {
 *     HardFOC::FunctionalGpioPin::LED_STATUS_OK,
 *     HardFOC::FunctionalGpioPin::LED_STATUS_ERROR
 * };
 * auto batch_result = gpio_manager.SetMultipleActive(pins);
 * 
 * // Get system diagnostics
 * auto diagnostics = gpio_manager.GetSystemDiagnostics();
 * @endcode
 */
#include "GpioManager.h"

/**
 * @brief PCAL95555 GPIO expander wrapper with BaseGpio interface.
 * 
 * Provides a comprehensive wrapper for the PCAL95555 I2C GPIO expander that
 * implements the BaseGpio interface for seamless integration with the GPIO
 * management system.
 * 
 * Key Features:
 * - Full BaseGpio interface implementation
 * - Thread-safe I2C communication
 * - Comprehensive error handling and diagnostics
 * - Support for all PCAL95555 features (16 GPIO pins)
 * - Hardware abstraction with BaseI2c interface
 * - Pin-level statistics and health monitoring
 * - Interrupt support for input pins
 * - Configurable pull-up/pull-down resistors
 * - Output drive strength configuration
 * 
 * Architecture:
 * - Uses BaseI2c for hardware abstraction
 * - Implements BaseGpio for GPIO interface compatibility
 * - Provides Pcal95555GpioPin for individual pin management
 * - Supports both chip-level and pin-level operations
 * - Includes comprehensive diagnostics and health monitoring
 * 
 * Usage:
 * @code
 * // Create wrapper with I2C bus
 * auto wrapper = CreatePcal95555GpioWrapper(i2c_bus, i2c_address);
 * 
 * // Create individual GPIO pins
 * auto pin = wrapper->CreateGpioPin(Pcal95555Chip1Pin::PIN_0);
 * 
 * // Use standard BaseGpio interface
 * pin->SetActive();
 * bool state = pin->IsActive();
 * 
 * // Configure pin properties
 * pin->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
 * pin->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
 * @endcode
 */
#include "Pcal95555GpioWrapper.h"

//==============================================================================
// ADC MANAGEMENT SYSTEM
//==============================================================================

/**
 * @brief ADC management system for multi-chip ADC support.
 * 
 * Provides comprehensive ADC management across multiple hardware sources
 * including ESP32-C6 internal ADC, TMC9660 integrated ADC, and external
 * ADC devices.
 * 
 * Key Features:
 * - Multi-chip ADC support (ESP32, TMC9660, external)
 * - Functional channel abstraction with platform mapping
 * - Thread-safe operation with comprehensive error handling
 * - Automatic channel registration and configuration
 * - Advanced filtering and averaging capabilities
 * - Real-time monitoring and diagnostics
 * - Calibration and compensation support
 * - Batch operations for multiple channels
 * 
 * Usage:
 * @code
 * // Get ADC manager instance
 * auto& adc_manager = AdcManager::GetInstance();
 * 
 * // Initialize ADC system
 * auto result = adc_manager.Initialize();
 * 
 * // Read ADC values using functional channel identifiers
 * uint32_t count;
 * auto read_result = adc_manager.GetCount(
 *     HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_A,
 *     count, 1, 0, TimeUnit::TIME_UNIT_MS
 * );
 * 
 * // Read voltage values
 * float voltage;
 * auto voltage_result = adc_manager.GetVolt(
 *     HardFOC::FunctionalAdcChannel::SYSTEM_VOLTAGE_3V3,
 *     voltage, 1, 0, TimeUnit::TIME_UNIT_MS
 * );
 * @endcode
 */
#include "AdcManager.h"

//==============================================================================
// TMC9660 MOTOR CONTROLLER SYSTEM
//==============================================================================

/**
 * @brief TMC9660 motor controller with comprehensive GPIO/ADC integration.
 * 
 * Provides a comprehensive motor controller system that integrates TMC9660
 * GPIO pins, ADC channels, and motor control functionality with the broader
 * HardFOC system.
 * 
 * Key Features:
 * - Complete TMC9660 chip management
 * - Integrated GPIO pin control (GPIO17, GPIO18)
 * - Integrated ADC channel support (AIN1, AIN2, AIN3)
 * - Motor control signal management
 * - Communication interface switching (SPI/UART)
 * - Fault detection and handling
 * - Power management (sleep/wake)
 * - Comprehensive diagnostics and health monitoring
 * - Thread-safe operation with error handling
 * 
 * Architecture:
 * - Singleton pattern for system-wide access
 * - Integration with GPIO and ADC managers
 * - Hardware resource mapping via platform configuration
 * - Comprehensive status tracking and diagnostics
 * - Error recovery and fault handling
 * 
 * Usage:
 * @code
 * // Get TMC9660 controller instance
 * auto& controller = Tmc9660MotorController::GetInstance();
 * 
 * // Initialize the controller
 * auto result = controller.Initialize();
 * 
 * // Register TMC9660 chip with configuration
 * Tmc9660Config config = {
 *     .chipId = Tmc9660ChipId::TMC9660_CHIP_1,
 *     .primaryInterface = Tmc9660CommInterface::TMC_COMM_SPI,
 *     // ... other configuration
 * };
 * 
 * Tmc9660GpioConfig gpio_config = {
 *     .gpio17Pin = HardFOC::FunctionalGpioPin::USER_OUTPUT_1,
 *     .gpio18Pin = HardFOC::FunctionalGpioPin::USER_OUTPUT_2,
 *     // ... other GPIO configuration
 * };
 * 
 * Tmc9660AdcConfig adc_config = {
 *     .ain1Sensor = HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_A,
 *     .ain2Sensor = HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_B,
 *     .ain3Sensor = HardFOC::FunctionalAdcChannel::MOTOR_CURRENT_PHASE_C
 * };
 * 
 * auto register_result = controller.RegisterTmc9660Chip(
 *     Tmc9660ChipId::TMC9660_CHIP_1, config, gpio_config, adc_config
 * );
 * 
 * // Control motor functions
 * controller.EnableDriver(Tmc9660ChipId::TMC9660_CHIP_1);
 * controller.EnableSpiCommunication(Tmc9660ChipId::TMC9660_CHIP_1);
 * 
 * // Read ADC values
 * uint32_t adc_value;
 * controller.ReadAdcValue(Tmc9660ChipId::TMC9660_CHIP_1, 1, adc_value);
 * 
 * // Control GPIO pins
 * controller.SetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 17, true);
 * bool gpio_state = controller.GetGpioState(Tmc9660ChipId::TMC9660_CHIP_1, 18);
 * 
 * // System diagnostics
 * auto health = controller.GetSystemHealth();
 * controller.PrintSystemStatus();
 * @endcode
 */
#include "Tmc9660MotorController.h"

/**
 * @brief TMC9660 GPIO pin management.
 * 
 * Provides direct access to TMC9660 GPIO pins (GPIO17, GPIO18) with
 * BaseGpio interface compatibility.
 * 
 * Key Features:
 * - BaseGpio interface implementation
 * - Direct TMC9660 GPIO pin access
 * - Thread-safe operation
 * - Error handling and diagnostics
 * - Integration with GPIO manager
 * 
 * Usage:
 * @code
 * // Create TMC9660 GPIO pin
 * auto gpio = std::make_unique<Tmc9660Gpio>(Tmc9660ChipId::TMC9660_CHIP_1, 17);
 * 
 * // Use standard BaseGpio interface
 * gpio->SetActive();
 * bool state = gpio->IsActive();
 * @endcode
 */
#include "Tmc9660Gpio.h"

/**
 * @brief TMC9660 controller interface.
 * 
 * Provides a simplified interface for TMC9660 motor controller operations
 * and configuration.
 * 
 * Key Features:
 * - Simplified TMC9660 configuration
 * - Basic motor control operations
 * - Status monitoring and diagnostics
 * - Error handling and recovery
 * 
 * Usage:
 * @code
 * // Create TMC9660 controller
 * auto controller = std::make_unique<TMC9660Controller>();
 * 
 * // Initialize and configure
 * controller->Initialize(config);
 * 
 * // Basic operations
 * controller->EnableDriver();
 * controller->SetDirection(true);
 * controller->SetSpeed(1000);
 * 
 * // Status monitoring
 * auto status = controller->GetStatus();
 * @endcode
 */
#include "TMC9660Controller.h"

//==============================================================================
// PLATFORM MAPPING INTEGRATION
//==============================================================================

/**
 * @brief Platform mapping system for hardware resource management.
 * 
 * Provides the core platform mapping system that connects functional
 * identifiers to actual hardware resources for the ESP32-C6 + TMC9660 + PCAL95555
 * platform.
 * 
 * Key Features:
 * - Hardware chip identification and mapping
 * - Functional pin and ADC channel abstraction
 * - Hardware resource descriptors and validation
 * - Platform-specific configuration and limits
 * - Automatic resource discovery and conflict detection
 * 
 * Architecture:
 * - HardwareChip enumeration for chip identification
 * - FunctionalGpioPin and FunctionalAdcChannel for functional abstraction
 * - GpioHardwareResource and AdcHardwareResource for hardware mapping
 * - Platform-specific mapping tables and validation
 * - Integration with GPIO and ADC managers
 * 
 * Usage:
 * @code
 * // Check if functional pin is available
 * bool available = HardFOC::GpioPlatformMapping::isPinAvailable(
 *     HardFOC::FunctionalGpioPin::MOTOR_ENABLE
 * );
 * 
 * // Get hardware resource information
 * const auto* resource = HardFOC::GpioPlatformMapping::getHardwareResource(
 *     HardFOC::FunctionalGpioPin::MOTOR_ENABLE
 * );
 * 
 * if (resource) {
 *     // Access hardware information
 *     auto chip_id = resource->chip_id;
 *     auto pin_id = resource->pin_id;
 *     auto max_current = resource->max_current_ma;
 * }
 * @endcode
 */
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"

//==============================================================================
// BASE INTERFACES AND UTILITIES
//==============================================================================

/**
 * @brief Base GPIO interface for hardware abstraction.
 * 
 * Provides the core BaseGpio interface that all GPIO implementations
 * must implement for compatibility with the GPIO management system.
 * 
 * Key Features:
 * - Standardized GPIO interface
 * - Direction control (input/output)
 * - State control (active/inactive)
 * - Pull-up/pull-down configuration
 * - Output mode configuration
 * - Interrupt support
 * - Statistics and diagnostics
 * - Error handling
 * 
 * Usage:
 * @code
 * // Use BaseGpio interface for any GPIO implementation
 * std::unique_ptr<BaseGpio> gpio = CreateSomeGpioPin();
 * 
 * // Standard operations
 * gpio->SetActive();
 * bool state = gpio->IsActive();
 * gpio->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
 * 
 * // Configuration
 * gpio->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
 * gpio->SetOutputMode(hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
 * 
 * // Interrupts
 * if (gpio->SupportsInterrupts()) {
 *     gpio->ConfigureInterrupt(trigger, callback, user_data);
 *     gpio->EnableInterrupt();
 * }
 * 
 * // Statistics
 * auto stats = gpio->GetStatistics();
 * gpio->ResetStatistics();
 * @endcode
 */
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseGpio.h"

/**
 * @brief Base I2C interface for hardware abstraction.
 * 
 * Provides the core BaseI2c interface for I2C communication abstraction
 * used by PCAL95555 and other I2C devices.
 * 
 * Key Features:
 * - Standardized I2C interface
 * - Read/write operations
 * - Transaction support
 * - Error handling
 * - Device addressing
 * - Clock frequency configuration
 * 
 * Usage:
 * @code
 * // Use BaseI2c interface for I2C communication
 * std::unique_ptr<BaseI2c> i2c = CreateSomeI2cBus();
 * 
 * // Basic operations
 * uint8_t data[4];
 * auto result = i2c->Read(device_address, data, sizeof(data));
 * 
 * uint8_t write_data[] = {0x01, 0x02, 0x03};
 * auto write_result = i2c->Write(device_address, write_data, sizeof(write_data));
 * 
 * // Transactions
 * std::vector<uint8_t> write_buf = {0x01, 0x02};
 * std::vector<uint8_t> read_buf(4);
 * auto trans_result = i2c->Transaction(device_address, write_buf, read_buf);
 * @endcode
 */
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseI2c.h"

/**
 * @brief Thread-safe I2C bus wrapper.
 * 
 * Provides a thread-safe wrapper around I2C communication with
 * comprehensive error handling and diagnostics.
 * 
 * Key Features:
 * - Thread-safe I2C operations
 * - Comprehensive error handling
 * - Performance monitoring and statistics
 * - Automatic retry and recovery
 * - Device health monitoring
 * - Integration with BaseI2c interface
 * 
 * Usage:
 * @code
 * // Create thread-safe I2C bus
 * auto i2c_bus = std::make_unique<SfI2cBus>(base_i2c_implementation);
 * 
 * // Thread-safe operations
 * auto result = i2c_bus->Read(device_address, data, size);
 * auto write_result = i2c_bus->Write(device_address, data, size);
 * 
 * // Health monitoring
 * auto health = i2c_bus->GetHealth();
 * auto stats = i2c_bus->GetStatistics();
 * @endcode
 */
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/thread_safe/SfI2cBus.h"

//==============================================================================
// CONVENIENCE FUNCTIONS AND GLOBAL ACCESS
//==============================================================================

/**
 * @brief Get the global GPIO manager instance.
 * 
 * Convenience function to access the GPIO manager singleton.
 * 
 * @return Reference to the GPIO manager instance
 */
[[nodiscard]] inline GpioManager& GetGpioManager() noexcept {
    return GpioManager::GetInstance();
}

/**
 * @brief Get the global ADC manager instance.
 * 
 * Convenience function to access the ADC manager singleton.
 * 
 * @return Reference to the ADC manager instance
 */
[[nodiscard]] inline AdcManager& GetAdcManager() noexcept {
    return AdcManager::GetInstance();
}

/**
 * @brief Get the global TMC9660 motor controller instance.
 * 
 * Convenience function to access the TMC9660 motor controller singleton.
 * 
 * @return Reference to the TMC9660 motor controller instance
 */
[[nodiscard]] inline Tmc9660MotorController& GetTmc9660Controller() noexcept {
    return Tmc9660MotorController::GetInstance();
}

//==============================================================================
// SYSTEM INITIALIZATION HELPERS
//==============================================================================

/**
 * @brief Initialize the complete HardFOC system.
 * 
 * This function initializes all major components of the HardFOC system
 * including GPIO management, ADC management, and TMC9660 motor controller.
 * 
 * @param i2c_bus Reference to the I2C bus for PCAL95555 communication
 * @return Result indicating success or specific error
 */
[[nodiscard]] inline Result<void> InitializeHardFocSystem(SfI2cBus& i2c_bus) noexcept {
    // Initialize GPIO manager
    auto& gpio_manager = GetGpioManager();
    auto& tmc9660_controller = GetTmc9660Controller();
    
    auto gpio_result = gpio_manager.Initialize(i2c_bus, tmc9660_controller);
    if (gpio_result.IsError()) {
        return gpio_result;
    }
    
    // Initialize ADC manager
    auto& adc_manager = GetAdcManager();
    auto adc_result = adc_manager.Initialize();
    if (adc_result.IsError()) {
        return adc_result;
    }
    
    // Initialize TMC9660 controller
    auto tmc_result = tmc9660_controller.Initialize();
    if (tmc_result.IsError()) {
        return tmc_result;
    }
    
    return HARDFOC_SUCCESS();
}

/**
 * @brief Shutdown the complete HardFOC system.
 * 
 * This function shuts down all major components of the HardFOC system
 * in the proper order to ensure clean termination.
 * 
 * @return Result indicating success or specific error
 */
[[nodiscard]] inline Result<void> ShutdownHardFocSystem() noexcept {
    // Shutdown in reverse order of initialization
    
    // Shutdown TMC9660 controller
    auto& tmc9660_controller = GetTmc9660Controller();
    auto tmc_result = tmc9660_controller.Shutdown();
    
    // Shutdown ADC manager
    auto& adc_manager = GetAdcManager();
    auto adc_result = adc_manager.Shutdown();
    
    // Shutdown GPIO manager
    auto& gpio_manager = GetGpioManager();
    auto gpio_result = gpio_manager.Shutdown();
    
    // Return first error encountered, or success
    if (tmc_result.IsError()) return tmc_result;
    if (adc_result.IsError()) return adc_result;
    if (gpio_result.IsError()) return gpio_result;
    
    return HARDFOC_SUCCESS();
}

/**
 * @brief Get system health information.
 * 
 * This function provides comprehensive health information for all
 * major components of the HardFOC system.
 * 
 * @return Result containing system health information
 */
[[nodiscard]] inline Result<std::string> GetHardFocSystemHealth() noexcept {
    std::stringstream health;
    health << "HardFOC System Health Report\n";
    health << "============================\n\n";
    
    // GPIO Manager Health
    auto& gpio_manager = GetGpioManager();
    if (gpio_manager.IsInitialized()) {
        auto gpio_health = gpio_manager.GetSystemHealth();
        if (gpio_health.IsSuccess()) {
            health << "GPIO Manager:\n" << gpio_health.GetValue() << "\n";
        } else {
            health << "GPIO Manager: Error getting health - " 
                   << gpio_health.GetDescription() << "\n";
        }
    } else {
        health << "GPIO Manager: Not initialized\n";
    }
    
    // ADC Manager Health
    auto& adc_manager = GetAdcManager();
    if (adc_manager.IsInitialized()) {
        auto adc_health = adc_manager.GetSystemHealth();
        if (adc_health.IsSuccess()) {
            health << "ADC Manager:\n" << adc_health.GetValue() << "\n";
        } else {
            health << "ADC Manager: Error getting health - " 
                   << adc_health.GetDescription() << "\n";
        }
    } else {
        health << "ADC Manager: Not initialized\n";
    }
    
    // TMC9660 Controller Health
    auto& tmc9660_controller = GetTmc9660Controller();
    if (tmc9660_controller.IsInitialized()) {
        health << "TMC9660 Controller:\n";
        tmc9660_controller.PrintSystemStatus();
        health << "\n";
    } else {
        health << "TMC9660 Controller: Not initialized\n";
    }
    
    return Result<std::string>(health.str());
}

//==============================================================================
// VERSION INFORMATION
//==============================================================================

/**
 * @brief HardFOC Component Handler version information.
 */
namespace HardFocVersion {
    static constexpr uint8_t MAJOR = 2;
    static constexpr uint8_t MINOR = 0;
    static constexpr uint8_t PATCH = 0;
    static constexpr const char* BUILD_DATE = __DATE__;
    static constexpr const char* BUILD_TIME = __TIME__;
    
    /**
     * @brief Get version string.
     * @return Version string in format "major.minor.patch"
     */
    [[nodiscard]] inline std::string GetVersionString() noexcept {
        return std::to_string(MAJOR) + "." + std::to_string(MINOR) + "." + std::to_string(PATCH);
    }
    
    /**
     * @brief Get full version information.
     * @return Full version information including build date and time
     */
    [[nodiscard]] inline std::string GetFullVersionInfo() noexcept {
        return "HardFOC Component Handler v" + GetVersionString() + 
               " (Built: " + BUILD_DATE + " " + BUILD_TIME + ")";
    }
}

#endif // COMPONENT_HANDLER_ALL_H_