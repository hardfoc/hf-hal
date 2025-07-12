#ifndef COMPONENT_HANDLER_HARDFOC_INTEGRATION_H_
#define COMPONENT_HANDLER_HARDFOC_INTEGRATION_H_

#include "SystemInit.h"
#include "../AdcManager.h"
#include "../GpioManager.h"

/**
 * @file HardFocIntegration.h
 * @brief Integration example showing how to use the new ADC and GPIO system.
 * 
 * This file demonstrates how to integrate the new comprehensive ADC and GPIO
 * management system into the HardFOC application.
 */

/**
 * @class HardFocIntegration
 * @brief Example integration class for the HardFOC system.
 */
class HardFocIntegration {
public:
    /**
     * @brief Initialize the HardFOC integration.
     * @return true if successful, false otherwise.
     */
    static bool Initialize() noexcept;
    
    /**
     * @brief Demo function showing ADC usage.
     */
    static void DemoAdcUsage() noexcept;
    
    /**
     * @brief Demo function showing GPIO usage.
     */
    static void DemoGpioUsage() noexcept;
    
    /**
     * @brief Demo function showing multi-channel ADC reading.
     */
    static void DemoMultiChannelAdc() noexcept;
    
    /**
     * @brief Demo function showing multi-pin GPIO operations.
     */
    static void DemoMultiPinGpio() noexcept;
    
    /**
     * @brief Run system diagnostics.
     */
    static void RunSystemDiagnostics() noexcept;
    
    /**
     * @brief Update system status LEDs based on system health.
     */
    static void UpdateStatusLeds() noexcept;
    
    /**
     * @brief Read motor control sensors.
     * @param current_a Reference to store phase A current.
     * @param current_b Reference to store phase B current.
     * @param current_c Reference to store phase C current.
     * @param bus_voltage Reference to store bus voltage.
     * @param temperature Reference to store motor temperature.
     * @return true if all readings successful, false otherwise.
     */
    static bool ReadMotorSensors(float& current_a, float& current_b, float& current_c,
                                float& bus_voltage, float& temperature) noexcept;
    
    /**
     * @brief Set motor control outputs.
     * @param enable Enable motor.
     * @param brake Apply brake.
     * @return true if successful, false otherwise.
     */
    static bool SetMotorControl(bool enable, bool brake) noexcept;
    
    /**
     * @brief Monitor system voltages.
     * @param voltage_3v3 Reference to store 3.3V rail voltage.
     * @param voltage_5v Reference to store 5V rail voltage.
     * @param voltage_12v Reference to store 12V rail voltage.
     * @return true if all readings successful, false otherwise.
     */
    static bool MonitorSystemVoltages(float& voltage_3v3, float& voltage_5v, float& voltage_12v) noexcept;
    
private:
    /**
     * @brief Check if system is initialized.
     */
    static bool IsSystemInitialized() noexcept;
    
    static bool initialized_;
};

/**
 * @brief Global function to initialize the HardFOC system for the main application.
 * Call this function in your main() or app_main() function.
 * @return true if initialization successful, false otherwise.
 */
bool InitializeHardFocForMain() noexcept;

/**
 * @brief Global function to run periodic system maintenance.
 * Call this function periodically in your main loop.
 */
void RunPeriodicSystemMaintenance() noexcept;

/**
 * @brief Global function to get a quick health status.
 * @return true if system is healthy, false if there are issues.
 */
bool GetQuickHealthStatus() noexcept;

#endif // COMPONENT_HANDLER_HARDFOC_INTEGRATION_H_
