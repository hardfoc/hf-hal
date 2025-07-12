#include "HardFocIntegration.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include <vector>

static const char* TAG = "HardFocIntegration";

// Static member definition
bool HardFocIntegration::initialized_ = false;

bool HardFocIntegration::Initialize() noexcept {
    if (initialized_) {
        console_info(TAG, "HardFOC integration already initialized");
        return true;
    }
    
    console_info(TAG, "Initializing HardFOC integration system");
    
    // Initialize the core system
    if (!InitializeHardFocSystem()) {
        console_error(TAG, "Failed to initialize HardFOC core system");
        return false;
    }
    
    // Run initial system test
    if (!RunHardFocSystemTest()) {
        console_warning(TAG, "System test failed during initialization");
        // Don't fail initialization, but log the warning
    }
    
    initialized_ = true;
    console_info(TAG, "HardFOC integration initialized successfully");
    return true;
}

void HardFocIntegration::DemoAdcUsage() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== ADC Usage Demo ===");
    
    AdcManager& adcManager = AdcManager::GetInstance();
    
    // Read motor current phase A (raw count)
    auto rawResult = adcManager.ReadRawValue(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A);
    if (rawResult.IsSuccess()) {
        console_info(TAG, "Motor Current Phase A: %lu counts", rawResult.GetValue());
    }
    
    // Read motor current phase A (voltage)
    auto readingResult = adcManager.ReadChannel(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A);
    if (readingResult.IsSuccess()) {
        const auto& reading = readingResult.GetValue();
        console_info(TAG, "Motor Current Phase A: %.3f V", reading.voltage);
    }
    
    // Read system 3.3V rail
    auto voltageResult = adcManager.ReadChannel(AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3);
    if (voltageResult.IsSuccess()) {
        const auto& reading = voltageResult.GetValue();
        console_info(TAG, "3.3V Rail: %lu counts, %.3f V", reading.rawValue, reading.voltage);
    }
    
    console_info(TAG, "ADC demo completed");
}

void HardFocIntegration::DemoGpioUsage() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== GPIO Usage Demo ===");
    
    GpioManager& gpioManager = GpioManager::GetInstance();
    
    // Set motor enable pin
    auto enableResult = gpioManager.SetActive(GpioPin::GPIO_MOTOR_ENABLE);
    if (enableResult.IsSuccess()) {
        console_info(TAG, "Motor enabled");
    }
    
    os_delay_msec(1000);
    
    // Toggle status LED
    auto toggleResult = gpioManager.Toggle(GpioPin::GPIO_LED_STATUS);
    if (toggleResult.IsSuccess()) {
        console_info(TAG, "Status LED toggled to %s", toggleResult.GetValue() ? "ON" : "OFF");
    }
    
    // Check if motor fault is active
    auto faultResult = gpioManager.IsActive(GpioPin::GPIO_MOTOR_FAULT);
    if (faultResult.IsSuccess() && faultResult.GetValue()) {
        console_warning(TAG, "Motor fault detected!");
    }
    
    // Set motor brake
    auto brakeResult = gpioManager.SetActive(GpioPin::GPIO_MOTOR_BRAKE);
    if (brakeResult.IsSuccess()) {
        console_info(TAG, "Motor brake applied");
    }
    
    os_delay_msec(500);
    
    // Release brake and disable motor
    gpioManager.SetInactive(GpioPin::GPIO_MOTOR_BRAKE);
    gpioManager.SetInactive(GpioPin::GPIO_MOTOR_ENABLE);
    
    console_info(TAG, "GPIO demo completed");
}

void HardFocIntegration::DemoMultiChannelAdc() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== Multi-Channel ADC Demo ===");
    
    AdcManager& adcManager = AdcManager::GetInstance();
    
    // Create a vector of ADC sensors to read
    std::vector<AdcInputSensor> sensors = {
        AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A,
        AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_B,
        AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_C,
        AdcInputSensor::ADC_MOTOR_VOLTAGE_BUS,
        AdcInputSensor::ADC_MOTOR_TEMPERATURE
    };
    
    // Read all channels simultaneously
    auto batchResult = adcManager.BatchRead(sensors);
    if (batchResult.IsSuccess()) {
        const auto& batch = batchResult.GetValue();
        console_info(TAG, "Multi-channel ADC read successful:");
        
        for (size_t i = 0; i < batch.sensors.size(); ++i) {
            if (batch.results[i] == ResultCode::SUCCESS) {
                console_info(TAG, "  %s: %.3f V (%lu counts)",
                            AdcInputSensorToString(batch.sensors[i]).data(),
                            batch.readings[i].voltage,
                            batch.readings[i].rawValue);
            }
        }
    } else {
        console_error(TAG, "Multi-channel ADC read failed");
    }
    
    console_info(TAG, "Multi-channel ADC demo completed");
}

void HardFocIntegration::DemoMultiPinGpio() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== Multi-Pin GPIO Demo ===");
    
    GpioManager& gpioManager = GpioManager::GetInstance();
    
    // Set multiple LEDs active
    std::vector<GpioPin> leds = {
        GpioPin::GPIO_LED_STATUS,
        GpioPin::GPIO_LED_COMM
    };
    
    auto multiActiveResult = gpioManager.SetMultipleActive(leds);
    if (multiActiveResult.IsSuccess()) {
        console_info(TAG, "Multiple LEDs activated");
    }
    
    os_delay_msec(1000);
    
    // Pattern control using individual operations
    std::vector<GpioPin> ledPins = {
        GpioPin::GPIO_LED_STATUS,
        GpioPin::GPIO_LED_ERROR,
        GpioPin::GPIO_LED_COMM
    };
    
    // Set pattern: status and comm on, error off
    gpioManager.SetActive(GpioPin::GPIO_LED_STATUS);
    gpioManager.SetInactive(GpioPin::GPIO_LED_ERROR);
    gpioManager.SetActive(GpioPin::GPIO_LED_COMM);
    console_info(TAG, "LED pattern set: Status ON, Error OFF, Comm ON");
    
    os_delay_msec(1000);
    
    // Read pin states to verify pattern
    auto batchReadResult = gpioManager.BatchRead(ledPins);
    if (batchReadResult.IsSuccess()) {
        const auto& batch = batchReadResult.GetValue();
        console_info(TAG, "Current LED states:");
        for (size_t i = 0; i < batch.pins.size(); ++i) {
            console_info(TAG, "  %s: %s", 
                        GpioPinToString(batch.pins[i]).data(),
                        batch.states[i] ? "ON" : "OFF");
        }
    }
    
    // Turn off all LEDs
    gpioManager.SetMultipleInactive(ledPins);
    
    console_info(TAG, "Multi-pin GPIO demo completed");
}

void HardFocIntegration::RunSystemDiagnostics() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== System Diagnostics ===");
    
    // Print system status
    SystemInit::PrintSystemStatus();
    
    // Check GPIO system health
    GpioManager& gpioManager = GpioManager::GetInstance();
    auto gpioHealthResult = gpioManager.GetSystemHealth();
    if (gpioHealthResult.IsSuccess()) {
        console_info(TAG, "GPIO system health: HEALTHY");
    } else {
        console_error(TAG, "GPIO system health: DEGRADED");
    }
    
    // Check ADC system health
    AdcManager& adcManager = AdcManager::GetInstance();
    auto adcHealthResult = adcManager.GetSystemHealth();
    if (adcHealthResult.IsSuccess()) {
        console_info(TAG, "ADC system health: HEALTHY");
    } else {
        console_error(TAG, "ADC system health: DEGRADED");
    }
    
    // Check individual ADC channels responsiveness
    for (int i = 0; i < static_cast<int>(AdcInputSensor::ADC_INPUT_COUNT); ++i) {
        AdcInputSensor sensor = static_cast<AdcInputSensor>(i);
        if (adcManager.IsChannelRegistered(sensor)) {
            auto readResult = adcManager.ReadChannel(sensor);
            if (readResult.IsSuccess()) {
                console_info(TAG, "ADC %s: RESPONDING", 
                            AdcInputSensorToString(sensor).data());
            }
        }
    }
    
    console_info(TAG, "System diagnostics completed");
}
    
    console_info(TAG, "System diagnostics completed");
}

void HardFocIntegration::UpdateStatusLeds() noexcept {
    if (!IsSystemInitialized()) return;
    
    GpioManager& gpioManager = GpioManager::GetInstance();
    
    // Get system health
    bool systemHealthy = GetHardFocSystemStatus();
    
    auto gpioHealthResult = gpioManager.GetSystemHealth();
    bool gpioHealthy = gpioHealthResult.IsSuccess();
    
    AdcManager& adcManager = AdcManager::GetInstance();
    auto adcHealthResult = adcManager.GetSystemHealth();
    bool adcHealthy = adcHealthResult.IsSuccess();
    
    // Update status LEDs
    bool statusLed = systemHealthy && gpioHealthy && adcHealthy;
    bool errorLed = !systemHealthy || !gpioHealthy || !adcHealthy;
    bool commLed = true; // Assume communication is working if we can toggle LEDs
    
    // Set LED states individually
    if (statusLed) {
        gpioManager.SetActive(GpioPin::GPIO_LED_STATUS);
    } else {
        gpioManager.SetInactive(GpioPin::GPIO_LED_STATUS);
    }
    
    if (errorLed) {
        gpioManager.SetActive(GpioPin::GPIO_LED_ERROR);
    } else {
        gpioManager.SetInactive(GpioPin::GPIO_LED_ERROR);
    }
    
    if (commLed) {
        gpioManager.SetActive(GpioPin::GPIO_LED_COMM);
    } else {
        gpioManager.SetInactive(GpioPin::GPIO_LED_COMM);
    }
}

bool HardFocIntegration::ReadMotorSensors(float& current_a, float& current_b, float& current_c,
                                          float& bus_voltage, float& temperature) noexcept {
    if (!IsSystemInitialized()) return false;
    
    AdcManager& adcManager = AdcManager::GetInstance();
    
    bool success = true;
    
    auto currentAResult = adcManager.ReadChannel(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A);
    if (currentAResult.IsSuccess()) {
        current_a = currentAResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    auto currentBResult = adcManager.ReadChannel(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_B);
    if (currentBResult.IsSuccess()) {
        current_b = currentBResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    auto currentCResult = adcManager.ReadChannel(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_C);
    if (currentCResult.IsSuccess()) {
        current_c = currentCResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    auto busVoltageResult = adcManager.ReadChannel(AdcInputSensor::ADC_MOTOR_VOLTAGE_BUS);
    if (busVoltageResult.IsSuccess()) {
        bus_voltage = busVoltageResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    auto temperatureResult = adcManager.ReadChannel(AdcInputSensor::ADC_MOTOR_TEMPERATURE);
    if (temperatureResult.IsSuccess()) {
        temperature = temperatureResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    return success;
}

bool HardFocIntegration::SetMotorControl(bool enable, bool brake) noexcept {
    if (!IsSystemInitialized()) return false;
    
    GpioManager& gpioManager = GpioManager::GetInstance();
    
    bool success = true;
    
    auto enableResult = enable ? 
        gpioManager.SetActive(GpioPin::GPIO_MOTOR_ENABLE) :
        gpioManager.SetInactive(GpioPin::GPIO_MOTOR_ENABLE);
    
    if (!enableResult.IsSuccess()) {
        success = false;
    }
    
    auto brakeResult = brake ?
        gpioManager.SetActive(GpioPin::GPIO_MOTOR_BRAKE) :
        gpioManager.SetInactive(GpioPin::GPIO_MOTOR_BRAKE);
    
    if (!brakeResult.IsSuccess()) {
        success = false;
    }
    
    return success;
}

bool HardFocIntegration::MonitorSystemVoltages(float& voltage_3v3, float& voltage_5v, float& voltage_12v) noexcept {
    if (!IsSystemInitialized()) return false;
    
    AdcManager& adcManager = AdcManager::GetInstance();
    
    bool success = true;
    
    auto voltage3v3Result = adcManager.ReadChannel(AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3);
    if (voltage3v3Result.IsSuccess()) {
        voltage_3v3 = voltage3v3Result.GetValue().voltage;
    } else {
        success = false;
    }
    
    auto voltage5vResult = adcManager.ReadChannel(AdcInputSensor::ADC_SYSTEM_VOLTAGE_5V);
    if (voltage5vResult.IsSuccess()) {
        voltage_5v = voltage5vResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    auto voltage12vResult = adcManager.ReadChannel(AdcInputSensor::ADC_SYSTEM_VOLTAGE_12V);
    if (voltage12vResult.IsSuccess()) {
        voltage_12v = voltage12vResult.GetValue().voltage;
    } else {
        success = false;
    }
    
    return success;
}

bool HardFocIntegration::IsSystemInitialized() noexcept {
    if (!initialized_) {
        console_error(TAG, "HardFOC integration not initialized");
        return false;
    }
    return true;
}

//=============================================================================
// Global Functions
//=============================================================================

bool InitializeHardFocForMain() noexcept {
    return HardFocIntegration::Initialize();
}

void RunPeriodicSystemMaintenance() noexcept {
    // Update status LEDs
    HardFocIntegration::UpdateStatusLeds();
    
    // Log system health periodically (every 100 calls, assuming 100ms intervals = 10 seconds)
    static uint32_t maintenanceCounter = 0;
    maintenanceCounter++;
    
    if (maintenanceCounter % 100 == 0) {
        console_info(TAG, "Periodic maintenance: System health %s", 
                    GetQuickHealthStatus() ? "OK" : "DEGRADED");
        
        // Print detailed status every 1000 calls (100 seconds)
        if (maintenanceCounter % 1000 == 0) {
            HardFocIntegration::RunSystemDiagnostics();
        }
    }
}

bool GetQuickHealthStatus() noexcept {
    return GetHardFocSystemStatus();
}
