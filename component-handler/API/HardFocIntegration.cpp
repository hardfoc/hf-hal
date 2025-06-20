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
    
    AdcData& adcData = AdcData::GetInstance();
    
    // Read single ADC channels
    uint32_t count;
    float voltage;
    
    // Read motor current phase A
    if (adcData.GetCount(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, count, 10, 1)) {
        console_info(TAG, "Motor Current Phase A: %lu counts", count);
    }
    
    if (adcData.GetVolt(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, voltage, 10, 1)) {
        console_info(TAG, "Motor Current Phase A: %.3f V", voltage);
    }
    
    // Read system 3.3V rail
    if (adcData.GetCountAndVolt(AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3, count, voltage, 5, 2)) {
        console_info(TAG, "3.3V Rail: %lu counts, %.3f V", count, voltage);
    }
    
    console_info(TAG, "ADC demo completed");
}

void HardFocIntegration::DemoGpioUsage() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== GPIO Usage Demo ===");
    
    GpioData& gpioData = GpioData::GetInstance();
    
    // Set motor enable pin
    if (gpioData.SetActive(GpioPin::GPIO_MOTOR_ENABLE)) {
        console_info(TAG, "Motor enabled");
    }
    
    os_delay_msec(1000);
    
    // Toggle status LED
    if (gpioData.Toggle(GpioPin::GPIO_LED_STATUS)) {
        console_info(TAG, "Status LED toggled");
    }
    
    // Check if motor fault is active
    if (gpioData.IsActive(GpioPin::GPIO_MOTOR_FAULT)) {
        console_warning(TAG, "Motor fault detected!");
    }
    
    // Set motor brake
    if (gpioData.SetActive(GpioPin::GPIO_MOTOR_BRAKE)) {
        console_info(TAG, "Motor brake applied");
    }
    
    os_delay_msec(500);
    
    // Release brake and disable motor
    gpioData.SetInactive(GpioPin::GPIO_MOTOR_BRAKE);
    gpioData.SetInactive(GpioPin::GPIO_MOTOR_ENABLE);
    
    console_info(TAG, "GPIO demo completed");
}

void HardFocIntegration::DemoMultiChannelAdc() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== Multi-Channel ADC Demo ===");
    
    AdcData& adcData = AdcData::GetInstance();
    
    // Create a vector of ADC read specifications
    std::vector<AdcInputSensorReadSpec> readSpecs = {
        AdcInputSensorReadSpec(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, 10),
        AdcInputSensorReadSpec(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_B, 10),
        AdcInputSensorReadSpec(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_C, 10),
        AdcInputSensorReadSpec(AdcInputSensor::ADC_MOTOR_VOLTAGE_BUS, 5),
        AdcInputSensorReadSpec(AdcInputSensor::ADC_MOTOR_TEMPERATURE, 3)
    };
    
    // Read all channels simultaneously
    if (adcData.GetMultiVolt(readSpecs, 3, 5, TimeUnit::TIME_UNIT_MS)) {
        console_info(TAG, "Multi-channel ADC read successful:");
        for (const auto& spec : readSpecs) {
            console_info(TAG, "  %s: %.3f V (%d successful readings)",
                        AdcInputSensorToString(spec.sensor).data(),
                        spec.channelAvgReadingVoltageStorage,
                        spec.numberOfSuccessfulReadings);
        }
    } else {
        console_error(TAG, "Multi-channel ADC read failed");
    }
    
    console_info(TAG, "Multi-channel ADC demo completed");
}

void HardFocIntegration::DemoMultiPinGpio() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== Multi-Pin GPIO Demo ===");
    
    GpioData& gpioData = GpioData::GetInstance();
    
    // Set multiple LEDs active
    std::vector<GpioPin> leds = {
        GpioPin::GPIO_LED_STATUS,
        GpioPin::GPIO_LED_COMM
    };
    
    if (gpioData.SetMultipleActive(leds)) {
        console_info(TAG, "Multiple LEDs activated");
    }
    
    os_delay_msec(1000);
    
    // Set LED pattern using bitmask
    std::vector<GpioPin> ledPins = {
        GpioPin::GPIO_LED_STATUS,
        GpioPin::GPIO_LED_ERROR,
        GpioPin::GPIO_LED_COMM
    };
    
    // Pattern: 101 (status and comm on, error off)
    uint32_t pattern = 0b101;
    if (gpioData.SetPinPattern(ledPins, pattern)) {
        console_info(TAG, "LED pattern set: 0x%02lX", pattern);
    }
    
    os_delay_msec(1000);
    
    // Read pin pattern
    uint32_t readPattern;
    if (gpioData.GetPinPattern(ledPins, readPattern)) {
        console_info(TAG, "Current LED pattern: 0x%02lX", readPattern);
    }
    
    // Turn off all LEDs
    gpioData.SetMultipleInactive(ledPins);
    
    console_info(TAG, "Multi-pin GPIO demo completed");
}

void HardFocIntegration::RunSystemDiagnostics() noexcept {
    if (!IsSystemInitialized()) return;
    
    console_info(TAG, "=== System Diagnostics ===");
    
    // Print system status
    SystemInit::PrintSystemStatus();
    
    // Run GPIO test
    GpioData& gpioData = GpioData::GetInstance();
    if (gpioData.RunGpioTest()) {
        console_info(TAG, "GPIO diagnostics: PASS");
    } else {
        console_error(TAG, "GPIO diagnostics: FAIL");
    }
    
    // Check ADC health
    AdcData& adcData = AdcData::GetInstance();
    console_info(TAG, "ADC system health: %s", 
                adcData.EnsureInitialized() ? "HEALTHY" : "DEGRADED");
    
    // Check individual ADC channels
    for (int i = 0; i < static_cast<int>(AdcInputSensor::ADC_INPUT_COUNT); ++i) {
        AdcInputSensor sensor = static_cast<AdcInputSensor>(i);
        if (adcData.IsResponding(sensor)) {
            console_info(TAG, "ADC %s: RESPONDING", 
                        AdcInputSensorToString(sensor).data());
        }
    }
    
    console_info(TAG, "System diagnostics completed");
}

void HardFocIntegration::UpdateStatusLeds() noexcept {
    if (!IsSystemInitialized()) return;
    
    GpioData& gpioData = GpioData::GetInstance();
    
    // Get system health
    bool systemHealthy = GetHardFocSystemStatus();
    bool gpioHealthy = gpioData.GetSystemHealth();
    
    AdcData& adcData = AdcData::GetInstance();
    bool adcHealthy = adcData.EnsureInitialized();
    
    // Update status LEDs
    bool statusLed = systemHealthy && gpioHealthy && adcHealthy;
    bool errorLed = !systemHealthy || !gpioHealthy || !adcHealthy;
    bool commLed = true; // Assume communication is working if we can toggle LEDs
    
    gpioData.SetLedStatus(
        GpioPin::GPIO_LED_STATUS,
        GpioPin::GPIO_LED_ERROR,
        GpioPin::GPIO_LED_COMM,
        statusLed,
        errorLed,
        commLed
    );
}

bool HardFocIntegration::ReadMotorSensors(float& current_a, float& current_b, float& current_c,
                                          float& bus_voltage, float& temperature) noexcept {
    if (!IsSystemInitialized()) return false;
    
    AdcData& adcData = AdcData::GetInstance();
    
    bool success = true;
    
    success &= adcData.GetVolt(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A, current_a, 10, 1);
    success &= adcData.GetVolt(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_B, current_b, 10, 1);
    success &= adcData.GetVolt(AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_C, current_c, 10, 1);
    success &= adcData.GetVolt(AdcInputSensor::ADC_MOTOR_VOLTAGE_BUS, bus_voltage, 5, 2);
    success &= adcData.GetVolt(AdcInputSensor::ADC_MOTOR_TEMPERATURE, temperature, 3, 5);
    
    return success;
}

bool HardFocIntegration::SetMotorControl(bool enable, bool brake) noexcept {
    if (!IsSystemInitialized()) return false;
    
    GpioData& gpioData = GpioData::GetInstance();
    
    bool success = true;
    
    if (enable) {
        success &= gpioData.SetActive(GpioPin::GPIO_MOTOR_ENABLE);
    } else {
        success &= gpioData.SetInactive(GpioPin::GPIO_MOTOR_ENABLE);
    }
    
    if (brake) {
        success &= gpioData.SetActive(GpioPin::GPIO_MOTOR_BRAKE);
    } else {
        success &= gpioData.SetInactive(GpioPin::GPIO_MOTOR_BRAKE);
    }
    
    return success;
}

bool HardFocIntegration::MonitorSystemVoltages(float& voltage_3v3, float& voltage_5v, float& voltage_12v) noexcept {
    if (!IsSystemInitialized()) return false;
    
    AdcData& adcData = AdcData::GetInstance();
    
    bool success = true;
    
    success &= adcData.GetVolt(AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3, voltage_3v3, 5, 2);
    success &= adcData.GetVolt(AdcInputSensor::ADC_SYSTEM_VOLTAGE_5V, voltage_5v, 5, 2);
    success &= adcData.GetVolt(AdcInputSensor::ADC_SYSTEM_VOLTAGE_12V, voltage_12v, 5, 2);
    
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
