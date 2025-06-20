#include "TMC9660Controller.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include <algorithm>

/**
 * @file TMC9660Controller.cpp
 * @brief Implementation of TMC9660 motor controller with integrated ADC.
 */

static const char* TAG = "TMC9660Controller";

//=============================================================================
// TMC9660Adc Implementation
//=============================================================================

TMC9660Adc::TMC9660Adc(TMC9660Controller& controller) noexcept
    : BaseAdc(), controller_(controller) {
}

bool TMC9660Adc::Initialize() noexcept {
    if (initialized) {
        return true;
    }

    // Initialize through parent controller
    initialized = controller_.Initialize();
    
    if (initialized) {
        console_info(TAG, "TMC9660 ADC initialized successfully");
    } else {
        console_error(TAG, "Failed to initialize TMC9660 ADC");
    }

    return initialized;
}

BaseAdc::AdcErr TMC9660Adc::ReadChannelV(uint8_t channel_num, float &channel_reading_v,
                                          uint8_t numOfSamplesToAvg,
                                          uint32_t timeBetweenSamples) noexcept {
    if (!EnsureInitialized()) {
        return AdcErr::ADC_ERR_CHANNEL_NOT_ENABLED;
    }

    if (channel_num > 2) {
        return AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }

    uint32_t totalSum = 0;
    uint8_t successfulReads = 0;

    for (uint8_t i = 0; i < numOfSamplesToAvg; ++i) {
        uint32_t rawValue;
        if (controller_.ReadInternalAdc(channel_num, rawValue)) {
            totalSum += rawValue;
            successfulReads++;
        }

        if (i < numOfSamplesToAvg - 1 && timeBetweenSamples > 0) {
            os_delay_msec(timeBetweenSamples);
        }
    }

    if (successfulReads == 0) {
        return AdcErr::ADC_ERR_CHANNEL_READ_ERR;
    }

    uint32_t averageValue = totalSum / successfulReads;
    channel_reading_v = controller_.ConvertRawToCurrent(averageValue, channel_num);

    return AdcErr::ADC_SUCCESS;
}

BaseAdc::AdcErr TMC9660Adc::ReadChannelCount(uint8_t channel_num, uint32_t &channel_reading_count,
                                              uint8_t numOfSamplesToAvg,
                                              uint32_t timeBetweenSamples) noexcept {
    if (!EnsureInitialized()) {
        return AdcErr::ADC_ERR_CHANNEL_NOT_ENABLED;
    }

    if (channel_num > 2) {
        return AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    }

    uint32_t totalSum = 0;
    uint8_t successfulReads = 0;

    for (uint8_t i = 0; i < numOfSamplesToAvg; ++i) {
        uint32_t rawValue;
        if (controller_.ReadInternalAdc(channel_num, rawValue)) {
            totalSum += rawValue;
            successfulReads++;
        }

        if (i < numOfSamplesToAvg - 1 && timeBetweenSamples > 0) {
            os_delay_msec(timeBetweenSamples);
        }
    }

    if (successfulReads == 0) {
        return AdcErr::ADC_ERR_CHANNEL_READ_ERR;
    }

    channel_reading_count = totalSum / successfulReads;
    return AdcErr::ADC_SUCCESS;
}

BaseAdc::AdcErr TMC9660Adc::ReadChannel(uint8_t channel_num, uint32_t &channel_reading_count,
                                         float &channel_reading_v, uint8_t numOfSamplesToAvg,
                                         uint32_t timeBetweenSamples) noexcept {
    // Read the count first
    AdcErr result = ReadChannelCount(channel_num, channel_reading_count, numOfSamplesToAvg, timeBetweenSamples);
    if (result != AdcErr::ADC_SUCCESS) {
        return result;
    }

    // Convert to voltage/current
    channel_reading_v = controller_.ConvertRawToCurrent(channel_reading_count, channel_num);
    return AdcErr::ADC_SUCCESS;
}

//=============================================================================
// TMC9660Controller Implementation
//=============================================================================

TMC9660Controller::TMC9660Controller() noexcept
    : adc_(*this), initialized_(false), currentMode_(ControlMode::DISABLED) {
    // Initialize status structure
    lastStatus_ = {};
}

bool TMC9660Controller::Initialize() noexcept {
    if (initialized_) {
        return true;
    }

    console_info(TAG, "Initializing TMC9660 motor controller");

    // TODO: Initialize SPI communication with TMC9660
    // TODO: Configure TMC9660 registers for motor control
    // TODO: Set up current sensing ADC channels
    // TODO: Configure motor parameters

    // For now, simulate successful initialization
    initialized_ = true;
    currentMode_ = ControlMode::DISABLED;

    console_info(TAG, "TMC9660 motor controller initialized successfully");
    return true;
}

bool TMC9660Controller::EnableMotor() noexcept {
    if (!initialized_) {
        console_error(TAG, "TMC9660 not initialized");
        return false;
    }

    console_info(TAG, "Enabling motor");
    
    // TODO: Send enable command to TMC9660
    lastStatus_.isEnabled = true;
    
    return true;
}

bool TMC9660Controller::DisableMotor() noexcept {
    if (!initialized_) {
        console_error(TAG, "TMC9660 not initialized");
        return false;
    }

    console_info(TAG, "Disabling motor");
    
    // TODO: Send disable command to TMC9660
    lastStatus_.isEnabled = false;
    
    return true;
}

bool TMC9660Controller::SetControlMode(ControlMode mode) noexcept {
    if (!initialized_) {
        console_error(TAG, "TMC9660 not initialized");
        return false;
    }

    console_info(TAG, "Setting control mode: %d", static_cast<int>(mode));
    
    // TODO: Configure TMC9660 for specified control mode
    currentMode_ = mode;
    
    return true;
}

bool TMC9660Controller::SetTargetPosition(int32_t position) noexcept {
    if (!initialized_ || currentMode_ != ControlMode::POSITION_CONTROL) {
        return false;
    }

    console_info(TAG, "Setting target position: %ld", position);
    
    // TODO: Send position command to TMC9660
    
    return true;
}

bool TMC9660Controller::SetTargetVelocity(int32_t velocity) noexcept {
    if (!initialized_ || currentMode_ != ControlMode::VELOCITY_CONTROL) {
        return false;
    }

    console_info(TAG, "Setting target velocity: %ld", velocity);
    
    // TODO: Send velocity command to TMC9660
    
    return true;
}

bool TMC9660Controller::SetTargetTorque(float torque) noexcept {
    if (!initialized_ || currentMode_ != ControlMode::TORQUE_CONTROL) {
        return false;
    }

    console_info(TAG, "Setting target torque: %.3f", torque);
    
    // TODO: Send torque command to TMC9660
    
    return true;
}

TMC9660Controller::Status TMC9660Controller::GetStatus() noexcept {
    if (!initialized_) {
        return lastStatus_;
    }

    // TODO: Read actual status from TMC9660
    lastStatus_.isInitialized = initialized_;
    
    // Read current values
    ReadPhaseCurrents(lastStatus_.currentA, lastStatus_.currentB, lastStatus_.currentC);
    
    return lastStatus_;
}

bool TMC9660Controller::ReadPhaseCurrents(float& currentA, float& currentB, float& currentC) noexcept {
    if (!initialized_) {
        return false;
    }

    // Read all three current sensing channels
    uint32_t rawA, rawB, rawC;
    
    bool success = true;
    success &= ReadInternalAdc(0, rawA);
    success &= ReadInternalAdc(1, rawB);
    success &= ReadInternalAdc(2, rawC);
    
    if (success) {
        currentA = ConvertRawToCurrent(rawA, 0);
        currentB = ConvertRawToCurrent(rawB, 1);
        currentC = ConvertRawToCurrent(rawC, 2);
    }
    
    return success;
}

bool TMC9660Controller::IsResponding() noexcept {
    if (!initialized_) {
        return false;
    }

    // TODO: Send ping command to TMC9660 and check response
    return true;
}

bool TMC9660Controller::RunDiagnostics() noexcept {
    if (!initialized_) {
        console_error(TAG, "TMC9660 not initialized for diagnostics");
        return false;
    }

    console_info(TAG, "Running TMC9660 diagnostics");
    
    bool allTestsPassed = true;
    
    // Test SPI communication
    if (!IsResponding()) {
        console_error(TAG, "SPI communication test failed");
        allTestsPassed = false;
    }
    
    // Test ADC channels
    for (uint8_t ch = 0; ch < 3; ++ch) {
        uint32_t rawValue;
        if (!ReadInternalAdc(ch, rawValue)) {
            console_error(TAG, "ADC channel %d test failed", ch);
            allTestsPassed = false;
        }
    }
    
    // Check for motor faults
    // TODO: Read fault status from TMC9660
    
    if (allTestsPassed) {
        console_info(TAG, "TMC9660 diagnostics passed");
    } else {
        console_error(TAG, "TMC9660 diagnostics failed");
    }
    
    return allTestsPassed;
}

bool TMC9660Controller::ReadInternalAdc(uint8_t channel, uint32_t& rawValue) noexcept {
    if (channel > 2) {
        return false;
    }

    // TODO: Implement actual TMC9660 ADC reading via SPI
    // For now, return simulated values
    rawValue = 2048 + (channel * 100); // Simulated 12-bit ADC value
    
    return true;
}

float TMC9660Controller::ConvertRawToCurrent(uint32_t rawValue, uint8_t channel) noexcept {
    // Convert raw ADC value to current in Amps
    // This is a simplified conversion - actual implementation would use
    // proper calibration coefficients for each channel
    
    float voltage = (rawValue / 4095.0f) * 3.3f; // Assuming 12-bit ADC, 3.3V reference
    float current = (voltage - CURRENT_OFFSET) / CURRENT_SCALE_FACTOR;
    
    return current;
}

bool TMC9660Controller::SendSpiCommand(uint8_t command, uint32_t data) noexcept {
    // TODO: Implement actual SPI communication with TMC9660
    return true;
}

bool TMC9660Controller::ReadSpiData(uint8_t command, uint32_t& data) noexcept {
    // TODO: Implement actual SPI communication with TMC9660
    data = 0;
    return true;
}
