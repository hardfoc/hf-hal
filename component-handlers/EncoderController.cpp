#include "EncoderController.h"
#include "CommChannelsManager.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include "utils-and-drivers/driver-handlers/Logger.h"
#include <algorithm>
#include <string>
#include <sstream>

EncoderController& EncoderController::GetInstance() {
    static EncoderController instance;
    return instance;
}

bool EncoderController::Initialize() noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    if (initialized_) {
        return true; // Already initialized
    }
    
    // Initialize CommChannelsManager first
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        Logger::GetInstance().Error("ENCODER_CONTROLLER", "Failed to initialize CommChannelsManager");
        return false;
    }
    
    // Automatically create onboard AS5047U device
    BaseSpi* spiInterface = commManager.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (spiInterface) {
        // Create onboard AS5047U handler with default config
        auto onboardHandler = std::make_unique<As5047uHandler>(
            *spiInterface, 
            As5047uHandler::GetDefaultConfig()
        );
        
        encoderHandlers_[ONBOARD_ENCODER_INDEX] = std::move(onboardHandler);
        deviceActive_[ONBOARD_ENCODER_INDEX] = true;
        deviceInitialized_[ONBOARD_ENCODER_INDEX] = false;
        activeDeviceCount_ = 1;
        
        Logger::GetInstance().Info("ENCODER_CONTROLLER", "Created onboard AS5047U handler");
    } else {
        Logger::GetInstance().Warning("ENCODER_CONTROLLER", "No SPI interface available for onboard AS5047U");
    }
    
    // Initialize all active devices
    bool allSuccess = true;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && encoderHandlers_[i]) {
            As5047uError result = encoderHandlers_[i]->Initialize();
            deviceInitialized_[i] = (result == As5047uError::SUCCESS);
            if (!deviceInitialized_[i]) {
                allSuccess = false;
                Logger::GetInstance().Error("ENCODER_CONTROLLER", 
                    "Failed to initialize encoder device " + std::to_string(i));
            }
        }
    }
    
    initialized_ = true;
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Initialization completed");
    return allSuccess;
}

As5047uHandler* EncoderController::handler(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    if (!IsValidDeviceIndex(deviceIndex) || !deviceActive_[deviceIndex] || !encoderHandlers_[deviceIndex]) {
        return nullptr; // Invalid or inactive device
    }
    
    return encoderHandlers_[deviceIndex].get();
}

std::shared_ptr<AS5047U> EncoderController::sensor(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    if (!IsValidDeviceIndex(deviceIndex) || !deviceActive_[deviceIndex] || !encoderHandlers_[deviceIndex]) {
        return nullptr; // Invalid or inactive device
    }
    
    if (!deviceInitialized_[deviceIndex]) {
        return nullptr; // Device not initialized
    }
    
    return encoderHandlers_[deviceIndex]->GetSensor();
}

bool EncoderController::CreateExternalDevice(uint8_t csDeviceIndex, 
                                           SpiDeviceId spiDeviceId, 
                                           const As5047uConfig& config) {
    MutexLockGuard lock(controller_mutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        Logger::GetInstance().Error("ENCODER_CONTROLLER", "Invalid device index for external device: " + std::to_string(csDeviceIndex));
        return false;
    }
    
    if (deviceActive_[csDeviceIndex]) {
        Logger::GetInstance().Warning("ENCODER_CONTROLLER", "Device slot already occupied: " + std::to_string(csDeviceIndex));
        return false;
    }
    
    // Get SPI interface from CommChannelsManager
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        Logger::GetInstance().Error("ENCODER_CONTROLLER", "CommChannelsManager not initialized");
        return false;
    }
    
    BaseSpi* spiInterface = commManager.GetSpiDevice(spiDeviceId);
    if (!spiInterface) {
        Logger::GetInstance().Error("ENCODER_CONTROLLER", "Invalid SPI device ID or interface not available");
        return false;
    }
    
    // Create external AS5047U handler
    auto externalHandler = std::make_unique<As5047uHandler>(*spiInterface, config);
    
    encoderHandlers_[csDeviceIndex] = std::move(externalHandler);
    deviceActive_[csDeviceIndex] = true;
    deviceInitialized_[csDeviceIndex] = false;
    activeDeviceCount_++;
    
    // If system is already initialized, initialize this device immediately
    if (initialized_) {
        As5047uError result = encoderHandlers_[csDeviceIndex]->Initialize();
        deviceInitialized_[csDeviceIndex] = (result == As5047uError::SUCCESS);
        if (!deviceInitialized_[csDeviceIndex]) {
            Logger::GetInstance().Error("ENCODER_CONTROLLER", 
                "Failed to initialize external encoder device " + std::to_string(csDeviceIndex));
        }
    }
    
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Created external encoder device at index " + std::to_string(csDeviceIndex));
    return true;
}

bool EncoderController::DeleteExternalDevice(uint8_t csDeviceIndex) {
    MutexLockGuard lock(controller_mutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        Logger::GetInstance().Error("ENCODER_CONTROLLER", "Cannot delete onboard device or invalid index: " + std::to_string(csDeviceIndex));
        return false;
    }
    
    if (!deviceActive_[csDeviceIndex]) {
        Logger::GetInstance().Warning("ENCODER_CONTROLLER", "Device doesn't exist at index: " + std::to_string(csDeviceIndex));
        return false;
    }
    
    // Reset device slot
    encoderHandlers_[csDeviceIndex].reset();
    deviceActive_[csDeviceIndex] = false;
    deviceInitialized_[csDeviceIndex] = false;
    activeDeviceCount_--;
    
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Deleted external encoder device at index " + std::to_string(csDeviceIndex));
    return true;
}

uint8_t EncoderController::GetDeviceCount() const noexcept {
    MutexLockGuard lock(controller_mutex_);
    return activeDeviceCount_;
}

std::vector<uint8_t> EncoderController::GetActiveDeviceIndices() const noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    std::vector<uint8_t> activeIndices;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i]) {
            activeIndices.push_back(i);
        }
    }
    return activeIndices;
}

bool EncoderController::IsDeviceActive(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(controller_mutex_);
    return IsValidDeviceIndex(deviceIndex) && deviceActive_[deviceIndex] && deviceInitialized_[deviceIndex];
}

As5047uError EncoderController::ReadAngle(uint8_t deviceIndex, uint16_t& angle) noexcept {
    As5047uHandler* handler_ptr = handler(deviceIndex);
    if (!handler_ptr) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    return handler_ptr->ReadAngle(angle);
}

As5047uError EncoderController::ReadAngleDegrees(uint8_t deviceIndex, double& angle_degrees) noexcept {
    uint16_t angle_lsb;
    As5047uError result = ReadAngle(deviceIndex, angle_lsb);
    if (result == As5047uError::SUCCESS) {
        angle_degrees = As5047uHandler::LSBToDegrees(angle_lsb);
    }
    return result;
}

As5047uError EncoderController::ReadVelocityRPM(uint8_t deviceIndex, double& velocity_rpm) noexcept {
    As5047uHandler* handler_ptr = handler(deviceIndex);
    if (!handler_ptr) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    return handler_ptr->ReadVelocityRPM(velocity_rpm);
}

As5047uError EncoderController::ReadDiagnostics(uint8_t deviceIndex, As5047uDiagnostics& diagnostics) noexcept {
    As5047uHandler* handler_ptr = handler(deviceIndex);
    if (!handler_ptr) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    return handler_ptr->ReadDiagnostics(diagnostics);
}

As5047uError EncoderController::SetZeroPosition(uint8_t deviceIndex, uint16_t zero_position) noexcept {
    As5047uHandler* handler_ptr = handler(deviceIndex);
    if (!handler_ptr) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    return handler_ptr->SetZeroPosition(zero_position);
}

std::vector<As5047uError> EncoderController::ReadAllAngles(std::vector<uint16_t>& angles, 
                                                          std::vector<uint8_t>& device_indices) noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    angles.clear();
    device_indices.clear();
    std::vector<As5047uError> errors;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && deviceInitialized_[i] && encoderHandlers_[i]) {
            uint16_t angle;
            As5047uError error = encoderHandlers_[i]->ReadAngle(angle);
            
            angles.push_back(angle);
            device_indices.push_back(i);
            errors.push_back(error);
        }
    }
    
    return errors;
}

std::vector<As5047uError> EncoderController::ReadAllVelocities(std::vector<double>& velocities_rpm, 
                                                              std::vector<uint8_t>& device_indices) noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    velocities_rpm.clear();
    device_indices.clear();
    std::vector<As5047uError> errors;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && deviceInitialized_[i] && encoderHandlers_[i]) {
            double velocity;
            As5047uError error = encoderHandlers_[i]->ReadVelocityRPM(velocity);
            
            velocities_rpm.push_back(velocity);
            device_indices.push_back(i);
            errors.push_back(error);
        }
    }
    
    return errors;
}

bool EncoderController::CheckAllDevicesHealth() noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && deviceInitialized_[i] && encoderHandlers_[i]) {
            As5047uDiagnostics diagnostics;
            As5047uError error = encoderHandlers_[i]->ReadDiagnostics(diagnostics);
            
            if (error != As5047uError::SUCCESS || !diagnostics.communication_ok || !diagnostics.magnetic_field_ok) {
                return false;
            }
        }
    }
    
    return true;
}

std::string EncoderController::GetStatusReport() const noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    std::ostringstream oss;
    oss << "=== ENCODER CONTROLLER STATUS ===\n";
    oss << "Initialized: " << (initialized_ ? "YES" : "NO") << "\n";
    oss << "Active Devices: " << static_cast<int>(activeDeviceCount_) << "/" << MAX_ENCODER_DEVICES << "\n\n";
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        oss << "Device " << static_cast<int>(i) << " (" << (i == 0 ? "Onboard" : "External") << "): ";
        
        if (!deviceActive_[i]) {
            oss << "INACTIVE\n";
            continue;
        }
        
        if (!deviceInitialized_[i]) {
            oss << "ACTIVE but NOT INITIALIZED\n";
            continue;
        }
        
        if (!encoderHandlers_[i]) {
            oss << "ACTIVE but NO HANDLER\n";
            continue;
        }
        
        // Read current angle and status
        uint16_t angle;
        As5047uError error = encoderHandlers_[i]->ReadAngle(angle);
        
        if (error == As5047uError::SUCCESS) {
            double angle_deg = As5047uHandler::LSBToDegrees(angle);
            oss << "HEALTHY - Angle: " << angle_deg << "° (" << angle << " LSB)\n";
        } else {
            oss << "ERROR - " << As5047uErrorToString(error) << "\n";
        }
    }
    
    return oss.str();
}

void EncoderController::DumpAllDiagnostics() const noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "=== ENCODER CONTROLLER DIAGNOSTICS ===");
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Controller Initialized: " + std::string(initialized_ ? "YES" : "NO"));
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Active Devices: " + std::to_string(activeDeviceCount_) + "/" + std::to_string(MAX_ENCODER_DEVICES));
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && encoderHandlers_[i]) {
            std::string deviceType = (i == 0) ? "Onboard" : "External";
            Logger::GetInstance().Info("ENCODER_CONTROLLER", "--- Device " + std::to_string(i) + " (" + deviceType + ") ---");
            encoderHandlers_[i]->DumpDiagnostics();
        }
    }
}

void EncoderController::ResetAllStatistics() noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && deviceInitialized_[i] && encoderHandlers_[i]) {
            // AS5047U handlers don't have explicit reset statistics method,
            // but we could add one if needed in the future
            Logger::GetInstance().Info("ENCODER_CONTROLLER", "Reset statistics for device " + std::to_string(i));
        }
    }
}

bool EncoderController::ResetAllDevices() noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && encoderHandlers_[i]) {
            As5047uError error = encoderHandlers_[i]->ResetToDefaults();
            if (error != As5047uError::SUCCESS) {
                allSuccess = false;
                Logger::GetInstance().Error("ENCODER_CONTROLLER", 
                    "Failed to reset device " + std::to_string(i) + ": " + As5047uErrorToString(error));
            } else {
                Logger::GetInstance().Info("ENCODER_CONTROLLER", "Reset device " + std::to_string(i) + " successfully");
            }
        }
    }
    
    return allSuccess;
}

void EncoderController::Shutdown() noexcept {
    MutexLockGuard lock(controller_mutex_);
    
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Shutting down all encoder devices");
    
    // Deinitialize all devices
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (deviceActive_[i] && encoderHandlers_[i]) {
            encoderHandlers_[i]->Deinitialize();
            encoderHandlers_[i].reset();
        }
        deviceActive_[i] = false;
        deviceInitialized_[i] = false;
    }
    
    activeDeviceCount_ = 0;
    initialized_ = false;
    
    Logger::GetInstance().Info("ENCODER_CONTROLLER", "Shutdown completed");
}

bool EncoderController::IsValidDeviceIndex(uint8_t deviceIndex) const noexcept {
    return deviceIndex < MAX_ENCODER_DEVICES;
}

bool EncoderController::IsExternalDeviceIndex(uint8_t deviceIndex) const noexcept {
    return (deviceIndex == EXTERNAL_DEVICE_1_INDEX || 
            deviceIndex == EXTERNAL_DEVICE_2_INDEX || 
            deviceIndex == EXTERNAL_DEVICE_3_INDEX);
}