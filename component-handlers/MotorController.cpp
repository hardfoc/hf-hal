#include "MotorController.h"
#include "CommChannelsManager.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include <algorithm>
#include <string>

MotorController& MotorController::GetInstance() {
    static MotorController instance;
    return instance;
}

MotorController::MotorController() 
    : onboardDeviceCreated_(false), initialized_(false), deviceMutex_() {
    // Initialize all device slots as empty and not active
    tmcHandlers_.fill(nullptr);
    deviceInitialized_.fill(false);
    deviceActive_.fill(false);
}

bool MotorController::Initialize() {
    MutexLockGuard lock(deviceMutex_);
    
    // Initialize CommChannelsManager first
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        return false;
    }
    
    // Automatically create onboard TMC9660 device if not already created
    if (!onboardDeviceCreated_) {
        BaseSpi* spiInterface = commManager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
        if (spiInterface) {
            // Create onboard TMC9660 handler with default address 1
            auto onboardHandler = std::make_unique<Tmc9660Handler>(
                *spiInterface, 
                0x01,  // Default TMC9660 address
                &Tmc9660Handler::kDefaultBootConfig
            );
            
            tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
            deviceActive_[ONBOARD_TMC9660_INDEX] = true;
            deviceInitialized_[ONBOARD_TMC9660_INDEX] = false;
            onboardDeviceCreated_ = true;
        }
    }
    
    // Initialize all active devices
    bool allSuccess = true;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i]) {
            deviceInitialized_[i] = tmcHandlers_[i]->Initialize();
            if (!deviceInitialized_[i]) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

bool MotorController::CreateOnboardDevice(BaseSpi& spiInterface, 
                                        uint8_t address,
                                        const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceActive_[ONBOARD_TMC9660_INDEX]) {
        return false; // Onboard device already exists
    }
    
    // Create onboard TMC9660 handler
    auto onboardHandler = std::make_unique<Tmc9660Handler>(
        spiInterface, 
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
    deviceActive_[ONBOARD_TMC9660_INDEX] = true;
    deviceInitialized_[ONBOARD_TMC9660_INDEX] = false;
    onboardDeviceCreated_ = true;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[ONBOARD_TMC9660_INDEX] = tmcHandlers_[ONBOARD_TMC9660_INDEX]->Initialize();
    }
    
    return true;
}

bool MotorController::CreateOnboardDevice(BaseUart& uartInterface,
                                        uint8_t address, 
                                        const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceActive_[ONBOARD_TMC9660_INDEX]) {
        return false; // Onboard device already exists
    }
    
    // Create onboard TMC9660 handler with UART
    auto onboardHandler = std::make_unique<Tmc9660Handler>(
        uartInterface, 
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
    deviceActive_[ONBOARD_TMC9660_INDEX] = true;
    deviceInitialized_[ONBOARD_TMC9660_INDEX] = false;
    onboardDeviceCreated_ = true;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[ONBOARD_TMC9660_INDEX] = tmcHandlers_[ONBOARD_TMC9660_INDEX]->Initialize();
    }
    
    return true;
}

bool MotorController::CreateExternalDevice(uint8_t csDeviceIndex, 
                                         SpiDeviceId spiDeviceId, 
                                         uint8_t address,
                                         const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false; // Invalid device index for external device
    }
    
    if (deviceActive_[csDeviceIndex]) {
        return false; // Device slot already occupied
    }
    
    // Get SPI interface from CommChannelsManager
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        return false; // CommChannelsManager not initialized
    }
    
    BaseSpi* spiInterface = commManager.GetSpiDevice(spiDeviceId);
    if (!spiInterface) {
        return false; // Invalid SPI device ID or interface not available
    }
    
    // Create external TMC9660 handler
    auto externalHandler = std::make_unique<Tmc9660Handler>(
        *spiInterface, 
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[csDeviceIndex] = std::move(externalHandler);
    deviceActive_[csDeviceIndex] = true;
    deviceInitialized_[csDeviceIndex] = false;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[csDeviceIndex] = tmcHandlers_[csDeviceIndex]->Initialize();
    }
    
    return true;
}

bool MotorController::CreateExternalDevice(uint8_t csDeviceIndex,
                                         BaseUart& uartInterface,
                                         uint8_t address, 
                                         const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false; // Invalid device index for external device
    }
    
    if (deviceActive_[csDeviceIndex]) {
        return false; // Device slot already occupied
    }
    
    // Create external TMC9660 handler with UART
    auto externalHandler = std::make_unique<Tmc9660Handler>(
        uartInterface, 
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[csDeviceIndex] = std::move(externalHandler);
    deviceActive_[csDeviceIndex] = true;
    deviceInitialized_[csDeviceIndex] = false;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[csDeviceIndex] = tmcHandlers_[csDeviceIndex]->Initialize();
    }
    
    return true;
}

bool MotorController::DeleteExternalDevice(uint8_t csDeviceIndex) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false; // Cannot delete onboard device or invalid index
    }
    
    if (!deviceActive_[csDeviceIndex]) {
        return false; // Device doesn't exist
    }
    
    // Reset device slot
    tmcHandlers_[csDeviceIndex].reset();
    deviceActive_[csDeviceIndex] = false;
    deviceInitialized_[csDeviceIndex] = false;
    
    return true;
}

bool MotorController::IsExternalDeviceIndex(uint8_t csDeviceIndex) const noexcept {
    return (csDeviceIndex == EXTERNAL_DEVICE_1_INDEX || 
            csDeviceIndex == EXTERNAL_DEVICE_2_INDEX);
}

uint8_t MotorController::GetDeviceCount() const noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i]) {
            count++;
        }
    }
    return count;
}

bool MotorController::IsDeviceValid(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(deviceMutex_);
    return (deviceIndex < MAX_TMC9660_DEVICES) && deviceActive_[deviceIndex];
}

bool MotorController::IsExternalSlotAvailable(uint8_t csDeviceIndex) const noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false;
    }
    
    return !deviceActive_[csDeviceIndex];
}

std::vector<uint8_t> MotorController::GetActiveDeviceIndices() const noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    std::vector<uint8_t> activeIndices;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i]) {
            activeIndices.push_back(i);
        }
    }
    return activeIndices;
}

Tmc9660Handler* MotorController::handler(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceIndex >= MAX_TMC9660_DEVICES || !deviceActive_[deviceIndex] || !tmcHandlers_[deviceIndex]) {
        return nullptr; // Invalid or inactive device
    }
    
    return tmcHandlers_[deviceIndex].get();
}

std::shared_ptr<TMC9660> MotorController::driver(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceIndex >= MAX_TMC9660_DEVICES || !deviceActive_[deviceIndex] || !tmcHandlers_[deviceIndex]) {
        return nullptr; // Invalid or inactive device
    }
    
    if (!deviceInitialized_[deviceIndex]) {
        return nullptr; // Device not initialized
    }
    
    return tmcHandlers_[deviceIndex]->driver();
}

std::vector<bool> MotorController::InitializeAllDevices() {
    MutexLockGuard lock(deviceMutex_);
    
    std::vector<bool> results;
    
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i]) {
            deviceInitialized_[i] = tmcHandlers_[i]->Initialize();
            results.push_back(deviceInitialized_[i]);
        }
    }
    
    return results;
}

std::vector<bool> MotorController::GetInitializationStatus() const {
    MutexLockGuard lock(deviceMutex_);
    
    std::vector<bool> status;
    
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i]) {
            status.push_back(deviceInitialized_[i]);
        }
    }
    
    return status;
} 
