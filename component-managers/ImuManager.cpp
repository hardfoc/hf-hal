#include "ImuManager.h"
#include "Bno08xImuDriver.h"
#include <iostream> // For debug output, remove in production

// TODO: Include actual transport and detection logic
// #include "YourI2cBusOrTransport.h"

ImuManager& ImuManager::GetInstance() {
    static ImuManager instance;
    return instance;
}

ImuManager::ImuManager() = default;
ImuManager::~ImuManager() = default;

bool ImuManager::EnsureInitialized() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) return true;

    // TODO: Detect all IMUs on the system (currently only BNO08x on I2C)
    // This is a stub. Replace with actual detection logic.
    // Example: If BNO08x is present at known I2C address, add it.
    if (imus_.empty()) {
        // Replace nullptr with actual transport instance
        IBNO085Transport* transport = nullptr; // TODO: Create real transport
        auto bno = std::make_unique<Bno08xImuDriver>(transport, "BNO08x_0");
        if (bno->EnsureInitialized()) {
            imus_.push_back(std::move(bno));
        } else {
            // Optionally log error
        }
    }
    initialized_ = !imus_.empty();
    return initialized_;
}

bool ImuManager::IsInitialized() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

bool ImuManager::Deinitialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    bool all_ok = true;
    for (auto& imu : imus_) {
        all_ok &= imu->Deinitialize();
    }
    imus_.clear();
    initialized_ = false;
    return all_ok;
}

size_t ImuManager::GetImuCount() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return imus_.size();
}

BaseImuDriver* ImuManager::GetImuByType(ImuType type) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& imu : imus_) {
        if (imu->GetType() == type) return imu.get();
    }
    return nullptr;
}

BaseImuDriver* ImuManager::GetImuByIndex(size_t idx) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (idx < imus_.size()) return imus_[idx].get();
    return nullptr;
}

std::vector<BaseImuDriver*> ImuManager::GetAllImus() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<BaseImuDriver*> result;
    for (auto& imu : imus_) result.push_back(imu.get());
    return result;
} 