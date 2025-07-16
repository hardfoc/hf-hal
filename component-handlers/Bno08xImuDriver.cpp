#include "Bno08xImuDriver.h"
// #include <BNO085.hpp> // Include the real driver when available

Bno08xImuDriver::Bno08xImuDriver(IBNO085Transport* transport, const std::string& name)
    : transport_(transport), name_(name) {
    // bno085_ will be constructed in Initialize
}

Bno08xImuDriver::~Bno08xImuDriver() = default;

bool Bno08xImuDriver::Initialize() noexcept {
    if (initialized_) return true;
    // TODO: Construct and initialize the BNO085 driver
    // bno085_ = std::make_unique<BNO085>(transport_);
    // initialized_ = bno085_->begin();
    // last_error_ = bno085_ ? bno085_->getLastError() : -1;
    // For now, stub:
    initialized_ = true;
    last_error_ = 0;
    return initialized_;
}

bool Bno08xImuDriver::EnsureInitialized() noexcept {
    if (!initialized_) {
        return Initialize();
    }
    return true;
}

bool Bno08xImuDriver::Deinitialize() noexcept {
    // TODO: Deinitialize the BNO085 driver if needed
    initialized_ = false;
    return true;
}

bool Bno08xImuDriver::IsInitialized() const noexcept {
    return initialized_;
}

ImuVector3 Bno08xImuDriver::GetAccel() const {
    // TODO: Map to bno085_->getLatest(BNO085Sensor::Accelerometer)
    return ImuVector3{0, 0, 0, 0};
}

ImuVector3 Bno08xImuDriver::GetGyro() const {
    // TODO: Map to bno085_->getLatest(BNO085Sensor::Gyroscope)
    return ImuVector3{0, 0, 0, 0};
}

ImuVector3 Bno08xImuDriver::GetMag() const {
    // TODO: Map to bno085_->getLatest(BNO085Sensor::Magnetometer)
    return ImuVector3{0, 0, 0, 0};
}

ImuQuaternion Bno08xImuDriver::GetQuaternion() const {
    // TODO: Map to bno085_->getLatest(BNO085Sensor::RotationVector)
    return ImuQuaternion{1, 0, 0, 0, 0};
}

bool Bno08xImuDriver::IsOnline() const noexcept {
    // TODO: Check actual device status
    return initialized_;
}

int Bno08xImuDriver::GetLastError() const noexcept {
    return last_error_;
}

ImuType Bno08xImuDriver::GetType() const noexcept {
    return ImuType::BNO08x;
}

std::string Bno08xImuDriver::GetName() const {
    return name_;
} 