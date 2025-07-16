#include "MotorController.h"
#include <stdexcept>

// Dummy communication interface for demonstration (replace with real implementation)
class DummyComm : public TMC9660CommInterface {
public:
    bool transfer(const TMCLFrame&, TMCLReply&, uint8_t) override { return true; }
};

MotorController& MotorController::GetInstance() {
    static MotorController instance;
    return instance;
}

MotorController::MotorController() {
    // Use dummy comm and default address/config for now
    static DummyComm comm;
    tmcHandler_ = std::make_unique<Tmc9660Handler>(comm, 0x01);
}

bool MotorController::Initialize() {
    return tmcHandler_ && tmcHandler_->Initialize();
}

Tmc9660Handler& MotorController::handler() {
    if (!tmcHandler_) throw std::runtime_error("Tmc9660Handler not initialized");
    return *tmcHandler_;
}

TMC9660& MotorController::driver() {
    return handler().driver();
}

Tmc9660Handler::Gpio& MotorController::gpio(uint8_t gpioNumber) {
    return handler().gpio(gpioNumber);
}

Tmc9660Handler::Adc& MotorController::adc() {
    return handler().adc();
} 