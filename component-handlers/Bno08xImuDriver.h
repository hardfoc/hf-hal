#pragma once

#include "BaseImuDriver.h"
#include <memory>

// Forward declaration for BNO085
class BNO085;
class IBNO085Transport;

class Bno08xImuDriver : public BaseImuDriver {
public:
    Bno08xImuDriver(IBNO085Transport* transport, const std::string& name = "BNO08x");
    ~Bno08xImuDriver() override;

    // Initialization
    bool Initialize() noexcept override;
    bool EnsureInitialized() noexcept override;
    bool Deinitialize() noexcept override;
    bool IsInitialized() const noexcept override;

    // Data retrieval
    ImuVector3 GetAccel() const override;
    ImuVector3 GetGyro() const override;
    ImuVector3 GetMag() const override;
    ImuQuaternion GetQuaternion() const override;

    // Status/error
    bool IsOnline() const noexcept override;
    int GetLastError() const noexcept override;
    ImuType GetType() const noexcept override;
    std::string GetName() const override;

    // Expose underlying driver for advanced use
    BNO085* GetRawDriver() noexcept { return bno085_.get(); }

private:
    std::unique_ptr<BNO085> bno085_;
    IBNO085Transport* transport_;
    std::string name_;
    bool initialized_ = false;
    int last_error_ = 0;
}; 