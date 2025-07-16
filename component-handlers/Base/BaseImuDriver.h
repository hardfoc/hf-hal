#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Forward declarations for vector/quaternion types
struct ImuVector3 {
    float x{0};
    float y{0};
    float z{0};
    uint8_t accuracy{0};
};

struct ImuQuaternion {
    float w{1};
    float x{0};
    float y{0};
    float z{0};
    uint8_t accuracy{0};
};

// Enum for IMU type
enum class ImuType {
    BNO08x,
    // Add more as needed
};

// Abstract base class for all IMU drivers
class BaseImuDriver {
public:
    virtual ~BaseImuDriver() = default;

    // Initialization
    virtual bool Initialize() noexcept = 0;
    virtual bool EnsureInitialized() noexcept = 0;
    virtual bool Deinitialize() noexcept = 0;
    virtual bool IsInitialized() const noexcept = 0;

    // Data retrieval
    virtual ImuVector3 GetAccel() const = 0;
    virtual ImuVector3 GetGyro() const = 0;
    virtual ImuVector3 GetMag() const = 0;
    virtual ImuQuaternion GetQuaternion() const = 0;

    // Status/error
    virtual bool IsOnline() const noexcept = 0;
    virtual int GetLastError() const noexcept = 0;
    virtual ImuType GetType() const noexcept = 0;
    virtual std::string GetName() const = 0;
}; 