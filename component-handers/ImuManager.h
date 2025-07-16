#pragma once

#include "BaseImuDriver.h"
#include <vector>
#include <memory>
#include <mutex>
#include <optional>

class ImuManager {
public:
    /**
     * @brief Get the singleton instance of ImuManager.
     * @return Reference to the singleton ImuManager.
     */
    static ImuManager& GetInstance();

    /**
     * @brief Ensure all IMUs are initialized (lazy initialization).
     * @return true if all IMUs are initialized, false otherwise.
     */
    bool EnsureInitialized();

    /**
     * @brief Check if all IMUs are initialized.
     * @return true if initialized, false otherwise.
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Deinitialize all IMUs.
     * @return true if deinitialized, false otherwise.
     */
    bool Deinitialize();

    /**
     * @brief Get the number of IMUs managed.
     * @return Number of IMUs.
     */
    size_t GetImuCount() const noexcept;

    /**
     * @brief Get a pointer to the IMU driver of the given type (if present).
     * @param type The IMU type to search for.
     * @return Pointer to the driver, or nullptr if not found.
     */
    BaseImuDriver* GetImuByType(ImuType type) noexcept;

    /**
     * @brief Get a pointer to the IMU driver by index.
     * @param idx Index in the internal list.
     * @return Pointer to the driver, or nullptr if out of range.
     */
    BaseImuDriver* GetImuByIndex(size_t idx) noexcept;

    /**
     * @brief Get a vector of all IMU drivers.
     * @return Vector of BaseImuDriver*.
     */
    std::vector<BaseImuDriver*> GetAllImus() noexcept;

    // Delete copy/move
    ImuManager(const ImuManager&) = delete;
    ImuManager& operator=(const ImuManager&) = delete;
    ImuManager(ImuManager&&) = delete;
    ImuManager& operator=(ImuManager&&) = delete;

private:
    ImuManager();
    ~ImuManager();

    bool initialized_ = false;
    std::vector<std::unique_ptr<BaseImuDriver>> imus_;
    mutable std::mutex mutex_;
}; 