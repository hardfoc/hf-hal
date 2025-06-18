#ifndef COMPONENT_HANDLER_MULTI_READINGS_H_
#define COMPONENT_HANDLER_MULTI_READINGS_H_

#include <array>
#include <initializer_list>
#include <algorithm>
#include <iterator>
#include <cstddef>
#include <functional>

/**
 * @file MultiReadings.h
 * @brief Generic multi-reading template for handling multiple sensor readings.
 */

/**
 * @class MultiReadings
 * @brief Template class for handling multiple readings from identifiers.
 *
 * @tparam IdentifierType The type of identifier (e.g., AdcInputSensor, GpioPin).
 * @tparam DataType The type of data being read (e.g., uint32_t, float).
 * @tparam MaxCount The maximum number of readings.
 * @tparam ExtraType Additional data type (use void if not needed).
 */
template <typename IdentifierType, typename DataType, uint8_t MaxCount, typename ExtraType = void>
class MultiReadings {
public:
    /**
     * @brief Structure representing a single reading.
     */
    struct Reading {
        IdentifierType identifier;
        uint8_t numSamples;
        DataType dataStorage;
        uint8_t successfulReadings;
        
        // Constructor
        Reading(IdentifierType id, uint8_t samples)
            : identifier(id), numSamples(samples), dataStorage{}, successfulReadings(0) {}
        
        // Reset method
        void Reset() {
            dataStorage = DataType{};
            successfulReadings = 0;
        }
    };

    /**
     * @brief Type alias for the string conversion function.
     */
    using StringConverter = std::function<std::string_view(IdentifierType)>;

    /**
     * @brief Default constructor.
     */
    MultiReadings() noexcept : stringConverter_(nullptr), count_(0) {}

    /**
     * @brief Constructor with string converter.
     * @param converter Function to convert identifier to string.
     */
    explicit MultiReadings(StringConverter converter) noexcept 
        : stringConverter_(converter), count_(0) {}

    /**
     * @brief Constructor with initializer list and string converter.
     * @param inputList Initializer list of identifier-sample pairs.
     * @param converter Function to convert identifier to string.
     */
    MultiReadings(const std::initializer_list<std::pair<IdentifierType, uint8_t>>& inputList,
                  StringConverter converter) noexcept
        : stringConverter_(converter), count_(0) {
        for (const auto& pair : inputList) {
            if (count_ >= MaxCount) break;
            readings_[count_++] = Reading(pair.first, pair.second);
        }
    }

    /**
     * @brief Copy constructor.
     */
    MultiReadings(const MultiReadings& other) noexcept = default;

    /**
     * @brief Copy assignment operator.
     */
    MultiReadings& operator=(const MultiReadings& other) noexcept = default;

    /**
     * @brief Destructor.
     */
    ~MultiReadings() noexcept = default;

    /**
     * @brief Add a reading specification.
     * @param identifier The identifier to read from.
     * @param numSamples Number of samples to average.
     * @return true if added successfully, false if full.
     */
    bool AddReading(IdentifierType identifier, uint8_t numSamples) noexcept {
        if (count_ >= MaxCount) return false;
        readings_[count_++] = Reading(identifier, numSamples);
        return true;
    }

    /**
     * @brief Get the number of readings.
     * @return Number of readings.
     */
    uint8_t GetCount() const noexcept {
        return count_;
    }

    /**
     * @brief Get a reading by index.
     * @param index Index of the reading.
     * @return Pointer to the reading or nullptr if out of bounds.
     */
    Reading* GetReading(uint8_t index) noexcept {
        if (index >= count_) return nullptr;
        return &readings_[index];
    }

    /**
     * @brief Get a reading by index (const version).
     * @param index Index of the reading.
     * @return Const pointer to the reading or nullptr if out of bounds.
     */
    const Reading* GetReading(uint8_t index) const noexcept {
        if (index >= count_) return nullptr;
        return &readings_[index];
    }

    /**
     * @brief Find a reading by identifier.
     * @param identifier The identifier to search for.
     * @return Pointer to the reading or nullptr if not found.
     */
    Reading* FindReading(IdentifierType identifier) noexcept {
        for (uint8_t i = 0; i < count_; ++i) {
            if (readings_[i].identifier == identifier) {
                return &readings_[i];
            }
        }
        return nullptr;
    }

    /**
     * @brief Reset all readings.
     */
    void ResetAll() noexcept {
        for (uint8_t i = 0; i < count_; ++i) {
            readings_[i].Reset();
        }
    }

    /**
     * @brief Get string representation of identifier.
     * @param identifier The identifier to convert.
     * @return String representation.
     */
    std::string_view GetIdentifierString(IdentifierType identifier) const noexcept {
        if (stringConverter_) {
            return stringConverter_(identifier);
        }
        return "UNKNOWN";
    }

    /**
     * @brief Iterator support - begin.
     */
    Reading* begin() noexcept {
        return readings_.data();
    }

    /**
     * @brief Iterator support - end.
     */
    Reading* end() noexcept {
        return readings_.data() + count_;
    }

    /**
     * @brief Const iterator support - begin.
     */
    const Reading* begin() const noexcept {
        return readings_.data();
    }

    /**
     * @brief Const iterator support - end.
     */
    const Reading* end() const noexcept {
        return readings_.data() + count_;
    }

private:
    std::array<Reading, MaxCount> readings_;
    StringConverter stringConverter_;
    uint8_t count_;
};

#endif // COMPONENT_HANDLER_MULTI_READINGS_H_
