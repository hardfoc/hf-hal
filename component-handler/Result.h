#ifndef COMPONENT_HANDLER_RESULT_H_
#define COMPONENT_HANDLER_RESULT_H_

#include <cstdint>
#include <string_view>

/**
 * @file Result.h
 * @brief Unified error handling and result types for the HardFOC HAL system.
 * 
 * This header provides a consistent error handling framework using X-macros
 * for maintainable and extensible error definitions. All HAL operations
 * return standardized result types with detailed error information.
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

//==============================================================================
// ERROR CODE DEFINITIONS USING X-MACROS
//==============================================================================

/**
 * @brief X-macro definition for all HardFOC error codes.
 * 
 * Format: X(ENUM_NAME, numeric_value, "description_string")
 * 
 * This approach provides:
 * - Automatic enum generation
 * - Compile-time string conversion
 * - Guaranteed consistency
 * - Easy maintenance and extension
 */
#define HARDFOC_ERROR_CODES(X) \
    /* Success codes */ \
    X(SUCCESS,                    0,   "Operation completed successfully") \
    X(SUCCESS_WITH_WARNING,       1,   "Operation completed with warnings") \
    \
    /* Generic error codes (1-99) */ \
    X(ERROR_GENERIC,              10,  "Generic error occurred") \
    X(ERROR_INVALID_PARAMETER,    11,  "Invalid parameter provided") \
    X(ERROR_NULL_POINTER,         12,  "Null pointer encountered") \
    X(ERROR_OUT_OF_RANGE,         13,  "Value out of valid range") \
    X(ERROR_BUFFER_OVERFLOW,      14,  "Buffer overflow detected") \
    X(ERROR_MEMORY_ALLOCATION,    15,  "Memory allocation failed") \
    X(ERROR_TIMEOUT,              16,  "Operation timed out") \
    X(ERROR_BUSY,                 17,  "Resource is busy") \
    X(ERROR_NOT_READY,            18,  "System not ready") \
    X(ERROR_OPERATION_FAILED,     19,  "Operation failed") \
    \
    /* Initialization errors (100-199) */ \
    X(ERROR_NOT_INITIALIZED,      100, "System not initialized") \
    X(ERROR_ALREADY_INITIALIZED,  101, "System already initialized") \
    X(ERROR_INIT_FAILED,          102, "Initialization failed") \
    X(ERROR_DEPENDENCY_MISSING,   103, "Required dependency not available") \
    X(ERROR_CONFIG_INVALID,       104, "Configuration is invalid") \
    X(ERROR_HARDWARE_NOT_FOUND,   105, "Hardware component not found") \
    X(ERROR_FIRMWARE_INCOMPATIBLE, 106, "Firmware version incompatible") \
    \
    /* GPIO specific errors (200-299) */ \
    X(ERROR_GPIO_PIN_NOT_FOUND,   200, "GPIO pin not found or invalid") \
    X(ERROR_GPIO_ALREADY_REGISTERED, 201, "GPIO pin already registered") \
    X(ERROR_GPIO_NOT_REGISTERED,  202, "GPIO pin not registered") \
    X(ERROR_GPIO_DIRECTION_INVALID, 203, "GPIO direction invalid for operation") \
    X(ERROR_GPIO_READ_FAILED,     204, "Failed to read GPIO state") \
    X(ERROR_GPIO_WRITE_FAILED,    205, "Failed to write GPIO state") \
    X(ERROR_GPIO_CONFIG_FAILED,   206, "GPIO configuration failed") \
    X(ERROR_GPIO_INTERRUPT_FAILED, 207, "GPIO interrupt configuration failed") \
    \
    /* ADC specific errors (300-399) */ \
    X(ERROR_ADC_CHANNEL_NOT_FOUND, 300, "ADC channel not found or invalid") \
    X(ERROR_ADC_ALREADY_REGISTERED, 301, "ADC channel already registered") \
    X(ERROR_ADC_NOT_REGISTERED,   302, "ADC channel not registered") \
    X(ERROR_ADC_READ_FAILED,      303, "Failed to read ADC value") \
    X(ERROR_ADC_CALIBRATION_FAILED, 304, "ADC calibration failed") \
    X(ERROR_ADC_OUT_OF_RANGE,     305, "ADC value out of expected range") \
    X(ERROR_ADC_INVALID_SAMPLES,  306, "Invalid number of samples specified") \
    X(ERROR_ADC_CHIP_NOT_FOUND,   307, "ADC chip not found or not responding") \
    \
    /* Communication errors (400-499) */ \
    X(ERROR_I2C_INIT_FAILED,      400, "I2C initialization failed") \
    X(ERROR_I2C_TRANSMISSION_FAILED, 401, "I2C transmission failed") \
    X(ERROR_SPI_INIT_FAILED,      402, "SPI initialization failed") \
    X(ERROR_SPI_TRANSMISSION_FAILED, 403, "SPI transmission failed") \
    X(ERROR_UART_INIT_FAILED,     404, "UART initialization failed") \
    X(ERROR_UART_TRANSMISSION_FAILED, 405, "UART transmission failed") \
    X(ERROR_CAN_INIT_FAILED,      406, "CAN bus initialization failed") \
    X(ERROR_CAN_TRANSMISSION_FAILED, 407, "CAN bus transmission failed") \
    \
    /* Resource errors (500-599) */ \
    X(ERROR_RESOURCE_UNAVAILABLE, 500, "Resource temporarily unavailable") \
    X(ERROR_RESOURCE_LOCKED,      501, "Resource is locked by another process") \
    X(ERROR_INSUFFICIENT_MEMORY,  502, "Insufficient memory available") \
    X(ERROR_QUOTA_EXCEEDED,       503, "Resource quota exceeded") \
    \
    /* System errors (600-699) */ \
    X(ERROR_SYSTEM_FAULT,         600, "System fault detected") \
    X(ERROR_WATCHDOG_TIMEOUT,     601, "Watchdog timeout") \
    X(ERROR_POWER_FAILURE,        602, "Power failure detected") \
    X(ERROR_TEMPERATURE_EXCEEDED, 603, "Temperature limit exceeded") \
    X(ERROR_VOLTAGE_OUT_OF_RANGE, 604, "Voltage out of acceptable range")

//==============================================================================
// ENUM GENERATION
//==============================================================================

/**
 * @brief Unified result codes for all HardFOC operations.
 * 
 * This enum is automatically generated from the X-macro definitions above,
 * ensuring consistency and maintainability.
 */
enum class ResultCode : uint16_t {
#define X(name, value, desc) name = value,
    HARDFOC_ERROR_CODES(X)
#undef X
};

//==============================================================================
// RESULT CLASS
//==============================================================================

/**
 * @brief Result wrapper class for HardFOC operations.
 * 
 * This class provides a structured way to return both success/failure status
 * and associated data from HAL operations. It follows modern C++ practices
 * for error handling without exceptions.
 * 
 * @tparam T The type of data returned on success
 */
template<typename T = void>
class Result {
public:
    /**
     * @brief Construct a successful result with data.
     * @param value The data to return
     */
    explicit Result(T&& value) noexcept
        : result_(ResultCode::SUCCESS), value_(std::forward<T>(value)) {}
    
    /**
     * @brief Construct a successful result with data (copy).
     * @param value The data to return
     */
    explicit Result(const T& value) noexcept
        : result_(ResultCode::SUCCESS), value_(value) {}
    
    /**
     * @brief Construct an error result.
     * @param error The error code
     */
    explicit Result(ResultCode error) noexcept
        : result_(error) {}
    
    /**
     * @brief Check if the result represents success.
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool IsSuccess() const noexcept {
        return result_ == ResultCode::SUCCESS || result_ == ResultCode::SUCCESS_WITH_WARNING;
    }
    
    /**
     * @brief Check if the result represents an error.
     * @return true if error, false otherwise
     */
    [[nodiscard]] bool IsError() const noexcept {
        return !IsSuccess();
    }
    
    /**
     * @brief Get the result code.
     * @return The result code
     */
    [[nodiscard]] ResultCode GetResult() const noexcept {
        return result_;
    }
    
    /**
     * @brief Get the result data (only valid if successful).
     * @return Reference to the data
     * @warning Only call this if IsSuccess() returns true
     */
    [[nodiscard]] const T& GetValue() const noexcept {
        return value_;
    }
    
    /**
     * @brief Get the result data (only valid if successful).
     * @return Reference to the data
     * @warning Only call this if IsSuccess() returns true
     */
    [[nodiscard]] T& GetValue() noexcept {
        return value_;
    }
    
    /**
     * @brief Get mutable reference to result data.
     * @return Mutable reference to the data
     * @warning Only call this if IsSuccess() returns true
     */
    [[nodiscard]] T& operator*() noexcept {
        return value_;
    }
    
    /**
     * @brief Get const reference to result data.
     * @return Const reference to the data
     * @warning Only call this if IsSuccess() returns true
     */
    [[nodiscard]] const T& operator*() const noexcept {
        return value_;
    }

private:
    ResultCode result_;
    T value_;
};

//==============================================================================
// VOID SPECIALIZATION
//==============================================================================

/**
 * @brief Specialization for void results (no data returned).
 */
template<>
class Result<void> {
public:
    /**
     * @brief Construct a successful void result.
     */
    explicit Result() noexcept : result_(ResultCode::SUCCESS) {}
    
    /**
     * @brief Construct an error result.
     * @param error The error code
     */
    explicit Result(ResultCode error) noexcept : result_(error) {}
    
    /**
     * @brief Check if the result represents success.
     * @return true if successful, false otherwise
     */
    [[nodiscard]] bool IsSuccess() const noexcept {
        return result_ == ResultCode::SUCCESS || result_ == ResultCode::SUCCESS_WITH_WARNING;
    }
    
    /**
     * @brief Check if the result represents an error.
     * @return true if error, false otherwise
     */
    [[nodiscard]] bool IsError() const noexcept {
        return !IsSuccess();
    }
    
    /**
     * @brief Get the result code.
     * @return The result code
     */
    [[nodiscard]] ResultCode GetResult() const noexcept {
        return result_;
    }

private:
    ResultCode result_;
};

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

/**
 * @brief Get human-readable description of a result code.
 * @param result The result code to describe
 * @return String description of the result
 */
[[nodiscard]] constexpr std::string_view GetResultDescription(ResultCode result) noexcept {
    switch (result) {
#define X(name, value, desc) case ResultCode::name: return desc;
        HARDFOC_ERROR_CODES(X)
#undef X
        default:
            return "Unknown result code";
    }
}

/**
 * @brief Get the name of a result code as a string.
 * @param result The result code
 * @return String name of the result code
 */
[[nodiscard]] constexpr std::string_view GetResultName(ResultCode result) noexcept {
    switch (result) {
#define X(name, value, desc) case ResultCode::name: return #name;
        HARDFOC_ERROR_CODES(X)
#undef X
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Check if a result code represents success.
 * @param result The result code to check
 * @return true if successful, false otherwise
 */
[[nodiscard]] constexpr bool IsSuccessResult(ResultCode result) noexcept {
    return result == ResultCode::SUCCESS || result == ResultCode::SUCCESS_WITH_WARNING;
}

//==============================================================================
// CONVENIENCE MACROS
//==============================================================================

/**
 * @brief Create a success result.
 */
#define HARDFOC_SUCCESS() Result<void>(ResultCode::SUCCESS)

/**
 * @brief Create a success result with data.
 */
#define HARDFOC_SUCCESS_WITH_DATA(data) Result(data)

/**
 * @brief Create an error result.
 */
#define HARDFOC_ERROR(code) Result<void>(ResultCode::code)

/**
 * @brief Create an error result with specific type.
 */
#define HARDFOC_ERROR_T(type, code) Result<type>(ResultCode::code)

/**
 * @brief Return early if result is an error.
 */
#define HARDFOC_RETURN_IF_ERROR(result) \
    do { \
        if ((result).IsError()) { \
            return Result<void>((result).GetResult()); \
        } \
    } while(0)

#endif // COMPONENT_HANDLER_RESULT_H_
