# Logger Handler - Unified Logging System

<div align="center">

![Driver](https://img.shields.io/badge/driver-LoggerHandler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-Unified%20Logging-orange.svg)
![Interface](https://img.shields.io/badge/interface-UART%20%7C%20File%20%7C%20Network-green.svg)

**Unified logging system with multiple output destinations and configurable levels**

</div>

## 📋 Overview

The `Logger` is a comprehensive logging system that provides unified access to logging functionality across the HardFOC HAL platform. It supports multiple output destinations including UART, file system, and network interfaces, with configurable log levels and formatting options.

### ✨ Key Features

- **📝 Multi-Destination Logging**: UART, file, network, and custom outputs
- **🔧 Configurable Levels**: Debug, Info, Warning, Error, and Critical levels
- **📊 Performance Monitoring**: Log performance metrics and statistics
- **🛡️ Thread-Safe**: Concurrent logging from multiple tasks
- **⚡ High Performance**: Optimized logging with minimal overhead
- **🔍 Advanced Filtering**: Category-based and level-based filtering
- **📈 Log Rotation**: Automatic log file rotation and management
- **🎯 Memory Efficient**: Configurable buffer sizes and memory management

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Logger                                  │
├─────────────────────────────────────────────────────────────────┤
│  UART Output    │ Serial console logging                       │
├─────────────────────────────────────────────────────────────────┤
│  File Output    │ File system logging with rotation            │
├─────────────────────────────────────────────────────────────────┤
│  Network Output │ Network-based logging (UDP/TCP)              │
├─────────────────────────────────────────────────────────────────┤
│  Custom Output  │ User-defined logging destinations            │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Logging

```cpp
#include "utils-and-drivers/driver-handlers/Logger.h"

void logger_basic_example() {
    // Get logger instance
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Basic logging
    logger.Log(LogLevel::INFO, "System initialized successfully");
    logger.Log(LogLevel::WARNING, "Temperature sensor reading high: %.2f°C", 85.5f);
    logger.Log(LogLevel::ERROR, "Failed to initialize motor controller");
    
    // Category-based logging
    logger.Log(LogCategory::GPIO, LogLevel::DEBUG, "GPIO pin %s set to %s", "LED_1", "HIGH");
    logger.Log(LogCategory::MOTOR, LogLevel::INFO, "Motor velocity: %.2f RPM", 1500.0f);
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class Logger {
public:
    // Singleton access
    static Logger& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool Initialize() noexcept;
    void Deinitialize() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### Basic Logging
```cpp
// Simple logging
void Log(LogLevel level, const char* format, ...) noexcept;

void Log(LogCategory category, LogLevel level, const char* format, ...) noexcept;

// Convenience methods
void Debug(const char* format, ...) noexcept;
void Info(const char* format, ...) noexcept;
void Warning(const char* format, ...) noexcept;
void Error(const char* format, ...) noexcept;
void Critical(const char* format, ...) noexcept;

// Category-based convenience methods
void Debug(LogCategory category, const char* format, ...) noexcept;
void Info(LogCategory category, const char* format, ...) noexcept;
void Warning(LogCategory category, const char* format, ...) noexcept;
void Error(LogCategory category, const char* format, ...) noexcept;
void Critical(LogCategory category, const char* format, ...) noexcept;
```

#### Configuration Management
```cpp
// Logger configuration
bool Configure(const LoggerConfig& config) noexcept;

bool GetConfiguration(LoggerConfig& config) noexcept;

// Output destination management
bool AddOutputDestination(const LogOutputConfig& output_config) noexcept;

bool RemoveOutputDestination(const std::string& destination_id) noexcept;

bool EnableOutputDestination(const std::string& destination_id, bool enable) noexcept;

// Level and category filtering
bool SetGlobalLogLevel(LogLevel level) noexcept;

bool SetCategoryLogLevel(LogCategory category, LogLevel level) noexcept;

bool SetDestinationLogLevel(const std::string& destination_id, LogLevel level) noexcept;
```

#### Performance and Statistics
```cpp
// Performance monitoring
LoggerStatistics GetStatistics() const noexcept;

void ResetStatistics() noexcept;

bool IsPerformanceModeEnabled() const noexcept;

void SetPerformanceMode(bool enable) noexcept;

// Buffer management
bool FlushBuffers() noexcept;

bool SetBufferSize(size_t buffer_size) noexcept;

size_t GetBufferSize() const noexcept;
```

#### Advanced Features
```cpp
// Custom formatters
bool SetCustomFormatter(const std::string& destination_id, 
                       LogFormatter formatter) noexcept;

// Log rotation
bool EnableLogRotation(const std::string& destination_id, 
                      const LogRotationConfig& config) noexcept;

bool RotateLogs(const std::string& destination_id) noexcept;

// Timestamp formatting
bool SetTimestampFormat(const std::string& destination_id, 
                       const std::string& format) noexcept;

// Color output
bool EnableColorOutput(const std::string& destination_id, bool enable) noexcept;
```

## 🎯 Hardware Support

### Supported Output Destinations

#### UART Console Output
- **Interface**: UART via CommChannelsManager
- **Baud Rate**: Configurable (typically 115200)
- **Format**: Text with optional color codes
- **Features**: Real-time output, configurable encoding

#### File System Output
- **Interface**: ESP32 file system (SPIFFS/LittleFS)
- **Format**: Text files with timestamps
- **Features**: Log rotation, compression, archiving
- **Storage**: Configurable file size limits

#### Network Output
- **Interface**: WiFi/Ethernet via network stack
- **Protocols**: UDP, TCP, syslog
- **Features**: Remote logging, network buffering
- **Security**: Optional encryption and authentication

#### Custom Output
- **Interface**: User-defined callbacks
- **Format**: Custom data formats
- **Features**: Integration with external systems
- **Performance**: Minimal overhead callbacks

### Logger Configuration

```cpp
struct LoggerConfig {
    // Global settings
    LogLevel global_log_level;
    bool enable_timestamps;
    bool enable_thread_info;
    bool enable_file_line_info;
    
    // Performance settings
    bool enable_performance_mode;
    size_t buffer_size;
    uint32_t flush_interval_ms;
    
    // Output destinations
    std::vector<LogOutputConfig> output_destinations;
    
    // Formatting
    std::string timestamp_format;
    bool enable_color_output;
    bool enable_category_prefix;
    
    // File output settings
    std::string log_file_path;
    size_t max_file_size_bytes;
    uint32_t max_file_count;
    bool enable_compression;
};

struct LogOutputConfig {
    std::string destination_id;
    LogOutputType output_type;
    
    // UART specific
    std::string uart_device;
    uint32_t baud_rate;
    
    // File specific
    std::string file_path;
    bool append_mode;
    
    // Network specific
    std::string network_address;
    uint16_t network_port;
    LogNetworkProtocol network_protocol;
    
    // General settings
    LogLevel minimum_level;
    bool enable_timestamps;
    bool enable_color;
};
```

## 📊 Examples

### Basic Logging Setup

```cpp
void basic_logging_setup() {
    auto& logger = Logger::GetInstance();
    
    // Configure logger
    LoggerConfig config;
    config.global_log_level = LogLevel::INFO;
    config.enable_timestamps = true;
    config.enable_thread_info = true;
    config.enable_performance_mode = false;
    config.buffer_size = 4096;
    
    // Add UART output
    LogOutputConfig uart_output;
    uart_output.destination_id = "UART_CONSOLE";
    uart_output.output_type = LogOutputType::UART;
    uart_output.uart_device = "UART_0";
    uart_output.baud_rate = 115200;
    uart_output.minimum_level = LogLevel::DEBUG;
    uart_output.enable_timestamps = true;
    uart_output.enable_color = true;
    
    config.output_destinations.push_back(uart_output);
    
    // Add file output
    LogOutputConfig file_output;
    file_output.destination_id = "FILE_LOG";
    file_output.output_type = LogOutputType::FILE;
    file_output.file_path = "/spiffs/system.log";
    file_output.append_mode = true;
    file_output.minimum_level = LogLevel::WARNING;
    file_output.enable_timestamps = true;
    
    config.output_destinations.push_back(file_output);
    
    // Apply configuration
    if (logger.Configure(config)) {
        logger.Info("Logger configured successfully");
    }
}
```

### Category-Based Logging

```cpp
void category_logging_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Set different log levels for different categories
    logger.SetCategoryLogLevel(LogCategory::GPIO, LogLevel::DEBUG);
    logger.SetCategoryLogLevel(LogCategory::MOTOR, LogLevel::INFO);
    logger.SetCategoryLogLevel(LogCategory::SYSTEM, LogLevel::WARNING);
    
    // Log with categories
    logger.Debug(LogCategory::GPIO, "GPIO pin %s configured as output", "LED_1");
    logger.Info(LogCategory::GPIO, "GPIO pin %s set to %s", "LED_1", "HIGH");
    
    logger.Debug(LogCategory::MOTOR, "Motor controller initialized");
    logger.Info(LogCategory::MOTOR, "Motor velocity set to %.2f RPM", 1500.0f);
    logger.Warning(LogCategory::MOTOR, "Motor temperature: %.2f°C", 75.5f);
    
    logger.Info(LogCategory::SYSTEM, "System startup completed");
    logger.Warning(LogCategory::SYSTEM, "Low memory warning: %u bytes free", 1024);
    logger.Error(LogCategory::SYSTEM, "Critical system error detected");
}
```

### Performance Logging

```cpp
void performance_logging_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Enable performance mode
    logger.SetPerformanceMode(true);
    
    // Log performance metrics
    logger.Info(LogCategory::PERFORMANCE, "Task execution time: %.3f ms", 12.5f);
    logger.Info(LogCategory::PERFORMANCE, "Memory usage: %u bytes", 2048);
    logger.Info(LogCategory::PERFORMANCE, "CPU utilization: %.1f%%", 45.2f);
    
    // Get logging statistics
    auto stats = logger.GetStatistics();
    printf("Logging statistics:\n");
    printf("  Total messages: %u\n", stats.total_messages);
    printf("  Messages by level:\n");
    printf("    Debug: %u\n", stats.messages_by_level[static_cast<int>(LogLevel::DEBUG)]);
    printf("    Info: %u\n", stats.messages_by_level[static_cast<int>(LogLevel::INFO)]);
    printf("    Warning: %u\n", stats.messages_by_level[static_cast<int>(LogLevel::WARNING)]);
    printf("    Error: %u\n", stats.messages_by_level[static_cast<int>(LogLevel::ERROR)]);
    printf("    Critical: %u\n", stats.messages_by_level[static_cast<int>(LogLevel::CRITICAL)]);
    printf("  Average message size: %.2f bytes\n", stats.average_message_size);
    printf("  Buffer overflow count: %u\n", stats.buffer_overflows);
    printf("  Flush operations: %u\n", stats.flush_operations);
}
```

### Network Logging

```cpp
void network_logging_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Configure network logging
    LogOutputConfig network_output;
    network_output.destination_id = "NETWORK_LOG";
    network_output.output_type = LogOutputType::NETWORK;
    network_output.network_address = "192.168.1.100";
    network_output.network_port = 514;
    network_output.network_protocol = LogNetworkProtocol::UDP;
    network_output.minimum_level = LogLevel::WARNING;
    network_output.enable_timestamps = true;
    
    if (logger.AddOutputDestination(network_output)) {
        logger.Info("Network logging destination added");
        
        // Send logs to network
        logger.Warning("System temperature high: %.2f°C", 85.5f);
        logger.Error("Motor controller fault detected");
        logger.Critical("System shutdown initiated");
    }
}
```

### Log Rotation

```cpp
void log_rotation_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Configure log rotation
    LogRotationConfig rotation_config;
    rotation_config.max_file_size_bytes = 1024 * 1024;  // 1MB
    rotation_config.max_file_count = 5;
    rotation_config.enable_compression = true;
    rotation_config.rotation_interval_hours = 24;
    
    if (logger.EnableLogRotation("FILE_LOG", rotation_config)) {
        logger.Info("Log rotation enabled");
        
        // Generate some logs
        for (int i = 0; i < 1000; i++) {
            logger.Info("Log message %d: System status normal", i);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // Manually trigger rotation
        if (logger.RotateLogs("FILE_LOG")) {
            logger.Info("Log rotation completed");
        }
    }
}
```

### Custom Formatter

```cpp
void custom_formatter_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Define custom formatter
    auto custom_formatter = [](const LogMessage& message, std::string& formatted_output) {
        // Custom JSON format
        formatted_output = "{";
        formatted_output += "\"timestamp\":\"" + message.timestamp + "\",";
        formatted_output += "\"level\":\"" + LogLevelToString(message.level) + "\",";
        formatted_output += "\"category\":\"" + LogCategoryToString(message.category) + "\",";
        formatted_output += "\"message\":\"" + message.message + "\",";
        formatted_output += "\"thread\":\"" + message.thread_name + "\"";
        formatted_output += "}\n";
    };
    
    // Apply custom formatter to file output
    if (logger.SetCustomFormatter("FILE_LOG", custom_formatter)) {
        logger.Info("Custom formatter applied");
        
        // Log messages will now be in JSON format
        logger.Info(LogCategory::SYSTEM, "System status: %s", "OK");
        logger.Warning(LogCategory::MOTOR, "Motor temperature: %.2f°C", 75.5f);
    }
}
```

### Error Handling and Recovery

```cpp
void error_handling_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Log system errors with context
    try {
        // Simulate an error condition
        throw std::runtime_error("Motor controller communication failed");
    } catch (const std::exception& e) {
        logger.Error(LogCategory::SYSTEM, "Exception caught: %s", e.what());
        logger.Error(LogCategory::SYSTEM, "Stack trace: %s", "Function call stack here");
        logger.Critical(LogCategory::SYSTEM, "System entering safe mode");
    }
    
    // Log recovery actions
    logger.Info(LogCategory::SYSTEM, "Attempting system recovery...");
    logger.Info(LogCategory::SYSTEM, "Motor controller reset initiated");
    logger.Info(LogCategory::SYSTEM, "System recovery completed successfully");
    
    // Check logger health
    auto stats = logger.GetStatistics();
    if (stats.buffer_overflows > 0) {
        logger.Warning(LogCategory::SYSTEM, "Logger buffer overflows detected: %u", stats.buffer_overflows);
    }
    
    // Flush any pending logs
    if (logger.FlushBuffers()) {
        logger.Info(LogCategory::SYSTEM, "All pending logs flushed successfully");
    }
}
```

## 🔍 Advanced Usage

### Multi-Destination Logging

```cpp
void multi_destination_logging() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Configure multiple output destinations
    std::vector<LogOutputConfig> outputs;
    
    // UART for real-time debugging
    LogOutputConfig uart_output;
    uart_output.destination_id = "DEBUG_UART";
    uart_output.output_type = LogOutputType::UART;
    uart_output.uart_device = "UART_0";
    uart_output.baud_rate = 115200;
    uart_output.minimum_level = LogLevel::DEBUG;
    uart_output.enable_color = true;
    outputs.push_back(uart_output);
    
    // File for persistent storage
    LogOutputConfig file_output;
    file_output.destination_id = "PERSISTENT_FILE";
    file_output.output_type = LogOutputType::FILE;
    file_output.file_path = "/spiffs/system.log";
    file_output.append_mode = true;
    file_output.minimum_level = LogLevel::INFO;
    outputs.push_back(file_output);
    
    // Network for remote monitoring
    LogOutputConfig network_output;
    network_output.destination_id = "REMOTE_MONITOR";
    network_output.output_type = LogOutputType::NETWORK;
    network_output.network_address = "192.168.1.100";
    network_output.network_port = 514;
    network_output.network_protocol = LogNetworkProtocol::UDP;
    network_output.minimum_level = LogLevel::WARNING;
    outputs.push_back(network_output);
    
    // Add all destinations
    for (const auto& output : outputs) {
        logger.AddOutputDestination(output);
    }
    
    // Log to all destinations
    logger.Info("Multi-destination logging configured");
    logger.Warning("System warning sent to all destinations");
    logger.Error("Critical error logged to all destinations");
}
```

### Conditional Logging

```cpp
void conditional_logging_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Enable/disable logging based on conditions
    bool debug_mode = true;
    bool production_mode = false;
    
    if (debug_mode) {
        logger.SetGlobalLogLevel(LogLevel::DEBUG);
        logger.Info("Debug mode enabled - verbose logging active");
    } else if (production_mode) {
        logger.SetGlobalLogLevel(LogLevel::WARNING);
        logger.Info("Production mode - minimal logging active");
    }
    
    // Conditional logging with macros
    #define LOG_DEBUG(fmt, ...) \
        if (debug_mode) logger.Debug(fmt, ##__VA_ARGS__)
    
    #define LOG_PERFORMANCE(fmt, ...) \
        logger.Info(LogCategory::PERFORMANCE, fmt, ##__VA_ARGS__)
    
    LOG_DEBUG("Debug information: %s", "Detailed debug data");
    LOG_PERFORMANCE("Performance metric: %.3f ms", 12.5f);
    
    // Log based on system state
    float temperature = 75.5f;
    if (temperature > 80.0f) {
        logger.Critical(LogCategory::SYSTEM, "Critical temperature: %.2f°C", temperature);
    } else if (temperature > 70.0f) {
        logger.Warning(LogCategory::SYSTEM, "High temperature: %.2f°C", temperature);
    } else {
        logger.Debug(LogCategory::SYSTEM, "Normal temperature: %.2f°C", temperature);
    }
}
```

### Performance Optimization

```cpp
void performance_optimization_example() {
    auto& logger = Logger::GetInstance();
    logger.EnsureInitialized();
    
    // Enable performance mode for high-frequency logging
    logger.SetPerformanceMode(true);
    logger.SetBufferSize(8192);  // Larger buffer for better performance
    
    // Batch logging operations
    const int batch_size = 100;
    for (int i = 0; i < batch_size; i++) {
        logger.Debug("Batch log message %d", i);
    }
    
    // Flush buffer after batch operations
    logger.FlushBuffers();
    
    // Monitor logging performance
    auto stats = logger.GetStatistics();
    printf("Logging performance:\n");
    printf("  Messages per second: %.2f\n", stats.messages_per_second);
    printf("  Average message size: %.2f bytes\n", stats.average_message_size);
    printf("  Buffer utilization: %.1f%%\n", stats.buffer_utilization_percent);
    printf("  Flush frequency: %.2f Hz\n", stats.flush_frequency_hz);
    
    // Optimize based on performance data
    if (stats.buffer_overflows > 0) {
        logger.SetBufferSize(stats.buffer_size * 2);
        logger.Info("Buffer size increased due to overflows");
    }
}
```

## 📚 See Also

- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - UART and network communication
- **[Performance Optimization Guide](../development/PERFORMANCE_OPTIMIZATION_GUIDE.md)** - Logging performance optimization
- **[Coding Standards](../development/CODING_STANDARDS.md)** - Logging standards and conventions
- **[Testing Requirements](../development/TESTING_REQUIREMENTS.md)** - Logging test requirements

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*