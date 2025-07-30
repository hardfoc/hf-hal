# LedManager - LED Control System

<div align="center">

![Component](https://img.shields.io/badge/component-LedManager-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-WS2812%20LEDs-orange.svg)
![Interface](https://img.shields.io/badge/interface-RMT%20%7C%20GPIO-green.svg)

**Unified LED control system with WS2812 RGB LED support**

</div>

## 📋 Overview

The `LedManager` is a comprehensive LED control system that provides unified access to LED devices across the HardFOC HAL platform. It supports WS2812 RGB LED strips, individual LEDs, and provides advanced features like animations, color management, and brightness control.

### ✨ Key Features

- **🎨 WS2812 Support**: Full RGB LED strip control with RMT interface
- **💡 Individual LED Control**: GPIO-based LED control
- **🌈 Color Management**: RGB, HSV, and temperature-based color control
- **🎭 Animation System**: Built-in animations and custom pattern support
- **⚡ High Performance**: Hardware-accelerated LED control
- **🛡️ Safety Features**: Current limiting and thermal protection
- **🔧 Configuration**: Flexible LED strip and individual LED configuration
- **📊 Status Monitoring**: LED health and performance monitoring

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      LedManager                                │
├─────────────────────────────────────────────────────────────────┤
│  WS2812 LED Strips │ RMT-based RGB LED control                │
├─────────────────────────────────────────────────────────────────┤
│  Individual LEDs   │ GPIO-based LED control                    │
├─────────────────────────────────────────────────────────────────┤
│  Animation Engine  │ Built-in and custom animation patterns    │
├─────────────────────────────────────────────────────────────────┤
│  Color Management  │ RGB, HSV, and temperature color support   │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic LED Control

```cpp
#include "component-handlers/LedManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

void led_basic_example() {
    auto& logger = Logger::GetInstance();
    
    // Get LED manager instance
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Set LED color using RGB values
    LedColor red_color(255, 0, 0);
    if (led_manager.SetColor(red_color) == LedError::SUCCESS) {
        logger.Info("LED", "Set LED to red");
    }
    
    // Set LED color using packed RGB value
    if (led_manager.SetColor(0x00FF00) == LedError::SUCCESS) {  // Green
        logger.Info("LED", "Set LED to green");
    }
    
    // Set brightness
    if (led_manager.SetBrightnessPercent(50) == LedError::SUCCESS) {
        logger.Info("LED", "Set brightness to 50%%");
    }
    
    // Start animation
    if (led_manager.StartAnimation(LedAnimation::RAINBOW) == LedError::SUCCESS) {
        logger.Info("LED", "Started rainbow animation");
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class LedManager {
public:
    // Singleton access
    static LedManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool Initialize() noexcept;
    void Deinitialize() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### WS2812 LED Strip Control
```cpp
// LED strip management
LedError RegisterStrip(const std::string& strip_id, 
                      const LedStripConfig& config) noexcept;

LedError UnregisterStrip(const std::string& strip_id) noexcept;

// Individual LED control
LedError SetLedColor(const std::string& strip_id, uint16_t led_index, 
                    uint8_t red, uint8_t green, uint8_t blue) noexcept;

LedError SetLedColorHSV(const std::string& strip_id, uint16_t led_index, 
                       float hue, float saturation, float value) noexcept;

LedError SetLedBrightness(const std::string& strip_id, uint16_t led_index, 
                         uint8_t brightness) noexcept;

// Strip operations
LedError UpdateStrip(const std::string& strip_id) noexcept;
LedError ClearStrip(const std::string& strip_id) noexcept;
LedError SetStripBrightness(const std::string& strip_id, uint8_t brightness) noexcept;
```

#### Individual LED Control
```cpp
// GPIO-based LED control
LedError RegisterLed(const std::string& led_id, 
                    const LedConfig& config) noexcept;

LedError UnregisterLed(const std::string& led_id) noexcept;

LedError SetLed(const std::string& led_id, bool state) noexcept;
LedError ToggleLed(const std::string& led_id) noexcept;
LedError SetLedBrightness(const std::string& led_id, uint8_t brightness) noexcept;
```

#### Animation System
```cpp
// Animation control
LedError StartAnimation(const std::string& strip_id, 
                       const LedAnimation& animation) noexcept;

LedError StopAnimation(const std::string& strip_id) noexcept;

LedError SetAnimationSpeed(const std::string& strip_id, float speed) noexcept;

// Built-in animations
LedError StartRainbowAnimation(const std::string& strip_id, float speed = 1.0f) noexcept;
LedError StartBreathingAnimation(const std::string& strip_id, 
                                uint8_t red, uint8_t green, uint8_t blue, 
                                float speed = 1.0f) noexcept;
LedError StartChaseAnimation(const std::string& strip_id, 
                            uint8_t red, uint8_t green, uint8_t blue, 
                            float speed = 1.0f) noexcept;
```

#### Color Management
```cpp
// Color conversion utilities
static void RGBToHSV(uint8_t red, uint8_t green, uint8_t blue, 
                    float& hue, float& saturation, float& value) noexcept;

static void HSVToRGB(float hue, float saturation, float value, 
                    uint8_t& red, uint8_t& green, uint8_t& blue) noexcept;

static void TemperatureToRGB(float temperature_kelvin, 
                           uint8_t& red, uint8_t& green, uint8_t& blue) noexcept;

// Color presets
static const LedColor& GetColorPreset(LedColorPreset preset) noexcept;
```

#### System Management
```cpp
// Device management
std::vector<std::string> GetAvailableStrips() const noexcept;
std::vector<std::string> GetAvailableLeds() const noexcept;

bool IsStripAvailable(const std::string& strip_id) const noexcept;
bool IsLedAvailable(const std::string& led_id) const noexcept;

// Configuration
LedError UpdateStripConfig(const std::string& strip_id, 
                          const LedStripConfig& config) noexcept;

LedError GetStripConfig(const std::string& strip_id, 
                       LedStripConfig& config) noexcept;

// System status
LedSystemStatus GetSystemStatus() const noexcept;
bool IsSystemHealthy() const noexcept;

// Performance statistics
LedStatistics GetStatistics() const noexcept;
void ResetStatistics() noexcept;
```

## 🎯 Hardware Support

### WS2812 LED Strips

- **Interface**: RMT (Remote Control Transceiver) hardware
- **Protocol**: WS2812B protocol support
- **Data Rate**: 800 KHz
- **Color Depth**: 24-bit RGB (8-bit per channel)
- **Maximum LEDs**: Configurable (typically 1000+ LEDs per strip)
- **Power Management**: Current limiting and thermal protection

### Individual LEDs

- **Interface**: GPIO pins via GpioManager
- **Control**: Digital on/off with optional PWM dimming
- **Types**: Standard LEDs, RGB LEDs, status indicators
- **Current Limiting**: Built-in current limiting resistors

### LED Configuration

```cpp
struct LedStripConfig {
    std::string strip_id;
    std::string rmt_channel;      // RMT channel identifier
    uint16_t led_count;           // Number of LEDs in strip
    uint8_t default_brightness;   // Default brightness (0-255)
    bool enable_power_management; // Enable power management
    float max_current_ma;         // Maximum current per LED
    bool enable_thermal_protection; // Enable thermal protection
};

struct LedConfig {
    std::string led_id;
    std::string gpio_pin;         // GPIO pin identifier
    bool active_high;             // LED active high/low
    uint8_t default_brightness;   // Default brightness (0-255)
    bool enable_pwm;              // Enable PWM dimming
    uint32_t pwm_frequency_hz;    // PWM frequency
};
```

## 📊 Examples

### Basic WS2812 Control

```cpp
void basic_ws2812_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Configure WS2812 strip
    LedStripConfig config;
    config.strip_id = "MAIN_STRIP";
    config.rmt_channel = "RMT_CH0";
    config.led_count = 60;
    config.default_brightness = 128;
    config.enable_power_management = true;
    config.max_current_ma = 20.0f;
    
    if (led_manager.RegisterStrip("MAIN_STRIP", config) == LedError::SUCCESS) {
        logger.Info("LED", "WS2812 strip registered\n");
    }
    
    // Set individual LED colors
    for (int i = 0; i < 10; i++) {
        led_manager.SetLedColor("MAIN_STRIP", i, 255, 0, 0);      // Red
        led_manager.SetLedColor("MAIN_STRIP", i+10, 0, 255, 0);   // Green
        led_manager.SetLedColor("MAIN_STRIP", i+20, 0, 0, 255);   // Blue
        led_manager.SetLedColor("MAIN_STRIP", i+30, 255, 255, 0); // Yellow
        led_manager.SetLedColor("MAIN_STRIP", i+40, 255, 0, 255); // Magenta
        led_manager.SetLedColor("MAIN_STRIP", i+50, 0, 255, 255); // Cyan
    }
    
    // Update the strip
    if (led_manager.UpdateStrip("MAIN_STRIP") == LedError::SUCCESS) {
        logger.Info("LED", "LED strip updated with rainbow pattern\n");
    }
}
```

### Advanced Color Control

```cpp
void advanced_color_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // HSV color control
    for (int i = 0; i < 60; i++) {
        float hue = (i * 360.0f) / 60.0f;  // Full hue spectrum
        float saturation = 1.0f;            // Full saturation
        float value = 0.5f;                 // 50% brightness
        
        led_manager.SetLedColorHSV("MAIN_STRIP", i, hue, saturation, value);
    }
    
    led_manager.UpdateStrip("MAIN_STRIP");
    
    // Temperature-based colors
    for (int i = 0; i < 60; i++) {
        float temp_kelvin = 2000.0f + (i * 5000.0f) / 60.0f;  // 2000K to 7000K
        uint8_t red, green, blue;
        LedManager::TemperatureToRGB(temp_kelvin, red, green, blue);
        led_manager.SetLedColor("MAIN_STRIP", i, red, green, blue);
    }
    
    led_manager.UpdateStrip("MAIN_STRIP");
}
```

### Animation Examples

```cpp
void animation_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Start rainbow animation
    if (led_manager.StartRainbowAnimation("MAIN_STRIP", 2.0f) == LedError::SUCCESS) {
        logger.Info("LED", "Rainbow animation started\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000));  // Run for 5 seconds
    
    // Stop animation
    led_manager.StopAnimation("MAIN_STRIP");
    
    // Start breathing animation
    if (led_manager.StartBreathingAnimation("MAIN_STRIP", 255, 0, 0, 1.0f) == LedError::SUCCESS) {
        logger.Info("LED", "Red breathing animation started\n");
    }
    
    vTaskDelay(pdMS_TO_TICKS(3000));  // Run for 3 seconds
    
    // Stop animation
    led_manager.StopAnimation("MAIN_STRIP");
    
    // Start chase animation
    if (led_manager.StartChaseAnimation("MAIN_STRIP", 0, 255, 0, 0.5f) == LedError::SUCCESS) {
        logger.Info("LED", "Green chase animation started\n");
    }
}
```

### Individual LED Control

```cpp
void individual_led_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Configure status LED
    LedConfig config;
    config.led_id = "STATUS_LED";
    config.gpio_pin = "ESP32_GPIO_2";
    config.active_high = true;
    config.default_brightness = 255;
    config.enable_pwm = true;
    config.pwm_frequency_hz = 1000;
    
    if (led_manager.RegisterLed("STATUS_LED", config) == LedError::SUCCESS) {
        logger.Info("LED", "Status LED registered\n");
    }
    
    // Control status LED
    led_manager.SetLed("STATUS_LED", true);   // Turn on
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    led_manager.SetLed("STATUS_LED", false);  // Turn off
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Toggle LED
    for (int i = 0; i < 5; i++) {
        led_manager.ToggleLed("STATUS_LED");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Set brightness
    led_manager.SetLedBrightness("STATUS_LED", 128);  // 50% brightness
}
```

### Custom Animation Pattern

```cpp
void custom_animation_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Create custom animation
    LedAnimation animation;
    animation.type = LedAnimationType::CUSTOM;
    animation.duration_ms = 5000;  // 5 seconds
    animation.repeat_count = 3;     // Repeat 3 times
    
    // Define animation frames
    for (int frame = 0; frame < 60; frame++) {
        LedAnimationFrame frame_data;
        frame_data.delay_ms = 83;  // ~12 FPS
        
        // Create wave pattern
        for (int led = 0; led < 60; led++) {
            float wave = sin((led + frame) * 0.2f) * 0.5f + 0.5f;
            uint8_t intensity = static_cast<uint8_t>(wave * 255);
            frame_data.led_colors[led] = {intensity, 0, intensity};  // Purple wave
        }
        
        animation.frames.push_back(frame_data);
    }
    
    // Start custom animation
    if (led_manager.StartAnimation("MAIN_STRIP", animation) == LedError::SUCCESS) {
        logger.Info("LED", "Custom wave animation started\n");
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Get system status
    auto status = led_manager.GetSystemStatus();
    logger.Info("LED", "LED system status:\n");
    logger.Info("LED", "  Overall healthy: %s\n", status.overall_healthy ? "Yes" : "No");
    logger.Info("LED", "  Active strips: %u\n", status.active_strips);
    logger.Info("LED", "  Active LEDs: %u\n", status.active_leds);
    logger.Info("LED", "  Animations running: %u\n", status.animations_running);
    logger.Info("LED", "  Power management active: %s\n", status.power_management_active ? "Yes" : "No");
    
    // List available devices
    logger.Info("LED", "Available LED strips:\n");
    for (const auto& strip_id : led_manager.GetAvailableStrips()) {
        logger.Info("LED", "  - %s\n", strip_id.c_str());
    }
    
    logger.Info("LED", "Available individual LEDs:\n");
    for (const auto& led_id : led_manager.GetAvailableLeds()) {
        logger.Info("LED", "  - %s\n", led_id.c_str());
    }
    
    // Get performance statistics
    auto stats = led_manager.GetStatistics();
    logger.Info("LED", "Performance statistics:\n");
    logger.Info("LED", "  Total updates: %u\n", stats.total_updates);
    logger.Info("LED", "  Successful updates: %u\n", stats.successful_updates);
    logger.Info("LED", "  Failed updates: %u\n", stats.failed_updates);
    logger.Info("LED", "  Average update time: %.2f ms\n", stats.average_update_time_ms);
    logger.Info("LED", "  Peak update time: %.2f ms\n", stats.peak_update_time_ms);
}
```

## 🔍 Advanced Usage

### Power Management

```cpp
void power_management_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Configure strip with power management
    LedStripConfig config;
    config.strip_id = "POWER_MANAGED_STRIP";
    config.rmt_channel = "RMT_CH1";
    config.led_count = 30;
    config.enable_power_management = true;
    config.max_current_ma = 15.0f;  // 15mA per LED max
    
    if (led_manager.RegisterStrip("POWER_MANAGED_STRIP", config) == LedError::SUCCESS) {
        logger.Info("LED", "Power-managed strip registered\n");
    }
    
    // Set brightness based on available power
    led_manager.SetStripBrightness("POWER_MANAGED_STRIP", 128);  // 50% brightness
}
```

### Multiple Strip Management

```cpp
void multiple_strips_example() {
    auto& led_manager = LedManager::GetInstance();
    led_manager.EnsureInitialized();
    
    // Register multiple strips
    std::vector<std::string> strip_ids = {"STRIP_1", "STRIP_2", "STRIP_3"};
    
    for (size_t i = 0; i < strip_ids.size(); i++) {
        LedStripConfig config;
        config.strip_id = strip_ids[i];
        config.rmt_channel = "RMT_CH" + std::to_string(i);
        config.led_count = 20;
        config.default_brightness = 100 + (i * 50);
        
        led_manager.RegisterStrip(strip_ids[i], config);
    }
    
    // Control all strips simultaneously
    for (const auto& strip_id : strip_ids) {
        // Set different patterns for each strip
        for (int led = 0; led < 20; led++) {
            uint8_t red = (led * 12) % 256;
            uint8_t green = (led * 8) % 256;
            uint8_t blue = (led * 16) % 256;
            led_manager.SetLedColor(strip_id, led, red, green, blue);
        }
        led_manager.UpdateStrip(strip_id);
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO system for individual LED control
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - RMT interface management
- **[WS2812 RMT Driver Documentation](../driver-handlers/WS2812_RMT_HANDLER_README.md)** - WS2812 LED driver
- **[Performance Optimization Guide](../development/PERFORMANCE_OPTIMIZATION_GUIDE.md)** - LED performance optimization

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*
