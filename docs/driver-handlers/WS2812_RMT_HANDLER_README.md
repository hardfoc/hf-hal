# WS2812 RMT Driver - RGB LED Strip Driver

<div align="center">

![Driver](https://img.shields.io/badge/driver-WS2812Strip-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-WS2812%20LEDs-orange.svg)
![Interface](https://img.shields.io/badge/interface-RMT-green.svg)

**Hardware-accelerated WS2812 RGB LED strip control with RMT interface**

</div>

## 📋 Overview

The WS2812 RMT driver provides high-performance control for WS2812 RGB LED strips using the ESP32's RMT (Remote Control Transceiver) hardware for precise timing control. It includes the `WS2812Strip` class for basic LED control and `WS2812Animator` for animation effects.

### ✨ Key Features

- **⚡ Hardware Acceleration**: RMT-based timing for precise WS2812 protocol
- **🎨 Full Color Support**: 24-bit RGB color control (8-bit per channel)
- **📊 Multi-Strip Support**: Multiple LED strips on different RMT channels
- **🌈 Color Management**: RGB color wheel and brightness control
- **🎭 Animation Engine**: Built-in animations via WS2812Animator
- **🛡️ Power Management**: Global brightness control
- **🔧 Flexible Configuration**: Configurable strip length and timing
- **📈 Performance**: Efficient DMA-based LED updates

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   WS2812Strip                                  │
├─────────────────────────────────────────────────────────────────┤
│  RMT Hardware │ ESP32 RMT channels for precise timing         │
├─────────────────────────────────────────────────────────────────┤
│  DMA Transfer │ Direct memory access for high performance     │
├─────────────────────────────────────────────────────────────────┤
│  Color Engine │ RGB color wheel and brightness control        │
├─────────────────────────────────────────────────────────────────┤
│  Animation System │ WS2812Animator for built-in effects       │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic LED Strip Control

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_effects.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void ws2812_basic_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip with default configuration
    WS2812Strip strip;
    
    // Initialize the strip
    if (strip.begin() == ESP_OK) {
        logger.Info("WS2812", "WS2812 strip initialized");
    }
    
    // Set LED colors using the actual API
    strip.setPixel(0, 0xFF0000);      // Red
    strip.setPixel(1, 0x00FF00);      // Green
    strip.setPixel(2, 0x0000FF);      // Blue
    
    // Update the strip
    if (strip.show() == ESP_OK) {
        logger.Info("WS2812", "LED strip updated");
    }
    
    // Use color wheel for rainbow effect
    for (int i = 0; i < strip.length(); i++) {
        strip.setPixel(i, WS2812Strip::colorWheel(i * 256 / strip.length()));
    }
    strip.show();
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class WS2812Strip {
public:
    // Constructor with default configuration
    explicit WS2812Strip(gpio_num_t gpio = CONFIG_WS2812_LED_RMT_TX_GPIO,
                        int channel = CONFIG_WS2812_LED_RMT_TX_CHANNEL, 
                        uint32_t numLeds = NUM_LEDS,
                        LedType type = LedType::RGB,
                        uint16_t t0h = WS2812_T0H, uint16_t t1h = WS2812_T1H,
                        uint16_t t0l = WS2812_T0L, uint16_t t1l = WS2812_T1L,
                        uint8_t brightness = WS2812_DEFAULT_BRIGHTNESS);
    
    // Initialization
    esp_err_t begin();
};
```

#### LED Control
```cpp
// Individual LED control
void setPixel(uint32_t index, uint32_t rgbw);

// Strip operations
esp_err_t show();
uint32_t length() const;
void setBrightness(uint8_t value);

// Color wheel utility
static uint32_t colorWheel(uint8_t pos);

// Timing configuration
void setTimings(uint16_t t0h, uint16_t t1h, uint16_t t0l, uint16_t t1l);
```

#### Animation System
```cpp
class WS2812Animator {
public:
    enum class Effect { Off, SolidColor, Rainbow, Chase, Blink, Breath, Larson };
    
    // Constructor
    explicit WS2812Animator(WS2812Strip &strip, uint32_t virtualLength = 0);
    
    // Animation control
    void setEffect(Effect effect, uint32_t color = 0xFFFFFF);
    void setVirtualLength(uint32_t length);
    void setStep(uint16_t step);
    uint16_t step() const;
    void tick();  // Advance animation by one step
};
```

#### Color Management
```cpp
// Color wheel utility
static uint32_t colorWheel(uint8_t pos);

// Color constants
#define RED     0xFF0000
#define GREEN   0x00FF00
#define BLUE    0x0000FF
#define WHITE   0xFFFFFF
#define BLACK   0x000000
```

#### Configuration and Status
```cpp
// LED type enumeration
enum class LedType { RGB, RGBW };

// Strip information
uint32_t length() const;  // Get number of LEDs
```

## 🎯 Hardware Support

### WS2812 LED Specifications

- **Protocol**: WS2812B protocol
- **Data Rate**: 800 KHz
- **Color Depth**: 24-bit RGB (8-bit per channel)
- **Maximum LEDs**: Configurable (typically 1000+ LEDs per strip)
- **Power Requirements**: 5V supply, 60mA per LED at full brightness
- **Timing Requirements**: Precise timing for reliable communication

### RMT Hardware Features

- **Channels**: 8 RMT channels available
- **DMA Support**: Direct memory access for high performance
- **Precise Timing**: Hardware-generated timing signals
- **Interrupt Support**: Completion interrupts for synchronization
- **Memory Efficient**: Optimized memory usage for large strips

### Configuration Structure

```cpp
struct WS2812Config {
    rmt_channel_t rmt_channel;     // RMT channel (0-7)
    gpio_num_t gpio_pin;           // GPIO pin for data output
    uint16_t led_count;            // Number of LEDs in strip
    uint8_t brightness;            // Global brightness (0-255)
    bool enable_power_management;  // Enable power management
    float max_current_ma;          // Maximum current per LED
    bool enable_thermal_protection; // Enable thermal protection
    uint32_t update_rate_hz;       // Maximum update rate
};
```

## 📊 Examples

### Basic LED Strip Setup

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void basic_ws2812_setup() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip with custom configuration
    WS2812Strip strip(GPIO_NUM_2, RMT_CHANNEL_0, 60, LedType::RGB);
    
    // Initialize the strip
    if (strip.begin() == ESP_OK) {
        logger.Info("WS2812", "WS2812 strip initialized");
    }
    
    // Set rainbow pattern using color wheel
    for (int i = 0; i < strip.length(); i++) {
        strip.setPixel(i, WS2812Strip::colorWheel(i * 256 / strip.length()));
    }
    
    strip.show();
}
```

### Advanced Color Control

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void advanced_color_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip
    WS2812Strip strip(GPIO_NUM_4, RMT_CHANNEL_1, 30, LedType::RGB);
    strip.begin();
    
    // Set brightness
    strip.setBrightness(128);  // 50% brightness
    
    // Create gradient pattern
    for (int i = 0; i < strip.length(); i++) {
        // Create a gradient from red to blue
        uint8_t red = 255 - (i * 255 / strip.length());
        uint8_t blue = i * 255 / strip.length();
        uint32_t color = (red << 16) | blue;
        strip.setPixel(i, color);
    }
    
    strip.show();
    
    // Use color wheel for smooth transitions
    for (int i = 0; i < strip.length(); i++) {
        strip.setPixel(i, WS2812Strip::colorWheel(i * 8));  // Faster color change
    }
    
    strip.show();
}
```

### Animation Examples

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_effects.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void animation_examples() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip and animator
    WS2812Strip strip(GPIO_NUM_5, RMT_CHANNEL_2, 50, LedType::RGB);
    strip.begin();
    strip.setBrightness(100);
    
    WS2812Animator animator(strip, 50);
    
    // Start rainbow animation
    animator.setEffect(WS2812Animator::Effect::Rainbow);
    logger.Info("WS2812", "Rainbow animation started");
    
    // Animation loop for 5 seconds
    for (int i = 0; i < 100; i++) {  // 100 * 50ms = 5 seconds
        animator.tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Switch to breathing animation
    animator.setEffect(WS2812Animator::Effect::Breath, 0xFF0000);  // Red breathing
    logger.Info("WS2812", "Red breathing animation started");
    
    // Animation loop for 3 seconds
    for (int i = 0; i < 60; i++) {  // 60 * 50ms = 3 seconds
        animator.tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Switch to chase animation
    animator.setEffect(WS2812Animator::Effect::Chase, 0x00FF00);  // Green chase
    logger.Info("WS2812", "Green chase animation started");
    
    // Animation loop
    for (int i = 0; i < 100; i++) {
        animator.tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Turn off
    animator.setEffect(WS2812Animator::Effect::Off);
    animator.tick();
}
```

### Multi-Strip Management

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void multi_strip_example() {
    auto& logger = Logger::GetInstance();
    
    // Create multiple WS2812 strips
    std::vector<std::unique_ptr<WS2812Strip>> strips;
    
    // Configure multiple strips
    std::vector<std::tuple<gpio_num_t, int, uint32_t>> configs = {
        {GPIO_NUM_2, RMT_CHANNEL_0, 30},
        {GPIO_NUM_4, RMT_CHANNEL_1, 30},
        {GPIO_NUM_5, RMT_CHANNEL_2, 30}
    };
    
    // Initialize all strips
    for (size_t i = 0; i < configs.size(); i++) {
        auto [gpio, channel, num_leds] = configs[i];
        auto strip = std::make_unique<WS2812Strip>(gpio, channel, num_leds, LedType::RGB);
        if (strip->begin() == ESP_OK) {
            strips.push_back(std::move(strip));
            logger.Info("WS2812", "Strip %zu initialized", i);
        }
    }
    
    // Control all strips simultaneously
    for (size_t strip_idx = 0; strip_idx < strips.size(); strip_idx++) {
        for (int led = 0; led < 30; led++) {
            uint8_t red = (led * 8) % 256;
            uint8_t green = (led * 6) % 256;
            uint8_t blue = (led * 10) % 256;
            uint32_t color = (red << 16) | (green << 8) | blue;
            strips[strip_idx]->setPixel(led, color);
        }
        strips[strip_idx]->show();
    }
}
```

### Performance Optimization

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void performance_optimization_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip with optimized settings
    WS2812Strip strip(GPIO_NUM_18, RMT_CHANNEL_3, 100, LedType::RGB);
    strip.begin();
    strip.setBrightness(64);  // Lower brightness for better performance
    
    // Efficient LED updates
    for (int i = 0; i < strip.length(); i++) {
        uint8_t red = (i * 2) % 256;
        uint8_t green = (i * 3) % 256;
        uint8_t blue = (i * 5) % 256;
        uint32_t color = (red << 16) | (green << 8) | blue;
        strip.setPixel(i, color);
    }
    
    // Update all LEDs at once
    strip.show();
    
    logger.Info("WS2812", "Performance optimized strip updated");
}
```

### Power Management

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void power_management_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip
    WS2812Strip strip(GPIO_NUM_19, RMT_CHANNEL_4, 60, LedType::RGB);
    strip.begin();
    
    // Set lower brightness for power saving
    strip.setBrightness(64);  // 25% brightness for power saving
    
    // Create power-efficient pattern
    for (int i = 0; i < strip.length(); i++) {
        if (i % 3 == 0) {  // Every 3rd LED on
            strip.setPixel(i, 0xFFFFFF);  // White
        } else {
            strip.setPixel(i, 0x000000);  // Off
        }
    }
    
    strip.show();
    
    logger.Info("WS2812", "Power-efficient pattern applied");
}
```

### Custom Animation Pattern

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_effects.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"
#include <cmath>

void custom_animation_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip and animator
    WS2812Strip strip(GPIO_NUM_21, RMT_CHANNEL_5, 40, LedType::RGB);
    strip.begin();
    strip.setBrightness(200);
    
    WS2812Animator animator(strip, 40);
    
    // Create custom wave animation
    for (int frame = 0; frame < 60; frame++) {
        // Create wave pattern
        for (int led = 0; led < strip.length(); led++) {
            float wave = sin((led + frame) * 0.3f) * 0.5f + 0.5f;
            uint8_t intensity = static_cast<uint8_t>(wave * 255);
            uint32_t color = (intensity << 16) | intensity;  // Purple wave
            strip.setPixel(led, color);
        }
        
        strip.show();
        vTaskDelay(pdMS_TO_TICKS(50));  // 20 FPS
    }
    
    logger.Info("WS2812", "Custom wave animation completed");
}
```

### System Diagnostics

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void diagnostics_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip
    WS2812Strip strip(GPIO_NUM_22, RMT_CHANNEL_6, 50, LedType::RGB);
    
    // Initialize and check status
    esp_err_t result = strip.begin();
    if (result == ESP_OK) {
        logger.Info("WS2812", "WS2812 strip initialized successfully");
        logger.Info("WS2812", "Strip length: %u LEDs", strip.length());
    } else {
        logger.Error("WS2812", "Failed to initialize WS2812 strip: %s", esp_err_to_name(result));
    }
    
    // Test basic functionality
    strip.setBrightness(150);
    strip.setPixel(0, 0xFF0000);  // Red
    strip.setPixel(1, 0x00FF00);  // Green
    strip.setPixel(2, 0x0000FF);  // Blue
    
    result = strip.show();
    if (result == ESP_OK) {
        logger.Info("WS2812", "LED strip updated successfully");
    } else {
        logger.Error("WS2812", "Failed to update LED strip: %s", esp_err_to_name(result));
    }
}
```

## 🔍 Advanced Usage

### RMT Channel Management

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void rmt_channel_management() {
    auto& logger = Logger::GetInstance();
    
    // Check available RMT channels
    logger.Info("WS2812", "Available RMT channels:");
    for (int i = 0; i < 8; i++) {
        bool available = true;  // Check if channel is available
        logger.Info("WS2812", "  RMT_CHANNEL_%d: %s", i, available ? "Available" : "In Use");
    }
    
    // Use specific RMT channel
    WS2812Strip strip(GPIO_NUM_23, RMT_CHANNEL_7, 25, LedType::RGB);
    strip.begin();
}
```

### Timing Optimization

```cpp
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"

void timing_optimization_example() {
    auto& logger = Logger::GetInstance();
    
    // Create WS2812 strip
    WS2812Strip strip(GPIO_NUM_2, RMT_CHANNEL_0, 100, LedType::RGB);
    strip.begin();
    
    // High-frequency animation
    for (int i = 0; i < 1000; i++) {
        // Update LED colors rapidly
        for (int led = 0; led < strip.length(); led++) {
            uint8_t red = (led + i) % 256;
            uint8_t green = (led + i * 2) % 256;
            uint8_t blue = (led + i * 3) % 256;
            uint32_t color = (red << 16) | (green << 8) | blue;
            strip.setPixel(led, color);
        }
        
        strip.show();
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms delay for 1kHz animation
    }
    
    logger.Info("WS2812", "High-frequency animation completed");
}
```

## 📚 See Also

- **[LedManager Documentation](../component-handlers/LED_MANAGER_README.md)** - High-level LED management system
- **[CommChannelsManager Documentation](../component-handlers/COMM_CHANNELS_MANAGER_README.md)** - RMT interface management
- **[Performance Optimization Guide](../development/PERFORMANCE_OPTIMIZATION_GUIDE.md)** - LED performance optimization
- **[Architecture Guidelines](../development/ARCHITECTURE_GUIDELINES.md)** - Hardware abstraction architecture

---

*This documentation covers the WS2812 RMT driver from the hf-ws2812-rmt-driver package. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*
 