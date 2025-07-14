/**
 * @file ModernHalExample.cpp
 * @brief Example demonstrating the modernized HardFOC HAL usage.
 * 
 * This example shows how to use the new consolidated manager classes
 * with modern error handling and Result<T> types.
 * 
 * @author HardFOC Team
 * @version 2.0
 * @date 2024
 */

#include "All.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ModernHalExample";

/**
 * @brief Example function demonstrating basic GPIO operations.
 */
void example_gpio_operations() {
    ESP_LOGI(TAG, "=== GPIO Operations Example ===");
    
    // Get the GPIO manager instance
    auto& gpio = HardFocComponentHandler::GetGpio();
    
    // Basic pin operations with error handling
    auto result = gpio.SetActive(GPIO_LED_STATUS_OK);
    if (result.IsSuccess()) {
        ESP_LOGI(TAG, "LED Status OK set to active");
    } else {
        ESP_LOGE(TAG, "Failed to set LED: %s", 
                 GetResultDescription(result.GetResult()).data());
    }
    
    // Read pin state
    auto stateResult = gpio.ReadState(GPIO_USER_INPUT_1);
    if (stateResult.IsSuccess()) {
        ESP_LOGI(TAG, "User Input 1 state: %s", 
                 stateResult.GetValue() ? "ACTIVE" : "INACTIVE");
    } else {
        ESP_LOGE(TAG, "Failed to read input: %s", 
                 GetResultDescription(stateResult.GetResult()).data());
    }
    
    // Toggle operation
    auto toggleResult = gpio.Toggle(GPIO_LED_STATUS_ERROR);
    if (toggleResult.IsSuccess()) {
        ESP_LOGI(TAG, "Error LED toggled to: %s", 
                 toggleResult.GetValue() ? "ACTIVE" : "INACTIVE");
    }
    
    // Batch operations
    std::vector<GpioPin> statusLeds = {
        GPIO_LED_STATUS_OK, 
        GPIO_LED_STATUS_ERROR, 
        GPIO_LED_STATUS_COMM
    };
    
    auto batchResult = gpio.SetMultipleActive(statusLeds);
    if (batchResult.IsSuccess()) {
        ESP_LOGI(TAG, "All status LEDs activated successfully");
        ESP_LOGI(TAG, "Success rate: %.1f%%", 
                 batchResult.GetValue().AllSuccessful() ? 100.0f : 
                 (static_cast<float>(batchResult.GetValue().results.size()) / 
                  static_cast<float>(statusLeds.size())) * 100.0f);
    }
}

/**
 * @brief Example function demonstrating basic ADC operations.
 */
void example_adc_operations() {
    ESP_LOGI(TAG, "=== ADC Operations Example ===");
    
    // Get the ADC manager instance
    auto& adc = HardFocComponentHandler::GetAdc();
    
    // Basic channel reading
    auto reading = adc.ReadChannel(ADC_MOTOR_CURRENT_PHASE_A);
    if (reading.IsSuccess()) {
        const auto& adcReading = reading.GetValue();
        ESP_LOGI(TAG, "Motor Current Phase A: %.3f V (raw: %lu)", 
                 adcReading.voltage, adcReading.rawValue);
    } else {
        ESP_LOGE(TAG, "Failed to read motor current: %s", 
                 GetResultDescription(reading.GetResult()).data());
    }
    
    // Multi-sample reading with averaging
    AdcSamplingSpec spec(ADC_SYSTEM_VOLTAGE_3V3, 10, 5, true, 0.1f);
    auto sampledReading = adc.ReadChannelWithSampling(spec);
    if (sampledReading.IsSuccess()) {
        ESP_LOGI(TAG, "3V3 System Voltage (10 samples, filtered): %.3f V", 
                 sampledReading.GetValue().voltage);
    }
    
    // Read filtered value
    auto filteredResult = adc.ReadFilteredValue(ADC_SYSTEM_TEMPERATURE_AMBIENT);
    if (filteredResult.IsSuccess()) {
        ESP_LOGI(TAG, "Ambient Temperature (filtered): %.3f V", 
                 filteredResult.GetValue());
    }
    
    // Batch reading
    std::vector<AdcInputSensor> voltageSensors = {
        ADC_SYSTEM_VOLTAGE_3V3,
        ADC_SYSTEM_VOLTAGE_5V,
        ADC_SYSTEM_VOLTAGE_12V
    };
    
    auto batchResult = adc.BatchRead(voltageSensors);
    if (batchResult.IsSuccess()) {
        const auto& batch = batchResult.GetValue();
        ESP_LOGI(TAG, "Voltage readings batch completed:");
        for (size_t i = 0; i < batch.readings.size(); ++i) {
            ESP_LOGI(TAG, "  Sensor %d: %.3f V", 
                     static_cast<int>(batch.sensors[i]), 
                     batch.readings[i].voltage);
        }
        ESP_LOGI(TAG, "Batch success rate: %.1f%%", batch.GetSuccessRate());
    }
}

/**
 * @brief Example function demonstrating system diagnostics.
 */
void example_system_diagnostics() {
    ESP_LOGI(TAG, "=== System Diagnostics Example ===");
    
    // Get system health information
    auto healthResult = HardFocComponentHandler::GetSystemHealth();
    if (healthResult.IsSuccess()) {
        ESP_LOGI(TAG, "System Health Report:\n%s", healthResult.GetValue().c_str());
    }
    
    // Perform self-test
    auto testResult = HardFocComponentHandler::PerformSelfTest();
    if (testResult.IsSuccess()) {
        ESP_LOGI(TAG, "Self-Test Results:\n%s", testResult.GetValue().c_str());
    } else {
        ESP_LOGE(TAG, "Self-test failed: %s", 
                 GetResultDescription(testResult.GetResult()).data());
    }
    
    // Individual manager statistics
    auto& gpio = HardFocComponentHandler::GetGpio();
    auto& adc = HardFocComponentHandler::GetAdc();
    
    ESP_LOGI(TAG, "GPIO Manager: %zu pins registered", gpio.GetRegisteredPinCount());
    ESP_LOGI(TAG, "ADC Manager: %zu channels registered", adc.GetRegisteredChannelCount());
}

/**
 * @brief Example function demonstrating error handling patterns.
 */
void example_error_handling() {
    ESP_LOGI(TAG, "=== Error Handling Example ===");
    
    auto& gpio = HardFocComponentHandler::GetGpio();
    
    // Example of handling different error conditions
    auto result = gpio.SetActive(static_cast<GpioPin>(999)); // Invalid pin
    if (result.IsError()) {
        auto errorCode = result.GetResult();
        ESP_LOGE(TAG, "Expected error occurred:");
        ESP_LOGE(TAG, "  Code: %d", static_cast<int>(errorCode));
        ESP_LOGE(TAG, "  Name: %s", GetResultName(errorCode).data());
        ESP_LOGE(TAG, "  Description: %s", GetResultDescription(errorCode).data());
        
        // Handle specific error types
        switch (errorCode) {
            case HardFocResult::ERROR_GPIO_PIN_NOT_FOUND:
                ESP_LOGW(TAG, "Handling pin not found error");
                break;
            case HardFocResult::ERROR_GPIO_INVALID_PIN:
                ESP_LOGW(TAG, "Handling invalid pin error");
                break;
            default:
                ESP_LOGW(TAG, "Handling generic error");
                break;
        }
    }
    
    // Example of using GetValueOr for default values
    auto& adc = HardFocComponentHandler::GetAdc();
    auto reading = adc.ReadChannel(static_cast<AdcInputSensor>(999)); // Invalid channel
    float voltage = reading.IsSuccess() ? reading.GetValue().voltage : 0.0f;
    ESP_LOGI(TAG, "Voltage reading (with fallback): %.3f V", voltage);
}

/**
 * @brief Example function demonstrating name-based access.
 */
void example_name_based_access() {
    ESP_LOGI(TAG, "=== Name-Based Access Example ===");
    
    auto& gpio = HardFocComponentHandler::GetGpio();
    auto& adc = HardFocComponentHandler::GetAdc();
    
    // GPIO operations by name
    auto result = gpio.SetActiveByName("MOTOR_ENABLE");
    if (result.IsSuccess()) {
        ESP_LOGI(TAG, "Motor enabled via name-based access");
    }
    
    auto stateResult = gpio.ReadStateByName("USER_INPUT_1");
    if (stateResult.IsSuccess()) {
        ESP_LOGI(TAG, "User Input 1 (by name): %s", 
                 stateResult.GetValue() ? "ACTIVE" : "INACTIVE");
    }
    
    // ADC operations by name
    auto readingResult = adc.ReadChannelByName("MOTOR_CURRENT_PHASE_A");
    if (readingResult.IsSuccess()) {
        ESP_LOGI(TAG, "Motor current (by name): %.3f V", 
                 readingResult.GetValue().voltage);
    }
    
    auto filteredResult = adc.ReadFilteredValueByName("SYSTEM_VOLTAGE_3V3");
    if (filteredResult.IsSuccess()) {
        ESP_LOGI(TAG, "3V3 voltage (filtered, by name): %.3f V", 
                 filteredResult.GetValue());
    }
}

/**
 * @brief Main example task function.
 */
void modern_hal_example_task(void* pvParameters) {
    ESP_LOGI(TAG, "Starting Modern HardFOC HAL Example");
    
    // Initialize the component handler system
    // Note: In a real application, you would have actual hardware interfaces
    // For this example, we assume they're available
    /*
    SfI2cBus i2cBus;
    Tmc9660MotorController tmc9660Controller;
    
    auto initResult = HardFocComponentHandler::Initialize(i2cBus, tmc9660Controller);
    if (initResult.IsError()) {
        ESP_LOGE(TAG, "Failed to initialize component handler: %s", 
                 GetResultDescription(initResult.GetResult()).data());
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Component handler initialized successfully");
    */
    
    // Run examples (commented out since we don't have real hardware initialized)
    /*
    example_gpio_operations();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_adc_operations();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_system_diagnostics();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_error_handling();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    example_name_based_access();
    */
    
    ESP_LOGI(TAG, "Modern HardFOC HAL Example completed");
    
    // In a real application, you might want to shutdown the system
    /*
    auto shutdownResult = HardFocComponentHandler::Shutdown();
    if (shutdownResult.IsError()) {
        ESP_LOGE(TAG, "Warning: Failed to shutdown cleanly: %s", 
                 GetResultDescription(shutdownResult.GetResult()).data());
    }
    */
    
    vTaskDelete(NULL);
}

/**
 * @brief Create and start the example task.
 */
void start_modern_hal_example() {
    xTaskCreate(modern_hal_example_task, 
                "modern_hal_example", 
                4096, 
                NULL, 
                5, 
                NULL);
}
