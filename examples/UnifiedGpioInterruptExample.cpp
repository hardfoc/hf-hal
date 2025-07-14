/**
 * @file UnifiedGpioInterruptExample.cpp
 * @brief Example demonstrating unified GPIO interrupt functionality
 * 
 * This example shows how the new interrupt-capable BaseGpio architecture
 * simplifies GPIO interrupt handling across different implementations.
 */

#include "McuDigitalGpio.h"
#include "component-handler/Pcal95555GpioWrapper.h"
#include "GpioGuard.h"
#include <memory>
#include <vector>

// Example callback function
void GpioInterruptHandler(BaseGpio* gpio, BaseGpio::InterruptTrigger trigger, void* user_data) {
    const char* pin_name = static_cast<const char*>(user_data);
    
    printf("Interrupt on %s (pin %d): %s\n", 
           pin_name, gpio->GetPin(), BaseGpio::ToString(trigger));
    
    // Example: Toggle an LED on interrupt
    if (gpio->GetDirection() == BaseGpio::Direction::Input) {
        // This is an input that triggered - find associated output to toggle
        // (Implementation would depend on your specific application logic)
    }
}

// Example class showing polymorphic GPIO usage
class MotorController {
public:
    MotorController(std::unique_ptr<BaseGpio> enable_pin,
                   std::unique_ptr<BaseGpio> fault_pin,
                   std::unique_ptr<BaseGpio> status_led)
        : enable_pin_(std::move(enable_pin))
        , fault_pin_(std::move(fault_pin))
        , status_led_(std::move(status_led)) {}

    bool Initialize() {
        // Initialize all pins
        if (!enable_pin_->Initialize() || 
            !fault_pin_->Initialize() || 
            !status_led_->Initialize()) {
            return false;
        }

        // Configure fault pin for interrupt if supported
        if (fault_pin_->SupportsInterrupts()) {
            HfGpioErr result = fault_pin_->ConfigureInterrupt(
                BaseGpio::InterruptTrigger::FallingEdge,
                GpioInterruptHandler,
                const_cast<char*>("Motor_Fault")
            );
            
            if (result == HfGpioErr::GPIO_SUCCESS) {
                fault_pin_->EnableInterrupt();
                printf("Fault interrupt configured successfully\n");
            }
        }

        return true;
    }

    void EnableMotor() {
        GpioGuard guard(*enable_pin_);
        // Motor is automatically enabled and will be disabled when guard destructs
        
        // Configure status LED
        status_led_->SetDirection(BaseGpio::Direction::Output);
        status_led_->SetActive();  // Turn on status LED
        
        printf("Motor enabled with status LED\n");
    }

    void CheckFaultStatus() {
        if (fault_pin_->SupportsInterrupts()) {
            BaseGpio::InterruptStatus status;
            if (fault_pin_->GetInterruptStatus(status) == HfGpioErr::GPIO_SUCCESS) {
                printf("Fault pin interrupt count: %u\n", status.interrupt_count);
            }
        } else {
            // Fallback to polling for non-interrupt capable pins
            bool is_active = false;
            if (fault_pin_->IsActive(is_active) == HfGpioErr::GPIO_SUCCESS && is_active) {
                printf("Fault detected via polling\n");
            }
        }
    }

private:
    std::unique_ptr<BaseGpio> enable_pin_;
    std::unique_ptr<BaseGpio> fault_pin_;
    std::unique_ptr<BaseGpio> status_led_;
};

// Example usage with different GPIO implementations
void UnifiedGpioExample() {
    // Create MCU GPIO instances (ESP32 pins)
    auto mcu_enable = std::make_unique<McuDigitalGpio>(
        GPIO_NUM_2, BaseGpio::Direction::Output
    );
    
    auto mcu_fault = std::make_unique<McuDigitalGpio>(
        GPIO_NUM_4, BaseGpio::Direction::Input, BaseGpio::ActiveState::Low
    );
    
    // Create I2C GPIO expander instance (PCAL95555 pin)
    // Note: In real implementation, you'd share the PCAL95555 wrapper instance
    auto pcal_wrapper = CreatePcal95555GpioWrapper(i2c_bus, 0x20);
    auto pcal_status_led = pcal_wrapper->CreateGpioPin(Pcal95555Chip1Pin::LED_STATUS_GREEN);
    
    // Create motor controller using polymorphic GPIO pins
    MotorController motor(
        std::move(mcu_enable),
        std::move(mcu_fault), 
        std::move(pcal_status_led)
    );
    
    // Initialize and use the motor controller
    if (motor.Initialize()) {
        printf("Motor controller initialized successfully\n");
        
        // Enable motor (demonstrates RAII pattern)
        motor.EnableMotor();
        
        // Check fault status (demonstrates polymorphic interrupt handling)
        motor.CheckFaultStatus();
    }
}

// Example showing direct interrupt usage
void DirectInterruptExample() {
    // Create a button input with interrupt
    McuDigitalGpio button(GPIO_NUM_0, BaseGpio::Direction::Input, 
                         BaseGpio::ActiveState::Low, 
                         BaseGpio::OutputMode::PushPull,
                         BaseGpio::PullMode::PullUp);
    
    if (!button.Initialize()) {
        printf("Failed to initialize button\n");
        return;
    }
    
    // Configure interrupt for button press
    if (button.SupportsInterrupts()) {
        HfGpioErr result = button.ConfigureInterrupt(
            BaseGpio::InterruptTrigger::FallingEdge,
            [](BaseGpio* gpio, BaseGpio::InterruptTrigger trigger, void* user_data) {
                printf("Button pressed! (pin %d)\n", gpio->GetPin());
                
                // Example: Clear interrupt statistics after handling
                gpio->ClearInterruptStats();
            },
            nullptr
        );
        
        if (result == HfGpioErr::GPIO_SUCCESS) {
            button.EnableInterrupt();
            printf("Button interrupt configured\n");
            
            // Example: Wait for button press with timeout
            printf("Waiting for button press (5 second timeout)...\n");
            HfGpioErr wait_result = button.WaitForInterrupt(5000);
            
            if (wait_result == HfGpioErr::GPIO_SUCCESS) {
                printf("Button was pressed!\n");
            } else {
                printf("Timeout waiting for button press\n");
            }
        }
    }
}

// Example showing how different implementations can coexist
void MixedImplementationExample() {
    std::vector<std::unique_ptr<BaseGpio>> gpio_pins;
    
    // Add various GPIO implementations
    gpio_pins.push_back(std::make_unique<McuDigitalGpio>(GPIO_NUM_1));
    gpio_pins.push_back(std::make_unique<McuDigitalGpio>(GPIO_NUM_2));
    // gpio_pins.push_back(pcal_wrapper->CreateGpioPin(Pcal95555Chip1Pin::LED_STATUS_GREEN));
    
    // Configure each pin polymorphically
    for (auto& pin : gpio_pins) {
        if (!pin->Initialize()) {
            printf("Failed to initialize pin %d\n", pin->GetPin());
            continue;
        }
        
        printf("Pin %d (%s): Interrupts %s\n", 
               pin->GetPin(),
               pin->GetDescription(),
               pin->SupportsInterrupts() ? "supported" : "not supported");
        
        // Configure interrupt if supported
        if (pin->SupportsInterrupts()) {
            pin->ConfigureInterrupt(BaseGpio::InterruptTrigger::BothEdges,
                                   GpioInterruptHandler,
                                   const_cast<char*>("GenericPin"));
            pin->EnableInterrupt();
        }
    }
    
    printf("All pins configured. Mixed implementations working together!\n");
}

/**
 * @brief Example: Migration from DigitalExternalIRQ
 * @details Shows how to migrate existing DigitalExternalIRQ code to the new unified architecture
 */
void MigrationExample() {
    printf("\n=== Migration from DigitalExternalIRQ ===\n");
    
    // OLD WAY (before modernization):
    // DigitalExternalIRQ irq_pin(GPIO_NUM_0, hf_gpio_intr_type_t::FallingEdge);
    // irq_pin.Enable();
    // if (irq_pin.Wait(1000)) { /* interrupt received */ }
    
    // NEW WAY (unified architecture):
    printf("Creating GPIO with unified interrupt architecture...\n");
    
    auto irq_pin = std::make_unique<McuDigitalGpio>(
        GPIO_NUM_0,                          // Same pin
        BaseGpio::Direction::Input,          // Input for interrupt
        BaseGpio::ActiveState::Low,          // Active low (for button)
        BaseGpio::OutputMode::PushPull,      // Not used for input
        BaseGpio::PullMode::PullUp           // Pull-up for button
    );
    
    if (!irq_pin->Initialize()) {
        printf("Failed to initialize GPIO\n");
        return;
    }
    
    // Configure interrupt (replaces old constructor parameter)
    HfGpioErr result = irq_pin->ConfigureInterrupt(
        BaseGpio::InterruptTrigger::FallingEdge  // Same as old hf_gpio_intr_type_t::FallingEdge
    );
    
    if (result != HfGpioErr::GPIO_SUCCESS) {
        printf("Failed to configure interrupt: %s\n", HfGpioErrToString(result));
        return;
    }
    
    // Enable interrupt (replaces old Enable() method)
    result = irq_pin->EnableInterrupt();
    if (result != HfGpioErr::GPIO_SUCCESS) {
        printf("Failed to enable interrupt: %s\n", HfGpioErrToString(result));
        return;
    }
    
    // Wait for interrupt (replaces old Wait() method)
    printf("Waiting for interrupt...\n");
    result = irq_pin->WaitForInterrupt(1000);  // 1 second timeout
    
    if (result == HfGpioErr::GPIO_SUCCESS) {
        printf("Interrupt received! (New unified API)\n");
    } else if (result == HfGpioErr::GPIO_ERR_TIMEOUT) {
        printf("Timeout - no interrupt received\n");
    } else {
        printf("Error waiting for interrupt: %s\n", HfGpioErrToString(result));
    }
    
    // Get interrupt statistics (new feature not available in old API)
    BaseGpio::InterruptStatus status;
    result = irq_pin->GetInterruptStatus(status);
    if (result == HfGpioErr::GPIO_SUCCESS) {
        printf("Interrupt statistics:\n");
        printf("  Enabled: %s\n", status.is_enabled ? "Yes" : "No");
        printf("  Trigger: %s\n", BaseGpio::ToString(status.trigger_type));
        printf("  Count: %u\n", status.interrupt_count);
        printf("  Has callback: %s\n", status.has_callback ? "Yes" : "No");
    }
    
    // Cleanup (replaces old Disable() method)
    irq_pin->DisableInterrupt();
    
    printf("Migration example completed successfully!\n");
    printf("\nBenefits of new architecture:\n");
    printf("- Single BaseGpio class handles all GPIO operations\n");
    printf("- Interrupt capability is optional and polymorphic\n");
    printf("- Better error handling with comprehensive error codes\n");
    printf("- Interrupt statistics and status reporting\n");
    printf("- Consistent API across MCU and expander GPIOs\n");
}

int main() {
    printf("=== Unified GPIO Interrupt Example ===\n\n");
    
    printf("1. Unified GPIO Example:\n");
    UnifiedGpioExample();
    
    printf("\n2. Direct Interrupt Example:\n");
    DirectInterruptExample();
    
    printf("\n3. Mixed Implementation Example:\n");
    MixedImplementationExample();
    
    printf("\n4. Migration Example:\n");
    MigrationExample();
    
    return 0;
}
