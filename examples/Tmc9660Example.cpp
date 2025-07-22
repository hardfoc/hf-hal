/**
 * @file Tmc9660Example.cpp
 * @brief Example usage of the TMC9660 Motor Controller system.
 * 
 * This file demonstrates how to use the TMC9660 motor controller singleton
 * for motor control operations in the HardFOC system.
 */

#include "Tmc9660Handler.h"
#include <iostream>

// Example usage of the new unified Tmc9660Handler
int main() {
    // Assume you have a TMC9660CommInterface implementation (e.g., SpiComm or UartComm)
    // For this example, we'll use a dummy placeholder
    class DummyComm : public TMC9660CommInterface {
    public:
        bool transfer(const TMCLFrame&, TMCLReply&, uint8_t) override { return true; }
    } comm;

    // Instantiate the handler (address 0x01, default config)
    Tmc9660Handler tmcHandler(comm, 0x01);

    // Initialize the device
    if (!tmcHandler.Initialize()) {
        std::cerr << "Failed to initialize TMC9660 device!\n";
        return 1;
    }

    // Access GPIO 17 and set it active
    auto& gpio17 = tmcHandler.gpio(17);
    gpio17.Initialize();
    gpio17.SetActive();
    std::cout << "GPIO 17 state: " << (gpio17.IsActive() ? "Active" : "Inactive") << std::endl;

    // Access ADC channel 0 and read voltage
    auto& adc = tmcHandler.adc();
    adc.Initialize();
    float voltage = 0.0f;
    if (adc.ReadChannelV(0, voltage) == BaseAdc::AdcErr::None) {
        std::cout << "ADC channel 0 voltage: " << voltage << " V" << std::endl;
    } else {
        std::cout << "Failed to read ADC channel 0" << std::endl;
    }

    // Access the TMC9660 driver directly for advanced parameter tweaking
    if (tmcHandler.IsDriverReady()) {
        auto driver = tmcHandler.driver(); // Now returns std::shared_ptr<TMC9660>
        // Example: set a parameter (replace with real parameter and value)
        driver->writeParameter(static_cast<tmc9660::tmcl::Parameters>(5), 1234);
        std::cout << "TMC9660 parameter set successfully" << std::endl;
        
        // You can safely store the shared_ptr for later use
        // std::shared_ptr<TMC9660> stored_driver = driver;
    } else {
        std::cout << "TMC9660 driver not available - ensure Initialize() was called" << std::endl;
    }

    std::cout << "TMC9660 example complete." << std::endl;
    return 0;
}
