#include "Tmc9660MotorController.h"
#include "ConsolePort.h"
#include "OsUtility.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_config.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_ext_pins_enum.hpp"

/**
 * @file Tmc9660MotorController.cpp
 * @brief Implementation of the TMC9660 motor controller singleton class.
 * 
 * This file contains the implementation of the comprehensive TMC9660 motor controller
 * management system for the HardFOC project.
 */

static const char* TAG = "Tmc9660MotorController";

// Static member definition
Tmc9660MotorController& Tmc9660MotorController::GetInstance() noexcept {
    static Tmc9660MotorController instance;
    return instance;
}

Tmc9660MotorController::Tmc9660MotorController() noexcept
    : initialized_(false),
      chipInstances_{},
      registeredChipCount_(0),
      adcSystem_(nullptr),
      gpioSystem_(nullptr)
{
    console_info(TAG, "Tmc9660MotorController constructor called");
    
    // Initialize all chip instances as unregistered
    for (auto& instance : chipInstances_) {
        instance.isRegistered = false;
        instance.chipId = Tmc9660ChipId::TMC9660_CHIP_COUNT; // Invalid ID
        instance.tmcDriver = nullptr;
    }
}

Tmc9660MotorController::~Tmc9660MotorController() noexcept {
    console_info(TAG, "Tmc9660MotorController destructor called");
}

bool Tmc9660MotorController::Initialize() noexcept {
    console_info(TAG, "Tmc9660MotorController::Initialize() - Setting up TMC9660 system");
    
    std::lock_guard<std::mutex> lock(systemMutex_);
    
    if (initialized_.load()) {
        console_info(TAG, "TMC9660 system already initialized");
        return true;
    }
    
    // Get references to ADC and GPIO manager systems
    adcSystem_ = &AdcManager::GetInstance();
    gpioSystem_ = &GpioManager::GetInstance();
    
    // Ensure the underlying systems are initialized
    if (!adcSystem_->IsInitialized()) {
        console_error(TAG, "ADC system not initialized");
        return false;
    }
    
    if (!gpioSystem_->EnsureInitialized()) {
        console_error(TAG, "Failed to initialize GPIO system");
        return false;
    }
    
    // Reset registration count
    registeredChipCount_ = 0;
    
    console_info(TAG, "TMC9660 system initialized successfully");
    initialized_.store(true);
    return true;
}

bool Tmc9660MotorController::EnsureInitialized() noexcept {
    if (!initialized_.load()) {
        return Initialize();
    }
    return true;
}

bool Tmc9660MotorController::RegisterTmc9660Chip(Tmc9660ChipId chipId, 
                                                 const Tmc9660Config& config,
                                                 const Tmc9660GpioConfig& gpioConfig,
                                                 const Tmc9660AdcConfig& adcConfig) noexcept {
    console_info(TAG, "Registering TMC9660 chip: %s", Tmc9660ChipIdToString(chipId).data());
    
    if (!EnsureInitialized()) {
        console_error(TAG, "System not initialized");
        return false;
    }
    
    std::lock_guard<std::mutex> lock(systemMutex_);
    
    // Validate chip ID
    if (chipId >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        console_error(TAG, "Invalid chip ID: %d", static_cast<int>(chipId));
        return false;
    }
    
    uint8_t chipIndex = static_cast<uint8_t>(chipId);
    Tmc9660ChipInstance& instance = chipInstances_[chipIndex];
    
    // Check if already registered
    if (instance.isRegistered) {
        console_warning(TAG, "Chip %s already registered", Tmc9660ChipIdToString(chipId).data());
        return true;
    }
    
    // Set up the chip instance
    instance.chipId = chipId;
    instance.config = config;
    instance.gpioConfig = gpioConfig;
    instance.adcConfig = adcConfig;
    instance.status = {}; // Initialize status to all false/zero
    
    // Initialize GPIO pins for this chip
    if (!InitializeChipGpio(instance)) {
        console_error(TAG, "Failed to initialize GPIO for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Initialize ADC channels for this chip
    if (!InitializeChipAdc(instance)) {
        console_error(TAG, "Failed to initialize ADC for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Create the TMC9660 driver instance
    // Note: This would need to be implemented based on the actual TMC9660 driver interface
    // For now, we'll set it to nullptr and implement the interface
    instance.tmcDriver = nullptr; // TODO: Create actual TMC9660 driver instance
    
    // Configure the chip
    if (!ConfigureChip(instance)) {
        console_error(TAG, "Failed to configure chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Mark as registered
    instance.isRegistered = true;
    registeredChipCount_++;
    
    console_info(TAG, "TMC9660 chip %s registered successfully", Tmc9660ChipIdToString(chipId).data());
    return true;
}

uint8_t Tmc9660MotorController::GetRegisteredChipCount() const noexcept {
    return registeredChipCount_;
}

bool Tmc9660MotorController::IsChipRegistered(Tmc9660ChipId chipId) const noexcept {
    if (chipId >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        return false;
    }
    
    uint8_t chipIndex = static_cast<uint8_t>(chipId);
    return chipInstances_[chipIndex].isRegistered;
}

bool Tmc9660MotorController::GetChipStatus(Tmc9660ChipId chipId, Tmc9660Status& status) noexcept {
    const Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        return false;
    }
    
    status = instance->status;
    return true;
}

bool Tmc9660MotorController::EnableSpiCommunication(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Enabling SPI communication for chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Enable SPI communication by pulling TMC_SPI_COMM_nEN low
    if (!gpioSystem_->SetInactive(instance->gpioConfig.spiCommEnablePin)) {
        console_error(TAG, "Failed to enable SPI communication for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    instance->config.spiEnabled = true;
    instance->status.isCommunicating = true;
    
    console_info(TAG, "SPI communication enabled for chip %s", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::DisableSpiCommunication(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Disabling SPI communication for chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Disable SPI communication by pulling TMC_SPI_COMM_nEN high
    if (!gpioSystem_->SetActive(instance->gpioConfig.spiCommEnablePin)) {
        console_error(TAG, "Failed to disable SPI communication for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    instance->config.spiEnabled = false;
    
    console_info(TAG, "SPI communication disabled for chip %s", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::SwitchCommunicationInterface(Tmc9660ChipId chipId, Tmc9660CommInterface newInterface) noexcept {
    console_info(TAG, "Switching communication interface for chip %s to %s", 
                 Tmc9660ChipIdToString(chipId).data(), 
                 Tmc9660CommInterfaceToString(newInterface).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Switch based on the new interface
    switch (newInterface) {
        case Tmc9660CommInterface::TMC_COMM_SPI:
            if (!EnableSpiCommunication(chipId)) {
                return false;
            }
            instance->config.primaryInterface = newInterface;
            break;
            
        case Tmc9660CommInterface::TMC_COMM_UART:
            if (!DisableSpiCommunication(chipId)) {
                return false;
            }
            instance->config.primaryInterface = newInterface;
            break;
            
        default:
            console_error(TAG, "Invalid communication interface: %d", static_cast<int>(newInterface));
            return false;
    }
    
    console_info(TAG, "Communication interface switched successfully for chip %s", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::EnableDriver(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Enabling driver for chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Enable driver by setting TMC_DRV_EN high
    if (!gpioSystem_->SetActive(instance->gpioConfig.driverEnablePin)) {
        console_error(TAG, "Failed to enable driver for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    instance->status.isDriverEnabled = true;
    
    console_info(TAG, "Driver enabled for chip %s", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::DisableDriver(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Disabling driver for chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Disable driver by setting TMC_DRV_EN low
    if (!gpioSystem_->SetInactive(instance->gpioConfig.driverEnablePin)) {
        console_error(TAG, "Failed to disable driver for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    instance->status.isDriverEnabled = false;
    
    console_info(TAG, "Driver disabled for chip %s", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::ResetChip(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Resetting chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Reset the chip by pulsing TMC_RST_CTRL low
    if (!gpioSystem_->SetInactive(instance->gpioConfig.resetControlPin)) {
        console_error(TAG, "Failed to assert reset for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Hold reset for at least 1ms
    os_delay_msec(10);
    
    if (!gpioSystem_->SetActive(instance->gpioConfig.resetControlPin)) {
        console_error(TAG, "Failed to release reset for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Wait for chip to come out of reset
    os_delay_msec(100);
    
    // Clear status and reinitialize
    instance->status = {};
    instance->status.isInitialized = false;
    
    // Reconfigure the chip
    if (!ConfigureChip(*instance)) {
        console_error(TAG, "Failed to reconfigure chip %s after reset", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    console_info(TAG, "Chip %s reset successfully", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::WakeUpChip(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Waking up chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Wake up the chip by setting TMC_nWAKE_CTRL high
    if (!gpioSystem_->SetActive(instance->gpioConfig.wakeControlPin)) {
        console_error(TAG, "Failed to wake up chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    instance->status.isAwake = true;
    
    console_info(TAG, "Chip %s woken up successfully", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::SleepChip(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Putting chip %s to sleep", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Put the chip to sleep by setting TMC_nWAKE_CTRL low
    if (!gpioSystem_->SetInactive(instance->gpioConfig.wakeControlPin)) {
        console_error(TAG, "Failed to put chip %s to sleep", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    instance->status.isAwake = false;
    
    console_info(TAG, "Chip %s put to sleep successfully", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::HasFault(Tmc9660ChipId chipId) noexcept {
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        return false;
    }
    
    // Read the fault status pin (active low)
    bool faultActive = !gpioSystem_->IsActive(instance->gpioConfig.faultStatusPin);
    
    instance->status.hasFault = faultActive;
    if (faultActive) {
        instance->status.faultCount++;
    }
    
    return faultActive;
}

bool Tmc9660MotorController::ClearFault(Tmc9660ChipId chipId) noexcept {
    console_info(TAG, "Clearing fault for chip %s", Tmc9660ChipIdToString(chipId).data());
    
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        console_error(TAG, "Chip %s not found", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    // Clear fault by disabling and re-enabling the driver
    if (!DisableDriver(chipId)) {
        return false;
    }
    
    os_delay_msec(10);
    
    if (!EnableDriver(chipId)) {
        return false;
    }
    
    // Check if fault is cleared
    os_delay_msec(10);
    if (HasFault(chipId)) {
        console_error(TAG, "Fault still present after clearing for chip %s", Tmc9660ChipIdToString(chipId).data());
        return false;
    }
    
    console_info(TAG, "Fault cleared for chip %s", Tmc9660ChipIdToString(chipId).data());
    return true;
}

bool Tmc9660MotorController::ReadAdcValue(Tmc9660ChipId chipId, uint8_t adcChannel, uint32_t& value) noexcept {
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        return false;
    }
    
    AdcInputSensor sensor;
    switch (adcChannel) {
        case 1:
            sensor = instance->adcConfig.ain1Sensor;
            break;
        case 2:
            sensor = instance->adcConfig.ain2Sensor;
            break;
        case 3:
            sensor = instance->adcConfig.ain3Sensor;
            break;
        default:
            console_error(TAG, "Invalid ADC channel: %d", adcChannel);
            return false;
    }
    
    return adcSystem_->GetCount(sensor, value, 1, 0, TimeUnit::TIME_UNIT_MS);
}

bool Tmc9660MotorController::ReadAdcVoltage(Tmc9660ChipId chipId, uint8_t adcChannel, float& voltage) noexcept {
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        return false;
    }
    
    AdcInputSensor sensor;
    switch (adcChannel) {
        case 1:
            sensor = instance->adcConfig.ain1Sensor;
            break;
        case 2:
            sensor = instance->adcConfig.ain2Sensor;
            break;
        case 3:
            sensor = instance->adcConfig.ain3Sensor;
            break;
        default:
            console_error(TAG, "Invalid ADC channel: %d", adcChannel);
            return false;
    }
    
    return adcSystem_->GetVolt(sensor, voltage, 1, 0, TimeUnit::TIME_UNIT_MS);
}

bool Tmc9660MotorController::SetGpioState(Tmc9660ChipId chipId, uint8_t gpioNumber, bool active) noexcept {
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        return false;
    }
    
    GpioPin pin;
    switch (gpioNumber) {
        case 17:
            pin = instance->gpioConfig.gpio17Pin;
            break;
        case 18:
            pin = instance->gpioConfig.gpio18Pin;
            break;
        default:
            console_error(TAG, "Invalid GPIO number: %d", gpioNumber);
            return false;
    }
    
    if (active) {
        return gpioSystem_->SetActive(pin);
    } else {
        return gpioSystem_->SetInactive(pin);
    }
}

bool Tmc9660MotorController::GetGpioState(Tmc9660ChipId chipId, uint8_t gpioNumber) noexcept {
    Tmc9660ChipInstance* instance = FindChipInstance(chipId);
    if (!instance) {
        return false;
    }
    
    GpioPin pin;
    switch (gpioNumber) {
        case 17:
            pin = instance->gpioConfig.gpio17Pin;
            break;
        case 18:
            pin = instance->gpioConfig.gpio18Pin;
            break;
        default:
            console_error(TAG, "Invalid GPIO number: %d", gpioNumber);
            return false;
    }
    
    return gpioSystem_->IsActive(pin);
}

bool Tmc9660MotorController::TestCommunication() noexcept {
    console_info(TAG, "Testing communication with all TMC9660 chips");
    
    bool allChipsResponding = true;
    
    for (uint8_t i = 0; i < MAX_TMC9660_CHIPS; i++) {
        if (chipInstances_[i].isRegistered) {
            Tmc9660ChipId chipId = chipInstances_[i].chipId;
            
            // Update chip status
            if (!UpdateChipStatus(chipInstances_[i])) {
                console_error(TAG, "Failed to update status for chip %s", Tmc9660ChipIdToString(chipId).data());
                allChipsResponding = false;
            } else {
                console_info(TAG, "Chip %s responding", Tmc9660ChipIdToString(chipId).data());
            }
        }
    }
    
    return allChipsResponding;
}

bool Tmc9660MotorController::RunDiagnostics() noexcept {
    console_info(TAG, "Running diagnostics on all TMC9660 chips");
    
    bool allChipsHealthy = true;
    
    for (uint8_t i = 0; i < MAX_TMC9660_CHIPS; i++) {
        if (chipInstances_[i].isRegistered) {
            Tmc9660ChipId chipId = chipInstances_[i].chipId;
            
            // Check for faults
            if (HasFault(chipId)) {
                console_error(TAG, "Fault detected on chip %s", Tmc9660ChipIdToString(chipId).data());
                allChipsHealthy = false;
            }
            
            // Test ADC channels
            uint32_t adcValue;
            for (uint8_t ch = 1; ch <= 3; ch++) {
                if (!ReadAdcValue(chipId, ch, adcValue)) {
                    console_error(TAG, "Failed to read ADC channel %d on chip %s", ch, Tmc9660ChipIdToString(chipId).data());
                    allChipsHealthy = false;
                }
            }
            
            // Test GPIO pins
            for (uint8_t gpio = 17; gpio <= 18; gpio++) {
                if (!SetGpioState(chipId, gpio, true)) {
                    console_error(TAG, "Failed to set GPIO %d on chip %s", gpio, Tmc9660ChipIdToString(chipId).data());
                    allChipsHealthy = false;
                }
            }
        }
    }
    
    return allChipsHealthy;
}

bool Tmc9660MotorController::GetSystemHealth() noexcept {
    bool systemHealthy = true;
    
    // Check if system is initialized
    if (!initialized_.load()) {
        return false;
    }
    
    // Check all registered chips
    for (uint8_t i = 0; i < MAX_TMC9660_CHIPS; i++) {
        if (chipInstances_[i].isRegistered) {
            // Update and check chip status
            if (!UpdateChipStatus(chipInstances_[i])) {
                systemHealthy = false;
            }
            
            // Check for faults
            if (chipInstances_[i].status.hasFault) {
                systemHealthy = false;
            }
        }
    }
    
    return systemHealthy;
}

void Tmc9660MotorController::PrintSystemStatus() noexcept {
    console_info(TAG, "=== TMC9660 System Status ===");
    console_info(TAG, "System initialized: %s", initialized_.load() ? "Yes" : "No");
    console_info(TAG, "Registered chips: %d", registeredChipCount_);
    
    for (uint8_t i = 0; i < MAX_TMC9660_CHIPS; i++) {
        if (chipInstances_[i].isRegistered) {
            const Tmc9660ChipInstance& instance = chipInstances_[i];
            console_info(TAG, "Chip %s:", Tmc9660ChipIdToString(instance.chipId).data());
            console_info(TAG, "  Initialized: %s", instance.status.isInitialized ? "Yes" : "No");
            console_info(TAG, "  Communicating: %s", instance.status.isCommunicating ? "Yes" : "No");
            console_info(TAG, "  Has fault: %s", instance.status.hasFault ? "Yes" : "No");
            console_info(TAG, "  Driver enabled: %s", instance.status.isDriverEnabled ? "Yes" : "No");
            console_info(TAG, "  Awake: %s", instance.status.isAwake ? "Yes" : "No");
            console_info(TAG, "  Fault count: %lu", instance.status.faultCount);
            console_info(TAG, "  Communication errors: %lu", instance.status.communicationErrors);
        }
    }
}

// Private methods

Tmc9660MotorController::Tmc9660ChipInstance* Tmc9660MotorController::FindChipInstance(Tmc9660ChipId chipId) noexcept {
    if (chipId >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        return nullptr;
    }
    
    uint8_t chipIndex = static_cast<uint8_t>(chipId);
    if (!chipInstances_[chipIndex].isRegistered) {
        return nullptr;
    }
    
    return &chipInstances_[chipIndex];
}

const Tmc9660MotorController::Tmc9660ChipInstance* Tmc9660MotorController::FindChipInstance(Tmc9660ChipId chipId) const noexcept {
    if (chipId >= Tmc9660ChipId::TMC9660_CHIP_COUNT) {
        return nullptr;
    }
    
    uint8_t chipIndex = static_cast<uint8_t>(chipId);
    if (!chipInstances_[chipIndex].isRegistered) {
        return nullptr;
    }
    
    return &chipInstances_[chipIndex];
}

bool Tmc9660MotorController::InitializeChipGpio(Tmc9660ChipInstance& instance) noexcept {
    console_info(TAG, "Initializing GPIO for chip %s", Tmc9660ChipIdToString(instance.chipId).data());
    
    // Initialize all GPIO pins to safe states
    
    // Set SPI communication to disabled (high) initially
    if (!gpioSystem_->SetActive(instance.gpioConfig.spiCommEnablePin)) {
        console_error(TAG, "Failed to initialize SPI comm enable pin");
        return false;
    }
    
    // Set driver to disabled initially
    if (!gpioSystem_->SetInactive(instance.gpioConfig.driverEnablePin)) {
        console_error(TAG, "Failed to initialize driver enable pin");
        return false;
    }
    
    // Set reset to inactive (high)
    if (!gpioSystem_->SetActive(instance.gpioConfig.resetControlPin)) {
        console_error(TAG, "Failed to initialize reset control pin");
        return false;
    }
    
    // Set wake control to active (wake up)
    if (!gpioSystem_->SetActive(instance.gpioConfig.wakeControlPin)) {
        console_error(TAG, "Failed to initialize wake control pin");
        return false;
    }
    
    // Initialize TMC GPIO pins to inactive
    if (!gpioSystem_->SetInactive(instance.gpioConfig.gpio17Pin)) {
        console_error(TAG, "Failed to initialize GPIO17 pin");
        return false;
    }
    
    if (!gpioSystem_->SetInactive(instance.gpioConfig.gpio18Pin)) {
        console_error(TAG, "Failed to initialize GPIO18 pin");
        return false;
    }
    
    console_info(TAG, "GPIO initialized successfully for chip %s", Tmc9660ChipIdToString(instance.chipId).data());
    return true;
}

bool Tmc9660MotorController::InitializeChipAdc(Tmc9660ChipInstance& instance) noexcept {
    console_info(TAG, "Initializing ADC for chip %s", Tmc9660ChipIdToString(instance.chipId).data());
    
    // Test ADC channels by reading them
    uint32_t testValue;
    
    if (!adcSystem_->GetCount(instance.adcConfig.ain1Sensor, testValue, 1, 0, TimeUnit::TIME_UNIT_MS)) {
        console_error(TAG, "Failed to initialize AIN1 sensor");
        return false;
    }
    
    if (!adcSystem_->GetCount(instance.adcConfig.ain2Sensor, testValue, 1, 0, TimeUnit::TIME_UNIT_MS)) {
        console_error(TAG, "Failed to initialize AIN2 sensor");
        return false;
    }
    
    if (!adcSystem_->GetCount(instance.adcConfig.ain3Sensor, testValue, 1, 0, TimeUnit::TIME_UNIT_MS)) {
        console_error(TAG, "Failed to initialize AIN3 sensor");
        return false;
    }
    
    console_info(TAG, "ADC initialized successfully for chip %s", Tmc9660ChipIdToString(instance.chipId).data());
    return true;
}

bool Tmc9660MotorController::UpdateChipStatus(Tmc9660ChipInstance& instance) noexcept {
    // Update communication status
    instance.status.isCommunicating = instance.config.spiEnabled || instance.config.uartEnabled;
    
    // Update fault status
    instance.status.hasFault = !gpioSystem_->IsActive(instance.gpioConfig.faultStatusPin);
    
    // Update driver enable status
    instance.status.isDriverEnabled = gpioSystem_->IsActive(instance.gpioConfig.driverEnablePin);
    
    // Update wake status
    instance.status.isAwake = gpioSystem_->IsActive(instance.gpioConfig.wakeControlPin);
    
    // Update last communication time
    instance.status.lastCommunicationTime = os_get_time_msec();
    
    return true;
}

bool Tmc9660MotorController::ConfigureChip(Tmc9660ChipInstance& instance) noexcept {
    console_info(TAG, "Configuring chip %s", Tmc9660ChipIdToString(instance.chipId).data());
    
    // Set initial communication interface
    if (instance.config.primaryInterface == Tmc9660CommInterface::TMC_COMM_SPI) {
        if (!EnableSpiCommunication(instance.chipId)) {
            return false;
        }
    } else {
        if (!DisableSpiCommunication(instance.chipId)) {
            return false;
        }
    }
    
    // Wake up the chip
    if (!WakeUpChip(instance.chipId)) {
        return false;
    }
    
    // Update status
    UpdateChipStatus(instance);
    
    instance.status.isInitialized = true;
    
    console_info(TAG, "Chip %s configured successfully", Tmc9660ChipIdToString(instance.chipId).data());
    return true;
}

//=============================================================================
// Global Functions
//=============================================================================

bool InitializePrimaryTmc9660() noexcept {
    console_info(TAG, "Initializing primary TMC9660 chip");
    
    Tmc9660MotorController& controller = Tmc9660MotorController::GetInstance();
    
    // Set up configuration for the primary TMC9660 chip
    Tmc9660Config config = {
        .chipId = Tmc9660ChipId::TMC9660_CHIP_1,
        .primaryInterface = Tmc9660CommInterface::TMC_COMM_SPI,
        .secondaryInterface = Tmc9660CommInterface::TMC_COMM_UART,
        .spiDeviceId = 0,
        .uartPort = 0,
        .spiFrequency = 1000000,  // 1 MHz
        .uartBaudRate = 115200,
        .spiEnabled = false,      // Will be enabled when needed
        .uartEnabled = true       // UART is always available
    };
    
    // Set up GPIO configuration
    Tmc9660GpioConfig gpioConfig = {
        .gpio17Pin = GpioPin::GPIO_TMC_GPIO17,
        .gpio18Pin = GpioPin::GPIO_TMC_GPIO18,
        .faultStatusPin = GpioPin::GPIO_TMC_nFAULT_STATUS,
        .driverEnablePin = GpioPin::GPIO_TMC_DRV_EN,
        .resetControlPin = GpioPin::GPIO_TMC_RST_CTRL,
        .spiCommEnablePin = GpioPin::GPIO_TMC_SPI_COMM_nEN,
        .wakeControlPin = GpioPin::GPIO_TMC_nWAKE_CTRL
    };
    
    // Set up ADC configuration
    Tmc9660AdcConfig adcConfig = {
        .ain1Sensor = AdcInputSensor::ADC_TMC9660_AIN1,
        .ain2Sensor = AdcInputSensor::ADC_TMC9660_AIN2,
        .ain3Sensor = AdcInputSensor::ADC_TMC9660_AIN3
    };
    
    // Register the chip
    if (!controller.RegisterTmc9660Chip(Tmc9660ChipId::TMC9660_CHIP_1, config, gpioConfig, adcConfig)) {
        console_error(TAG, "Failed to register primary TMC9660 chip");
        return false;
    }
    
    console_info(TAG, "Primary TMC9660 chip initialized successfully");
    return true;
}

bool Tmc9660SystemHealthy() noexcept {
    Tmc9660MotorController& controller = Tmc9660MotorController::GetInstance();
    return controller.GetSystemHealth();
}
