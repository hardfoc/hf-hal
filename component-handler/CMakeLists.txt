idf_component_register(
    SRCS        # Modern consolidated manager classes (Phase 1)
        "GpioManager.cpp"
        "AdcManager.cpp"
        
        # System initialization
        "SystemInit.cpp"
        
        # TMC9660 integration
        "Tmc9660MotorController.cpp"
        "TMC9660Controller.cpp"
        "Tmc9660Gpio.cpp"
        
        # PCAL95555 integration
        "Pcal95555Gpio.cpp"        
        # MCU-agnostic drivers
        "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/McuAdc.cpp"
        "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/McuDigitalGpio.cpp"
        # Thread-safe wrappers
        "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/thread_safe/SfAdc.cpp"
        "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/thread_safe/SfGpio.cpp"
        "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/thread_safe/SfI2cBus.cpp"
          # Examples and demos
        "ExampleUsage.cpp"
        "Tmc9660Example.cpp"
        "ModernHalExample.cpp"
        
    INCLUDE_DIRS
        "."
        "../utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include"
        "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/include"
        
    REQUIRES
        driver
        esp_common
        freertos
        log
        spi_flash
        
    PRIV_REQUIRES
        # Private requirements for internal components
)

# Compiler definitions for build configuration
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    # Version information
    HARDFOC_HAL_VERSION_MAJOR=2
    HARDFOC_HAL_VERSION_MINOR=0
    HARDFOC_HAL_VERSION_PATCH=0
)
