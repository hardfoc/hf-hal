idf_component_register(
    SRCS
        "AdcData.cpp"
        "GpioData.cpp"
        "SystemInit.cpp"
        "HardFocIntegration.cpp"
        "Tmc9660MotorController.cpp"
        "Tmc9660Example.cpp"
    
    INCLUDE_DIRS
        "."
        
    REQUIRES
        driver
        esp_common
        freertos
        log
        spi_flash
        
    PRIV_REQUIRES
        # Private requirements for internal components
)
