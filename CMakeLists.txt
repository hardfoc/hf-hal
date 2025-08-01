###############################################################################
#  HardFOC HAL – **single-component** build script
#  Copy this file to hf-hal/CMakeLists.txt
###############################################################################

cmake_minimum_required(VERSION 3.16)

set(HF_HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})

#==============================================================================
# SYSTEMATIC FILE COLLECTION - VORTEX API DEPENDENCY ANALYSIS
#==============================================================================

# Expose external drivers as components for header access
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/")
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver")

# Include the API directory for Vortex API component
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/API")

#==============================================================================
# 1. SYSTEMATIC SOURCE FILE COLLECTION
#==============================================================================

# API Layer Sources
set(API_SOURCES
    "${HF_HAL_ROOT}/API/Vortex.cpp"
)

# Component Handlers Sources (8 managers)
set(COMPONENT_HANDLER_SOURCES
    "${HF_HAL_ROOT}/component-handlers/CommChannelsManager.cpp"
    "${HF_HAL_ROOT}/component-handlers/GpioManager.cpp"
    "${HF_HAL_ROOT}/component-handlers/AdcManager.cpp"
    "${HF_HAL_ROOT}/component-handlers/MotorController.cpp"
    "${HF_HAL_ROOT}/component-handlers/ImuManager.cpp"
    "${HF_HAL_ROOT}/component-handlers/EncoderManager.cpp"
    "${HF_HAL_ROOT}/component-handlers/LedManager.cpp"
    "${HF_HAL_ROOT}/component-handlers/TemperatureManager.cpp"
)

# Driver Handlers Sources (6 drivers + 1 wrapper)
set(DRIVER_HANDLER_SOURCES
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/Tmc9660Handler.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/Pcal95555Handler.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/As5047uHandler.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/Bno08xHandler.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/NtcTemperatureHandler.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/Logger.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers/Tmc9660AdcWrapper.cpp"
)

# Core Drivers Sources (Interface Wrappers)
set(CORE_DRIVER_SOURCES
    # ESP32 implementations
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspSpi.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspI2c.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspGpio.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspAdc.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspUart.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspCan.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspPwm.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspPio.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspNvs.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspLogger.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspTemperature.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspPeriodicTimer.cpp"
)

# Core Utils Sources
set(CORE_UTILS_SOURCES
    # RTOS wrappers
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/BaseThread.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/CriticalSection.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/FreeRTOSUtils.c"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/Mutex.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/MutexGuard.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/OsUtility.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/PeriodicTimer.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/src/SignalSemaphore.cpp"
    
    # General utilities
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/src/SoftwareVersion.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/src/TimestampedVariable.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/src/CrcCalculator.c"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/src/Utility.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/src/VariableAnomalyMonitor.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/src/VariableMonitor.cpp"
    
    # CAN utilities
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-canopen/src/CanOpenUtils.cpp"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-canopen/src/CanOpenMotorUtils.cpp"
)

# Combine all sources
set(HF_HAL_SRCS
    ${API_SOURCES}
    ${COMPONENT_HANDLER_SOURCES}
    ${DRIVER_HANDLER_SOURCES}
    ${CORE_DRIVER_SOURCES}
    ${CORE_UTILS_SOURCES}
)

#==============================================================================
# 2. SYSTEMATIC INCLUDE DIRECTORY COLLECTION
#==============================================================================

# API include directories
set(API_INCLUDE_DIRS
    "${HF_HAL_ROOT}/API"
)

# Component handlers include directories
set(COMPONENT_HANDLER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/component-handlers"
)

# Driver handlers include directories
set(DRIVER_HANDLER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/utils-and-drivers/driver-handlers"
)

# Core drivers include directories
set(CORE_DRIVER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src"
)

# Core utils include directories
set(CORE_UTILS_INCLUDE_DIRS
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-general/include"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-canopen/include"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-utils/hf-utils-thread-safe/include"
)

# External driver include directories
set(EXTERNAL_DRIVER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-as5047u-driver/src"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-ntc-thermistor-driver/include"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-pcal95555-driver/src"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/register_mode"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode"
    "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include"
)

# Combine all include directories
set(HF_HAL_INC_DIRS
    ${API_INCLUDE_DIRS}
    ${COMPONENT_HANDLER_INCLUDE_DIRS}
    ${DRIVER_HANDLER_INCLUDE_DIRS}
    ${CORE_DRIVER_INCLUDE_DIRS}
    ${CORE_UTILS_INCLUDE_DIRS}
    ${EXTERNAL_DRIVER_INCLUDE_DIRS}
)

#==============================================================================
# 3. COMPONENT REGISTRATION
#==============================================================================

idf_component_register(
    SRCS          ${HF_HAL_SRCS}
    INCLUDE_DIRS  ${HF_HAL_INC_DIRS}
    
    # PUBLIC dependencies: anything the application must also link against
    REQUIRES      
        driver 
        freertos 
        hf-ws2812-rmt-driver 
        nvs_flash
        esp_timer
        esp_system
        esp_common
        esp_rom
        esp_hw_support
        hal
        soc
        log
        esp_adc
        esp_gpio
        esp_i2c
        esp_spi
        esp_can
        esp_uart
        esp_pm
        esp_wifi
        esp_netif
        esp_event
        esp_partition
        esp_ota
        esp_http_client
        esp_https_ota
        esp_websocket_client
        esp_http_server
    
    # PRIVATE dependencies: used only while building this library
    PRIV_REQUIRES 
        hal 
        esp_timer 
        esp_adc 
        esp_driver_spi 
        esp_driver_i2c 
        esp_driver_uart 
        esp_driver_gpio
        esp_driver_can
        esp_driver_pwm
        esp_driver_pio
        esp_driver_nvs
        esp_driver_wifi
        esp_driver_bluetooth
)

#==============================================================================
# 4. COMPILE OPTIONS AND DEFINITIONS
#==============================================================================

# Compile options for the entire HAL
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Wall 
    -Wextra 
    -Werror
    -Wpedantic
    -Werror=return-type
    -Werror=unused-variable
    -Werror=unused-function
    -Werror=unused-parameter
    -Werror=missing-field-initializers
    -Werror=switch
    -Werror=switch-enum
    -Werror=switch-default
    -Werror=implicit-fallthrough
    -Werror=format
    -Werror=format-security
    -Werror=format-nonliteral
    -Werror=cast-align
    -Werror=cast-qual
    -Werror=conversion
    -Werror=sign-conversion
    -Werror=float-conversion
    -Werror=float-equal
    -Werror=shadow
    -Werror=undef
    -Werror=redundant-decls
    -Werror=inline
    -Werror=disabled-optimization
    -Werror=stack-protector
    -Werror=stack-usage=4096
    -Werror=frame-larger-than=4096
    -Werror=larger-than=8192
    -Werror=array-bounds
    -Werror=array-bounds=2
    -Werror=stringop-overflow
    -Werror=stringop-overread
    -Werror=stringop-truncation
    -Werror=restrict
    -Werror=null-dereference
    -Werror=nonnull
    -Werror=nonnull-compare
    -Werror=address
    -Werror=aggressive-loop-optimizations
    -Werror=alloca
    -Werror=builtin-macro-redefined
    -Werror=div-by-zero
    -Werror=double-promotion
    -Werror=duplicated-branches
    -Werror=duplicated-cond
    -Werror=empty-body
    -Werror=enum-compare
    -Werror=enum-conversion
    -Werror=extra
    -Werror=free-nonheap-object
    -Werror=ignored-qualifiers
    -Werror=implicit
    -Werror=implicit-function-declaration
    -Werror=init-self
    -Werror=logical-not-parentheses
    -Werror=logical-op
    -Werror=main
    -Werror=maybe-uninitialized
    -Werror=memset-transposed-args
    -Werror=mismatched-dealloc
    -Werror=mismatched-new-delete
    -Werror=missing-braces
    -Werror=missing-declarations
    -Werror=missing-include-dirs
    -Werror=missing-profile
    -Werror=multichar
    -Werror=multistatement-macros
    -Werror=narrowing
    -Werror=noexcept
    -Werror=non-template-friend
    -Werror=old-style-cast
    -Werror=openmp-simd
    -Werror=overloaded-virtual
    -Werror=packed
    -Werror=packed-bitfield-compat
    -Werror=padded
    -Werror=parentheses
    -Werror=pointer-arith
    -Werror=pointer-sign
    -Werror=reorder
    -Werror=return-local-addr
    -Werror=sequence-point
    -Werror=shift-count-negative
    -Werror=shift-count-overflow
    -Werror=shift-negative-value
    -Werror=shift-overflow
    -Werror=shift-overflow=2
    -Werror=suggest-attribute=const
    -Werror=suggest-attribute=format
    -Werror=suggest-attribute=malloc
    -Werror=suggest-attribute=noreturn
    -Werror=suggest-attribute=pure
    -Werror=suggest-final-methods
    -Werror=suggest-final-types
    -Werror=suggest-override
    -Werror=switch-bool
    -Werror=tautological-compare
    -Werror=trigraphs
    -Werror=type-limits
    -Werror=undef
    -Werror=uninitialized
    -Werror=unknown-pragmas
    -Werror=unreachable-code
    -Werror=unsafe-loop-optimizations
    -Werror=unused
    -Werror=unused-but-set-parameter
    -Werror=unused-but-set-variable
    -Werror=unused-function
    -Werror=unused-label
    -Werror=unused-local-typedefs
    -Werror=unused-macros
    -Werror=unused-parameter
    -Werror=unused-result
    -Werror=unused-value
    -Werror=unused-variable
    -Werror=useless-cast
    -Werror=varargs
    -Werror=variadic-macros
    -Werror=vector-operation-performance
    -Werror=vla
    -Werror=volatile-register-var
    -Werror=write-strings
    -Werror=zero-as-null-pointer-constant
)

# Compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    -DHARDFOC_HAL=1
    -DHARDFOC_VORTEX_V1=1
    -DHARDFOC_COMPONENT_HANDLER=1
    -DHARDFOC_GPIO_MANAGER=1
    -DHARDFOC_ADC_MANAGER=1
    -DHARDFOC_MOTOR_CONTROLLER=1
    -DHARDFOC_IMU_MANAGER=1
    -DHARDFOC_ENCODER_MANAGER=1
    -DHARDFOC_LED_MANAGER=1
    -DHARDFOC_TEMPERATURE_MANAGER=1
    -DHARDFOC_COMM_CHANNELS_MANAGER=1
    -DHARDFOC_PCAL95555_SUPPORT=1
    -DHARDFOC_TMC9660_SUPPORT=1
    -DHARDFOC_AS5047U_SUPPORT=1
    -DHARDFOC_BNO08X_SUPPORT=1
    -DHARDFOC_NTC_THERMISTOR_SUPPORT=1
    -DHARDFOC_WS2812_SUPPORT=1
    -DHARDFOC_PLATFORM_MAPPING=1
    -DHARDFOC_VORTEX_API=1
    -DHARDFOC_LOGGER=1
    -DHARDFOC_RTOS_WRAP=1
    -DHARDFOC_CORE_UTILS=1
    -DHARDFOC_CORE_DRIVERS=1
    -DHARDFOC_DRIVER_HANDLERS=1
)

# Set C++ standard
set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

#==============================================================================
# 5. BUILD INFORMATION
#==============================================================================

# Print build information
message(STATUS "=== HardFOC HAL Build Configuration ===")
message(STATUS "Root Directory: ${HF_HAL_ROOT}")
message(STATUS "API Sources: ${API_SOURCES}")
message(STATUS "Component Handler Sources: ${COMPONENT_HANDLER_SOURCES}")
message(STATUS "Driver Handler Sources: ${DRIVER_HANDLER_SOURCES}")
message(STATUS "Core Driver Sources: ${CORE_DRIVER_SOURCES}")
message(STATUS "Core Utils Sources: ${CORE_UTILS_SOURCES}")
message(STATUS "Total Sources: ${HF_HAL_SRCS}")
message(STATUS "Include Directories: ${HF_HAL_INC_DIRS}")
message(STATUS "Component: ${COMPONENT_LIB}")
message(STATUS "=== End Build Configuration ===")
