#==============================================================================
# Vortex API CMakeLists.txt
#==============================================================================
# This file configures the build for the Vortex API singleton that provides
# unified access to all component handlers in the HardFOC Vortex platform.
#
# Author: HardFOC Team
# Date: 2025
# Version: 1.0
#==============================================================================

# Set component name
set(COMPONENT_NAME "vortex-api")

# Register the component
idf_component_register(
    SRCS 
        "Vortex.cpp"
    
    INCLUDE_DIRS 
        "."
    
    REQUIRES 
        "component-handlers"
        "driver-handlers"
        "hf-core-utils"
        "hf-core-drivers"
        "esp_timer"
        "freertos"
        "driver"
        "esp_common"
        "esp_system"
        "esp_rom"
        "esp_hw_support"
        "hal"
        "soc"
        "log"
)

#==============================================================================
# COMPILE DEFINITIONS
#==============================================================================

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    VORTEX_API_VERSION_MAJOR=1
    VORTEX_API_VERSION_MINOR=0
    VORTEX_API_VERSION_PATCH=0
    VORTEX_API_VERSION_STRING="1.0.0"
)

#==============================================================================
# COMPILE OPTIONS
#==============================================================================

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
    -fno-exceptions
    -fno-rtti
)

#==============================================================================
# LINK OPTIONS
#==============================================================================

target_link_options(${COMPONENT_LIB} PRIVATE
    -Wl,--gc-sections
    -Wl,--as-needed
)

#==============================================================================
# COMPONENT DEPENDENCIES
#==============================================================================

# Ensure proper dependency order
set_property(TARGET ${COMPONENT_LIB} PROPERTY 
    INTERFACE_COMPONENT_DEPENDENCIES 
        "component-handlers"
        "driver-handlers"
        "hf-core-utils"
        "hf-core-drivers"
)

#==============================================================================
# INSTALLATION
#==============================================================================

# Install headers
install(FILES 
    "Vortex.h"
    DESTINATION include/vortex-api
)

#==============================================================================
# TESTING SUPPORT
#==============================================================================

# Enable testing if requested
if(BUILD_TESTING)
    enable_testing()
    
    # Add test targets here if needed
    # add_executable(vortex_api_test test_vortex_api.cpp)
    # target_link_libraries(vortex_api_test ${COMPONENT_LIB})
    # add_test(NAME VortexApiTest COMMAND vortex_api_test)
endif()

#==============================================================================
# DOCUMENTATION
#==============================================================================

# Generate documentation if Doxygen is available
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    
    add_custom_target(vortex_api_docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating Vortex API documentation with Doxygen"
        VERBATIM
    )
endif()

#==============================================================================
# VERSION INFORMATION
#==============================================================================

# Create version header
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/VortexVersion.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/VortexVersion.h"
    @ONLY
)

#==============================================================================
# BUILD INFORMATION
#==============================================================================

message(STATUS "Configuring Vortex API v${VORTEX_API_VERSION_STRING}")
message(STATUS "  - Component: ${COMPONENT_NAME}")
message(STATUS "  - Source files: Vortex.cpp")
message(STATUS "  - Headers: Vortex.h")
message(STATUS "  - Dependencies: component-handlers, driver-handlers, hf-core-utils, hf-core-drivers") 