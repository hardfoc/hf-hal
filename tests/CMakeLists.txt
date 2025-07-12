# HardFOC Consolidated Test Suite
cmake_minimum_required(VERSION 3.16)

project(HardFOC_Tests)

# Add test subdirectories
add_subdirectory(component-handler)
add_subdirectory(interface-wrapper)
add_subdirectory(external-drivers)

# Optional: Create a master test runner
# add_executable(master_test_runner master_test_runner.cpp)
# target_link_libraries(master_test_runner 
#     component_handler_tests 
#     interface_wrapper_tests 
#     external_driver_tests
# )
