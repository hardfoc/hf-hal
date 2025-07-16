/**
 * @file GpioSystemIntegrationTest.h
 * @brief Header for GPIO system integration tests.
 * 
 * This header declares test functions for validating the comprehensive
 * GPIO data handler system integration.
 */

#ifndef COMPONENT_HANDLER_GPIO_SYSTEM_INTEGRATION_TEST_H_
#define COMPONENT_HANDLER_GPIO_SYSTEM_INTEGRATION_TEST_H_

/**
 * @brief Test basic GPIO system initialization and functionality.
 * @return true if all tests pass, false otherwise.
 */
bool TestGpioSystemIntegration();

/**
 * @brief Test error handling and recovery.
 * @return true if error handling works correctly, false otherwise.
 */
bool TestGpioErrorHandling();

/**
 * @brief Main integration test runner.
 * @return true if all tests pass, false otherwise.
 */
bool RunGpioIntegrationTests();

#endif // COMPONENT_HANDLER_GPIO_SYSTEM_INTEGRATION_TEST_H_
