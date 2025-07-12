/**
 * @file TestManager.h
 * @brief Simple test management utilities for HardFOC HAL tests.
 *
 * This file provides basic test management functionality including
 * test result tracking, timing, and reporting utilities.
 * 
 * @author HardFOC Development Team
 * @date 2024
 * @version 1.0
 */

#ifndef TEST_MANAGER_H
#define TEST_MANAGER_H

#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <iostream>
#include <iomanip>

/**
 * @brief Basic test result structure
 */
struct TestResult {
    std::string testName;
    bool passed;
    std::string errorMessage;
    std::chrono::microseconds executionTime;
    
    TestResult(const std::string& name = "", bool success = false, 
               const std::string& error = "", 
               std::chrono::microseconds time = std::chrono::microseconds(0))
        : testName(name), passed(success), errorMessage(error), executionTime(time) {}
};

/**
 * @brief Simple test manager for organizing and running tests
 */
class TestManager {
public:
    /**
     * @brief Constructor
     */
    TestManager(const std::string& suiteName = "Test Suite") 
        : suiteName_(suiteName), totalTests_(0), passedTests_(0) {}
    
    /**
     * @brief Add and run a test
     * @param testName Name of the test
     * @param testFunc Test function to execute
     * @return Test result
     */
    TestResult RunTest(const std::string& testName, std::function<bool()> testFunc) {
        auto start = std::chrono::steady_clock::now();
        
        bool success = false;
        std::string errorMessage;
        
        try {
            success = testFunc();
        } catch (const std::exception& e) {
            success = false;
            errorMessage = e.what();
        } catch (...) {
            success = false;
            errorMessage = "Unknown exception";
        }
        
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        TestResult result(testName, success, errorMessage, duration);
        results_.push_back(result);
        
        totalTests_++;
        if (success) {
            passedTests_++;
        }
        
        LogTestResult(result);
        return result;
    }
    
    /**
     * @brief Get all test results
     * @return Vector of test results
     */
    const std::vector<TestResult>& GetResults() const {
        return results_;
    }
    
    /**
     * @brief Get test statistics
     * @return True if all tests passed
     */
    bool AllTestsPassed() const {
        return passedTests_ == totalTests_;
    }
    
    /**
     * @brief Get number of passed tests
     */
    size_t GetPassedCount() const {
        return passedTests_;
    }
    
    /**
     * @brief Get total number of tests
     */
    size_t GetTotalCount() const {
        return totalTests_;
    }
    
    /**
     * @brief Print summary report
     */
    void PrintSummary() const {
        std::cout << "\n" << suiteName_ << " Summary:\n";
        std::cout << std::string(suiteName_.length() + 9, '=') << "\n";
        std::cout << "Total Tests: " << totalTests_ << "\n";
        std::cout << "Passed: " << passedTests_ << "\n";
        std::cout << "Failed: " << (totalTests_ - passedTests_) << "\n";
        std::cout << "Success Rate: " << std::fixed << std::setprecision(1) 
                  << (100.0 * passedTests_ / totalTests_) << "%\n";
        
        // Calculate total execution time
        auto totalTime = std::chrono::microseconds(0);
        for (const auto& result : results_) {
            totalTime += result.executionTime;
        }
        std::cout << "Total Execution Time: " 
                  << std::chrono::duration_cast<std::chrono::milliseconds>(totalTime).count() 
                  << "ms\n";
        
        // List failed tests
        bool hasFailures = false;
        for (const auto& result : results_) {
            if (!result.passed) {
                if (!hasFailures) {
                    std::cout << "\nFailed Tests:\n";
                    hasFailures = true;
                }
                std::cout << "  " << result.testName;
                if (!result.errorMessage.empty()) {
                    std::cout << ": " << result.errorMessage;
                }
                std::cout << "\n";
            }
        }
        std::cout << "\n";
    }
    
    /**
     * @brief Clear all results
     */
    void Reset() {
        results_.clear();
        totalTests_ = 0;
        passedTests_ = 0;
    }

private:
    /**
     * @brief Log individual test result
     */
    void LogTestResult(const TestResult& result) {
        std::cout << "  " << std::setw(40) << std::left << result.testName 
                  << ": " << (result.passed ? "PASS" : "FAIL");
        
        if (result.executionTime.count() > 0) {
            std::cout << " [" << result.executionTime.count() << "μs]";
        }
        
        if (!result.passed && !result.errorMessage.empty()) {
            std::cout << " (" << result.errorMessage << ")";
        }
        
        std::cout << "\n";
    }
    
private:
    std::string suiteName_;
    std::vector<TestResult> results_;
    size_t totalTests_;
    size_t passedTests_;
};

/**
 * @brief Assertion macros for tests
 */
#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            throw std::runtime_error("Assertion failed: " #condition); \
        } \
    } while(0)

#define ASSERT_FALSE(condition) \
    do { \
        if (condition) { \
            throw std::runtime_error("Assertion failed: " #condition " should be false"); \
        } \
    } while(0)

#define ASSERT_EQ(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            throw std::runtime_error("Assertion failed: " #expected " != " #actual); \
        } \
    } while(0)

#define ASSERT_NE(expected, actual) \
    do { \
        if ((expected) == (actual)) { \
            throw std::runtime_error("Assertion failed: " #expected " == " #actual); \
        } \
    } while(0)

#define ASSERT_NULL(ptr) \
    do { \
        if ((ptr) != nullptr) { \
            throw std::runtime_error("Assertion failed: " #ptr " is not null"); \
        } \
    } while(0)

#define ASSERT_NOT_NULL(ptr) \
    do { \
        if ((ptr) == nullptr) { \
            throw std::runtime_error("Assertion failed: " #ptr " is null"); \
        } \
    } while(0)

#endif // TEST_MANAGER_H
