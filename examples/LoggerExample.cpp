/**
 * @file LoggerExample.cpp
 * @brief Example demonstrating the new Logger system with ASCII art integration.
 *
 * This example shows how to use the new Logger class with formatting
 * capabilities and ASCII art support.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "utils-and-drivers/driver-handlers/Logger.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/AsciiArtGenerator.h"

#include <cstdio>
#include <cstring>

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

void DemonstrateBasicLogging() {
    printf("=== Basic Logging Demo ===\n");
    
    Logger& logger = Logger::GetInstance();
    
    // Initialize logger
    LogConfig config;
    config.level = LogLevel::DEBUG;
    config.enable_colors = true;
    config.enable_effects = true;
    
    if (!logger.Initialize(config)) {
        printf("Failed to initialize logger\n");
        return;
    }
    
    // Basic logging
    logger.Info("DEMO", "This is a basic info message");
    logger.Warn("DEMO", "This is a warning message");
    logger.Error("DEMO", "This is an error message");
    logger.Debug("DEMO", "This is a debug message");
    
    printf("Basic logging demo completed\n\n");
}

void DemonstrateFormattedLogging() {
    printf("=== Formatted Logging Demo ===\n");
    
    Logger& logger = Logger::GetInstance();
    
    // Formatted logging with colors and styles
    logger.Info("DEMO", LogColor::GREEN, LogStyle::BOLD, "This is bold green text");
    logger.Warn("DEMO", LogColor::YELLOW, LogStyle::UNDERLINE, "This is underlined yellow text");
    logger.Error("DEMO", LogColor::BRIGHT_RED, LogStyle::BOLD, "This is bold bright red text");
    logger.Debug("DEMO", LogColor::CYAN, LogStyle::ITALIC, "This is italic cyan text");
    
    printf("Formatted logging demo completed\n\n");
}

void DemonstrateAsciiArtLogging() {
    printf("=== ASCII Art Logging Demo ===\n");
    
    Logger& logger = Logger::GetInstance();
    AsciiArtGenerator generator;
    
    // Generate ASCII art
    std::string art = generator.Generate("HELLO");
    
    // Log ASCII art with different formatting
    AsciiArtFormat format;
    
    // Simple ASCII art
    logger.LogAsciiArt("ART", art);
    
    // Colored ASCII art
    format.color = LogColor::BRIGHT_GREEN;
    format.style = LogStyle::BOLD;
    logger.LogAsciiArt("ART", art, format);
    
    // Centered and bordered ASCII art
    format.center_art = true;
    format.add_border = true;
    format.border_char = '*';
    format.border_padding = 2;
    format.color = LogColor::BRIGHT_CYAN;
    logger.LogAsciiArt("ART", art, format);
    
    printf("ASCII art logging demo completed\n\n");
}

void DemonstrateBannerLogging() {
    printf("=== Banner Logging Demo ===\n");
    
    Logger& logger = Logger::GetInstance();
    AsciiArtGenerator generator;
    
    // Generate banner text
    std::string banner_art = generator.Generate("HARDFOC");
    
    // Log as banner (automatically styled)
    logger.LogBanner("BANNER", banner_art);
    
    // Custom banner format
    AsciiArtFormat format;
    format.color = LogColor::BRIGHT_MAGENTA;
    format.style = LogStyle::BOLD;
    format.center_art = true;
    format.add_border = true;
    format.border_char = '=';
    format.border_padding = 1;
    
    logger.LogBanner("BANNER", banner_art, format);
    
    printf("Banner logging demo completed\n\n");
}

void DemonstrateLogLevels() {
    printf("=== Log Levels Demo ===\n");
    
    Logger& logger = Logger::GetInstance();
    
    // Set different log levels for different tags
    logger.SetLogLevel("ERROR", LogLevel::ERROR);
    logger.SetLogLevel("WARN", LogLevel::WARN);
    logger.SetLogLevel("INFO", LogLevel::INFO);
    logger.SetLogLevel("DEBUG", LogLevel::DEBUG);
    logger.SetLogLevel("VERBOSE", LogLevel::VERBOSE);
    
    // Test each level
    logger.Error("ERROR", "This should appear");
    logger.Warn("ERROR", "This should NOT appear (level too high)");
    
    logger.Error("WARN", "This should appear");
    logger.Warn("WARN", "This should appear");
    logger.Info("WARN", "This should NOT appear (level too high)");
    
    logger.Error("INFO", "This should appear");
    logger.Warn("INFO", "This should appear");
    logger.Info("INFO", "This should appear");
    logger.Debug("INFO", "This should NOT appear (level too high)");
    
    printf("Log levels demo completed\n\n");
}

void DemonstrateConvenienceMacros() {
    printf("=== Convenience Macros Demo ===\n");
    
    // Using convenience macros
    LOG_INFO("MACRO", "This uses the LOG_INFO macro");
    LOG_WARN("MACRO", "This uses the LOG_WARN macro");
    LOG_ERROR("MACRO", "This uses the LOG_ERROR macro");
    
    LOG_INFO_FORMATTED("MACRO", LogColor::BRIGHT_GREEN, LogStyle::BOLD, 
                      "This uses the LOG_INFO_FORMATTED macro");
    
    printf("Convenience macros demo completed\n\n");
}

void DemonstrateIntegration() {
    printf("=== Integration Demo ===\n");
    
    Logger& logger = Logger::GetInstance();
    AsciiArtGenerator generator;
    
    // Create a complete application startup sequence
    std::string app_name = generator.Generate("HARDFOC");
    std::string version = generator.Generate("V1.0");
    
    // Application banner
    AsciiArtFormat banner_format;
    banner_format.color = LogColor::BRIGHT_CYAN;
    banner_format.style = LogStyle::BOLD;
    banner_format.center_art = true;
    banner_format.add_border = true;
    banner_format.border_char = '=';
    banner_format.border_padding = 1;
    
    logger.LogBanner("STARTUP", app_name, banner_format);
    
    // Version info
    AsciiArtFormat version_format;
    version_format.color = LogColor::BRIGHT_GREEN;
    version_format.center_art = true;
    
    logger.LogAsciiArt("VERSION", version, version_format);
    
    // Status messages
    logger.Info("STARTUP", LogColor::GREEN, LogStyle::BOLD, "System initialized successfully");
    logger.Info("STARTUP", LogColor::YELLOW, LogStyle::NORMAL, "Loading configuration...");
    logger.Info("STARTUP", LogColor::GREEN, LogStyle::NORMAL, "Configuration loaded");
    logger.Info("STARTUP", LogColor::YELLOW, LogStyle::NORMAL, "Initializing hardware...");
    logger.Info("STARTUP", LogColor::GREEN, LogStyle::NORMAL, "Hardware initialized");
    logger.Info("STARTUP", LogColor::BRIGHT_GREEN, LogStyle::BOLD, "System ready!");
    
    printf("Integration demo completed\n\n");
}

//==============================================================================
// MAIN FUNCTION
//==============================================================================

extern "C" void app_main(void) {
    printf("=== Logger System Demo ===\n\n");
    
    // Run all demonstrations
    DemonstrateBasicLogging();
    DemonstrateFormattedLogging();
    DemonstrateAsciiArtLogging();
    DemonstrateBannerLogging();
    DemonstrateLogLevels();
    DemonstrateConvenienceMacros();
    DemonstrateIntegration();
    
    printf("=== Logger Demo Completed ===\n");
    printf("Check the output above for various logging examples with formatting and ASCII art\n");
} 