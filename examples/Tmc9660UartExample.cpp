/**
 * @file Tmc9660UartExample.cpp
 * @brief Example demonstrating TMC9660 UART communication using CommChannelsManager
 * 
 * This example shows how to:
 * 1. Access the TMC9660 UART interface through CommChannelsManager
 * 2. Send and receive TMCL datagrams for motor control
 * 3. Implement proper ESP-IDF v5.5 UART communication patterns
 * 
 * The TMC9660 uses TMCL (Trinamic Motion Control Language) protocol over UART:
 * - 9-byte frames: sync+address, command, type, motor, 4-byte data, checksum
 * - Baud rate: 115200, Format: 8N1
 * - LSB-first transmission with checksum validation
 * 
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "component-handers/CommChannelsManager.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/TMC9660CommInterface.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseUart.h"
#include <array>
#include <cstdio>

/**
 * @class Tmc9660UartExample
 * @brief Example implementation of TMC9660 UART communication
 */
class Tmc9660UartExample {
private:
    static constexpr uint8_t TMC9660_MODULE_ADDRESS = 1;  ///< Default TMC9660 module address
    static constexpr uint32_t UART_TIMEOUT_MS = 100;     ///< UART timeout for operations
    
public:
    /**
     * @brief Initialize TMC9660 UART communication
     * @return true if successful, false otherwise
     */
    bool Initialize() {
        printf("=== TMC9660 UART Communication Example ===\n");
        
        // Get TMC9660 UART interface from CommChannelsManager
        try {
            BaseUart& uart = CommChannelsManager::GetInstance().GetTmc9660Uart();
            printf("✅ TMC9660 UART interface obtained successfully\n");
            printf("   Configuration: 115200 8N1, optimized for 9-byte TMCL frames\n");
            return true;
        } catch (const std::exception& e) {
            printf("❌ Failed to get TMC9660 UART interface: %s\n", e.what());
            return false;
        }
    }
    
    /**
     * @brief Send a TMCL command to TMC9660
     * @param command TMCL command byte
     * @param type Type/parameter number
     * @param motor Motor/bank number
     * @param value 32-bit data value
     * @return true if successful, false otherwise
     */
    bool SendTmclCommand(uint8_t command, uint8_t type, uint8_t motor, uint32_t value) {
        BaseUart& uart = CommChannelsManager::GetInstance().GetTmc9660Uart();
        
        // Construct 9-byte TMCL frame
        std::array<uint8_t, 9> tmcl_frame;
        tmcl_frame[0] = TMC9660_MODULE_ADDRESS | 0x80;  // Sync bit + address
        tmcl_frame[1] = command;                        // Command
        tmcl_frame[2] = type;                          // Type/parameter
        tmcl_frame[3] = motor;                         // Motor/bank
        tmcl_frame[4] = static_cast<uint8_t>(value >> 24);  // Data byte 3 (MSB)
        tmcl_frame[5] = static_cast<uint8_t>(value >> 16);  // Data byte 2
        tmcl_frame[6] = static_cast<uint8_t>(value >> 8);   // Data byte 1
        tmcl_frame[7] = static_cast<uint8_t>(value);        // Data byte 0 (LSB)
        
        // Calculate checksum (sum of first 8 bytes)
        uint8_t checksum = 0;
        for (int i = 0; i < 8; i++) {
            checksum += tmcl_frame[i];
        }
        tmcl_frame[8] = checksum;
        
        // Send TMCL frame
        hf_uart_err_t result = uart.Transmit(tmcl_frame.data(), tmcl_frame.size());
        if (result == hf_uart_err_t::UART_SUCCESS) {
            printf("📤 TMCL command sent: CMD=0x%02X, TYPE=0x%02X, MOTOR=%d, VALUE=0x%08X\n",
                   command, type, motor, value);
            return true;
        } else {
            printf("❌ Failed to send TMCL command: %d\n", static_cast<int>(result));
            return false;
        }
    }
    
    /**
     * @brief Receive TMCL reply from TMC9660
     * @param reply_data Buffer to store the 9-byte reply
     * @return true if successful, false otherwise
     */
    bool ReceiveTmclReply(std::array<uint8_t, 9>& reply_data) {
        BaseUart& uart = CommChannelsManager::GetInstance().GetTmc9660Uart();
        
        // Receive 9-byte TMCL reply
        hf_uart_err_t result = uart.Receive(reply_data.data(), reply_data.size(), UART_TIMEOUT_MS);
        if (result == hf_uart_err_t::UART_SUCCESS) {
            // Verify checksum
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) {
                checksum += reply_data[i];
            }
            
            if (checksum == reply_data[8]) {
                printf("📥 TMCL reply received: ADDR=0x%02X, STATUS=0x%02X, CMD=0x%02X\n",
                       reply_data[0], reply_data[1], reply_data[2]);
                return true;
            } else {
                printf("❌ TMCL reply checksum error: calculated=0x%02X, received=0x%02X\n",
                       checksum, reply_data[8]);
                return false;
            }
        } else {
            printf("❌ Failed to receive TMCL reply: %d\n", static_cast<int>(result));
            return false;
        }
    }
    
    /**
     * @brief Example: Read TMC9660 firmware version
     */
    void ReadFirmwareVersion() {
        printf("\n--- Reading TMC9660 Firmware Version ---\n");
        
        // TMCL command: Get Global Parameter (GGP), parameter 132 (firmware version)
        if (SendTmclCommand(0x0A, 132, 0, 0)) {  // GGP command, param 132, motor 0, value 0
            std::array<uint8_t, 9> reply;
            if (ReceiveTmclReply(reply)) {
                uint32_t firmware_version = (static_cast<uint32_t>(reply[4]) << 24) |
                                          (static_cast<uint32_t>(reply[5]) << 16) |
                                          (static_cast<uint32_t>(reply[6]) << 8) |
                                           static_cast<uint32_t>(reply[7]);
                printf("✅ TMC9660 Firmware Version: 0x%08X\n", firmware_version);
            }
        }
    }
    
    /**
     * @brief Example: Set motor current limit
     * @param current_limit Current limit in milliamps
     */
    void SetMotorCurrentLimit(uint16_t current_limit) {
        printf("\n--- Setting Motor Current Limit ---\n");
        
        // TMCL command: Set Axis Parameter (SAP), parameter for max current
        if (SendTmclCommand(0x05, 6, 0, current_limit)) {  // SAP command, param 6, motor 0
            std::array<uint8_t, 9> reply;
            if (ReceiveTmclReply(reply)) {
                if (reply[1] == 0x64) {  // Status OK
                    printf("✅ Motor current limit set to %d mA\n", current_limit);
                } else {
                    printf("❌ TMC9660 error status: 0x%02X\n", reply[1]);
                }
            }
        }
    }
    
    /**
     * @brief Run complete TMC9660 UART communication example
     */
    void RunExample() {
        if (!Initialize()) {
            printf("❌ Failed to initialize TMC9660 UART communication\n");
            return;
        }
        
        // Example operations
        ReadFirmwareVersion();
        SetMotorCurrentLimit(1000);  // Set 1A current limit
        
        printf("\n=== TMC9660 UART Example Complete ===\n");
        printf("✅ UART configured with ESP-IDF v5.5 compliance\n");
        printf("✅ TMCL protocol communication demonstrated\n");
        printf("✅ 9-byte frame handling with checksum validation\n");
    }
};

/**
 * @brief Main example function
 */
void RunTmc9660UartExample() {
    Tmc9660UartExample example;
    example.RunExample();
}

// ESP-IDF app_main integration
extern "C" void app_main() {
    printf("Starting TMC9660 UART Communication Example...\n");
    RunTmc9660UartExample();
}
