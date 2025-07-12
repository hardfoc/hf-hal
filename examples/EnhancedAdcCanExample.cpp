/**
 * @file EnhancedAdcCanExample.cpp
 * @brief Example demonstrating enhanced ADC calibration and CAN-FD capabilities
 * 
 * This example shows how to use the new calibration features for ADC and
 * CAN-FD capabilities for automotive and industrial applications.
 */

#include "McuAdc.h"
#include "McuCan.h"
#include "McuDigitalGpio.h"
#include "NvsStorage.h"
#include <memory>
#include <vector>

//==============================================================================
// ADC CALIBRATION EXAMPLE
//==============================================================================

class AdcCalibrationExample {
public:
    AdcCalibrationExample(std::unique_ptr<BaseAdc> adc) 
        : adc_(std::move(adc)), storage_("adc_cal") {}
    
    /**
     * @brief Perform two-point calibration for high-precision measurements
     */
    bool PerformTwoPointCalibration(uint8_t channel) {
        printf("Starting two-point calibration for channel %d\n", channel);
        
        BaseAdc::CalibrationConfig config;
        config.type = BaseAdc::CalibrationType::TwoPoint;
        config.num_points = 2;
        config.temperature_compensation = true;
        config.temp_coefficient_ppm_c = 50.0f; // 50 ppm/°C typical
        
        // Point 1: 0V reference
        printf("Apply 0V to channel and press Enter...\n");
        getchar();
        uint32_t zero_reading;
        float zero_voltage;
        if (adc_->ReadChannel(channel, zero_reading, zero_voltage) != HfAdcErr::ADC_SUCCESS) {
            printf("Failed to read zero reference\n");
            return false;
        }
        config.points[0] = BaseAdc::CalibrationPoint(0.0f, zero_reading, 25.0f);
        
        // Point 2: Full-scale reference (e.g., 3.3V)
        printf("Apply 3.300V to channel and press Enter...\n");
        getchar();
        uint32_t fs_reading;
        float fs_voltage;
        if (adc_->ReadChannel(channel, fs_reading, fs_voltage) != HfAdcErr::ADC_SUCCESS) {
            printf("Failed to read full-scale reference\n");
            return false;
        }
        config.points[1] = BaseAdc::CalibrationPoint(3.300f, fs_reading, 25.0f);
        
        // Perform calibration with progress callback
        auto progress_callback = [](uint8_t ch, float progress, const char* step, void* user_data) {
            printf("Channel %d: %.1f%% - %s\n", ch, progress, step);
        };
        
        HfAdcErr result = adc_->CalibrateChannel(channel, config, progress_callback);
        if (result != HfAdcErr::ADC_SUCCESS) {
            printf("Calibration failed: %s\n", HfAdcErrToString(result).data());
            return false;
        }
        
        // Save calibration to NVS
        if (adc_->SaveCalibration(channel) != HfAdcErr::ADC_SUCCESS) {
            printf("Warning: Failed to save calibration to NVS\n");
        }
        
        // Verify calibration accuracy
        return VerifyCalibration(channel);
    }
    
    /**
     * @brief Verify calibration accuracy using known references
     */
    bool VerifyCalibration(uint8_t channel) {
        printf("Verifying calibration accuracy...\n");
        
        std::vector<float> test_voltages = {0.500f, 1.650f, 2.500f, 3.100f};
        
        for (float reference : test_voltages) {
            printf("Apply %.3fV and press Enter...\n", reference);
            getchar();
            
            float measured_voltage;
            float error_percent;
            
            HfAdcErr result = adc_->VerifyCalibration(channel, reference, measured_voltage, error_percent);
            if (result == HfAdcErr::ADC_SUCCESS) {
                printf("Reference: %.3fV, Measured: %.3fV, Error: %.2f%%\n", 
                       reference, measured_voltage, error_percent);
                
                if (fabs(error_percent) > 0.1f) {  // > 0.1% error
                    printf("Warning: High measurement error detected\n");
                }
            } else {
                printf("Verification failed for %.3fV\n", reference);
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * @brief Demonstrate advanced multi-point calibration
     */
    bool PerformMultiPointCalibration(uint8_t channel) {
        printf("Starting multi-point calibration for channel %d\n", channel);
        
        BaseAdc::CalibrationConfig config;
        config.type = BaseAdc::CalibrationType::MultiPoint;
        config.num_points = 5;
        config.enable_drift_detection = true;
        config.max_drift_threshold = 0.05f; // 0.05% maximum drift
        
        std::vector<float> reference_voltages = {0.0f, 0.825f, 1.650f, 2.475f, 3.300f};
        
        for (size_t i = 0; i < reference_voltages.size(); ++i) {
            printf("Apply %.3fV to channel and press Enter...\n", reference_voltages[i]);
            getchar();
            
            uint32_t reading;
            float voltage;
            if (adc_->ReadChannel(channel, reading, voltage) == HfAdcErr::ADC_SUCCESS) {
                config.points[i] = BaseAdc::CalibrationPoint(reference_voltages[i], reading, 25.0f);
                printf("Point %zu: %.3fV -> %u counts\n", i+1, reference_voltages[i], reading);
            } else {
                printf("Failed to read point %zu\n", i+1);
                return false;
            }
        }
        
        // Perform calibration
        HfAdcErr result = adc_->CalibrateChannel(channel, config);
        if (result != HfAdcErr::ADC_SUCCESS) {
            printf("Multi-point calibration failed: %s\n", HfAdcErrToString(result).data());
            return false;
        }
        
        printf("Multi-point calibration completed successfully\n");
        return true;
    }
    
private:
    std::unique_ptr<BaseAdc> adc_;
    NvsStorage storage_;
};

//==============================================================================
// CAN-FD EXAMPLE
//==============================================================================

class CanFdExample {
public:
    CanFdExample(std::unique_ptr<BaseCan> can) : can_(std::move(can)) {}
    
    /**
     * @brief Configure CAN-FD with optimal settings for automotive applications
     */
    bool ConfigureCanFD() {
        if (!can_->SupportsCanFD()) {
            printf("CAN-FD not supported by this implementation\n");
            return false;
        }
        
        // Enable CAN-FD with 500kbps nominal, 2Mbps data rate
        if (!can_->SetCanFDMode(true, 2000000, true)) {
            printf("Failed to enable CAN-FD mode\n");
            return false;
        }
        
        // Configure advanced timing for high-speed data phase
        if (!can_->ConfigureCanFDTiming(
            10, 13, 2,  // Nominal: 500kbps @ 40MHz
            5, 3, 1,    // Data: 2Mbps @ 40MHz  
            1           // SJW
        )) {
            printf("Failed to configure CAN-FD timing\n");
            return false;
        }
        
        // Set Transmitter Delay Compensation for high-speed operation
        if (!can_->SetTransmitterDelayCompensation(14, 8)) {
            printf("Failed to set TDC\n");
            return false;
        }
        
        // Set enhanced receive callback for detailed information
        auto canfd_callback = [this](const CanMessage& msg, const CanReceptionInfo& info) {
            ProcessCanFdMessage(msg, info);
        };
        
        if (!can_->SetReceiveCallbackFD(canfd_callback)) {
            printf("Failed to set CAN-FD callback\n");
            return false;
        }
        
        printf("CAN-FD configured successfully\n");
        return true;
    }
    
    /**
     * @brief Send various CAN-FD message types
     */
    void SendCanFdMessages() {
        // Classic CAN message for compatibility
        CanMessage classic_msg;
        classic_msg.id = 0x123;
        classic_msg.format = CanFrameFormat::Classic;
        classic_msg.data_length = 8;
        classic_msg.dlc = 8;
        for (int i = 0; i < 8; ++i) {
            classic_msg.data[i] = i;
        }
        
        printf("Sending classic CAN message...\n");
        can_->SendMessage(classic_msg);
        
        // CAN-FD message without BRS (up to 64 bytes at nominal rate)
        CanMessage canfd_std;
        canfd_std.id = 0x456;
        canfd_std.format = CanFrameFormat::FD_Standard;
        canfd_std.data_length = 32;
        canfd_std.dlc = CanMessage::DataLengthToDLC(32);
        for (int i = 0; i < 32; ++i) {
            canfd_std.data[i] = i % 256;
        }
        
        printf("Sending CAN-FD standard message (32 bytes)...\n");
        can_->SendMessage(canfd_std);
        
        // CAN-FD message with BRS (high-speed data phase)
        CanMessage canfd_brs;
        canfd_brs.id = 0x789;
        canfd_brs.format = CanFrameFormat::FD_BRS;
        canfd_brs.data_length = 64;
        canfd_brs.dlc = 15; // DLC 15 = 64 bytes
        canfd_brs.bit_rate_switch = true;
        
        // Fill with test pattern
        for (int i = 0; i < 64; ++i) {
            canfd_brs.data[i] = (i * 3) % 256;
        }
        
        printf("Sending CAN-FD BRS message (64 bytes)...\n");
        can_->SendMessage(canfd_brs);
    }
    
    /**
     * @brief Send a batch of CAN-FD messages for high-throughput applications
     */
    void SendMessageBatch() {
        std::vector<CanMessage> messages(10);
        
        // Create batch of different message types
        for (size_t i = 0; i < messages.size(); ++i) {
            messages[i].id = 0x200 + i;
            messages[i].format = (i % 2 == 0) ? CanFrameFormat::FD_BRS : CanFrameFormat::Classic;
            messages[i].data_length = (i % 2 == 0) ? 32 : 8;
            messages[i].dlc = CanMessage::DataLengthToDLC(messages[i].data_length);
            messages[i].bit_rate_switch = (i % 2 == 0);
            
            // Fill with test data
            for (uint8_t j = 0; j < messages[i].data_length; ++j) {
                messages[i].data[j] = (i * 16 + j) % 256;
            }
        }
        
        printf("Sending batch of %zu messages...\n", messages.size());
        uint32_t sent = can_->SendMessageBatch(messages.data(), messages.size(), 1000);
        printf("Successfully sent %u out of %zu messages\n", sent, messages.size());
    }
    
    /**
     * @brief Monitor CAN-FD bus and display advanced statistics
     */
    void MonitorCanFdBus() {
        CanBusStatus status;
        if (can_->GetStatus(status)) {
            printf("\n=== CAN-FD Bus Status ===\n");
            printf("CAN-FD Enabled: %s\n", status.canfd_enabled ? "Yes" : "No");
            printf("BRS Enabled: %s\n", status.canfd_brs_enabled ? "Yes" : "No");
            printf("Nominal Baudrate: %u bps\n", status.nominal_baudrate);
            printf("Data Baudrate: %u bps\n", status.data_baudrate);
            printf("CAN-FD TX Count: %u\n", status.canfd_tx_count);
            printf("CAN-FD RX Count: %u\n", status.canfd_rx_count);
            printf("BRS TX Count: %u\n", status.brs_tx_count);
            printf("BRS RX Count: %u\n", status.brs_rx_count);
            printf("Form Errors: %u\n", status.form_errors);
            printf("Stuff Errors: %u\n", status.stuff_errors);
            printf("CRC Errors: %u\n", status.crc_errors);
            printf("Bit Errors: %u\n", status.bit_errors);
            printf("ACK Errors: %u\n", status.ack_errors);
            printf("Bus Off: %s\n", status.bus_off ? "Yes" : "No");
            printf("========================\n\n");
        }
    }
    
private:
    std::unique_ptr<BaseCan> can_;
    
    void ProcessCanFdMessage(const CanMessage& msg, const CanReceptionInfo& info) {
        printf("Received %s message ID: 0x%03X, Length: %u bytes\n",
               (msg.format == CanFrameFormat::Classic) ? "CAN" : "CAN-FD",
               msg.id, msg.data_length);
        
        if (msg.format != CanFrameFormat::Classic) {
            printf("  BRS: %s, ESI: %s\n", 
                   msg.bit_rate_switch ? "Yes" : "No",
                   msg.error_state_indicator ? "Yes" : "No");
        }
        
        printf("  Timestamp: %u us, FIFO Level: %u\n", 
               info.timestamp_us, info.rx_fifo_level);
               
        if (info.data_phase_error) {
            printf("  Warning: Data phase error detected\n");
        }
    }
};

//==============================================================================
// MAIN EXAMPLE APPLICATION
//==============================================================================

void RunEnhancedAdcCanExample() {
    printf("HardFOC Enhanced ADC Calibration & CAN-FD Example\n");
    printf("================================================\n\n");
    
    // Initialize ADC with calibration support
    auto adc = std::make_unique<McuAdc>(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_BITWIDTH_12);
    if (!adc->EnsureInitialized()) {
        printf("Failed to initialize ADC\n");
        return;
    }
    
    // Initialize CAN with FD support
    CanBusConfig can_config;
    can_config.tx_pin = GPIO_NUM_21;
    can_config.rx_pin = GPIO_NUM_22;
    can_config.baudrate = 500000;  // 500 kbps nominal
    can_config.enable_canfd = true;
    can_config.data_baudrate = 2000000;  // 2 Mbps data rate
    can_config.enable_brs = true;
    
    auto can = std::make_unique<McuCan>(can_config);
    if (!can->EnsureInitialized()) {
        printf("Failed to initialize CAN\n");
        return;
    }
    
    // Run ADC calibration examples
    printf("=== ADC Calibration Examples ===\n");
    AdcCalibrationExample adc_example(std::move(adc));
    
    printf("1. Two-point calibration\n");
    if (adc_example.PerformTwoPointCalibration(0)) {
        printf("Two-point calibration successful!\n\n");
    }
    
    printf("2. Multi-point calibration\n");
    if (adc_example.PerformMultiPointCalibration(1)) {
        printf("Multi-point calibration successful!\n\n");
    }
    
    // Run CAN-FD examples
    printf("=== CAN-FD Examples ===\n");
    CanFdExample canfd_example(std::move(can));
    
    if (canfd_example.ConfigureCanFD()) {
        printf("1. Sending individual CAN-FD messages\n");
        canfd_example.SendCanFdMessages();
        
        printf("\n2. Sending message batch\n");
        canfd_example.SendMessageBatch();
        
        printf("\n3. Monitoring bus status\n");
        canfd_example.MonitorCanFdBus();
    }
    
    printf("Example completed!\n");
}

// Entry point for demonstration
int main() {
    RunEnhancedAdcCanExample();
    return 0;
}
