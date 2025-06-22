#ifndef COMPONENT_HANDLER_ADC_HANDLER_H_
#define COMPONENT_HANDLER_ADC_HANDLER_H_

#include "AdcData.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/BaseAdc.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/Esp32C6Adc.h"
#include <array>
#include <string_view>
#include <memory>

/**
 * @file AdcHandler.h
 * @brief Legacy compatibility wrapper for the new AdcData system.
 *
 * This wrapper provides backward compatibility with the old AdcHandler interface
 * while using the new comprehensive AdcData system under the hood.
 * 
 * @deprecated This interface is deprecated. Use AdcData directly for new code.
 */

template <std::size_t N> 
class AdcHandler {
public:
  struct ChannelInfo {
    std::string_view name; ///< Logical name
    AdcInputSensor sensor; ///< Mapped sensor ID
    uint8_t channel;       ///< Hardware channel number
  };

  /** 
   * @brief Access the singleton instance.
   * @deprecated Use AdcData::GetInstance() instead.
   */
  static AdcHandler &GetInstance() noexcept {
    static AdcHandler inst;
    return inst;
  }

  AdcHandler(const AdcHandler &) = delete;
  AdcHandler &operator=(const AdcHandler &) = delete;

  /**
   * @brief Initialize the ADC handler and all associated hardware.
   * @return true if initialization successful, false otherwise.
   */
  bool Initialize() noexcept {
    if (initialized_) {
      return true;  // Already initialized
    }

    // Initialize ESP32-C6 internal ADC units
    if (!InitializeEsp32C6Adcs()) {
      return false;
    }

    // Register ESP32-C6 ADC channels with the ADC system
    if (!RegisterEsp32C6Channels()) {
      return false;
    }

    initialized_ = true;
    return true;
  }

  /**
   * @brief Register a channel with a name.
   * @param name Logical channel name.
   * @param adc Reference to a BaseAdc derived driver.
   * @param channel Underlying ADC channel number.
   * @return true on success, false if the handler is full.
   * @deprecated Use AdcData::RegisterAdcChannel() instead.
   */
  bool RegisterChannel(std::string_view name, BaseAdc &adc,
                       uint8_t channel) noexcept;
  /**
   * @brief Read channel voltage.
   * @deprecated Use AdcData::GetVolt() instead.
   */
  HfAdcErr ReadChannelV(std::string_view name, float &v,
                        uint8_t samples = 1,
                        uint32_t delay_ms = 0) noexcept;

  /**
   * @brief Read channel count.
   * @deprecated Use AdcData::GetCount() instead.
   */
  HfAdcErr ReadChannelCount(std::string_view name, uint32_t &count,
                            uint8_t samples = 1,
                            uint32_t delay_ms = 0) noexcept;

  /**
   * @brief Read channel count and voltage.
   * @deprecated Use AdcData::GetCountAndVolt() instead.
   */
  HfAdcErr ReadChannel(std::string_view name, uint32_t &count, float &v,
                       uint8_t samples = 1,
                       uint32_t delay_ms = 0) noexcept;

private:
  AdcHandler() = default;
  /**
   * @brief Initialize ESP32-C6 internal ADC units.
   * @return true if successful, false otherwise.
   */
  bool InitializeEsp32C6Adcs() noexcept {
    try {
      // Create ESP32-C6 ADC Unit 1 (only ADC1 is available on ESP32-C6)
      esp32c6_adc1_ = std::make_unique<Esp32C6Adc>(
        ADC_UNIT_1, 
        ADC_ATTEN_DB_11,  // 0-3.1V range
        ADC_BITWIDTH_12   // 12-bit resolution
      );

      // Initialize the ADC unit
      if (!esp32c6_adc1_->Initialize()) {
        return false;
      }

      return true;
    } catch (...) {
      return false;
    }
  }
  /**
   * @brief Register ESP32-C6 ADC channels with the ADC system.
   * @return true if successful, false otherwise.
   */
  bool RegisterEsp32C6Channels() noexcept {
    AdcData& adcData = AdcData::GetInstance();
    bool success = true;

    // Register ADC1 channels (ADC1_CHANNEL_0 through ADC1_CHANNEL_6)
    if (esp32c6_adc1_) {
      success &= RegisterAdc1Channels(adcData);
    }

    return success;
  }
  /**
   * @brief Register ESP32-C6 ADC Unit 1 channels.
   * @param adcData Reference to ADC data system.
   * @return true if successful, false otherwise.
   */
  bool RegisterAdc1Channels(AdcData& adcData) noexcept {
    bool success = true;

    // ESP32-C6 ADC1 has channels 0-6
    const std::array<std::pair<AdcInputSensor, uint8_t>, 7> adc1Channels = {{
      {AdcInputSensor::ADC_INTERNAL_CH0, 0},
      {AdcInputSensor::ADC_INTERNAL_CH1, 1},
      {AdcInputSensor::ADC_INTERNAL_CH2, 2},
      {AdcInputSensor::ADC_INTERNAL_CH3, 3},
      {AdcInputSensor::ADC_INTERNAL_CH4, 4},
      {AdcInputSensor::ADC_INTERNAL_CH5, 5},
      {AdcInputSensor::ADC_INTERNAL_CH6, 6}
    }};

    for (const auto& [sensor, channel] : adc1Channels) {
      HfAdcErr result = adcData.RegisterAdcChannel(sensor, *esp32c6_adc1_, channel);
      success &= (result == HfAdcErr::ADC_SUCCESS);
    }

    return success;
  }
  ChannelInfo *Find(std::string_view name) noexcept {
    for (std::size_t i = 0; i < count_; ++i)
      if (channels_[i].name == name)
        return &channels_[i];
    return nullptr;
  }

  /**
   * @brief Map a channel name to an AdcInputSensor.
   * @param name The channel name.
   * @return Mapped sensor ID.
   */
  AdcInputSensor MapNameToSensor(std::string_view name) noexcept;

  std::array<ChannelInfo, N> channels_{};
  std::size_t count_ = 0;
    // ESP32-C6 ADC management
  bool initialized_ = false;                          ///< Initialization status
  std::unique_ptr<Esp32C6Adc> esp32c6_adc1_;        ///< ESP32-C6 ADC Unit 1 (only ADC1 exists)
};

template <std::size_t N>
bool AdcHandler<N>::RegisterChannel(std::string_view name, BaseAdc &adc,
                                    uint8_t channel) noexcept {
  if (count_ >= N)
    return false;
      // Map the name to a sensor ID
  AdcInputSensor sensor = MapNameToSensor(name);
  
  // Register with the new AdcData system
  AdcData& adcData = AdcData::GetInstance();
  HfAdcErr result = adcData.RegisterAdcChannel(sensor, adc, channel);
  if (result != HfAdcErr::ADC_SUCCESS) {
    return false;
  }
  
  // Store in our local table for name lookup
  channels_[count_++] = {name, sensor, channel};
  return true;
}

template <std::size_t N>
HfAdcErr AdcHandler<N>::ReadChannelV(std::string_view name, float &v,
                                     uint8_t samples,
                                     uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    
  AdcData& adcData = AdcData::GetInstance();
  return adcData.GetVolt(info->sensor, v, samples, delay_ms, TimeUnit::TIME_UNIT_MS);
}

template <std::size_t N>
HfAdcErr
AdcHandler<N>::ReadChannelCount(std::string_view name, uint32_t &count,
                                uint8_t samples, uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    
  AdcData& adcData = AdcData::GetInstance();
  return adcData.GetCount(info->sensor, count, samples, delay_ms, TimeUnit::TIME_UNIT_MS);
}

template <std::size_t N>
HfAdcErr
AdcHandler<N>::ReadChannel(std::string_view name, uint32_t &count, float &v,
                           uint8_t samples, uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return HfAdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    
  AdcData& adcData = AdcData::GetInstance();
  return adcData.GetCountAndVolt(info->sensor, count, v, samples, delay_ms, TimeUnit::TIME_UNIT_MS);
}

template <std::size_t N>
AdcInputSensor AdcHandler<N>::MapNameToSensor(std::string_view name) noexcept {
  // Simple name-to-sensor mapping
  // In a real implementation, this would be more sophisticated
  if (name == "motor_current_a") return AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_A;
  if (name == "motor_current_b") return AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_B;
  if (name == "motor_current_c") return AdcInputSensor::ADC_MOTOR_CURRENT_PHASE_C;
  if (name == "motor_voltage") return AdcInputSensor::ADC_MOTOR_VOLTAGE_BUS;
  if (name == "motor_temp") return AdcInputSensor::ADC_MOTOR_TEMPERATURE;
  if (name == "system_3v3") return AdcInputSensor::ADC_SYSTEM_VOLTAGE_3V3;
  if (name == "system_5v") return AdcInputSensor::ADC_SYSTEM_VOLTAGE_5V;
  if (name == "system_temp") return AdcInputSensor::ADC_SYSTEM_TEMPERATURE;
  if (name == "user_input_1") return AdcInputSensor::ADC_USER_INPUT_1;
  if (name == "user_input_2") return AdcInputSensor::ADC_USER_INPUT_2;
  
  // Default to first internal channel if no match
  return AdcInputSensor::ADC_INTERNAL_CH0;
}

#endif // COMPONENT_HANDLER_ADC_HANDLER_H_
