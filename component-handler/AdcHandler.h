#ifndef COMPONENT_HANDLER_ADC_HANDLER_H_
#define COMPONENT_HANDLER_ADC_HANDLER_H_

#include "AdcData.h"
#include "CommonIDs.h"
#include "ThingsToString.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/BaseAdc.h"
#include <array>
#include <string_view>

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
  static AdcHandler &Instance() noexcept {
    static AdcHandler inst;
    return inst;
  }

  AdcHandler(const AdcHandler &) = delete;
  AdcHandler &operator=(const AdcHandler &) = delete;

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
  BaseAdc::AdcErr ReadChannelV(std::string_view name, float &v,
                               uint8_t samples = 1,
                               uint32_t delay_ms = 0) noexcept;

  /**
   * @brief Read channel count.
   * @deprecated Use AdcData::GetCount() instead.
   */
  BaseAdc::AdcErr ReadChannelCount(std::string_view name, uint32_t &count,
                                   uint8_t samples = 1,
                                   uint32_t delay_ms = 0) noexcept;

  /**
   * @brief Read channel count and voltage.
   * @deprecated Use AdcData::GetCountAndVolt() instead.
   */
  BaseAdc::AdcErr ReadChannel(std::string_view name, uint32_t &count, float &v,
                              uint8_t samples = 1,
                              uint32_t delay_ms = 0) noexcept;

private:
  AdcHandler() = default;

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
  if (!adcData.RegisterAdcChannel(sensor, adc, channel)) {
    return false;
  }
  
  // Store in our local table for name lookup
  channels_[count_++] = {name, sensor, channel};
  return true;
}

template <std::size_t N>
BaseAdc::AdcErr AdcHandler<N>::ReadChannelV(std::string_view name, float &v,
                                            uint8_t samples,
                                            uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return BaseAdc::AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    
  AdcData& adcData = AdcData::GetInstance();
  bool success = adcData.GetVolt(info->sensor, v, samples, delay_ms, TimeUnit::TIME_UNIT_MS);
  
  return success ? BaseAdc::AdcErr::ADC_SUCCESS : BaseAdc::AdcErr::ADC_ERR_CHANNEL_READ_ERR;
}

template <std::size_t N>
BaseAdc::AdcErr
AdcHandler<N>::ReadChannelCount(std::string_view name, uint32_t &count,
                                uint8_t samples, uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return BaseAdc::AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    
  AdcData& adcData = AdcData::GetInstance();
  bool success = adcData.GetCount(info->sensor, count, samples, delay_ms, TimeUnit::TIME_UNIT_MS);
  
  return success ? BaseAdc::AdcErr::ADC_SUCCESS : BaseAdc::AdcErr::ADC_ERR_CHANNEL_READ_ERR;
}

template <std::size_t N>
BaseAdc::AdcErr
AdcHandler<N>::ReadChannel(std::string_view name, uint32_t &count, float &v,
                           uint8_t samples, uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return BaseAdc::AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
    
  AdcData& adcData = AdcData::GetInstance();
  bool success = adcData.GetCountAndVolt(info->sensor, count, v, samples, delay_ms, TimeUnit::TIME_UNIT_MS);
  
  return success ? BaseAdc::AdcErr::ADC_SUCCESS : BaseAdc::AdcErr::ADC_ERR_CHANNEL_READ_ERR;
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
  if (name == "system_12v") return AdcInputSensor::ADC_SYSTEM_VOLTAGE_12V;
  if (name == "system_temp") return AdcInputSensor::ADC_SYSTEM_TEMPERATURE;
  if (name == "user_input_1") return AdcInputSensor::ADC_USER_INPUT_1;
  if (name == "user_input_2") return AdcInputSensor::ADC_USER_INPUT_2;
  
  // Default to first internal channel if no match
  return AdcInputSensor::ADC_INTERNAL_CH0;
}

#endif // COMPONENT_HANDLER_ADC_HANDLER_H_
