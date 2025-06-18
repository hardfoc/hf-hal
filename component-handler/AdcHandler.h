#ifndef COMPONENT_HANDLER_ADC_HANDLER_H_
#define COMPONENT_HANDLER_ADC_HANDLER_H_

#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/BaseAdc.h"
#include <array>
#include <string_view>

/**
 * @file AdcHandler.h
 * @brief Aggregates multiple ADC sources and provides named channel access.
 *
 * Use this helper to register ADC channels from various drivers derived from
 * BaseAdc. Channels can then be read by name without worrying about which
 * ADC instance owns them.
 */
template <std::size_t N> class AdcHandler {
public:
  struct ChannelInfo {
    std::string_view name; ///< Logical name
    BaseAdc &adc;          ///< Reference to ADC driver
    uint8_t channel;       ///< Hardware channel number
  };

  /** Access the singleton instance. */
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
   */
  bool RegisterChannel(std::string_view name, BaseAdc &adc,
                       uint8_t channel) noexcept;

  /**
   * @brief Read channel voltage.
   */
  BaseAdc::AdcErr ReadChannelV(std::string_view name, float &v,
                               uint8_t samples = 1,
                               uint32_t delay_ms = 0) noexcept;

  /**
   * @brief Read channel count.
   */
  BaseAdc::AdcErr ReadChannelCount(std::string_view name, uint32_t &count,
                                   uint8_t samples = 1,
                                   uint32_t delay_ms = 0) noexcept;

  /**
   * @brief Read channel count and voltage.
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

  std::array<ChannelInfo, N> channels_{};
  std::size_t count_ = 0;
};

template <std::size_t N>
bool AdcHandler<N>::RegisterChannel(std::string_view name, BaseAdc &adc,
                                    uint8_t channel) noexcept {
  if (count_ >= N)
    return false;
  channels_[count_++] = {name, adc, channel};
  return true;
}

template <std::size_t N>
BaseAdc::AdcErr AdcHandler<N>::ReadChannelV(std::string_view name, float &v,
                                            uint8_t samples,
                                            uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return BaseAdc::AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
  return info->adc.ReadChannelV(info->channel, v, samples, delay_ms);
}

template <std::size_t N>
BaseAdc::AdcErr
AdcHandler<N>::ReadChannelCount(std::string_view name, uint32_t &count,
                                uint8_t samples, uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return BaseAdc::AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
  return info->adc.ReadChannelCount(info->channel, count, samples, delay_ms);
}

template <std::size_t N>
BaseAdc::AdcErr
AdcHandler<N>::ReadChannel(std::string_view name, uint32_t &count, float &v,
                           uint8_t samples, uint32_t delay_ms) noexcept {
  auto info = Find(name);
  if (!info)
    return BaseAdc::AdcErr::ADC_ERR_CHANNEL_NOT_FOUND;
  return info->adc.ReadChannel(info->channel, count, v, samples, delay_ms);
}

#endif // COMPONENT_HANDLER_ADC_HANDLER_H_
