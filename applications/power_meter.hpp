#ifndef APPLICATIONS_POWER_METER_HPP
#define APPLICATIONS_POWER_METER_HPP

#include <cstdint>

struct PowerMeterData
{
  uint8_t data[8] = {};  // 原始报文，方便核对。
  uint32_t frame_count = 0;
  uint32_t last_rx_ms = 0;

  float current_a = 0.0f;
  float voltage_v = 0.0f;
  float power_w = 0.0f;
  bool has_read = false;
  bool online = false;  // 获取快照时判断是否超时。
};

class PowerMeter
{
public:
  void update(const uint8_t data[8], uint32_t now_ms);
  PowerMeterData snapshot() const;

private:
  PowerMeterData data_;
};

#endif  // APPLICATIONS_POWER_METER_HPP