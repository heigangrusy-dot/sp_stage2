#include "power_meter.hpp"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

namespace
{
constexpr uint32_t POWER_METER_TIMEOUT_MS = 100;

// 当前按协议的无符号数解析，负值编码仍需确认。
float decode_scaled_u16(uint8_t high, uint8_t low)
{
  const uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;
  return raw / 100.0f;
}
}  // namespace

void PowerMeter::update(const uint8_t data[8], uint32_t now_ms)
{
  for (uint32_t i = 0; i < 8; ++i) {
    data_.data[i] = data[i];
  }
  data_.current_a = decode_scaled_u16(data[0], data[1]);
  data_.voltage_v = decode_scaled_u16(data[2], data[3]);
  data_.power_w = decode_scaled_u16(data[4], data[5]);
  data_.last_rx_ms = now_ms;
  data_.has_read = true;
  ++data_.frame_count;
}

PowerMeterData PowerMeter::snapshot() const
{
  PowerMeterData result;
  uint32_t now_ms;

  taskENTER_CRITICAL();
  result = data_;
  now_ms = osKernelSysTick();
  taskEXIT_CRITICAL();

  result.online = result.has_read && (now_ms - result.last_rx_ms < POWER_METER_TIMEOUT_MS);

  return result;
}