#ifndef SCHOOL_PM02_HPP
#define SCHOOL_PM02_HPP

#include <array>
#include <cstddef>

#include "referee/pm02/pm02.hpp"

class SchoolPm02 : public sp::PM02
{
public:
  explicit SchoolPm02(UART_HandleTypeDef * uart) : sp::PM02(uart) {}

  // ---------- 接收与解析 ----------
  void request();
  void update(uint16_t size);
  void reset_receive();

  // ---------- 状态观测 ----------
  // 仅在收到校验通过、长度正确的机器人状态帧时更新。
  uint32_t status_count = 0;
  uint32_t status_last_rx_ms = 0;

private:
  // DMA本次接收缓存，以及用于跨批次拼帧的缓存。
  std::array<uint8_t, sp::PM02_BUFF_SIZE> rx_{};
  std::array<uint8_t, sp::PM02_BUFF_SIZE * 2> pending_{};
  size_t pending_size_ = 0;

  void parse_frame(const uint8_t * frame, size_t data_len);
};

#endif