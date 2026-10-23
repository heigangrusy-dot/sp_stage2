#include <cstring>

#include "school_pm02.hpp"
#include "tools/crc/crc.hpp"

// ---------- 启动DMA接收 ----------
void SchoolPm02::request()
{
  const auto result =
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_.data(), static_cast<uint16_t>(rx_.size()));

  if (result == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
  }
}

// ---------- 通信异常后丢弃残留半帧 ----------
void SchoolPm02::reset_receive()
{
  pending_size_ = 0;
  request();
}

// ---------- 接收数据拼接与拆帧 ----------
void SchoolPm02::update(uint16_t size)
{
  if (size == 0) return;

  if (size > rx_.size()) {
    pending_size_ = 0;
    return;
  }

  if (pending_size_ + size > pending_.size()) {
    pending_size_ = 0;
  }

  std::memcpy(pending_.data() + pending_size_, rx_.data(), size);
  pending_size_ += size;

  size_t offset = 0;

  while (pending_size_ - offset >= sp::referee::HEAD_LEN) {
    const uint8_t * frame = pending_.data() + offset;

    // 帧头CRC通过后，才能相信其中的长度字段。
    if (frame[0] != sp::referee::SOF || !sp::check_crc8(frame, sp::referee::HEAD_LEN)) {
      ++offset;
      continue;
    }

    const size_t data_len = static_cast<size_t>(frame[1]) | (static_cast<size_t>(frame[2]) << 8);

    const size_t frame_len =
      sp::referee::HEAD_LEN + sp::referee::CMD_ID_LEN + data_len + sp::referee::TAIL_LEN;

    if (frame_len > sp::PM02_BUFF_SIZE) {
      ++offset;
      continue;
    }

    // 尚未收完整，留到下一次接收后继续处理。
    if (pending_size_ - offset < frame_len) break;

    if (!sp::check_crc16(frame, frame_len)) {
      ++offset;
      continue;
    }

    parse_frame(frame, data_len);
    offset += frame_len;
  }

  // 移除已处理数据，保留末尾的半帧。
  if (offset != 0) {
    pending_size_ -= offset;
    std::memmove(pending_.data(), pending_.data() + offset, pending_size_);
  }
}

// ---------- 按命令解析有效数据 ----------
void SchoolPm02::parse_frame(const uint8_t * frame, size_t data_len)
{
  const uint16_t command = static_cast<uint16_t>(frame[5]) | (static_cast<uint16_t>(frame[6]) << 8);

  const uint8_t * data = frame + sp::referee::DATA_START;

  if (command == sp::referee::cmd_id::ROBOT_STATUS) {
    if (data_len == 13) {
      // 实测13字节版本：前12字节与原结构一致。
      std::memcpy(&robot_status, data, 12);

      // 此版本未提供弹速上限。
      robot_status.bullet_speed_limit = 0.0f;

      const uint8_t flags = data[12];
      robot_status.power_management_gimbal_output = flags & 1u;
      robot_status.power_management_chassis_output = (flags >> 1) & 1u;
      robot_status.power_management_shooter_output = (flags >> 2) & 1u;
      robot_status.reserved = (flags >> 3) & 0x1Fu;
    }
    else if (data_len == sizeof(robot_status)) {
      // 同时兼容中间件原来的17字节版本。
      std::memcpy(&robot_status, data, sizeof(robot_status));
    }
    else {
      return;
    }

    status_last_rx_ms = HAL_GetTick();
    ++status_count;
  }
  else if (command == sp::referee::cmd_id::POWER_HEAT_DATA && data_len == sizeof(power_heat)) {
    std::memcpy(&power_heat, data, sizeof(power_heat));
  }
}