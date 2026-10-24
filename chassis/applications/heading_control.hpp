#ifndef APPLICATIONS_HEADING_CONTROL_HPP
#define APPLICATIONS_HEADING_CONTROL_HPP

#include <cstdint>

#include "imu_task.hpp"

struct HeadingControl
{
  float target_yaw = 0.0f;
  float error = 0.0f;
  float correction = 0.0f;
  bool locked = false;
  bool turning = false;
  bool settling = false;
  uint32_t settled_since_ms = 0;
};

// 输入遥控角速度，输出解算用角速度；平移、停车不重置已锁定的航向。
float update_heading_control(
  float manual_wz, const ImuHeading & feedback, bool enabled, uint32_t now_ms,
  HeadingControl & state);

#endif  // APPLICATIONS_HEADING_CONTROL_HPP
