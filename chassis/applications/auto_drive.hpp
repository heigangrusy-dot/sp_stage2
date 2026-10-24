#ifndef APPLICATIONS_AUTO_DRIVE_HPP
#define APPLICATIONS_AUTO_DRIVE_HPP

#include <cstdint>

enum class AutoDrivePhase
{
  IDLE,
  RUNNING,
  STOPPING,
  COMPLETE,
  FAULT
};

// ---------- 自动直行状态 ----------
struct AutoDrive
{
  AutoDrivePhase phase = AutoDrivePhase::IDLE;
  bool armed = false;
  float start_position = 0.0f;
  float distance = 0.0f;  // 轮编码器估计前进距离，m，不是绝对定位。
  float speed_command = 0.0f;
  uint32_t started_ms = 0;
  uint32_t last_update_ms = 0;
  uint32_t stopped_since_ms = 0;
  bool settling = false;
};

// selected：上档；allowed：安全条件与IMU有效；rearm：下档且已停稳、系统就绪。
// position为轮侧反馈解算的车体前向位置(m)。
float update_auto_drive(
  bool selected, bool allowed, bool rearm, float position, bool all_stopped, uint32_t now_ms,
  AutoDrive & state);

bool auto_drive_active(const AutoDrive & state);

// 底盘任务发布的完成状态，供connect任务发送；未完成或运行条件失效时为false。
bool chassis_auto_complete();

#endif
