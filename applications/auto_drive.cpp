#include "auto_drive.hpp"

#include <cmath>

#include "chassis_config.hpp"

static_assert(
  chassis_config::AUTO_DISTANCE_M > chassis_config::AUTO_STOP_TOLERANCE_M &&
  chassis_config::AUTO_STOP_TOLERANCE_M > 0.0f && chassis_config::AUTO_SPEED_MPS > 0.0f &&
  chassis_config::AUTO_ACCELERATION > 0.0f && chassis_config::AUTO_DECELERATION > 0.0f &&
  chassis_config::AUTO_POSITION_KP > 0.0f && chassis_config::AUTO_TIMEOUT_MS > 0 &&
  chassis_config::AUTO_MAX_GAP_MS > 0 && chassis_config::AUTO_STOP_SETTLE_MS > 0);

bool auto_drive_active(const AutoDrive & state)
{
  return state.phase == AutoDrivePhase::RUNNING || state.phase == AutoDrivePhase::STOPPING;
}

// ---------- 模式切换与一次性启动 ----------
float update_auto_drive(
  bool selected, bool allowed, bool rearm, float position, bool all_stopped, uint32_t now_ms,
  AutoDrive & state)
{
  if (rearm && !selected && std::isfinite(position)) {
    state = {};
    state.armed = true;
    return 0.0f;
  }
  if (!selected) {
    if (auto_drive_active(state)) state.phase = AutoDrivePhase::FAULT;
    state.speed_command = 0.0f;
    return 0.0f;
  }
  if (state.phase == AutoDrivePhase::COMPLETE || state.phase == AutoDrivePhase::FAULT) return 0.0f;

  const auto fail = [&state]() {
    state.phase = AutoDrivePhase::FAULT;
    state.armed = false;
    state.speed_command = 0.0f;
    return 0.0f;
  };
  if (!allowed || !std::isfinite(position)) return fail();

  if (state.phase == AutoDrivePhase::IDLE) {
    // 只有下档停稳后才允许启动；拒绝在手动运动中直接切上档起跑。
    if (!state.armed || !all_stopped) return fail();
    state.armed = false;
    state.phase = AutoDrivePhase::RUNNING;
    state.start_position = position;
    state.started_ms = now_ms;
    state.last_update_ms = now_ms;
  }

  // ---------- 编码器测距与执行异常 ----------
  const uint32_t gap_ms = now_ms - state.last_update_ms;
  if (
    gap_ms > chassis_config::AUTO_MAX_GAP_MS ||
    now_ms - state.started_ms >= chassis_config::AUTO_TIMEOUT_MS)
    return fail();
  state.last_update_ms = now_ms;
  state.distance = position - state.start_position;

  const float remaining = chassis_config::AUTO_DISTANCE_M - state.distance;
  if (remaining <= chassis_config::AUTO_STOP_TOLERANCE_M) state.phase = AutoDrivePhase::STOPPING;

  // ---------- 接近终点减速 ----------
  float target = 0.0f;
  if (state.phase == AutoDrivePhase::RUNNING) {
    // 制动距离约束与末段位置比例共同降低速度，不向后追补过冲。
    const float brake_speed = std::sqrt(2.0f * chassis_config::AUTO_DECELERATION * remaining);
    target = chassis_config::AUTO_SPEED_MPS;
    if (target > brake_speed) target = brake_speed;
    const float approach_speed = chassis_config::AUTO_POSITION_KP * remaining;
    if (target > approach_speed) target = approach_speed;
  }
  const float rate = target > state.speed_command ? chassis_config::AUTO_ACCELERATION
                                                  : chassis_config::AUTO_DECELERATION;
  const float step = rate * (gap_ms * 0.001f);
  float change = target - state.speed_command;
  if (change > step) change = step;
  if (change < -step) change = -step;
  state.speed_command += change;

  // ---------- 停稳确认与完成锁定 ----------
  if (state.phase == AutoDrivePhase::STOPPING && state.speed_command == 0.0f && all_stopped) {
    if (!state.settling) {
      state.settling = true;
      state.stopped_since_ms = now_ms;
    }
    if (now_ms - state.stopped_since_ms >= chassis_config::AUTO_STOP_SETTLE_MS)
      state.phase = AutoDrivePhase::COMPLETE;
  }
  else {
    state.settling = false;
  }
  return state.speed_command;
}
