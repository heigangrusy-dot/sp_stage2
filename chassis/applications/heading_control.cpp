#include "heading_control.hpp"

#include <cmath>

#include "chassis_config.hpp"
#include "tools/math_tools/math_tools.hpp"

static_assert(
  chassis_config::HEADING_KP >= 0.0f && chassis_config::HEADING_KD >= 0.0f &&
  chassis_config::HEADING_MAX_CORRECTION > 0.0f);

// ---------- 航向锁定与主动转弯 ----------
float update_heading_control(
  float manual_wz, const ImuHeading & feedback, bool enabled, uint32_t now_ms,
  HeadingControl & state)
{
  if (
    !enabled || !chassis_config::HEADING_HOLD_ENABLED || !feedback.valid ||
    !std::isfinite(feedback.yaw) || !std::isfinite(feedback.yaw_rate)) {
    state = {};
    return enabled ? manual_wz : 0.0f;
  }

  state.error = 0.0f;
  state.correction = 0.0f;
  if (manual_wz != 0.0f) {
    state.target_yaw = feedback.yaw;
    state.locked = false;
    state.turning = true;
    state.settling = false;
    return manual_wz;
  }

  if (state.turning) {
    // 松开旋转拨轮后先按零角速度刹停，避免拉回转弯结束前的方向。
    state.target_yaw = feedback.yaw;
    if (std::abs(feedback.yaw_rate) > chassis_config::HEADING_SETTLE_RATE) {
      state.settling = false;
      return 0.0f;
    }
    if (!state.settling) {
      state.settling = true;
      state.settled_since_ms = now_ms;
    }
    if (now_ms - state.settled_since_ms < chassis_config::HEADING_SETTLE_MS) return 0.0f;
    state.turning = false;
    state.settling = false;
  }

  if (!state.locked) {
    state.target_yaw = feedback.yaw;
    state.locked = true;
  }

  // ---------- 航向PD修正 ----------
  // 最短角差处理±pi跳变；D项直接用角速度，避免差分Yaw放大噪声。
  state.error = sp::limit_angle(state.target_yaw - feedback.yaw);
  if (
    std::abs(state.error) < chassis_config::HEADING_DEADBAND &&
    std::abs(feedback.yaw_rate) < chassis_config::HEADING_RATE_DEADBAND)
    return 0.0f;

  state.correction = sp::limit_max(
    chassis_config::HEADING_KP * state.error - chassis_config::HEADING_KD * feedback.yaw_rate,
    chassis_config::HEADING_MAX_CORRECTION);
  return state.correction;
}
