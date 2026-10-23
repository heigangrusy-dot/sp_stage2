#include <cmath>

#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "chassis_power.hpp"
#include "cmsis_os.h"
#include "control_safety.hpp"
#include "heading_control.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "plotter.hpp"
#include "power_meter.hpp"
#include "school_pm02.hpp"
#include "task.h"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;
extern SchoolPm02 pm02;

namespace
{
enum WheelIndex
{
  LEFT_FRONT,
  LEFT_REAR,
  RIGHT_FRONT,
  RIGHT_REAR,
  WHEEL_COUNT
};

// ---------- 电机与控制对象 ----------
sp::PID make_speed_pid()
{
  return sp::PID(
    chassis_config::CONTROL_PERIOD_S, chassis_config::SPEED_KP, chassis_config::SPEED_KI,
    chassis_config::SPEED_KD, chassis_config::MAX_WHEEL_TORQUE,
    chassis_config::MAX_INTEGRAL_TORQUE);
}

sp::CAN chassis_can(&hcan1);
sp::RM_Motor motors[WHEEL_COUNT] = {
  {2, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO},
  {3, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO},
  {1, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO},
  {4, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO}};

PowerMeter power_meter;

sp::PID speed_pids[WHEEL_COUNT] = {
  make_speed_pid(), make_speed_pid(), make_speed_pid(), make_speed_pid()};

// 最终轮侧转矩，供发送、功率预测和观测共用。
float wheel_commands[WHEEL_COUNT] = {};

sp::Mecanum mecanum(
  chassis_config::WHEEL_RADIUS, chassis_config::HALF_LENGTH, chassis_config::HALF_WIDTH);

struct ChassisInput
{
  float wheel_speeds[WHEEL_COUNT] = {};
  bool motors_ready = false;
  bool can_ready = false;
  bool remote_valid = false;
  float power_limit_w = 0.0f;
  bool power_ready = false;
  sp::DBusSwitchMode right_switch = sp::DBusSwitchMode::DOWN;
  float forward = 0.0f;
  float right = 0.0f;
  float rotation = 0.0f;
  ImuHeading heading;
};

// ---------- 输入快照与遥控处理 ----------
ChassisInput read_input()
{
  ChassisInput input;
  // 只在临界区复制中断更新的数据，不在其中计算PID、发送或等待。
  taskENTER_CRITICAL();
  const auto now_ms = osKernelSysTick();
  input.motors_ready = true;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    input.wheel_speeds[i] = motors[i].speed;
    input.motors_ready = input.motors_ready && motors[i].is_alive(now_ms);
  }
  input.remote_valid = remote.is_alive(now_ms);
  if (input.remote_valid) {
    input.right_switch = remote.sw_r;
    input.forward = remote.ch_lv;
    input.right = remote.ch_lh;
    input.rotation = remote.ch_lu;
  }
  input.can_ready = HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING &&
                    HAL_CAN_GetError(&hcan1) == HAL_CAN_ERROR_NONE;

  // PM02上限、接收记录一起读取；计时与适配类统一使用HAL_GetTick。
  input.power_limit_w = sp::limit_min_max(
    static_cast<float>(pm02.robot_status.chassis_power_limit), 0.0f,
    chassis_config::COMPETITION_POWER_LIMIT_W);
  if constexpr (chassis_config::POWER_TEST_LIMIT_W > 0.0f) {
    if (input.power_limit_w > chassis_config::POWER_TEST_LIMIT_W) {
      input.power_limit_w = chassis_config::POWER_TEST_LIMIT_W;
    }
  }
  input.power_ready = pm02.status_count != 0 &&
                      HAL_GetTick() - pm02.status_last_rx_ms < chassis_config::PM02_TIMEOUT_MS &&
                      input.power_limit_w > chassis_config::POWER_MARGIN_W;
  taskEXIT_CRITICAL();
  input.heading = read_imu_heading();
  return input;
}

// ---------- 底盘运行条件 ----------
bool control_ready(const ChassisInput & input)
{
  return input.motors_ready && input.can_ready &&
         (!chassis_config::POWER_LIMIT_ENABLED || input.power_ready);
}

bool manual_control_ready(const ChassisInput & input)
{
  return input.remote_valid && input.right_switch == sp::DBusSwitchMode::MID &&
         control_ready(input);
}

float filter_stick(float value)
{
  // 死区抑制归中抖动
  return std::abs(value) < chassis_config::INPUT_DEADBAND ? 0.0f : sp::limit_max(value, 1.0f);
}

void snap_stick_direction(float & forward, float & right)
{
  static_assert(
    chassis_config::STICK_AXIS_SNAP_RATIO >= 0.0f && chassis_config::STICK_AXIS_SNAP_RATIO < 1.0f);
  // 主轴吸附
  if (std::abs(right) <= std::abs(forward) * chassis_config::STICK_AXIS_SNAP_RATIO) {
    right = 0.0f;
  }
  else if (std::abs(forward) <= std::abs(right) * chassis_config::STICK_AXIS_SNAP_RATIO) {
    forward = 0.0f;
  }
}

// ---------- 前后速度过渡 ----------
float smooth_forward_speed(float target, float current)
{
  static_assert(
    chassis_config::FORWARD_ACCELERATION > 0.0f && chassis_config::FORWARD_DECELERATION > 0.0f);
  // 换向先将命令降到零，下一周期再向反方向加速。
  const float goal = target * current < 0.0f ? 0.0f : target;
  const bool slowing_down = std::abs(goal) < std::abs(current);
  const float acceleration =
    slowing_down ? chassis_config::FORWARD_DECELERATION : chassis_config::FORWARD_ACCELERATION;
  const float step = acceleration * chassis_config::CONTROL_PERIOD_S;
  return current + sp::limit_max(goal - current, step);
}

void clear_speed_pids()
{
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    speed_pids[i].clear();
    wheel_commands[i] = 0.0f;
  }
}

// ---------- 轮速控制与停车保持 ----------
struct WheelControlState
{
  bool stopped_pending = false;
  uint32_t stopped_since_ms = 0;
  float integral_weight = 1.0f;
  PowerLimitResult power_limit;  // 观察scale、限制前后功率及模型内是否有解。
};

static_assert(chassis_config::STOP_INTEGRAL_FADE_S > 0.0f);
static_assert(
  chassis_config::POWER_MARGIN_W >= 0.0f &&
  chassis_config::COMPETITION_POWER_LIMIT_W > chassis_config::POWER_MARGIN_W);
static_assert(
  chassis_config::POWER_TEST_LIMIT_W == 0.0f ||
  chassis_config::POWER_TEST_LIMIT_W > chassis_config::POWER_MARGIN_W);
static_assert(chassis_config::PM02_TIMEOUT_MS > 0);

void update_wheel_control(
  const float targets[WHEEL_COUNT], const ChassisInput & input, bool enabled,
  WheelControlState & state)
{
  // 只有整车四轮目标全零才退出积分，单轮目标为零不触发。
  bool zero_target = true;
  bool all_stopped = enabled;
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    zero_target = zero_target && std::abs(targets[i]) < 0.001f;
    all_stopped =
      all_stopped && std::abs(input.wheel_speeds[i]) <= chassis_config::STOPPED_WHEEL_SPEED;
  }
  all_stopped = all_stopped && zero_target;
  const float target_integral_weight = zero_target ? 0.0f : 1.0f;
  const float weight_step = chassis_config::CONTROL_PERIOD_S / chassis_config::STOP_INTEGRAL_FADE_S;
  state.integral_weight = sp::limit_min_max(
    state.integral_weight +
      sp::limit_max(target_integral_weight - state.integral_weight, weight_step),
    0.0f, 1.0f);

  // 零目标时P项仍正常制动，积分贡献平滑退出；停稳后再清历史状态。
  const auto control_now_ms = osKernelSysTick();
  if (all_stopped && !state.stopped_pending) {
    state.stopped_pending = true;
    state.stopped_since_ms = control_now_ms;
  }
  else if (!all_stopped) {
    state.stopped_pending = false;
  }
  // 等积分贡献完全退出后再清状态，避免停得较快时提前截断淡出过程。
  const bool reset_at_rest =
    state.stopped_pending && state.integral_weight == 0.0f &&
    control_now_ms - state.stopped_since_ms >= chassis_config::STOPPED_RESET_DELAY_MS;
  if (reset_at_rest) {
    clear_speed_pids();
    state.power_limit = {};
  }
  else if (enabled) {
    for (int i = 0; i < WHEEL_COUNT; ++i) {
      // 上一周期已限功率时，暂停继续推高输出的积分；允许反方向积分纠偏。
      const bool pause_integral = state.power_limit.scale < 1.0f &&
                                  (targets[i] - input.wheel_speeds[i]) * speed_pids[i].out > 0.0f;
      if (pause_integral)
        speed_pids[i].calc(targets[i], input.wheel_speeds[i], 0.0f);
      else
        speed_pids[i].calc(targets[i], input.wheel_speeds[i]);
      const auto & data = speed_pids[i].data;
      // 在未截断的P/I/D项上加权，再统一限幅；不能从已限幅out中减去I项。
      wheel_commands[i] = sp::limit_max(
        data.pout + state.integral_weight * data.iout + data.dout,
        chassis_config::MAX_WHEEL_TORQUE);
    }
  }
}

// ---------- 功率限制 ----------
void update_power_limit(const ChassisInput & input, WheelControlState & state)
{
  if constexpr (chassis_config::POWER_LIMIT_ENABLED) {
    const float budget_w = input.power_limit_w - chassis_config::POWER_MARGIN_W;
    state.power_limit = limit_chassis_power(wheel_commands, input.wheel_speeds, budget_w);
  }
}

// ---------- 观测数据 ----------
// 通道顺序见plotter.cpp，串口发送由绘图任务完成。
void publish_plot_data(
  const float targets[WHEEL_COUNT], const ChassisInput & input, bool output_allowed,
  uint32_t elapsed_ms, const HeadingControl & heading, const WheelControlState & wheel_control)
{
  const auto power = power_meter.snapshot();

  // 展示原始请求功率、有效预算和实测；失能时按已清零的最终命令预测。
  const float predicted_power = predict_chassis_power(wheel_commands, input.wheel_speeds);
  const float requested_power = chassis_config::POWER_LIMIT_ENABLED && output_allowed
                                  ? wheel_control.power_limit.before_w
                                  : predicted_power;
  const float budget_w = input.power_ready
                           ? input.power_limit_w - chassis_config::POWER_MARGIN_W
                           : 0.0f;

  PlotData data = {
    targets[LEFT_FRONT],
    input.wheel_speeds[LEFT_FRONT],
    targets[LEFT_REAR],
    input.wheel_speeds[LEFT_REAR],
    targets[RIGHT_FRONT],
    input.wheel_speeds[RIGHT_FRONT],
    targets[RIGHT_REAR],
    input.wheel_speeds[RIGHT_REAR],
    static_cast<float>(elapsed_ms),
    output_allowed ? 1.0f : 0.0f,
    wheel_commands[LEFT_FRONT],
    wheel_commands[LEFT_REAR],
    wheel_commands[RIGHT_FRONT],
    wheel_commands[RIGHT_REAR],
    power.online ? 1.0f : 0.0f,
    requested_power,
    budget_w,
    power.power_w};
  if constexpr (chassis_config::PLOT_HEADING) {
    data[10] = input.heading.yaw;
    data[11] = heading.target_yaw;
    data[12] = input.heading.yaw_rate;
    data[13] = heading.correction;
  }
  update_plot_data(data);
}

// ---------- CAN电机命令 ----------
HAL_StatusTypeDef send_motor_commands(bool allowed)
{
  // cmd保存转矩；write按电调ID写入字节位置，不依赖数组排列顺序。
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    motors[i].cmd(allowed ? wheel_commands[i] : 0.0f);
    motors[i].write(chassis_can.tx_data);
  }

  CAN_TxHeaderTypeDef header = {};
  header.StdId = motors[LEFT_FRONT].tx_id;  // ID1-4共用0x200帧。
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  uint32_t mailbox = 0;
  // HAL_OK只表示成功入队，不保证电调已收到；保留返回值供任务处理。
  return HAL_CAN_AddTxMessage(&hcan1, &header, chassis_can.tx_data, &mailbox);
}

}  // namespace

// ---------- 底盘主任务 ----------
extern "C" void chassis_task(void const * argument)
{
  (void)argument;
  control::ControlSafety safety;
  chassis_can.config();
  chassis_can.start();
  uint32_t previous_wake_time = osKernelSysTick();
  const uint32_t plot_start_time = previous_wake_time;
  uint32_t last_plot_time = previous_wake_time;

  WheelControlState wheel_control;
  HeadingControl heading_control;
  float vx_command = 0.0f;

  while (true) {
    // 1. 输入快照与下档确认。
    const auto input = read_input();
    const bool enabled = safety.update(
      input.remote_valid, input.right_switch == sp::DBusSwitchMode::DOWN,
      input.right_switch == sp::DBusSwitchMode::MID, control_ready(input));

    // 2. 遥控处理与四轮目标解算。
    float forward = enabled ? filter_stick(input.forward) : 0.0f;
    float right = enabled ? filter_stick(input.right) : 0.0f;
    const float rotation = enabled ? filter_stick(input.rotation) : 0.0f;
    snap_stick_direction(forward, right);

    const float target_vx = forward * chassis_config::MAX_LINEAR_SPEED;
    // 普通遥控使用速度过渡；失能立即清零，不经过斜坡。
    vx_command = enabled ? smooth_forward_speed(target_vx, vx_command) : 0.0f;
    // 航向保持先修正角速度，之后仍统一经过轮速环与功率限制。
    const float wz_command = update_heading_control(
      rotation * chassis_config::MAX_ANGULAR_SPEED, input.heading, enabled, osKernelSysTick(),
      heading_control);
    mecanum.calc(vx_command, -right * chassis_config::MAX_LATERAL_SPEED, wz_command);
    const float targets[WHEEL_COUNT] = {
      mecanum.speed_lf, mecanum.speed_lr, mecanum.speed_rf, mecanum.speed_rr};

    // 3. 速度环与停车积分。
    update_wheel_control(targets, input, enabled, wheel_control);

    // 4. 按PM02上限和测试预算限制四轮转矩，不改变原速度环目标。
    if (enabled) update_power_limit(input, wheel_control);

    // 5. 发送前复查，响应计算期间新到的下档或掉线。
    const auto latest = read_input();
    bool output_allowed = enabled && manual_control_ready(latest);
    if (enabled && !output_allowed) safety.reset();
    if (!output_allowed) {
      clear_speed_pids();
      wheel_control.integral_weight = 1.0f;
      wheel_control.power_limit = {};
      vx_command = 0.0f;
      heading_control = {};
    }

    if (send_motor_commands(output_allowed) != HAL_OK) {
      safety.reset();
      clear_speed_pids();
      wheel_control.integral_weight = 1.0f;
      wheel_control.power_limit = {};
      vx_command = 0.0f;
      heading_control = {};
      output_allowed = false;
      HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }

    // 6. 按观测周期发布快照，预测与CAN发送共用限幅后的转矩。
    const auto now_ms = osKernelSysTick();
    if (now_ms - last_plot_time >= chassis_config::PLOT_PERIOD_MS) {
      last_plot_time = now_ms;

      publish_plot_data(
        targets, input, output_allowed, now_ms - plot_start_time, heading_control, wheel_control);
    }
    osDelayUntil(&previous_wake_time, chassis_config::CONTROL_PERIOD_MS);
  }
}

// ---------- CAN反馈接收 ----------
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) continue;

    // 接收反馈与功率计数据，不在中断内发送命令。
    const auto now_ms = osKernelSysTick();

    // 功率计报文与电机反馈共用CAN1，按ID分别处理。
    if (header.StdId == 0x301) {
      power_meter.update(data, now_ms);
      continue;
    }

    for (auto & motor : motors) {
      if (header.StdId == motor.rx_id) {
        motor.read(data, now_ms);
        break;
      }
    }
  }
}
