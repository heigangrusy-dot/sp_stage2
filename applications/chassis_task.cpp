#include <cmath>

#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "cmsis_os.h"
#include "control_safety.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;

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

sp::CAN chassis_can(&hcan1);
sp::RM_Motor motors[WHEEL_COUNT] = {
  {2, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO},
  {3, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO},
  {1, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO},
  {4, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO}};

sp::PID make_speed_pid();
sp::PID speed_pids[WHEEL_COUNT] = {
  make_speed_pid(), make_speed_pid(), make_speed_pid(), make_speed_pid()};

// 默认轮向仍需实车确认；calc输入车体速度，输出轮侧rad/s。
sp::Mecanum mecanum(
  chassis_config::WHEEL_RADIUS, chassis_config::HALF_LENGTH, chassis_config::HALF_WIDTH);

struct ChassisInput
{
  float wheel_speeds[WHEEL_COUNT] = {};
  bool motors_ready = false;
  bool can_ready = false;
  bool remote_valid = false;
  sp::DBusSwitchMode right_switch = sp::DBusSwitchMode::DOWN;
  float forward = 0.0f;
  float right = 0.0f;
  float rotation = 0.0f;
};

ChassisInput read_input();
bool manual_control_ready(const ChassisInput & input);
float filter_stick(float value);
void clear_speed_pids();
HAL_StatusTypeDef send_motor_commands(bool allowed);

sp::PID make_speed_pid()
{
  // 每次返回独立对象：四轮参数相同，但误差、积分和历史反馈互不共用。
  return sp::PID(
    chassis_config::CONTROL_PERIOD_S, chassis_config::SPEED_KP, chassis_config::SPEED_KI,
    chassis_config::SPEED_KD, chassis_config::MAX_WHEEL_TORQUE,
    chassis_config::MAX_INTEGRAL_TORQUE);
}

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
  taskEXIT_CRITICAL();
  return input;
}

bool manual_control_ready(const ChassisInput & input)
{
  return input.remote_valid && input.right_switch == sp::DBusSwitchMode::MID &&
         input.motors_ready && input.can_ready;
}

float filter_stick(float value)
{
  // 死区消除归中抖动，限幅限制输入比例；不是通信有效性校验。
  return std::abs(value) < chassis_config::INPUT_DEADBAND ? 0.0f : sp::limit_max(value, 1.0f);
}

void clear_speed_pids()
{
  for (auto & pid : speed_pids) pid.clear();
}

HAL_StatusTypeDef send_motor_commands(bool allowed)
{
  // cmd保存转矩；write按电调ID写入字节位置，不依赖数组排列顺序。
  for (int i = 0; i < WHEEL_COUNT; ++i) {
    motors[i].cmd(allowed ? speed_pids[i].out : 0.0f);
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
// 同方向提速限制步长；减速、归中和反向直接更新目标。
float limit_acceleration(float target, float current, float max_step)
{
  const bool accelerating = target * current >= 0.0f && std::abs(target) > std::abs(current);
  return accelerating ? current + sp::limit_max(target - current, max_step) : target;
}
}  // namespace

extern "C" void chassis_task(void const * argument)
{
  (void)argument;
  control::ControlSafety safety;
  chassis_can.config();
  chassis_can.start();
  uint32_t previous_wake_time = osKernelSysTick();
  uint32_t last_plot_time = previous_wake_time;

  float vx_command = 0.0f;  // 经过加速度限制的前后速度指令，m/s。
  float vy_command = 0.0f;  // 左移为正，m/s。
  float wz_command = 0.0f;  // 逆时针为正，rad/s。

  while (true) {
    // 1. 输入快照和使能状态：每轮只更新一次下档确认状态机。
    const auto input = read_input();
    const bool enabled = safety.update(
      input.remote_valid, input.right_switch == sp::DBusSwitchMode::DOWN,
      input.right_switch == sp::DBusSwitchMode::MID, input.motors_ready && input.can_ready);

    // 2. 遥控指令转换为车体速度，三个方向复用同一提速限制。
    if (enabled) {
      // 摇杆希望达到的速度，允许立即变化。
      const float target_vx = filter_stick(input.forward) * chassis_config::MAX_LINEAR_SPEED;
      // 遥控横向向右为正，解算向左为正，因此取负。
      const float target_vy = -filter_stick(input.right) * chassis_config::MAX_LATERAL_SPEED;
      // 拨轮正输入暂对应逆时针，实车确认操纵方向。
      const float target_wz = filter_stick(input.rotation) * chassis_config::MAX_ANGULAR_SPEED;
      // 每个控制周期允许改变的速度：加速度 × 时间。
      const float max_step =
        chassis_config::MAX_LINEAR_ACCELERATION * chassis_config::CONTROL_PERIOD_S;
      vx_command = limit_acceleration(target_vx, vx_command, max_step);
      vy_command = limit_acceleration(target_vy, vy_command, max_step);
      wz_command = limit_acceleration(
        target_wz, wz_command,
        chassis_config::MAX_ANGULAR_ACCELERATION * chassis_config::CONTROL_PERIOD_S);
    }
    else {
      vx_command = 0.0f;  // 失能时清零，重新使能从0开始。
      vy_command = 0.0f;
      wz_command = 0.0f;
    }
    mecanum.calc(vx_command, vy_command, wz_command);
    const float targets[WHEEL_COUNT] = {
      mecanum.speed_lf, mecanum.speed_lr, mecanum.speed_rf, mecanum.speed_rr};
    if (enabled) {
      for (int i = 0; i < WHEEL_COUNT; ++i) {
        speed_pids[i].calc(targets[i], input.wheel_speeds[i]);
      }
    }

    // 3. 发送前复查最新输入，避免计算期间新到的下档/掉线被旧快照覆盖。
    const auto latest = read_input();
    bool output_allowed = enabled && manual_control_ready(latest);
    if (enabled && !output_allowed) safety.reset();
    // 普通下档由update记住；不能每次!enabled都reset，否则永远无法重新使能。
    if (!output_allowed) {
      vx_command = 0.0f;
      vy_command = 0.0f;
      wz_command = 0.0f;
      clear_speed_pids();
    }

    if (send_motor_commands(output_allowed) != HAL_OK) {
      safety.reset();
      vx_command = 0.0f;
      vy_command = 0.0f;
      wz_command = 0.0f;
      clear_speed_pids();
      output_allowed = false;
      HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }

    // 4. 每20ms更新15路观测快照，串口发送由独立绘图任务完成。
    const auto now_ms = osKernelSysTick();
    if (now_ms - last_plot_time >= chassis_config::PLOT_PERIOD_MS) {
      last_plot_time = now_ms;
      update_plot_data({
        targets[LEFT_FRONT], input.wheel_speeds[LEFT_FRONT], targets[LEFT_REAR],
        input.wheel_speeds[LEFT_REAR], targets[RIGHT_FRONT], input.wheel_speeds[RIGHT_FRONT],
        targets[RIGHT_REAR], input.wheel_speeds[RIGHT_REAR], latest.remote_valid ? 1.0f : 0.0f,
        output_allowed ? 1.0f : 0.0f, speed_pids[LEFT_FRONT].out, speed_pids[LEFT_REAR].out,
        speed_pids[RIGHT_FRONT].out, speed_pids[RIGHT_REAR].out, speed_pids[LEFT_FRONT].data.iout});
    }
    osDelayUntil(&previous_wake_time, chassis_config::CONTROL_PERIOD_MS);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) continue;

    // 只接收这四个反馈ID；read完成减速比和单位换算，不发送电机命令。
    const auto now_ms = osKernelSysTick();
    for (auto & motor : motors) {
      if (header.StdId == motor.rx_id) {
        motor.read(data, now_ms);
        break;
      }
    }
  }
}
