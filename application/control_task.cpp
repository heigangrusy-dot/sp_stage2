#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "laser.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_limit.hpp"
#include "task.h"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "tracking_score.hpp"
#include "trajectory_task.hpp"

sp::RM_Motor motor1(1, sp::RM_Motors::GM6020);                //云台底部，用can2
sp::RM_Motor motor2(2, sp::RM_Motors::GM6020);                //侧面用can1
sp::RM_Motor motor3(3, sp::RM_Motors::M2006, sp::M2006_P36);  //侧面用can1

//云台底部yaw电机：角度环输出rad/s，速度环输出N·m
sp::PID motor1_pid_speed(0.001f, 0.19f, 0.0f, 0.0f, 2.5f, 1.5f, 1.0f, false, true);
sp::PID motor1_pid_angle(0.001f, 15.0f, 0.0f, 0.4f, 10.0f, 1.5f, 0.8f, false, true);

//云台侧面摆头pitch电机
// 双环：角度环rad -> rad/s，速度环rad/s -> N·m。
// Pitch测试外环使用imu.pitch，内环使用motor2.speed；普通保持外环使用motor2.angle。
sp::PID motor2_pid_speed(0.001f, 0.02f, 0.0f, 0.0f, 1.0f, 0.25f, 0.3f, false, true);
sp::PID motor2_pid_angle(0.001f, 5.0f, 0.0f, 0.0f, 10.0f, 0.25f, 0.3f, false, true);
//0.001f, 0.8025f, 0.52f, 0.0f, 1.0f, 0.25f, 0.3f, false, true

//云台侧面摆臂pitch电机：单角度环输出N·m
sp::PID motor3_pid_angle(0.001f, 0.375f, 0.1f, 0.031f, 10.0f, 0.1f, 0.5f, false, true);
//0.001f, 0.42f, 0.1f, 0.03f, 1.0f, 0.1f, 0.5f, false, true
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::DBusSwitchMode dbus_sw_r = sp::DBusSwitchMode::DOWN;
sp::DBusSwitchMode dbus_sw_l = sp::DBusSwitchMode::DOWN;
extern Laser laser;
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

// ==================== motor3重力前馈（上电时横杆水平） ====================
constexpr float MOTOR3_GRAVITY_TORQUE = 0.6f;  // 水平时补偿力矩，N·m（含力臂的等效系数）
constexpr float MOTOR3_MAX_TORQUE = 2.5f;      // PID与前馈叠加后的限幅，N·m
static float motor3_horizontal_angle = 0.0f;
static bool motor3_horizontal_ready = false;
float motor3_gravity_ff = 0.0f;  // 调试观察：实际加入的重力前馈
float motor1_torque_cmd = 0.0f;
float motor1_accel_ff = 0.0f;    // 调试：Yaw轨迹加速度前馈力矩，N·m
float motor2_torque_cmd = 0.0f;  // 速度环与前馈叠加、限幅后的命令，N·m
float motor3_torque_cmd = 0.0f;
// ==================== motor3重力前馈结束 ====================

constexpr uint32_t CONNECT_OFFLINE_MS = 50;
constexpr float MOTOR3_NORMAL_RAD = -1.25f;                  // -60°
constexpr float MOTOR2_TRACK_ZERO_RAD = 3.14159265f / 4.0f;  // 45°
constexpr float MOTOR3_FOLD_RAD = -3.14159265f / 12.0f;      // -30°
constexpr uint16_t FOLD_DISTANCE_CM = 40;                    //想一直处于初始状态则值调为大于200即可
constexpr float ARM_RESET_MOTOR1_RAD = -2.15f;
constexpr float ARM_RESET_MOTOR3_RAD = -1.25f;
constexpr float ARM_FOLD_MOTOR3_RAD = -0.2f;
constexpr uint32_t FOLD_RESET_DELAY_MS = 2000;
constexpr uint32_t LASER_FRESH_MS = 500;
static bool hold_arm_started = false;

//折叠复位功能相关标志
uint32_t connect_last_rx_ms = 0;
bool arm_folded = false;
bool arm_waiting_reset = false;
uint32_t fold_exit_ms = 0;

//轨迹跟踪功能相关标志
bool track_latched = false;
float track_yaw0 = 0.0f;
float track_pitch0 = 0.0f;

// 平滑摩擦补偿：按误差方向补偿，接近目标时连续减小至0。
constexpr float MOTOR1_FRICTION_TORQUE = 0.0f;                          // N·m
constexpr float MOTOR2_FRICTION_TORQUE = 0.40f;                         // N·m 0.155f
constexpr float MOTOR3_FRICTION_TORQUE = 0.1f;                          // N·m
constexpr float FRICTION_TRANSITION_RAD = 2.5f * 3.14159265f / 180.0f;  // 过渡角度2.5°
constexpr float MOTOR1_MAX_TORQUE = 2.5f;
constexpr float MOTOR2_MAX_TORQUE = 1.0f;
static float friction_feedforward(float error, float torque);
static void test_pitch_trajectory(bool enabled);
static void test_yaw_trajectory(bool enabled);

extern "C" void lose_power();
extern "C" void drive_motor(float motor1_set, float motor2_set, float motor3_set);
extern "C" void hold_arm(float motor1_zero, float motor2_zero, float motor3_zero);
void track_arm(float motor1_zero, float motor2_zero, float motor3_zero);
static void drive_motor_feedback(
  float motor1_set, float motor2_set, float motor3_set, float motor2_feedback);

// ==================== 临时：Pitch分阶段测试（整段可删除） ====================
// 0=未启动，1=motor3抬升，2=Pitch回零，3=轨迹跟踪。
int pitch_test_stage = 0;
float pitch_test_target = 0.0f;
float yaw_ratio = 0.0f, pitch_ratio = 0.0f;
float yaw_score = 0.0f, pitch_score = 0.0f;

// 绝对编码器角度限位，单位rad；边界值允许。
constexpr float MOTOR1_MIN_ANGLE = -3.8f, MOTOR1_MAX_ANGLE = 4.2f;  //-3.0 -1.3
constexpr float MOTOR2_MIN_ANGLE = -10.0f, MOTOR2_MAX_ANGLE = 10.0f;
constexpr float MOTOR3_MIN_ANGLE = -3.8f, MOTOR3_MAX_ANGLE = 4.2f;  //-1.9 0.25
volatile MotorLimitState motor_limit_state = MotorLimitState::NORMAL;
static bool motor_limit_blocked(bool feedback_valid, uint32_t now);

extern "C" void control_task(void * argument)
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  //电机初始化确认
  bool motor_ready = false;
  float motor1_zero = motor1.angle;
  float motor2_zero = motor2.angle;
  float motor3_zero = motor3.angle;

  TickType_t control_wake_tick = xTaskGetTickCount();
  while (1) {
    if (!motor3_horizontal_ready && motor3.is_alive(osKernelSysTick())) {
      motor3_horizontal_angle = motor3.angle;
      motor3_horizontal_ready = true;
    }
    if (osKernelSysTick() - connect_last_rx_ms > CONNECT_OFFLINE_MS) {
      dbus_sw_r = sp::DBusSwitchMode::DOWN;
    }
    //电机初始化确认，防止初始化电机读取有问题后续伤害机械结构
    bool motors_alive = motor1.is_alive(osKernelSysTick()) && motor2.is_alive(osKernelSysTick()) &&
                        motor3.is_alive(osKernelSysTick());
    if (motors_alive && !motor_ready) {
      motor_ready = true;
      motor1_zero = motor1.angle;
      motor2_zero = motor2.angle;
      motor3_zero = motor3.angle;
    }

    if (motor_limit_blocked(motors_alive, osKernelSysTick())) {
      track_latched = false;
      pitch_test_stage = 0;
      pitch_test_target = 0.0f;
      test_yaw_trajectory(false);  // 失能、清空PID并重置Yaw测试起点
      vTaskDelayUntil(&control_wake_tick, pdMS_TO_TICKS(1));
      continue;
    }

    switch (dbus_sw_r) {
      case sp::DBusSwitchMode::DOWN:  //右拨杆下档，失能
        track_latched = false;
        test_pitch_trajectory(false);  // 临时Pitch测试：退出时复位
        break;
      case sp::DBusSwitchMode::MID:
      case sp::DBusSwitchMode::UP:
        test_pitch_trajectory(motors_alive);  // 临时Pitch测试

        //hold_arm(motor1_zero, motor2_zero, motor3_zero);  // 复位/避障折叠
        //test_yaw_trajectory(motor1.is_alive(osKernelSysTick()));  // 临时Yaw测试
        //drive_motor(motor1_zero + 1.0f, motor2_zero, motor3_zero);  //保持机械臂
        // if (dbus_sw_l == sp::DBusSwitchMode::UP) {
        //   track_arm(motor1_zero, motor2_zero, motor3_zero);
        // }
        // else {
        //   track_latched = false;
        //   hold_arm(motor1_zero, motor2_zero, motor3_zero);
        // }
        break;
    }

    vTaskDelayUntil(&control_wake_tick, pdMS_TO_TICKS(1));
  }
}

static bool motor_limit_blocked(bool feedback_valid, uint32_t now)
{
  static uint32_t recovery_start = 0;
  if (!feedback_valid) {
    // 未收到全部有效反馈前禁止运行；故障恢复期间掉线则重新等待。
    if (motor_limit_state != MotorLimitState::NORMAL)
      motor_limit_state = MotorLimitState::OUT_OF_RANGE;
    return true;
  }
  const bool in_range = std::isfinite(motor1.angle) && motor1.angle >= MOTOR1_MIN_ANGLE &&
                        motor1.angle <= MOTOR1_MAX_ANGLE && std::isfinite(motor2.angle) &&
                        motor2.angle >= MOTOR2_MIN_ANGLE && motor2.angle <= MOTOR2_MAX_ANGLE &&
                        std::isfinite(motor3.angle) && motor3.angle >= MOTOR3_MIN_ANGLE &&
                        motor3.angle <= MOTOR3_MAX_ANGLE;
  if (!in_range) {
    motor_limit_state = MotorLimitState::OUT_OF_RANGE;
    return true;
  }
  if (motor_limit_state == MotorLimitState::OUT_OF_RANGE) {
    recovery_start = now;
    motor_limit_state = MotorLimitState::RECOVERING;
  }
  if (motor_limit_state == MotorLimitState::RECOVERING && now - recovery_start >= 10000U)
    motor_limit_state = MotorLimitState::NORMAL;
  return motor_limit_state != MotorLimitState::NORMAL;
}

static float friction_feedforward(float error, float torque)
{
  return torque * std::tanh(error / FRICTION_TRANSITION_RAD);
}

static void test_pitch_trajectory(bool enabled)
{
  constexpr float kDeg = 3.14159265f / 180.0f;
  constexpr uint32_t kSettleMs = 200;
  static uint32_t stable_since = 0, track_start = 0, phase0 = 0;
  static bool settling = false;
  static float yaw_hold = 0.0f, motor2_hold = 0.0f;

  if (!enabled || !motor3_horizontal_ready || !std::isfinite(imu.pitch)) {
    pitch_test_stage = 0;
    pitch_test_target = 0.0f;
    settling = false;
    lose_power();
    return;
  }

  const uint32_t now = osKernelSysTick();
  const float arm_target = ARM_RESET_MOTOR3_RAD;  // 与复位状态共用同一个绝对角度目标
  if (pitch_test_stage == 0) {
    settling = false;
    yaw_hold = motor1.angle;
    motor2_hold = motor2.angle;
    motor1_pid_angle.clear();
    motor1_pid_speed.clear();
    motor2_pid_angle.clear();
    motor2_pid_speed.clear();
    motor3_pid_angle.clear();
    pitch_test_stage = 1;
  }

  if (pitch_test_stage == 1) {
    drive_motor(yaw_hold, motor2_hold, arm_target);
  }
  else {
    pitch_test_target =
      pitch_test_stage == 3 ? trajectory_sample_at_ms(now - track_start + phase0).pitch : 0.0f;
    // 直接用IMU Pitch闭环；正力矩使motor2.angle和imu.pitch增加。
    drive_motor_feedback(yaw_hold, pitch_test_target, arm_target, imu.pitch);
  }

  if (pitch_test_stage == 3) {
    pitch_detect(imu.pitch, pitch_test_target, &pitch_ratio);
    pitch_score = 100.0f * pitch_ratio;
    return;
  }
  const bool reached = pitch_test_stage == 1 ? std::abs(motor3.angle - arm_target) < 2.5f * kDeg
                                             : std::abs(imu.pitch) < 2.5f * kDeg;
  if (!reached) {
    settling = false;
    return;
  }
  if (!settling) {
    settling = true;
    stable_since = now;
  }
  if (now - stable_since < kSettleMs) return;
  settling = false;

  if (pitch_test_stage == 1) {
    motor2_pid_angle.clear();  // 从编码器闭环切换到IMU闭环
    motor2_pid_speed.clear();
    pitch_test_stage = 2;
  }
  else {
    // 从原始Pitch曲线最接近0的位置开始，避免直接跳到-30°。
    phase0 = 0;
    float best = std::abs(trajectory_sample_at_ms(0).pitch);
    for (uint32_t t = 1; t < TRAJECTORY_PERIOD_MS; ++t) {
      const float error = std::abs(trajectory_sample_at_ms(t).pitch);
      if (error < best) {
        best = error;
        phase0 = t;
      }
    }
    track_start = osKernelSysTick();
    pitch_ratio = 0.0f;
    pitch_score = 0.0f;  // 仅从正式轨迹阶段开始评分，不计抬升/回零
    pitch_test_stage = 3;
  }
}
// ==================== 临时Pitch测试结束 ====================

// ==================== 临时：仅测试Yaw轨迹（整段可删除） ====================
constexpr float YAW_SPEED_FF_GAIN = 0.0f;      // 轨迹速度前馈增益，设0可关闭
constexpr float YAW_TARGET_SPEED_MAX = 10.0f;  // 叠加后的速度限幅，rad/s
constexpr float YAW_ACCEL_FF_GAIN = 0.00f;     // 等效惯量kg·m²，试调初值，设0可关闭
static void test_yaw_trajectory(bool enabled)
{
  static bool started = false;
  static uint32_t start_ms = 0;
  static float angle0 = 0.0f;
  static float imu_yaw0 = 0.0f, imu_yaw_last = 0.0f, imu_yaw_unwrapped = 0.0f;

  if (!enabled) {
    started = false;
    lose_power();
    return;
  }

  const uint32_t now = osKernelSysTick();
  if (!started) {
    started = true;
    start_ms = now;
    angle0 = motor1.angle;
    imu_yaw0 = imu.yaw;
    imu_yaw_last = imu_yaw0;
    imu_yaw_unwrapped = imu_yaw0;
    yaw_ratio = 0.0f;
    yaw_score = 0.0f;
    motor1_pid_angle.clear();
    motor1_pid_speed.clear();
  }

  const TrajectoryPoint ref = trajectory_sample_at_ms(now - start_ms);
  const float target = angle0 + ref.yaw - trajectory_sample_at_ms(0).yaw;
  // 在调用评分函数前统一到IMU坐标，并展开±pi跳变；评分函数本体保持原样。
  imu_yaw_unwrapped += std::remainder(imu.yaw - imu_yaw_last, 2.0f * 3.14159265f);
  imu_yaw_last = imu.yaw;
  const float plan_yaw = imu_yaw0 + ref.yaw - trajectory_sample_at_ms(0).yaw;
  yaw_detect(imu_yaw_unwrapped, plan_yaw, &yaw_ratio);
  yaw_score = 100.0f * yaw_ratio;
  motor1_pid_angle.calc(target, motor1.angle);
  // 角度与速度使用同一次轨迹采样，角度环负责位置纠偏。
  const float speed_target =
    sp::limit_max(motor1_pid_angle.out + YAW_SPEED_FF_GAIN * ref.yaw_vel, YAW_TARGET_SPEED_MAX);
  motor1_pid_speed.calc(speed_target, motor1.speed);
  // 与角度、速度目标使用同一次采样；加速度前馈直接作用于力矩层。
  motor1_accel_ff = YAW_ACCEL_FF_GAIN * ref.yaw_acc;
  motor1_torque_cmd = sp::limit_max(
    motor1_pid_speed.out + motor1_accel_ff +
      friction_feedforward(motor1_pid_angle.data.err[0], MOTOR1_FRICTION_TORQUE),
    MOTOR1_MAX_TORQUE);
  motor1.cmd(motor1_torque_cmd);
  motor2.cmd(0.0f);
  motor3.cmd(0.0f);
  motor2_pid_angle.clear();
  motor2_pid_speed.clear();
  motor3_pid_angle.clear();
  motor2_torque_cmd = 0.0f;
  motor3_torque_cmd = 0.0f;
  motor3_gravity_ff = 0.0f;

  motor1.write(can2.tx_data);
  motor2.write(can1.tx_data);
  motor3.write(can1.tx_data);
  can2.send(motor1.tx_id);
  can1.send(motor2.tx_id);
  can1.send(motor3.tx_id);
}
// ==================== 临时Yaw测试结束 ====================

extern "C" void lose_power()  //失能函数
{
  hold_arm_started = false;
  arm_folded = false;
  arm_waiting_reset = false;
  motor1_pid_angle.clear();
  motor1_pid_speed.clear();
  motor2_pid_angle.clear();
  motor2_pid_speed.clear();
  motor3_pid_angle.clear();
  motor2_torque_cmd = 0.0f;
  motor3_torque_cmd = 0.0f;
  motor3_gravity_ff = 0.0f;
  motor1_torque_cmd = 0.0f;
  motor1_accel_ff = 0.0f;
  motor1.cmd(0.0f);
  motor2.cmd(0.0f);
  motor3.cmd(0.0f);
  motor1.write(can2.tx_data);
  motor2.write(can1.tx_data);
  motor3.write(can1.tx_data);
  can2.send(motor1.tx_id);
  can1.send(motor2.tx_id);
  can1.send(motor3.tx_id);
}

extern "C" void drive_motor(float motor1_set, float motor2_set, float motor3_set)  //驱动电机函数
{
  drive_motor_feedback(motor1_set, motor2_set, motor3_set, motor2.angle);
}

static void drive_motor_feedback(
  float motor1_set, float motor2_set, float motor3_set, float motor2_feedback)
{
  motor1_pid_angle.calc(motor1_set, motor1.angle);
  motor1_accel_ff = 0.0f;  // 普通保持模式不使用轨迹加速度前馈
  motor1_pid_speed.calc(motor1_pid_angle.out, motor1.speed);
  motor2_pid_angle.calc(motor2_set, motor2_feedback);
  motor2_pid_speed.calc(motor2_pid_angle.out, motor2.speed);
  motor3_pid_angle.calc(motor3_set, motor3.angle);

  motor1_torque_cmd = sp::limit_max(
    motor1_pid_speed.out +
      friction_feedforward(motor1_pid_angle.data.err[0], MOTOR1_FRICTION_TORQUE),
    MOTOR1_MAX_TORQUE);
  motor1.cmd(motor1_torque_cmd);
  motor2_torque_cmd = sp::limit_max(
    motor2_pid_speed.out +
      friction_feedforward(motor2_pid_angle.data.err[0], MOTOR2_FRICTION_TORQUE),
    MOTOR2_MAX_TORQUE);
  motor2.cmd(motor2_torque_cmd);
  // ==================== motor3重力前馈 ====================
  // 负向转动为抬升，水平夹角 = -(当前角度 - 上电水平角度)，单位rad。
  const float arm_pitch = -(motor3.angle - motor3_horizontal_angle);
  motor3_gravity_ff = motor3_horizontal_ready ? -MOTOR3_GRAVITY_TORQUE * std::cos(arm_pitch) : 0.0f;
  motor3_torque_cmd =
    motor3_horizontal_ready
      ? sp::limit_max(
          motor3_pid_angle.out + motor3_gravity_ff +
            friction_feedforward(motor3_pid_angle.data.err[0], MOTOR3_FRICTION_TORQUE),
          MOTOR3_MAX_TORQUE)
      : 0.0f;
  motor3.cmd(motor3_torque_cmd);
  // ==================== motor3重力前馈结束 ====================

  motor1.write(can2.tx_data);
  motor2.write(can1.tx_data);
  motor3.write(can1.tx_data);

  can2.send(motor1.tx_id);
  can1.send(motor2.tx_id);
  can1.send(motor3.tx_id);
}

extern "C" void hold_arm(float, float, float)
{
  static float motor2_hold = 0.0f;
  const uint32_t now_ms = osKernelSysTick();
  if (!hold_arm_started) {
    hold_arm_started = true;
    motor2_hold = motor2.angle;  // 未指定motor2复位目标，保持进入模式时的位置
    arm_folded = false;
    arm_waiting_reset = false;
    motor1_pid_angle.clear();
    motor1_pid_speed.clear();
    motor2_pid_angle.clear();
    motor2_pid_speed.clear();
    motor3_pid_angle.clear();
  }
  const uint16_t distance = laser.distance;
  const bool valid =
    laser.has_valid_frame && distance != 65535 && now_ms - laser.last_valid_rx_ms <= LASER_FRESH_MS;
  if (valid && distance < FOLD_DISTANCE_CM) {
    arm_folded = true;
    arm_waiting_reset = false;
  }
  else if (arm_folded) {
    if (!valid || distance <= FOLD_DISTANCE_CM) {
      arm_waiting_reset = false;  // 无效/断线/等于40cm均打断连续清障计时
    }
    else {
      if (!arm_waiting_reset) {
        arm_waiting_reset = true;
        fold_exit_ms = now_ms;
      }
      if (now_ms - fold_exit_ms > FOLD_RESET_DELAY_MS) {
        arm_folded = false;
        arm_waiting_reset = false;
      }
    }
  }
  drive_motor(
    ARM_RESET_MOTOR1_RAD, motor2_hold, arm_folded ? ARM_FOLD_MOTOR3_RAD : ARM_RESET_MOTOR3_RAD);
}

void track_arm(float motor1_zero, float motor2_zero, float motor3_zero)
{
  TrajectoryPoint ref = trajectory_read();
  if (!track_latched) {
    track_latched = true;
    track_yaw0 = ref.yaw;
    track_pitch0 = ref.pitch;
  }

  drive_motor(
    motor1_zero + ref.yaw - track_yaw0, motor2_zero + (ref.pitch - track_pitch0),
    motor3_zero + MOTOR3_NORMAL_RAD);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)  //CAN回调
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor3.rx_id)
        motor3.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor2.rx_id)
        motor2.read(can1.rx_data, stamp_ms);
    }
    if (hcan == &hcan2) {
      can2.recv();

      if (can2.rx_id == motor1.rx_id)
        motor1.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == 0x111) {
        if (can2.rx_data[0] == 0xFF || can2.rx_data[1] == 0xFF) {
          dbus_sw_r = sp::DBusSwitchMode::DOWN;
          dbus_sw_l = sp::DBusSwitchMode::DOWN;
          connect_last_rx_ms = stamp_ms;
          return;
        }  //底盘遥控器断联，can通信却良好
        else {
          uint8_t raw = can2.rx_data[0];
          connect_last_rx_ms = stamp_ms;  //负责两板子can通信检查
          dbus_sw_r = static_cast<sp::DBusSwitchMode>(raw);
          raw = can2.rx_data[1];
          dbus_sw_l = static_cast<sp::DBusSwitchMode>(raw);
        }
      }
    }
  }
}

//待会调试先缠电工胶布，再测试上下通信，再测试测距模块，最后测试电机

// ==================== 任务提供的评分函数（保持原样） ====================
void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;            // 1ms的freertos对应的滤波系数
  constexpr float threshold = 0.5f / 57.3f;  // 角度转换成弧度

  static uint8_t fire = 0;

  fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;            // 1ms的freertos对应的滤波系数
  constexpr float threshold = 1.5f / 57.3f;  // 角度转换成弧度

  static uint8_t fire = 0;

  fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}
