#include <cmath>

#include "FreeRTOS.h"
#include "cmsis_os.h"
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
sp::PID motor1_pid_speed(0.001f, 0.1f, 0.005f, 0.0f, 2.5f, 1.5f, 1.0f, false, true);
sp::PID motor1_pid_angle(0.001f, 15.0f, 0.0f, 0.0f, 10.0f, 1.5f, 0.8f, false, true);

//云台侧面摆头pitch电机
// 双环：角度环rad -> rad/s，速度环rad/s -> N·m。
// Pitch测试外环使用imu.pitch，内环使用motor2.speed；普通保持外环使用motor2.angle。
sp::PID motor2_pid_speed(0.001f, 0.021f, 0.0f, 0.0f, 1.0f, 0.25f, 0.3f, false, true);
sp::PID motor2_pid_angle(0.001f, 5.0f, 0.0f, 0.0f, 10.0f, 0.25f, 0.3f, false, true);
//0.001f, 0.8025f, 0.52f, 0.0f, 1.0f, 0.25f, 0.3f, false, true

//云台侧面摆臂pitch电机：单角度环输出N·m
sp::PID motor3_pid_angle(0.001f, 0.375f, 0.1f, 0.031f, 10.0f, 0.1f, 0.5f, false, true);
//0.001f, 0.42f, 0.1f, 0.03f, 1.0f, 0.1f, 0.5f, false, true
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
volatile sp::DBusSwitchMode dbus_sw_r = sp::DBusSwitchMode::DOWN;
volatile sp::DBusSwitchMode dbus_sw_l = sp::DBusSwitchMode::DOWN;
volatile bool auto_track_requested = false;  // CAN2 0x111 data[2]：1=跟踪，0=保持
extern Laser laser;
extern sp::Mahony imu;
volatile float motor1_startup_angle = 0.0f;  // 上电角度，供控制和绘图共用
static volatile bool motor1_startup_ready = false;

// ==================== motor3重力前馈（上电时横杆水平） ====================
constexpr float MOTOR3_GRAVITY_TORQUE = 0.6f;  // 水平时补偿力矩，N·m（含力臂的等效系数）
constexpr float MOTOR3_MAX_TORQUE = 2.5f;      // PID与前馈叠加后的限幅，N·m
static float motor3_horizontal_angle = 0.0f;
static bool motor3_horizontal_ready = false;
float motor3_gravity_ff = 0.0f;  // 调试观察：实际加入的重力前馈
float motor1_torque_cmd = 0.0f;
float motor1_accel_ff = 0.0f;     // 调试：Yaw轨迹加速度前馈力矩，N·m
float motor1_friction_ff = 0.0f;  // 调试：叠加到限幅前力矩中的摩擦前馈，N·m
float motor2_torque_cmd = 0.0f;   // 速度环与前馈叠加、限幅后的命令，N·m
float motor3_torque_cmd = 0.0f;
// ==================== motor3重力前馈结束 ====================

constexpr uint32_t CONNECT_OFFLINE_MS = 50;
constexpr uint16_t FOLD_DISTANCE_CM = 55;  //想一直处于初始状态则值调为大于200即可
constexpr float ARM_RESET_MOTOR3_RAD = -1.25f;
constexpr float ARM_FOLD_MOTOR3_RAD = -0.2f;
constexpr uint32_t FOLD_RESET_DELAY_MS = 2000;
constexpr uint32_t FOLD_TRIGGER_DELAY_MS = 50;  // 连续近距离确认时间
constexpr uint32_t LASER_FRESH_MS = 500;
static bool hold_arm_started = false;

//折叠复位功能相关标志
volatile uint32_t connect_last_rx_ms = 0;
bool arm_folded = false;
bool arm_waiting_reset = false;
uint32_t fold_exit_ms = 0;

//轨迹跟踪功能相关标志
bool track_latched = false;

// 平滑摩擦补偿：按误差方向补偿，接近目标时连续减小至0。
constexpr float MOTOR1_FRICTION_TORQUE = 0.0f;                          // N·m  0.185
constexpr float MOTOR2_FRICTION_TORQUE = 0.40f;                         // N·m 0.155f
constexpr float MOTOR3_FRICTION_TORQUE = 0.1f;                          // N·m
constexpr float FRICTION_TRANSITION_RAD = 2.5f * 3.14159265f / 180.0f;  // 过渡角度2.5°
constexpr float MOTOR1_MAX_TORQUE = 2.5f;
constexpr float MOTOR2_MAX_TORQUE = 1.0f;
static float friction_feedforward(float error, float torque);
static void test_pitch_trajectory(bool enabled, bool track_yaw = false);
static void test_yaw_trajectory(bool enabled);
constexpr float YAW_SPEED_FF_GAIN = 0.15f;     // 轨迹速度前馈增益，设0可关闭
constexpr float YAW_TARGET_SPEED_MAX = 10.0f;  // 叠加后的速度限幅，rad/s
constexpr float YAW_ACCEL_FF_GAIN = 0.00f;     // 等效惯量kg·m²，设0可关闭

extern "C" void lose_power();
extern "C" void drive_motor(float motor1_set, float motor2_set, float motor3_set);
extern "C" void hold_arm();
void track_arm();
static void drive_motor_feedback(
  float motor1_set, float motor2_set, float motor3_set, float motor2_feedback,
  float yaw_speed_ff = 0.0f, float yaw_accel_ff = 0.0f, bool use_imu_yaw = false,
  float motor1_angle_offset = 0.0f);

// ==================== Pitch分阶段控制（单轴测试和双轴跟踪共用） ====================
// 0=未启动，1=motor3到位，2=姿态到达轨迹起点，3=轨迹跟踪。
int pitch_test_stage = 0;
float pitch_test_target = 0.0f;
float yaw_ratio = 0.0f, pitch_ratio = 0.0f;
float yaw_score = 0.0f, pitch_score = 0.0f;

// 临时注释角度限位，单位rad；恢复时同时启用参数、函数声明/定义及调用。
// constexpr float MOTOR1_MIN_ANGLE = -3.8f, MOTOR1_MAX_ANGLE = 4.2f;  //-3.0 -1.3
// constexpr float MOTOR2_MIN_ANGLE = -10.0f, MOTOR2_MAX_ANGLE = 10.0f;
// constexpr float MOTOR3_MIN_ANGLE = -0.1f, MOTOR3_MAX_ANGLE = 0.1f;  //-1.9 0.25
volatile MotorLimitState motor_limit_state = MotorLimitState::NORMAL;
// static bool motor_limit_blocked(bool feedback_valid, uint32_t now);

extern "C" void control_task(void * argument)
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  TickType_t control_wake_tick = xTaskGetTickCount();
  while (1) {
    // 即使处于DOWN也维护发送邮箱，否则切换模式无法清除堵塞的旧命令。
    const uint32_t can_now = HAL_GetTick();
    can1.service(can_now);
    can2.service(can_now);
    // 暂停alive超时检查；仍等待首帧反馈后记录水平角度。
    // if (!motor3_horizontal_ready && motor3.is_alive(osKernelSysTick())) {
    if (!motor3_horizontal_ready && motor3.is_open()) {
      motor3_horizontal_angle = motor3.angle;
      motor3_horizontal_ready = true;
    }
    // 同一帧的拨杆和自动标志一起读取，避免回调更新到一半。
    taskENTER_CRITICAL();
    const auto right_switch = dbus_sw_r;
    const auto left_switch = dbus_sw_l;
    const bool auto_track = auto_track_requested;
    const uint32_t connect_stamp = connect_last_rx_ms;
    taskEXIT_CRITICAL();
    const bool connect_alive = osKernelSysTick() - connect_stamp <= CONNECT_OFFLINE_MS;
    // const uint32_t now = osKernelSysTick();  // 恢复alive/限位检查时启用
    // 临时关闭电机反馈超时导致的失能，恢复时启用下面的alive判断。
    // const bool motors_alive =
    //   motor1_startup_ready && motor1.is_alive(now) && motor2.is_alive(now) && motor3.is_alive(now);
    // if (motor_limit_blocked(motors_alive, now)) {
    const bool feedback_ready = motor1_startup_ready && motor2.is_open() && motor3_horizontal_ready;
    // if (motor_limit_blocked(feedback_ready, now)) {
    if (!feedback_ready) {  // 仅保留首帧初始化等待，不再判断角度范围和恢复倒计时
      track_latched = false;
      pitch_test_stage = 0;
      pitch_test_target = 0.0f;
      test_yaw_trajectory(false);  // 失能、清空PID并重置Yaw测试起点
      vTaskDelayUntil(&control_wake_tick, pdMS_TO_TICKS(1));
      continue;
    }

    switch (connect_alive ? right_switch : sp::DBusSwitchMode::DOWN) {
      case sp::DBusSwitchMode::DOWN:  //右拨杆下档，失能
      default:
        track_latched = false;
        test_pitch_trajectory(false);  // 失能并复位跟踪准备流程
        break;
      case sp::DBusSwitchMode::MID:  // 中档：左拨杆上档时跟踪
        if (left_switch == sp::DBusSwitchMode::UP) {
          track_arm();
        }
        else {
          track_latched = false;
          pitch_test_stage = 0;
          hold_arm();
        }
        break;

        test_yaw_trajectory(true);  // 临时：中档仅测试Yaw
        break;
      case sp::DBusSwitchMode::UP:  // 上档：自动标志有效时跟踪
        if (auto_track) {
          track_arm();
        }
        else {
          track_latched = false;
          pitch_test_stage = 0;
          hold_arm();
        }
        break;
    }

    vTaskDelayUntil(&control_wake_tick, pdMS_TO_TICKS(1));
  }
}

/* 临时关闭：角度限位与10秒恢复等待，保留原实现便于恢复。
static bool motor_limit_blocked(bool feedback_valid, uint32_t now)
{
  static uint32_t recovery_start = 0;
  if (!feedback_valid) {
    // 未完成首帧反馈初始化前禁止运行。
    if (motor_limit_state != MotorLimitState::NORMAL)
      motor_limit_state = MotorLimitState::OUT_OF_RANGE;
    return true;
  }
  const bool in_range = std::isfinite(motor1.angle) && motor1.angle >= MOTOR1_MIN_ANGLE &&
                        motor1.angle <= MOTOR1_MAX_ANGLE && std::isfinite(motor2.angle) &&
                        motor2.angle >= MOTOR2_MIN_ANGLE && motor2.angle <= MOTOR2_MAX_ANGLE &&
                        std::isfinite(motor3.angle);
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
*/

static float friction_feedforward(float error, float torque)
{
  return torque * std::tanh(error / FRICTION_TRANSITION_RAD);
}

static void test_pitch_trajectory(bool enabled, bool track_yaw)
{
  constexpr float kDeg = 3.14159265f / 180.0f;
  constexpr uint32_t kSettleMs = 200;
  static uint32_t stable_since = 0, track_start = 0;
  static bool settling = false;
  static float yaw_hold = 0.0f, motor2_hold = 0.0f;

  if (
    !enabled || !motor3_horizontal_ready || !std::isfinite(imu.pitch) ||
    (track_yaw && !std::isfinite(imu.yaw))) {
    pitch_test_stage = 0;
    pitch_test_target = 0.0f;
    settling = false;
    lose_power();
    return;
  }

  const uint32_t now = osKernelSysTick();
  const float arm_target = ARM_RESET_MOTOR3_RAD;  // 与复位状态共用同一个绝对角度目标
  const TrajectoryPoint initial = trajectory_sample_at_ms(0);
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
    const bool following = pitch_test_stage == 3;
    // 准备时保持轨迹第0点，正式开始后才推进时间，两轴使用同一次采样。
    const TrajectoryPoint ref = following ? trajectory_sample_at_ms(now - track_start) : initial;
    pitch_test_target = ref.pitch;
    const bool yaw_following = following && track_yaw;
    const float yaw_target = track_yaw ? ref.yaw : yaw_hold;
    // 到达轨迹起点和正式跟踪均使用IMU姿态；单轴Pitch测试仍保持Yaw电机角度。
    drive_motor_feedback(
      yaw_target, pitch_test_target, arm_target, imu.pitch,
      yaw_following ? YAW_SPEED_FF_GAIN * ref.yaw_vel : 0.0f,
      yaw_following ? YAW_ACCEL_FF_GAIN * ref.yaw_acc : 0.0f, track_yaw);
    if (yaw_following) {
      yaw_detect(imu.yaw, ref.yaw, &yaw_ratio);
      yaw_score = 100.0f * yaw_ratio;
    }
  }

  if (pitch_test_stage == 3) {
    pitch_detect(imu.pitch, pitch_test_target, &pitch_ratio);
    pitch_score = 100.0f * pitch_ratio;
    return;
  }
  const bool arm_reached = std::abs(motor3.angle - arm_target) < 2.5f * kDeg;
  const bool pitch_reached = std::abs(imu.pitch - initial.pitch) < 2.5f * kDeg;
  const bool yaw_reached =
    !track_yaw || std::abs(std::remainder(initial.yaw - imu.yaw, 2.0f * 3.14159265f)) < 2.5f * kDeg;
  const bool reached = arm_reached && (pitch_test_stage == 1 || (pitch_reached && yaw_reached));
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
    if (track_yaw) {
      motor1_pid_angle.clear();  // 从编码器保持切换到IMU Yaw闭环
      motor1_pid_speed.clear();
    }
    motor2_pid_angle.clear();  // 从编码器闭环切换到IMU闭环
    motor2_pid_speed.clear();
    pitch_test_stage = 2;
  }
  else {
    // 姿态已到达第0点，从这里开始计时，保留准备阶段的PID状态。
    track_start = osKernelSysTick();
    yaw_ratio = 0.0f;
    yaw_score = 0.0f;
    pitch_ratio = 0.0f;
    pitch_score = 0.0f;  // 仅从正式轨迹阶段开始评分，不计准备过程
    pitch_test_stage = 3;
  }
}
// ==================== Pitch分阶段控制结束 ====================

// ==================== 临时：仅测试Yaw轨迹（整段可删除） ====================
static void test_yaw_trajectory(bool enabled)
{
  constexpr float kTolerance = 2.5f * 3.14159265f / 180.0f;
  constexpr uint32_t kSettleMs = 200;
  static int stage = 0;  // 0=未启动，1=摆臂到位，2=Pitch到起点，3=Yaw跟踪
  static bool settling = false;
  static uint32_t start_ms = 0, stable_since = 0;
  static float motor2_hold = 0.0f;

  if (
    !enabled || !motor1_startup_ready || !motor3_horizontal_ready || !std::isfinite(motor1.angle) ||
    !std::isfinite(imu.pitch)) {
    stage = 0;
    settling = false;
    lose_power();
    return;
  }

  const uint32_t now = osKernelSysTick();
  const TrajectoryPoint initial = trajectory_sample_at_ms(0);
  const float arm_target = ARM_RESET_MOTOR3_RAD;
  const float yaw_offset = initial.yaw - motor1_startup_angle;
  if (stage == 0) {
    motor2_hold = motor2.angle;
    settling = false;
    hold_arm_started = false;
    yaw_ratio = 0.0f;
    yaw_score = 0.0f;
    motor1_pid_angle.clear();
    motor1_pid_speed.clear();
    motor2_pid_angle.clear();
    motor2_pid_speed.clear();
    motor3_pid_angle.clear();
    stage = 1;
  }

  const bool following = stage == 3;
  const TrajectoryPoint ref = following ? trajectory_sample_at_ms(now - start_ms) : initial;
  // Yaw保持编码器闭环；motor2在摆臂到位后切换到IMU Pitch闭环并持续保持起点。
  drive_motor_feedback(
    ref.yaw, stage == 1 ? motor2_hold : initial.pitch, arm_target,
    stage == 1 ? motor2.angle : imu.pitch, following ? YAW_SPEED_FF_GAIN * ref.yaw_vel : 0.0f,
    following ? YAW_ACCEL_FF_GAIN * ref.yaw_acc : 0.0f, false, yaw_offset);

  if (following) {
    yaw_detect(imu.yaw, ref.yaw, &yaw_ratio);
    yaw_score = 100.0f * yaw_ratio;
    return;
  }
  const bool reached = std::abs(motor3.angle - arm_target) < kTolerance &&
                       (stage == 1 || std::abs(imu.pitch - initial.pitch) < kTolerance);
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
  if (stage == 1) {
    motor2_pid_angle.clear();
    motor2_pid_speed.clear();
    stage = 2;
  }
  else {
    start_ms = osKernelSysTick();
    stage = 3;
  }
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
  motor1_friction_ff = 0.0f;
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
  float motor1_set, float motor2_set, float motor3_set, float motor2_feedback, float yaw_speed_ff,
  float yaw_accel_ff, bool use_imu_yaw, float motor1_angle_offset)
{
  // IMU闭环按最短角度纠偏；编码器保持模式仍使用原来的多圈角度。
  const float yaw_feedback =
    use_imu_yaw ? motor1_set - std::remainder(motor1_set - imu.yaw, 2.0f * 3.14159265f)
                : motor1.angle + motor1_angle_offset;
  motor1_pid_angle.calc(motor1_set, yaw_feedback);
  motor1_accel_ff = yaw_accel_ff;  // 普通保持模式默认传入0
  motor1_pid_speed.calc(
    sp::limit_max(motor1_pid_angle.out + yaw_speed_ff, YAW_TARGET_SPEED_MAX), motor1.speed);
  motor2_pid_angle.calc(motor2_set, motor2_feedback);
  motor2_pid_speed.calc(motor2_pid_angle.out, motor2.speed);
  motor3_pid_angle.calc(motor3_set, motor3.angle);

  motor1_friction_ff = friction_feedforward(motor1_pid_angle.data.err[0], MOTOR1_FRICTION_TORQUE);
  motor1_torque_cmd =
    sp::limit_max(motor1_pid_speed.out + motor1_accel_ff + motor1_friction_ff, MOTOR1_MAX_TORQUE);
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

extern "C" void hold_arm()
{
  static float motor2_hold = 0.0f;
  static bool near_waiting = false;
  static uint32_t near_start_rx_ms = 0;
  const uint32_t now_ms = osKernelSysTick();
  if (!hold_arm_started) {
    hold_arm_started = true;
    motor2_hold = motor2.angle;  // 未指定motor2复位目标，保持进入模式时的位置
    arm_folded = false;
    arm_waiting_reset = false;
    near_waiting = false;
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
    arm_waiting_reset = false;
    if (!near_waiting) {
      near_waiting = true;
      near_start_rx_ms = laser.last_valid_rx_ms;
    }
    // 必须有跨越确认时间的新测距反馈，避免同一个噪声样本被反复读取后触发。
    if (laser.last_valid_rx_ms - near_start_rx_ms >= FOLD_TRIGGER_DELAY_MS) {
      arm_folded = true;
    }
  }
  else {
    near_waiting = false;
    if (arm_folded) {
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
  }
  drive_motor(
    motor1_startup_angle, motor2_hold, arm_folded ? ARM_FOLD_MOTOR3_RAD : ARM_RESET_MOTOR3_RAD);
}

void track_arm()
{
  if (!track_latched) {
    track_latched = true;
    pitch_test_stage = 0;
    hold_arm_started = false;  // 下次返回保持时重新记录motor2保持位置、清空PID
  }
  test_pitch_trajectory(true, true);  // 先到达轨迹起点，再同时跟踪两轴
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)  //CAN回调
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.frame_type || can1.rx_len != 8) continue;
      if (can1.rx_id == motor3.rx_id)
        motor3.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor2.rx_id)
        motor2.read(can1.rx_data, stamp_ms);
    }
    if (hcan == &hcan2) {
      can2.recv();

      if (!can2.frame_type && can2.rx_id == motor1.rx_id && can2.rx_len == 8) {
        motor1.read(can2.rx_data, stamp_ms);
        // 仅在上电后第一帧有效反馈时记录，切换模式和掉线重连均不覆盖。
        if (!motor1_startup_ready) {
          motor1_startup_angle = motor1.angle;
          motor1_startup_ready = true;
        }
      }
      else if (!can2.frame_type && can2.rx_id == 0x111) {
        if (
          can2.rx_len < 3 || can2.rx_data[0] > static_cast<uint8_t>(sp::DBusSwitchMode::UP) ||
          can2.rx_data[1] > static_cast<uint8_t>(sp::DBusSwitchMode::UP)) {
          dbus_sw_r = sp::DBusSwitchMode::DOWN;
          dbus_sw_l = sp::DBusSwitchMode::DOWN;
          auto_track_requested = false;
          connect_last_rx_ms = stamp_ms;
          continue;
        }  //底盘遥控器断联，can通信却良好
        else {
          uint8_t raw = can2.rx_data[0];
          connect_last_rx_ms = stamp_ms;  //负责两板子can通信检查
          dbus_sw_r = static_cast<sp::DBusSwitchMode>(raw);
          raw = can2.rx_data[1];
          dbus_sw_l = static_cast<sp::DBusSwitchMode>(raw);
          auto_track_requested = can2.rx_data[2] == 1;
        }
      }
    }
  }
}

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
