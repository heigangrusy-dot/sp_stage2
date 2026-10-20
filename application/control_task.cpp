#include <cmath>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "laser.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

sp::RM_Motor motor1(1, sp::RM_Motors::GM6020);                //云台底部，用can2
sp::RM_Motor motor2(2, sp::RM_Motors::GM6020);                //侧面用can1
sp::RM_Motor motor3(3, sp::RM_Motors::M2006, sp::M2006_P36);  //侧面用can1

sp::PID motor1_pid_angle(
  0.001f, 1.0f, 0.1f, 0.0f, 2.5f, 0.3f, 0.7f, false, true);  //云台底部yaw电机
sp::PID motor2_pid_angle(
  0.001f, 0.1f, 0.1f, 0.0f, 1.0f, 0.1f, 1.0f, false, true);  //云台侧面摆头pitch电机
sp::PID motor3_pid_angle(
  0.001f, 0.57f, 0.1f, 0.0f, 1.0f, 0.1f, 1.0f, false, true);  //云台侧面摆臂pitch电机

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::DBusSwitchMode dbus_sw_r = sp::DBusSwitchMode::DOWN;
sp::DBusSwitchMode dbus_sw_l = sp::DBusSwitchMode::DOWN;
extern Laser laser;
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

// ==================== motor3重力前馈（上电时横杆水平） ====================
constexpr float MOTOR3_GRAVITY_TORQUE = 0.5f;  // 水平时补偿力矩，N·m（含力臂的等效系数）
constexpr float MOTOR3_MAX_TORQUE = 2.5f;      // PID与前馈叠加后的限幅，N·m
static float motor3_horizontal_angle = 0.0f;
static bool motor3_horizontal_ready = false;
float motor3_gravity_ff = 0.0f;  // 调试观察：实际加入的重力前馈
// ==================== motor3重力前馈结束 ====================

constexpr uint32_t CONNECT_OFFLINE_MS = 50;
constexpr float MOTOR3_NORMAL_RAD = -3.14159265f / 3.0f;     // -60°
constexpr float MOTOR2_TRACK_ZERO_RAD = 3.14159265f / 4.0f;  // 45°
constexpr float MOTOR3_FOLD_RAD = -3.14159265f / 12.0f;      // -30°
constexpr uint16_t FOLD_DISTANCE_CM = 40;                    //想一直处于初始状态则值调为大于200即可
constexpr uint32_t FOLD_RESET_DELAY_MS = 1000;

//折叠复位功能相关标志
uint32_t connect_last_rx_ms = 0;
bool arm_folded = false;
bool arm_waiting_reset = false;
uint32_t fold_exit_ms = 0;

//轨迹跟踪功能相关标志
bool track_latched = false;
float track_yaw0 = 0.0f;
float track_pitch0 = 0.0f;

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
  const float arm_target = motor3_horizontal_angle - 60.0f * kDeg;
  if (pitch_test_stage == 0) {
    yaw_hold = motor1.angle;
    motor2_hold = motor2.angle;
    motor1_pid_angle.clear();
    motor2_pid_angle.clear();
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

  if (pitch_test_stage == 3) return;
  const bool reached = pitch_test_stage == 1 ? std::abs(motor3.angle - arm_target) < 2.0f * kDeg
                                             : std::abs(imu.pitch) < 1.0f * kDeg;
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
    pitch_test_stage = 3;
  }
}
// ==================== 临时Pitch测试结束 ====================

// ==================== 临时：仅测试Yaw轨迹（整段可删除） ====================
static void test_yaw_trajectory(bool enabled)
{
  static bool started = false;
  static uint32_t start_ms = 0;
  static float angle0 = 0.0f;

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
    motor1_pid_angle.clear();
  }

  const float target =
    angle0 + trajectory_sample_at_ms(now - start_ms).yaw - trajectory_sample_at_ms(0).yaw;
  motor1_pid_angle.calc(target, motor1.angle);
  motor1.cmd(motor1_pid_angle.out);
  motor2.cmd(0.0f);
  motor3.cmd(0.0f);

  motor1.write(can2.tx_data);
  motor2.write(can1.tx_data);
  motor3.write(can1.tx_data);
  can2.send(motor1.tx_id);
  can1.send(motor2.tx_id);
  can1.send(motor3.tx_id);
}
// ==================== 临时Yaw测试结束 ====================

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

    switch (dbus_sw_r) {
      case sp::DBusSwitchMode::DOWN:  //右拨杆下档，失能
        track_latched = false;
        test_pitch_trajectory(false);  // 临时Pitch测试：退出时复位
        break;
      case sp::DBusSwitchMode::MID:
      case sp::DBusSwitchMode::UP:
        test_pitch_trajectory(motors_alive);  // 临时Pitch测试
        //drive_motor(motor1_zero, motor2_zero, motor3_zero - 1.0f);  //驱动电机
        // hold_arm(motor1_zero, motor2_zero, motor3_zero);  //保持机械臂
        // test_yaw_trajectory(motor1.is_alive(osKernelSysTick()));  // 临时Yaw测试
        // if (dbus_sw_l == sp::DBusSwitchMode::UP) {
        //   track_arm(motor1_zero, motor2_zero, motor3_zero);
        // }
        // else {
        //   track_latched = false;
        //   hold_arm(motor1_zero, motor2_zero, motor3_zero);
        // }
        break;
    }

    osDelay(1);
  }
}

extern "C" void lose_power()  //失能函数
{
  motor1_pid_angle.clear();
  motor2_pid_angle.clear();
  motor3_pid_angle.clear();
  motor3_gravity_ff = 0.0f;
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
  motor2_pid_angle.calc(motor2_set, motor2_feedback);
  motor3_pid_angle.calc(motor3_set, motor3.angle);

  motor1.cmd(motor1_pid_angle.out);
  motor2.cmd(motor2_pid_angle.out);
  // ==================== motor3重力前馈 ====================
  // 负向转动为抬升，水平夹角 = -(当前角度 - 上电水平角度)，单位rad。
  const float arm_pitch = -(motor3.angle - motor3_horizontal_angle);
  motor3_gravity_ff = motor3_horizontal_ready ? -MOTOR3_GRAVITY_TORQUE * std::cos(arm_pitch) : 0.0f;
  const float motor3_torque =
    sp::limit_max(motor3_pid_angle.out + motor3_gravity_ff, MOTOR3_MAX_TORQUE);
  motor3.cmd(motor3_horizontal_ready ? motor3_torque : 0.0f);
  // ==================== motor3重力前馈结束 ====================

  motor1.write(can2.tx_data);
  motor2.write(can1.tx_data);
  motor3.write(can1.tx_data);

  can2.send(motor1.tx_id);
  can1.send(motor2.tx_id);
  can1.send(motor3.tx_id);
}

extern "C" void hold_arm(float motor1_zero, float motor2_zero, float motor3_zero)
{
  float motor3_set = MOTOR3_NORMAL_RAD;
  uint32_t now_ms = osKernelSysTick();

  if (laser.distance < FOLD_DISTANCE_CM) {
    arm_folded = true;
    arm_waiting_reset = false;
    motor3_set = MOTOR3_FOLD_RAD;
  }
  else if (arm_folded) {
    if (!arm_waiting_reset) {
      arm_waiting_reset = true;
      fold_exit_ms = now_ms;
    }
    if (now_ms - fold_exit_ms < FOLD_RESET_DELAY_MS) {
      motor3_set = MOTOR3_FOLD_RAD;
    }
    else {
      arm_folded = false;
      arm_waiting_reset = false;
    }
  }

  drive_motor(motor1_zero, motor2_zero, motor3_zero + motor3_set);
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
