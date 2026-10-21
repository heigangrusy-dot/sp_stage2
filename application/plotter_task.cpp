#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "laser.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern Laser laser;
extern sp::RM_Motor motor1;
extern sp::RM_Motor motor2;
extern sp::RM_Motor motor3;

extern sp::PID motor1_pid_angle;
extern sp::PID motor2_pid_angle;
extern sp::PID motor3_pid_angle;

extern sp::Mahony imu;
extern float pitch_test_target;
extern float motor3_gravity_ff;

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void * argument)
{
  while (true) {
    // ==================== 临时：Yaw轨迹对比（目标、实际，单位rad） ====================
    // plotter.plot(motor1_pid_angle.data.set, motor1.angle);
    // ==================== 临时Yaw轨迹对比结束 ====================
    // 临时Pitch测试：目标、实际姿态，单位rad。
    plotter.plot(
      pitch_test_target, imu.pitch, motor2.angle, motor2_pid_angle.out, motor3.angle,
      motor3_pid_angle.out);  //调试电机2和电机3
    // plotter.plot(motor3.angle, motor3_pid_angle.out, (float)(laser.distance));  //调试电机3
    //plotter.plot((float)(laser.distance));
    // plotter.plot(
    //   motor3_pid_angle.data.set,  // 目标
    //   motor3.angle,               // 实际角度
    //   motor3_pid_angle.data.pout, motor3_pid_angle.data.iout, motor3_gravity_ff,
    //   motor3.torque);  // 电流换算的反馈力矩
    osDelay(10);
  }
}
