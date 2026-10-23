#ifndef APPLICATIONS_IMU_TASK_HPP
#define APPLICATIONS_IMU_TASK_HPP

// C板固定在底盘上；Yaw与角速度均以逆时针为正，单位rad、rad/s。
struct ImuHeading
{
  float yaw = 0.0f;
  float yaw_rate = 0.0f;
  bool valid = false;  // 完成静置校准、数据有限且未超时。
};

ImuHeading read_imu_heading();

#endif  // APPLICATIONS_IMU_TASK_HPP
