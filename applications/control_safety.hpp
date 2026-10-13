#ifndef CONTROL_SAFETY_HPP
#define CONTROL_SAFETY_HPP

namespace control
{
class ControlSafety
{
public:
  // remote_valid：遥控数据有效且未超时。
  // switch_down：右拨杆是否在下档；remote_valid为false时不使用它。
  // motion_requested：当前模式是否请求运动。
  // system_ready：该控制系统的初始化、关键反馈等是否满足使能条件。
  bool update(bool remote_valid, bool switch_down, bool motion_requested, bool system_ready);

  void reset();

private:
  bool down_observed_ = false;
};

}  // namespace control

#endif  // CONTROL_SAFETY_HPP