#ifndef CHASSIS_CONFIG_HPP
#define CHASSIS_CONFIG_HPP
#include <cstdint>

namespace chassis_config
{
// 底盘几何参数，单位：m
constexpr float WHEEL_RADIUS = 0.077f;
constexpr float HALF_LENGTH = 0.165f;
constexpr float HALF_WIDTH = 0.185f;

// 电机轴转速 / 输出轴转速，无量纲。
constexpr float GEAR_RATIO = 13.7f;  // 按导师确认的实际底盘减速比。

// 初期目标计算使用的单轴线速度幅值，单位m/s。
constexpr float MAX_LINEAR_SPEED = 0.55f;

// 横移先单独限制为低速，单位：m/s。
constexpr float MAX_LATERAL_SPEED = 0.6f;

// 前后、横移速度指令的最大变化率，单位：m/s²。
constexpr float MAX_LINEAR_ACCELERATION = 0.55f;

// 拨轮对应的目标角速度幅值，单位rad/s。
constexpr float MAX_ANGULAR_SPEED = 1.0f;

// 旋转指令的最大变化率，单位：rad/s²。
constexpr float MAX_ANGULAR_ACCELERATION = 1.0f;

// 摇杆归中附近的小变化视为0，无量纲，后续按实测调整。
constexpr float INPUT_DEADBAND = 0.05f;

// 计划控制周期：1ms。实际运行是否按期完成，后续需要验证。
constexpr uint32_t CONTROL_PERIOD_MS = 1;

// PID单位换算。
constexpr float CONTROL_PERIOD_S = CONTROL_PERIOD_MS * 0.001f;

// 曲线每20ms刷新一次，与1ms控制周期分开。
constexpr uint32_t PLOT_PERIOD_MS = 20;

// 四轮共用的速度环参数；实际负载变化后需复查。
constexpr float SPEED_KP = 0.25f;
constexpr float SPEED_KI = 0.14f;
constexpr float SPEED_KD = 0.0f;
constexpr float MAX_WHEEL_TORQUE = 0.8f;
constexpr float MAX_INTEGRAL_TORQUE = 0.3f;
}  // namespace chassis_config

#endif  // CHASSIS_CONFIG_HPP
