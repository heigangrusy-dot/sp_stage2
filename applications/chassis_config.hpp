#ifndef CHASSIS_CONFIG_HPP
#define CHASSIS_CONFIG_HPP

namespace chassis_config
{
// 底盘几何参数，单位：m
constexpr float WHEEL_RADIUS = 0.077f;
constexpr float HALFLENGTH = 0.165f;
constexpr float half_width = 0.185f;

// 电机轴转速 / 输出轴转速，无量纲。
constexpr float GEAR_RATIO = 14.9f;
}  // namespace chassis_config

#endif  // CHASSIS_CONFIG_HPP