#ifndef APPLICATIONS_PLOTTER_HPP
#define APPLICATIONS_PLOTTER_HPP

#include <array>

// 控制任务发布本轮观测快照；通道排列、模式切换和单位说明统一放在plotter.cpp。
struct PlotSnapshot
{
  // 四轮顺序：左前、左后、右前、右后。
  std::array<float, 4> target_speed{};
  std::array<float, 4> actual_speed{};
  std::array<float, 4> torque{};
  bool output_allowed = false;

  bool power_online = false;
  float pm02_limit_w = 0.0f;
  float requested_power = 0.0f;
  float power_budget = 0.0f;
  float measured_power = 0.0f;

  float yaw = 0.0f;
  float target_yaw = 0.0f;
  float yaw_rate = 0.0f;
  float yaw_correction = 0.0f;

  float auto_distance = 0.0f;
  float auto_target_distance = 0.0f;
  float forward_command = 0.0f;
  float auto_phase = 0.0f;
};

void update_plot_data(const PlotSnapshot & data);

#endif  // APPLICATIONS_PLOTTER_HPP
