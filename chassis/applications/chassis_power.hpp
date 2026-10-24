#ifndef APPLICATIONS_CHASSIS_POWER_HPP
#define APPLICATIONS_CHASSIS_POWER_HPP

// 四轮顺序：左前、左后、右前、右后。
// torques：轮侧命令转矩，N·m。
// speeds：轮侧实际角速度，rad/s。
// 返回预测输入功率，W；系数见chassis_config.hpp，误差需按工况验证。
float predict_chassis_power(const float torques[4], const float speeds[4]);

// 限制结果可在调试器中观察；feasible只表示模型内可行，不保证实测功率。
struct PowerLimitResult
{
  float scale = 1.0f;     // 四轮统一缩放系数，0~1。
  float before_w = 0.0f;  // 限制前预测功率。
  float after_w = 0.0f;   // 限制后预测功率。
  bool feasible = true;   // 当前轮速下，是否存在满足预算的缩放系数。
};

// 原地修改四轮转矩，speeds保持不变；budget_w已扣除误差余量。
// 若预算不可满足或输入无效，转矩清零并返回feasible=false。
PowerLimitResult limit_chassis_power(float torques[4], const float speeds[4], float budget_w);

#endif  // APPLICATIONS_CHASSIS_POWER_HPP
