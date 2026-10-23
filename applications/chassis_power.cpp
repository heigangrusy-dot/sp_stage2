#include "chassis_power.hpp"

#include <cmath>

#include "chassis_config.hpp"

namespace
{
// ---------- 功率模型 ----------
// 转矩统一乘s后：P(s)=a*s²+b*s+c；轮速不能瞬间随指令缩放。
struct PowerModel
{
  float a = 0.0f;
  float b = 0.0f;
  float c = chassis_config::POWER_K3;

  float at(float s) const { return (a * s + b) * s + c; }
};

static_assert(
  chassis_config::POWER_K1 >= 0.0f && chassis_config::POWER_K2 >= 0.0f &&
  chassis_config::POWER_K3 >= 0.0f);

PowerModel make_model(const float torques[4], const float speeds[4])
{
  PowerModel model;
  for (int i = 0; i < 4; ++i) {
    model.a += chassis_config::POWER_K1 * torques[i] * torques[i];
    model.b += torques[i] * speeds[i];
    model.c += chassis_config::POWER_K2 * speeds[i] * speeds[i];
  }
  return model;
}
}  // namespace

float predict_chassis_power(const float torques[4], const float speeds[4])
{
  return make_model(torques, speeds).at(1.0f);
}

// ---------- 统一转矩限幅 ----------
PowerLimitResult limit_chassis_power(float torques[4], const float speeds[4], float budget_w)
{
  const auto model = make_model(torques, speeds);
  PowerLimitResult result;
  result.before_w = model.at(1.0f);
  result.after_w = result.before_w;

  const bool valid = std::isfinite(budget_w) && budget_w >= 0.0f && std::isfinite(model.a) &&
                     std::isfinite(model.b) && std::isfinite(model.c) &&
                     std::isfinite(result.before_w);
  if (valid && result.before_w <= budget_w) return result;

  // 制动时b可能为负，P(s)并非全程递增；先找[0,1]内的最低点。
  float low = model.a > 0.0f ? -model.b / (2.0f * model.a) : 0.0f;
  if (low < 0.0f) low = 0.0f;
  if (low > 1.0f) low = 1.0f;
  result.feasible = valid && model.at(low) <= budget_w;
  result.scale = 0.0f;

  if (result.feasible) {
    // 在最低点右侧二分，取满足预算的最大转矩比例，保留可行侧端点。
    float high = 1.0f;
    for (int i = 0; i < 16; ++i) {
      const float mid = (low + high) * 0.5f;
      if (model.at(mid) <= budget_w)
        low = mid;
      else
        high = mid;
    }
    result.scale = low;
  }

  // 无解时不再输出转矩；惯性转动及静态功耗仍可能超过预算，不能宣称已限住。
  for (int i = 0; i < 4; ++i) {
    torques[i] = result.scale == 0.0f ? 0.0f : torques[i] * result.scale;
  }
  result.after_w = model.at(result.scale);
  return result;
}
