#include "imu_task.hpp"

#include <cmath>

#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "task.h"
#include "tools/mahony/mahony.hpp"

namespace
{
constexpr float R_AB[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
ImuHeading heading_feedback;
uint32_t last_heading_ms = 0;
static_assert(
  chassis_config::IMU_CALIBRATION_SAMPLES > 0 && chassis_config::IMU_HEADING_TIMEOUT_MS > 0);
static_assert(
  chassis_config::HEADING_YAW_SIGN == 1.0f || chassis_config::HEADING_YAW_SIGN == -1.0f);
}  // namespace

//C板板载 BMI088
sp::BMI088 bmi088(&hspi1, CS1_ACC_GPIO_Port, CS1_ACC_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, R_AB);

sp::Mahony imu(1e-3f);

// ---------- 航向快照 ----------
ImuHeading read_imu_heading()
{
  taskENTER_CRITICAL();
  auto result = heading_feedback;
  const auto age_ms = osKernelSysTick() - last_heading_ms;
  taskEXIT_CRITICAL();
  result.valid = result.valid && age_ms < chassis_config::IMU_HEADING_TIMEOUT_MS;
  return result;
}

extern "C" void imu_task(void const * argument)
{
  (void)argument;

  bmi088.init();
  uint32_t previous_wake_ms = osKernelSysTick();
  float gyro_bias[3] = {};
  float gyro_sum[3] = {};
  uint32_t still_samples = 0;
  bool calibrated = false;

  while (true) {
    bmi088.update();

    // ---------- 开机静置零偏校准 ----------
    // 有明显转动或加速度异常就重新计数；校准期间仍可遥控，但不启用航向保持。
    if (!calibrated) {
      float acc_squared = 0.0f;
      float gyro_squared = 0.0f;
      for (int i = 0; i < 3; ++i) {
        acc_squared += bmi088.acc[i] * bmi088.acc[i];
        gyro_squared += bmi088.gyro[i] * bmi088.gyro[i];
      }
      const bool still =
        std::abs(std::sqrt(acc_squared) - 9.81f) < 0.5f && gyro_squared < 0.05f * 0.05f;
      if (still) {
        for (int i = 0; i < 3; ++i) gyro_sum[i] += bmi088.gyro[i];
        if (++still_samples >= chassis_config::IMU_CALIBRATION_SAMPLES) {
          for (int i = 0; i < 3; ++i) gyro_bias[i] = gyro_sum[i] / still_samples;
          calibrated = true;
        }
      }
      else {
        still_samples = 0;
        for (auto & sum : gyro_sum) sum = 0.0f;
      }
    }

    // ---------- 姿态更新与发布 ----------
    float corrected_gyro[3];
    for (int i = 0; i < 3; ++i) corrected_gyro[i] = bmi088.gyro[i] - gyro_bias[i];
    imu.update(bmi088.acc, corrected_gyro);
    const bool valid = calibrated && std::isfinite(imu.yaw) && std::isfinite(imu.vyaw);
    const ImuHeading feedback = {
      chassis_config::HEADING_YAW_SIGN * imu.yaw, chassis_config::HEADING_YAW_SIGN * imu.vyaw,
      valid};
    taskENTER_CRITICAL();
    heading_feedback = feedback;
    last_heading_ms = osKernelSysTick();
    taskEXIT_CRITICAL();
    osDelayUntil(&previous_wake_ms, 1);
  }
}
