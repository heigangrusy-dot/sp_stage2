#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/bmi088/bmi088_defs.h"
#include "tim.h"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

constexpr float IMU_TEMP = 50.0f;
// PID参数
constexpr float IMU_TEMP_KP = 400.0f;  // 你可能需要根据实际发热丝的功率微调这个KP
constexpr float IMU_TEMP_KI = 0.0f;
constexpr float IMU_TEMP_KD = 0.0f;
constexpr float IMU_TEMP_MAXOUT = 2000.0f;  // 建议限制 PID 最大输出，防止静态积分时全功率烘烤
constexpr float IMU_TEMP_MAXIOUT = 0.0f;

sp::PID imu_temp_pid(
  1e-3, IMU_TEMP_KP, IMU_TEMP_KI, IMU_TEMP_KD, IMU_TEMP_MAXOUT, IMU_TEMP_MAXIOUT, 1.0f);

// 陀螺仪温度控制函数
void imu_temp_control(float temp)
{
  uint16_t tempPWM = 0;

  if (IMU_TEMP - temp > 10.0f) {
    tempPWM = 4000;
  }
  else {
    imu_temp_pid.calc(IMU_TEMP, temp);

    // 限制输出下限，防止输出负数导致 PWM 溢出异常
    if (imu_temp_pid.out < 0.0f) {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.out;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, tempPWM);
}
const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);

extern "C" void imu_task(void * argument)
{
  bmi088.init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  while (true) {
    bmi088.update();
    imu_temp_control(bmi088.temp);
    imu.update(bmi088.acc, bmi088.gyro);
    osDelay(1);
  }
}