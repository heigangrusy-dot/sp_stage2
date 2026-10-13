#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

namespace
{
constexpr float R_AB[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
}  // namespace

//C板板载 BMI088
sp::BMI088 bmi088(&hspi1, CS1_ACC_GPIO_Port, CS1_ACC_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, R_AB);

sp::Mahony imu(1e-3f);

sp::Plotter plotter(&huart1);

extern "C" void imu_task(void const * argument)
{
  (void)argument;

  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    // 顺序：加速度3轴 + 角速度3轴 + 温度 + 姿态角3个 = 10路
    plotter.plot(
      bmi088.acc[0], bmi088.acc[1], bmi088.acc[2], bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],
      bmi088.temp, imu.roll, imu.pitch, imu.yaw);

    osDelay(1);
  }
}