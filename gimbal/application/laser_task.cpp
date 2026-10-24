#include "cmsis_os.h"
#include "laser.hpp"

Laser laser(&huart6);
extern "C" void laser_task(void * argument)
{
  laser.config();
  laser.request();
  while (true) {
    osDelay(10);
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart6) {
    laser.update(LASER_TX_BUFFER_SIZE);
    laser.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart6) {
    laser.request();
  }
}