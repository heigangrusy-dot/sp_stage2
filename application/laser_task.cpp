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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart6) {
    laser.request();
    laser.update(Size);
  }
}