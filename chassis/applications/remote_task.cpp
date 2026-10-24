#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "school_pm02.hpp"

extern SchoolPm02 pm02;

sp::DBus remote(&huart3);

extern "C" void remote_task(void const * argument)
{
  (void)argument;

  remote.request();

  while (true) {
    osDelay(10);
  }
}

// ---------- 串口接收分发 ----------
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart == &huart3) {
    // USART3：遥控器。
    remote.update(Size, osKernelSysTick());
    remote.request();
  }
  else if (huart == pm02.huart) {
    // USART6：裁判系统。
    pm02.update(Size);
    pm02.request();
  }
}

// ---------- 串口错误恢复 ----------
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == pm02.huart) {
    pm02.reset_receive();
  }
}