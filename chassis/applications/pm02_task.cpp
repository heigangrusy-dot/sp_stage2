#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "school_pm02.hpp"
#include "task.h"

// ---------- PM02通信对象 ----------
SchoolPm02 pm02(&huart6);

// ---------- PM02接收任务 ----------
extern "C" void pm02_task(void const * argument)
{
  (void)argument;

  // 启动USART6接收；后续由串口回调解析并重新启动接收。
  pm02.request();

  while (true) {
    // 沿用中间件示例，维护雷达扩展数据的超时。
    // 这不是底盘功率上限的在线判断。
    taskENTER_CRITICAL();
    pm02.update_radar_data_timeout(HAL_GetTick());
    taskEXIT_CRITICAL();

    osDelay(10);
  }
}