#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "plotter.hpp"
#include "task.h"

namespace
{
sp::Plotter plotter(&huart1);
PlotData latest_data = {};
bool data_ready = false;
}  // namespace

void update_plot_data(const PlotData & data)
{
  // 短临界区只复制数据，防止绘图任务读到两轮混合的快照。
  taskENTER_CRITICAL();
  latest_data = data;
  data_ready = true;
  taskEXIT_CRITICAL();
}

extern "C" void plotter_task(void const * argument)
{
  (void)argument;
  uint32_t previous_wake_time = osKernelSysTick();

  while (true) {
    PlotData data;
    taskENTER_CRITICAL();
    data = latest_data;
    const bool ready = data_ready;
    taskEXIT_CRITICAL();

    // 串口发送在临界区外执行，仅此任务调用plotter。
    // 通道1~8：目标/反馈；9~10：在线/许可；11~14：转矩；15：左前积分。
    if (ready) {
      plotter.plot(
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8],
        data[9], data[10], data[11], data[12], data[13], data[14]);
    }
    osDelayUntil(&previous_wake_time, chassis_config::PLOT_PERIOD_MS);
  }
}
