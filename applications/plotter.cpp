#include "io/plotter/plotter.hpp"

#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "cmsis_os.h"
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

  while (true) {
    PlotData data;
    taskENTER_CRITICAL();
    data = latest_data;
    const bool ready = data_ready;
    taskEXIT_CRITICAL();

    // 串口发送在临界区外执行，仅此任务调用plotter。
    // 1~8：四轮目标/反馈；9：快照时间戳(ms)；10：输出许可；11~14：最终命令转矩。
    // 15：功率计在线；16：限制前请求功率(W)；17：扣除余量后的预算(W)；18：实测功率(W)。
    // PLOT_HEADING=true时，11~14改为Yaw、目标Yaw、Yaw角速度、修正角速度。
    if (ready && huart1.gState == HAL_UART_STATE_READY) {
      plotter.plot(
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
        data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17]);
    }
    // 观测不需要追赶错过的周期，避免调试/调度延迟后连续补发。
    osDelay(chassis_config::PLOT_PERIOD_MS);
  }
}
