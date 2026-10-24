#include "io/plotter/plotter.hpp"

#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "cmsis_os.h"
#include "plotter.hpp"
#include "task.h"

namespace
{
// ---------- 串口绘图总开关：false时不向USART1发送任何数据 ----------
// 任务照常按周期唤醒、快照照常更新，只是不再调用发送；恢复时改回true。
constexpr bool PLOT_ENABLED = true;

// ---------- 绘图模式：只改这一行即可切换11~14通道 ----------
enum class PlotMode
{
  TORQUE,
  HEADING,
  AUTO_DRIVE
};
constexpr PlotMode PLOT_MODE = PlotMode::AUTO_DRIVE;

// TORQUE：四轮最终转矩，N·m，顺序左前/左后/右前/右后。
// HEADING：实际Yaw(rad)、目标Yaw(rad)、角速度(rad/s)、修正角速度(rad/s)。
// AUTO_DRIVE：距离(m)、目标距离(m)、前向命令(m/s)、状态(0待命1行驶2停车3完成4中止)。

// ---------- 串口对象与数据快照 ----------
sp::Plotter plotter(&huart1);
PlotSnapshot latest_data;
bool data_ready = false;

// ---------- 通道选择与串口输出 ----------
void send_plot_data(const PlotSnapshot & data)
{
  float channel11 = data.torque[0];
  float channel12 = data.torque[1];
  float channel13 = data.torque[2];
  float channel14 = data.torque[3];

  if constexpr (PLOT_MODE == PlotMode::HEADING) {
    channel11 = data.yaw;
    channel12 = data.target_yaw;
    channel13 = data.yaw_rate;
    channel14 = data.yaw_correction;
  }
  else if constexpr (PLOT_MODE == PlotMode::AUTO_DRIVE) {
    channel11 = data.auto_distance;
    channel12 = data.auto_target_distance;
    channel13 = data.forward_command;
    channel14 = data.auto_phase;
  }

  // 固定18通道，与现有SerialPlot帧设置一致；发送在临界区外进行。
  plotter.plot(
    data.target_speed[0],                 // 1：左前目标轮速，rad/s。
    data.actual_speed[0],                 // 2：左前实际轮速，rad/s。
    data.target_speed[1],                 // 3：左后目标轮速，rad/s。
    data.actual_speed[1],                 // 4：左后实际轮速，rad/s。
    data.target_speed[2],                 // 5：右前目标轮速，rad/s。
    data.actual_speed[2],                 // 6：右前实际轮速，rad/s。
    data.target_speed[3],                 // 7：右后目标轮速，rad/s。
    data.actual_speed[3],                 // 8：右后实际轮速，rad/s。
    static_cast<float>(data.elapsed_ms),  // 9：快照时间戳，ms。
    data.output_allowed ? 1.0f : 0.0f,    // 10：底盘输出许可。
    channel11,                            // 11：所选模式的第1项，含义见文件顶部。
    channel12,                            // 12：所选模式的第2项。
    channel13,                            // 13：所选模式的第3项。
    channel14,                            // 14：所选模式的第4项。
    data.power_online ? 1.0f : 0.0f,      // 15：功率计在线。
    data.requested_power,                 // 16：限制前请求功率，W。
    data.power_budget,                    // 17：扣除余量后的功率限幅线，W。
    data.measured_power);                 // 18：实测功率，W。
}
}  // namespace

// ---------- 控制任务发布快照 ----------
void update_plot_data(const PlotSnapshot & data)
{
  // 短临界区只复制数据，防止绘图任务读到两轮混合的快照。
  taskENTER_CRITICAL();
  latest_data = data;
  data_ready = true;
  taskEXIT_CRITICAL();
}

// ---------- 独立绘图任务 ----------
extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    PlotSnapshot data;
    taskENTER_CRITICAL();
    data = latest_data;
    const bool ready = data_ready;
    taskEXIT_CRITICAL();

    // PLOT_ENABLED为false时整条发送链不执行，快照仍保持最新，改回true即恢复。
    if (PLOT_ENABLED && ready && huart1.gState == HAL_UART_STATE_READY) {
      send_plot_data(data);
    }
    // 观测不需要追赶错过的周期，避免调试/调度延迟后连续补发。
    osDelay(chassis_config::PLOT_PERIOD_MS);
  }
}
