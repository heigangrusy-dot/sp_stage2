#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task(void * argument)
{
  buzzer.start();
  uint8_t nums = 0;
  while (1) {
    if (nums >= 3) {
      osDelay(50);
      buzzer.stop();
    }
    else {
      uint16_t freq[7] = {262, 294, 330, 349, 392, 440, 494};  //中音区频率，高x2，低/2
      for (int i = 0; i < 1; i++) {
        buzzer.set(freq[i], 0.005);
        osDelay(50);
      }
      nums++;
    }
    osDelay(10);
  }
  // osDelay(100);
}

// if (dbus_connected_buzzer && remote.is_alive(osKernelSysTick()) && nums >= 3) {  //连接成功
//   buzzer.set(440, 0.005);
//   dbus_connected_buzzer = 0;
//   osDelay(100);
//   buzzer.stop();
// }
// else if (
//   !dbus_connected_buzzer && !remote.is_alive(osKernelSysTick()) && nums >= 3) {  //断开连接
//   buzzer.set(220, 0.005);
//   osDelay(50);
//   buzzer.set(110, 0.005);
//   osDelay(50);
//   dbus_connected_buzzer = 1;
//   buzzer.stop();
// }