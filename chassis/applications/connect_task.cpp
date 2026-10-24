#include "FreeRTOS.h"
#include "auto_drive.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "task.h"

sp::CAN can2(&hcan2);
extern sp::DBus remote;
extern "C" void connect_task(void const * argument)
{
  (void)argument;

  can2.config();
  can2.start();

  while (true) {
    uint8_t sw_r = 0xFF;
    uint8_t sw_l = 0xFF;
    bool arrived = false;
    taskENTER_CRITICAL();
    if (remote.is_alive(osKernelSysTick())) {
      sw_r = static_cast<uint8_t>(remote.sw_r);
      sw_l = static_cast<uint8_t>(remote.sw_l);
      arrived = remote.sw_r == sp::DBusSwitchMode::UP && chassis_auto_complete();
    }
    taskEXIT_CRITICAL();
    can2.tx_data[0] = sw_r;
    can2.tx_data[1] = sw_l;
    can2.tx_data[2] = arrived ? 1 : 0;
    can2.tx_data[3] = 0;
    can2.tx_data[4] = 0;
    can2.tx_data[5] = 0;
    can2.tx_data[6] = 0;
    can2.tx_data[7] = 0;
    can2.send(0x111);
    osDelay(10);
  }
}
