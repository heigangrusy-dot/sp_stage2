#include "FreeRTOS.h"
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
    taskENTER_CRITICAL();
    if (remote.is_alive(osKernelSysTick())) {
      sw_r = static_cast<uint8_t>(remote.sw_r);
    }
    taskEXIT_CRITICAL();
    can2.tx_data[0] = sw_r;
    can2.tx_data[1] = 0;
    can2.tx_data[2] = 0;
    can2.tx_data[3] = 0;
    can2.tx_data[4] = 0;
    can2.tx_data[5] = 0;
    can2.tx_data[6] = 0;
    can2.tx_data[7] = 0;
    can2.send(0x111);
    osDelay(10);
  }
}