#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void * argument)
{
  led.start();
  while (1) {
    led.set(1, 0, 0);
    osDelay(400);
    led.set(0, 1, 0);
    osDelay(400);
    led.set(0, 0, 1);
    osDelay(400);
  }
}