#include "cmsis_os.h"

extern "C" void laser_task(void * argument)
{
  while (true) {
    osDelay(1);
  }
}