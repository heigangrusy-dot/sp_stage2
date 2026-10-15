#include "cmsis_os.h"

extern "C" void control_task(void * argument)
{
  while (1) {
    osDelay(1);
  }
}