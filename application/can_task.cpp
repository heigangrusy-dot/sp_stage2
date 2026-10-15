#include "cmsis_os.h"

extern "C" void can_task(void * argument)
{
  while (true) {
    osDelay(1);
  }
}