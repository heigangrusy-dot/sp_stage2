#include "cmsis_os.h"

extern "C" void plotter_task(void * argument)
{
  while (true) {
    osDelay(1);
  }
}