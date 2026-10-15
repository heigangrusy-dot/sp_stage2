#include "cmsis_os.h"

extern "C" void uart_task(void * argument)
{
  while (true) {
    osDelay(1);
  }
}