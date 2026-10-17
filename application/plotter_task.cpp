#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "laser.hpp"

extern Laser laser;

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void * argument)
{
  while (true) {
    plotter.plot((float)(laser.distance));
    osDelay(10);
  }
}
