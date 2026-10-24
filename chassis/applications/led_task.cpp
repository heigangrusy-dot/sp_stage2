#include "cmsis_os.h"
#include "io/led/led.hpp"

namespace
{
//一次呼吸分20步，每步25ms -> 渐亮 500ms + 渐暗 500ms = 1 秒呼吸一次
constexpr uint8_t BREATH_STEPS = 20;
constexpr uint32_t BREATH_STEP_MS = 25;
constexpr float BRIGHTNESS_STEP = 1.0f / BREATH_STEPS;

//三个颜色的亮度权重：红 -> 绿 -> 蓝
constexpr float COLORS[3][3] = {
  {1.0f, 0.0f, 0.0f},  // 红色
  {0.0f, 1.0f, 0.0f},  // 绿色
  {0.0f, 0.0f, 1.0f}   // 蓝色
};
}  //namespace

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
  (void)argument;

  led.start();

  while (true) {
    for (uint8_t c = 0; c < 3; c++) {
      // 渐亮：亮度 0 -> 1
      for (uint8_t s = 0; s < BREATH_STEPS; s++) {
        const float brightness = s * BRIGHTNESS_STEP;
        led.set(COLORS[c][0] * brightness, COLORS[c][1] * brightness, COLORS[c][2] * brightness);
        osDelay(BREATH_STEP_MS);
      }

      // 渐暗：亮度 1 -> 0
      for (uint8_t s = BREATH_STEPS; s > 0; s--) {
        const float brightness = s * BRIGHTNESS_STEP;
        led.set(COLORS[c][0] * brightness, COLORS[c][1] * brightness, COLORS[c][2] * brightness);
        osDelay(BREATH_STEP_MS);
      }
    }
  }
}
