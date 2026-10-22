#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "motor_limit.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task(void * argument)
{
  MotorLimitState last_state = MotorLimitState::NORMAL;
  uint32_t phase_start = osKernelSysTick();
  uint8_t startup_beeps = 0;
  bool sounding = false;
  buzzer.stop();

  while (true) {
    const uint32_t now = osKernelSysTick();
    const MotorLimitState state = motor_limit_state;
    if (state != last_state) {
      phase_start = now;
      last_state = state;
      startup_beeps = 3;  // 保护报警优先，结束后不再重播开机提示
    }

    bool should_sound = false;
    if (state != MotorLimitState::NORMAL) {
      const uint32_t interval = state == MotorLimitState::OUT_OF_RANGE ? 2000U : 1000U;
      should_sound = (now - phase_start) % interval < 100U;  // 每次短响100ms
    }
    else if (startup_beeps < 3) {
      const uint32_t elapsed = now - phase_start;
      should_sound = elapsed % 200U < 50U;
      if (elapsed >= 600U) {
        startup_beeps = 3;
        should_sound = false;
      }
    }

    if (should_sound && !sounding) {
      buzzer.set(state == MotorLimitState::NORMAL ? 262.0f : 440.0f, 0.005f);
      buzzer.start();
    }
    else if (!should_sound && sounding) {
      buzzer.stop();
    }
    sounding = should_sound;
    osDelay(10);
  }
}
