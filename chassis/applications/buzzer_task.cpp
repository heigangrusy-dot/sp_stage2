#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

namespace
{
// 音名+频率（Hz）
constexpr uint16_t NOTE_C4 = 262;  //中音Do
constexpr uint16_t NOTE_D4 = 294;
constexpr uint16_t NOTE_E4 = 330;
constexpr uint16_t NOTE_F4 = 349;
constexpr uint16_t NOTE_G4 = 392;
constexpr uint16_t NOTE_A4 = 440;
constexpr uint16_t NOTE_B4 = 494;
constexpr uint16_t NOTE_C5 = 523;  //高音Do

constexpr float BUZZER_DUTY = 0.3f;

struct Note
{
  uint16_t hz;  //音高
  uint16_t ms;  //持续时间
};

//“上电成功”提示音
constexpr Note STARTUP_SONG[] = {
  {NOTE_C4, 150}, {NOTE_D4, 150}, {NOTE_E4, 150}, {NOTE_F4, 150}, {NOTE_G4, 150}, {NOTE_A4, 150},
  {NOTE_B4, 150}, {NOTE_C5, 300}, {NOTE_A4, 150}, {NOTE_F4, 150}, {NOTE_C4, 400},
};
constexpr uint32_t SONG_LENGTH = sizeof(STARTUP_SONG) / sizeof(STARTUP_SONG[0]);
}  // namespace

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task(void const * argument)
{
  (void)argument;

  osDelay(300);

  for (uint32_t i = 0; i < SONG_LENGTH; i++) {
    buzzer.set(STARTUP_SONG[i].hz, BUZZER_DUTY);
    buzzer.start();
    osDelay(STARTUP_SONG[i].ms);
    buzzer.stop();
    osDelay(30);
  }

  while (true) {
    osDelay(1000);
  }
}