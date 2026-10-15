#include "FreeRTOS.h"
#include "chassis_config.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"

namespace
{
sp::CAN chassis_can(&hcan1);

sp::RM_Motor motor_right_front(1, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO);
sp::RM_Motor motor_left_front(2, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO);
sp::RM_Motor motor_left_rear(3, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO);
sp::RM_Motor motor_right_rear(4, sp::RM_Motors::M3508, chassis_config::GEAR_RATIO);
}  // namespace

extern sp::Plotter plotter;

extern "C" void chassis_task(void const * argument)
{
  (void)argument;

  chassis_can.config();
  chassis_can.start();

  while (true) {
    float values[10] = {};

    taskENTER_CRITICAL();
    const auto now_ms = osKernelSysTick();
    values[0] = motor_left_front.speed;
    values[1] = motor_left_rear.speed;
    values[2] = motor_right_front.speed;
    values[3] = motor_right_rear.speed;
    values[4] = motor_left_front.is_alive(now_ms) ? 1.0f : 0.0f;
    values[5] = motor_left_rear.is_alive(now_ms) ? 1.0f : 0.0f;
    values[6] = motor_right_front.is_alive(now_ms) ? 1.0f : 0.0f;
    values[7] = motor_right_rear.is_alive(now_ms) ? 1.0f : 0.0f;

    taskEXIT_CRITICAL();
    values[8] = static_cast<float>(HAL_CAN_GetState(&hcan1));
    values[9] = static_cast<float>(HAL_CAN_GetError(&hcan1));

    plotter.plot(
      values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
      values[8], values[9]);
    osDelay(20);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) return;

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      break;
    }

    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) {
      continue;
    }

    const auto now_ms = osKernelSysTick();

    // 右前ID1，反馈ID0x201。
    if (header.StdId == motor_right_front.rx_id) {
      motor_right_front.read(data, now_ms);
    }
    // 左前ID2，反馈ID0x202。
    else if (header.StdId == motor_left_front.rx_id) {
      motor_left_front.read(data, now_ms);
    }
    // 左后ID3，反馈ID0x203。
    else if (header.StdId == motor_left_rear.rx_id) {
      motor_left_rear.read(data, now_ms);
    }
    // 右后ID4，反馈ID0x204。
    else if (header.StdId == motor_right_rear.rx_id) {
      motor_right_rear.read(data, now_ms);
    }
  }
}