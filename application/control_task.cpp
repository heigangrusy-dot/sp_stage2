#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "laser.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

sp::RM_Motor motor1(1, sp::RM_Motors::GM6020, sp::M3508_P19);
sp::RM_Motor motor2(2, sp::RM_Motors::GM6020, sp::M3508_P19);
sp::RM_Motor motor3(3, sp::RM_Motors::M2006, sp::M3508_P19);

sp::PID motor1_pid_angle(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false, false);
sp::PID motor2_pid_angle(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false, false);
sp::PID motor3_pid_angle(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false, false);

sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::DBusSwitchMode dbus_sw_r = sp::DBusSwitchMode::DOWN;
extern sp::CAN can2;
extern Laser laser;
extern sp::DBusSwitchMode dbus_sw_r;

extern "C" void lose_power();

extern "C" void control_task(void * argument)
{
  can1.config();
  can1.start();

  can2.config();
  can2.start();

  while (1) {
    switch (dbus_sw_r) {
      case sp::DBusSwitchMode::DOWN:
        lose_power();
        break;
      case sp::DBusSwitchMode::MID:
        if (laser.distance < 1000) {
          motor1_pid_angle.clear();
          motor2_pid_angle.clear();
          motor3_pid_angle.clear();
        }
        motor1_pid_angle.calc(0.5f, motor1.angle);
        motor2_pid_angle.calc(0.5f, motor2.angle);
        motor3_pid_angle.calc(0.5f, motor3.angle);

        motor1.cmd(motor1_pid_angle.out);
        motor2.cmd(motor2_pid_angle.out);
        motor3.cmd(motor3_pid_angle.out);

        motor1.write(can1.tx_data);
        motor2.write(can1.tx_data);
        motor3.write(can2.tx_data);
        can1.send(motor1.tx_id);
        can2.send(motor3.tx_id);
        break;
      case sp::DBusSwitchMode::UP:
        motor1_pid_angle.clear();
        motor2_pid_angle.clear();
        motor3_pid_angle.clear();
        break;
    }

    osDelay(1);
  }
}

extern "C" void lose_power()  //失能函数
{
  motor1_pid_angle.clear();
  motor2_pid_angle.clear();
  motor3_pid_angle.clear();
  motor1.cmd(0.0f);
  motor2.cmd(0.0f);
  motor3.cmd(0.0f);
  motor1.write(can1.tx_data);
  motor2.write(can1.tx_data);
  motor3.write(can2.tx_data);
  can1.send(motor1.tx_id);
  can2.send(motor3.tx_id);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)  //CAN回调
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor1.rx_id)
        motor1.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor2.rx_id)
        motor2.read(can1.rx_data, stamp_ms);
    }
    if (hcan == &hcan2) {
      can2.recv();

      if (can2.rx_id == motor3.rx_id)
        motor3.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == 0x111) {
        uint8_t raw = can2.rx_data[0];
        dbus_sw_r = static_cast<sp::DBusSwitchMode>(raw);
      }
    }
  }
}