#ifndef LASER_HPP
#define LASER_HPP
#include "cmsis_os.h"
#include "usart.h"

#define LASER_TX_BUFFER_SIZE 8  // 激光测距模块数据帧长度

class Laser
{
public:
  Laser(UART_HandleTypeDef * huart);
  uint16_t distance;  // 只读! 激光测距值, 单位: cm, 范围: [0, 65535], 65535表示无效值
  uint8_t mode;       //0x00表示长距离测量    0x01快速测量  0x02高精度测量    0x03一般测量
  uint8_t temp;
  void config();
  void request();
  void update(uint16_t size);

private:
  UART_HandleTypeDef * huart_;
  uint8_t tx_buffer_[LASER_TX_BUFFER_SIZE];
  uint16_t tx_buffer_size_ = LASER_TX_BUFFER_SIZE;
};

#endif  // LASER_HPP
