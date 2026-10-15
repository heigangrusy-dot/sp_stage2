#include "laser.hpp"

Laser::Laser(UART_HandleTypeDef * huart) : huart_(huart)
{
  distance = 65535;
  mode = 0x00;
  temp = 0;
}

void Laser::config()
{
  if (huart_->Init.BaudRate != 9600) {
    huart_->Init.BaudRate = 9600;
    HAL_UART_Init(huart_);
  }
  uint8_t config_command1[4] = {0xA5, 0x76, 0x01, 0x1C};  // 将模块波特率切换到115200
  HAL_UART_Transmit(huart_, config_command1, sizeof(config_command1), 1000);

  huart_->Init.BaudRate = 115200;
  HAL_UART_Init(huart_);
  uint8_t config_command2[4] = {0xA5, 0x65, 0x01, 0x0B};  // 将模块输出模式改为连续输出
  HAL_UART_Transmit(huart_, config_command2, sizeof(config_command2), 1000);
}

void Laser::request()
{
  HAL_UARTEx_ReceiveToIdle_IT(huart_, tx_buffer_, tx_buffer_size_);
  return;
}

void Laser::update()
{
  uint8_t checksum = tx_buffer_[0] + tx_buffer_[1] + tx_buffer_[2] + tx_buffer_[3] + tx_buffer_[4] +
                     tx_buffer_[5] + tx_buffer_[6] + tx_buffer_[7];  //校验位
  if (tx_buffer_[0] != 0x5A || tx_buffer_[1] != 0x5A || tx_buffer_[8] != checksum) {
    distance = 65535;
    return;
  }
  //数据解析
  distance = tx_buffer_[5] | ((uint16_t)(tx_buffer_[4]) << 8);  //距离值
  mode = tx_buffer_[6];                                         //模式
  temp = tx_buffer_[7];                                         //温度值
}