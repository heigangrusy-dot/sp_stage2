#pragma once

// Host-only HAL substitute for exercising the real CAN transport implementation.
#include <cstddef>
#include <cstdint>
#include <cstring>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum HAL_CAN_StateTypeDef { HAL_CAN_STATE_RESET, HAL_CAN_STATE_LISTENING };
struct CAN_TypeDef { uint32_t ESR = 0; };
struct CAN_TxHeaderTypeDef { uint32_t RTR, DLC, IDE, StdId, ExtId; };
using CAN_RxHeaderTypeDef = CAN_TxHeaderTypeDef;
struct CAN_FilterTypeDef {
  uint32_t FilterActivation, FilterFIFOAssignment, FilterMode, FilterScale;
  uint32_t FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow;
  uint32_t FilterBank, SlaveStartFilterBank;
};
struct CAN_HandleTypeDef {
  CAN_TypeDef * Instance;
  HAL_CAN_StateTypeDef state = HAL_CAN_STATE_LISTENING;
  uint32_t pending = 0, error = 0, abort_calls = 0, last_abort_mask = 0;
  HAL_StatusTypeDef add_result = HAL_OK, abort_result = HAL_OK, rx_result = HAL_OK;
  CAN_TxHeaderTypeDef last_header = {};
  CAN_RxHeaderTypeDef rx_header = {0, 8, 0, 0x206, 0};
  uint8_t last_data[8] = {};
};
extern CAN_HandleTypeDef hcan1, hcan2;
extern uint32_t mock_tick;
constexpr uint32_t HAL_CAN_ERROR_NONE = 0, CAN_ESR_BOFF = 4;
constexpr uint32_t CAN_FILTER_ENABLE = 1, CAN_FILTER_FIFO0 = 0;
constexpr uint32_t CAN_RX_FIFO0 = 0, CAN_FILTERMODE_IDMASK = 0, CAN_FILTERSCALE_32BIT = 0;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING = 1, CAN_IT_RX_FIFO1_MSG_PENDING = 2;
constexpr uint32_t CAN_ID_STD = 0, CAN_ID_EXT = 4, CAN_RTR_DATA = 0;
inline uint32_t HAL_GetTick() { return mock_tick; }
inline HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, const CAN_FilterTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t) { return HAL_OK; }
inline HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef * h) { return h->state; }
inline uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef * h) { return h->error; }
inline uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef * h) {
  uint32_t free = 0;
  for (uint32_t mask = 1; mask <= 4; mask <<= 1) if (!(h->pending & mask)) ++free;
  return free;
}
inline uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef * h, uint32_t mask) {
  return (h->pending & mask) != 0;
}
inline HAL_StatusTypeDef HAL_CAN_AddTxMessage(
  CAN_HandleTypeDef * h, const CAN_TxHeaderTypeDef * header, const uint8_t * data, uint32_t * mailbox) {
  if (h->add_result != HAL_OK) return h->add_result;
  for (uint32_t mask = 1; mask <= 4; mask <<= 1) {
    if (h->pending & mask) continue;
    *mailbox = mask;
    h->pending |= mask;
    h->last_header = *header;
    std::memcpy(h->last_data, data, 8);
    return HAL_OK;
  }
  return HAL_ERROR;
}
inline HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef * h, uint32_t mask) {
  ++h->abort_calls;
  h->last_abort_mask = mask;
  if (h->abort_result == HAL_OK) h->pending &= ~mask;
  return h->abort_result;
}
inline HAL_StatusTypeDef HAL_CAN_GetRxMessage(
  CAN_HandleTypeDef * h, uint32_t, CAN_RxHeaderTypeDef * header, uint8_t * data) {
  if (h->rx_result != HAL_OK) return h->rx_result;
  *header = h->rx_header;
  std::memset(data, 0, 8);
  return HAL_OK;
}
