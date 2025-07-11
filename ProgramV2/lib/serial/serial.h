#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdbool.h>
#include <stdlib.h>

#include "usart.h"

typedef struct {
      UART_HandleTypeDef *huart;
      uint8_t *rxBuf;
      uint16_t rxTop;
      uint16_t rxBtm;
      uint16_t rxBufSize;
      bool useDMA;
} Serial;

// インスタンス生成
static inline void Serial_Init(Serial *self, UART_HandleTypeDef *huart, uint16_t rxBufSize, bool dma) {
      self->huart = huart;
      self->rxBuf = (uint8_t *)malloc(rxBufSize);
      self->rxTop = 0;
      self->rxBtm = 0;
      self->rxBufSize = rxBufSize;
      self->useDMA = dma;
      if (dma) {
            HAL_UART_Receive_DMA(huart, self->rxBuf, rxBufSize);
      }
}

// データ受信可否
static inline bool Serial_Available(Serial *self) {
      uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->CNDTR;
      return rxTop != self->rxBtm;
}

// 1バイト受信
static inline uint8_t Serial_Read(Serial *self) {
      uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->CNDTR;
      if (rxTop == self->rxBtm) {
            return 0;
      }
      uint8_t data = self->rxBuf[self->rxBtm];
      self->rxBtm = (self->rxBtm + 1) % self->rxBufSize;
      return data;
}

// 1バイト送信
static inline void Serial_WriteByte(Serial *self, uint8_t data) {
      HAL_UART_Transmit(self->huart, &data, 1, 100);
}

// 複数バイト送信
static inline void Serial_Write(Serial *self, const uint8_t *data, uint16_t len) {
      HAL_UART_Transmit(self->huart, (uint8_t *)data, len, 100);
}

#endif