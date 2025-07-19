#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdbool.h>
#include <stdlib.h>

#include "usart.h"

static uint16_t debug_top, debug_btm;  // デバッグ用の受信バッファインデックス

typedef struct {
      UART_HandleTypeDef *huart;
      uint8_t *rxBuf;
      uint16_t rxTop;
      uint16_t rxBtm;
      uint16_t rxBufSize;
} Serial;

// インスタンス生成
static inline void Serial_Init(Serial *self, UART_HandleTypeDef *huart, uint16_t rxBufSize) {
      self->huart = huart;
      self->rxBuf = (uint8_t *)malloc(rxBufSize);
      memset(self->rxBuf, 0, rxBufSize);  // バッファを0でクリア
      self->rxTop = 0;
      self->rxBtm = 0;
      self->rxBufSize = rxBufSize;
      HAL_UART_Receive_DMA(huart, self->rxBuf, rxBufSize);
}

// データ受信可否
static inline bool Serial_Available(Serial *self) {
      uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->CNDTR;
      debug_top = rxTop;
      debug_btm = self->rxBtm;
      return rxTop != self->rxBtm;
}

// 1バイト受信
static inline uint8_t Serial_Read(Serial *self) {
      uint16_t rxTop = self->rxBufSize - self->huart->hdmarx->Instance->CNDTR;
      if (rxTop == self->rxBtm) {
            return 0;
      }
      // オーバーフロー対策: バッファが追い越された場合は古いデータを捨てる
      if (((rxTop + self->rxBufSize - self->rxBtm) % self->rxBufSize) == 0) {
            // すべて読み切った状態
            return 0;
      }
      // 受信可能データ数
      uint16_t available = (rxTop + self->rxBufSize - self->rxBtm) % self->rxBufSize;
      if (available > self->rxBufSize - 1) {
            // オーバーフロー: rxBtmを最新位置に合わせる
            self->rxBtm = (rxTop + self->rxBufSize - 1) % self->rxBufSize;
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

static inline void Serial_Reset(Serial *self) {
      HAL_UART_AbortReceive(self->huart);                               // UART受信を完全に停止
      HAL_UART_DMAStop(self->huart);                                    // DMA停止
      memset(self->rxBuf, 0, self->rxBufSize);                          // バッファクリア（必要なら）
      HAL_UART_Receive_DMA(self->huart, self->rxBuf, self->rxBufSize);  // DMA再開
      self->rxBtm = 0;
}

#endif