#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"

// 同時に使う Serial インスタンス数の上限 (このMCUのUARTペリフェラル数と同じ)。
#define SERIAL_MAX_INSTANCES 2

typedef struct {
      UART_HandleTypeDef *huart;
      uint8_t *rxBuf;
      uint16_t rxBufSize;
      volatile uint32_t rxWritten;  // DMA割り込みで更新する総受信バイト数 (ラップしない)
      uint32_t rxRead;              // Serial_Read で読み出した総バイト数 (ラップしない)
} Serial;

// HAL_UART_RxHalfCpltCallback/RxCpltCallback から self を引くための登録表。
// この2つはHAL側でweak定義された「UARTごとに1つ」のグローバル関数なので、
// 複数の Serial インスタンス (huart違い) を区別するために自前で対応付ける。
static Serial *serial_registry[SERIAL_MAX_INSTANCES];

static inline void Serial_OnDmaProgress(UART_HandleTypeDef *huart) {
      for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
            Serial *self = serial_registry[i];
            if (self != NULL && self->huart == huart) {
                  // Half/Full 割り込みはそれぞれちょうどバッファ半分ずつ進んだ時点で発火する。
                  self->rxWritten += huart->RxXferSize / 2;
                  return;
            }
      }
}

// DMAの半分/満了割り込みは RecvSerial() の呼び出し頻度に関係なくハードウェアが
// 自発的に発火するので、過電流/過熱/電圧異常のフォルト処理でRecvSerial()が
// 長時間(バッファが何周もラップするほど)呼ばれなくても取りこぼさず追跡できる。
//
// HAL_UART_RxHalfCpltCallback/RxCpltCallback はweakなグローバル関数 (UARTごとではなく
// リンク全体で1つ) なので、この2つの実体は SERIAL_DEFINE_DMA_CALLBACKS を定義した
// **1つの.cファイルだけ**で有効にすること (Serial_Init を呼ぶ側で定義する想定)。
// serial.h を複数の.cファイルがインクルードしても多重定義にならないようにするための
// ガードで、serial_registry 自体は各.cファイルごとに static (内部リンケージ) のまま
// で構わない (Serial_Init を呼ばない側では空のまま使われない)。
#ifdef SERIAL_DEFINE_DMA_CALLBACKS
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) { Serial_OnDmaProgress(huart); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { Serial_OnDmaProgress(huart); }
#endif

// インスタンス生成
static inline void Serial_Init(Serial *self, UART_HandleTypeDef *huart, uint16_t rxBufSize) {
      self->huart = huart;
      self->rxBuf = (uint8_t *)malloc(rxBufSize);
      if (self->rxBuf == NULL) {
            printf("Serial_Init: rxBuf malloc failed (size=%u)\n", rxBufSize);
            self->rxBufSize = 0;
            self->rxWritten = 0;
            self->rxRead = 0;
            return;
      }
      memset(self->rxBuf, 0, rxBufSize);
      self->rxBufSize = rxBufSize;
      self->rxWritten = 0;
      self->rxRead = 0;

      for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
            if (serial_registry[i] == NULL) {
                  serial_registry[i] = self;
                  break;
            }
      }

      HAL_UART_Receive_DMA(huart, self->rxBuf, rxBufSize);
}

// データ受信可否
static inline bool Serial_Available(Serial *self) { return self->rxWritten != self->rxRead; }

// 1バイト受信
static inline uint8_t Serial_Read(Serial *self) {
      if (self->rxWritten == self->rxRead) {
            return 0;
      }

      // DMAの書き込みがバッファを1周以上追い越していたら、もう読み出せない古いデータは
      // 諦めて捨て、直近 (rxBufSize-1) バイトだけを対象に読み直す。
      if (self->rxWritten - self->rxRead > (uint32_t)(self->rxBufSize - 1)) {
            self->rxRead = self->rxWritten - (self->rxBufSize - 1);
      }

      uint16_t idx = (uint16_t)(self->rxRead % self->rxBufSize);
      uint8_t data = self->rxBuf[idx];
      self->rxRead++;
      return data;
}

// 1バイト送信
static inline void Serial_WriteByte(Serial *self, uint8_t data) {
      HAL_UART_Transmit_DMA(self->huart, &data, 1);
}

// 複数バイト送信
static inline void Serial_Write(Serial *self, const uint8_t *data, uint16_t len) {
      HAL_UART_Transmit_DMA(self->huart, (uint8_t *)data, len);
}

static inline void Serial_Reset(Serial *self) {
      HAL_UART_AbortReceive(self->huart);
      HAL_UART_DMAStop(self->huart);
      memset(self->rxBuf, 0, self->rxBufSize);
      self->rxWritten = 0;
      self->rxRead = 0;
      HAL_UART_Receive_DMA(self->huart, self->rxBuf, self->rxBufSize);
}

#endif
