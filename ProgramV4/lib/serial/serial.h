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
      volatile uint32_t rxWrapBase;  // DMAがバッファを周回した累計バイト数 (満了割り込みで rxBufSize ずつ進む)
      uint32_t rxLastWritten;        // 前回 Serial_Written が返した値 (折り返し直後の補正用)
      uint32_t rxRead;               // Serial_Read で読み出した総バイト数 (ラップしない)
} Serial;

// HAL_UART_RxCpltCallback から self を引くための登録表。
// このコールバックはHAL側でweak定義された「UARTごとに1つ」のグローバル関数なので、
// 複数の Serial インスタンス (huart違い) を区別するために自前で対応付ける。
static Serial *serial_registry[SERIAL_MAX_INSTANCES];

static inline void Serial_OnDmaWrap(UART_HandleTypeDef *huart) {
      for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
            Serial *self = serial_registry[i];
            if (self != NULL && self->huart == huart) {
                  self->rxWrapBase += self->rxBufSize;
                  return;
            }
      }
}

// 周回数は満了割り込みで数え、周回内の位置は DMA の残り転送数 (CNDTR) から毎回読む。
//
// 以前は半分/満了割り込みだけで受信済みバイト数を進めていたため、256バイトのバッファでは
// **128バイト溜まるまで受信データが1バイトも見えなかった**。上位は 1ms ごとに 6バイト送って
// くるので、指令が約21msぶん溜まってからまとめて届き、平均約11ms・最大約21ms古い指令で
// 制御していた。
//
// 周回数を割り込みで数えるのは、過電流/過熱/電圧異常のフォルト処理で RecvSerial() が
// 長時間 (バッファが何周もラップするほど) 呼ばれなくても、何周遅れたかを見失わないため。
//
// HAL_UART_RxCpltCallback はweakなグローバル関数 (UARTごとではなくリンク全体で1つ) なので、
// 実体は SERIAL_DEFINE_DMA_CALLBACKS を定義した**1つの.cファイルだけ**で有効にすること
// (Serial_Init を呼ぶ側で定義する想定)。serial_registry 自体は各.cファイルごとに static
// (内部リンケージ) のままで構わない (Serial_Init を呼ばない側では空のまま使われない)。
#ifdef SERIAL_DEFINE_DMA_CALLBACKS
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { Serial_OnDmaWrap(huart); }
#endif

// インスタンス生成
static inline void Serial_Init(Serial *self, UART_HandleTypeDef *huart, uint16_t rxBufSize) {
      self->huart = huart;
      self->rxBuf = (uint8_t *)malloc(rxBufSize);
      if (self->rxBuf == NULL) {
            printf("Serial_Init: rxBuf malloc failed (size=%u)\n", rxBufSize);
            self->rxBufSize = 0;
            self->rxWrapBase = 0;
            self->rxLastWritten = 0;
            self->rxRead = 0;
            return;
      }
      memset(self->rxBuf, 0, rxBufSize);
      self->rxBufSize = rxBufSize;
      self->rxWrapBase = 0;
      self->rxLastWritten = 0;
      self->rxRead = 0;

      for (uint8_t i = 0; i < SERIAL_MAX_INSTANCES; i++) {
            if (serial_registry[i] == NULL) {
                  serial_registry[i] = self;
                  break;
            }
      }

      HAL_UART_Receive_DMA(huart, self->rxBuf, rxBufSize);
}

// DMAが書き込んだ総バイト数 (ラップしない)。
static inline uint32_t Serial_Written(Serial *self) {
      if (self->rxBufSize == 0) return 0;

      // 読んでいる最中に満了割り込みが入ると周回数と位置の組が食い違うので、読み直す
      uint32_t base;
      uint32_t remaining;
      do {
            base = self->rxWrapBase;
            remaining = __HAL_DMA_GET_COUNTER(self->huart->hdmarx);
      } while (base != self->rxWrapBase);

      // 循環モードの CNDTR は 0 になると即座に rxBufSize へ再ロードされるが、
      // その瞬間に 0 と読める可能性に備えて丸める
      uint32_t pos = self->rxBufSize - remaining;
      if (pos >= self->rxBufSize) pos = 0;
      uint32_t written = base + pos;

      // DMAが末尾から先頭へ折り返してから満了割り込みが実行されるまでの数サイクルの間は、
      // 位置だけが先頭へ戻り周回数が古いままになる。受信済みバイト数は減らないので、
      // 前回値を下回ったらこの状態とみなして1周ぶん補う (割り込みが入れば同じ値になる)
      if (written < self->rxLastWritten) written += self->rxBufSize;
      self->rxLastWritten = written;
      return written;
}

// データ受信可否
static inline bool Serial_Available(Serial *self) { return Serial_Written(self) != self->rxRead; }

// 1バイト受信
static inline uint8_t Serial_Read(Serial *self) {
      uint32_t written = Serial_Written(self);
      if (written == self->rxRead) {
            return 0;
      }

      // DMAの書き込みがバッファを1周以上追い越していたら、もう読み出せない古いデータは
      // 諦めて捨て、直近 (rxBufSize-1) バイトだけを対象に読み直す。
      if (written - self->rxRead > (uint32_t)(self->rxBufSize - 1)) {
            self->rxRead = written - (self->rxBufSize - 1);
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

// 複数バイト送信 (DMA)。
//
// TX の DMA は Normal モードで使うこと。Circular だと最初の1回の後は DMA が同じバッファを
// 延々と再送し続け (gState も BUSY_TX のまま戻らず、以降の送信要求は全て HAL_BUSY で捨てられる)、
// 再送中のバッファを呼び出し側が書き換えるので、送信途中で中身が入れ替わったフレームが
// CRC不一致で上位に捨てられていた。
//
// Normal モードでは DMA 完了後に UART の TC 割り込みで gState が READY に戻るが、この基板は
// USART の NVIC 割り込みを有効にしていないため戻らない。そこで毎回 Abort で READY に戻してから
// 送る。Abort は送信レジスタ/シフトレジスタに入っているバイトは破棄しないが、DMA が未転送の
// 分は捨てるので、**前のフレームを送り終える間隔で呼ぶこと**。data の指す領域は送信完了まで
// 書き換えないこと (DMA が直接読むため)。
static inline void Serial_Write(Serial *self, const uint8_t *data, uint16_t len) {
      HAL_UART_AbortTransmit(self->huart);
      HAL_UART_Transmit_DMA(self->huart, (uint8_t *)data, len);
}

static inline void Serial_Reset(Serial *self) {
      HAL_UART_AbortReceive(self->huart);
      HAL_UART_DMAStop(self->huart);
      memset(self->rxBuf, 0, self->rxBufSize);
      self->rxWrapBase = 0;
      self->rxLastWritten = 0;
      self->rxRead = 0;
      HAL_UART_Receive_DMA(self->huart, self->rxBuf, self->rxBufSize);
}

#endif
