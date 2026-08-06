#ifndef SERIAL_PROTOCOL_H_
#define SERIAL_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#include "crc8.h"

// 上位の車両制御基板 (STM32F446RE) との通信フレーム。
//
//   250000 bps 8N1、多バイト値はすべてビッグエンディアン。
//   両方向ともフッタは持たず、末尾は CRC-8/AUTOSAR (crc8.h) のみ。
//   CRC の計算対象は「先頭の 0xAA を除き、CRC 自身の手前まで」。
//
// このヘッダにはフレームの並べ替えだけを置き、意味づけ (どのモードか、
// どのスケールか) は app.c 側に置いてある。**ホストPCでもそのままコンパイルできる**
// ようにするためで、テストベクタの検証を実機に上げずに回せる。
//
// --- 受信フレーム (上位 → MD): 6バイト固定長 ---
//   0    : 0xAA          ヘッダ
//   1    : mode          0xBA〜0xBF (app.c の SERIAL_COMMANDS 表)
//   2-3  : setpoint      i16 BE、スケールは mode ごと
//   4    : torque_limit  u8 × 0.001 → 0〜0.255 N・m
//   5    : crc8          byte1〜4 に対する CRC-8
//
// --- 送信フレーム (MD → 上位): 11バイト固定長 ---
//   0    : 0xAA          ヘッダ
//   1    : status        bit3:過電流 bit2:過熱 bit1:電源電圧異常 bit0:動作中
//   2    : temperature   u8  × 1 degC
//   3-4  : theta         u16 BE × 0.0001 rad
//   5-6  : speed         i16 BE × 0.01 rad/s
//   7-8  : iq            i16 BE × 0.001 A
//   9    : torque_limit  u8  × 0.001 N・m  ← **適用中**の値
//   10   : crc8          byte1〜9 に対する CRC-8
//
// **上位が指定する制限はトルクだけ。** 以前は speed_limit も送っていたが、
// 速度は上位のモーションコントローラが指令値そのもので決めるべきもので、
// MD 側にもう一段の速度飽和を置くと同じ量を2箇所で作ることになる。
// MD が持つべきなのは「モータとFETを焼かないための電流の天井」だけ。
// 速度の絶対上限 (MAX_ANGULAR_SPEED) は MD 固定のハード保護として残っており、
// これは上位から変えられない (変えられてはいけない) ので通信には載せない。
#define SERIAL_HEADER 0xAA

#define SERIAL_RX_FRAME_SIZE 6
#define SERIAL_TX_FRAME_SIZE 11

// トルク制限のスケール。**受信・エコーバック・上限との比較で必ずこれを使うこと。**
//
// **u8 のスケールは「最大トルク / 255 以上」でなければならない。**
// 上限指定でモータの全域に届く必要があるため。このモータは
//   Kt = 1.5 × POLE_PAIRS × ψm = 0.01947 N・m/A、最大 = Kt × MAX_CURRENT(10A) = 0.1947 N・m
// なので下限は 0.1947/255 = 0.00076 N・m/LSB。
//
//   0.01  : 0〜2.55 N・m  分解能 10 mN・m → 最大トルク内が **19段しかない**
//   0.001 : 0〜0.255 N・m 分解能  1 mN・m → 194段。天井はモータ最大の 1.31 倍で適切  ←これ
//   0.0001: 0〜0.0255 N・m → **最大トルクに届かない**
//
// **このスケールの天井 0.255 N・m が MAX_CURRENT 13.1A 相当。** MAX_CURRENT を
// それ以上に上げるなら、ここのスケールも見直さないと上位から上限を指定しきれない。
//
// 指令側 (0xBD/0xBF の i16、0.0001 N・m) を細かくしないのは別の理由。
// 電流センサの量子化が 0.0081 A/LSB = 0.158 mN・m なので、0.1 mN・m 刻みの指令は
// **既にハードが測れる分解能より細かい**。下げてもノイズの下に潜るだけ。
#define SERIAL_TORQUE_LIMIT_SCALE 0.001f  // [N・m / LSB]

// 受信フレームの中身 (生の整数のまま。物理量への換算は app.c 側で行う)
typedef struct {
  uint8_t mode;          // モードヘッダ 0xBA〜0xBF
  int16_t setpoint;      // スケールは mode ごと
  uint8_t torque_limit;  // × 0.001 [N・m]
} SerialRxFrame;

// 送信フレームの中身 (スケール済みの整数)
typedef struct {
  uint8_t status;
  uint8_t temperature;   // [degC]
  uint16_t theta;        // [0.1mrad]
  int16_t speed;         // [0.01rad/s]
  int16_t iq;            // [mA]
  uint8_t torque_limit;  // × 0.001 [N・m]  適用中の値
} SerialTxFrame;

// SERIAL_RX_FRAME_SIZE バイト受け取ってフレームとして解釈する。
// ヘッダかCRCが合わなければ false を返し、out には触れない。
// **モードヘッダの妥当性は見ない** (表を持っているのは app.c 側なので)。
static inline bool SerialProtocol_Decode(const uint8_t* buf, SerialRxFrame* out) {
  if (buf[0] != SERIAL_HEADER) return false;
  if (Crc8(&buf[1], SERIAL_RX_FRAME_SIZE - 2) != buf[SERIAL_RX_FRAME_SIZE - 1]) return false;

  out->mode = buf[1];
  out->setpoint = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
  out->torque_limit = buf[4];
  return true;
}

// 状態フレームを SERIAL_TX_FRAME_SIZE バイトに書き出す (buf はそれ以上であること)
static inline void SerialProtocol_Encode(const SerialTxFrame* in, uint8_t* buf) {
  buf[0] = SERIAL_HEADER;
  buf[1] = in->status;
  buf[2] = in->temperature;
  buf[3] = (uint8_t)(in->theta >> 8);
  buf[4] = (uint8_t)(in->theta & 0xFF);
  buf[5] = (uint8_t)((uint16_t)in->speed >> 8);
  buf[6] = (uint8_t)((uint16_t)in->speed & 0xFF);
  buf[7] = (uint8_t)((uint16_t)in->iq >> 8);
  buf[8] = (uint8_t)((uint16_t)in->iq & 0xFF);
  buf[9] = in->torque_limit;
  buf[10] = Crc8(&buf[1], SERIAL_TX_FRAME_SIZE - 2);
}

#endif  // SERIAL_PROTOCOL_H_
