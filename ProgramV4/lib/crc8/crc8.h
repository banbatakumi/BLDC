#ifndef CRC8_H_
#define CRC8_H_

#include <stdint.h>

// CRC-8/AUTOSAR (AUTOSAR CRC ライブラリの Crc_CalculateCRC8H2F と同一)
//
//   poly = 0x2F, init = 0xFF, xorout = 0xFF, 入出力のビット反転なし
//   チェック値: "123456789" (0x31〜0x39 の9バイト) → 0xDF
//
// **なぜフッタ (固定値 0xFF) では駄目だったか**
//   固定値のフッタが検出できるのは「フレーム長がずれた」ことだけで、
//   ペイロードのビット化けは一切検出できない。しかもデータ部には 0xAA も 0xFF も
//   現れうるので、バイトが1つ落ちて同期がずれると、たまたま位置の合った
//   0xAA/0xFF を頭と尻だと誤認してデタラメな値をそのまま制御に入れてしまう。
//
// **なぜ 0x2F なのか (0x07 や 0x31 ではなく)**
//   0x2F は 8bit CRC としてハミング距離 6 を長いデータ長まで保つ多項式で、
//   AUTOSAR が車載通信向けに選定したもの。7バイトのフレームなら
//   「2ビットまでの化けは必ず検出、それ以上も 1/256 の見逃し率」に収まる。
//   多項式を変えると HD が落ちるので、上位側と揃えたまま触らないこと。
//
// **速度**
//   ビットごとに回す素朴な実装。10バイトで80ループ = 数百サイクルで、
//   1kHz でも 20kHz の制御ループから見れば無視できる。256バイトのテーブルを
//   置けば4倍ほど速いが、ROM を使ってまで削る場面ではない。
#define CRC8_POLY 0x2F
#define CRC8_INIT 0xFF
#define CRC8_XOROUT 0xFF

static inline uint8_t Crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = CRC8_INIT;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC8_POLY) : (uint8_t)(crc << 1);
    }
  }
  return crc ^ CRC8_XOROUT;
}

#endif  // CRC8_H_
