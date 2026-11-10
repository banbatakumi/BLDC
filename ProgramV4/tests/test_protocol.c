// シリアルプロトコルのホスト側テスト。
// 実機のヘッダ (lib/crc8/crc8.h, src/app/serial_protocol.h) をそのまま include する。
//
//   make -C tests
#include <stdio.h>
#include <string.h>

#include "serial_protocol.h"

static int failures = 0;

static void Check(int ok, const char* name) {
  printf("%s %s\n", ok ? "  ok  " : "  FAIL", name);
  if (!ok) failures++;
}

static void Dump(const char* label, const uint8_t* b, int n) {
  printf("       %s:", label);
  for (int i = 0; i < n; i++) printf(" %02X", b[i]);
  printf("\n");
}

// ---------------------------------------------------------------------------
// 1. CRC のチェック値
// ---------------------------------------------------------------------------
static void TestCrcCheckValue(void) {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  uint8_t crc = Crc8(s, 9);
  printf("[1] CRC-8/AUTOSAR チェック値\n");
  printf("       Crc8(\"123456789\", 9) = 0x%02X (期待 0xDF)\n", crc);
  Check(crc == 0xDF, "チェック値が 0xDF");
}

// ---------------------------------------------------------------------------
// 2. テストベクタ (受信) — 位置指令 0.5rad / トルク上限 0.050N・m
// ---------------------------------------------------------------------------
static const uint8_t RX_VECTOR[SERIAL_RX_FRAME_SIZE] = {0xAA, 0xBB, 0x01, 0xF4, 0x32, 0x26};

static void TestRxVector(void) {
  printf("[2] 受信テストベクタ AA BB 01 F4 32 26\n");
  Dump("given", RX_VECTOR, SERIAL_RX_FRAME_SIZE);

  // 解析側
  SerialRxFrame f;
  int ok = SerialProtocol_Decode(RX_VECTOR, &f);
  Check(ok, "Decode が成功する");
  if (ok) {
    printf("       mode=0x%02X setpoint=%d (%.3f rad) torque_limit=%u (%.3f N・m)\n",
           f.mode, f.setpoint, f.setpoint * 0.001, f.torque_limit,
           f.torque_limit * (double)SERIAL_TORQUE_LIMIT_SCALE);
    Check(f.mode == 0xBB, "mode = 0xBB (位置指令)");
    Check(f.setpoint == 500, "setpoint = 500 (= 0.500 rad)");
    Check(f.torque_limit == 50, "torque_limit = 50 (= 0.050 N・m)");
  }

  // 生成側 (上位側と同じ組み立てをして、バイト列が一致するか)
  uint8_t buf[SERIAL_RX_FRAME_SIZE];
  buf[0] = SERIAL_HEADER;
  buf[1] = 0xBB;
  buf[2] = (uint8_t)((500 >> 8) & 0xFF);
  buf[3] = (uint8_t)(500 & 0xFF);
  buf[4] = 50;
  buf[5] = Crc8(&buf[1], SERIAL_RX_FRAME_SIZE - 2);
  Dump("built", buf, SERIAL_RX_FRAME_SIZE);
  Check(memcmp(buf, RX_VECTOR, SERIAL_RX_FRAME_SIZE) == 0, "生成側のバイト列が一致");
}

// ---------------------------------------------------------------------------
// 3. テストベクタ (送信)
// ---------------------------------------------------------------------------
static const uint8_t TX_VECTOR[SERIAL_TX_FRAME_SIZE] = {0xAA, 0x09, 0x2A, 0x27, 0x10, 0xFB,
                                                        0x2E, 0x09, 0xC4, 0x32, 0x7C};

static void TestTxVector(void) {
  printf("[3] 送信テストベクタ AA 09 2A 27 10 FB 2E 09 C4 32 7C\n");

  SerialTxFrame f;
  f.status = 0x09;                        // 過電流(bit3) + 動作中(bit0)
  f.temperature = 42;                     // 42 degC
  f.theta = (uint16_t)(1.0 * 10000);      // 1.0 rad
  f.speed = (int16_t)(-12.34 * 100);      // -12.34 rad/s
  f.iq = (int16_t)(2.5 * 1000);           // 2.5 A
  f.torque_limit = 50;                    // 0.050 N・m

  uint8_t buf[SERIAL_TX_FRAME_SIZE];
  SerialProtocol_Encode(&f, buf);
  Dump("built", buf, SERIAL_TX_FRAME_SIZE);
  Dump("given", TX_VECTOR, SERIAL_TX_FRAME_SIZE);
  Check(memcmp(buf, TX_VECTOR, SERIAL_TX_FRAME_SIZE) == 0, "生成側のバイト列が一致");

  // 解析側 (上位が受け取ったとして読み戻せるか)
  Check(TX_VECTOR[0] == SERIAL_HEADER, "ヘッダ 0xAA");
  Check(Crc8(&TX_VECTOR[1], SERIAL_TX_FRAME_SIZE - 2) == TX_VECTOR[SERIAL_TX_FRAME_SIZE - 1],
        "CRC が一致 (0x7C)");
  uint16_t theta = ((uint16_t)TX_VECTOR[3] << 8) | TX_VECTOR[4];
  int16_t speed = (int16_t)(((uint16_t)TX_VECTOR[5] << 8) | TX_VECTOR[6]);
  int16_t iq = (int16_t)(((uint16_t)TX_VECTOR[7] << 8) | TX_VECTOR[8]);
  printf("       status=0x%02X temp=%u theta=%.4frad speed=%.2frad/s iq=%.3fA "
         "limit=%.3f N・m\n",
         TX_VECTOR[1], TX_VECTOR[2], theta * 0.0001, speed * 0.01, iq * 0.001,
         TX_VECTOR[9] * (double)SERIAL_TORQUE_LIMIT_SCALE);
  Check(theta == 10000 && speed == -1234 && iq == 2500, "theta/speed/iq が解析できる");
  Check(TX_VECTOR[9] == 50, "トルク制限のエコーバックが解析できる");
}

// ---------------------------------------------------------------------------
// 4. 1ビット反転を必ず弾く (CRC バイトを含む byte1〜末尾すべて)
// ---------------------------------------------------------------------------
// 走査範囲はフレーム長から導く。フレームを縮めたときに「範囲だけ元のまま」に
// なると、増えも減りもしていないように見えて検査が手薄になる。
#define RX_LAST_BYTE (SERIAL_RX_FRAME_SIZE - 1)
#define RX_FLIP_COMBOS (RX_LAST_BYTE * 8)

static void TestSingleBitFlip(void) {
  printf("[4] 1ビット反転の検出 (byte1〜%d の %d 通り)\n", RX_LAST_BYTE, RX_FLIP_COMBOS);
  int accepted = 0;
  for (int byte = 1; byte <= RX_LAST_BYTE; byte++) {
    for (int bit = 0; bit < 8; bit++) {
      uint8_t buf[SERIAL_RX_FRAME_SIZE];
      memcpy(buf, RX_VECTOR, sizeof(buf));
      buf[byte] ^= (uint8_t)(1u << bit);
      SerialRxFrame f;
      if (SerialProtocol_Decode(buf, &f)) {
        printf("       byte%d bit%d の反転が受理された!\n", byte, bit);
        accepted++;
      }
    }
  }
  printf("       %d 通り中 受理されたのは %d 通り\n", RX_FLIP_COMBOS, accepted);
  Check(accepted == 0, "1ビット反転はすべて破棄される");

  // ついでに 2ビット反転も見ておく
  int accepted2 = 0, combos2 = 0;
  for (int b1 = 1; b1 <= RX_LAST_BYTE; b1++) {
    for (int i1 = 0; i1 < 8; i1++) {
      for (int b2 = b1; b2 <= RX_LAST_BYTE; b2++) {
        for (int i2 = (b2 == b1 ? i1 + 1 : 0); i2 < 8; i2++) {
          uint8_t buf[SERIAL_RX_FRAME_SIZE];
          memcpy(buf, RX_VECTOR, sizeof(buf));
          buf[b1] ^= (uint8_t)(1u << i1);
          buf[b2] ^= (uint8_t)(1u << i2);
          SerialRxFrame f;
          combos2++;
          if (SerialProtocol_Decode(buf, &f)) accepted2++;
        }
      }
    }
  }
  printf("       2ビット反転 %d 通り中 受理されたのは %d 通り\n", combos2, accepted2);
  Check(accepted2 == 0, "2ビット反転もすべて破棄される (HD=6 の裏取り)");
}

// ---------------------------------------------------------------------------
// 5. 受信の状態機械 (app.c と同じアルゴリズム)
//    データ部の 0xAA、1バイト欠落からの再同期を確認する
// ---------------------------------------------------------------------------
typedef struct {
  uint8_t buf[SERIAL_RX_FRAME_SIZE];
  uint8_t len;
  int accepted;
  int discarded;
} RxState;

// app.c の RecvSerial と同じ処理 (テストのため切り出したもの)
static void RxFeed(RxState* s, uint8_t byte, SerialRxFrame* out_last) {
  s->buf[s->len++] = byte;

  if (s->len == 1) {
    if (s->buf[0] != SERIAL_HEADER) s->len = 0;
    return;
  }
  if (s->len < SERIAL_RX_FRAME_SIZE) return;

  SerialRxFrame f;
  if (SerialProtocol_Decode(s->buf, &f)) {
    s->accepted++;
    *out_last = f;
    s->len = 0;
    return;
  }

  // 破棄。バッファ内の次の 0xAA を先頭に詰め直して再同期する。
  s->discarded++;
  uint8_t next = 1;
  while (next < SERIAL_RX_FRAME_SIZE && s->buf[next] != SERIAL_HEADER) next++;
  s->len = (uint8_t)(SERIAL_RX_FRAME_SIZE - next);
  memmove(s->buf, &s->buf[next], s->len);
}

static void BuildRx(uint8_t* buf, uint8_t mode, int16_t sp, uint8_t tl) {
  buf[0] = SERIAL_HEADER;
  buf[1] = mode;
  buf[2] = (uint8_t)(((uint16_t)sp >> 8) & 0xFF);
  buf[3] = (uint8_t)((uint16_t)sp & 0xFF);
  buf[4] = tl;
  buf[5] = Crc8(&buf[1], SERIAL_RX_FRAME_SIZE - 2);
}

static void TestStateMachine(void) {
  printf("[5] 受信の状態機械\n");

  // (a) データ部に 0xAA を含むフレーム
  //     setpoint = 0xAAAA (-21846), torque_limit = 0xAA
  {
    uint8_t frame[SERIAL_RX_FRAME_SIZE];
    BuildRx(frame, 0xBA, (int16_t)0xAAAA, 0xAA);
    Dump("0xAA だらけのフレーム", frame, SERIAL_RX_FRAME_SIZE);

    RxState s = {{0}, 0, 0, 0};
    SerialRxFrame last = {0, 0, 0};
    for (int i = 0; i < SERIAL_RX_FRAME_SIZE; i++) RxFeed(&s, frame[i], &last);
    Check(s.accepted == 1 && last.setpoint == (int16_t)0xAAAA && last.torque_limit == 0xAA,
          "データ部の 0xAA を含んでも正常にパースできる");
  }

  // (b) 1バイト欠落からの再同期
  {
    RxState s = {{0}, 0, 0, 0};
    SerialRxFrame last = {0, 0, 0};

    // 正常なフレームを 3 個 → 1 バイト欠けたフレーム → 正常なフレームを 5 個
    uint8_t frame[SERIAL_RX_FRAME_SIZE];
    for (int n = 0; n < 3; n++) {
      BuildRx(frame, 0xBB, (int16_t)(100 + n), 50);
      for (int i = 0; i < SERIAL_RX_FRAME_SIZE; i++) RxFeed(&s, frame[i], &last);
    }
    int before = s.accepted;
    Check(before == 3, "正常なフレーム 3 個を受理");

    BuildRx(frame, 0xBB, 999, 50);
    for (int i = 0; i < SERIAL_RX_FRAME_SIZE; i++) {
      if (i == 3) continue;  // 4 バイト目を欠落させる
      RxFeed(&s, frame[i], &last);
    }

    int recovered_at = -1;
    for (int n = 0; n < 5; n++) {
      BuildRx(frame, 0xBB, (int16_t)(200 + n), 50);
      for (int i = 0; i < SERIAL_RX_FRAME_SIZE; i++) RxFeed(&s, frame[i], &last);
      if (recovered_at < 0 && s.accepted > before) recovered_at = n;
    }
    printf("       欠落後 %d フレーム目で同期が復帰 (受理 %d, 破棄 %d)\n", recovered_at + 1,
           s.accepted, s.discarded);
    Check(recovered_at >= 0 && recovered_at <= 1, "後続フレームで同期が復帰する");
    Check(last.setpoint == 204, "復帰後は最新のフレームを読めている");
  }

  // (c) CRC の壊れたフレームは受理されず、破棄カウンタが増える
  {
    RxState s = {{0}, 0, 0, 0};
    SerialRxFrame last = {0, 0, 0};
    uint8_t frame[SERIAL_RX_FRAME_SIZE];
    BuildRx(frame, 0xBB, 500, 50);
    for (int i = 0; i < SERIAL_RX_FRAME_SIZE; i++) RxFeed(&s, frame[i], &last);
    BuildRx(frame, 0xBB, 500, 50);
    frame[4] ^= 0x01;  // torque_limit を 1 ビット化けさせる (CRC は直さない)
    for (int i = 0; i < SERIAL_RX_FRAME_SIZE; i++) RxFeed(&s, frame[i], &last);
    Check(s.accepted == 1 && s.discarded == 1, "化けたフレームは破棄されカウンタが増える");
    Check(last.torque_limit == 50, "破棄されたので前回値が保持されている");
  }
}

// ---------------------------------------------------------------------------
// 6. トルク制限のクランプとエコーバック (app.c の ApplyLimits と同じ計算)
// ---------------------------------------------------------------------------
static uint8_t ClampTorqueLimit(uint8_t rx_torque, float hw_torque_max) {
  float tb = hw_torque_max / SERIAL_TORQUE_LIMIT_SCALE;
  uint8_t hw_torque_byte = (tb >= 255.0f) ? 255 : (uint8_t)tb;
  return (rx_torque < hw_torque_byte) ? rx_torque : hw_torque_byte;
}

static void TestLimitClamp(void) {
  printf("[6] トルク制限のクランプとエコーバック\n");
  const float HW_TORQUE = 0.19467f;  // Kt(0.019467) × MAX_CURRENT(10.0)

  // 0.001 N・m/LSB なので上限は 194 LSB (切り捨て。上限を上回る値をエコーしない)
  const uint8_t HW_TORQUE_BYTE = 194;

  uint8_t t;

  t = ClampTorqueLimit(50, HW_TORQUE);
  printf("       受信 %.3f N・m → 適用 %.3f N・m\n", 50 * (double)SERIAL_TORQUE_LIMIT_SCALE,
         t * (double)SERIAL_TORQUE_LIMIT_SCALE);
  Check(t == 50, "上限以下のトルクはそのまま通る (0.01 スケールでは 15 に潰れていた)");

  t = ClampTorqueLimit(255, HW_TORQUE);
  printf("       受信 255 → 適用 %.3f N・m (MD の上限)\n",
         t * (double)SERIAL_TORQUE_LIMIT_SCALE);
  Check(t == HW_TORQUE_BYTE, "255 を送っても MD の上限で頭打ちになる");
  Check(t * SERIAL_TORQUE_LIMIT_SCALE <= HW_TORQUE,
        "エコーした値が MD の実際の上限を上回らない (切り捨て)");

  Check(ClampTorqueLimit(0, HW_TORQUE) == 0, "0 は 0 のまま (無制限と解釈しない)");

  // 分解能の確認: 0.001 スケールならモータの全域を 194 段で刻める
  printf("       モータ最大 %.4f N・m を %u 段で刻める (0.01 スケールなら %u 段)\n",
         (double)HW_TORQUE, HW_TORQUE_BYTE, (unsigned)(HW_TORQUE / 0.01f));
  Check(HW_TORQUE_BYTE >= 100, "最大トルク内を100段以上で指定できる");

  // 天井がモータ最大に届くこと (スケールを下げすぎると届かなくなる)
  Check(255 * SERIAL_TORQUE_LIMIT_SCALE >= HW_TORQUE,
        "u8 の天井 (255 LSB) がモータの最大トルクに届く");

  // バイト領域で比較しているので、上限以下は往復が完全一致する
  int mismatch = 0;
  for (int v = 0; v <= HW_TORQUE_BYTE; v++) {
    if (ClampTorqueLimit((uint8_t)v, HW_TORQUE) != v) mismatch++;
  }
  Check(mismatch == 0, "上限以下のトルクは 0〜194 すべてが 1LSB もずれずにエコーされる");
}

int main(void) {
  TestCrcCheckValue();
  TestRxVector();
  TestTxVector();
  TestSingleBitFlip();
  TestStateMachine();
  TestLimitClamp();
  printf("\n%s (失敗 %d 件)\n", failures ? "=== 失敗 ===" : "=== すべて成功 ===", failures);
  return failures != 0;
}
