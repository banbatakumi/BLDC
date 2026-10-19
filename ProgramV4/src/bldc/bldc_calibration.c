// このファイルだけサイズ優先で最適化する (ROM が残り約2KB で、PROFILE_ISR を有効にすると入りきらないため。
// -O2 比で約1.4KB 減)。校正は起動時に1回走るだけなので速度は要らない。20kHz 割り込みの処理
// (bldc.c / bldc_internal.h のインライン関数) は bldc.c 側でコンパイルされるので -O2 のまま。
// include より前に置くのは、calib_math.h の計算関数もサイズ優先にするため (ISR が使う
// CalibMath_LutCorrection は bldc.c の中で -O2 で展開されるので影響しない)。
// Makefile は CubeMX が再生成するので、ここに書く
#pragma GCC optimize("Os")

#include <math.h>    // sqrtf / atan2f (起動時の校正のみ。ISRからは呼ばない)
#include <stddef.h>  // offsetof

#include "bldc_internal.h"

// スイッチ校正 (エンコーダ/モータ定数の実測) とフラッシュの読み書き。手順と順序の理由は config.h。
// 測定はすべて制御ループ (20kHz/1kHz) の中で記録・平均する。メインループの HAL_Delay は
// 待ち時間にしか使わない (HAL_Delay(1) は実際には約2ms 待つので、周期の基準にはできない)。

#define CALIB_CURRENT (CALIB_CURRENT_RATIO * MAX_CURRENT)  // 校正に流す電流 [A]
#define ELEC_PER_REV (TWO_PI_F * POLE_PAIRS)               // 機械角1周ぶんの電気角 [rad]
#define ENC_SWEEP_S 2.0f        // エンコーダ校正で機械角1周にかける時間 [s]
#define J_LOG_SAMPLES 400       // J/Kt 測定の速度記録 (1kHz × 400 = 400ms)
#define J_TOP_SPEED (0.5f * MAX_ANGULAR_SPEED)  // J/Kt 測定で加速を止める速度 [rad/s]
// J/Kt 測定で許す最大の角加速度 [rad/s²]。オブザーバがロック中に追える上限
// (PLL_KI × PLL_SPEED_ERROR_LIMIT_RAD = 4050) の8割にして、加速度を低く読まないようにする。
// 減速は摩擦が加わるぶん加速より速いので、摩擦が大きく慣性が小さいと両立せず測れない
#define J_MAX_ACCEL (0.8f * PLL_KI * PLL_SPEED_ERROR_LIMIT_RAD)

// ISR と共有するデータの受け渡し。開始フラグ (volatile) より前の通常の書き込みや、完了フラグを見た後の
// 読み出しをコンパイラが入れ替えないよう、フラグの前後にメモリバリアを置く (配列ごと volatile にするより軽い)
#define CALIB_BARRIER() __DMB()

// 測定ごとに使い分ける作業領域 (同時には使わない)
static union {
  struct {
    float sum[2][ENCODER_LUT_SIZE];     // ビンごとの生値の和 (正転 / 逆転)
    uint16_t cnt[2][ENCODER_LUT_SIZE];  // ビンごとのサンプル数
  } enc;
  float step[BLDC_CAPTURE_SAMPLES];  // 同期加算したステップ応答 [A]
  float w[J_LOG_SAMPLES];            // J/Kt 測定の速度記録 [rad/s]
} work;

// ---------------------------------------------------------------------------
// ISR から呼ばれる部分 (svc.calib_hook が真の間だけ)
// ---------------------------------------------------------------------------
// 20kHz の平均 (Id, Iq, Vd, Vq, ω, エンコーダの偏差とその2乗)
enum { AVG_ID, AVG_IQ, AVG_VD, AVG_VQ, AVG_W, AVG_ENC, AVG_ENC2, AVG_COUNT };
static volatile uint32_t avg_left;
static float avg_sum[AVG_COUNT];
static float avg_enc0;  // エンコーダの基準値 (2乗和の桁落ちを防ぐため差で集計する)

// エンコーダのビン集計。1: 正転 (飽和も含めて集計、レールを探す) / 2: 正転 / 3: 逆転 (2,3 は飽和を除外して数える)
static volatile uint8_t enc_mode;
static uint32_t enc_total, enc_sat;

// J/Kt 測定の状態 (1kHz)。1: 加速 / 2: 減速 / 3: 完了 / 4: 記録が溢れた
static volatile uint8_t j_state;
static volatile uint16_t j_n, j_switch;
static float j_dir, j_current;

void BLDC_CalibIsr(void) {
  if (avg_left > 0) {
    float d = svc.encoder_raw - avg_enc0;
    avg_sum[AVG_ID] += svc.id;
    avg_sum[AVG_IQ] += svc.iq;
    avg_sum[AVG_VD] += svc.vd;
    avg_sum[AVG_VQ] += svc.vq;
    avg_sum[AVG_W] += svc.angular_speed;
    avg_sum[AVG_ENC] += d;
    avg_sum[AVG_ENC2] += d * d;
    avg_left--;
  }

  if (enc_mode != 0) {
    // 真の機械角 φ = 強制電気角 / P の、機械角1周内での位置からビンを決める
    float frac = svc.calib_theta * (1.0f / ELEC_PER_REV);  // 強制角は校正中ずっと正
    frac -= (float)(int32_t)frac;
    uint16_t bin = (uint16_t)(frac * (float)ENCODER_LUT_SIZE) & (ENCODER_LUT_SIZE - 1);
    uint8_t slot = (enc_mode == 3) ? 1 : 0;
    if (enc_mode != 1) {
      enc_total++;
      if (BLDC_IsEncoderSaturated(svc.encoder_raw)) {
        enc_sat++;
        return;
      }
    }
    work.enc.sum[slot][bin] += svc.encoder_raw;
    work.enc.cnt[slot][bin]++;
  }
}

void BLDC_CalibOuterIsr(void) {
  if (j_state != 1 && j_state != 2) return;
  float w = svc.angular_speed;
  if (j_n >= J_LOG_SAMPLES) {
    j_state = 4;
    svc.target_current = 0.0f;
    return;
  }
  work.w[j_n++] = w;
  if (j_state == 1 && j_dir * w >= J_TOP_SPEED) {
    j_state = 2;
    j_switch = j_n;
    svc.target_current = -j_dir * j_current;  // 同じ大きさで減速させる
  } else if (j_state == 2 && j_dir * w <= 0.0f) {
    j_state = 3;
    svc.target_current = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// 測定の部品 (メインループ)
// ---------------------------------------------------------------------------
// 制御ループで n サンプル (50µs ごと) 平均する。出力が切れて終わらなければ false
static bool Calib_Average(uint32_t n, float* out) {
  for (uint8_t i = 0; i < AVG_COUNT; i++) avg_sum[i] = 0.0f;
  avg_enc0 = svc.encoder_raw;
  CALIB_BARRIER();
  avg_left = n;
  uint32_t t0 = HAL_GetTick();
  while (avg_left > 0) {
    if (HAL_GetTick() - t0 > n / 20 + 200) {
      avg_left = 0;
      printf("  平均が終わらない (出力停止?)\n");
      return false;
    }
  }
  CALIB_BARRIER();
  for (uint8_t i = 0; i < AVG_COUNT; i++) out[i] = avg_sum[i] / (float)n;
  out[AVG_ENC] += avg_enc0;  // 平均値は基準を戻す (AVG_ENC2 は基準からの2乗平均のまま)
  return true;
}

// 強制電気角を target まで speed [rad/s (電気角)] で動かし、着くまで待つ
static bool Calib_MoveTo(float target, float speed) {
  float dist = Abs(target - svc.calib_theta);
  svc.calib_step = speed * CURRENT_LOOP_DT;
  svc.calib_target = target;
  uint32_t timeout = (uint32_t)(dist / speed * 1000.0f) + 500;
  uint32_t t0 = HAL_GetTick();
  while (svc.calib_theta != target) {
    if (svc.is_overcurrent || HAL_GetTick() - t0 > timeout) {
      printf("  強制転流が止まった\n");
      return false;
    }
  }
  return true;
}

// q軸指令0のトルクモードで出力を有効にし、整定を待つ
static void Calib_BeginTorqueMode(void) {
  svc.target_current = 0.0f;
  svc.mode = BLDC_MODE_TORQUE;
  svc.enable = true;
  HAL_Delay(50);
}

// 記録したIdの定常値 [A]。後ろ20%の平均
static float Calib_CaptureSteadyState(const float* i) {
  const uint16_t tail = BLDC_CAPTURE_SAMPLES / 5;
  float sum = 0.0f;
  for (uint16_t k = BLDC_CAPTURE_SAMPLES - tail; k < BLDC_CAPTURE_SAMPLES; k++) sum += i[k];
  return sum / (float)tail;
}

// ステップを1回打って記録が埋まるまで待ち、work.step に [A] で足し込む。成功したら true
static bool Calib_RunCapture(float step, bool is_voltage) {
  svc.capture_step = step;
  svc.capture_is_voltage = is_voltage;
  svc.capture_state = BLDC_CAPTURE_ARMED;

  // 記録は10msで終わるので、500msで打ち切る
  uint32_t t0 = HAL_GetTick();
  while (svc.capture_state != BLDC_CAPTURE_DONE && (HAL_GetTick() - t0) < 500) {
  }
  bool ok = (svc.capture_state == BLDC_CAPTURE_DONE);
  svc.capture_state = BLDC_CAPTURE_IDLE;
  if (!ok) {
    printf("  記録が完了しなかった (%u/%u)\n", svc.capture_index, (unsigned)BLDC_CAPTURE_SAMPLES);
    return false;
  }
  for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] += bldc_capture[k] * 0.001f;
  return true;
}

// 運動が収まるのを待つ (トルクは0にしておくこと)。3秒で止まらなければ false
static bool Calib_WaitForRest(void) {
  uint32_t t0 = HAL_GetTick();
  while (Abs(svc.angular_speed) > 1.0f) {
    if (HAL_GetTick() - t0 > 3000) {
      printf("  ロータが止まらない\n");
      return false;
    }
  }
  HAL_Delay(200);
  return true;
}

// ---------------------------------------------------------------------------
// 1. R と L (強制角0、PIをバイパスした電圧印加)
// ---------------------------------------------------------------------------
// ロータは強制角0 (d軸) に引き込まれるのでトルクは出ず、応答は素の1次系 (τ = L/R) になる。
//   R: 0.25〜1.0×V_cal の4点の定常電流から V = R·I + V0 を最小二乗 (V0 = デッドタイムの電圧誤差)
//   L: V_cal のステップを4回打って同期加算し、後半の対数の傾きから τ (calib_math.h)。L = τ·R
static bool Calib_MeasureRL(void) {
  printf("[CAL] R,L\n");
  svc.calib_theta = 0.0f;
  svc.calib_target = 0.0f;
  svc.calib_angle_active = true;
  svc.rl_aborted = false;
  svc.vd_override = 0.0f;
  svc.vd_override_active = true;
  Calib_BeginTorqueMode();

  // Id が校正電流に届く電圧を探す (ロータもここで d軸に引き込まれる)
  const float v_max = 0.5f * svc.supply_volt * MAX_MODULATION_RATIO;
  float v = 0.0f;
  while (svc.id < CALIB_CURRENT) {
    v += 0.002f;
    if (v > v_max || svc.rl_aborted || !svc.enable) {
      printf("  %.2fA に届かない (V=%.3fV)\n", (double)CALIB_CURRENT, (double)v);
      return false;
    }
    svc.vd_override = v;
    HAL_Delay(1);
  }
  const float v_cal = v;
  HAL_Delay(300);  // 引き込みの揺れが収まるのを待つ

  float vs[4], is[4], a[AVG_COUNT];
  for (uint8_t k = 0; k < 4; k++) {
    vs[k] = v_cal * (float)(k + 1) * 0.25f;
    svc.vd_override = vs[k];
    HAL_Delay(30);
    if (!Calib_Average(2000, a)) return false;
    is[k] = a[AVG_ID];
  }
  float r, v0;
  if (!CalibMath_LineFit(is, vs, 4, &r, &v0) || !(r > 0.001f && r < 100.0f)) {
    printf("  R が求まらない\n");
    return false;
  }

  for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] = 0.0f;
  for (uint8_t rep = 0; rep < 4; rep++) {
    svc.vd_override = 0.0f;
    HAL_Delay(20);
    if (!Calib_RunCapture(v_cal, true) || svc.rl_aborted) return false;
  }
  svc.vd_override = 0.0f;
  svc.vd_override_active = false;
  for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] *= 0.25f;

  float i_ss = Calib_CaptureSteadyState(work.step);
  float tau = CalibMath_FitTau(work.step, BLDC_CAPTURE_SAMPLES, i_ss, CURRENT_LPF_COEF) * CURRENT_LOOP_DT;
  float l = tau * r;
  printf("  V_cal %.3fV  R %.4fΩ (V0 %.3fV)  τ %.3fms  L %.1fµH\n", (double)v_cal, (double)r,
         (double)v0, (double)(tau * 1000.0f), (double)(l * 1e6f));
  if (!(l > 1e-6f && l < 0.1f)) {
    printf("  L が範囲外\n");
    return false;
  }
  if (tau < 2.0f * CURRENT_LOOP_DT) printf("  注意 τ が2サンプル未満で L の誤差が大きい\n");

  svc.motor_r = r;
  svc.motor_l = l;
  BLDC_UpdateCurrentGains();
  return true;
}

// ---------------------------------------------------------------------------
// 2. 電流ループの帯域合わせ (強制角0で Id ステップ)
// ---------------------------------------------------------------------------
// 演算遅れと電流LPFのぶん、実測帯域は設計帯域より低く出る。1回測って比で補正し、もう1回で確かめる。
// 帯域は 63%到達時間から (半周期の遅れを引く)。オーバーシュート 15% 超は上げすぎ。
static bool Calib_TuneCurrentBw(void) {
  printf("[CAL] 電流帯域 (狙い %uHz)\n", (unsigned)CURRENT_BW_TARGET_HZ);
  for (uint8_t it = 0; it < 2; it++) {
    for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] = 0.0f;
    svc.target_id_override = 0.0f;
    HAL_Delay(20);
    bool ok = Calib_RunCapture(CALIB_CURRENT, false);
    svc.target_id_override = 0.0f;
    if (!ok) return false;

    float i_ss = Calib_CaptureSteadyState(work.step);
    float thr = 0.632f * i_ss, k63 = 0.0f, peak = 0.0f;
    for (uint16_t k = 1; k < BLDC_CAPTURE_SAMPLES; k++) {
      if (work.step[k] > peak) peak = work.step[k];
      if (k63 == 0.0f && work.step[k] >= thr) {
        float d = work.step[k] - work.step[k - 1];
        k63 = (d > 1e-6f) ? ((float)(k - 1) + (thr - work.step[k - 1]) / d) : (float)k;
      }
    }
    float tau = (k63 - 0.5f) * CURRENT_LOOP_DT;
    if (!(i_ss > 0.5f * CALIB_CURRENT) || !(tau > 0.0f)) {
      printf("  応答が取れない (定常 %.3fA)\n", (double)i_ss);
      return false;
    }
    float f_bw = 1.0f / (TWO_PI_F * tau);
    float overshoot = (peak - i_ss) / i_ss * 100.0f;
    printf("  設計 %dHz → 実測 %dHz  オーバーシュート %.1f%%\n", (int)svc.current_bw_hz, (int)f_bw,
           (double)overshoot);
    if (it == 0) {
      svc.current_bw_hz = Constrain(svc.current_bw_hz * CURRENT_BW_TARGET_HZ / f_bw,
                                    0.8f * CURRENT_BW_TARGET_HZ, 2.0f * CURRENT_BW_TARGET_HZ);
      BLDC_UpdateCurrentGains();
    } else if (overshoot > 15.0f) {
      printf("  オーバーシュート過大\n");
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 3. エンコーダ (電流制御の強制転流)
// ---------------------------------------------------------------------------
// 強制角は真の電気角そのもので、φ = 強制角 / P が真の機械角になる。機械角1周を
// ENCODER_LUT_SIZE 個のビンに分け、ISR がビンごとに生値を平均する。
//   (a) 静止ノイズ σ   : レール判定の幅を 4σ + 2 LSB に決める
//   (b) 正転1周        : ビン平均の最小/最大 = 下/上のレール (単発の外れ値に左右されない)
//   (c) 正転1周+逆転1周: 飽和サンプルの割合 = 盲点幅 (強制角は等速なので時間の割合 = 角度の割合)。
//                        正転と逆転の平均で、摩擦による引き込み遅れとセンサの遅れが相殺される
// 線形写像した角度と φ の差が補正テーブル (平均がd軸オフセット)。回転方向は生値の増減から決める。
static void Calib_ClearBins(void) {
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    work.enc.sum[0][b] = work.enc.sum[1][b] = 0.0f;
    work.enc.cnt[0][b] = work.enc.cnt[1][b] = 0;
  }
}

static bool Calib_Encoder(void) {
  printf("[CAL] エンコーダ (正転2周・逆転1周)\n");
  const float speed = ELEC_PER_REV / ENC_SWEEP_S;
  const float lead = ELEC_PER_REV / 8.0f;  // 記録前の助走 (向きを変えた直後の遅れの切り替わりを避ける)

  // 強制角0のまま電流制御で保持する
  svc.target_id_override = CALIB_CURRENT;
  HAL_Delay(300);
  float a[AVG_COUNT];
  if (!Calib_Average(4000, a)) return false;
  float var = a[AVG_ENC2] - (a[AVG_ENC] - avg_enc0) * (a[AVG_ENC] - avg_enc0);
  float sigma = sqrtf(var > 0.0f ? var : 0.0f);

  // (b) レール
  Calib_ClearBins();
  if (!Calib_MoveTo(lead, speed)) return false;
  CALIB_BARRIER();
  enc_mode = 1;
  bool ok = Calib_MoveTo(lead + ELEC_PER_REV, speed);
  enc_mode = 0;
  CALIB_BARRIER();
  if (!ok) return false;
  float lo = 1e9f, hi = -1e9f;
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    if (work.enc.cnt[0][b] == 0) continue;
    float v = work.enc.sum[0][b] / (float)work.enc.cnt[0][b];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  if (!(hi - lo > MAX_ADC_VAL * 0.5f)) {
    printf("  レンジ不足 (%d〜%d)。ロータが回っていないか磁石が遠い\n", (int)lo, (int)hi);
    return false;
  }
  svc.min_encoder_val = (uint16_t)(lo + 0.5f);
  svc.max_encoder_val = (uint16_t)(hi + 0.5f);
  svc.encoder_margin = 4.0f * sigma + 2.0f;

  // (c) 正転・逆転
  Calib_ClearBins();
  enc_total = enc_sat = 0;
  const float a0 = lead + ELEC_PER_REV;
  CALIB_BARRIER();
  enc_mode = 2;
  ok = Calib_MoveTo(a0 + ELEC_PER_REV, speed);
  enc_mode = 0;
  CALIB_BARRIER();
  ok = ok && Calib_MoveTo(a0 + ELEC_PER_REV + lead, speed) &&
       Calib_MoveTo(a0 + ELEC_PER_REV, speed);
  if (!ok) return false;
  CALIB_BARRIER();
  enc_mode = 3;
  ok = Calib_MoveTo(a0, speed);
  enc_mode = 0;
  CALIB_BARRIER();
  svc.target_id_override = 0.0f;
  if (!ok) return false;

  svc.encoder_dead_zone = TWO_PI_F * (float)enc_sat / (float)(enc_total ? enc_total : 1);
  if (!(svc.encoder_dead_zone < TWO_PI_F * 0.25f)) {
    printf("  盲点が1/4周以上\n");
    return false;
  }

  // 有効なビン (正転・逆転とも、期待サンプル数の半分以上)
  const float expect = (float)enc_total / (2.0f * ENCODER_LUT_SIZE);
  bool valid[ENCODER_LUT_SIZE];
  uint16_t n_valid = 0;
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    valid[b] = (work.enc.cnt[0][b] >= 0.5f * expect) && (work.enc.cnt[1][b] >= 0.5f * expect);
    if (valid[b]) n_valid++;
  }

  // 回転方向: 隣り合う有効ビンの生値の増減 (継ぎ目の大きな跳びは除く)
  uint16_t inc = 0, dec = 0;
  for (uint16_t b = 0; b + 1 < ENCODER_LUT_SIZE; b++) {
    if (!valid[b] || !valid[b + 1]) continue;
    float d = work.enc.sum[0][b + 1] / work.enc.cnt[0][b + 1] -
              work.enc.sum[0][b] / work.enc.cnt[0][b];
    if (Abs(d) > 0.5f * (hi - lo)) continue;
    if (d > 0.0f) inc++;
    if (d < 0.0f) dec++;
  }
  if (inc + dec < ENCODER_LUT_SIZE / 2 || (inc > dec ? inc : dec) < 0.9f * (inc + dec)) {
    printf("  回転方向が決まらない (増 %u / 減 %u)。ロータが滑っている\n", inc, dec);
    return false;
  }
  svc.encoder_dir = (inc > dec) ? 1.0f : -1.0f;
  BLDC_UpdateEncoderScale();

  // ビンごとの θ_lin (正転と逆転の円平均) を sum[0] に、作業用を sum[1] に置く
  float max_lag = 0.0f;
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    if (!valid[b]) continue;
    float th_f = BLDC_EncoderLinear(work.enc.sum[0][b] / work.enc.cnt[0][b]);
    float th_b = BLDC_EncoderLinear(work.enc.sum[1][b] / work.enc.cnt[1][b]);
    float lag = FOC_AngleDiff(th_f, th_b);
    if (Abs(lag) > max_lag) max_lag = Abs(lag);
    work.enc.sum[0][b] = FOC_NormalizeRadians(th_b + 0.5f * lag);
  }
  float offset = 0.0f;
  float max_step = CalibMath_BuildLut(work.enc.sum[0], valid, ENCODER_LUT_SIZE, work.enc.sum[1],
                                      &offset, svc.encoder_lut);
  svc.encoder_offset_theta = offset;

  float lut_max = 0.0f;
  for (uint16_t k = 0; k < ENCODER_LUT_SIZE; k++) {
    float c = Abs((float)svc.encoder_lut[k] * CALIB_LUT_SCALE);
    if (c > lut_max) lut_max = c;
  }
  printf(
      "  min %u max %u σ %.2f margin %.1f dir %+d 盲点 %.2f%% offset %.4f\n"
      "  補正 最大 %.4frad 隣接差 %.4frad 正逆差 %.4frad 有効 %u/%u\n",
      svc.min_encoder_val, svc.max_encoder_val, (double)sigma, (double)svc.encoder_margin,
      (int)svc.encoder_dir, (double)(svc.encoder_dead_zone * 100.0f / TWO_PI_F), (double)offset,
      (double)lut_max, (double)max_step, (double)max_lag, n_valid, (unsigned)ENCODER_LUT_SIZE);

  // 隣接差が大きい = 補正量が途中で跳んでいる (ロータの滑り・POLE_PAIRS の不一致)。
  // 正逆差が大きい = 強制角にロータが付いてきていない (摩擦・負荷が大きい)
  if (!(max_step >= 0.0f && max_step < 0.1f) || max_lag > 0.1f ||
      n_valid < ENCODER_LUT_SIZE * 3 / 4) {
    printf("  校正不良 (滑り・POLE_PAIRS・負荷を確認)\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 4. J/Kt と摩擦 (トルクモードで加速→減速。ロータが回る)
// ---------------------------------------------------------------------------
// 同じ電流 I で加速 (+I) と減速 (−I) をさせ、同じ速度帯 0.2〜0.8×J_TOP_SPEED の平均加速度を比べる:
//   (J/Kt)·a_acc =  I − i_f(ω)
//   (J/Kt)·a_dec = −I − i_f(ω)    → J/Kt = 2I / (a_acc − a_dec),  i_f = −(J/Kt)(a_acc + a_dec)/2
// 摩擦 i_f は速度に依存してよい (同じ速度帯なので差で消える)。正逆両方向で測って平均する。
// 電流は加速度が J_MAX_ACCEL 以下・記録が溢れない範囲になるよう自動で決める。
typedef enum { J_OK, J_TOO_FAST, J_TOO_SLOW, J_FAILED } JResult;

static JResult Calib_InertiaRun(float dir, float current, float* jkt, float* fric) {
  Calib_BeginTorqueMode();
  if (!Calib_WaitForRest()) return J_FAILED;
  j_n = 0;
  j_switch = 0;
  j_dir = dir;
  j_current = current;
  svc.target_current = dir * current;
  CALIB_BARRIER();
  j_state = 1;
  uint32_t t0 = HAL_GetTick();
  while (j_state < 3 && HAL_GetTick() - t0 < 1000) {
  }
  uint8_t state = j_state;
  j_state = 0;
  CALIB_BARRIER();
  svc.target_current = 0.0f;
  if (svc.is_overcurrent) return J_FAILED;
  if (state != 3) return J_TOO_SLOW;

  float s_acc = 0.0f, s_dec = 0.0f;
  uint16_t n_acc = CalibMath_BandSlope(work.w, j_switch, dir, 0.2f * J_TOP_SPEED,
                                       0.8f * J_TOP_SPEED, &s_acc);
  uint16_t n_dec = CalibMath_BandSlope(&work.w[j_switch], j_n - j_switch, dir,
                                       0.2f * J_TOP_SPEED, 0.8f * J_TOP_SPEED, &s_dec);
  float a_acc = dir * s_acc / OUTER_LOOP_DT;  // 回転方向を正にした加速度 [rad/s²]
  float a_dec = dir * s_dec / OUTER_LOOP_DT;
  printf("  %+d方向 I=%.2fA: 加速 %d (%u点) 減速 %d (%u点) rad/s²\n", (int)dir, (double)current,
         (int)a_acc, n_acc, (int)a_dec, n_dec);
  if (n_acc < 10 || n_dec < 10 || a_acc > J_MAX_ACCEL || -a_dec > J_MAX_ACCEL) return J_TOO_FAST;
  if (!(a_acc > 0.0f && a_dec < 0.0f)) return J_TOO_SLOW;

  *jkt = 2.0f * current / (a_acc - a_dec);
  *fric = -*jkt * (a_acc + a_dec) * 0.5f;
  return J_OK;
}

static bool Calib_Inertia(void) {
  printf("[CAL] J/Kt と摩擦 (**ロータが回る**)\n");
  float current = 0.1f * MAX_CURRENT;
  float jkt[2], fric[2];
  for (uint8_t d = 0; d < 2; d++) {
    float dir = (d == 0) ? 1.0f : -1.0f;
    JResult r = J_FAILED;
    for (uint8_t attempt = 0; attempt < 6; attempt++) {
      r = Calib_InertiaRun(dir, current, &jkt[d], &fric[d]);
      if (r == J_OK || r == J_FAILED) break;
      current *= (r == J_TOO_FAST) ? 0.5f : 2.0f;
      if (current > 0.5f * MAX_CURRENT) current = 0.5f * MAX_CURRENT;
    }
    if (r != J_OK) {
      printf("  測定できない\n");
      return false;
    }
  }
  BLDC_Stop();

  float j_per_kt = 0.5f * (jkt[0] + jkt[1]);
  float friction = 0.5f * (fric[0] + fric[1]);
  printf("  J/Kt %.4e A·s²/rad (正 %.4e / 逆 %.4e)  摩擦 %.3fA\n", (double)j_per_kt,
         (double)jkt[0], (double)jkt[1], (double)friction);
  if (Abs(jkt[0] - jkt[1]) > 0.2f * j_per_kt) printf("  注意 正逆で20%%以上違う\n");
  if (!(j_per_kt > 1e-7f && j_per_kt < 1.0f)) {
    printf("  J/Kt が範囲外\n");
    return false;
  }
  svc.j_per_kt = j_per_kt;
  svc.friction_current = (friction > 0.0f) ? friction : 0.0f;
  BLDC_UpdateSpeedGains();
  return true;
}

// ---------------------------------------------------------------------------
// 5. ψm と角度の遅れ (速度制御で正逆8点。ロータが回る)
// ---------------------------------------------------------------------------
// 推定角が真の角度より δ 進んでいると、定常状態の電圧は
//   Vq − R·Iq − ω_e·L·Id = ω_e·ψm·cos δ + V0·sign(ω)    (V0: デッドタイムの電圧誤差)
//   Vd − R·Id + ω_e·L·Iq = ω_e·ψm·sin δ
// 1式目を最小二乗して ψm と V0、2つの比から各点の δ を出し、δ = δ0 + ω_e·Δt を当てはめる。
// Δt は遅れ補償の残り、δ0 は速度に依らない電気角のずれ (エンコーダのオフセット誤差) で、両方補正する。
#define PSI_POINTS 8

static bool Calib_SteadyPoint(float target, float* a) {
  BLDC_AngularSpeedControl(target);
  uint32_t t0 = HAL_GetTick();
  while (Abs(svc.speed_ramp - target) > 0.01f ||
         Abs(svc.angular_speed - target) > 0.05f * Abs(target) + 1.0f) {
    if (HAL_GetTick() - t0 > 4000 || !svc.enable) {
      printf("  %d rad/s に届かない (%.1f)\n", (int)target, (double)svc.angular_speed);
      return false;
    }
  }
  HAL_Delay(300);
  return Calib_Average(10000, a);  // 0.5s
}

static bool Calib_PsiDelay(void) {
  printf("[CAL] ψm と角度遅れ (**ロータが回る**)\n");
  const float v_limit = svc.supply_volt * MAX_MODULATION_RATIO;
  float a[AVG_COUNT];

  // 暫定の ψm から、電圧の天井に当たらない試験速度を決める
  if (!Calib_SteadyPoint(0.25f * MAX_ANGULAR_SPEED, a)) return false;
  float we = a[AVG_W] * POLE_PAIRS;
  float psi1 = (a[AVG_VQ] - svc.motor_r * a[AVG_IQ] - we * svc.motor_l * a[AVG_ID]) / we;
  if (!(psi1 > 1e-5f)) {
    printf("  ψm が正にならない (%.6f)\n", (double)psi1);
    return false;
  }
  svc.motor_psi = psi1;  // 掃引中の逆起電力FFに使う
  float w_test = 0.7f * v_limit / (psi1 * POLE_PAIRS);
  if (w_test > 0.9f * MAX_ANGULAR_SPEED) w_test = 0.9f * MAX_ANGULAR_SPEED;

  float x[PSI_POINTS], s[PSI_POINTS], y[PSI_POINTS], z[PSI_POINTS];
  for (uint8_t k = 0; k < PSI_POINTS; k++) {
    s[k] = (k < PSI_POINTS / 2) ? 1.0f : -1.0f;
    float target = s[k] * w_test * (float)(k % (PSI_POINTS / 2) + 1) / (float)(PSI_POINTS / 2);
    if (!Calib_SteadyPoint(target, a)) return false;
    x[k] = a[AVG_W] * POLE_PAIRS;
    y[k] = a[AVG_VQ] - svc.motor_r * a[AVG_IQ] - x[k] * svc.motor_l * a[AVG_ID];
    z[k] = a[AVG_VD] - svc.motor_r * a[AVG_ID] + x[k] * svc.motor_l * a[AVG_IQ];
  }
  BLDC_AngularSpeedControl(0.0f);
  HAL_Delay(1500);
  BLDC_Stop();

  float psi_c, v0;
  if (!CalibMath_FitPsi(x, s, y, PSI_POINTS, &psi_c, &v0) || !(psi_c > 1e-5f)) {
    printf("  ψm が求まらない\n");
    return false;
  }
  // 各点の δ と、その当てはめ。ψm は cos δ を掛け戻して求め直す
  float delta[PSI_POINTS], num = 0.0f, den = 0.0f;
  for (uint8_t k = 0; k < PSI_POINTS; k++) {
    delta[k] = atan2f(z[k] / x[k], (y[k] - v0 * s[k]) / x[k]);
    float sin_d, cos_d;  // libm の cosf は引数縮約で約3KB 増えるので表引きを使う
    FOC_SinCos(delta[k], &sin_d, &cos_d);
    float c = x[k] * cos_d;
    num += c * (y[k] - v0 * s[k]);
    den += c * c;
  }
  float dt_resid, delta0;
  if (!(den > 1e-9f) || !CalibMath_LineFit(x, delta, PSI_POINTS, &dt_resid, &delta0)) return false;
  float psi = num / den;
  float delay = svc.angle_delay - dt_resid;
  float rms = 0.0f;
  for (uint8_t k = 0; k < PSI_POINTS; k++) {
    float r = delta[k] - (delta0 + dt_resid * x[k]);
    rms += r * r;
  }
  rms = sqrtf(rms / PSI_POINTS);

  printf("  試験速度 %d rad/s  ψm %.6fWb  V0 %.3fV\n", (int)w_test, (double)psi, (double)v0);
  printf("  遅れ %.3f→%.3fms  静的ずれ δ0 %+.1f°(電気角)  当てはめ残差 %.1f°\n",
         (double)(svc.angle_delay * 1000.0f), (double)(delay * 1000.0f),
         (double)(delta0 * 57.29578f), (double)(rms * 57.29578f));
  if (!(psi > 1e-5f && psi < 1.0f) || !(delay >= 0.0f && delay < 0.005f) ||
      !(Abs(delta0) < 0.5f)) {
    printf("  範囲外\n");
    return false;
  }
  svc.motor_psi = psi;
  svc.angle_delay = delay;
  // 推定電気角を δ0 だけ戻す = 機械角のオフセットを δ0/P 増やす
  svc.encoder_offset_theta = FOC_NormalizeRadians(svc.encoder_offset_theta + delta0 / POLE_PAIRS);
  return true;
}

// ---------------------------------------------------------------------------
// 全体
// ---------------------------------------------------------------------------
bool BLDC_Calibrate(void) {
  printf("[CAL] スイッチ校正を開始 (校正電流 %.1fA)\n", (double)CALIB_CURRENT);
  svc.iq_limit = MAX_CURRENT;  // Kt が未定なので BLDC_SetLimits を介さず開ける (BLDC_Init の最後で閉じる)
  svc.calib_hook = true;

  // 1〜3 は強制角
  bool ok = Calib_MeasureRL() && Calib_TuneCurrentBw() && Calib_Encoder();
  svc.vd_override_active = false;
  svc.target_id_override = 0.0f;
  BLDC_Stop();
  svc.calib_angle_active = false;

  if (ok) {
    // 4,5 は校正したエンコーダで閉ループ
    svc.encoder_calibrated = true;
    svc.encoder_primed = false;  // 新しい校正値で張り直す (次の ISR)
    HAL_Delay(50);
    ok = Calib_Inertia() && Calib_PsiDelay();
    BLDC_Stop();
  }
  svc.calib_hook = false;

  svc.motor_calibrated = ok;
  if (!ok) {
    svc.encoder_calibrated = false;
    return false;
  }
  svc.encoder_primed = false;  // δ0 でオフセットが変わったので張り直す
  printf("[CAL] 結果: R %.4fΩ L %.1fµH ψm %.6fWb 遅れ %.3fms J/Kt %.4e 摩擦 %.3fA 帯域 %dHz\n",
         (double)svc.motor_r, (double)(svc.motor_l * 1e6f), (double)svc.motor_psi,
         (double)(svc.angle_delay * 1000.0f), (double)svc.j_per_kt,
         (double)svc.friction_current, (int)svc.current_bw_hz);
  printf("[CAL] J = %.4e kg·m² (= J/Kt × Kt)\n", (double)(svc.j_per_kt * BLDC_GetTorqueConstant()));
  return true;
}

// ---------------------------------------------------------------------------
// ゲイン
// ---------------------------------------------------------------------------
// 電流PI: Kp = L·ω_bw, Ki = R·ω_bw (config.h)。未校正 (0) なら0になり電圧は出ない
void BLDC_UpdateCurrentGains(void) {
  float w = TWO_PI_F * svc.current_bw_hz;
  svc.id_pi.kp = svc.motor_l * w;
  svc.id_pi.ki = svc.motor_r * w;
  svc.id_pi.output_limit = svc.supply_volt * MAX_MODULATION_RATIO;
  FOC_PI_Reset(&svc.id_pi);
  svc.iq_pi = svc.id_pi;
  if (svc.motor_l > 0.0f) {
    printf("[CAL] 電流PI: Kp=%.4f Ki=%.1f (設計 %dHz)\n", (double)svc.id_pi.kp,
           (double)svc.id_pi.ki, (int)svc.current_bw_hz);
  }
}

// 速度PID: Kp = (J/Kt)·ω_c, Ki = Kp·ω_c / SPEED_KI_RATIO (config.h)。未校正なら0
void BLDC_UpdateSpeedGains(void) {
  float wc = TWO_PI_F * SPEED_BW_HZ;
  svc.speed_pid.kp = svc.j_per_kt * wc;
  svc.speed_pid.ki = svc.speed_pid.kp * wc / SPEED_KI_RATIO;
  svc.speed_pid.integral = 0;
  if (svc.j_per_kt > 0.0f) {
    printf("[CAL] 速度PID: Kp=%.5f Ki=%.4f (設計 %uHz)\n", (double)svc.speed_pid.kp,
           (double)svc.speed_pid.ki, (unsigned)SPEED_BW_HZ);
  }
}

// ---------------------------------------------------------------------------
// フラッシュ
// ---------------------------------------------------------------------------
static uint32_t Calib_Checksum(const BLDCFlashData* d) {
  const uint32_t* p = (const uint32_t*)d;
  uint32_t sum = 0x5A5A5A5AUL;
  for (size_t i = 0; i < offsetof(BLDCFlashData, checksum) / 4; i++) sum += p[i];
  return sum;
}

_Static_assert(offsetof(BLDCFlashData, checksum) % 4 == 0, "checksum must be word aligned");
_Static_assert(sizeof(BLDCFlashData) <= FLASH_USER_END_ADDR + 1 - FLASH_USER_START_ADDR,
               "calibration data does not fit in the flash page");

void BLDC_ResetCalibration(void) {
  svc.encoder_calibrated = false;
  svc.motor_calibrated = false;
  svc.min_encoder_val = 0;
  svc.max_encoder_val = MAX_ADC_VAL;
  svc.encoder_margin = 4.0f;
  svc.encoder_dir = 1.0f;
  svc.encoder_offset_theta = 0.0f;
  svc.encoder_dead_zone = 0.0f;
  for (uint16_t k = 0; k < ENCODER_LUT_SIZE; k++) svc.encoder_lut[k] = 0;
  svc.motor_r = 0.0f;
  svc.motor_l = 0.0f;
  svc.motor_psi = 0.0f;
  svc.angle_delay = ANGLE_DELAY_DEFAULT;
  svc.j_per_kt = 0.0f;
  svc.friction_current = 0.0f;
  svc.current_bw_hz = CURRENT_BW_TARGET_HZ;
}

void BLDC_SaveCalibration(void) {
  static BLDCFlashData d;  // 300B 近いのでスタックに置かない
  memset(&d, 0, sizeof(d));
  d.magic = BLDC_FLASH_MAGIC;
  d.max_encoder_val = svc.max_encoder_val;
  d.min_encoder_val = svc.min_encoder_val;
  d.encoder_margin = (uint16_t)(svc.encoder_margin + 0.5f);
  d.encoder_dir = (int16_t)svc.encoder_dir;
  d.encoder_offset_theta = svc.encoder_offset_theta;
  d.encoder_dead_zone = svc.encoder_dead_zone;
  d.motor_r = svc.motor_r;
  d.motor_l = svc.motor_l;
  d.motor_psi = svc.motor_psi;
  d.angle_delay = svc.angle_delay;
  d.j_per_kt = svc.j_per_kt;
  d.friction_current = svc.friction_current;
  d.current_bw_hz = svc.current_bw_hz;
  memcpy(d.encoder_lut, svc.encoder_lut, sizeof(d.encoder_lut));
  d.checksum = Calib_Checksum(&d);
  printf("[CAL] フラッシュ書き込み%s\n",
         Flash_WriteData(FLASH_USER_START_ADDR, &d, sizeof(d)) == HAL_OK ? "完了" : "に失敗");
}

// マジックナンバー・チェックサム・範囲で検証する (NaN も落とせるよう不等号は肯定形で書く)
void BLDC_LoadFlashCalibration(void) {
  static BLDCFlashData d;
  Flash_ReadData(FLASH_USER_START_ADDR, &d, sizeof(d));

  bool ok = (d.magic == BLDC_FLASH_MAGIC) && (d.checksum == Calib_Checksum(&d)) &&
            (d.max_encoder_val > d.min_encoder_val) && (d.max_encoder_val <= MAX_ADC_VAL) &&
            (d.encoder_dir == 1 || d.encoder_dir == -1) && (d.encoder_margin < 100) &&
            (d.encoder_offset_theta >= 0.0f && d.encoder_offset_theta < TWO_PI_F) &&
            (d.encoder_dead_zone >= 0.0f && d.encoder_dead_zone < TWO_PI_F * 0.25f) &&
            (d.motor_r > 0.001f && d.motor_r < 100.0f) && (d.motor_l > 1e-6f && d.motor_l < 0.1f) &&
            (d.motor_psi > 1e-5f && d.motor_psi < 1.0f) &&
            (d.angle_delay >= 0.0f && d.angle_delay < 0.005f) &&
            (d.j_per_kt > 1e-7f && d.j_per_kt < 1.0f) &&
            (d.friction_current >= 0.0f && d.friction_current < MAX_CURRENT) &&
            (d.current_bw_hz >= 0.8f * CURRENT_BW_TARGET_HZ &&
             d.current_bw_hz <= 2.0f * CURRENT_BW_TARGET_HZ);
  if (!ok) {
    printf("BLDC: 校正値が無いか不正。スイッチを押しながら起動して校正すること (駆動しない)\n");
    return;
  }
  svc.max_encoder_val = d.max_encoder_val;
  svc.min_encoder_val = d.min_encoder_val;
  svc.encoder_margin = (float)d.encoder_margin;
  svc.encoder_dir = (float)d.encoder_dir;
  svc.encoder_offset_theta = d.encoder_offset_theta;
  svc.encoder_dead_zone = d.encoder_dead_zone;
  memcpy(svc.encoder_lut, d.encoder_lut, sizeof(svc.encoder_lut));
  svc.motor_r = d.motor_r;
  svc.motor_l = d.motor_l;
  svc.motor_psi = d.motor_psi;
  svc.angle_delay = d.angle_delay;
  svc.j_per_kt = d.j_per_kt;
  svc.friction_current = d.friction_current;
  svc.current_bw_hz = d.current_bw_hz;
  svc.encoder_calibrated = true;
  svc.motor_calibrated = true;
  printf("(Flash) enc %u-%u dir %+d offset %.4f 盲点 %.4f / R %.4f L %.1fµH ψm %.6f 遅れ %.3fms"
         " J/Kt %.3e 摩擦 %.3fA\n",
         d.min_encoder_val, d.max_encoder_val, d.encoder_dir, (double)d.encoder_offset_theta,
         (double)d.encoder_dead_zone, (double)d.motor_r, (double)(d.motor_l * 1e6f),
         (double)d.motor_psi, (double)(d.angle_delay * 1000.0f), (double)d.j_per_kt,
         (double)d.friction_current);
}
