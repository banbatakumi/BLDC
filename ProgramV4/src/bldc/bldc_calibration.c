// このファイルだけサイズ優先で最適化する。校正は起動時に1回走るだけなので速度は要らない。
// 20kHz 割り込みの処理 (bldc.c / bldc_internal.h のインライン関数) は bldc.c 側でコンパイルされる
// ので -O2 のまま。include より前に置くのは、calib_math.h の計算関数もサイズ優先にするため
// (ISR が使う CalibMath_LutCorrection は bldc.c の中で -O2 で展開されるので影響しない)。
// Makefile は CubeMX が再生成するので、ここに書く
#pragma GCC optimize("Os")

#include <math.h>    // sqrtf / atan2f (起動時の校正のみ。ISRからは呼ばない)
#include <stddef.h>  // offsetof

#include "bldc_internal.h"
#include "fixfmt.h"

// スイッチ校正 (エンコーダ/モータ定数の実測) とフラッシュの読み書き。手順と順序の理由は config.h。
// 測定はすべて制御ループ (20kHz/1kHz) の中で記録・平均する。メインループの HAL_Delay は
// 待ち時間にしか使わない (HAL_Delay(1) は実際には約2ms 待つので、周期の基準にはできない)。
// 全体で約40秒 (1分以内に収める。延ばしても結果が変わらなかった測定は削った)。

#define CALIB_CURRENT (CALIB_CURRENT_RATIO * MAX_CURRENT)  // 校正に流す電流 [A]
#define ELEC_PER_REV (TWO_PI_F * POLE_PAIRS)               // 機械角1周ぶんの電気角 [rad]
#define ENC_SWEEP_S 2.0f        // エンコーダ校正で機械角1周にかける時間 [s]
#define ENC_HIST 64             // レール探しのヒストグラムの幅 [LSB]
// 補正テーブルの惰性回転。正転・逆転を交互に。記録は格子線1本あたり 2 バイトで、
// 4回 × 約2周 × 128本 が入る (RAM の都合でこれ以上増やさない)
#define COAST_RUNS 4
#define COAST_BUF 1024
#define COAST_END (0.12f * MAX_ANGULAR_SPEED)  // 記録を打ち切る速度 [rad/s]
#define COAST_REVS 2.0f                        // 1回の惰性で回したい周数 (開始速度を摩擦から決める)
#define RL_ANGLES 6             // R,L を測る電気角の数 (60°刻み。θ と θ+180° で相電流が反転する)
#define RL_AVG 2000             // R の各点の平均サンプル数 (0.1s。24点あるので十分)
#define RL_TAU_PER_ANGLE 2      // τ のステップを各角度で何回打つか
#define BW_CAPTURES 4           // 電流帯域の各回で同期加算するステップ数
// ψm・遅れの掃引点数 (正逆それぞれ半分) と各点の平均サンプル数 (0.5s)。
// 8点×0.5秒で ψm が2回の校正で一致し当てはめ残差も 0.3° だったので、これ以上延ばさない
#define PSI_POINTS 8
#define PSI_AVG 10000
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
    float sum[2][ENCODER_LUT_SIZE];      // ビンごとの生値の和 (正転 / 逆転)。結果の置き場も兼ねる
    uint16_t cnt[2][ENCODER_LUT_SIZE];   // ビンごとのサンプル数
    float raw_i1[ENCODER_LUT_SIZE];      // ビンごとの生値 (正逆・全周の平均)
    uint16_t hist[2][ENC_HIST];          // レール付近のヒストグラム (下側 / 上側)
  } enc;
  struct {
    uint16_t buf[COAST_BUF];             // 格子線を横切った時刻 (CalibMath_CoastSample)
    float sum[2 * ENCODER_LUT_SIZE];     // 当てはめの作業用 (正転 / 逆転ごとの残差の和)
    uint8_t cnt[2 * ENCODER_LUT_SIZE];
    float e[ENCODER_LUT_SIZE];           // 補正量 [rad]
  } coast;
  float step[BLDC_CAPTURE_SAMPLES];  // 同期加算したステップ応答 [A]
  float w[J_LOG_SAMPLES];            // 速度の記録 [rad/s] (J/Kt 測定・確認試験)
} work;

// ---------------------------------------------------------------------------
// ISR から呼ばれる部分 (svc.calib_hook が真の間だけ)
// ---------------------------------------------------------------------------
// 20kHz の平均 (Id, Iq, Vd, Vq, ω, エンコーダの偏差とその2乗)
enum { AVG_ID, AVG_IQ, AVG_VD, AVG_VQ, AVG_W, AVG_ENC, AVG_ENC2, AVG_COUNT };
static volatile uint32_t avg_left;
static float avg_sum[AVG_COUNT];
static float avg_enc0;  // エンコーダの基準値 (2乗和の桁落ちを防ぐため差で集計する)

// エンコーダの集計。1: 極値 / 2: レール付近のヒストグラム / 3: 正転のビン / 4: 逆転のビン
// (3,4 は飽和を除外して、飽和の割合 = 盲点幅も数える) / 5: 惰性回転で格子線を横切った時刻
enum { ENC_OFF, ENC_EXTREME, ENC_HISTOGRAM, ENC_FORWARD, ENC_BACKWARD, ENC_COAST };
static volatile uint8_t enc_mode;
static uint32_t enc_total, enc_sat;
static float enc_lo, enc_hi;  // 1周目の極値 [LSB]
static CoastRec coast_rec;

// 1kHz の速度記録。J/Kt 測定 (1: 加速 / 2: 減速 / 3: 完了 / 4: 記録が溢れた) と、
// 確認試験の追従誤差の記録 (log_on。2ms ごと)
static volatile uint8_t j_state;
static volatile uint16_t j_n, j_switch;
static float j_dir, j_current;
static volatile bool log_on;
static uint8_t log_div;

void BLDC_CalibIsr(void) {
  float raw = svc.encoder_raw;
  if (avg_left > 0) {
    float d = raw - avg_enc0;
    avg_sum[AVG_ID] += svc.id;
    avg_sum[AVG_IQ] += svc.iq;
    avg_sum[AVG_VD] += svc.vd;
    avg_sum[AVG_VQ] += svc.vq;
    avg_sum[AVG_W] += svc.angular_speed;
    avg_sum[AVG_ENC] += d;
    avg_sum[AVG_ENC2] += d * d;
    avg_left--;
  }

  switch (enc_mode) {
    case ENC_EXTREME:
      if (raw < enc_lo) enc_lo = raw;
      if (raw > enc_hi) enc_hi = raw;
      break;
    case ENC_HISTOGRAM: {
      float d_lo = raw - enc_lo, d_hi = enc_hi - raw;
      if (d_lo >= 0.0f && d_lo < (float)ENC_HIST) work.enc.hist[0][(uint16_t)d_lo]++;
      if (d_hi >= 0.0f && d_hi < (float)ENC_HIST) work.enc.hist[1][(uint16_t)d_hi]++;
      break;
    }
    case ENC_FORWARD:
    case ENC_BACKWARD: {
      // 真の機械角 φ = 強制電気角 / P の、機械角1周内での位置からビンを決める
      float frac = svc.calib_theta * (1.0f / ELEC_PER_REV);  // 強制角は校正中ずっと正
      frac -= (float)(int32_t)frac;
      uint16_t bin = (uint16_t)(frac * (float)ENCODER_LUT_SIZE) & (ENCODER_LUT_SIZE - 1);
      uint8_t slot = (enc_mode == ENC_BACKWARD) ? 1 : 0;
      if (BLDC_IsEncoderSaturated(raw)) {
        enc_total++;
        enc_sat++;
        break;
      }
      // 飽和を抜けた直後は出力がレール間を移る途中の値なので使わない (運転中も同じ区間を棄却する)。
      // 盲点幅の母数にも入れない (時間で決まる区間で、角度の割合としては無視できる)
      if (svc.since_saturated <= ENCODER_SAT_HOLDOFF_SAMPLES) break;
      enc_total++;
      work.enc.sum[slot][bin] += raw;
      work.enc.cnt[slot][bin]++;
      break;
    }
    case ENC_COAST: {
      // 運転中 (BLDC_UpdateEncoder) と同じく、飽和とその直後は使わない
      bool ok = !BLDC_IsEncoderSaturated(raw) && svc.since_saturated > ENCODER_SAT_HOLDOFF_SAMPLES;
      float th = BLDC_EncoderLinear(raw) * ((float)ENCODER_LUT_SIZE / TWO_PI_F);
      if (th >= (float)ENCODER_LUT_SIZE) th -= (float)ENCODER_LUT_SIZE;
      CalibMath_CoastSample(&coast_rec, th, ok, ENCODER_LUT_SIZE);
      break;
    }
    default:
      break;
  }
}

void BLDC_CalibOuterIsr(void) {
  // 母線電圧の追従。校正中はメインループ (app の GetSensors) が止まっていて電圧が更新されないので、
  // ADC から直接読む (時定数 約50ms)。電圧指令→デューティの換算に使うので R と ψm に直接効く
  if (svc.supply_raw != NULL) {
    float v = (float)*svc.supply_raw * (ADC2VOLT * SUPPLY_VOLT_DIVIDER);
    float vs = svc.supply_volt + 0.02f * (v - svc.supply_volt);
    if (vs < 1.0f) vs = 1.0f;
    svc.supply_volt = vs;
    svc.id_pi.output_limit = vs * MAX_MODULATION_RATIO;
    svc.iq_pi.output_limit = svc.id_pi.output_limit;
  }

  // 確認試験: 速度ランプへの追従誤差を 2ms ごとに記録する
  if (log_on) {
    log_div ^= 1;
    if (log_div) {
      work.w[j_n++] = svc.speed_ramp - svc.angular_speed;
      if (j_n >= J_LOG_SAMPLES) log_on = false;
    }
  }

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

// エンコーダを集計しながら強制角を動かす
static bool Calib_Record(uint8_t mode, float target, float speed) {
  CALIB_BARRIER();
  enc_mode = mode;
  bool ok = Calib_MoveTo(target, speed);
  enc_mode = ENC_OFF;
  CALIB_BARRIER();
  return ok;
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

static void Calib_ClearStep(void) {
  for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] = 0.0f;
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

// 速度制御で target に到達して整定するまで待つ
static bool Calib_ReachSpeed(float target) {
  BLDC_AngularSpeedControl(target);
  uint32_t t0 = HAL_GetTick();
  while (Abs(svc.speed_ramp - target) > 0.01f ||
         Abs(svc.angular_speed - target) > 0.05f * Abs(target) + 1.0f) {
    if (HAL_GetTick() - t0 > 4000 || !svc.enable) {
      printf("  %d rad/s に届かない (%s)\n", (int)target, FixFmt(svc.angular_speed, 1));
      return false;
    }
  }
  HAL_Delay(300);
  return true;
}

// ---------------------------------------------------------------------------
// 1. R と L (強制角、PIをバイパスした電圧印加)
// ---------------------------------------------------------------------------
// ロータは強制角 (d軸) に引き込まれるのでトルクは出ず、応答は素の1次系 (τ = L/R) になる。
// 電気角 0〜300° の6か所で測る。θ と θ+180° では各相の電流が反転するので、電流センサの
// オフセットと相ごとのデッドタイムの偏りが平均で打ち消される (シャントは2相だけ)。
//   R: 各角度で 0.25〜1.0×V_cal の4点、計24点の定常電流から V = R·I + V0 を最小二乗
//      (V0 = デッドタイムの電圧誤差)。角度ごとの R のばらつきも表示する
//   L: 各角度で V_cal のステップを2回、計12回を同期加算し、電流LPFを逆算して対数の傾きから τ。L = τ·R
static bool Calib_MeasureRL(void) {
  printf("[CAL] R,L (%u角度)\n", (unsigned)RL_ANGLES);
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
      printf("  %sA に届かない (V=%sV)\n", FixFmt(CALIB_CURRENT, 2), FixFmt(v, 3));
      return false;
    }
    svc.vd_override = v;
    HAL_Delay(1);
  }
  const float v_cal = v;

  float vs[RL_ANGLES * 4], is[RL_ANGLES * 4], a[AVG_COUNT];
  float r_ang[RL_ANGLES];
  float r_min = 1e9f, r_max = 0.0f;
  Calib_ClearStep();
  for (uint8_t ang = 0; ang < RL_ANGLES; ang++) {
    // V_cal をかけたまま次の角度へ回し (ロータが付いてくる)、揺れが収まるのを待つ
    svc.vd_override = v_cal;
    if (!Calib_MoveTo((float)ang * (TWO_PI_F / RL_ANGLES), ELEC_PER_REV / ENC_SWEEP_S)) return false;
    HAL_Delay(150);

    for (uint8_t k = 0; k < 4; k++) {
      uint8_t i = (uint8_t)(ang * 4 + k);
      vs[i] = v_cal * (float)(k + 1) * 0.25f;
      svc.vd_override = vs[i];
      HAL_Delay(30);
      if (!Calib_Average(RL_AVG, a)) return false;
      is[i] = a[AVG_ID];
    }
    r_ang[ang] = 0.0f;
    if (CalibMath_LineFit(&is[ang * 4], &vs[ang * 4], 4, &r_ang[ang], 0)) {
      if (r_ang[ang] < r_min) r_min = r_ang[ang];
      if (r_ang[ang] > r_max) r_max = r_ang[ang];
    }

    for (uint8_t rep = 0; rep < RL_TAU_PER_ANGLE; rep++) {
      svc.vd_override = 0.0f;
      HAL_Delay(20);
      if (!Calib_RunCapture(v_cal, true) || svc.rl_aborted) return false;
    }
  }
  svc.vd_override = 0.0f;
  svc.vd_override_active = false;

  float r, v0;
  if (!CalibMath_LineFit(is, vs, RL_ANGLES * 4, &r, &v0) || !(r > 0.001f && r < 100.0f)) {
    printf("  R が求まらない\n");
    return false;
  }
  const float n_cap = (float)(RL_ANGLES * RL_TAU_PER_ANGLE);
  for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] /= n_cap;
  float i_ss = Calib_CaptureSteadyState(work.step);
  float tau = CalibMath_FitTau(work.step, BLDC_CAPTURE_SAMPLES, i_ss, CURRENT_LPF_COEF) *
              CURRENT_LOOP_DT;
  float l = tau * r;
  printf("  V_cal %sV  R %sΩ (V0 %sV、角度ごと %s〜%s)  τ %sms  L %sµH\n", FixFmt(v_cal, 3),
         FixFmt(r, 4), FixFmt(v0, 3), FixFmt(r_min, 4), FixFmt(r_max, 4), FixFmt(tau * 1000.0f, 3),
         FixFmt(l * 1e6f, 1));
  // 角度ごとの R。180° 周期 (0° と 180°、60° と 240° … が同じ) なら2相の電流センサの倍率差、
  // 360° 周期ならオフセットや相ごとの抵抗差を疑う (オフセットは θ と θ+180° で打ち消し合う)
  printf("  角度ごとの R [Ω]:");
  for (uint8_t ang = 0; ang < RL_ANGLES; ang++) {
    printf(" %u°:%s", (unsigned)(ang * 360 / RL_ANGLES), FixFmt(r_ang[ang], 4));
  }
  printf("\n");
  if (r_max - r_min > 0.2f * r) printf("  注意 角度による R の差が 20%% 超\n");
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
// 2. 電流ループの帯域合わせ (強制角で Id ステップ)
// ---------------------------------------------------------------------------
// 演算遅れと電流LPFのぶん、実測帯域は設計帯域より低く出る。1回測って比で補正し、もう1回で確かめる。
// 各回 BW_CAPTURES 回のステップを同期加算する。帯域は 63%到達時間から (半周期の遅れを引く)。
// オーバーシュート 15% 超は上げすぎ。
static bool Calib_TuneCurrentBw(void) {
  printf("[CAL] 電流帯域 (狙い %uHz)\n", (unsigned)CURRENT_BW_TARGET_HZ);
  for (uint8_t it = 0; it < 2; it++) {
    Calib_ClearStep();
    for (uint8_t rep = 0; rep < BW_CAPTURES; rep++) {
      svc.target_id_override = 0.0f;
      HAL_Delay(20);
      bool ok = Calib_RunCapture(CALIB_CURRENT, false);
      svc.target_id_override = 0.0f;
      if (!ok) return false;
    }
    for (uint16_t k = 0; k < BLDC_CAPTURE_SAMPLES; k++) work.step[k] /= (float)BW_CAPTURES;

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
      printf("  応答が取れない (定常 %sA)\n", FixFmt(i_ss, 3));
      return false;
    }
    float f_bw = 1.0f / (TWO_PI_F * tau);
    float overshoot = (peak - i_ss) / i_ss * 100.0f;
    printf("  設計 %dHz → 実測 %dHz  オーバーシュート %s%%\n", (int)svc.current_bw_hz, (int)f_bw,
           FixFmt(overshoot, 1));
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
//   (b) レール          : 1周目で極値、2周目でその近くのヒストグラムを取り、最頻値をレールにする
//   (c) 正逆の往復      : 校正電流で正転1周+逆転1周。
//                         正逆の平均で摩擦による引き込み遅れとセンサの遅れが相殺される。
//                         飽和サンプルの割合が盲点幅 (強制角は等速なので時間の割合 = 角度の割合)
// 線形写像した角度と φ の差の平均が d軸オフセット。補正テーブル (センサの非線形) はここでは作らず、
// 後で惰性回転で測る (Calib_EncoderLut)。強制転流ではロータの位置ずれと区別できず、
// 2026-10-19 の実機ではテーブルに 14回/周の偽の誤差が入って速度が揺れた。
static void Calib_ClearBins(void) {
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    work.enc.sum[0][b] = work.enc.sum[1][b] = 0.0f;
    work.enc.cnt[0][b] = work.enc.cnt[1][b] = 0;
  }
}

// 正転1周と逆転1周を記録し、ビンごとに正逆平均した生値を out[] に書く (out は sum[0] でもよい)。
// 正逆とも期待数の 95% 以上サンプルが無いビン (盲点と、継ぎ目にかかって一部が飽和で抜けたビン) と、
// 正逆で値が大きく食い違うビンは valid を偽にする。一部が抜けたビンの平均はビン中心の角度からずれる。
// max_lag には正逆の差の最大 [LSB] を入れる (摩擦による引き込み遅れの2倍)。
static bool Calib_EncPair(float speed, float* out, bool* valid, float* max_lag) {
  const float lead = ELEC_PER_REV / 8.0f;  // 記録前の助走 (向きを変えた直後の遅れの切り替わりを避ける)
  const float p = svc.calib_theta;
  const uint32_t total0 = enc_total;
  Calib_ClearBins();
  bool ok = Calib_MoveTo(p + lead, speed) &&
            Calib_Record(ENC_FORWARD, p + lead + ELEC_PER_REV, speed) &&
            Calib_MoveTo(p + 2.0f * lead + ELEC_PER_REV, speed) &&
            Calib_MoveTo(p + lead + ELEC_PER_REV, speed) &&
            Calib_Record(ENC_BACKWARD, p + lead, speed);
  if (!ok) return false;

  const float expect = (float)(enc_total - total0) / (2.0f * ENCODER_LUT_SIZE);
  const float range = (float)(svc.max_encoder_val - svc.min_encoder_val);
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    bool v = (work.enc.cnt[0][b] >= 0.95f * expect) && (work.enc.cnt[1][b] >= 0.95f * expect);
    float fwd = 0.0f, bwd = 0.0f;
    if (v) {
      fwd = work.enc.sum[0][b] / work.enc.cnt[0][b];
      bwd = work.enc.sum[1][b] / work.enc.cnt[1][b];
      float lag = Abs(fwd - bwd);
      if (lag > 0.25f * range) {
        v = false;  // 継ぎ目をはさんで正逆が別の側を読んだ
      } else if (lag > *max_lag) {
        *max_lag = lag;
      }
    }
    if (!v) valid[b] = false;
    out[b] = 0.5f * (fwd + bwd);
  }
  return true;
}

static bool Calib_Encoder(void) {
  printf("[CAL] エンコーダ (%sA で正逆1周)\n", FixFmt(CALIB_CURRENT, 1));
  const float speed = ELEC_PER_REV / ENC_SWEEP_S;

  // (a) 強制角のまま電流制御で保持し、静止ノイズを測る
  svc.target_id_override = CALIB_CURRENT;
  HAL_Delay(300);
  float a[AVG_COUNT];
  if (!Calib_Average(4000, a)) return false;
  float var = a[AVG_ENC2] - (a[AVG_ENC] - avg_enc0) * (a[AVG_ENC] - avg_enc0);
  float sigma = sqrtf(var > 0.0f ? var : 0.0f);

  // (b) レール: 1周目で極値、2周目でヒストグラム。ビンを使わないので倍速で回す
  enc_lo = 1e9f;
  enc_hi = -1e9f;
  for (uint8_t i = 0; i < ENC_HIST; i++) work.enc.hist[0][i] = work.enc.hist[1][i] = 0;
  float p = svc.calib_theta;
  if (!Calib_Record(ENC_EXTREME, p + ELEC_PER_REV, 2.0f * speed) ||
      !Calib_Record(ENC_HISTOGRAM, p + 2.0f * ELEC_PER_REV, 2.0f * speed)) {
    return false;
  }
  float off_lo = CalibMath_RailOffset(work.enc.hist[0], ENC_HIST);
  float off_hi = CalibMath_RailOffset(work.enc.hist[1], ENC_HIST);
  float rail_lo = enc_lo + off_lo, rail_hi = enc_hi - off_hi;
  if (!(off_lo >= 0.0f && off_hi >= 0.0f && rail_hi - rail_lo > MAX_ADC_VAL * 0.5f)) {
    printf("  レンジ不足 (%d〜%d)。ロータが回っていないか磁石が遠い\n", (int)enc_lo, (int)enc_hi);
    return false;
  }
  svc.min_encoder_val = (uint16_t)(rail_lo + 0.5f);
  svc.max_encoder_val = (uint16_t)(rail_hi + 0.5f);
  svc.encoder_margin = 4.0f * sigma + 2.0f;

  // (c) 2つの電流で正逆往復
  bool valid[ENCODER_LUT_SIZE];
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) valid[b] = true;
  enc_total = enc_sat = 0;
  float max_lag = 0.0f;
  svc.target_id_override = CALIB_CURRENT;
  HAL_Delay(200);
  if (!Calib_EncPair(speed, work.enc.raw_i1, valid, &max_lag)) return false;
  svc.target_id_override = 0.0f;

  svc.encoder_dead_zone = TWO_PI_F * (float)enc_sat / (float)(enc_total ? enc_total : 1);
  if (!(svc.encoder_dead_zone < TWO_PI_F * 0.25f)) {
    printf("  盲点が1/4周以上\n");
    return false;
  }

  // 回転方向: 隣り合う有効ビンの生値の増減 (継ぎ目の大きな跳びは除く)
  const float range = rail_hi - rail_lo;
  uint16_t inc = 0, dec = 0, n_valid = 0;
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    if (valid[b]) n_valid++;
    if (b + 1 >= ENCODER_LUT_SIZE || !valid[b] || !valid[b + 1]) continue;
    float d = work.enc.raw_i1[b + 1] - work.enc.raw_i1[b];
    if (Abs(d) > 0.5f * range) continue;
    if (d > 0.0f) inc++;
    if (d < 0.0f) dec++;
  }
  if (inc + dec < ENCODER_LUT_SIZE / 2 || (inc > dec ? inc : dec) < 0.9f * (inc + dec)) {
    printf("  回転方向が決まらない (増 %u / 減 %u)。ロータが滑っている\n", inc, dec);
    return false;
  }
  svc.encoder_dir = (inc > dec) ? 1.0f : -1.0f;
  BLDC_UpdateEncoderScale();

  // d軸オフセット。補正テーブルは惰性回転で作るまで0
  for (uint16_t b = 0; b < ENCODER_LUT_SIZE; b++) {
    if (valid[b]) work.enc.sum[0][b] = BLDC_EncoderLinear(work.enc.raw_i1[b]);
  }
  float max_step = 0.0f;
  float offset = CalibMath_BinOffset(work.enc.sum[0], valid, ENCODER_LUT_SIZE, &max_step);
  svc.encoder_offset_theta = (offset >= 0.0f) ? offset : 0.0f;
  for (uint16_t k = 0; k < ENCODER_LUT_SIZE; k++) svc.encoder_lut[k] = 0;

  // 生値の差 [LSB] → 機械角 [rad]
  const float k_rad = svc.encoder_scale;
  printf("  レール %u〜%u (極値 %d〜%d) σ %s margin %s dir %+d 盲点 %s%% offset %s\n",
         svc.min_encoder_val, svc.max_encoder_val, (int)enc_lo, (int)enc_hi, FixFmt(sigma, 2),
         FixFmt(svc.encoder_margin, 1), (int)svc.encoder_dir,
         FixFmt(svc.encoder_dead_zone * 100.0f / TWO_PI_F, 2), FixFmt(offset, 4));
  printf("  正逆差 %srad  有効 %u/%u  隣接差 %srad\n", FixFmt(max_lag * k_rad, 4), n_valid,
         (unsigned)ENCODER_LUT_SIZE, FixFmt(max_step, 4));

  // 隣接差が大きい = θ_lin − φ が途中で跳んでいる (滑り・POLE_PAIRS の不一致)。
  // 正逆差が大きい = 強制角にロータが付いてきていない (摩擦・負荷が大きい)
  if (offset < 0.0f || !(max_step < 0.1f) || max_lag * k_rad > 0.1f ||
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
// 摩擦 i_f は速度に依存してよい (同じ速度帯なので差で消える)。
// 電流は加速度が J_MAX_ACCEL 以下・記録が溢れない範囲になるよう自動で決める
// (速すぎ/遅すぎの両側が分かったら、その間を対数上で二分探索する。摩擦が大きいと
// 半分/倍だけでは「摩擦以下で回らない」と「減速が速すぎる」の間を往復して終わらない)。
// 見つかった電流 I に加えて別の電流 (1.5I か 0.75I) でも1回測り、結果がそろうかを確かめる
// (J/Kt が電流によらず同じ = モデルが合っている)。各方向2回、計4回の平均。
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
                                       0.8f * J_TOP_SPEED, true, &s_acc);
  uint16_t n_dec = CalibMath_BandSlope(&work.w[j_switch], j_n - j_switch, dir,
                                       0.2f * J_TOP_SPEED, 0.8f * J_TOP_SPEED, false, &s_dec);
  float a_acc = dir * s_acc / OUTER_LOOP_DT;  // 回転方向を正にした加速度 [rad/s²]
  float a_dec = dir * s_dec / OUTER_LOOP_DT;
  printf("  %+d方向 I=%sA: 加速 %d (%u点) 減速 %d (%u点) rad/s²\n", (int)dir, FixFmt(current, 2),
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
  float js[4], fs[4];
  uint8_t n = 0;
  for (uint8_t d = 0; d < 2; d++) {
    const float dir = (d == 0) ? 1.0f : -1.0f;

    // 電流の探索 (最初に成功した測定もそのまま使う)
    JResult r = J_FAILED;
    float too_slow = 0.0f, too_fast = 0.0f;  // これまでに分かった下限・上限 (0 = 未知)
    for (uint8_t attempt = 0; attempt < 8; attempt++) {
      r = Calib_InertiaRun(dir, current, &js[n], &fs[n]);
      if (r == J_OK || r == J_FAILED) break;
      if (r == J_TOO_FAST) {
        too_fast = current;
      } else {
        too_slow = current;
      }
      if (too_slow > 0.0f && too_fast > 0.0f) {
        current = sqrtf(too_slow * too_fast);
      } else {
        current *= (r == J_TOO_FAST) ? 0.5f : 2.0f;
      }
      if (current > 0.5f * MAX_CURRENT) current = 0.5f * MAX_CURRENT;
    }
    if (r != J_OK) {
      printf("  測定できない\n");
      return false;
    }
    n++;

    // 別の電流でもう1回 (速すぎ/遅すぎなら反対側の電流で)
    float other = (1.5f * current <= 0.5f * MAX_CURRENT) ? 1.5f * current : 0.75f * current;
    r = Calib_InertiaRun(dir, other, &js[n], &fs[n]);
    if (r == J_TOO_FAST || r == J_TOO_SLOW) {
      r = Calib_InertiaRun(dir, (other > current) ? 0.75f * current : 1.5f * current, &js[n], &fs[n]);
    }
    if (r != J_OK) {
      printf("  %+d方向で2つ目の電流が測れない\n", (int)dir);
      return false;
    }
    n++;
  }
  BLDC_Stop();

  float j_sum = 0.0f, f_sum = 0.0f, j_min = 1e9f, j_max = 0.0f;
  for (uint8_t k = 0; k < n; k++) {
    j_sum += js[k];
    f_sum += fs[k];
    if (js[k] < j_min) j_min = js[k];
    if (js[k] > j_max) j_max = js[k];
  }
  const float j_per_kt = j_sum / (float)n;
  const float friction = f_sum / (float)n;
  const float spread = (j_max - j_min) / j_per_kt * 100.0f;
  printf("  J/Kt %s e-6 A·s²/rad (%u回、ばらつき %s%%)  摩擦 %sA\n", FixFmt(j_per_kt * 1e6f, 1), n,
         FixFmt(spread, 1), FixFmt(friction, 3));
  if (spread > 30.0f) {
    printf("  ばらつき過大 (機構の引っかかり・負荷の変動を確認)\n");
    return false;
  }
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
// 各点 0.5 秒平均。試験速度を返す (確認試験で使う)。
static bool Calib_PsiDelay(float* out_w_test) {
  printf("[CAL] ψm と角度遅れ (%u点、**ロータが回る**)\n", (unsigned)PSI_POINTS);
  const float v_limit = svc.supply_volt * MAX_MODULATION_RATIO;
  float a[AVG_COUNT];

  // 暫定の ψm から、電圧の天井に当たらない試験速度を決める
  if (!Calib_ReachSpeed(0.25f * MAX_ANGULAR_SPEED) || !Calib_Average(PSI_AVG, a)) return false;
  float we = a[AVG_W] * POLE_PAIRS;
  float psi1 = (a[AVG_VQ] - svc.motor_r * a[AVG_IQ] - we * svc.motor_l * a[AVG_ID]) / we;
  if (!(psi1 > 1e-5f)) {
    printf("  ψm が正にならない (%s)\n", FixFmt(psi1, 6));
    return false;
  }
  svc.motor_psi = psi1;  // 掃引中の逆起電力FFに使う
  float w_test = 0.7f * v_limit / (psi1 * POLE_PAIRS);
  if (w_test > 0.9f * MAX_ANGULAR_SPEED) w_test = 0.9f * MAX_ANGULAR_SPEED;

  float x[PSI_POINTS], s[PSI_POINTS], y[PSI_POINTS], z[PSI_POINTS];
  const uint8_t half = PSI_POINTS / 2;
  for (uint8_t k = 0; k < PSI_POINTS; k++) {
    s[k] = (k < half) ? 1.0f : -1.0f;
    float target = s[k] * w_test * (float)(k % half + 1) / (float)half;
    if (!Calib_ReachSpeed(target) || !Calib_Average(PSI_AVG, a)) return false;
    x[k] = a[AVG_W] * POLE_PAIRS;
    y[k] = a[AVG_VQ] - svc.motor_r * a[AVG_IQ] - x[k] * svc.motor_l * a[AVG_ID];
    z[k] = a[AVG_VD] - svc.motor_r * a[AVG_ID] + x[k] * svc.motor_l * a[AVG_IQ];
  }
  BLDC_AngularSpeedControl(0.0f);
  HAL_Delay(1000);
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

  printf("  試験速度 %d rad/s  ψm %sWb  V0 %sV\n", (int)w_test, FixFmt(psi, 6), FixFmt(v0, 3));
  printf("  遅れ %s→%sms  静的ずれ δ0 %s°(電気角)  当てはめ残差 %s°\n",
         FixFmt(svc.angle_delay * 1000.0f, 3), FixFmt(delay * 1000.0f, 3),
         FixFmtSigned(delta0 * 57.29578f, 1), FixFmt(rms * 57.29578f, 1));
  if (!(psi > 1e-5f && psi < 1.0f) || !(delay >= 0.0f && delay < 0.005f) ||
      !(Abs(delta0) < 0.5f)) {
    printf("  範囲外\n");
    return false;
  }
  svc.motor_psi = psi;
  svc.angle_delay = delay;
  // 推定電気角を δ0 だけ戻す = 機械角のオフセットを δ0/P 増やす
  svc.encoder_offset_theta = FOC_NormalizeRadians(svc.encoder_offset_theta + delta0 / POLE_PAIRS);
  svc.encoder_primed = false;  // 新しいオフセットで張り直す
  *out_w_test = w_test;
  return true;
}

// ---------------------------------------------------------------------------
// 7. 確認試験 (保存前に閉ループで確かめる。ロータが回る)
// ---------------------------------------------------------------------------
//   (a) 継ぎ目の段差: 試験速度の半分で正逆それぞれ 0.5 秒 (約5周) 回し、採用したイノベーションの最大 emax を見る
//       (補正テーブルと盲点幅の出来。0.05 rad 超は注意、0.1 rad 超は失敗)
//   (b) 速度ループ: 試験速度の 1/4 → 3/4 へ指令を変え、ランプ (MAX_ANGULAR_ACCEL) への追従誤差を
//       2ms ごとに記録する。ランプ中の最大誤差 (加速度FFの出来)、ランプ後の行き過ぎ、最後 0.2 秒の
//       残留誤差 (振動していないか) を見る。行き過ぎが段差の 30% 超か、残留誤差 5 rad/s 超は失敗
static bool Calib_Verify(float w_test) {
  printf("[CAL] 確認試験 (**ロータが回る**)\n");
  // emax は方向ごとに、どこで出たか (生値と、飽和を抜けて何サンプル目か) も表示する。
  // 飽和直後 (数十サンプル以内) なら AS5600 の出力の遷移中の値、そうでなければ補正テーブルの誤差を疑う
  float emax = 0.0f;
  for (uint8_t d = 0; d < 2; d++) {
    if (!Calib_ReachSpeed((d == 0 ? 0.5f : -0.5f) * w_test)) return false;
    BLDC_ResetEncoderStats();
    HAL_Delay(500);
    float e = svc.max_innovation;
    printf("  %+d方向 emax %srad: 生値 %s (レール %u〜%u)、飽和から %u サンプル目 (%sms)\n",
           d == 0 ? 1 : -1, FixFmt(e, 4), FixFmt(svc.max_innovation_raw, 1), svc.min_encoder_val,
           svc.max_encoder_val, svc.max_innovation_after_sat,
           FixFmt(svc.max_innovation_after_sat * CURRENT_LOOP_DT * 1000.0f, 2));
    if (e > emax) emax = e;
  }

  const float w_lo = 0.25f * w_test, w_hi = 0.75f * w_test, step = w_hi - w_lo;
  if (!Calib_ReachSpeed(w_lo)) return false;
  j_n = 0;
  log_div = 0;
  CALIB_BARRIER();
  log_on = true;
  BLDC_AngularSpeedControl(w_hi);
  uint32_t t0 = HAL_GetTick();
  while (log_on && HAL_GetTick() - t0 < 2000) {
  }
  log_on = false;
  CALIB_BARRIER();
  BLDC_AngularSpeedControl(0.0f);
  HAL_Delay(1000);
  BLDC_Stop();

  // ランプの所要サンプル数 (2ms ごと)
  const uint16_t n_ramp = (uint16_t)(step / MAX_ANGULAR_ACCEL / 0.002f);
  float track = 0.0f, overshoot = 0.0f, rms = 0.0f;
  const uint16_t tail = J_LOG_SAMPLES / 4;
  for (uint16_t k = 0; k < j_n; k++) {
    float e = work.w[k];  // 指令ランプ − 実速度
    if (k < n_ramp) {
      if (Abs(e) > track) track = Abs(e);
    } else if (-e > overshoot) {
      overshoot = -e;
    }
    if (k >= J_LOG_SAMPLES - tail) rms += e * e;
  }
  rms = sqrtf(rms / (float)tail);
  printf("  emax %srad  追従誤差 最大 %s  行き過ぎ %s  残留 %s rad/s (段差 %d)\n", FixFmt(emax, 4),
         FixFmt(track, 1), FixFmt(overshoot, 1), FixFmt(rms, 2), (int)step);
  if (emax > 0.05f) printf("  注意 emax が 0.05rad 超 (継ぎ目の補正が不十分)\n");
  if (j_n < J_LOG_SAMPLES || emax > 0.1f || overshoot > 0.3f * step || rms > 5.0f) {
    printf("  確認試験に不合格\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 6. 補正テーブル (惰性回転。ロータが回る)
// ---------------------------------------------------------------------------
// 速度制御で回してから出力を切り、惰性で回る間に θ_lin が格子線を横切った時刻を記録する
// (計算と理由は calib_math.h の CalibMath_CoastFit)。正転・逆転を交互に COAST_RUNS 回。
// 開始速度は、摩擦で減速しても COAST_REVS 周回れる速さを J/Kt と摩擦電流から決める:
//   ω_top² = ω_end² + 2·a·2π·COAST_REVS、a = 摩擦電流 / (J/Kt)
// 出力を切ると全相がオフになる。逆起電力は電源電圧より十分低いのでダイオードも導通せず、電流は流れない。
static bool Calib_EncoderLut(void) {
  printf("[CAL] 補正テーブル (惰性回転 %u回、**ロータが回る**)\n", (unsigned)COAST_RUNS);
  const float decel = svc.friction_current / svc.j_per_kt;
  float top = sqrtf(COAST_END * COAST_END + 2.0f * decel * TWO_PI_F * COAST_REVS);
  if (top < 2.0f * COAST_END) top = 2.0f * COAST_END;
  if (top > 0.9f * MAX_ANGULAR_SPEED) top = 0.9f * MAX_ANGULAR_SPEED;

  CoastSeg seg[COAST_RUNS];
  uint16_t used = 0;
  for (uint8_t r = 0; r < COAST_RUNS; r++) {
    const float dir = (r & 1) ? -1.0f : 1.0f;
    if (!Calib_ReachSpeed(dir * top)) return false;
    coast_rec = (CoastRec){0};
    coast_rec.buf = &work.coast.buf[used];
    coast_rec.cap = (uint16_t)((COAST_BUF - used) / (COAST_RUNS - r));
    coast_rec.dir = (int8_t)dir;
    BLDC_Stop();
    HAL_Delay(2);  // 電流が消えるのを待つ (L/R ≒ 0.3ms)
    CALIB_BARRIER();
    enc_mode = ENC_COAST;
    uint32_t t0 = HAL_GetTick();
    while (!coast_rec.full && dir * svc.angular_speed > COAST_END && HAL_GetTick() - t0 < 2000) {
    }
    enc_mode = ENC_OFF;
    CALIB_BARRIER();
    seg[r].base = used;
    seg[r].n = coast_rec.n;
    seg[r].k0 = coast_rec.k0;
    seg[r].dir = coast_rec.dir;
    used += coast_rec.n;
    if (coast_rec.n < ENCODER_LUT_SIZE) {
      printf("  惰性で1周回らない (%u本)\n", coast_rec.n);
      return false;
    }
  }

  float fb = 0.0f;
  uint16_t missing = 0;
  float rms = CalibMath_CoastFit(work.coast.buf, seg, COAST_RUNS, ENCODER_LUT_SIZE, work.coast.e,
                                 work.coast.sum, work.coast.cnt, &fb, &missing);
  float e_max = 0.0f;
  for (uint16_t k = 0; k < ENCODER_LUT_SIZE; k++) {
    if (Abs(work.coast.e[k]) > e_max) e_max = Abs(work.coast.e[k]);
  }
  printf("  開始 %d rad/s  記録 %u本  残差 RMS %srad  正逆差 %srad  補正 最大 %srad  補間 %u本\n",
         (int)top, used, FixFmt(rms, 4), FixFmt(fb, 4), FixFmt(e_max, 4), missing);
  // 残差が大きい = 惰性中の速度が滑らかでない (引っかかり・負荷の変動)。補間が多い = 盲点が広すぎる
  if (!(rms >= 0.0f && rms < 0.01f) || e_max > 0.1f || missing > ENCODER_LUT_SIZE / 16) {
    printf("  補正テーブル不良\n");
    return false;
  }
  for (uint16_t k = 0; k < ENCODER_LUT_SIZE; k++) {
    float q = work.coast.e[k] / CALIB_LUT_SCALE;
    svc.encoder_lut[k] = (int16_t)(q + (q >= 0.0f ? 0.5f : -0.5f));
  }
  return true;
}

// ---------------------------------------------------------------------------
// 全体
// ---------------------------------------------------------------------------
bool BLDC_Calibrate(void) {
  printf("[CAL] スイッチ校正を開始 (校正電流 %sA、約36秒)\n", FixFmt(CALIB_CURRENT, 1));
  const uint32_t t_start = HAL_GetTick();
  svc.iq_limit = MAX_CURRENT;  // Kt が未定なので BLDC_SetLimits を介さず開ける (BLDC_Init の最後で閉じる)
  svc.calib_hook = true;

  // 1〜3 は強制角
  bool ok = Calib_MeasureRL() && Calib_TuneCurrentBw() && Calib_Encoder();
  svc.vd_override_active = false;
  svc.target_id_override = 0.0f;
  BLDC_Stop();
  svc.calib_angle_active = false;

  if (ok) {
    // 4〜7 は校正したエンコーダで閉ループ
    svc.encoder_calibrated = true;
    svc.encoder_primed = false;  // 新しい校正値で張り直す (次の ISR)
    HAL_Delay(50);
    float w_test = 0.0f;
    ok = Calib_Inertia() && Calib_PsiDelay(&w_test) && Calib_EncoderLut() && Calib_Verify(w_test);
    BLDC_Stop();
  }
  svc.calib_hook = false;

  svc.motor_calibrated = ok;
  if (!ok) {
    svc.encoder_calibrated = false;
    return false;
  }
  svc.encoder_primed = false;
  printf("[CAL] 結果: R %sΩ L %sµH ψm %sWb 遅れ %sms J/Kt %se-6 摩擦 %sA 帯域 %dHz\n",
         FixFmt(svc.motor_r, 4), FixFmt(svc.motor_l * 1e6f, 1), FixFmt(svc.motor_psi, 6),
         FixFmt(svc.angle_delay * 1000.0f, 3), FixFmt(svc.j_per_kt * 1e6f, 1),
         FixFmt(svc.friction_current, 3), (int)svc.current_bw_hz);
  printf("[CAL] J = %s g·cm² (= J/Kt × Kt)  所要 %lu 秒\n",
         FixFmt(svc.j_per_kt * BLDC_GetTorqueConstant() * 1e7f, 2),
         (unsigned long)((HAL_GetTick() - t_start) / 1000));
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
    printf("[CAL] 電流PI: Kp=%s Ki=%s (設計 %dHz)\n", FixFmt(svc.id_pi.kp, 4),
           FixFmt(svc.id_pi.ki, 1), (int)svc.current_bw_hz);
  }
}

// 速度PID: Kp = (J/Kt)·ω_c, Ki = Kp·ω_c / SPEED_KI_RATIO (config.h)。未校正なら0
void BLDC_UpdateSpeedGains(void) {
  float wc = TWO_PI_F * SPEED_BW_HZ;
  svc.speed_pid.kp = svc.j_per_kt * wc;
  svc.speed_pid.ki = svc.speed_pid.kp * wc / SPEED_KI_RATIO;
  svc.speed_pid.integral = 0;
  if (svc.j_per_kt > 0.0f) {
    printf("[CAL] 速度PID: Kp=%s Ki=%s (設計 %uHz)\n", FixFmt(svc.speed_pid.kp, 5),
           FixFmt(svc.speed_pid.ki, 4), (unsigned)SPEED_BW_HZ);
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

// 保存と読み出しで共用する (500B 超あるのでスタックに置かず、2つ持たない)
static BLDCFlashData flash_buf;

void BLDC_SaveCalibration(void) {
  BLDCFlashData* d = &flash_buf;
  memset(d, 0, sizeof(*d));
  d->magic = BLDC_FLASH_MAGIC;
  d->max_encoder_val = svc.max_encoder_val;
  d->min_encoder_val = svc.min_encoder_val;
  d->encoder_margin = (uint16_t)(svc.encoder_margin + 0.5f);
  d->encoder_dir = (int16_t)svc.encoder_dir;
  d->encoder_offset_theta = svc.encoder_offset_theta;
  d->encoder_dead_zone = svc.encoder_dead_zone;
  d->motor_r = svc.motor_r;
  d->motor_l = svc.motor_l;
  d->motor_psi = svc.motor_psi;
  d->angle_delay = svc.angle_delay;
  d->j_per_kt = svc.j_per_kt;
  d->friction_current = svc.friction_current;
  d->current_bw_hz = svc.current_bw_hz;
  memcpy(d->encoder_lut, svc.encoder_lut, sizeof(d->encoder_lut));
  d->checksum = Calib_Checksum(d);
  printf("[CAL] フラッシュ書き込み%s\n",
         Flash_WriteData(FLASH_USER_START_ADDR, d, sizeof(*d)) == HAL_OK ? "完了" : "に失敗");
}

// マジックナンバー・チェックサム・範囲で検証する (NaN も落とせるよう不等号は肯定形で書く)
void BLDC_LoadFlashCalibration(void) {
  BLDCFlashData* d = &flash_buf;
  Flash_ReadData(FLASH_USER_START_ADDR, d, sizeof(*d));

  bool ok = (d->magic == BLDC_FLASH_MAGIC) && (d->checksum == Calib_Checksum(d)) &&
            (d->max_encoder_val > d->min_encoder_val) && (d->max_encoder_val <= MAX_ADC_VAL) &&
            (d->encoder_dir == 1 || d->encoder_dir == -1) && (d->encoder_margin < 100) &&
            (d->encoder_offset_theta >= 0.0f && d->encoder_offset_theta < TWO_PI_F) &&
            (d->encoder_dead_zone >= 0.0f && d->encoder_dead_zone < TWO_PI_F * 0.25f) &&
            (d->motor_r > 0.001f && d->motor_r < 100.0f) && (d->motor_l > 1e-6f && d->motor_l < 0.1f) &&
            (d->motor_psi > 1e-5f && d->motor_psi < 1.0f) &&
            (d->angle_delay >= 0.0f && d->angle_delay < 0.005f) &&
            (d->j_per_kt > 1e-7f && d->j_per_kt < 1.0f) &&
            (d->friction_current >= 0.0f && d->friction_current < MAX_CURRENT) &&
            (d->current_bw_hz >= 0.8f * CURRENT_BW_TARGET_HZ &&
             d->current_bw_hz <= 2.0f * CURRENT_BW_TARGET_HZ);
  if (!ok) {
    printf("BLDC: 校正値が無いか不正。スイッチを押しながら起動して校正すること (駆動しない)\n");
    return;
  }
  svc.max_encoder_val = d->max_encoder_val;
  svc.min_encoder_val = d->min_encoder_val;
  svc.encoder_margin = (float)d->encoder_margin;
  svc.encoder_dir = (float)d->encoder_dir;
  svc.encoder_offset_theta = d->encoder_offset_theta;
  svc.encoder_dead_zone = d->encoder_dead_zone;
  memcpy(svc.encoder_lut, d->encoder_lut, sizeof(svc.encoder_lut));
  svc.motor_r = d->motor_r;
  svc.motor_l = d->motor_l;
  svc.motor_psi = d->motor_psi;
  svc.angle_delay = d->angle_delay;
  svc.j_per_kt = d->j_per_kt;
  svc.friction_current = d->friction_current;
  svc.current_bw_hz = d->current_bw_hz;
  svc.encoder_calibrated = true;
  svc.motor_calibrated = true;
  printf("(Flash) enc %u-%u dir %+d offset %s 盲点 %s / R %s L %sµH ψm %s 遅れ %sms"
         " J/Kt %se-6 摩擦 %sA\n",
         d->min_encoder_val, d->max_encoder_val, d->encoder_dir, FixFmt(d->encoder_offset_theta, 4),
         FixFmt(d->encoder_dead_zone, 4), FixFmt(d->motor_r, 4), FixFmt(d->motor_l * 1e6f, 1),
         FixFmt(d->motor_psi, 6), FixFmt(d->angle_delay * 1000.0f, 3), FixFmt(d->j_per_kt * 1e6f, 1),
         FixFmt(d->friction_current, 3));
}
