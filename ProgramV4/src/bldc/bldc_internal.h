#ifndef BLDC_INTERNAL_H_
#define BLDC_INTERNAL_H_

// bldc.c (20kHz制御ループ) と bldc_calibration.c (校正・同定) が共有する内部状態とヘルパー。
// 公開APIは bldc.h。この2ファイル以外からはインクルードしないこと。
#include "bldc.h"

// 測定関数はスイッチ校正からも呼ぶので、診断表示が無効でもコンパイルする
#define BLDC_NEED_RL (MEASURE_MOTOR_RL || MOTOR_AUTO_CALIBRATION)
#define BLDC_NEED_PSI (MEASURE_MOTOR_PSI || MOTOR_AUTO_CALIBRATION)
#define BLDC_MEASURING (BLDC_NEED_RL || MEASURE_CURRENT_STEP)

#if BLDC_MEASURING
// ステップ応答の記録状態 (ISRとメインループの両方から見る)
typedef enum {
  BLDC_CAPTURE_IDLE = 0,
  BLDC_CAPTURE_ARMED,    // 次の制御周期から記録を始める
  BLDC_CAPTURE_RUNNING,  // 記録中
  BLDC_CAPTURE_DONE,     // バッファが埋まった
} BLDCCaptureState;

// Id の記録 [mA]。電圧ステップと電流ステップで共用。bldc.c の ISR が書き、
// bldc_calibration.c は読むだけ。
extern int16_t bldc_capture[BLDC_CAPTURE_SAMPLES];
#endif

typedef struct {
  // --- 設定 ---
  volatile uint16_t* encoder_val_ptr;  // ADC2のDMAが更新するエンコーダ値へのポインタ
  uint16_t max_encoder_val;            // エンコーダーの最大値
  uint16_t min_encoder_val;            // エンコーダーの最小値
  float encoder_dead_zone;             // レール飽和で角度が読めない区間の幅 [rad]
  float encoder_scale;                 // (2π - 盲点) / (max - min)。事前計算して割り算を避ける
  float encoder_offset_theta;          // エンコーダーのオフセット値 [rad]

  // --- モータの電気的パラメータ ---
  // config.h の既定値ではなく、スイッチ校正でフラッシュに保存した実測値を使う。
  float motor_r;      // 相抵抗 [Ω]
  float motor_l;      // 相インダクタンス [H]
  float motor_psi;    // 永久磁石の鎖交磁束 [Wb]
  float angle_delay;  // 電気角の実効遅れ [s]

  bool encoder_calibrated;        // エンコーダの校正値が有効か (フラッシュから読めた / このセッションで測った)
  bool encoder_primed;            // 機械角をエンコーダ実測値で初期化済みか
  uint16_t encoder_reject_count;  // 飽和域で連続して棄却した回数
  uint16_t encoder_glitch_count;  // イノベーション過大で連続して棄却した回数
  uint32_t saturated_total;       // 飽和で棄却した総数     (調査用)
  uint32_t glitch_total;          // グリッチで棄却した総数 (調査用)
  float max_innovation;           // 採用したイノベーションのピーク [rad] (調査用)

  // --- 指令 ---
  volatile BLDCMode mode;
  volatile bool enable;
  volatile float supply_volt;           // 母線電圧 [V]
  volatile float target_angular_speed;  // [rad/s]
  volatile float target_position;       // [rad]
  volatile float target_current;        // トルク制御のIq指令 [A]
  volatile float brake_current;         // ブレーキ電流の大きさ [A]

  // --- 上位から指定される制限 (BLDC_SetLimits) ---
  // 既定値は 0 = 動かない。メインループが書き、20kHz割り込みが読む。
  volatile float iq_limit;  // トルク指令の飽和値をIqへ換算した値 [A]

  // --- 状態 ---
  float mech_theta;     // 機械角 [rad]
  float elec_theta;     // 電気角 [rad]
  float angular_speed;  // 角速度 [rad/s]
  float brake_direction;  // 制動電流に掛ける係数 (-1〜1)。外側ループ(1kHz)が速度から更新する

  float iu, iv;  // 相電流 [A] (W相は Iw = -(Iu+Iv) なので持たない)
  float id, iq;  // dq軸電流 [A]
  float vd, vq;  // dq軸電圧指令 [V]
  float vq_ff;   // Vqのうちフィードフォワードで作った分 [V] (調査用)
  float target_iq;
  float target_id_override;  // d軸指令の上書き。通常0 (SPMなので弱め界磁しない)
  float speed_ramp;          // 加速度制限をかけた角速度指令 [rad/s]

#if BLDC_MEASURING
  volatile BLDCCaptureState capture_state;
  uint16_t capture_index;
  float capture_step;       // ステップの大きさ ([A] または [V])
  bool capture_is_voltage;  // true なら電圧ステップ (PIをバイパスする)
#endif
#if BLDC_NEED_RL
  volatile bool vd_override_active;  // true の間 PI を通さず Vd を直接与える
  float vd_override;                 // 与える d軸電圧 [V]
  volatile bool rl_aborted;          // 電流が上限を超えて中断したか
#endif

  bool is_overcurrent;
  uint16_t overcurrent_count;  // 連続してしきい値を超えた回数
  BLDCTripInfo trip;           // 保護が働いた瞬間の状態
  float peak_current;          // 相電流ピークの記録 [A]

  // --- 制御器 ---
  PIDController speed_pid;     // 角速度制御用PID (出力: Iq指令 [A])
  PIDController position_pid;  // 位置制御用PID   (出力: Iq指令 [A])
  PIController id_pi;          // d軸電流PI       (出力: Vd [V])
  PIController iq_pi;          // q軸電流PI       (出力: Vq [V])
} SensoredVectorControl;

// 実体は bldc.c
extern SensoredVectorControl svc;

// ---------------------------------------------------------------------------
// 共有ヘルパー (static inline で各ファイルに展開)
// ---------------------------------------------------------------------------
// デューティ → CCR。デューティ = CCR / PWM_ARR (センター揃え)。
// 整数でクランプするのは、float比較が VMRS でパイプラインを待たせて遅い (3相6回で約50サイクル差) のと、
// VCVT が NaN を 0 に、範囲外を飽和させるので入力が何でも必ず範囲に収まるため。
static inline uint32_t BLDC_DutyToCcr(float duty) {
  int32_t ccr = (int32_t)(duty * PWM_ARR);
  if (ccr < MIN_DUTY_CCR) ccr = MIN_DUTY_CCR;
  if (ccr > MAX_DUTY_CCR) ccr = MAX_DUTY_CCR;
  return (uint32_t)ccr;
}

static inline void BLDC_WritePwm(float u, float v, float w) {
  TIM1->CCR1 = BLDC_DutyToCcr(u);
  TIM1->CCR2 = BLDC_DutyToCcr(v);
  TIM1->CCR3 = BLDC_DutyToCcr(w);
}

// 生のADC値がレール付近か (AS5600のアナログ出力が飽和し角度情報が無いか)。
// 校正の盲点幅は「この判定に引っかかったサンプルの割合」で測るので、
// 校正 (bldc_calibration.c) と運転中 (bldc.c) で同じ判定でなければならない。
static inline bool BLDC_IsEncoderSaturated(uint16_t encoder_val) {
  return (encoder_val <= svc.min_encoder_val + ENCODER_EDGE_MARGIN_LSB) ||
         (encoder_val >= svc.max_encoder_val - ENCODER_EDGE_MARGIN_LSB);
}

// 角度追従オブザーバ (2次PLL)。機械角と角速度を同時に更新する (パラメータは config.h)。
//   e   = θ_meas - θ̂
//   ω̂ += PLL_KI * e * dt
//   θ̂ += (ω̂ + PLL_KP * e) * dt
// 微分でなく積分で速度を作るので、量子化ノイズが増幅されず、等速回転で定常誤差がなく、
// 観測が欠測しても ω̂ で外挿して進み続ける (AS5600の飽和域で速度が落ちる対策)。
static inline void BLDC_UpdateEncoder(uint16_t encoder_val) {
  uint16_t clamped_encoder_val = Constrain(encoder_val, svc.min_encoder_val, svc.max_encoder_val);
  float theta_meas = (float)(clamped_encoder_val - svc.min_encoder_val) * svc.encoder_scale;
  theta_meas = FOC_NormalizeRadians(theta_meas - svc.encoder_offset_theta);

  // 初回は推定値を実測値に張り付ける (しないと 0 から実角度まで電気角が何回転もして破綻する)

  if (!svc.encoder_primed) {
    svc.encoder_primed = true;
    svc.mech_theta = theta_meas;
    svc.angular_speed = 0.0f;
    svc.encoder_reject_count = 0;
    svc.encoder_glitch_count = 0;
    return;
  }

  // 0/2πの境目を跨いでも壊れないよう -π〜+π で差を取る
  float e = FOC_AngleDiff(theta_meas, svc.mech_theta);

  // --- 観測値の妥当性チェック (棄却したサンプルは ω̂ の外挿だけで進む) ---
  // 「観測できない」と「推定がずれている」は区別すること (config.h 参照)。
  if (BLDC_IsEncoderSaturated(encoder_val)) {
    // (a) 飽和: 角度情報が無いと生のADC値から確実に分かるので、長めに外挿してよい
    if (svc.encoder_reject_count < ENCODER_REJECT_MAX) {
      svc.encoder_reject_count++;
      svc.saturated_total++;
      e = 0.0f;
    }
  } else if (Abs(e) > ENCODER_INNOVATION_LIMIT_RAD) {
    // (b) イノベーション過大: グリッチか推定ずれか区別できないので ENCODER_GLITCH_MAX サンプル
    //     様子を見て、続くなら棄却をやめて e を全量PLLに通し再ロックさせる
    if (svc.encoder_glitch_count < ENCODER_GLITCH_MAX) {
      svc.encoder_glitch_count++;
      svc.glitch_total++;
      e = 0.0f;
    }
  } else {
    // 観測と推定が一致している = ロックしている
    svc.encoder_reject_count = 0;
    svc.encoder_glitch_count = 0;
  }

  // --- PLL本体 ---
  // 速度推定(積分パス)のイノベーションはロック中のみ制限する (理由は config.h の PLL_SPEED_ERROR_LIMIT_RAD)。
  // glitch_count が上限に張り付いている状態 = 再取得中。
  bool reacquiring = (svc.encoder_glitch_count >= ENCODER_GLITCH_MAX);
  float e_speed = reacquiring
                      ? e
                      : Constrain(e, -PLL_SPEED_ERROR_LIMIT_RAD, PLL_SPEED_ERROR_LIMIT_RAD);

  svc.angular_speed += PLL_KI * e_speed * CURRENT_LOOP_DT;
  svc.mech_theta = FOC_NormalizeRadians(
      svc.mech_theta + (svc.angular_speed + PLL_KP * e) * CURRENT_LOOP_DT);

  // 採用したイノベーションのピーク (継ぎ目の段差の大きさ = 校正の善し悪し)
  float abs_e = Abs(e);
  if (abs_e > svc.max_innovation) svc.max_innovation = abs_e;
}

// エンコーダのレンジからラジアン変換係数を作り直す。max/min/盲点幅を変えたら必ず呼ぶこと。
// 定義は bldc.c
void BLDC_UpdateEncoderScale(void);

// --- bldc_calibration.c で定義し、BLDC_Init から呼ぶ関数 ---
// エンコーダの最大/最小値と電気角のオフセットを実測する (フラッシュ書き込みはしない)
void BLDC_SetEncoder(volatile uint16_t* encoder_val);

// フラッシュに保存した校正値を読み出して適用する
void BLDC_LoadFlashCalibration(void);

// 電流PIのゲインを、いまの L と R から計算し直す
void BLDC_UpdateCurrentGains(void);

// 起動時の診断 (config.h の MEASURE_*。全部0なら空関数)
void BLDC_RunStartupMeasurements(void);

#if MOTOR_AUTO_CALIBRATION
// モータ定数の自動測定 (R,L → ψm → 角度遅れ の順)
void BLDC_CalibrateMotor(void);

// 校正値をまとめてフラッシュに保存する
void BLDC_SaveCalibration(void);
#endif

#endif  // BLDC_INTERNAL_H_
