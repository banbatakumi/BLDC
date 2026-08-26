#ifndef BLDC_INTERNAL_H_
#define BLDC_INTERNAL_H_

// bldc.c (20kHz制御ループ本体) と bldc_calibration.c (エンコーダ/モータ定数の
// 校正・同定) が共有する内部状態と内部ヘルパー。
//
// 公開API (app 側が使ってよいもの) は bldc.h。こちらは実装の詳細なので、
// bldc.c と bldc_calibration.c 以外からはインクルードしないこと。
#include "bldc.h"

// 測定関数はスイッチ校正からも呼ぶので、診断表示が無効でもコンパイルする必要がある。
#define BLDC_NEED_RL (MEASURE_MOTOR_RL || MOTOR_AUTO_CALIBRATION)
#define BLDC_NEED_PSI (MEASURE_MOTOR_PSI || MOTOR_AUTO_CALIBRATION)
#define BLDC_MEASURING (BLDC_NEED_RL || MEASURE_CURRENT_STEP)

#if BLDC_MEASURING
// ステップ応答の記録状態。ISRとメインループの両方から見るので volatile。
typedef enum {
  BLDC_CAPTURE_IDLE = 0,
  BLDC_CAPTURE_ARMED,    // 次の制御周期から記録を始める
  BLDC_CAPTURE_RUNNING,  // 記録中
  BLDC_CAPTURE_DONE,     // バッファが埋まった
} BLDCCaptureState;

// Id の記録 [mA]。int16 なので ±32A まで入る。200点で 400バイト。
// 電圧ステップと電流ステップは同時に走らないので共用する。
// 実体は bldc.c (20kHz ISR が書く)。bldc_calibration.c は読むだけ。
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
  // config.h の既定値ではなく、スイッチ校正で実測してフラッシュに保存した値を使う。
  // ISRから毎周期読むので svc に置く (定数畳み込みは効かなくなるが、
  // ロード数回ぶんなので実測で 0.1µs も変わらない)。
  float motor_r;      // 相抵抗 [Ω]
  float motor_l;      // 相インダクタンス [H]
  float motor_psi;    // 永久磁石の鎖交磁束 [Wb]
  float angle_delay;  // 電気角の実効遅れ [s]

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
  // **既定値は 0 = 動かない。** 上位から制限値を受け取るまでモータは回らない。
  // メインループが書き、20kHzの割り込みが読むので volatile。
  // 速度の制限は持たない (上位が指令値そのもので決める)。
  volatile float iq_limit;  // トルク指令の飽和値をIqへ換算した値 [A]

  // --- 状態 ---
  float mech_theta;     // 機械角 [rad]
  float elec_theta;     // 電気角 [rad]
  float angular_speed;  // 角速度 [rad/s]

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

// 実体は bldc.c。20kHz ISR (bldc.c) とキャリブレーション (bldc_calibration.c)
// の両方が同じ状態を読み書きするので extern で共有する。
extern SensoredVectorControl svc;

// ---------------------------------------------------------------------------
// bldc.c と bldc_calibration.c が共有する内部ヘルパー
// ---------------------------------------------------------------------------
// PWM出力。20kHz ISR (bldc.c の BLDC_CurrentLoop) と強制転流
// (bldc_calibration.c の BLDC_OpenLoopDrive) の両方から呼ばれるので、
// static inline で各ファイルにインライン展開する。
//
// float の比較は VCMP.F32 + VMRS APSR_nzcv,FPSCR になり、FPUのフラグを
// コアへ転送する VMRS でパイプラインが待たされて1回8〜10サイクルかかる。
// 整数の比較はコアのフラグを直接使うので1サイクル。3相ぶんで6回比較するので、
// ここだけで50サイクル近く違う。
//
// 安全性もこちらのほうが高い。ARMの VCVT は NaN を 0 に、範囲外を飽和させるので、
// 変換してから整数でクランプすれば入力が何であれ必ず範囲に収まる。float のまま
// 比較すると NaN は「どの比較も偽」ですり抜けてしまう。
// センター揃えなので デューティ = CCR / PWM_ARR (エッジ揃えの ARR+1 ではない)。
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

// 生のADC値がレール付近か = AS5600のアナログ出力が飽和していて角度情報が無いか。
//
// 校正 (盲点幅の測定、bldc_calibration.c) と運転中のオブザーバ (bldc.c) は
// **同じ判定でなければならない**。盲点幅は「この判定に引っかかったサンプルの
// 割合」として測っているので、判定が食い違うと encoder_scale が実際の飽和区間と
// 合わなくなり、継ぎ目に段差が出る。
static inline bool BLDC_IsEncoderSaturated(uint16_t encoder_val) {
  return (encoder_val <= svc.min_encoder_val + ENCODER_EDGE_MARGIN_LSB) ||
         (encoder_val >= svc.max_encoder_val - ENCODER_EDGE_MARGIN_LSB);
}

// 角度追従オブザーバ (2次PLL)。機械角と角速度を同時に更新する。
// 20kHz ISR (bldc.c) とエンコーダ校正 (bldc_calibration.c の BLDC_SetEncoder)
// の両方から呼ばれる。
//
// 「角度を微分して速度を出す」のをやめ、推定角 θ̂ を観測角に追従させる制御ループを
// 回して、その内部状態として θ̂ と ω̂ を得る (パラメータは config.h)。
//
//   e   = θ_meas - θ̂                (イノベーション)
//   ω̂ += PLL_KI * e * dt            (速度推定 = 積分器)
//   θ̂ += (ω̂ + PLL_KP * e) * dt      (角度推定)
//
// 微分ではなく積分で速度を作るので、
//   1. 量子化ノイズが増幅されない (微分は高周波を持ち上げる)
//   2. 等速回転に対して定常誤差ゼロ (1次ローパスと違い位相が遅れない)
//   3. 観測が一時的に欠測しても ω̂ による外挿で角度が進み続ける
// という3つの利点が同時に得られる。3つ目が、AS5600の飽和域を渡るときに
// 速度がゼロに落ちてから跳ね上がる (= 1回転ごとの「がくっ」) の直接の対策になる。
static inline void BLDC_UpdateEncoder(uint16_t encoder_val) {
  uint16_t clamped_encoder_val = Constrain(encoder_val, svc.min_encoder_val, svc.max_encoder_val);
  float theta_meas = (float)(clamped_encoder_val - svc.min_encoder_val) * svc.encoder_scale;
  theta_meas = FOC_NormalizeRadians(theta_meas - svc.encoder_offset_theta);

  // 初回は推定値を実測値に張り付ける。
  // これをしないと mech_theta が 0 から実際の角度まで移動する間、
  // 電気角がその極対数倍(7倍)の速さで何回転もしてしまい、電流制御が破綻する。
  if (!svc.encoder_primed) {
    svc.encoder_primed = true;
    svc.mech_theta = theta_meas;
    svc.angular_speed = 0.0f;
    svc.encoder_reject_count = 0;
    svc.encoder_glitch_count = 0;
    return;
  }

  // 0と2πの境目を跨いでも壊れないよう、-π〜+π に正規化して差を取る
  float e = FOC_AngleDiff(theta_meas, svc.mech_theta);

  // --- 観測値の妥当性チェック ---
  // 棄却したサンプルは補正に使わず、ω̂ による外挿(デッドレコニング)だけで進む。
  //
  // 「観測できない」と「推定がずれている」は必ず区別すること。混ぜると、
  // ロータを手で急停止させたときに θ̂ が ω̂ のまま自走を始め、その状態を
  // 「異常な観測」と誤認して棄却し続ける → 永久にロックが戻らない
  // (= オープンループ駆動と同じ、脱調したような挙動) という事故になる。
  if (BLDC_IsEncoderSaturated(encoder_val)) {
    // (a) 飽和: AS5600のアナログ出力がレールに張り付いていて角度情報が存在しない。
    //     「観測できない」ことが生のADC値から確実に分かるので、長めに外挿してよい。
    if (svc.encoder_reject_count < ENCODER_REJECT_MAX) {
      svc.encoder_reject_count++;
      svc.saturated_total++;
      e = 0.0f;
    }
  } else if (Abs(e) > ENCODER_INNOVATION_LIMIT_RAD) {
    // (b) イノベーション過大: 0/2π遷移中のグリッチかもしれないし、ロータが
    //     急停止して推定が本当にずれたのかもしれない。1サンプルでは区別できない。
    //     グリッチは短いので、まず ENCODER_GLITCH_MAX サンプルだけ様子を見る。
    //     それでも続くならグリッチではないので棄却をやめ、e を全量PLLに通して
    //     観測に再ロックさせる。ここを打ち切らないと上記の自走から戻れない。
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
  // 速度推定(積分パス)に入れるイノベーションは「ロック中のみ」制限する。
  //
  //   ロック中: 残っているイノベーションは継ぎ目の段差(校正誤差)や外挿の誤差で
  //     あって本物の回転ではない。全量を積分に入れると1回転ごとに速度スパイクに
  //     なるので制限する。比例パスは無制限なので角度は素早く追いつき、段差は
  //     「角度の補正」として吸収される。
  //   再取得中: 制限してはいけない。dω̂/dt が PLL_KI × 制限値 で頭打ちになり、
  //     手で急停止させたときの減速に追従できず、脱調状態から戻れなくなる。
  //
  // 上のゲート(b)が「グリッチではない = 推定が本当にずれている」と判定した状態
  // (glitch_count が上限に張り付いている) が、そのまま再取得中の判定になる。
  bool reacquiring = (svc.encoder_glitch_count >= ENCODER_GLITCH_MAX);
  float e_speed = reacquiring
                      ? e
                      : Constrain(e, -PLL_SPEED_ERROR_LIMIT_RAD, PLL_SPEED_ERROR_LIMIT_RAD);

  svc.angular_speed += PLL_KI * e_speed * CURRENT_LOOP_DT;
  svc.mech_theta = FOC_NormalizeRadians(
      svc.mech_theta + (svc.angular_speed + PLL_KP * e) * CURRENT_LOOP_DT);

  // 採用したイノベーションのピークを記録する (調査用)。
  // これが継ぎ目の段差の大きさそのものなので、校正の善し悪しが直接見える。
  float abs_e = Abs(e);
  if (abs_e > svc.max_innovation) svc.max_innovation = abs_e;
}

// エンコーダのレンジからラジアン変換係数を作り直す。max/min/盲点幅を変えたら必ず呼ぶこと。
// 定義は bldc.c。BLDC_Init (bldc.c) と、校正で min/max/盲点を測り直した直後
// (bldc_calibration.c の BLDC_SetEncoder) の両方から呼ぶ。
void BLDC_UpdateEncoderScale(void);

// ---------------------------------------------------------------------------
// bldc_calibration.c で定義し、bldc.c の BLDC_Init から呼ぶ関数
// ---------------------------------------------------------------------------
// エンコーダの最大/最小値と電気角のオフセットを実測する (フラッシュ書き込みはしない)
void BLDC_SetEncoder(volatile uint16_t* encoder_val);

// フラッシュに保存した校正値を読み出して適用する
void BLDC_LoadFlashCalibration(void);

// 電流PIのゲインを、いまの L と R から計算し直す
void BLDC_UpdateCurrentGains(void);

// 起動時の診断表示 (config.h の MEASURE_* で個別に有効化。全部0なら空関数)
void BLDC_RunStartupMeasurements(void);

#if MOTOR_AUTO_CALIBRATION
// モータ定数の自動測定 (R,L → ψm → 角度遅れ の順)
void BLDC_CalibrateMotor(void);

// 校正値をまとめてフラッシュに保存する
void BLDC_SaveCalibration(void);
#endif

#endif  // BLDC_INTERNAL_H_
