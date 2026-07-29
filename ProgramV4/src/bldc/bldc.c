#include "bldc.h"

// 測定関数はスイッチ校正からも呼ぶので、診断表示が無効でもコンパイルする必要がある。
#define BLDC_NEED_RL (MEASURE_MOTOR_RL || MOTOR_AUTO_CALIBRATION)
#define BLDC_NEED_PSI (MEASURE_MOTOR_PSI || MOTOR_AUTO_CALIBRATION)
#define BLDC_MEASURING (BLDC_NEED_RL || MEASURE_CURRENT_STEP)

#if BLDC_NEED_RL || BLDC_NEED_PSI
#include <math.h>  // logf / asinf (起動時の同定でしか使わない。ISRからは呼ばない)
#endif

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
static int16_t bldc_capture[BLDC_CAPTURE_SAMPLES];
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

  bool encoder_primed;                 // 機械角をエンコーダ実測値で初期化済みか
  uint16_t encoder_reject_count;       // 飽和域で連続して棄却した回数
  uint16_t encoder_glitch_count;       // イノベーション過大で連続して棄却した回数
  uint32_t saturated_total;            // 飽和で棄却した総数     (調査用)
  uint32_t glitch_total;               // グリッチで棄却した総数 (調査用)
  float max_innovation;                // 採用したイノベーションのピーク [rad] (調査用)

  // --- 指令 ---
  volatile BLDCMode mode;
  volatile bool enable;
  volatile float supply_volt;           // 母線電圧 [V]
  volatile float target_angular_speed;  // [rad/s]
  volatile float target_position;       // [rad]
  volatile float target_current;        // トルク制御のIq指令 [A]
  volatile float brake_current;         // ブレーキ電流の大きさ [A]

  // --- 状態 ---
  float mech_theta;     // 機械角 [rad]
  float elec_theta;     // 電気角 [rad]
  float angular_speed;  // 角速度 [rad/s]
  float angular_accel;  // 角加速度 [rad/s^2]

  float iu, iv, iw;  // 相電流 [A]
  float id, iq;      // dq軸電流 [A]
  float vd, vq;      // dq軸電圧指令 [V]
  float vq_ff;       // Vqのうちフィードフォワードで作った分 [V] (調査用)
  float target_id, target_iq;
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

static SensoredVectorControl svc;

static PwmOut u_pwm;
static PwmOut v_pwm;
static PwmOut w_pwm;

// ---------------------------------------------------------------------------
// PWM出力
// ---------------------------------------------------------------------------
// 20kHzで毎回叩くのでCCRレジスタを直接書く。
// PwmOut_Write は __HAL_TIM_SET_COMPARE のチャンネル判定(switch)が展開されて
// 1相あたり約100命令になるため、ここでは使わない。
// チャンネルの割り当ては BLDC_Init の PwmOut_Init と対応させること。
//   CH1 = u_pwm (OUTC),  CH2 = v_pwm (OUTB),  CH3 = w_pwm (OUTA)
// デューティ [0,1] → CCR値。クランプは float ではなく整数で行う。
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

// 出力の有効/無効。無効側は「フリーラン(コースト)」= 上下FETともOFF。
//
// 全相デューティ0.5は線間電圧こそ0だが、3相の端子が常に同電位で振られるため
// モータの3相を短絡しているのと同じ状態になる。止まっていれば無害でも、回転中は
// 逆起電力が短絡電流 I ≈ E / |R + jωL| を流すので短絡制動になってしまう。
// 過電流保護が働いた直後にこれをやると、一番電流を流したくない場面で
// 一番電流が流れる状態に入ることになる。
//
// MOE=0 にすると全チャンネルの出力が止まる。BDTR の OSSI=1 と
// OISx/OISxN=0 (tim.c で TIM_OCIDLESTATE_RESET) の組み合わせにより、出力ピンは
// Hi-Z ではなくアイドルレベル(Low)に固定される。DRV8300 の INH/INL が
// どちらも Low になるので上下FETともOFF、つまりコーストになる。
static inline void BLDC_SetOutputEnable(bool on) {
  if (on) {
    __HAL_TIM_MOE_ENABLE(&htim1);
  } else {
    // __HAL_TIM_MOE_DISABLE は「全チャンネルが無効なとき」しか効かないマクロなので使わない
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }
}

// 強制転流(オープンループ駆動)。エンコーダ校正でのみ使う。
static void BLDC_OpenLoopDrive(float amp, float phase) {
  phase = NormalizeRadians(phase);

  float u = 0.5f + 0.5f * amp * Cos(phase);
  float v = 0.5f + 0.5f * amp * Cos(phase - TWO_THIRDS_PI);
  float w = 0.5f + 0.5f * amp * Cos(phase + TWO_THIRDS_PI);

  BLDC_WritePwm(u, v, w);
}

// ---------------------------------------------------------------------------
// センサ処理
// ---------------------------------------------------------------------------
// エンコーダのレンジからラジアン変換係数を作り直す。max/min/盲点幅を変えたら必ず呼ぶこと。
//
// 素朴に 2π / (max - min) としてはいけない。AS5600のアナログ出力はレール付近で
// 飽和しており、ADC値 [min, max] が実際にカバーしているのは 2π ではなく
// 「2π - 盲点幅」だから。2π を割り当てると θ_meas が 2π/(2π-dead) 倍に引き伸ばされ、
//   - 観測できる区間では ω̂ がその比率ぶん高く出る
//   - 盲点を外挿で渡ると θ̂ が dead × その比率 だけ進み、観測再開時に段差になる
// という形で、1回転ごとの速度スパイクの原因になる。
// (実測: 盲点 0.108rad のとき比率 1.0173、段差 0.11rad ≒ emax の実測値 0.09rad)
static void BLDC_UpdateEncoderScale(void) {
  float range = (float)(svc.max_encoder_val - svc.min_encoder_val);
  float span = TWO_PI_F - svc.encoder_dead_zone;  // ADCレンジが実際にカバーする機械角
  svc.encoder_scale = (range > 0.0f) ? (span / range) : 0.0f;
}

// 角度追従オブザーバ (2次PLL)。機械角と角速度を同時に更新する。
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
  bool saturated = (encoder_val <= svc.min_encoder_val + ENCODER_EDGE_MARGIN_LSB) ||
                   (encoder_val >= svc.max_encoder_val - ENCODER_EDGE_MARGIN_LSB);
  if (saturated) {
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

static inline void BLDC_CalculateAngularAccel(void) {
  static float pre_speed = 0;

  float accel = (svc.angular_speed - pre_speed) * (1.0f / ACCEL_CALC_DT);
  svc.angular_accel = accel * ACCEL_LPF_INV + svc.angular_accel * ACCEL_LPF;  // ローパスフィルタ
  pre_speed = svc.angular_speed;
}

// ---------------------------------------------------------------------------
// 制御器
// ---------------------------------------------------------------------------
static inline float BLDC_PIDControl(PIDController* pid, float error, float dt, bool enable_integral) {
  // 比例項
  float p_term = pid->kp * error;

  // 積分項
  if (enable_integral) {
    pid->integral += pid->ki * error * dt;
  }
  pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);

  // 微分項
  float raw_d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->d_term = pid->d_term * pid->d_lpf + raw_d_term * (1.0f - pid->d_lpf);
  pid->prev_error = error;

  // 出力の計算
  float output = p_term + pid->integral + pid->d_term;
  return Constrain(output, -pid->output_limit, pid->output_limit);
}

// 外側ループ (1kHz)。各モードに応じてq軸電流指令を作る。
static void BLDC_OuterLoop(void) {
  switch (svc.mode) {
    case BLDC_MODE_SPEED: {
      // 最大角加速度制限 (指令のスルーレート制限)
      float target = Constrain(svc.target_angular_speed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      float accel = Constrain((target - svc.speed_ramp) * (1.0f / OUTER_LOOP_DT),
                              -MAX_ANGULAR_ACCEL, MAX_ANGULAR_ACCEL);
      svc.speed_ramp += accel * OUTER_LOOP_DT;

      svc.target_iq = BLDC_PIDControl(&svc.speed_pid, svc.speed_ramp - svc.angular_speed,
                                      OUTER_LOOP_DT, true);
      break;
    }

    case BLDC_MODE_POSITION: {
      // 0と2πのまたぎ対策込みで誤差を求める
      float error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
      float abs_error = Abs(error);
      if (abs_error < POSITION_INTEGRAL_STOP_RAD) {
        svc.position_pid.integral = 0;
      }

      if (abs_error < POSITION_DEADBAND_RAD && Abs(svc.angular_speed) < POSITION_SETTLE_SPEED_RAD_S) {
        // 目標位置に整定した。無駄な電流を流さない。
        svc.position_pid.prev_error = error;
        svc.position_pid.d_term = 0;
        svc.target_iq = 0;
        break;
      }

      svc.target_iq = BLDC_PIDControl(&svc.position_pid, error, OUTER_LOOP_DT,
                                      abs_error >= POSITION_INTEGRAL_STOP_RAD);
      break;
    }

    case BLDC_MODE_TORQUE:
      svc.target_iq = svc.target_current;
      break;

    case BLDC_MODE_BRAKE:
      // 回転方向と逆向きの電流を流す。速度が落ちるほど電流も減らす。
      svc.target_iq = svc.brake_current * Constrain(svc.angular_speed * -0.05f, -1.0f, 1.0f);
      break;

    case BLDC_MODE_STOP:
    default:
      svc.target_iq = 0;
      break;
  }

  svc.target_iq = Constrain(svc.target_iq, -MAX_CURRENT, MAX_CURRENT);
  // target_id は電流ループ側で決める (1kHzのここで決めると、ステップ応答測定の
  // ステップ位置が最大1msぶれてしまう)
}

// 電流ループ (20kHz)。ADCの変換完了ごとに呼ばれる。
static void BLDC_CurrentLoop(void) {
  // --- 相電流の取得 ---
  float iu, iv, iw;
  CurrentSense_Read(&iu, &iv, &iw);

  // --- 過電流保護 ---
  // 検出が遅れないよう、フィルタをかける前の生値で判定する
  float abs_iu = Abs(iu), abs_iv = Abs(iv), abs_iw = Abs(iw);
  float peak = abs_iu;
  if (abs_iv > peak) peak = abs_iv;
  if (abs_iw > peak) peak = abs_iw;

  if (peak > svc.peak_current) svc.peak_current = peak;  // ピーク保持(調査用)

  // 制御にはノイズを抑えたフィルタ後の値を使う
  svc.iu += CURRENT_LPF_COEF * (iu - svc.iu);
  svc.iv += CURRENT_LPF_COEF * (iv - svc.iv);
  svc.iw = -(svc.iu + svc.iv);

  // 1サンプルだけのノイズで誤検出しないよう、連続して超えたときだけ保護を発動する。
  // (この基板にはハードウェアの過電流保護が無いので、短絡のような急峻な故障は
  //  どのみちソフトでは間に合わない。ここで守るのは持続的な過電流。)
  if (peak > OVERCURRENT_LIMIT) {
    if (++svc.overcurrent_count >= OVERCURRENT_TRIP_COUNT && !svc.is_overcurrent) {
      // 原因調査のため、発動した瞬間の状態を丸ごと記録する
      svc.trip.peak_current = peak;
      svc.trip.iu = iu;
      svc.trip.iv = iv;
      svc.trip.iw = iw;
      svc.trip.id = svc.id;
      svc.trip.iq = svc.iq;
      svc.trip.target_iq = svc.target_iq;
      svc.trip.vd = svc.vd;
      svc.trip.vq = svc.vq;
      svc.trip.mech_theta = svc.mech_theta;
      svc.trip.angular_speed = svc.angular_speed;
      CurrentSense_GetRaw(&svc.trip.raw_u, &svc.trip.raw_v);

      svc.is_overcurrent = true;
      svc.enable = false;
      svc.mode = BLDC_MODE_STOP;
    }
  } else {
    svc.overcurrent_count = 0;
  }

  if (!svc.enable) {
    // 出力を切ってフリーランさせる。
    // ここで全相デューティ0.5にすると3相短絡になり、回転中は制動電流が流れてしまう。
    BLDC_SetOutputEnable(false);
    svc.vd = 0;
    svc.vq = 0;
    svc.id = 0;
    svc.iq = 0;
    svc.target_iq = 0;
    svc.speed_ramp = 0;
    FOC_PI_Reset(&svc.id_pi);
    FOC_PI_Reset(&svc.iq_pi);
    svc.speed_pid.integral = 0;
    svc.position_pid.integral = 0;
    BLDC_WritePwm(0.5f, 0.5f, 0.5f);  // 再開時に中性から始まるようCCRは戻しておく
    return;
  }

  // --- 電気角 ---
  // エンコーダ校正 (BLDC_SetEncoder) により、ロータのd軸は mech_theta * POLE_PAIRS に一致する。
  //
  // ここで 0〜2π に正規化しないこと。FOC_SinCos は内部で x/2π の小数部を取るので
  // 範囲を問わない。一方 FOC_NormalizeRadians は while ループなので、
  // mech_theta × POLE_PAIRS (最大 44rad) を渡すと平均3〜4回まわり、
  // 1周につき VCMP+VMRS を余分に消費するだけの無駄になる。
  // 表示用の正規化は BLDC_GetElecTheta 側で行う。
  //
  // **角度の遅れを補償する。** AS5600の内部フィルタ(約1ms)と演算遅れ(0.075ms)で
  // 推定角は実際のロータ位置より遅れている。一定の時間遅れは回転中に
  // 「速度に比例した角度のずれ」になるので、ω̂ ぶん先に進めれば打ち消せる。
  // 補償しないと 120 rad/s で電気角が 49° ずれ、逆起電力がd軸に漏れて
  // Vd が本来の40倍出る (実測)。詳細と合わせ方は config.h の ANGLE_DELAY_S。
  float mech_theta_now = svc.mech_theta + svc.angular_speed * svc.angle_delay;
  svc.elec_theta = mech_theta_now * POLE_PAIRS + ELEC_THETA_OFFSET;
  float sin_t, cos_t;
  FOC_SinCos(svc.elec_theta, &sin_t, &cos_t);

  // --- Clarke変換 → Park変換 ---
  float i_alpha, i_beta;
  FOC_Clarke(svc.iu, svc.iv, &i_alpha, &i_beta);
  FOC_Park(i_alpha, i_beta, sin_t, cos_t, &svc.id, &svc.iq);

  // d軸指令。表面磁石型(SPM)なので弱め界磁はせず、通常は0。
  // ステップ応答測定のときだけ target_id_override が非0になる。
  svc.target_id = svc.target_id_override;

#if BLDC_MEASURING
  // ステップ応答の記録。**Idを記録してからステップを入れる**ので、
  // サンプル0はステップ直前の値 = 基準になる。
  // サンプルnの時刻は「ステップから n × 50µs 後」。
  if (svc.capture_state == BLDC_CAPTURE_ARMED) {
    svc.capture_state = BLDC_CAPTURE_RUNNING;
    svc.capture_index = 0;
  }
  if (svc.capture_state == BLDC_CAPTURE_RUNNING) {
    bldc_capture[svc.capture_index] = (int16_t)(svc.id * 1000.0f);  // [mA]
    if (++svc.capture_index >= BLDC_CAPTURE_SAMPLES) {
      svc.capture_state = BLDC_CAPTURE_DONE;
    } else if (svc.capture_index == 1) {
      // ここでステップを入れる
#if BLDC_NEED_RL
      if (svc.capture_is_voltage) {
        svc.vd_override = svc.capture_step;
      } else
#endif
      {
        svc.target_id = svc.capture_step;
        svc.target_id_override = svc.capture_step;
      }
    }
  }
#endif

  // --- 電流PI制御 ---
#if BLDC_NEED_RL
  if (svc.vd_override_active) {
    // L と R の同定中は PI を通さず Vd を直接与える (開ループ)。
    // 制御器が挟まらないので、応答はモデルの仮定が要らない素の1次系
    // (時定数 L/R) になる。閉ループのステップ応答から極を逆算するのと違い、
    // リンギングやゼロ点の影響を受けない。
    svc.vd = svc.vd_override;
    svc.vq = 0.0f;

    // 安全弁。R が想定より小さいと I = V/R が伸びるので、超えたら即座に切る。
    // 開ループなので電流を制限してくれるものが他に無い。
    if (Abs(svc.id) > MOTOR_RL_ABORT_AMPS) {
      svc.vd = 0.0f;
      svc.vd_override = 0.0f;
      svc.rl_aborted = true;
    }
  } else
#endif
  {
    svc.vd = FOC_PI_Update(&svc.id_pi, svc.target_id - svc.id, CURRENT_LOOP_DT);
    svc.vq = FOC_PI_Update(&svc.iq_pi, svc.target_iq - svc.iq, CURRENT_LOOP_DT);

#if CURRENT_FF_ENABLE
    // --- 逆起電力とdq干渉のフィードフォワード ---
    // 電圧方程式のうち速度に比例する項を先回りして足す (config.h 参照)。
    //   Vd ← −ω_e·L·Iq            (dq干渉)
    //   Vq ← +ω_e·L·Id + ω_e·ψm   (dq干渉 + 逆起電力)
    //
    // PIから見るとこれらは「速度に比例して大きくなる外乱」で、積分が誤差を
    // 見てから追いかける形になっていた。先に足しておけばPIは残差だけを相手に
    // すればよく、高速域ほど追従が良くなる。
    //
    // ω̂ はPLLの出力をそのまま使う。帯域32Hzで平滑されているのでノイズは乗らない。
    float w_e = svc.angular_speed * POLE_PAIRS;
    svc.vq_ff = w_e * (svc.motor_l * svc.id + svc.motor_psi);

    // **FFに電圧の全予算を使わせない。** FF単独で変調限界を超えると、PIは
    // 「FFの出しすぎを打ち消す」ためだけに使われ、電流を制御する権限を失う。
    // ψm を過大に見積もったときに実際にそうなり、指令+0.50Aに対し Iq が -0.24A、
    // 相電流ピークが8Aに達した。逆起電力が本当に電圧を超える速度ではモータは
    // それ以上回れないのが物理で、FFを頭打ちにしてもその物理は変わらない。
    // 変わるのは「PIが常に一定の権限を持つ」ことだけ。
    float ff_limit = svc.supply_volt * (MAX_MODULATION_RATIO * CURRENT_FF_MAX_RATIO);
    svc.vq_ff = Constrain(svc.vq_ff, -ff_limit, ff_limit);

    svc.vd -= w_e * svc.motor_l * svc.iq;
    svc.vq += svc.vq_ff;
#endif
  }

  // 電圧ベクトルを変調限界の円内に収める。
  // 制限がかかったぶんだけ積分項も縮めてワインドアップを防ぐ。
  float scale = FOC_LimitVoltageVector(&svc.vd, &svc.vq, svc.supply_volt * MAX_MODULATION_RATIO);
  if (scale < 1.0f) {
    svc.id_pi.integral *= scale;
    svc.iq_pi.integral *= scale;
  }

  // --- 逆Park変換 → SVPWM ---
  float v_alpha, v_beta;
  FOC_InvPark(svc.vd, svc.vq, sin_t, cos_t, &v_alpha, &v_beta);

  float du, dv, dw;
  FOC_SVPWM(v_alpha, v_beta, svc.supply_volt, &du, &dv, &dw);
  BLDC_WritePwm(du, dv, dw);
  BLDC_SetOutputEnable(true);  // コーストから復帰する
}

// ---------------------------------------------------------------------------
// 実行時間プロファイル
// ---------------------------------------------------------------------------
#if PROFILE_ISR
// 割り込みハンドラ全体と呼び出し周期。stm32f3xx_it.c から更新される。
Profile bldc_prof_irq;
ProfilePeriod bldc_prof_period;

// このコールバックの中身だけ。bldc_prof_irq との差がHALのディスパッチ分。
static Profile prof_callback;
#endif  // PROFILE_ISR

// 内訳の計測 (PROFILE_ISR = 2 のときだけ)。
// 無効なら ((void)0) に展開されるので、呼び出し側に #if を撒かなくてよい。
#if PROFILE_ISR >= 2
static Profile prof_encoder;  // 角度追従オブザーバ
static Profile prof_current;  // 電流ループ (Clarke/Park/PI/SVPWM)
static Profile prof_outer;    // 外側ループ (1kHzのときだけカウントされる)
#define PROF2_BEGIN(p) Profile_Begin(&(p))
#define PROF2_END(p) Profile_End(&(p))
#else
#define PROF2_BEGIN(p) ((void)0)
#define PROF2_END(p) ((void)0)
#endif

// ADC1(PWM同期の電流計測)の変換完了割り込み。これが20kHzの制御ループ本体。
// ADC2も同じコールバックを共有するのでインスタンスの判定が必要。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) return;

#if PROFILE_ISR
  Profile_Begin(&prof_callback);
#endif

  static uint16_t accel_cnt = 0;
  static uint16_t outer_cnt = 0;

  // 機械角と角速度はオブザーバが20kHzで同時に更新する (分周しない)
  PROF2_BEGIN(prof_encoder);
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);
  PROF2_END(prof_encoder);

  if (++accel_cnt >= ACCEL_CALC_DIV) {
    accel_cnt = 0;
    BLDC_CalculateAngularAccel();
  }
  if (++outer_cnt >= OUTER_LOOP_DIV) {
    outer_cnt = 0;
    PROF2_BEGIN(prof_outer);
    BLDC_OuterLoop();
    PROF2_END(prof_outer);
  }

  PROF2_BEGIN(prof_current);
  BLDC_CurrentLoop();
  PROF2_END(prof_current);

#if PROFILE_ISR
  Profile_End(&prof_callback);
#endif
}

#if PROFILE_ISR
// 実行時間の集計を1行で表示して、最大値と平均をリセットする。
// elapsed_s には前回の表示からの実経過時間を渡すこと。
//
// 見かた:
//   irq   : 割り込みハンドラ全体 (HALのディスパッチ込み) の平均/最大。
//   CPU   : 合計サイクル数 ÷ 実経過時間。想定周波数を使わないので嘘をつかない。
//   rate  : 実測した割り込み頻度。20.0kHz でなければ何かがおかしい。
//   cb    : このファイルのコールバック本体。irq - cb がHALのディスパッチ分で、
//           ここが大きいならベアメタルのハンドラに置き換える価値がある。
//   周期  : 50.00µs から外れていたらADCトリガか優先度設定を疑う。
//           max が 50µs を大きく超えていたら1回取りこぼしている。
//   max   : 外側ループが回る20回に1回が最悪ケースになるはず。
//           これが50µsに近づいたら、これ以上処理を足す余裕は無い。
// 集計を捨てて、ここから測り直す。
// 起動直後は BLDC_Init 中に溜まったぶんが最初の1回に混ざり、実測レートが
// 実際より高く出てしまう。前回の失敗 (レートの異常を見落とす) を繰り返さないよう、
// 表示用タイマを張るのと同時にこれを呼んで測定開始点を揃える。
void BLDC_ResetIsrProfile(void) {
  ProfileResult dummy;
  Profile_Snapshot(&bldc_prof_irq, &dummy, true);
  Profile_Snapshot(&prof_callback, &dummy, true);
  ProfilePeriod_Snapshot(&bldc_prof_period, &dummy, true);
#if PROFILE_ISR >= 2
  Profile_Snapshot(&prof_encoder, &dummy, true);
  Profile_Snapshot(&prof_current, &dummy, true);
  Profile_Snapshot(&prof_outer, &dummy, true);
#endif
}

void BLDC_PrintIsrProfile(float elapsed_s) {
  ProfileResult irq, cb, period;
  Profile_Snapshot(&bldc_prof_irq, &irq, true);
  Profile_Snapshot(&prof_callback, &cb, true);
  ProfilePeriod_Snapshot(&bldc_prof_period, &period, true);

  printf("[PROF] irq %5.2f/%5.2fus  CPU %4.1f%%  rate %5.2fkHz  cb %5.2fus  周期 %5.2f-%5.2fus\n",
         (double)Profile_CyclesToUs(irq.mean), (double)Profile_CyclesToUs((float)irq.max),
         (double)Profile_CpuPercent(irq.total, elapsed_s),
         (double)(Profile_RateHz(irq.count, elapsed_s) * 0.001f),
         (double)Profile_CyclesToUs(cb.mean),
         (double)Profile_CyclesToUs((float)period.min),
         (double)Profile_CyclesToUs((float)period.max));

  // センター揃えではデューティ書き込みの期限がある (config.h の ISR_DEADLINE_US)。
  // 超えると書き込みが谷を過ぎてから届き、次の「頂点」で反映されて低側ON区間が
  // 左右非対称になる。電流のサンプリング点がずれるので、症状は
  // 「なぜか電流値がおかしい」という切り分けの難しい形で出る。必ず気づけるようにする。
  float irq_max_us = Profile_CyclesToUs((float)irq.max);
  if (irq_max_us > ISR_DEADLINE_US) {
    printf("       警告 割り込みが期限 %.1fus を超えた (最大 %.2fus)。"
           "デューティの反映が1周期ずれる恐れがある\n",
           (double)ISR_DEADLINE_US, (double)irq_max_us);
  }

#if PROFILE_ISR >= 2
  ProfileResult enc, cur, out;
  Profile_Snapshot(&prof_encoder, &enc, true);
  Profile_Snapshot(&prof_current, &cur, true);
  Profile_Snapshot(&prof_outer, &out, true);

  printf("       内訳: PLL %4.2fus  電流ループ %4.2f/%4.2fus  外側 %4.2fus (%4.2fkHz)\n",
         (double)Profile_CyclesToUs(enc.mean),
         (double)Profile_CyclesToUs(cur.mean), (double)Profile_CyclesToUs((float)cur.max),
         (double)Profile_CyclesToUs(out.mean),
         (double)(Profile_RateHz(out.count, elapsed_s) * 0.001f));
#endif
}
#endif  // PROFILE_ISR

#if BLDC_MEASURING
// ---------------------------------------------------------------------------
// ステップ応答測定の共通部分
// ---------------------------------------------------------------------------
// 記録したIdの定常値 [A]。後ろ20%の平均 (過渡が完全に終わったところ)。
static float BLDC_CaptureSteadyState(void) {
  const uint16_t tail = BLDC_CAPTURE_SAMPLES / 5;
  int32_t sum = 0;
  for (uint16_t i = BLDC_CAPTURE_SAMPLES - tail; i < BLDC_CAPTURE_SAMPLES; i++) {
    sum += bldc_capture[i];
  }
  return (float)sum / (float)tail * 0.001f;
}

// 63.2%到達点をサンプル単位で返す (0 なら見つからなかった)。
//
// **またぐ2点の間を線形補間すること。** 「最初に閾値を超えたサンプル」を
// そのまま使うと時定数を切り上げる方向に読み、帯域や L を系統的に過小評価する。
// サンプリングが50µsなので、応答が速いほど誤差が効く。
// (合成波形での検証: 1000Hzの1次系を 補間なし 796Hz / 補間あり 992Hz と読んだ)
static float BLDC_CaptureTimeTo63(float i_ss, uint16_t* out_index) {
  float thr = i_ss * 0.632f;
  for (uint16_t i = 1; i < BLDC_CAPTURE_SAMPLES; i++) {
    float cur = (float)bldc_capture[i] * 0.001f;
    if (cur >= thr) {
      float prev = (float)bldc_capture[i - 1] * 0.001f;
      float d = cur - prev;
      *out_index = i;
      return (d > 1e-6f) ? ((float)(i - 1) + (thr - prev) / d) : (float)i;
    }
  }
  *out_index = 0;
  return 0.0f;
}

// 波形を表示する。立ち上がりを見たいので前半は1サンプルずつ、後半は間引く。
static void BLDC_CapturePrint(float i_ss) {
  printf("   t[ms]   Id[A]\n");
  for (uint16_t i = 0; i < BLDC_CAPTURE_SAMPLES; i += (i < 20) ? 1 : 10) {
    float v = (float)bldc_capture[i] * 0.001f;
    int bars = (i_ss > 0.001f) ? (int)(v / i_ss * 40.0f) : 0;
    if (bars < 0) bars = 0;
    if (bars > 50) bars = 50;
    printf("  %6.2f  %+6.3f |", (double)(i * CURRENT_LOOP_DT * 1000.0f), (double)v);
    for (int b = 0; b < bars; b++) printf("#");
    printf("\n");
  }
}

// ステップを1回打って記録が埋まるまで待つ。成功したら true。
static bool BLDC_RunCapture(float step, bool is_voltage) {
  svc.capture_step = step;
  svc.capture_is_voltage = is_voltage;
  svc.capture_state = BLDC_CAPTURE_ARMED;

  // 200サンプル = 10ms。余裕をみて500msで打ち切る。
  uint32_t t0 = HAL_GetTick();
  while (svc.capture_state != BLDC_CAPTURE_DONE && (HAL_GetTick() - t0) < 500) {
  }
  bool ok = (svc.capture_state == BLDC_CAPTURE_DONE);
  svc.capture_state = BLDC_CAPTURE_IDLE;
  if (!ok) {
    printf("  記録が完了しなかった (%u/%u)。制御ループが回っているか確認すること。\n",
           svc.capture_index, (unsigned)BLDC_CAPTURE_SAMPLES);
  }
  return ok;
}
#endif  // BLDC_MEASURING

#if BLDC_NEED_RL
// ---------------------------------------------------------------------------
// L と R の同定 (PIをバイパスした電圧ステップ)
// ---------------------------------------------------------------------------
// PIを通さず Vd を直接与えるので、応答は制御器の挟まらない素の1次系になる。
//   i(t) = (V/R)(1 - e^(-t/τ)),  τ = L/R
//   定常値から R = V / I_ss、63%到達から τ → L = τ·R
//
// 電圧を2点 (V と 2V) 振って差を取るのは、デッドタイムによる電圧誤差を
// 消すため。低側FETと高側FETの切り替わりに 278ns の空白があるぶん、実際に
// モータへ加わる電圧は指令より一定量小さい (12Vなら約0.067V)。この誤差は
// 電圧指令の大きさによらずほぼ一定なので、
//   R = (V2 - V1) / (I2 - I1)
// とすれば消える。1点だけで R = V/I とすると10%近い誤差が乗る。
bool BLDC_MeasureMotorRL(float* out_r, float* out_l) {
  const float v1 = MOTOR_RL_STEP_VOLTS;
  const float v2 = MOTOR_RL_STEP_VOLTS * 2.0f;
  float i1 = 0.0f, i2 = 0.0f, tau = 0.0f;

  printf("[RL] L と R の同定 (Vd を直接印加、PIはバイパス)\n");

  svc.target_id_override = 0.0f;
  svc.target_current = 0.0f;
  svc.mode = BLDC_MODE_TORQUE;
  svc.enable = true;
  HAL_Delay(200);  // 出力が立ち上がって電流が0に整定するのを待つ

  svc.rl_aborted = false;
  svc.vd_override = 0.0f;
  svc.vd_override_active = true;

  for (uint8_t pass = 0; pass < 2; pass++) {
    float v = (pass == 0) ? v1 : v2;
    if (!BLDC_RunCapture(v, true)) break;

    float i_ss = BLDC_CaptureSteadyState();
    uint16_t k;
    float k63 = BLDC_CaptureTimeTo63(i_ss, &k);
    printf("  V=%.3fV → 定常 %.3fA, 63%%到達 %.2f サンプル (%.3fms)\n",
           (double)v, (double)i_ss, (double)k63, (double)(k63 * CURRENT_LOOP_DT * 1000.0f));

    if (pass == 0) {
      i1 = i_ss;
    } else {
      i2 = i_ss;
      tau = k63 * CURRENT_LOOP_DT;  // 大きいほうがSNRが良いのでこちらのτを使う
      BLDC_CapturePrint(i_ss);
    }

    // 次の測定の前に電流を落として熱を溜めない
    svc.vd_override = 0.0f;
    HAL_Delay(100);
    if (svc.rl_aborted) break;
  }

  svc.vd_override = 0.0f;
  svc.vd_override_active = false;
  BLDC_Stop();

  if (svc.rl_aborted) {
    printf("  中断: 電流が %.1fA を超えた。MOTOR_RL_STEP_VOLTS を下げること。\n",
           (double)MOTOR_RL_ABORT_AMPS);
    return false;
  }
  if (i2 - i1 < 0.05f) {
    printf("  電流差が小さすぎて R を求められない (I1=%.3f I2=%.3f)。\n"
           "  MOTOR_RL_STEP_VOLTS を上げること。\n", (double)i1, (double)i2);
    return false;
  }

  float r = (v2 - v1) / (i2 - i1);  // デッドタイム誤差がキャンセルされる
  float r_naive = v2 / i2;          // 1点だけで出した場合 (比較用)

  printf("  → **MOTOR_R = %.4f** [Ω]  (1点だけなら %.4f。差がデッドタイム誤差)\n",
         (double)r, (double)r_naive);
  if (tau <= 0.0f) {
    printf("  τ を検出できなかった。L は求められない。\n");
    return false;
  }

  // 測定した63%到達時間には L/R 以外の遅れが2つ混ざっている。
  // 引かないと L を4割ほど過大評価する。Kp = L·ω_bw なので、そのぶん
  // 実際の帯域が設計値を超えて位相余裕が削られる。
  //
  //   (a) 電流LPFの時定数。svc.id はフィルタ後の電流から計算されている。
  //       y += a(x-y) の離散極は (1-a) なので τ = -Ts / ln(1-a)。
  //       a=0.5, Ts=50µs で 72µs。L/R の3割を超える大きさがある。
  //   (b) 電圧を出してからサンプリングするまでの半周期。
  //       CCRはプリロード付きなので書いた値は次の周期の頭から効き、電流を拾うのは
  //       低側ON区間の中央 = その周期の真ん中。よってサンプル n の実際の経過時間は
  //       (n - 0.5)·Ts であって n·Ts ではない。
  //
  // 1次系の縦続なので63%点はおよそ時定数の和になる。この差し引きの誤差は数%。
  const float tau_lpf = -CURRENT_LOOP_DT / logf(1.0f - CURRENT_LPF_COEF);
  const float tau_delay = 0.5f * CURRENT_LOOP_DT;
  float tau_lr = tau - tau_lpf - tau_delay;

  printf("  → τ(実測) %.3fms − LPF %.3fms − 半周期 %.3fms = **τ(L/R) %.3fms**\n",
         (double)(tau * 1000.0f), (double)(tau_lpf * 1000.0f),
         (double)(tau_delay * 1000.0f), (double)(tau_lr * 1000.0f));

  if (tau_lr <= 0.0f) {
    printf("  補正後のτが0以下。L/R がフィルタの時定数より速く、この方法では測れない。\n"
           "  CURRENT_LPF_COEF を大きくして測り直すこと。\n");
    return false;
  }

  float l = tau_lr * r;
  printf("  → **MOTOR_L = %.6f** [H] (= %.1f µH)  ※補正なしなら %.1f µH\n",
         (double)l, (double)(l * 1e6f), (double)(tau * r * 1e6f));
  printf("  → R/L = %.0f rad/s  (PIのゼロ点 Ki/Kp をここに合わせる)\n", (double)(r / l));
  printf("  → この値で %.0fHz にすると Kp=%.4f Ki=%.1f\n",
         (double)CURRENT_BW_HZ, (double)(l * 6.28318531f * CURRENT_BW_HZ),
         (double)(r * 6.28318531f * CURRENT_BW_HZ));
  if (tau_lr < 3.0f * CURRENT_LOOP_DT) {
    printf("  注意 補正後のτが %.1f サンプルしかない。50µsサンプリングに対して速すぎるので\n"
           "       L の値は誤差が大きい。\n", (double)(tau_lr / CURRENT_LOOP_DT));
  }
  *out_r = r;
  *out_l = l;
  return true;
}
#endif  // BLDC_NEED_RL

#if BLDC_NEED_PSI
// ---------------------------------------------------------------------------
// 逆起電力定数 ψm の同定
// ---------------------------------------------------------------------------
// 一定速度の定常状態では dIq/dt = 0 なので
//   Vq = R·Iq + ω_e·ψm     →     ψm = (Vq - R·Iq) / ω_e
//
// 2速度で測って差を取り、デッドタイムなどの一定オフセットを消す:
//   ψm = ((Vq2 - Vq1) - R(Iq2 - Iq1)) / (ω_e2 - ω_e1)
//
// 注: フィードフォワードが有効でも測定は成立する。定常状態で必要な Vq の総量は
//     ψm の真値だけで決まり、それをPIとFFのどちらが出すかは結果に影響しないため
//     (FFが足りなければPIが埋め、出しすぎればPIが引く)。

// 1点測って、平均した ω_e / Vq / Iq を返す。
static bool BLDC_MeasurePsiPoint(float target_w, float* out_we, float* out_vq, float* out_iq) {
  BLDC_AngularSpeedControl(target_w);

  // 加速度制限 (MAX_ANGULAR_ACCEL) でランプするので、到達 + 整定を待つ
  HAL_Delay(2500);

  // 500ms 平均。PWMリプルや速度のゆらぎを均す。
  const uint16_t n = 500;
  float sum_w = 0.0f, sum_vq = 0.0f, sum_iq = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    sum_w += svc.angular_speed;
    sum_vq += svc.vq;
    sum_iq += svc.iq;
    HAL_Delay(1);
  }
  float w = sum_w / (float)n;
  *out_we = w * POLE_PAIRS;
  *out_vq = sum_vq / (float)n;
  *out_iq = sum_iq / (float)n;

  printf("  %.0f rad/s 指令 → 実測 %.1f rad/s (ω_e %.0f), Vq %.3fV, Iq %.3fA\n",
         (double)target_w, (double)w, (double)*out_we, (double)*out_vq, (double)*out_iq);

  // 指令に届いていないと ω_e が想定とずれて ψm が狂う。
  if (Abs(w - target_w) > target_w * 0.2f) {
    printf("  指令速度に届いていない。負荷がかかっているか電圧が足りない。\n");
    return false;
  }
  return true;
}

bool BLDC_MeasureMotorPsi(float* out_psi) {
  printf("[PSI] 逆起電力定数 ψm の同定\n");
  printf("  **モータが回ります。** 危険なら電源を切ってください。\n");
  for (int8_t i = 3; i > 0; i--) {
    printf("  %d...\n", i);
    HAL_Delay(1000);
  }

  float we1, vq1, iq1, we2, vq2, iq2;
  bool ok = BLDC_MeasurePsiPoint(MOTOR_PSI_SPEED1, &we1, &vq1, &iq1) &&
            BLDC_MeasurePsiPoint(MOTOR_PSI_SPEED2, &we2, &vq2, &iq2);
  BLDC_Stop();

  if (!ok) return false;
  if (we2 - we1 < 1.0f) {
    printf("  2点の速度差が小さすぎる。MOTOR_PSI_SPEED1/2 を離すこと。\n");
    return false;
  }

  float psi = ((vq2 - vq1) - svc.motor_r * (iq2 - iq1)) / (we2 - we1);
  float psi_1pt = (vq2 - svc.motor_r * iq2) / we2;  // 1点だけで出した場合 (比較用)

  printf("  → **MOTOR_PSI = %.6f** [Wb]  (1点だけなら %.6f。差が一定オフセット分)\n",
         (double)psi, (double)psi_1pt);

  // **2点法は「オフセットが2点で同じ」ことが前提。** その前提が崩れていないか確かめる。
  // 各点から逆算したオフセットが揃っていなければ、消したつもりの誤差が ψm に化けている。
  //   Vq = R·Iq + ω_e·ψm + V0  →  V0 = Vq − R·Iq − ω_e·ψm
  float v0_1 = vq1 - svc.motor_r * iq1 - we1 * psi;
  float v0_2 = vq2 - svc.motor_r * iq2 - we2 * psi;
  printf("  逆算したオフセット: %.3fV / %.3fV\n", (double)v0_1, (double)v0_2);
  if (Abs(v0_1 - v0_2) > 0.05f) {
    printf("  警告 2点でオフセットが揃っていない。2点法の前提が崩れているので\n"
           "       ψm は当てにならない。MOTOR_PSI_SPEED1/2 を上げて測り直すこと\n"
           "       (速度が高いほど ω_e·ψm が大きくなり、オフセットの影響が減る)。\n");
  }

  if (psi > 1e-6f) {
    // 無負荷での到達速度の目安。実際に回してみた速度と大きく違うなら
    // どこかが間違っている (符号、極対数、電圧の見積もりなど)。
    float v_limit = svc.supply_volt * MAX_MODULATION_RATIO;
    printf("  → 逆起電力定数 %.4f V/(rad/s) [電気角]\n", (double)psi);
    printf("  → 電圧上限 %.2fV から無負荷の到達速度は約 %.0f rad/s (機械角) の見込み\n",
           (double)v_limit, (double)(v_limit / (psi * POLE_PAIRS)));
    *out_psi = psi;
    return true;
  }
  printf("  ψm が負またはゼロ。Vq の符号か POLE_PAIRS を疑うこと。\n");
  return false;
}
// ---------------------------------------------------------------------------
// 電気角の実効遅れの同定
// ---------------------------------------------------------------------------
// 推定角が真の角より δ ずれていると、逆起電力が推定d軸に漏れて Vd に現れる:
//   Vd = R·Id − ω_e·L·Iq + ω_e·ψm·sin δ
// ここから残差 δ が直接求まる:
//   sin δ = (Vd − R·Id + ω_e·L·Iq) / (ω_e·ψm)
// δ は「いまの補償値で回したときの残り」なので、真の遅れは
//   t_true = t_いま − δ / ω_e
//
// 1回の測定で決まる (反復不要)。実データでの検算:
//   補償前 t=0,    120 rad/s, Vd=−1.25V → 1.074 ms
//   補償後 t=1.08ms, 247 rad/s, Vd=+0.01V → 1.077 ms
//
// ψm が必要なので、必ず ψm の同定より後に呼ぶこと。
bool BLDC_MeasureAngleDelay(float* out_delay) {
  const float target_w = ANGLE_DELAY_MEASURE_SPEED;
  printf("[DLY] 電気角の実効遅れの同定 (%.0f rad/s で回す)\n", (double)target_w);

  float we, vq, iq;
  if (!BLDC_MeasurePsiPoint(target_w, &we, &vq, &iq)) {
    BLDC_Stop();
    return false;
  }

  // Vd と Id は BLDC_MeasurePsiPoint が拾っていないのでここで平均する。
  // (速度は既に整定しているので短くてよい)
  const uint16_t n = 300;
  float sum_vd = 0.0f, sum_id = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    sum_vd += svc.vd;
    sum_id += svc.id;
    HAL_Delay(1);
  }
  float vd = sum_vd / (float)n;
  float id = sum_id / (float)n;
  BLDC_Stop();

  float denom = we * svc.motor_psi;
  if (denom < 1e-6f) {
    printf("  ω_e·ψm が小さすぎて解けない。ψm の同定を先に済ませること。\n");
    return false;
  }

  float sin_d = (vd - svc.motor_r * id + we * svc.motor_l * iq) / denom;
  printf("  Vd %.3fV, Id %.3fA, ω_e %.0f, いまの補償 %.3fms → sinδ = %.4f\n",
         (double)vd, (double)id, (double)we, (double)(svc.angle_delay * 1000.0f),
         (double)sin_d);

  if (sin_d > 0.9f || sin_d < -0.9f) {
    printf("  δ が90°に近く、この式では精度が出ない。\n"
           "  ANGLE_DELAY_MEASURE_SPEED を下げるか、既定値を実際に近づけること。\n");
    return false;
  }

  float delta = asinf(sin_d);
  float delay = svc.angle_delay - delta / we;
  printf("  → δ = %+.1f° → **ANGLE_DELAY = %.5f** [s] (= %.3f ms)\n",
         (double)(delta * 57.29578f), (double)delay, (double)(delay * 1000.0f));

  // 負や極端に大きい値は測定失敗。既定値のほうがまだ安全。
  if (!(delay > 0.0f && delay < 0.01f)) {
    printf("  値が範囲外。採用しない。\n");
    return false;
  }
  *out_delay = delay;
  return true;
}
#endif  // BLDC_NEED_PSI

#if MEASURE_CURRENT_STEP
// ---------------------------------------------------------------------------
// 電流ループ(閉ループ)のステップ応答測定
// ---------------------------------------------------------------------------
// d軸に電流ステップを入れて Id の応答を記録し、帯域を求める。
//
// なぜ d軸なのか:
//   d軸はロータの磁束方向なので、電流を流してもトルクが出ない。ロータが動かない
//   ので逆起電力も負荷変動も混ざらず、「電流ループの電気的な応答」だけが取れる。
//   表面磁石型(SPM)は Ld = Lq でPIゲインも共通なので、結果はq軸にそのまま使える。
//
// なぜ測るのか:
//   Kp を上げてよいかは L 次第で、L が分からないと判断できない。
//   一方 Kp と Ki を同じ倍率で動かせば帯域は倍率どおりに動く (config.h 参照)。
//   つまり必要なのは「いまの帯域が何Hzか」という1つの数字だけ。
static void BLDC_AnalyzeCurrentStep(float step_amps) {
  float i_ss = BLDC_CaptureSteadyState();

  printf("[STEP] 電流ループのステップ応答 (Id指令 %.2fA)\n", (double)step_amps);

  // 定常値が指令から大きく外れていたら、そもそも電流が流せていない。
  // (電源電圧不足、出力が有効になっていない、過電流でラッチ、など)
  if (i_ss < step_amps * 0.5f) {
    printf("  定常値 %.3fA が指令 %.2fA に届いていない。測定は無効。\n"
           "  電源電圧・過電流ラッチ・エンコーダ校正を確認すること。\n",
           (double)i_ss, (double)step_amps);
    return;
  }

  uint16_t k63;
  float k63f = BLDC_CaptureTimeTo63(i_ss, &k63);

  // ピーク → オーバーシュート。15%を超えていたらゲインが高すぎる。
  int16_t peak = bldc_capture[0];
  for (uint16_t i = 0; i < BLDC_CAPTURE_SAMPLES; i++) {
    if (bldc_capture[i] > peak) peak = bldc_capture[i];
  }
  float overshoot = ((float)peak * 0.001f - i_ss) / i_ss * 100.0f;

  printf("  定常値 %.3fA  63%%到達 %.2f サンプル (%.3fms)  オーバーシュート %.1f%%\n",
         (double)i_ss, (double)k63f, (double)(k63f * CURRENT_LOOP_DT * 1000.0f),
         (double)overshoot);

  if (k63 == 0 || k63f <= 0.0f) {
    printf("  1サンプル(50µs)以内に立ち上がっている。帯域は測定限界の外。\n");
    BLDC_CapturePrint(i_ss);
    return;
  }

  float tau = k63f * CURRENT_LOOP_DT;

  // **本当に1次系か検証する。** 1次系なら 2τ で 86.5%、3τ で 95.0% のはず。
  // 極とゼロが打ち消せていないと応答が速い成分と遅い成分に割れ、
  // 「速く立ち上がったあと長い尾を引く」形になってここが合わなくなる。
  // 63%点だけ見て時定数を語ると、その尾を見落として帯域を過大評価する。
  // **ここも線形補間すること。** τ は整数サンプルとは限らない (63%点自体を
  // 補間で求めているので当然)。インデックスを切り捨てると交差点の手前の値を
  // 拾ってしまい、1τ が定義上63%のはずなのに51%と出るような矛盾が起きる。
  // しかも誤差の大きさが τ の小数部に依存するので、同じ応答でも通ったり
  // 落ちたりする。判定として成立しない。
  const float frac[3] = {0.632f, 0.865f, 0.950f};
  float dev[3] = {0.0f, 0.0f, 0.0f};
  printf("  1次系との比較:");
  for (uint8_t m = 1; m <= 3; m++) {
    float fidx = k63f * (float)m;
    uint16_t i0 = (uint16_t)fidx;
    if (i0 > BLDC_CAPTURE_SAMPLES - 2) i0 = BLDC_CAPTURE_SAMPLES - 2;
    float f = fidx - (float)i0;
    float v0 = (float)bldc_capture[i0] * 0.001f;
    float v1 = (float)bldc_capture[i0 + 1] * 0.001f;
    float actual = (v0 + f * (v1 - v0)) / i_ss;
    dev[m - 1] = (actual - frac[m - 1]) * 100.0f;
    printf("  %uτ %.0f%%(理想%.0f%%)", m, (double)(actual * 100.0f),
           (double)(frac[m - 1] * 100.0f));
  }
  printf("\n");

  // 判定の主役は **3τ**。
  //
  // 極とゼロが合っていないと応答が速い成分と遅い成分に割れ、3τ になっても
  // 95% に届かない「長い尾」が残る。これが直したい状態。
  // 一方 2τ が理想より**高め**に出るのは、ループの中の遅れ (電流LPFと演算遅れ)
  // のせいで立ち上がりが後ろにずれ、そのぶん τ を長めに読むため。
  // 遅れは避けられないので、こちらは異常ではない。
  //
  // 実測での切り分け:
  //   壊れていたとき (Kp=0.75): 2τ +17%, **3τ -13%**  ← 尾が残っている
  //   直したあと            : 2τ  +9%, **3τ  +4%**  ← 尾は無い
  if (dev[2] < -8.0f) {
    printf("  **遅い尾が残っている (3τ で %+.0f%%)。** PIのゼロ点 Ki/Kp が\n"
           "  プラントの極 R/L と合っていないと、応答が速い成分と遅い成分に割れる。\n"
           "  MEASURE_MOTOR_RL で L と R を実測して MOTOR_L / MOTOR_R を直すこと。\n"
           "  この状態では下の帯域の値は当てにならない。\n", (double)dev[2]);
  } else if (dev[1] > 20.0f) {
    printf("  **応答が1次系から外れている (2τ で %+.0f%%)。**\n"
           "  ゲインが高すぎて振動しているか、ループ内の遅れが想定より大きい。\n", (double)dev[1]);
  } else {
    // L の同定と同じく、電圧を出してからサンプリングするまでの半周期を引く。
    // CCRはプリロード付きなので書いた値は次の周期の頭から効き、電流を拾うのは
    // 低側ON区間の中央。よってサンプル n の実際の経過時間は (n - 0.5)·Ts。
    // 引かないと帯域を1割ほど低く読む。
    float tau_corr = tau - 0.5f * CURRENT_LOOP_DT;
    if (tau_corr <= 0.0f) tau_corr = tau;
    float f_bw = 1.0f / (TWO_PI_F * tau_corr);
    printf("  → 1次系とみなせる。**帯域 約%.0fHz** (狙い %.0fHz, 補正前 %.0fHz)\n",
           (double)f_bw, (double)CURRENT_BW_TARGET_HZ, (double)(1.0f / (TWO_PI_F * tau)));
    // 比較対象は CURRENT_BW_HZ ではなく CURRENT_BW_TARGET_HZ。
    // 設計パラメータ自身を目標にすると、実測がそこに届くたびにさらに高い値を
    // 要求する追いかけっこになる (config.h 参照)。
    printf("  → 狙いに合わせるなら CURRENT_BW_HZ = %.0f (いまは %.0f)\n",
           (double)(CURRENT_BW_HZ * CURRENT_BW_TARGET_HZ / f_bw), (double)CURRENT_BW_HZ);
  }

  if (k63 < 3) {
    printf("  注意 63%%到達が %u サンプルしかない。50µsサンプリングに対して速すぎるので\n"
           "       帯域の値は誤差が大きい。\n", k63);
  }
  if (overshoot > 15.0f) {
    printf("  警告 オーバーシュートが大きい。CURRENT_BW_HZ を下げること。\n");
  }

  BLDC_CapturePrint(i_ss);
}

// 注意: これを呼ぶとモータに電流が流れる。d軸なのでトルクは出ない**はず**だが、
// エンコーダ校正がずれているとトルクが出てロータが動く。校正が済んでいることと、
// ロータが自由に回っても安全な状態であることを確認してから使うこと。
void BLDC_MeasureCurrentStep(void) {
  const float step_amps = Constrain(CURRENT_STEP_AMPS, 0.0f, MAX_CURRENT);

  // q軸は0のまま、d軸だけを動かす。トルク指令0のトルクモードにする。
  svc.target_id_override = 0.0f;
  svc.target_current = 0.0f;
  svc.mode = BLDC_MODE_TORQUE;
  svc.enable = true;
  HAL_Delay(200);  // 出力が立ち上がって電流が0に整定するのを待つ

  bool ok = BLDC_RunCapture(step_amps, false);

  // 必ず電流を切ってから戻る
  svc.target_id_override = 0.0f;
  BLDC_Stop();

  if (ok) BLDC_AnalyzeCurrentStep(step_amps);
}
#endif  // MEASURE_CURRENT_STEP

// ---------------------------------------------------------------------------
// 初期化
// ---------------------------------------------------------------------------

// エンコーダの最大/最小値と電気角のオフセットを実測してフラッシュに保存する
static void BLDC_SetEncoder(volatile uint16_t* encoder_val) {
  svc.encoder_offset_theta = 0;  // オフセット計測前は0で初期化
  svc.max_encoder_val = 0;
  svc.min_encoder_val = MAX_ADC_VAL;

  // エンコーダー出力の最大値・最小値を取得する
  float phase = 0;
  for (uint16_t i = 0; i < 3000; i++) {
    phase += 0.2f;
    if (svc.max_encoder_val < *encoder_val) svc.max_encoder_val = *encoder_val;
    if (svc.min_encoder_val > *encoder_val) svc.min_encoder_val = *encoder_val;
    BLDC_OpenLoopDrive(0.15f, phase);
    HAL_Delay(1);
  }

  // 盲点(レール飽和で角度が読めない区間)の幅を測る。
  // 等速で回しながら「レール付近に張り付いていたサンプルの割合」を数えると、
  // 等速なので時間の割合＝機械角の割合になり、そのまま盲点の角度幅が求まる。
  //   ADC値 [min, max] が実際にカバーするのは 2π ではなく 2π - 盲点幅。
  //   この差を無視すると θ_meas が引き伸ばされ、継ぎ目に段差ができる
  //   (BLDC_UpdateEncoderScale のコメント参照)。
  // 上と同じ速度・同じ駆動条件で回すこと。3000ms で約13回転するので、
  // 半端な回転ぶんの誤差は 1/13 程度に収まる。
  uint32_t sat_samples = 0;
  for (uint16_t i = 0; i < 3000; i++) {
    phase += 0.2f;
    uint16_t v = *encoder_val;
    if (v <= svc.min_encoder_val + ENCODER_EDGE_MARGIN_LSB ||
        v >= svc.max_encoder_val - ENCODER_EDGE_MARGIN_LSB) {
      sat_samples++;
    }
    BLDC_OpenLoopDrive(0.15f, phase);
    HAL_Delay(1);
  }
  svc.encoder_dead_zone = TWO_PI_F * (float)sat_samples / 3000.0f;

  // 測れなかった/明らかにおかしい場合は補正なし(従来どおり)に落とす。
  // 盲点が1周の1/4もあるようならエンコーダか磁石の取り付けを疑うべき。
  if (!(svc.encoder_dead_zone >= 0.0f && svc.encoder_dead_zone < TWO_PI_F * 0.25f)) {
    svc.encoder_dead_zone = 0.0f;
  }

  BLDC_UpdateEncoderScale();  // これ以降 BLDC_UpdateEncoder が使えるようになる

  // 電気角度0の位置にロータを引き込んで、そのときの機械角をオフセットとする。
  // これを電気角1回転ぶんずつ位置をずらしながら POLE_PAIRS 回くり返して平均する。
  //
  // 測定点は機械角で 2π/POLE_PAIRS ずつ離れているが、そのまま平均してよい。
  //   測定値 v_k = (θ0 + k・2π/P) mod 2π   (k = 0..P-1)
  //   Σv_k = P・θ0 + (2π/P)・P(P-1)/2 - 2π・M   (M: 2πを跨いだ回数)
  //   平均 = θ0 + (2π/P)・((P-1)/2 - M)
  // つまり平均は θ0 と 2π/P の整数倍しか違わない。電気角は機械角を P 倍して
  // 2π で折り返すので、2π/P のずれは電気角では 2π のずれ = 同一。よって等価。
  // ただしこの相殺は「各測定値が等間隔かつ同じ誤差を持つ」ことが前提なので、
  // 特定の周回だけ誤差が乗ると成り立たなくなる (下の encoder_primed 参照)。
  float offset_sum = 0;
  for (uint8_t i = 0; i < POLE_PAIRS; i++) {
    BLDC_OpenLoopDrive(0.15f, 0);
    HAL_Delay(200);
    BLDC_OpenLoopDrive(0.3f, 0);
    HAL_Delay(100);

    // 直前の位相送りでロータは 2π/POLE_PAIRS ≒ 0.9rad 動いている。
    // オブザーバは CURRENT_LOOP_DT (50µs) 前提のゲインなのに、この校正ループは
    // HAL_Delay(1) で回るため実時間では20倍ゆっくりしか追従しない。さらに
    // 0.9rad はイノベーション上限を超えるので観測が棄却され続けてしまう。
    // 張り直さずに測ると追従中のランプが平均に入り、オフセットが
    // 0.2rad(機械角) ≒ 70°(電気角) もずれる。
    svc.encoder_primed = false;
    BLDC_UpdateEncoder(*encoder_val);

    // 整定位置がたまたま 0/2π の境目にあっても平均が壊れないよう、
    // 1点目からの差分 (-π〜+π に正規化済み) で平均する。
    float base = svc.mech_theta;
    float diff_sum = 0;
    for (uint16_t j = 0; j < 200; j++) {
      BLDC_UpdateEncoder(*encoder_val);
      diff_sum += FOC_AngleDiff(svc.mech_theta, base);
      HAL_Delay(1);
    }
    offset_sum += FOC_NormalizeRadians(base + diff_sum * 0.005f);

    // 電気角をおよそ1回転ぶん送って、次の測定点までロータを進める。
    // 6.2rad で止めても、次のループ頭で phase=0 (≡2π) に引き込まれるので
    // 結果として正確に電気角1回転ぶん進む。
    phase = 0;
    for (uint16_t j = 0; j < (uint16_t)(TWO_PI * 10); j++) {
      phase += 0.1f;
      BLDC_OpenLoopDrive(0.15f, phase);
      HAL_Delay(1);
    }
  }
  svc.encoder_offset_theta = offset_sum / POLE_PAIRS;

  printf("(Measure)max: %lu, min: %lu, offset: %.6f rad, dead_zone: %.4f rad (%.2f°, 1周の%.1f%%)\n",
         (unsigned long)svc.max_encoder_val,
         (unsigned long)svc.min_encoder_val,
         svc.encoder_offset_theta,
         svc.encoder_dead_zone,
         (double)(svc.encoder_dead_zone * 57.29578f),
         (double)(svc.encoder_dead_zone * 100.0f / TWO_PI_F));

  // フラッシュ書き込みはここではしない。
  // モータ定数の測定が終わってから、全部まとめて1回で書く (BLDC_SaveCalibration)。
  // フラッシュはページ単位でしか消せないので、分けて書くと先に書いたほうが消える。
}

#if MOTOR_AUTO_CALIBRATION
// 校正値をまとめてフラッシュに保存する。
// **1回で全項目を書くこと。** Flash_WriteData はページ消去を伴うので、
// 項目ごとに呼ぶと前に書いたものが消える。
static void BLDC_SaveCalibration(void) {
  BLDCFlashData d = {BLDC_FLASH_MAGIC,
                     svc.max_encoder_val,
                     svc.min_encoder_val,
                     svc.encoder_offset_theta,
                     svc.encoder_dead_zone,
                     svc.motor_r,
                     svc.motor_l,
                     svc.motor_psi,
                     svc.angle_delay};
  if (Flash_WriteData(FLASH_USER_START_ADDR, &d, sizeof(d)) == HAL_OK) {
    printf("[CAL] フラッシュに保存した\n");
  } else {
    printf("[CAL] フラッシュ書き込みに失敗した\n");
  }
}
#endif  // MOTOR_AUTO_CALIBRATION

// 電流PIのゲインを、いまの L と R から計算し直す。
// L と R を実測したあとに必ず呼ぶこと。
static void BLDC_UpdateCurrentGains(void) {
  svc.id_pi.kp = CURRENT_PI_KP_FROM(svc.motor_l);
  svc.id_pi.ki = CURRENT_PI_KI_FROM(svc.motor_r);
  svc.id_pi.output_limit = svc.supply_volt * MAX_MODULATION_RATIO;
  FOC_PI_Reset(&svc.id_pi);
  svc.iq_pi = svc.id_pi;
  printf("[CAL] 電流PI: Kp=%.4f Ki=%.1f (Ki/Kp=%.0f = R/L, 目標帯域 %.0fHz)\n",
         (double)svc.id_pi.kp, (double)svc.id_pi.ki,
         (double)(svc.motor_r / svc.motor_l), (double)CURRENT_BW_HZ);
}

#if MOTOR_AUTO_CALIBRATION
// モータ定数の自動測定。制御ループが回り始めてから呼ぶこと。
//
// 順番に意味がある:
//   1. R,L → PIをバイパスして測るのでゲイン不要。ここで測ってゲインを決める
//   2. ψm  → 速度制御で回すのでゲインが要る。1の後
//   3. 遅れ → Vd から求めるので ψm が要る。2の後
//
// どれか失敗しても、そこまでに取れた値は活かして続ける
// (失敗した項目は既定値かフラッシュの旧値のまま)。
static void BLDC_CalibrateMotor(void) {
  float r, l, psi, delay;

  printf("[CAL] モータ定数の自動測定を開始\n");

  if (BLDC_MeasureMotorRL(&r, &l)) {
    svc.motor_r = r;
    svc.motor_l = l;
    BLDC_UpdateCurrentGains();  // 以降の測定はこのゲインで走る
  } else {
    printf("[CAL] R,L の測定に失敗。既定値のまま続行する\n");
  }

  if (BLDC_MeasureMotorPsi(&psi)) {
    svc.motor_psi = psi;
  } else {
    printf("[CAL] ψm の測定に失敗。既定値のまま続行する\n");
  }

  if (BLDC_MeasureAngleDelay(&delay)) {
    svc.angle_delay = delay;
  } else {
    printf("[CAL] 角度遅れの測定に失敗。既定値のまま続行する\n");
  }

  printf("[CAL] 結果: R=%.4fΩ L=%.6fH(%.1fµH) ψm=%.6fWb 遅れ=%.3fms\n",
         (double)svc.motor_r, (double)svc.motor_l, (double)(svc.motor_l * 1e6f),
         (double)svc.motor_psi, (double)(svc.angle_delay * 1000.0f));
}
#endif  // MOTOR_AUTO_CALIBRATION

void BLDC_Init(bool do_set_encoder, volatile uint16_t* encoder_val, float supply_volt) {
  printf("BLDC_Init\n");

  svc.encoder_val_ptr = encoder_val;
  svc.mode = BLDC_MODE_STOP;
  svc.enable = false;
  // **母線電圧は最初に確定させること。**
  // SVPWM は「電圧指令 → デューティ」の変換にこの値を使うので、実際の母線と
  // 食い違っていると、指令した電圧と実際に加わる電圧の比がそのままずれる。
  //
  // 以前は 12.0V の暫定値のまま校正まで走っていた。実測が 8.6V だったので、
  // 実際に加わる電圧は指令の 8.6/12 = 0.717 倍。電流ループは閉じているので
  // PIが 1/0.717 = 1.395 倍の電圧を指令して埋め合わせ、その水増しされた Vq から
  // ψm を計算していた (実測 0.002663 に対し真値 0.001908)。
  //
  // R と L は同じ倍率で狂っても、ゲイン (Kp=L·ω, Ki=R·ω) の水増しと印加電圧の
  // 目減りが打ち消し合うので実害が出ず、**ψm だけが表に出た**。
  // 測定時と使用時で母線の扱いが違う量は、この手の誤差が打ち消されない。
  BLDC_SetSupplyVolt(supply_volt);

  // TIM1 をセンター揃え (アップダウンカウント) に切り替える。
  //
  // エッジ揃えだと低側FETのON区間 [CCR, ARR] の**後ろ側しか使えず**、
  // サンプリング点より手前のデューティしか出せなかった (MAX_DUTY = 0.80 →
  // 変調率 0.346 で、SVPWMの理論上限 0.577 の6割しか母線電圧を使えていなかった)。
  // センター揃えなら低側ON区間がカウンタ頂点を中心に左右へ分散するので、
  // 同じ絶対時間の余裕で 0.88 まで引ける (変調率 0.439, +27%)。
  // おまけに3相の立ち上がりが同時でなくなるのでリプルとEMIも下がる。
  //
  // tim.c は CubeMX の再生成で上書きされるので、BDTR の OSSI と同じくここで設定する。
  // (current_sense.c が ADC1 を再初期化しているのと同じ考え方)
  // PwmOut_Init = HAL_TIM_PWM_Start より前に呼ぶこと。
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_ARR;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    printf("TIM1 センター揃えへの再初期化に失敗\n");
  }

  // TIM1のPWM出力を開始
  PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
  PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
  PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);

  // MOE=0 のときに出力ピンをHi-Zではなくアイドルレベル(Low)に固定する。
  // CubeMXの既定 (TIM_OSSI_DISABLE) のままだとピンが浮いてゲートドライバの入力が
  // 不定になり、コーストにならない。tim.c は再生成で上書きされるのでここで設定する。
  TIM1->BDTR |= TIM_BDTR_OSSI;

  // 電流センシング(TIM1同期ADC)を開始。TIM1が動いてから呼ぶ必要がある。
  CurrentSense_Init();

  // --- モータ定数の初期値 ---
  // フラッシュに有効な値があれば下で上書きされる。校正するときも、R,L を測るまでは
  // この値で電流ループを回す必要があるので、先に入れておく。
  svc.motor_r = MOTOR_R_DEFAULT;
  svc.motor_l = MOTOR_L_DEFAULT;
  svc.motor_psi = MOTOR_PSI_DEFAULT;
  svc.angle_delay = ANGLE_DELAY_DEFAULT;

  // エンコーダ校正 (スイッチを押しながら起動したときだけ)。
  // フラッシュ書き込みは、この後のモータ定数測定まで終わってから1回でまとめて行う。
  if (do_set_encoder) {
    printf("BLDC_SetEncoder\n");
    BLDC_SetEncoder(encoder_val);
  }

  if (do_set_encoder) {
    // 今まさに測った値を使う。フラッシュの古い値で上書きしないこと。
    printf("(Measured) このセッションの校正値を使う\n");
  } else {
    // フラッシュから読み込み
    BLDCFlashData d;
    Flash_ReadData(FLASH_USER_START_ADDR, &d, sizeof(d));

    // 未校正のフラッシュ(消去状態は全ビット1)や旧フォーマットのデータを読むと、
    // max == min による0除算や、盲点幅にゴミが入って角度が狂う。マジックナンバーと
    // 範囲チェックの両方で弾く。NaN も落とせるよう不等号は肯定形で書くこと。
    bool enc_ok = (d.magic == BLDC_FLASH_MAGIC) &&
                  (d.max_encoder_val > d.min_encoder_val) && (d.max_encoder_val <= MAX_ADC_VAL) &&
                  (d.encoder_offset_theta > -TWO_PI_F && d.encoder_offset_theta < TWO_PI_F) &&
                  (d.encoder_dead_zone >= 0.0f && d.encoder_dead_zone < TWO_PI_F * 0.25f);

    // モータ定数は別に検証する。エンコーダ側が壊れていてもこちらは使えることがあるが、
    // 逆に「もっともらしいゴミ」を掴むと制御が静かに劣化するので範囲を厳しめに見る。
    bool motor_ok = (d.magic == BLDC_FLASH_MAGIC) &&
                    (d.motor_r > 0.001f && d.motor_r < 100.0f) &&
                    (d.motor_l > 1e-6f && d.motor_l < 0.1f) &&
                    (d.motor_psi > 1e-5f && d.motor_psi < 1.0f) &&
                    (d.angle_delay >= 0.0f && d.angle_delay < 0.01f);

    if (enc_ok) {
      svc.max_encoder_val = d.max_encoder_val;
      svc.min_encoder_val = d.min_encoder_val;
      svc.encoder_offset_theta = d.encoder_offset_theta;
      svc.encoder_dead_zone = d.encoder_dead_zone;
      printf("(From Flash)max: %lu, min: %lu, offset: %.6f rad, dead_zone: %.4f rad\n",
             (unsigned long)d.max_encoder_val, (unsigned long)d.min_encoder_val,
             d.encoder_offset_theta, d.encoder_dead_zone);
    } else {
      printf("BLDC: エンコーダ校正値が不正です。スイッチを押しながら再起動して校正してください。\n");
      svc.min_encoder_val = 0;
      svc.max_encoder_val = MAX_ADC_VAL;
      svc.encoder_offset_theta = 0.0f;
      svc.encoder_dead_zone = 0.0f;
    }

    if (motor_ok) {
      svc.motor_r = d.motor_r;
      svc.motor_l = d.motor_l;
      svc.motor_psi = d.motor_psi;
      svc.angle_delay = d.angle_delay;
      printf("(From Flash)R: %.4fΩ, L: %.1fµH, ψm: %.6fWb, 遅れ: %.3fms\n",
             d.motor_r, (double)(d.motor_l * 1e6f), d.motor_psi,
             (double)(d.angle_delay * 1000.0f));
    } else {
      printf("BLDC: モータ定数が未校正です。config.h の既定値を使います。\n");
    }
  }
  BLDC_UpdateEncoderScale();

  // --- 制御器のゲイン ---
  // 外側ループの出力は「Iq指令 [A]」。電圧制御だった頃とは単位が違うので再調整が必要。
  svc.speed_pid.kp = 0.1f;
  svc.speed_pid.ki = 0.2f;
  svc.speed_pid.kd = 0;
  svc.speed_pid.d_term = 0;
  svc.speed_pid.d_lpf = 0.0f;
  svc.speed_pid.output_limit = MAX_CURRENT;

  svc.position_pid.kp = 7.5;
  svc.position_pid.ki = 15;
  svc.position_pid.kd = 0.2f;
  svc.position_pid.d_term = 0;
  svc.position_pid.d_lpf = 0.8f;
  svc.position_pid.output_limit = MAX_CURRENT;

  // 電流PIの出力は電圧 [V]。上限は変調限界。ゲインは L と R から導く。
  BLDC_UpdateCurrentGains();

  // 電流センサのゼロ点校正。全相デューティ0.5で電流が流れない状態にして行う。
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);
  HAL_Delay(200);
  CurrentSense_Calibrate();

  // 制御ループを回す前に、機械角をいまのエンコーダ実測値で確定させておく。
  // (フィルタで0から実角度まで移動する間、電気角が高速に回ってしまうのを防ぐ)
  svc.encoder_primed = false;
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);
  printf("BLDC: mech_theta = %.3f rad で開始\n", svc.mech_theta);

  // 指令が来るまでは出力を切っておく (enable = false と状態を合わせる)。
  // 電流センサのゼロ点校正まではPWMを出しておく必要があるので、ここまで来てから切る。
  BLDC_SetOutputEnable(false);

#if PROFILE_ISR
  // 制御ループが回り出す前にサイクルカウンタを動かしておく。
  // (lib/timer の Timer_Init も同じことをするが、あちらが呼ばれるのは
  //  BLDC_Init より後なので、ここで自前で有効化する)
  Profile_EnableDWT();
  if (!Profile_IsDWTRunning()) {
    printf("BLDC: 警告 DWT->CYCCNT が動いていません。プロファイル値は無効です。\n");
  } else {
    // 計測自身のコストを実測して表示する。この値を知らないと、
    // 出てきた数字のうちどこまでが本当の処理時間か判断できない。
    uint32_t ov = Profile_MeasureOverhead();
    printf("BLDC: プロファイル有効 (PROFILE_ISR=%d) 計測オーバーヘッド %lu cycle/区間 (%.3fus)\n",
           PROFILE_ISR, (unsigned long)ov, (double)Profile_CyclesToUs((float)ov));
  }
#endif

  // ここまで来たら20kHzの制御ループ(ADC変換完了割り込み)を回し始める
  CurrentSense_EnableInterrupt();
  printf("BLDC control loop start (%.0f Hz)\n", (double)PWM_FREQ);

  // 測定モード。制御ループが回っていないと測れないのでここ。
  // 測り終わったら config.h の該当フラグを 0 に戻すこと。
#if MOTOR_AUTO_CALIBRATION
  // スイッチを押しながら起動したときだけ、モータ定数も測ってまとめて保存する。
  // ここで呼ぶのは、R,L,ψm の測定に制御ループ(ADC割り込み)が必要なため。
  if (do_set_encoder) {
    BLDC_CalibrateMotor();
    BLDC_SaveCalibration();
  }
#endif

  // --- 起動時の診断表示 (校正とは別。config.h の MEASURE_* で個別に有効化) ---
#if MEASURE_MOTOR_RL
  {
    float r, l;
    BLDC_MeasureMotorRL(&r, &l);
  }
#endif
#if MEASURE_CURRENT_STEP
  BLDC_MeasureCurrentStep();
#endif
#if MEASURE_MOTOR_PSI
  {
    float psi;
    BLDC_MeasureMotorPsi(&psi);
  }
#endif
}

// ---------------------------------------------------------------------------
// app から呼ぶAPI
// ---------------------------------------------------------------------------
void BLDC_SetSupplyVolt(float supply_volt) {
  if (supply_volt < 1.0f) supply_volt = 1.0f;  // 0除算防止
  svc.supply_volt = supply_volt;

  float v_limit = supply_volt * MAX_MODULATION_RATIO;
  svc.id_pi.output_limit = v_limit;
  svc.iq_pi.output_limit = v_limit;
}

void BLDC_Stop(void) {
  svc.mode = BLDC_MODE_STOP;
  svc.enable = false;
}

// 過電流保護がラッチされている間は起動させない
static inline void BLDC_Enable(BLDCMode mode) {
  if (svc.is_overcurrent) return;
  svc.mode = mode;
  svc.enable = true;
}

void BLDC_AngularSpeedControl(float target_angular_speed) {
  svc.target_angular_speed = target_angular_speed;
  BLDC_Enable(BLDC_MODE_SPEED);
}

void BLDC_PositionControl(float target_position) {
  svc.target_position = target_position;
  BLDC_Enable(BLDC_MODE_POSITION);
}

void BLDC_TorqueControl(float target_current) {
  svc.target_current = Constrain(target_current, -MAX_CURRENT, MAX_CURRENT);
  BLDC_Enable(BLDC_MODE_TORQUE);
}

void BLDC_BrakeControl(float brake_current) {
  svc.brake_current = Constrain(brake_current, 0.0f, MAX_CURRENT);
  BLDC_Enable(BLDC_MODE_BRAKE);
}

float BLDC_GetMechTheta(void) { return svc.mech_theta; }

// 割り込みの中では電気角を正規化していない (FOC_SinCos が範囲を問わないため)。
// 表示・シリアル送信用にここで 0〜2π に畳む。
float BLDC_GetElecTheta(void) { return FOC_NormalizeRadians(svc.elec_theta); }
float BLDC_GetAngularSpeed(void) { return svc.angular_speed; }
float BLDC_GetAngularAccel(void) { return svc.angular_accel; }
float BLDC_GetId(void) { return svc.id; }
float BLDC_GetIq(void) { return svc.iq; }
float BLDC_GetTargetIq(void) { return svc.target_iq; }
float BLDC_GetVd(void) { return svc.vd; }
float BLDC_GetVq(void) { return svc.vq; }

// Vq のうちフィードフォワードで作った分。
// FFが正しく効いていれば、速度が上がるほど Vq に占めるこの割合が大きくなり、
// PIは残差だけを相手にする。**符号が逆だと Vq と逆向きに出る**ので、
// 実装ミスの確認にはこれを見るのが一番早い。
float BLDC_GetVqFF(void) { return svc.vq_ff; }
bool BLDC_IsOvercurrent(void) { return svc.is_overcurrent; }
void BLDC_GetTripInfo(BLDCTripInfo* info) { *info = svc.trip; }

void BLDC_ClearOvercurrent(void) {
  svc.overcurrent_count = 0;
  svc.is_overcurrent = false;
}

float BLDC_GetPeakCurrent(void) { return svc.peak_current; }
void BLDC_ResetPeakCurrent(void) { svc.peak_current = 0.0f; }

void BLDC_GetEncoderStats(BLDCEncoderStats* stats) {
  stats->saturated = svc.saturated_total;
  stats->glitch = svc.glitch_total;
  stats->max_innovation = svc.max_innovation;
}

void BLDC_ResetEncoderStats(void) {
  svc.saturated_total = 0;
  svc.glitch_total = 0;
  svc.max_innovation = 0.0f;
}
