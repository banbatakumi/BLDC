#include "bldc.h"

typedef struct {
  // --- 設定 ---
  volatile uint16_t* encoder_val_ptr;  // ADC2のDMAが更新するエンコーダ値へのポインタ
  uint16_t max_encoder_val;            // エンコーダーの最大値
  uint16_t min_encoder_val;            // エンコーダーの最小値
  float encoder_dead_zone;             // レール飽和で角度が読めない区間の幅 [rad]
  float encoder_scale;                 // (2π - 盲点) / (max - min)。事前計算して割り算を避ける
  float encoder_offset_theta;          // エンコーダーのオフセット値 [rad]
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
  float target_id, target_iq;
  float speed_ramp;  // 加速度制限をかけた角速度指令 [rad/s]

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
static inline void BLDC_WritePwm(float u, float v, float w) {
  TIM1->CCR1 = (uint32_t)(Constrain(u, MIN_DUTY, MAX_DUTY) * PWM_ARR);
  TIM1->CCR2 = (uint32_t)(Constrain(v, MIN_DUTY, MAX_DUTY) * PWM_ARR);
  TIM1->CCR3 = (uint32_t)(Constrain(w, MIN_DUTY, MAX_DUTY) * PWM_ARR);
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
  svc.target_id = 0.0f;  // 表面磁石型(SPM)なので弱め界磁はしない
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
  // エンコーダ校正 (BLDC_SetEncoder) により、ロータのd軸は mech_theta * POLE_PAIRS に一致する
  svc.elec_theta = FOC_NormalizeRadians(svc.mech_theta * POLE_PAIRS + ELEC_THETA_OFFSET);
  float sin_t, cos_t;
  FOC_SinCos(svc.elec_theta, &sin_t, &cos_t);

  // --- Clarke変換 → Park変換 ---
  float i_alpha, i_beta;
  FOC_Clarke(svc.iu, svc.iv, &i_alpha, &i_beta);
  FOC_Park(i_alpha, i_beta, sin_t, cos_t, &svc.id, &svc.iq);

  // --- 電流PI制御 ---
  svc.vd = FOC_PI_Update(&svc.id_pi, svc.target_id - svc.id, CURRENT_LOOP_DT);
  svc.vq = FOC_PI_Update(&svc.iq_pi, svc.target_iq - svc.iq, CURRENT_LOOP_DT);

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

// ADC1(PWM同期の電流計測)の変換完了割り込み。これが20kHzの制御ループ本体。
// ADC2も同じコールバックを共有するのでインスタンスの判定が必要。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) return;

  static uint16_t accel_cnt = 0;
  static uint16_t outer_cnt = 0;

  // 機械角と角速度はオブザーバが20kHzで同時に更新する (分周しない)
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);

  if (++accel_cnt >= ACCEL_CALC_DIV) {
    accel_cnt = 0;
    BLDC_CalculateAngularAccel();
  }
  if (++outer_cnt >= OUTER_LOOP_DIV) {
    outer_cnt = 0;
    BLDC_OuterLoop();
  }

  BLDC_CurrentLoop();
}

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

  BLDCFlashData write_data = {BLDC_FLASH_MAGIC, svc.max_encoder_val, svc.min_encoder_val,
                              svc.encoder_offset_theta, svc.encoder_dead_zone};
  Flash_WriteData(FLASH_USER_START_ADDR, &write_data, sizeof(write_data));
}

void BLDC_Init(bool do_set_encoder, volatile uint16_t* encoder_val) {
  printf("BLDC_Init\n");

  svc.encoder_val_ptr = encoder_val;
  svc.mode = BLDC_MODE_STOP;
  svc.enable = false;
  svc.supply_volt = 12.0f;  // 実測値が来るまでの暫定値

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

  // エンコーダ校正 (必要なときだけ。フラッシュ書き込みを伴う)
  if (do_set_encoder) {
    printf("BLDC_SetEncoder\n");
    BLDC_SetEncoder(encoder_val);
  }

  // フラッシュから読み込み
  BLDCFlashData read_data;
  Flash_ReadData(FLASH_USER_START_ADDR, &read_data, sizeof(read_data));
  printf("(From Flash)max: %lu, min: %lu, offset: %.6f rad, dead_zone: %.4f rad\n",
         (unsigned long)read_data.max_encoder_val,
         (unsigned long)read_data.min_encoder_val,
         read_data.encoder_offset_theta,
         read_data.encoder_dead_zone);

  svc.max_encoder_val = read_data.max_encoder_val;
  svc.min_encoder_val = read_data.min_encoder_val;
  svc.encoder_offset_theta = read_data.encoder_offset_theta;
  svc.encoder_dead_zone = read_data.encoder_dead_zone;

  // 未校正のフラッシュ(消去状態は全ビット1)や旧フォーマットのデータを読むと、
  // max == min による0除算や、盲点幅にゴミが入って角度が狂う。マジックナンバーと
  // 範囲チェックの両方で弾く。NaN も落とせるよう不等号は肯定形で書くこと。
  if (read_data.magic != BLDC_FLASH_MAGIC ||
      svc.max_encoder_val <= svc.min_encoder_val || svc.max_encoder_val > MAX_ADC_VAL ||
      !(svc.encoder_offset_theta > -TWO_PI_F && svc.encoder_offset_theta < TWO_PI_F) ||
      !(svc.encoder_dead_zone >= 0.0f && svc.encoder_dead_zone < TWO_PI_F * 0.25f)) {
    printf("BLDC: エンコーダ校正値が不正です。スイッチを押しながら再起動して校正してください。\n");
    svc.min_encoder_val = 0;
    svc.max_encoder_val = MAX_ADC_VAL;
    svc.encoder_offset_theta = 0.0f;
    svc.encoder_dead_zone = 0.0f;
  }
  BLDC_UpdateEncoderScale();

  // --- 制御器のゲイン ---
  // 外側ループの出力は「Iq指令 [A]」。電圧制御だった頃とは単位が違うので再調整が必要。
  svc.speed_pid.kp = 0.2f;
  svc.speed_pid.ki = 0.4f;
  svc.speed_pid.kd = 0;
  svc.speed_pid.d_term = 0;
  svc.speed_pid.d_lpf = 0.0f;
  svc.speed_pid.output_limit = MAX_CURRENT;

  svc.position_pid.kp = 5;
  svc.position_pid.ki = 10;
  svc.position_pid.kd = 0.2f;
  svc.position_pid.d_term = 0;
  svc.position_pid.d_lpf = 0.9f;
  svc.position_pid.output_limit = MAX_CURRENT;

  // 電流PIの出力は電圧 [V]。上限は変調限界。
  svc.id_pi.kp = CURRENT_PI_KP;
  svc.id_pi.ki = CURRENT_PI_KI;
  svc.id_pi.output_limit = svc.supply_volt * MAX_MODULATION_RATIO;
  FOC_PI_Reset(&svc.id_pi);
  svc.iq_pi = svc.id_pi;

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

  // ここまで来たら20kHzの制御ループ(ADC変換完了割り込み)を回し始める
  CurrentSense_EnableInterrupt();
  printf("BLDC control loop start (%.0f Hz)\n", (double)PWM_FREQ);
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
float BLDC_GetElecTheta(void) { return svc.elec_theta; }
float BLDC_GetAngularSpeed(void) { return svc.angular_speed; }
float BLDC_GetAngularAccel(void) { return svc.angular_accel; }
float BLDC_GetId(void) { return svc.id; }
float BLDC_GetIq(void) { return svc.iq; }
float BLDC_GetTargetIq(void) { return svc.target_iq; }
float BLDC_GetVd(void) { return svc.vd; }
float BLDC_GetVq(void) { return svc.vq; }
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
