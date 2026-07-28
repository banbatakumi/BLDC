#include "bldc.h"

typedef struct {
  // --- 設定 ---
  volatile uint16_t* encoder_val_ptr;  // ADC2のDMAが更新するエンコーダ値へのポインタ
  uint16_t max_encoder_val;            // エンコーダーの最大値
  uint16_t min_encoder_val;            // エンコーダーの最小値
  float encoder_scale;                 // 2π / (max - min)。毎回割り算しないよう事前計算する
  float encoder_offset_theta;          // エンコーダーのオフセット値 [rad]
  bool encoder_primed;                 // 機械角をエンコーダ実測値で初期化済みか

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
// エンコーダのレンジからラジアン変換係数を作り直す。max/minを変えたら必ず呼ぶこと。
static void BLDC_UpdateEncoderScale(void) {
  float range = (float)(svc.max_encoder_val - svc.min_encoder_val);
  svc.encoder_scale = (range > 0.0f) ? (TWO_PI_F / range) : 0.0f;
}

static inline void BLDC_UpdateEncoder(uint16_t encoder_val) {
  uint16_t clamped_encoder_val = Constrain(encoder_val, svc.min_encoder_val, svc.max_encoder_val);
  float theta = (float)(clamped_encoder_val - svc.min_encoder_val) * svc.encoder_scale;
  theta = FOC_NormalizeRadians(theta - svc.encoder_offset_theta);

  // 初回はフィルタを通さず実測値をそのまま入れる。
  // これをしないと mech_theta が 0 から実際の角度まで移動する約1msの間、
  // 電気角がその極対数倍(7倍)の速さで何回転もしてしまい、電流制御が破綻する。
  if (!svc.encoder_primed) {
    svc.encoder_primed = true;
    svc.mech_theta = theta;
    return;
  }

  // 0と2πの境目を跨いでも壊れないよう、差分を-π〜+πに正規化してから扱う。
  float diff = FOC_AngleDiff(theta, svc.mech_theta);

  // 物理的にありえない飛びはセンサのグリッチなので頭打ちにする。
  // これが無いと、0/2πの切り替わりで拾った1サンプルのノイズが
  // 極対数倍された電気角の飛びになり、トルクが乱れて「カチッ」と鳴る。
  diff = Constrain(diff, -ENCODER_MAX_STEP_RAD, ENCODER_MAX_STEP_RAD);

  // 低速域ではエンコーダのノイズが効くのでローパスをかける。
  // 高速域(50rad/s超)ではフィルタの遅れのほうが害になるので素通しにする。
  float abs_speed = Abs(svc.angular_speed);
  float enc_lpf = (abs_speed <= 50.0f)
                      ? Constrain((50.0f - abs_speed) * K_ENC_LPF, 0.0f, 0.75f)
                      : 0.0f;

  svc.mech_theta = FOC_NormalizeRadians(svc.mech_theta + diff * (1.0f - enc_lpf));
}

static inline void BLDC_CalculateAngularSpeed(void) {
  static float pre_theta = 0;

  // 0と2πの境目を跨いだ場合の補正込みで角度差を求める
  float delta_theta = FOC_AngleDiff(svc.mech_theta, pre_theta);
  pre_theta = svc.mech_theta;

  float speed = delta_theta * (1.0f / SPEED_CALC_DT);
  svc.angular_speed = speed * SPEED_LPF_INV + svc.angular_speed * SPEED_LPF;  // ローパスフィルタ
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
    // 全相デューティ0.5。線間電圧0なので電流は流れない。
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
    BLDC_WritePwm(0.5f, 0.5f, 0.5f);
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
}

// ADC1(PWM同期の電流計測)の変換完了割り込み。これが20kHzの制御ループ本体。
// ADC2も同じコールバックを共有するのでインスタンスの判定が必要。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) return;

  static uint16_t speed_cnt = 0;
  static uint16_t accel_cnt = 0;
  static uint16_t outer_cnt = 0;

  BLDC_UpdateEncoder(*svc.encoder_val_ptr);

  if (++speed_cnt >= SPEED_CALC_DIV) {
    speed_cnt = 0;
    BLDC_CalculateAngularSpeed();
  }
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
static void BLDC_SetEncoder(uint16_t* encoder_val) {
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
  BLDC_UpdateEncoderScale();  // これ以降 BLDC_UpdateEncoder が使えるようになる

  // 電気角度0の位置にロータを引き込んで、そのときの機械角をオフセットとする
  float offset_sum = 0;
  for (uint8_t i = 0; i < POLE_PAIRS; i++) {
    float theta_sum = 0;
    BLDC_OpenLoopDrive(0.15f, 0);
    HAL_Delay(200);
    BLDC_OpenLoopDrive(0.3f, 0);
    HAL_Delay(100);
    for (uint16_t j = 0; j < 200; j++) {
      BLDC_UpdateEncoder(*encoder_val);
      theta_sum += svc.mech_theta;
      HAL_Delay(1);
    }
    offset_sum += theta_sum * 0.005f;

    phase = 0;
    for (uint16_t j = 0; j < (uint16_t)(TWO_PI * 10); j++) {
      phase += 0.1f;
      BLDC_OpenLoopDrive(0.15f, phase);
      HAL_Delay(1);
    }
  }
  svc.encoder_offset_theta = offset_sum / POLE_PAIRS;

  printf("(Measure)max_encoder_val: %lu, min_encoder_val: %lu, encoder_offset_theta: %.6f\n",
         (unsigned long)svc.max_encoder_val,
         (unsigned long)svc.min_encoder_val,
         svc.encoder_offset_theta);

  BLDCFlashData write_data = {svc.max_encoder_val, svc.min_encoder_val, svc.encoder_offset_theta};
  Flash_WriteData(FLASH_USER_START_ADDR, &write_data, sizeof(write_data));
}

void BLDC_Init(bool do_set_encoder, uint16_t* encoder_val) {
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
  printf("(From Flash)max_encoder_val: %lu, min_encoder_val: %lu, encoder_offset_theta: %.6f\n",
         (unsigned long)read_data.max_encoder_val,
         (unsigned long)read_data.min_encoder_val,
         read_data.encoder_offset_theta);

  svc.max_encoder_val = read_data.max_encoder_val;
  svc.min_encoder_val = read_data.min_encoder_val;
  svc.encoder_offset_theta = read_data.encoder_offset_theta;

  // 未校正のフラッシュ(消去状態は全ビット1)を読むと max == min になり、
  // 機械角の計算で0除算してNaNが伝播してしまうので弾いておく。
  if (svc.max_encoder_val <= svc.min_encoder_val || svc.max_encoder_val > MAX_ADC_VAL ||
      !(svc.encoder_offset_theta > -TWO_PI_F && svc.encoder_offset_theta < TWO_PI_F)) {
    printf("BLDC: エンコーダ校正値が不正です。スイッチを押しながら再起動して校正してください。\n");
    svc.min_encoder_val = 0;
    svc.max_encoder_val = MAX_ADC_VAL;
    svc.encoder_offset_theta = 0.0f;
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

  svc.position_pid.kp = 10;
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
