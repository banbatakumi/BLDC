#include "bldc.h"

static SensoredVectorControl svc;

static PwmOut u_pwm;
static PwmOut v_pwm;
static PwmOut w_pwm;

static Timer dt_timer;
static Timer speed_dt_timer;

static inline void BLDC_GetEncoder(uint16_t encoder_val) {
  uint16_t correction_adc_val = encoder_val * svc.adc_correction_factor;
  float theta = correction_adc_val * ADC2RADIAN;               // 0〜2πの範囲に変換
  theta = NormalizeRadians(theta - svc.encoder_offset_theta);  // オフセット値を引いて正規化

  // ローパスフィルタ
  static float x_filt = 1.0f, y_filt = 0.0f;
  float enc_lpf = Constrain((50 - Abs(svc.angular_speed)) * K_ENC_LPF, 0, 0.75);  // フィルタ強度

  if (Abs(svc.angular_speed) <= 50) {
    float x = Cos(theta);
    float y = Sin(theta);

    x_filt = x * (1 - enc_lpf) + x_filt * enc_lpf;
    y_filt = y * (1 - enc_lpf) + y_filt * enc_lpf;
    svc.mech_theta = NormalizeRadians(Atan2(y_filt, x_filt));
  } else {
    svc.mech_theta = theta;
  }
}

static inline float BLDC_GetMaxEncoderVal(uint16_t encoder_val) {
  static uint16_t max_val = 4000;
  if (encoder_val > max_val) max_val = encoder_val;
  return max_val;
}

static inline void BLDC_GetAngularSpeedAndAccel(void) {
  static float pre_speed = 0;
  static float pre_theta = 0;
  static float pre_delta_theta = 0;
  static float pre_accel = 0;

  float dt = Timer_Read(&speed_dt_timer);
  if (dt < 0.005) return;
  if (dt > 0.01) dt = 0.01;  // 制御周期が大きすぎる場合は無視
  Timer_Reset(&speed_dt_timer);

  if (pre_theta == svc.mech_theta) {
    pre_theta = svc.mech_theta + pre_delta_theta;
    return;
  }

  float delta_theta = svc.mech_theta - pre_theta;

  // 0と2πの境目を跨いだ場合の補正
  if (delta_theta > PI) delta_theta -= TWO_PI;
  if (delta_theta < -PI) delta_theta += TWO_PI;

  // ノイズによる速度の異常値を補正
  if (Abs(delta_theta) > PI) delta_theta = pre_delta_theta;
  pre_delta_theta = delta_theta;

  float speed = delta_theta / dt;
  speed = speed * SPEED_LPF_INV + pre_speed * SPEED_LPF;  // ローパスフィルタ

  // 角加速度の計算
  float accel = (speed - pre_speed) / dt;
  accel = accel * (1.0f - ACCEL_LPF) + pre_accel * ACCEL_LPF;  // ローパスフィルタ
  pre_accel = accel;

  svc.angular_speed = speed;
  svc.angular_accel = accel;
  pre_speed = speed;
  pre_theta = svc.mech_theta;
}

static inline float BLDC_PIDControl(PIDController* pid, float error, float dt) {
  // 比例項
  float p_term = pid->kp * error;

  // 積分項
  pid->integral += pid->ki * error * dt;
  pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);

  // 微分項
  float raw_d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->d_term = pid->d_term * pid->d_lpf + raw_d_term * (1.0f - pid->d_lpf);
  pid->prev_error = error;

  // 出力の計算
  float output = p_term + pid->integral + pid->d_term;
  output = Constrain(output, -pid->output_limit, pid->output_limit);

  return output;
}

static inline void BLDC_SetEncoder(uint16_t* encoder_val) {
  svc.encoder_offset_theta = 0;  // オフセット計測前は0で初期化
  // エンコーダー出力の最大値を取得する
  float phase = 0;
  for (uint16_t i = 0; i < 3000; i++) {
    phase += 0.2;
    svc.max_encoder_val = BLDC_GetMaxEncoderVal(*encoder_val);
    BLDC_OpenLoopDrive(0.15, phase);
    HAL_Delay(1);
  }
  svc.adc_correction_factor = (float)MAX_ADC_VAL / svc.max_encoder_val;

  // 電気角度を0にオフセットする
  float offset_sum = 0;
  for (uint8_t i = 0; i < POLE_PAIRS; i++) {
    float theta_sum = 0;
    BLDC_OpenLoopDrive(0.2, 0);
    HAL_Delay(200);
    BLDC_OpenLoopDrive(0.4, 0);
    HAL_Delay(100);
    for (uint16_t i = 0; i < 200; i++) {
      BLDC_GetEncoder(*encoder_val);
      theta_sum += svc.mech_theta;
      HAL_Delay(1);
    }
    offset_sum += theta_sum * 0.005f;

    phase = 0;
    for (uint16_t i = 0; i < (TWO_PI * 10); i++) {
      phase += 0.1;
      BLDC_OpenLoopDrive(0.15, phase);
      HAL_Delay(1);
    }
  }
  svc.encoder_offset_theta = offset_sum / POLE_PAIRS;

  printf("(Measure)max_encoder_val: %d, encoder_offset_theta: %.6f\n", svc.max_encoder_val, svc.encoder_offset_theta);

  BLDCFlashData write_data = {svc.max_encoder_val, (float)svc.encoder_offset_theta};
  Flash_WriteData(FLASH_USER_START_ADDR, &write_data, sizeof(write_data));
}

static inline void BLDC_ApplyCurrentLimit(void) {
  float v_back_emf = KE * svc.angular_speed;
  svc.amp_volt = Constrain(svc.amp_volt, v_back_emf - MAX_CURRENT * R_PHASE,
                           v_back_emf + MAX_CURRENT * R_PHASE);
}

static inline void BLDC_WritePwm(float u, float v, float w) {
  u = Constrain(u, MIN_DUTY, MAX_DUTY);
  v = Constrain(v, MIN_DUTY, MAX_DUTY);
  w = Constrain(w, MIN_DUTY, MAX_DUTY);

  PwmOut_Write(&u_pwm, u);
  PwmOut_Write(&v_pwm, v);
  PwmOut_Write(&w_pwm, w);
}

void BLDC_Init(bool do_set_encoder, uint16_t* encoder_val) {
  printf("BLDC_Init\n");
  PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
  PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
  PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

  Timer_Init(&dt_timer);
  Timer_Init(&speed_dt_timer);

  // フラッシュに書き込み
  if (do_set_encoder) {
    printf("BLDC_EncoderOffset\n");
    BLDC_SetEncoder(encoder_val);
  }
  // フラッシュから読み込み
  BLDCFlashData read_data;
  Flash_ReadData(FLASH_USER_START_ADDR, &read_data, sizeof(read_data));
  printf("(From Flash)max_encoder_val: %ld, encoder_offset_theta: %.6f\n", read_data.max_encoder_val, read_data.encoder_offset_theta);
  svc.max_encoder_val = read_data.max_encoder_val;            // エンコーダーの最大値
  svc.encoder_offset_theta = read_data.encoder_offset_theta;  // エンコーダーのオフセット値
  svc.adc_correction_factor = (float)MAX_ADC_VAL / svc.max_encoder_val;

  // PIDコントローラ
  // 角速度制御
  svc.speed_pid.kp = 0.1;
  svc.speed_pid.ki = 0.2;
  svc.speed_pid.kd = 0;
  svc.speed_pid.d_term = 0;
  svc.speed_pid.d_lpf = 0.0f;
  svc.speed_pid.output_limit = MAX_AMP_VOLT;

  // 位置制御
  svc.position_pid.kp = 10;
  svc.position_pid.ki = 30;
  svc.position_pid.kd = 0.01;
  svc.position_pid.d_term = 0;
  svc.position_pid.d_lpf = 0.9f;
  svc.position_pid.output_limit = MAX_AMP_VOLT;
}

void BLDC_Stop(bool brake) {
  if (brake) {
    BLDC_WritePwm(0, 0, 0);
  } else {
    BLDC_WritePwm(0.5, 0.5, 0.5);
  }
}

void BLDC_OpenLoopDrive(float amp, float phase) {
  phase = NormalizeRadians(phase);

  float u = 0.5 + 0.5 * amp * Cos(phase);
  float v = 0.5 + 0.5 * amp * Cos(phase - TWO_THIRDS_PI);
  float w = 0.5 + 0.5 * amp * Cos(phase + TWO_THIRDS_PI);

  BLDC_WritePwm(u, v, w);
}

void BLDC_SensoredVectorControlDrive(uint16_t encoder_value, float supply_volt) {
  // 制御周期の取得
  svc.dt = Timer_Read(&dt_timer);
  Timer_Reset(&dt_timer);
  if (svc.dt > 0.001) svc.dt = 0.001;  // 制御周期が大きすぎる場合は無視

  // エンコーダ値を処理
  BLDC_GetEncoder(encoder_value);  // ラジアン(0〜2π)に変換
  BLDC_GetAngularSpeedAndAccel();  // 角速度・角加速度を計算

  // 電気角度を計算
  svc.elec_theta = svc.mech_theta * POLE_PAIRS;
  svc.elec_theta += Constrain(svc.angular_speed * K_ADV, -1.5, 1.5);  // 進角を加算(これがあると高速回転時に安定する)
  svc.elec_theta = NormalizeRadians(svc.elec_theta);

  svc.amp = svc.amp * AMP_LPF_COEF + (svc.amp_volt / supply_volt) * AMP_VOLT_LPF_COEF;
  svc.amp = Constrain(svc.amp, -1, 1);

  // 正弦波を生成 (sin+cos各1回で3相を導出)
  float s = Sin(svc.elec_theta);
  float c = Cos(svc.elec_theta);
  float half_amp = 0.5f * svc.amp;
  float u = 0.5f + half_amp * s;
  float v = 0.5f + half_amp * (-0.5f * s - SQRT3_2 * c);
  float w = 0.5f + half_amp * (-0.5f * s + SQRT3_2 * c);
  BLDC_WritePwm(u, v, w);
}

void BLDC_AngularSpeedControl(float target_angular_speed) {
  // 最大角速度制限
  target_angular_speed = Constrain(target_angular_speed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

  // 最大角加速度制限
  static float prev_target_angular_speed = 0;
  float accel = (target_angular_speed - prev_target_angular_speed) / svc.dt;
  accel = Constrain(accel, -MAX_ANGULAR_ACCEL, MAX_ANGULAR_ACCEL);
  target_angular_speed = prev_target_angular_speed + accel * svc.dt;
  prev_target_angular_speed = target_angular_speed;

  svc.amp_volt = -BLDC_PIDControl(&svc.speed_pid, target_angular_speed - svc.angular_speed, svc.dt);
  BLDC_ApplyCurrentLimit();
}

void BLDC_PositionControl(float target_position) {
  // 位置制御のためのPID計算
  float error = target_position - svc.mech_theta;  // 目標位置と現在位置の誤差

  // 0と2πのまたぎ対策
  while (error > PI) error -= TWO_PI;
  while (error < -PI) error += TWO_PI;
  svc.amp_volt = -BLDC_PIDControl(&svc.position_pid, error, svc.dt);
  BLDC_ApplyCurrentLimit();
}

void BLDC_TorqueControl(float target_torque) {
  // モデルベース電流推定によるトルク制御
  // T = Kt * I  →  I_target = target_torque / KT
  // V = R * I + Ke * ω  →  amp_volt = I_target * R_PHASE + KE * angular_speed
  float target_current = Constrain(target_torque / KT, -MAX_CURRENT, MAX_CURRENT);
  svc.amp_volt = target_current * R_PHASE + KE * svc.angular_speed;
}

void BLDC_VoltageControl(float target_volt) {
  svc.amp_volt = target_volt;
}

float BLDC_GetAngularSpeed(void) { return svc.angular_speed; }
float BLDC_GetAngularAccel(void) { return svc.angular_accel; }
float BLDC_GetMechTheta(void) { return svc.mech_theta; }
float BLDC_GetAmpVolt(void) { return svc.amp_volt; }
