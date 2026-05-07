#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

Timer dt_timer;

Timer speed_dt_timer;

static inline float BLDC_GetEncoder(SensoredVectorControl* svc, uint16_t encoder_val, float encoder_offset_theta) {
  uint16_t correction_adc_val = encoder_val * svc->adc_correction_factor;
  float theta = correction_adc_val * ADC2RADIAN;           // 0〜2πの範囲に変換
  theta = NormalizeRadians(theta - encoder_offset_theta);  // オフセット値を引いて正規化

  // ローパスフィルタ
  static float x_filt = 1.0f, y_filt = 0.0f;
  float enc_lpf = Constrain((50 - Abs(svc->speed)) * K_ENC_LPF, 0, 0.75);  // フィルタ強度

  if (Abs(svc->speed) <= 50) {
    float x = Cos(theta);
    float y = Sin(theta);

    x_filt = x * (1 - enc_lpf) + x_filt * enc_lpf;
    y_filt = y * (1 - enc_lpf) + y_filt * enc_lpf;
    return NormalizeRadians(Atan2(y_filt, x_filt));
  } else {
    return theta;
  }
}

static inline float BLDC_GetMaxEncoderVal(uint16_t encoder_val) {
  static uint16_t max_val = 4000;
  if (encoder_val > max_val) max_val = encoder_val;
  return max_val;
}

static inline float BLDC_GetSpeed(float theta, float dt) {
  static float pre_speed = 0;
  static float pre_theta = 0;
  static float pre_delta_theta = 0;

  if (pre_theta == theta) {
    pre_theta = theta + pre_delta_theta;
    return pre_speed;
  }

  float delta_theta = theta - pre_theta;

  // 0と2πの境目を跨いだ場合の補正
  if (delta_theta > PI) delta_theta -= TWO_PI;
  if (delta_theta < -PI) delta_theta += TWO_PI;

  // スパイク除去
  if (Abs(delta_theta) > PI) delta_theta = pre_delta_theta;
  pre_delta_theta = delta_theta;

  float speed = delta_theta / dt;
  speed = speed * (1 - SPEED_LPF) + pre_speed * SPEED_LPF;
  pre_speed = speed;
  pre_theta = theta;

  return speed;
}

static inline float BLDC_PIDControl(PIDController* pid, float error, float dt) {
  // 比例項
  float p_term = pid->kp * error;

  // 積分項
  pid->integral += pid->ki * error * dt;
  if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
  if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;

  // 微分項
  float d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->prev_error = error;

  // 出力の計算
  float output = p_term + pid->integral + d_term;
  if (output > pid->output_limit) output = pid->output_limit;
  if (output < -pid->output_limit) output = -pid->output_limit;

  return output;
}

static inline void BLDC_SetEncoder(SensoredVectorControl* svc, uint16_t* encoder_val) {
  // エンコーダー出力の最大値を取得する
  float phase = 0;
  for (uint16_t i = 0; i < 3000; i++) {
    phase += 0.2;
    svc->max_encoder_val = BLDC_GetMaxEncoderVal(*encoder_val);
    BLDC_OpenLoopDrive(0.15, phase);
    HAL_Delay(1);
  }
  svc->adc_correction_factor = (float)MAX_ADC_VAL / svc->max_encoder_val;

  // 電気角度を0にオフセットする
  float offset_sum = 0;
  for (uint8_t i = 0; i < svc->pole_pairs; i++) {
    float theta_sum = 0;
    BLDC_OpenLoopDrive(0.2, 0);
    HAL_Delay(200);
    BLDC_OpenLoopDrive(0.4, 0);
    HAL_Delay(100);
    for (uint16_t i = 0; i < 200; i++) {
      theta_sum += BLDC_GetEncoder(svc, *encoder_val, 0);
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
  svc->encoder_offset_theta = offset_sum / svc->pole_pairs;

  printf("(Measure)max_encoder_val: %d, encoder_offset_theta: %.6f\n", svc->max_encoder_val, svc->encoder_offset_theta);

  BLDCFlashData write_data = {svc->max_encoder_val, (float)svc->encoder_offset_theta};
  Flash_WriteData(FLASH_USER_START_ADDR, &write_data, sizeof(write_data));
}

static inline void BLDC_WritePwm(float u, float v, float w) {
  u = Constrain(u, MIN_DUTY, MAX_DUTY);
  v = Constrain(v, MIN_DUTY, MAX_DUTY);
  w = Constrain(w, MIN_DUTY, MAX_DUTY);

  PwmOut_Write(&u_pwm, u);
  PwmOut_Write(&v_pwm, v);
  PwmOut_Write(&w_pwm, w);
}

void BLDC_Init(SensoredVectorControl* svc, bool do_set_encoder, uint16_t* encoder_val) {
  printf("BLDC_Init\n");
  PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
  PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
  PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

  Timer_Init(&dt_timer);
  Timer_Init(&speed_dt_timer);

  // BLDCの固有パラメーター
  svc->pole_pairs = 7;  // 極対数 (磁石の数/2)

  // フラッシュに書き込み
  if (do_set_encoder) {
    printf("BLDC_EncoderOffset\n");
    BLDC_SetEncoder(svc, encoder_val);
  }
  // フラッシュから読み込み
  BLDCFlashData read_data;
  Flash_ReadData(FLASH_USER_START_ADDR, &read_data, sizeof(read_data));
  printf("(From Flash)max_encoder_val: %ld, encoder_offset_theta: %.6f\n", read_data.max_encoder_val, read_data.encoder_offset_theta);
  svc->max_encoder_val = read_data.max_encoder_val;            // エンコーダーの最大値
  svc->encoder_offset_theta = read_data.encoder_offset_theta;  // エンコーダーのオフセット値
  svc->adc_correction_factor = (float)MAX_ADC_VAL / svc->max_encoder_val;

  // PIDコントローラ
  // 速度制御
  svc->speed_pid.kp = 0.1;
  svc->speed_pid.ki = 0.2;
  svc->speed_pid.kd = 0;
  svc->speed_pid.output_limit = 3;

  // 位置制御
  svc->position_pid.kp = 10;
  svc->position_pid.ki = 20;
  svc->position_pid.kd = 0;
  svc->position_pid.output_limit = 3;
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

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, float supply_volt) {
  // 制御周期の取得
  svc->dt = Timer_Read(&dt_timer);
  Timer_Reset(&dt_timer);
  if (svc->dt > 0.001) return;  // 制御周期が大きすぎる場合は無視

  // エンコーダ値を処理
  svc->mech_theta = BLDC_GetEncoder(svc, encoder_value, svc->encoder_offset_theta);  // ラジアン(0〜2π)に変換
  if (Timer_Read(&speed_dt_timer) > 0.01) {                                          // 10msごとに速度を計算
    svc->speed = BLDC_GetSpeed(svc->mech_theta, Timer_Read(&speed_dt_timer));        // 角速度を計算
    Timer_Reset(&speed_dt_timer);
  }

  // 電気角度を計算
  svc->elec_theta = svc->mech_theta * svc->pole_pairs;
  svc->elec_theta += Constrain(svc->speed * K_ADV, -1.5, 1.5);  // 進角を加算(これがあると高速回転時に安定する)
  svc->elec_theta = NormalizeRadians(svc->elec_theta);

  svc->amp = svc->amp * 0.4 + (svc->amp_volt / supply_volt) * 0.6;  // ローパスフィルタ
  svc->amp = Constrain(svc->amp, -1, 1);

  // 正弦波を生成
  float u, v, w;
  u = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta);
  v = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta - TWO_THIRDS_PI);
  w = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta + TWO_THIRDS_PI);
  BLDC_WritePwm(u, v, w);
}

void BLDC_SpeedControl(SensoredVectorControl* svc, float target_speed) {
  // 最大速度制限
  if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
  if (target_speed < -MAX_SPEED) target_speed = -MAX_SPEED;

  // 最大加速度制限
  static float prev_target_speed = 0;
  float accel = (target_speed - prev_target_speed) / svc->dt;
  if (accel > MAX_ACCEL) accel = MAX_ACCEL;
  if (accel < -MAX_ACCEL) accel = -MAX_ACCEL;
  target_speed = prev_target_speed + accel * svc->dt;
  prev_target_speed = target_speed;

  svc->amp_volt = -BLDC_PIDControl(&svc->speed_pid, target_speed - svc->speed, svc->dt);
}

void BLDC_PositionControl(SensoredVectorControl* svc, float target_position) {
  // 位置制御のためのPID計算
  float error = target_position - (svc->mech_theta);  // 目標位置と現在位置の誤差

  // 0と2πのまたぎ対策
  while (error > PI) error -= TWO_PI;
  while (error < -PI) error += TWO_PI;
  svc->amp_volt = -BLDC_PIDControl(&svc->position_pid, error, svc->dt);
}

void BLDC_TorqueControl(SensoredVectorControl* svc, float target_torque) {
  svc->amp_volt = target_torque;
}