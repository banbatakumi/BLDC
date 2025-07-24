#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

Timer dt_timer;
Timer vector_dt_timer;

void BLDC_Init(SensoredVectorControl* svc) {
      PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
      PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
      PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

      Timer_Init(&dt_timer);
      Timer_Init(&vector_dt_timer);

      // BLDCの固有パラメーター
      svc->pole_pairs = 7;  // 極対数 (磁石の数/2)

      // エンコーダー固有パラメーター
      // app.cでBLDC_SetEncoder()を呼び出してエンコーダーのオフセット値を取得する
      // svc->max_encoder_val = 4022;
      // svc->encoder_offset_theta = 3.553851;

      // svc->encoder_offset_theta = 0.223735;
      // svc->max_encoder_val = 4019;

      svc->encoder_offset_theta = 3.067202;
      svc->max_encoder_val = 4014;

      // svc->encoder_offset_theta = 0.856002;
      svc->adc_correction_factor = (double)MAX_ADC_VAL / svc->max_encoder_val;

      // PIDコントローラ
      // 速度制御
      svc->speed_pid.kp = 0.02;
      svc->speed_pid.ki = 5;
      svc->speed_pid.kd = 0;
      svc->speed_pid.output_limit = 3;

      // 位置制御
      svc->position_pid.kp = 5;
      svc->position_pid.ki = 5;
      svc->position_pid.kd = 0.025;
      svc->position_pid.output_limit = 2;
}

static inline double BLDC_GetEncoder(SensoredVectorControl* svc, uint16_t encoder_val, double encoder_offset_theta) {
      uint16_t correction_adc_val = encoder_val * svc->adc_correction_factor;
      double theta = correction_adc_val * ADC2RADIAN;          // 0〜2πの範囲に変換
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

static inline double BLDC_GetMaxEncoderVal(uint16_t encoder_val) {
      static uint16_t max_val = 4000;
      if (encoder_val > max_val) max_val = encoder_val;
      return max_val;
}
void BLDC_SetEncoder(SensoredVectorControl* svc, uint16_t* encoder_val) {
      static double theta_sum = 0;
      // エンコーダー出力の最大値を取得する
      for (uint16_t i = 0; i < 3000; i++) {
            svc->max_encoder_val = BLDC_GetMaxEncoderVal(*encoder_val);
            BLDC_OpenLoopDrive(0.1, 10);
            HAL_Delay(1);
      }
      // 電気角度を0にオフセットする
      for (uint16_t i = 0; i < 500; i++) {
            BLDC_OpenLoopDrive(i * 0.0008, 0);
            HAL_Delay(1);
      }
      for (uint16_t i = 0; i < 500; i++) {
            theta_sum += BLDC_GetEncoder(svc, *encoder_val, 0);
            HAL_Delay(1);
      }
      svc->encoder_offset_theta = theta_sum * 0.002f;
      BLDC_OpenLoopDrive(0, 0);
      printf("encoder_offset_theta: %.6f, max_encoder_val: %d\n", svc->encoder_offset_theta, svc->max_encoder_val);
}

static inline void BLDC_WritePwm(double u, double v, double w) {
      u = Constrain(u, MIN_DUTY, MAX_DUTY);
      v = Constrain(v, MIN_DUTY, MAX_DUTY);
      w = Constrain(w, MIN_DUTY, MAX_DUTY);

      PwmOut_Write(&u_pwm, u);
      PwmOut_Write(&v_pwm, v);
      PwmOut_Write(&w_pwm, w);
}

void BLDC_Stop(bool brake) {
      if (brake) {
            BLDC_WritePwm(0, 0, 0);
      } else {
            BLDC_WritePwm(0.5, 0.5, 0.5);
      }
}

void BLDC_OpenLoopDrive(double amp, double freq) {
      // 制御周期の取得
      double dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);

      static double phase = 0;
      if (freq == 0) phase = HALF_PI;
      double delta_phase = TWO_PI * freq * dt;
      phase += delta_phase;
      phase = NormalizeRadians(phase);

      double u = 0.5 + 0.5 * amp * Sin(phase);
      double v = 0.5 + 0.5 * amp * Sin(phase + TWO_THIRDS_PI);
      double w = 0.5 + 0.5 * amp * Sin(phase - TWO_THIRDS_PI);

      BLDC_WritePwm(u, v, w);
}

static inline double BLDC_GetSpeed(double theta, double dt) {
      static double pre_speed = 0;
      static double pre_theta = 0;
      static double pre_delta_theta = 0;

      if (pre_theta == theta) return pre_speed;

      double delta_theta = theta - pre_theta;

      // 0と2πの境目を跨いだ場合の補正
      if (delta_theta > PI) delta_theta -= TWO_PI;
      if (delta_theta < -PI) delta_theta += TWO_PI;

      // スパイク除去
      if (Abs(delta_theta) > PI) delta_theta = pre_delta_theta;
      pre_delta_theta = delta_theta;

      double speed = delta_theta / dt;
      speed = speed * (1 - SPEED_LPF) + pre_speed * SPEED_LPF;
      pre_speed = speed;
      pre_theta = theta;

      return speed;
}

static inline double BLDC_PIDControl(PIDController* pid, double error, double dt) {
      // 比例項
      double p_term = pid->kp * error;

      // 積分項
      pid->integral += pid->ki * error * dt;
      if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
      if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;

      // 微分項
      double d_term = pid->kd * (error - pid->prev_error) / dt;
      pid->prev_error = error;

      // 出力の計算
      double output = p_term + pid->integral + d_term;
      if (output > pid->output_limit) output = pid->output_limit;
      if (output < -pid->output_limit) output = -pid->output_limit;

      return output;
}

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, double supply_volt) {
      // 制御周期の取得
      svc->dt = Timer_Read(&vector_dt_timer);
      Timer_Reset(&vector_dt_timer);
      if (svc->dt > 0.001) return;  // 制御周期が大きすぎる場合は無視

      // エンコーダ値を処理
      svc->mech_theta = BLDC_GetEncoder(svc, encoder_value, svc->encoder_offset_theta);  // ラジアン(0〜2π)に変換
      svc->speed = BLDC_GetSpeed(svc->mech_theta, svc->dt);                              // 角速度を計算

      // 電気角度を計算
      svc->elec_theta = svc->mech_theta * svc->pole_pairs;
      svc->elec_theta += Constrain(svc->speed * K_ADV, -1.5, 1.5);  // 進角を加算(これがあると高速回転時に安定する)
      svc->elec_theta = NormalizeRadians(svc->elec_theta);

      svc->amp = svc->amp * 0.25 + (svc->amp_volt / supply_volt) * 0.75;  // ローパスフィルタ
      svc->amp = Constrain(svc->amp, -1, 1);

      // 正弦波を生成
      double u = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta);
      double v = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta - TWO_THIRDS_PI);
      double w = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta + TWO_THIRDS_PI);

      BLDC_WritePwm(u, v, w);
}

void BLDC_SpeedControl(SensoredVectorControl* svc, double target_speed) {
      double dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);
      if (dt > 0.001) return;

      // 最大速度制限
      if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
      if (target_speed < -MAX_SPEED) target_speed = -MAX_SPEED;

      // 最大加速度制限
      static double prev_target_speed = 0;
      double accel = (target_speed - prev_target_speed) / dt;
      if (accel > MAX_ACCEL) accel = MAX_ACCEL;
      if (accel < -MAX_ACCEL) accel = -MAX_ACCEL;
      target_speed = prev_target_speed + accel * dt;
      prev_target_speed = target_speed;

      double ff_term = K_FF * target_speed;
      svc->amp_volt = -(BLDC_PIDControl(&svc->speed_pid, target_speed - svc->speed, dt) + ff_term);

      // 低速時は積分ゲインを上げて回転を安定させる
      if (Abs(svc->speed) < 5) {
            svc->speed_pid.ki = (5 - Abs(svc->speed)) * 2 + 0.2;
            svc->speed_pid.kp = Abs(svc->speed) * 0.002;
      } else {
            svc->speed_pid.ki = 0.2;
            svc->speed_pid.kp = 0.01;
      }
}

void BLDC_PositionControl(SensoredVectorControl* svc, double target_position) {
      double dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);
      if (dt > 0.001) return;

      // 位置制御のためのPID計算
      double error = target_position - (svc->mech_theta + svc->encoder_offset_theta);  // 目標位置と現在位置の誤差

      // 0と2πのまたぎ対策
      while (error > PI) error -= TWO_PI;
      while (error < -PI) error += TWO_PI;
      svc->amp_volt = -BLDC_PIDControl(&svc->position_pid, error, dt);
}