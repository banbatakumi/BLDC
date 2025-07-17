#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

Timer dt_timer;
Timer encoder_offset_timer;

void BLDC_Init(SensoredVectorControl* svc) {
      PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
      PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
      PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

      Timer_Init(&dt_timer);

      // BLDCの固有パラメーター
      svc->pole_pairs = 7;  // 極対数 (磁石の数/2)

      // エンコーダー固有パラメーター
      // app.cでBLDC_SetEncoder()を呼び出してエンコーダーのオフセット値を取得する
      svc->max_encoder_val = 4032;
      svc->encoder_offset_theta = -1.127315;

      // PIDコントローラ
      // 速度制御
      svc->speed_pid.kp = 0.01;
      svc->speed_pid.ki = 0.5;
      svc->speed_pid.kd = 0;
      svc->speed_pid.output_limit = 2.5;

      // 位置制御
      svc->position_pid.kp = 5;
      svc->position_pid.ki = 2.5;
      svc->position_pid.kd = 0;
      svc->position_pid.output_limit = 2.5;
}

static inline double BLDC_GetEncoder(SensoredVectorControl* svc, uint16_t encoder_val, double encoder_offset_theta) {
      uint16_t correction_adc_val = encoder_val * ((double)MAX_ADC_VAL / svc->max_encoder_val);
      double theta = (double)correction_adc_val * (TWO_PI / MAX_ADC_VAL);  // 0〜2πの範囲に変換
      theta = NormalizeRadians(theta - encoder_offset_theta);              // オフセット値を引いて正規化

      // ローパスフィルタ
      static float x_filt = 1.0f, y_filt = 0.0f;
      float enc_lpf = Constrain((100 - Abs(svc->speed)) * K_ENC_LPF, 0, 0.5);  // フィルタ強度

      float x = Cos(theta);
      float y = Sin(theta);

      x_filt = x * (1.0f - enc_lpf) + x_filt * enc_lpf;
      y_filt = y * (1.0f - enc_lpf) + y_filt * enc_lpf;

      float theta_filt = atan2(y_filt, x_filt);

      return theta_filt;
}

static inline double BLDC_GetMaxEncoderVal(uint16_t encoder_val) {
      static uint16_t max_val = 4000;
      if (encoder_val > max_val) max_val = encoder_val;
      return max_val;
}

bool BLDC_SetEncoder(SensoredVectorControl* svc, uint16_t encoder_value) {
      static bool is_first = true;
      if (is_first) {
            Timer_Init(&encoder_offset_timer);
            Timer_Reset(&encoder_offset_timer);
            is_first = false;
      }
      static uint16_t cnt = 0;
      static double theta_sum = 0;
      if (Timer_Read(&encoder_offset_timer) < 2) {
            // エンコーダー出力の最大値を取得する
            svc->max_encoder_val = BLDC_GetMaxEncoderVal(encoder_value);
            BLDC_OpenLoopDrive(0.3, 30);
      } else if (Timer_Read(&encoder_offset_timer) < 3) {
            // 電気角度を0にオフセットする
            BLDC_OpenLoopDrive(0.3, 0);
            cnt = 0;
            theta_sum = 0;
      } else if (cnt < 500) {
            theta_sum += BLDC_GetEncoder(svc, encoder_value, 0);
            HAL_Delay(1);
            cnt++;
      } else {
            svc->encoder_offset_theta = theta_sum * 0.002f;
            BLDC_OpenLoopDrive(0, 0);
            printf("encoder_offset_theta: %.6f, max_encoder_val: %d\n", svc->encoder_offset_theta, svc->max_encoder_val);
            is_first = true;
            return true;
      }
      return false;
}

static inline void BLDC_WritePwm(double u, double v, double w) {
      u = Constrain(u, 0, MAX_DUTY);
      v = Constrain(v, 0, MAX_DUTY);
      w = Constrain(w, 0, MAX_DUTY);

      PwmOut_Write(&u_pwm, u);
      PwmOut_Write(&v_pwm, v);
      PwmOut_Write(&w_pwm, w);
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
      double v = 0.5 + 0.5 * amp * Sin(phase + (TWO_PI / 3.0));
      double w = 0.5 + 0.5 * amp * Sin(phase - (TWO_PI / 3.0));

      BLDC_WritePwm(u, v, w);
}

static inline double BLDC_GetSpeed(double theta, double dt) {
      static double pre_speed = 0;
      static double prev_theta = 0;
      static double pre_delta_theta = 0;

      double delta_theta = theta - prev_theta;

      // 0と2πの境目を跨いだ場合の補正
      if (delta_theta > PI) delta_theta -= TWO_PI;
      if (delta_theta < -PI) delta_theta += TWO_PI;

      // スパイク除去
      if (Abs(delta_theta) > PI) delta_theta = pre_delta_theta;
      pre_delta_theta = delta_theta;

      double speed = delta_theta / dt;
      speed = speed * (1 - SPEED_LPF) + pre_speed * SPEED_LPF;
      pre_speed = speed;
      prev_theta = theta;

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
      svc->dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);
      if (svc->dt > 0.01) return;  // 制御周期が大きすぎる場合は無視

      // エンコーダ値を処理
      svc->mech_theta = BLDC_GetEncoder(svc, encoder_value, svc->encoder_offset_theta);  // ラジアン(0〜2π)に変換
      svc->speed = BLDC_GetSpeed(svc->mech_theta, svc->dt);                              // 角速度を計算

      // 電気角度を計算
      svc->elec_theta = svc->mech_theta * svc->pole_pairs;
      svc->elec_theta += svc->speed * K_ADV;  // 進角を加算(これがあると高速回転時に安定する)
      svc->elec_theta = NormalizeRadians(svc->elec_theta);

      svc->amp = svc->amp_volt / supply_volt;
      if (svc->amp > 1.0f) svc->amp = 1.0f;

      // 正弦波を生成
      double u = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta);
      double v = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta + (TWO_PI / 3.0));
      double w = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta - (TWO_PI / 3.0));

      BLDC_WritePwm(u, w, v);
}

void BLDC_SpeedControl(SensoredVectorControl* svc, double target_speed) {
      if (svc->dt > 0.01) return;
      double ff_term = K_FF * target_speed;
      svc->amp_volt = -(BLDC_PIDControl(&svc->speed_pid, target_speed - svc->speed, svc->dt) + ff_term);
}

void BLDC_PositionControl(SensoredVectorControl* svc, double target_position) {
      if (svc->dt > 0.01) return;

      // 位置制御のためのPID計算
      double error = target_position - (svc->mech_theta + svc->encoder_offset_theta);  // 目標位置と現在位置の誤差

      // 0と2πのまたぎ対策
      while (error > PI) error -= TWO_PI;
      while (error < -PI) error += TWO_PI;
      svc->amp_volt = -BLDC_PIDControl(&svc->position_pid, error, svc->dt);
      svc->position_pid.kd = 0.1;
      if (Abs(error) < 0.15) svc->position_pid.kd = 0;
}