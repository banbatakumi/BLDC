#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

Timer dt_timer;

void BLDC_Init(SensoredVectorControl* svc) {
      PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
      PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
      PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

      Timer_Init(&dt_timer);

      svc->pole_pairs = POLE_PAIRS;
      svc->amp = 0;
      svc->mech_theta = 0;
      svc->elec_theta = 0;

      // PIDコントローラの初期化
      svc->speed_pid.kp = 0.0001;  // 比例ゲイン
      svc->speed_pid.ki = 0.1;     // 積分ゲイン
      svc->speed_pid.kd = 0;       // 微分ゲイン
      svc->speed_pid.integral = 0;
      svc->speed_pid.prev_error = 0;
      svc->speed_pid.output_limit = 0.7;

      svc->position_pid.kp = 0.5;  // 比例ゲイン
      svc->position_pid.ki = 0.2;  // 積分ゲイン
      svc->position_pid.kd = 0;    // 微分ゲイン
      svc->position_pid.integral = 0;
      svc->position_pid.prev_error = 0;
      svc->position_pid.output_limit = 0.7;
}

static inline void BLDC_WritePwm(float u, float v, float w) {
      // デューティ比の制限
      u = Constrain(u, 0, MAX_DUTY);
      v = Constrain(v, 0, MAX_DUTY);
      w = Constrain(w, 0, MAX_DUTY);

      PwmOut_Write(&u_pwm, u);
      PwmOut_Write(&v_pwm, v);
      PwmOut_Write(&w_pwm, w);
}

void BLDC_OpenLoopDrive(float amp, float freq) {
      static float phase = 0;
      float dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);

      float delta_phase = 360 * freq * dt;
      phase += delta_phase;
      if (phase >= 360) phase -= 360;

      float u = 0.5 + 0.5 * amp * SinDeg((int)phase);
      float v = 0.5 + 0.5 * amp * SinDeg((int)phase - 120);
      float w = 0.5 + 0.5 * amp * SinDeg((int)phase + 120);

      BLDC_WritePwm(u, v, w);
}

/*
ベクトル制御
*/

static inline float BLDC_GetEncoder(uint16_t adc_val) {
      static uint16_t max_adc_val = 4000;
      if (adc_val > max_adc_val) max_adc_val = adc_val;

      uint16_t correction_adc_val = adc_val * ((float)MAX_ADC_VAL / max_adc_val);
      const float conversion_factor = TWO_PI / MAX_ADC_VAL;

      return (float)correction_adc_val * conversion_factor;
}

static inline float BLDC_GetSpeed(float theta, double dt) {
      static float pre_speed = 0;
      static float prev_theta = 0;
      static float pre_delta_theta = 0;

      float delta_theta = prev_theta - theta;

      // 0と2πの境目を跨いだ場合の補正
      if (delta_theta > PI) delta_theta -= TWO_PI;
      if (delta_theta < -PI) delta_theta += TWO_PI;

      // スパイク除去
      if (Abs(delta_theta) > PI) delta_theta = pre_delta_theta;
      pre_delta_theta = delta_theta;

      float speed = delta_theta / dt;
      speed = speed * (1 - lpf) + pre_speed * lpf;
      pre_speed = speed;
      prev_theta = theta;

      return speed;
}
static inline float BLDC_PIDControl(PIDController* pid, float error, double dt) {
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

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value) {
      static double pre_dt = 0;
      svc->dt = Timer_Read(&dt_timer) * 0.1 + pre_dt * 0.9;
      Timer_Reset(&dt_timer);
      pre_dt = svc->dt;
      if (svc->dt > 0.01) return;

      // エンコーダ値を処理
      svc->mech_theta = BLDC_GetEncoder(encoder_value);      // エンコーダ値をラジアン(0〜2π)に変換
      svc->speed = BLDC_GetSpeed(svc->mech_theta, svc->dt);  // 速度を計算

      // 電気角度を計算
      svc->elec_theta = svc->mech_theta * svc->pole_pairs + svc->speed * K_ADV;  // 電気角度を計算
      svc->elec_theta = NormalizeRadians(svc->elec_theta);                       // 電気角度を0〜2πに正規化

      // 正弦波を生成
      float u = 0.5 + 0.5 * Abs(svc->amp) * Sin(svc->elec_theta);
      float v = 0.5 + 0.5 * Abs(svc->amp) * Sin(svc->elec_theta + (TWO_PI / 3.0));
      float w = 0.5 + 0.5 * Abs(svc->amp) * Sin(svc->elec_theta - (TWO_PI / 3.0));

      if (svc->amp > 0) {
            BLDC_WritePwm(u, v, w);
      } else {
            BLDC_WritePwm(w, u, v);
      }
}

void BLDC_SpeedControl(SensoredVectorControl* svc, float target_speed) {
      float ff_term = K_FF * target_speed;
      svc->amp = BLDC_PIDControl(&svc->speed_pid, target_speed - svc->speed, svc->dt) + ff_term;
      if (target_speed > 0 && svc->amp < 0) svc->amp = 0;
      if (target_speed < 0 && svc->amp > 0) svc->amp = 0;
}

void BLDC_PositionControl(SensoredVectorControl* svc, float target_position) {
      target_position = NormalizeRadians(target_position);  // 目標位置を0〜2πの範囲に制限
      // 位置制御のためのPID計算
      float error = target_position - svc->mech_theta;  // 目標位置と現在位置の誤差
                                                        // 0と2πのまたぎ対策
      if (error > PI) error -= TWO_PI;
      if (error < -PI) error += TWO_PI;
      svc->amp = -BLDC_PIDControl(&svc->position_pid, error, svc->dt);
}