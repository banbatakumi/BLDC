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
      svc->amp = 0.0f;
      svc->mech_theta = 0.0f;
      svc->elec_theta = 0.0f;

      // PIDコントローラの初期化
      svc->speed_pid.kp = 1.0f;   // 比例ゲイン
      svc->speed_pid.ki = 0.1f;   // 積分ゲイン
      svc->speed_pid.kd = 0.01f;  // 微分ゲイン
      svc->speed_pid.integral = 0.0f;
      svc->speed_pid.prev_error = 0.0f;
      svc->speed_pid.output_limit = MAX_DUTY;
}

static inline void BLDC_WritePwm(float u, float v, float w) {
      // デューティ比の制限
      u = Constrain(u, 0.0f, MAX_DUTY);
      v = Constrain(v, 0.0f, MAX_DUTY);
      w = Constrain(w, 0.0f, MAX_DUTY);

      PwmOut_Write(&u_pwm, u);
      PwmOut_Write(&v_pwm, v);
      PwmOut_Write(&w_pwm, w);
}

void BLDC_OpenLoopDrive(float amp, float freq) {
      static float phase = 0.0f;
      float dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);

      float delta_phase = 360.0f * freq * dt;
      phase += delta_phase;
      if (phase >= 360.0f) phase -= 360.0f;

      float u = 0.5f + 0.5f * amp * SinDeg((int)phase);
      float v = 0.5f + 0.5f * amp * SinDeg((int)phase - 120);
      float w = 0.5f + 0.5f * amp * SinDeg((int)phase + 120);

      BLDC_WritePwm(u, v, w);
}

/*
ベクトル制御
*/

static inline float BLDC_AdcToRadians(uint16_t adc_val) {
      // エンコーダー値の実際の最大値を求める
      static uint16_t max_adc_val = 4000;
      if (adc_val > max_adc_val) max_adc_val = adc_val;

      // エンコーダー値を最大値に合わせて補正
      uint16_t correction_adc_val = adc_val * (float)(MAX_ADC_VAL / max_adc_val);

      // ADCの最大値4095を2πで割る
      const double conversion_factor = (2.0f * PI) / max_adc_val;

      return (float)correction_adc_val * conversion_factor;
}

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, float target_speed) {
      // エンコーダ値をラジアン(0〜2π)に変換
      svc->mech_theta = BLDC_AdcToRadians(encoder_value);

      // 電気角度を計算
      svc->elec_theta = svc->mech_theta * svc->pole_pairs;
      svc->elec_theta = fmodf(svc->elec_theta, 2.0f * PI);  // 0〜2πの範囲に制限

      svc->amp = target_speed;

      // 正弦波を生成
      float u = 0.5f + 0.5f * svc->amp * Sin(svc->elec_theta);
      float v = 0.5f + 0.5f * svc->amp * Sin(svc->elec_theta - (2.0f * PI / 3.0f));
      float w = 0.5f + 0.5f * svc->amp * Sin(svc->elec_theta + (4.0f * PI / 3.0f));

      BLDC_WritePwm(u, v, w);

      // printf("Elec Theta: %.2f, U: %.2f, V: %.2f, W: %.2f\n",
      //        svc->elec_theta, u, v, w);
}