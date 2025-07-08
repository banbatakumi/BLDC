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
      svc->speed_pid.kp = 0.005f;  // 比例ゲイン
      svc->speed_pid.ki = 0.02f;   // 積分ゲイン
      svc->speed_pid.kd = 0.0f;    // 微分ゲイン
      svc->speed_pid.integral = 0.0f;
      svc->speed_pid.prev_error = 0.0f;
      svc->speed_pid.output_limit = 0.7;
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

static inline float BLDC_GetEncoder(uint16_t adc_val) {
      // エンコーダー値の実際の最大値を求める
      static uint16_t max_adc_val = 4000;
      if (adc_val > max_adc_val) max_adc_val = adc_val;

      // エンコーダー値を最大値に合わせて補正
      uint16_t correction_adc_val = adc_val * (float)(MAX_ADC_VAL / max_adc_val);

      // ADCの最大値4095を2πで割る
      const double conversion_factor = (2.0f * PI) / max_adc_val;

      return (float)correction_adc_val * conversion_factor;
}

static inline float BLDC_GetSpeed(float theta, float dt) {
      static float prev_theta = 0.0f;

      float delta_theta = theta - prev_theta;

      // 0と2πの境目を跨いだ場合の補正
      if (delta_theta > PI) {
            delta_theta -= 2.0f * PI;
      } else if (delta_theta < -PI) {
            delta_theta += 2.0f * PI;
      }

      float speed = delta_theta / dt;
      prev_theta = theta;

      return speed;
}
static inline float BLDC_PIDControl(PIDController* pid, float error, float dt) {
      // 比例項
      float p_term = pid->kp * error;

      // 積分項
      pid->integral += pid->ki * error * dt;
      if (pid->integral > pid->output_limit) {
            pid->integral = pid->output_limit;
      } else if (pid->integral < -pid->output_limit) {
            pid->integral = -pid->output_limit;
      }

      // 微分項
      float d_term = pid->kd * (error - pid->prev_error) / dt;
      pid->prev_error = error;

      // 出力の計算
      float output = p_term + pid->integral + d_term;
      if (output > pid->output_limit) {
            output = pid->output_limit;
      } else if (output < -pid->output_limit) {
            output = -pid->output_limit;
      }

      return output;
}

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, float target_speed) {
      float dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);

      // エンコーダ値を処理
      svc->mech_theta = BLDC_GetEncoder(encoder_value);  // エンコーダ値をラジアン(0〜2π)に変換
      svc->speed = BLDC_GetSpeed(svc->mech_theta, dt);   // 速度を計算

      // 電気角度を計算
      svc->elec_theta = svc->mech_theta * svc->pole_pairs;
      svc->elec_theta = fmodf(svc->elec_theta, 2.0f * PI);  // 0〜2πの範囲に制限

      svc->amp = -BLDC_PIDControl(&svc->speed_pid, target_speed - svc->speed, dt);
      if (target_speed > 0 && svc->amp > 0.0f) svc->amp = 0.0f;
      if (target_speed < 0 && svc->amp < 0.0f) svc->amp = 0.0f;
      // svc->amp = -0.2;

      // 正弦波を生成
      float u = 0.5f + 0.5f * svc->amp * Sin(svc->elec_theta);
      float v = 0.5f + 0.5f * svc->amp * Sin(svc->elec_theta - (2.0f * PI / 3.0f));
      float w = 0.5f + 0.5f * svc->amp * Sin(svc->elec_theta + (4.0f * PI / 3.0f));

      BLDC_WritePwm(u, v, w);
      // printf("speed: %3f\n", svc->speed);
      printf("amp: %2f\n", svc->amp);
}