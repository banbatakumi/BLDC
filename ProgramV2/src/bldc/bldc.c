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

      svc->pole_pairs = POLE_PAIRS;
      svc->amp = 0;
      svc->mech_theta = 0;
      svc->elec_theta = 0;

      // PIDコントローラの初期化
      svc->speed_pid.kp = 0.001;  // 比例ゲイン
      svc->speed_pid.ki = 0.1;    // 積分ゲイン
      svc->speed_pid.kd = 0;      // 微分ゲイン
      svc->speed_pid.integral = 0;
      svc->speed_pid.prev_error = 0;
      svc->speed_pid.output_limit = 0.5;

      svc->position_pid.kp = 0.6;  // 比例ゲイン
      svc->position_pid.ki = 0.2;  // 積分ゲイン
      svc->position_pid.kd = 0;    // 微分ゲイン
      svc->position_pid.integral = 0;
      svc->position_pid.prev_error = 0;
      svc->position_pid.output_limit = 0.5;
}

static inline double BLDC_GetEncoder(uint16_t adc_val, double encoder_zero_theta) {
      static uint16_t max_adc_val = 4023;
      // if (adc_val > max_adc_val) max_adc_val = adc_val;

      uint16_t correction_adc_val = adc_val * ((double)MAX_ADC_VAL / max_adc_val);
      double theta = (double)correction_adc_val * (TWO_PI / MAX_ADC_VAL);
      theta = NormalizeRadians(theta - encoder_zero_theta);  // 0〜2πの範囲に正規化

      // ローパスフィルタ
      return theta;
}

bool BLDC_SetEncoderZero(SensoredVectorControl* svc, uint16_t encoder_value) {
      static bool is_first = true;
      if (is_first) {
            Timer_Init(&encoder_offset_timer);
            Timer_Reset(&encoder_offset_timer);
            is_first = false;
      }
      static uint16_t cnt = 0;
      static double theta_sum = 0;
      if (Timer_Read(&encoder_offset_timer) < 1) {
            BLDC_OpenLoopDrive(0.3, 0);
            cnt = 0;
            theta_sum = 0;
      } else if (cnt < 100) {
            theta_sum += BLDC_GetEncoder(encoder_value, 0);
            HAL_Delay(1);
            cnt++;
      } else {
            svc->encoder_zero_theta = theta_sum * 0.01f;  // エンコーダゼロ点を
            BLDC_OpenLoopDrive(0, 0);
            printf("encoder_zero_theta: %.2f\n", svc->encoder_zero_theta);
            is_first = true;
            return true;
      }
      return false;
}

static inline void BLDC_WritePwm(double u, double v, double w) {
      // デューティ比の制限
      u = Constrain(u, 0, MAX_DUTY);
      v = Constrain(v, 0, MAX_DUTY);
      w = Constrain(w, 0, MAX_DUTY);

      PwmOut_Write(&u_pwm, u);
      PwmOut_Write(&v_pwm, v);
      PwmOut_Write(&w_pwm, w);
}

void BLDC_OpenLoopDrive(double amp, double freq) {
      static double phase = 0;
      double dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);

      double delta_phase = TWO_PI * freq * dt;
      phase += delta_phase;
      if (phase >= TWO_PI) phase -= TWO_PI;
      if (phase < 0) phase += TWO_PI;
      phase = HALF_PI;

      double u = 0.5 + 0.5 * amp * Sin(phase);
      double v = 0.5 + 0.5 * amp * Sin(phase + (TWO_PI / 3.0));
      double w = 0.5 + 0.5 * amp * Sin(phase - (TWO_PI / 3.0));
      BLDC_WritePwm(u, v, w);
}

/*
ベクトル制御
*/

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
      speed = speed * (1 - lpf) + pre_speed * lpf;
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

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value) {
      svc->dt = Timer_Read(&dt_timer);
      Timer_Reset(&dt_timer);

      // エンコーダ値を処理
      svc->mech_theta = BLDC_GetEncoder(encoder_value, svc->encoder_zero_theta);  // エンコーダ値をラジアン(0〜2π)に変換
      svc->speed = BLDC_GetSpeed(svc->mech_theta, svc->dt);                       // 速度を計算

      // 電気角度を計算
      svc->elec_theta = svc->mech_theta * svc->pole_pairs + svc->speed * K_ADV;  // 電気角度を計算
      svc->elec_theta = NormalizeRadians(svc->elec_theta);                       // 電気角度を0〜2πに正規化
      // printf("elec_theta: %.2f, speed: %.2f, amp: %.2f\n", svc->elec_theta, svc->speed, svc->amp);

      // 正弦波を生成
      double u = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta);
      double v = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta + (TWO_PI / 3.0));
      double w = 0.5 + 0.5 * svc->amp * Sin(svc->elec_theta - (TWO_PI / 3.0));
      BLDC_WritePwm(u, v, w);
      // printf("mech_theta: %.2f, elec_theta: %.2f, speed: %.2f, amp: %.2f\n",
      //        svc->mech_theta, svc->elec_theta, svc->speed, svc->amp);
}

void BLDC_SpeedControl(SensoredVectorControl* svc, double target_speed) {
      double ff_term = K_FF * target_speed;
      svc->amp = -BLDC_PIDControl(&svc->speed_pid, target_speed - svc->speed, svc->dt);  // + ff_term;
}

void BLDC_PositionControl(SensoredVectorControl* svc, double target_position) {
      target_position = NormalizeRadians(target_position);  // 目標位置を0〜2πの範囲に制限
      // 位置制御のためのPID計算
      double error = target_position - svc->mech_theta;  // 目標位置と現在位置の誤差
                                                         // 0と2πのまたぎ対策
      if (error > PI) error -= TWO_PI;
      if (error < -PI) error += TWO_PI;
      svc->amp = -BLDC_PIDControl(&svc->position_pid, error, svc->dt);
}