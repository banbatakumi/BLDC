#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

Timer dt_timer;

void BLDC_Init() {
      PwmOut_Init(&u_pwm, &htim1, TIM_CHANNEL_1);
      PwmOut_Init(&v_pwm, &htim1, TIM_CHANNEL_2);
      PwmOut_Init(&w_pwm, &htim1, TIM_CHANNEL_3);

      Timer_Init(&dt_timer);
}

static inline void BLDC_WritePwm(float u, float v, float w) {
      // デューティ比の制限
      if (u > MAX_DUTY) u = MAX_DUTY;
      if (v > MAX_DUTY) v = MAX_DUTY;
      if (w > MAX_DUTY) w = MAX_DUTY;

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
