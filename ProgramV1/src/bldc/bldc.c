#include "bldc.h"

PwmOut u_pwm;
PwmOut v_pwm;
PwmOut w_pwm;

void BLDC_Init() {
      PwmOut_Init(&u_pwm, &htim3, TIM_CHANNEL_1);
      PwmOut_Init(&v_pwm, &htim3, TIM_CHANNEL_2);
      PwmOut_Init(&w_pwm, &htim3, TIM_CHANNEL_3);
}

void BLDC_Drive(float speed) {
      PwmOut_Write(&u_pwm, speed);
      PwmOut_Write(&v_pwm, speed);
      PwmOut_Write(&w_pwm, speed);
}