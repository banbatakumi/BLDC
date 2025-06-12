#include "app.h"

DigitalOut LED3;
DigitalOut LED4;
PwmOut LED2;
PwmOut LED1;

void setup() {
      DigitalOut_Init(&LED3, GPIOB, LED3_Pin);    // LED3の初期化
      DigitalOut_Init(&LED4, GPIOB, LED4_Pin);    // LED4の初期化
      PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);  // LED2のPWM初期化
      PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);  // LED1のPWM初期化

      BLDC_Init();
}

void main_app() {
      while (1) {
            BLDC_OpenLoopDrive(0.8, 50.0f);
      }
}