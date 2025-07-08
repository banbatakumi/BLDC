#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
Timer timer;

uint16_t adc_val[3];  // ADCの値を格納する配列

void setup() {
      printf("Hello World\n");
      printf("SystemCoreClock = %ld\n", SystemCoreClock);

      // ADCの初期化
      HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&adc_val, 3);
      for (uint8_t i = 0; i < 3; i++) {
            while (!(adc_val[i] > 0));
      }
      printf("ADC_DMA start\n");

      // LEDの初期化
      PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);
      PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);
      PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);
      PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);

      BLDC_Init();
}

void main_app() {
      while (1) {
            BLDC_OpenLoopDrive(0.15, 20);
      }
}