#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
Timer control_timer;
Timer timer;

uint16_t adc_val[3];  // ADCの値を格納する配列

SensoredVectorControl svc;

Serial pc;

void setup() {
      printf("Hello World\n");
      printf("SystemCoreClock = %ld\n", SystemCoreClock);

      // ADCの初期化
      HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&adc_val, 3);
      for (uint8_t i = 0; i < 3; i++) {
            while (!(adc_val[i] > 0));
      }
      printf("ADC_DMA start\n");
      HAL_Delay(500);

      // LEDの初期化
      PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);
      PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);
      PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);
      PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);

      BLDC_Init(&svc);
      while (BLDC_SetEncoderZero(&svc, adc_val[0]) == false);

      Timer_Init(&control_timer);
      Timer_Reset(&control_timer);

      Timer_Init(&timer);
      Timer_Reset(&timer);

      // Serialの初期化
      Serial_Init(&pc, &huart1, 256, true);
}

void main_app() {
      while (1) {
            // BLDC_OpenLoopDrive(0.1, 0);
            if (Timer_Read(&timer) < 0.3) {
                  BLDC_PositionControl(&svc, 2);  // 目標速度を50.0 rad/sに設定
            } else if (Timer_Read(&timer) < 1) {
                  BLDC_PositionControl(&svc, 0);  // 目標速度を50.0 rad/sに設定
            } else {
                  Timer_Reset(&timer);
            }
            // if (Timer_Read(&timer) < 1) {
            //       BLDC_SpeedControl(&svc, 50.0f);  // 目標速度を0.0 rad/sに設定
            // } else if (Timer_Read(&timer) < 2) {
            //       BLDC_SpeedControl(&svc, -50.0f);  // 目標速度を0.0 rad/sに設定
            // } else {
            //       Timer_Reset(&timer);
            // }
            // BLDC_PositionControl(&svc, 0.0f);
            static uint8_t speed = 0.0f;
            if (Serial_Available(&pc)) {
                  speed = Serial_Read(&pc);
            }
            // BLDC_SpeedControl(&svc, speed);  // 目標速度を0.0 rad/sに設定

            BLDC_SensoredVectorControlDrive(&svc, adc_val[0]);

            // 制御周期の一定化
            // while (Timer_Read(&control_timer) <= CONTROL_PERIOD);
            // Timer_Reset(&control_timer);
      }
}