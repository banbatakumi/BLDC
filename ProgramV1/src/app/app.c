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
            BLDC_Drive(0.01f);           // BLDCモーターを50%の速度で駆動
            DigitalOut_Write(&LED3, 1);  // LED3を点灯
            DigitalOut_Write(&LED4, 1);  // LED4を点灯
            PwmOut_Write(&LED2, 0.1f);   // LED2のデューティ比を50%に設定

            HAL_Delay(500);
            // BLDC_Drive(1);               // 500ミリ秒待機
            DigitalOut_Write(&LED3, 0);  // LED3を消灯
            DigitalOut_Write(&LED4, 0);  // LED4を消灯
            PwmOut_Write(&LED2, 0.0f);   // LED2のデューティ比を0%に設定
            HAL_Delay(500);              // 500ミリ秒待機
      }
}