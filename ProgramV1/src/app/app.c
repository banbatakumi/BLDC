#include "app.h"

DigitalOut LED3;
DigitalOut LED4;

void setup() {
      DigitalOut_Init(&LED3, GPIOB, LED3_Pin);  // LED3の初期化
      DigitalOut_Init(&LED4, GPIOB, LED4_Pin);  // LED4の初期化
}

void main_app() {
      while (1) {
            DigitalOut_Write(&LED3, 1);  // LED3を点灯
            DigitalOut_Write(&LED4, 1);  // LED4を点灯
            HAL_Delay(500);              // 500ミリ秒待機
            DigitalOut_Write(&LED3, 0);  // LED3を消灯
            DigitalOut_Write(&LED4, 0);  // LED4を消灯
            HAL_Delay(500);              // 500ミリ秒待機
      }
}