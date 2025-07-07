#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
Timer timer;

void setup() {
      PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);  // LED1のPWM初期化
      PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);  // LED2のPWM初期化
      PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);  // LED3のPWM初期化
      PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);  // LED4のPWM初期化

      BLDC_Init();
      Timer_Init(&timer);   // タイマーの初期化
      Timer_Reset(&timer);  // タイマーのリセット
}

int flag = 0;
const int max_speed = 100;     // 最大速度
const int min_speed = -100;    // 最小速度
const int acceleration = 300;  // 加速度

void main_app() {
      while (1) {
            int speed;
            if (flag == 0) {
                  speed = Timer_Read(&timer) * acceleration;  // タイマーの読み取り
                  if (speed > max_speed) {
                        if (speed > max_speed * 2) {  // 0.5秒経過したら
                              flag = 1;
                              Timer_Reset(&timer);  // タイマーのリセット
                        }
                        speed = max_speed;  // 最大速度に制限
                  }
            } else if (flag == 1) {
                  speed = max_speed - Timer_Read(&timer) * acceleration;  // タイマーの読み取り
                  if (speed < 0) {
                        flag = 2;
                        Timer_Reset(&timer);  // タイマーのリセット
                  }
            } else if (flag == 2) {
                  speed = -Timer_Read(&timer) * acceleration;  // タイマーの読み取り
                  if (speed < min_speed) {
                        if (speed < min_speed * 2) {  // 0.5秒経過したら
                              flag = 3;
                              Timer_Reset(&timer);  // タイマーのリセット
                        }
                        speed = min_speed;  // 最小速度に制限
                  }
            } else if (flag == 3) {
                  speed = min_speed + Timer_Read(&timer) * acceleration;  // タイマーの読み取り
                  if (speed > 0) {
                        flag = 0;
                        Timer_Reset(&timer);  // タイマーのリセット
                  }
            }

            BLDC_OpenLoopDrive(50 * 0.0028, 10);
      }
}