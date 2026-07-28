#ifndef APP_H_
#define APP_H_

#include <stdio.h>

#include "adc.h"
#include "bldc.h"
#include "config.h"
#include "digitalinout.h"
#include "lpf.h"
#include "main.h"
#include "pwm_out.h"
#include "serial.h"
#include "stdbool.h"
#include "timer.h"

void Setup();
void MainApp();
void CurrentSense_Init();            // 電流センシングADC1をPWM同期トリガで初期化
void CurrentSenseTest();             // 電流計測テスト(停止状態)
void ForcedCommutationCurrentTest();  // 強制転流しながら電流計測するテスト

#endif  // APP_H_