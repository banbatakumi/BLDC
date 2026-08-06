#ifndef APP_H_
#define APP_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "bldc.h"
#include "config.h"
#include "digitalinout.h"
#include "lpf.h"
#include "main.h"
#include "pwm_out.h"
#include "serial.h"
#include "serial_protocol.h"
#include "timer.h"

// app は外部との入出力 (シリアル通信・LED・スイッチ・保護判定) を担当する。
// モーター制御そのものは src/bldc 側の20kHz割り込みの中で完結している。
void Setup(void);
void MainApp(void);

#endif  // APP_H_
