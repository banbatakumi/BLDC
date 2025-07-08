#ifndef BLDC_H_
#define BLDC_H_

#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f  // 最大デューティ比

void BLDC_Init();

void BLDC_OpenLoopDrive(float amp, float freq);

#endif  // BLDC_H_