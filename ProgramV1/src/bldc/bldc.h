#ifndef BLDC_H_
#define BLDC_H_

#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

void BLDC_Init();

void BLDC_OpenLoopDrive(float amp, float freq);

static inline void BLDC_WritePwm(float u, float v, float w);

#endif  // BLDC_H_