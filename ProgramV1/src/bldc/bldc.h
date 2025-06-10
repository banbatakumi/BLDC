#ifndef BLDC_H_
#define BLDC_H_

#include "main.h"
#include "pwm_out.h"

void BLDC_Init();

void BLDC_Drive(float speed);

#endif  // BLDC_H_