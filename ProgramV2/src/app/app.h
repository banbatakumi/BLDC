#ifndef APP_H_
#define APP_H_

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

#endif  // APP_H_