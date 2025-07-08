#ifndef BLDC_H_
#define BLDC_H_

#include "config.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f    // 最大デューティ比
#define MAX_ADC_VAL 4095  // ADCの最大値
#define lpf 0.3

// 構造体
typedef struct {
      float kp;            // 比例ゲイン
      float ki;            // 積分ゲイン
      float kd;            // 微分ゲイン
      float integral;      // 積分項
      float prev_error;    // 前回の誤差
      float output_limit;  // 出力制限
} PIDController;

typedef struct {
      float amp;                // 電圧振幅
      float mech_theta;         // 機械角度 [rad]
      float elec_theta;         // 電気角度 [rad]
      float speed;              // 速度 [rad/s]
      uint8_t pole_pairs;       // 極対数
      PIDController speed_pid;  // 速度制御用PID
} SensoredVectorControl;

void BLDC_Init(SensoredVectorControl* svc);

void BLDC_OpenLoopDrive(float amp, float freq);

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, float target_speed);

#endif  // BLDC_H_