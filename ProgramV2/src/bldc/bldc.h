#ifndef BLDC_H_
#define BLDC_H_

#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f    // 最大デューティ比
#define MAX_ADC_VAL 4095  // ADCの最大値
#define lpf 0.2
#define K_ADV 0.005f  // 進角ゲイン
#define K_FF 0.001f   // フィードフォワードゲイン
// 構造体
typedef struct {
      double kp;            // 比例ゲイン
      double ki;            // 積分ゲイン
      double kd;            // 微分ゲイン
      double integral;      // 積分項
      double prev_error;    // 前回の誤差
      double output_limit;  // 出力制限
} PIDController;

typedef struct {
      double dt;                   // 制御周期 [s]
      double amp;                  // 電圧振幅
      double encoder_zero_theta;   // エンコーダゼロ点
      double mech_theta;           // 機械角度 [rad]
      double elec_theta;           // 電気角度 [rad]
      double speed;                // 速度 [rad/s]
      uint8_t pole_pairs;          // 極対数
      PIDController speed_pid;     // 速度制御用PID
      PIDController position_pid;  // 位置制御用PID
} SensoredVectorControl;

void BLDC_Init(SensoredVectorControl* svc);

bool BLDC_SetEncoderZero(SensoredVectorControl* svc, uint16_t encoder_value);

void BLDC_OpenLoopDrive(double amp, double freq);

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value);

void BLDC_SpeedControl(SensoredVectorControl* svc, double target_speed);
void BLDC_PositionControl(SensoredVectorControl* svc, double target_position);

#endif  // BLDC_H_