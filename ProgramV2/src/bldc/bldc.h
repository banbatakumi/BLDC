#ifndef BLDC_H_
#define BLDC_H_

#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f    // 最大デューティ比
#define MAX_ADC_VAL 4095  // ADCの最大値(12bit)
#define SPEED_LPF 0.5     // 速度のローパスフィルタ係数
#define K_ENC_LPF 0.005   // エンコーダのローパスフィルタ係数ゲイン
#define K_ADV 0.01f       // 進角ゲイン
#define K_FF 0.02f        // 速度制御フィードフォワードゲイン
// 構造体
typedef struct {
      double kp;
      double ki;
      double kd;
      double integral;
      double prev_error;
      double output_limit;
} PIDController;

typedef struct {
      double dt;                    // 制御周期 [s]
      double amp;                   // 電圧振幅 [0 to 1]
      double amp_volt;              // 電圧振幅 [v]
      double encoder_offset_theta;  // エンコーダオフセット値
      uint16_t max_encoder_val;
      double mech_theta;           // 機械角度 [rad]
      double elec_theta;           // 電気角度 [rad]
      double speed;                // 速度 [rad/s]
      uint8_t pole_pairs;          // 極対数 (磁石の数/2)
      PIDController speed_pid;     // 速度制御用PID
      PIDController position_pid;  // 位置制御用PID
} SensoredVectorControl;

void BLDC_Init(SensoredVectorControl* svc);

bool BLDC_SetEncoder(SensoredVectorControl* svc, uint16_t encoder_value);

void BLDC_OpenLoopDrive(double amp, double freq);

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, double supply_volt);

void BLDC_SpeedControl(SensoredVectorControl* svc, double target_speed);
void BLDC_PositionControl(SensoredVectorControl* svc, double target_position);

#endif  // BLDC_H_