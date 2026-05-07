#ifndef BLDC_H_
#define BLDC_H_

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "flash.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f                     // 最大デューティ比
#define MIN_DUTY 0.01f                     // 最小デューティ比
#define MAX_ADC_VAL 4095                   // ADCの最大値(12bit)
#define SPEED_LPF 0.5f                     // 速度のローパスフィルタ係数
#define SPEED_LPF_INV 0.5f                 // (1.0 - SPEED_LPF) 事前計算値
#define AMP_LPF_COEF 0.4f                  // 振幅ローパスフィルタ係数
#define AMP_VOLT_LPF_COEF 0.6f             // 電圧振幅ローパスフィルタ係数
#define K_ENC_LPF 0.015f                   // エンコーダのローパスフィルタ係数ゲイン
#define K_ADV 0.01f                        // 進角ゲイン
#define ADC2RADIAN 0.0015339807878856412f  // ADC値をラジアンに変換する係数(2π/4096)
#define MAX_SPEED 100.0f                   // 最大速度 [rad/s]
#define MAX_ACCEL 50.0f                    // 最大加速度 [rad/s^2]

typedef struct {
  uint32_t max_encoder_val;    // エンコーダーの最大値
  float encoder_offset_theta;  // エンコーダーのオフセット値
} BLDCFlashData;

// 構造体
typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float output_limit;
} PIDController;

typedef struct {
  float dt;                    // 制御周期 [s]
  float amp;                   // 電圧振幅 [0 to 1]
  float amp_volt;              // 電圧振幅 [v]
  float encoder_offset_theta;  // エンコーダオフセット値
  uint16_t max_encoder_val;
  float adc_correction_factor;
  float mech_theta;            // 機械角度 [rad]
  float elec_theta;            // 電気角度 [rad]
  float speed;                 // 速度 [rad/s]
  uint8_t pole_pairs;          // 極対数 (磁石の数/2)
  PIDController speed_pid;     // 速度制御用PID
  PIDController position_pid;  // 位置制御用PID
} SensoredVectorControl;

void BLDC_Init(SensoredVectorControl* svc, bool do_set_encoder, uint16_t* encoder_val);

void BLDC_Stop(bool brake);

void BLDC_OpenLoopDrive(float amp, float freq);

void BLDC_SensoredVectorControlDrive(SensoredVectorControl* svc, uint16_t encoder_value, float supply_volt);

void BLDC_SpeedControl(SensoredVectorControl* svc, float target_speed);
void BLDC_PositionControl(SensoredVectorControl* svc, float target_position);
void BLDC_TorqueControl(SensoredVectorControl* svc, float target_torque);

#endif  // BLDC_H_