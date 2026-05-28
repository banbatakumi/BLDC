#ifndef BLDC_H_
#define BLDC_H_

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "config.h"
#include "flash.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "timer.h"

#define MAX_DUTY 0.99f                     // 最大デューティ比
#define MIN_DUTY 0.01f                     // 最小デューティ比
#define MAX_ADC_VAL 4095                   // ADCの最大値(12bit)
#define SPEED_LPF 0.4f                     // 角速度のローパスフィルタ係数
#define SPEED_LPF_INV 0.6f                 // (1.0 - SPEED_LPF) 事前計算値
#define ACCEL_LPF 0.6f                     // 角加速度のローパスフィルタ係数
#define ACCEL_LPF_INV 0.4f                 // (1.0 - ACCEL_LPF) 事前計算値
#define AMP_LPF_COEF 0.4f                  // 振幅ローパスフィルタ係数
#define AMP_VOLT_LPF_COEF 0.6f             // 電圧振幅ローパスフィルタ係数
#define K_ENC_LPF 0.015f                   // エンコーダのローパスフィルタ係数ゲイン
#define K_ADV 0.01f                        // 進角ゲイン
#define ELEC_THETA_OFFSET (PI)             // 電気角の位相合わせ用オフセット
#define ADC2RADIAN 0.0015339807878856412f  // ADC値をラジアンに変換する係数(2π/4096)
#define POSITION_DEADBAND_RAD 0.025f       // 位置制御の停止判定誤差 [rad]
#define POSITION_SETTLE_SPEED_RAD_S 0.05f  // 位置制御の停止判定角速度 [rad/s]
#define POSITION_INTEGRAL_STOP_RAD 0.05f   // 位置制御で積分を止める誤差 [rad]

typedef struct {
  uint32_t max_encoder_val;    // エンコーダーの最大値
  uint32_t min_encoder_val;    // エンコーダーの最小値
  float encoder_offset_theta;  // エンコーダーのオフセット値
} BLDCFlashData;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float d_term;
  float d_lpf;
  float output_limit;
} PIDController;

typedef struct {
  bool enable;                 // モーターを停止するかどうか
  float dt;                    // 制御周期 [s]
  float amp_volt;              // 電圧振幅 [v]
  float encoder_offset_theta;  // エンコーダオフセット値
  uint16_t max_encoder_val;
  uint16_t min_encoder_val;
  float mech_theta;            // 機械角度 [rad]
  float elec_theta;            // 電気角度 [rad]
  float angular_speed;         // 角速度 [rad/s]
  float angular_accel;         // 角加速度 [rad/s^2]
  uint8_t pole_pairs;          // 極対数 (磁石の数/2)
  PIDController speed_pid;     // 角速度制御用PID
  PIDController position_pid;  // 位置制御用PID
} SensoredVectorControl;

void BLDC_Init(bool do_set_encoder, uint16_t* encoder_val);
void BLDC_Stop();
void BLDC_OpenLoopDrive(float amp, float freq);
void BLDC_SensoredVectorControlDrive(uint16_t encoder_value, float supply_volt);
void BLDC_AngularSpeedControl(float target_angular_speed);
void BLDC_PositionControl(float target_position);
void BLDC_VoltageControl(float target_volt);

float BLDC_GetAngularSpeed(void);
float BLDC_GetAngularAccel(void);
float BLDC_GetMechTheta(void);
float BLDC_GetElecTheta(void);
float BLDC_GetAmpVolt(void);

#endif  // BLDC_H_
