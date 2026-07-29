#ifndef BLDC_H_
#define BLDC_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "current_sense.h"
#include "flash.h"
#include "foc.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"

// センサ付きベクトル制御 (FOC)
//
// 制御の流れ:
//   TIM1(20kHz) → PWM同期でADC1がU/V相電流を1点サンプリング
//     → DMA転送完了割り込み (20kHz, 電流ループ)
//         Clarke変換 → Park変換 → 電流PI制御 → 逆Park変換 → SVPWM → PWM出力
//     → 20回に1回 (1kHz) 外側ループ (速度制御 / 位置制御) がIq指令を更新
//
// app 側は BLDC_SetSupplyVolt() でセンサ値を渡し、BLDC_XxxControl() で
// 目標値を指示するだけでよい。実際の制御は全て割り込みの中で走る。

#define MAX_ADC_VAL 4095    // ADCの最大値(12bit)
#define ACCEL_LPF 0.6f      // 角加速度のローパスフィルタ係数
#define ACCEL_LPF_INV 0.4f  // (1.0 - ACCEL_LPF) 事前計算値

#define POSITION_DEADBAND_RAD 0.05f        // 位置制御の停止判定誤差 [rad]
#define POSITION_SETTLE_SPEED_RAD_S 0.05f  // 位置制御の停止判定角速度 [rad/s]
#define POSITION_INTEGRAL_STOP_RAD 0.05f   // 位置制御で積分を止める誤差 [rad]

typedef enum {
  BLDC_MODE_STOP = 0,  // 停止 (全相デューティ0.5)
  BLDC_MODE_SPEED,     // 角速度制御
  BLDC_MODE_POSITION,  // 位置制御
  BLDC_MODE_TORQUE,    // トルク(Iq)制御
  BLDC_MODE_BRAKE,     // ブレーキ
} BLDCMode;

// フラッシュに保存する校正値。
// 項目を追加/変更したら BLDC_FLASH_MAGIC も変えること。旧フォーマットのデータを
// 新フォーマットとして読むと、追加した項目にゴミが入って静かに誤動作する。
#define BLDC_FLASH_MAGIC 0x424C4432UL  // "BLD2"

typedef struct {
  uint32_t magic;              // BLDC_FLASH_MAGIC。フォーマット判定用
  uint32_t max_encoder_val;    // エンコーダーの最大値
  uint32_t min_encoder_val;    // エンコーダーの最小値
  float encoder_offset_theta;  // エンコーダーのオフセット値 [rad]
  float encoder_dead_zone;     // レール飽和で角度が読めない区間の幅 [rad]
} BLDCFlashData;

// 過電流保護が働いた瞬間の状態。原因の切り分けに使う。
typedef struct {
  float peak_current;   // 検出した相電流のピーク [A]
  float iu, iv, iw;     // そのときの相電流 [A]
  float id, iq;         // dq軸電流 [A]
  float target_iq;      // q軸電流指令 [A]
  float vd, vq;         // dq軸電圧指令 [V]
  float mech_theta;     // 機械角 [rad]
  float angular_speed;  // 角速度 [rad/s]
  uint16_t raw_u;       // 生ADC値 (U相 = PA1/SENSEC)
  uint16_t raw_v;       // 生ADC値 (V相 = PA0/SENSEB)
} BLDCTripInfo;

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

// encoder_val は ADC2 の DMA が非同期に書き換えるので volatile であること
void BLDC_Init(bool do_set_encoder, volatile uint16_t* encoder_val);

// app から呼ぶセンサ入力
void BLDC_SetSupplyVolt(float supply_volt);

// app から呼ぶ指令
void BLDC_Stop(void);
void BLDC_AngularSpeedControl(float target_angular_speed);  // [rad/s]
void BLDC_PositionControl(float target_position);           // [rad]
void BLDC_TorqueControl(float target_current);              // Iq指令 [A]
void BLDC_BrakeControl(float brake_current);                // 制動電流の大きさ [A]

// 状態の取得
float BLDC_GetMechTheta(void);              // 機械角 [rad]
float BLDC_GetElecTheta(void);              // 電気角 [rad]
float BLDC_GetAngularSpeed(void);           // 角速度 [rad/s]
float BLDC_GetAngularAccel(void);           // 角加速度 [rad/s^2]
float BLDC_GetId(void);                     // d軸電流 [A]
float BLDC_GetIq(void);                     // q軸電流 [A]
float BLDC_GetTargetIq(void);               // q軸電流指令 [A]
float BLDC_GetVd(void);                     // d軸電圧 [V]
float BLDC_GetVq(void);                     // q軸電圧 [V]
bool BLDC_IsOvercurrent(void);              // 過電流保護が働いたか
void BLDC_GetTripInfo(BLDCTripInfo* info);  // 保護が働いた瞬間の状態を取得
void BLDC_ClearOvercurrent(void);           // 過電流保護の解除

// 相電流ピークの記録。実際にどこまで電流が振れているかを確認するのに使う。
float BLDC_GetPeakCurrent(void);
void BLDC_ResetPeakCurrent(void);

// 角度追従オブザーバの調査用データ。エンコーダの継ぎ目で何が起きているかが分かる。
typedef struct {
  uint32_t saturated;    // 飽和で棄却したサンプル数 (AS5600の出力がレールに張り付いた)
  uint32_t glitch;       // イノベーション過大で棄却したサンプル数 (遷移グリッチ等)
  float max_innovation;  // 採用したイノベーションのピーク [rad]
} BLDCEncoderStats;

// max_innovation が継ぎ目の段差の大きさそのもの。0.01rad 程度なら校正は良好で、
// 0.05rad を超えるようなら encoder_scale (min/max) の精度が足りていない。
void BLDC_GetEncoderStats(BLDCEncoderStats* stats);
void BLDC_ResetEncoderStats(void);

#endif  // BLDC_H_
