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
#include "profile.h"
#include "tim.h"  // htim1 (PWM出力は CCR 直書きなので lib/pwm_out は使わない)

// センサ付きベクトル制御 (FOC)
//
//   TIM1(20kHz) 同期で ADC1 が U/V相電流をサンプリング
//     → DMA転送完了割り込み (20kHz, 電流ループ)
//         Clarke → Park → 電流PI → 逆Park → SVPWM → PWM出力
//     → 20回に1回 (1kHz) 外側ループ (速度/位置制御) が Iq指令を更新
//
// app は BLDC_SetSupplyVolt() でセンサ値を渡し、BLDC_XxxControl() で目標値を指示するだけでよい。

#define POSITION_DEADBAND_RAD 0.015f       // 位置制御の停止判定誤差 [rad]
#define POSITION_SETTLE_SPEED_RAD_S 0.03f  // 位置制御の停止判定角速度 [rad/s]
#define POSITION_INTEGRAL_STOP_RAD 0.015f  // 位置制御で積分を止める誤差 [rad]

// 位置制御は目標角との偏差をそのまま PID に入れる素の構成 (位置指令のランプや速度制限は持たない)。
// 拘束中の保持トルクは torque_limit で決まるので、ラックエンド等に当て続けても焼かない電流を
// 選ぶ責任は上位の torque_limit にある。ワインドアップは BLDC_PIDControl のバックカリキュレーションが抑える。

typedef enum {
  BLDC_MODE_STOP = 0,  // 停止 (全相デューティ0.5)
  BLDC_MODE_SPEED,     // 角速度制御
  BLDC_MODE_POSITION,  // 位置制御
  BLDC_MODE_TORQUE,    // トルク(Iq)制御
  BLDC_MODE_BRAKE,     // ブレーキ
} BLDCMode;

// フラッシュに保存する校正値。
// 項目を追加/変更したら BLDC_FLASH_MAGIC も変えること (旧データを新フォーマットで読むと誤動作する)。
// BLD3 でモータの電気的パラメータ (R, L, ψm, 角度遅れ) を追加した。
#define BLDC_FLASH_MAGIC 0x424C4433UL  // "BLD3"

typedef struct {
  uint32_t magic;              // BLDC_FLASH_MAGIC。フォーマット判定用
  uint32_t max_encoder_val;    // エンコーダーの最大値
  uint32_t min_encoder_val;    // エンコーダーの最小値
  float encoder_offset_theta;  // エンコーダーのオフセット値 [rad]
  float encoder_dead_zone;     // レール飽和で角度が読めない区間の幅 [rad]
  float motor_r;               // 相抵抗 [Ω]
  float motor_l;               // 相インダクタンス [H]
  float motor_psi;             // 永久磁石の鎖交磁束 [Wb]
  float angle_delay;           // 電気角の実効遅れ [s]
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

// encoder_val は ADC2 の DMA が非同期に書き換えるので volatile。
// supply_volt は実測した母線電圧を渡すこと。SVPWM の電圧→デューティ変換に使うので、
// 実際と違うと指令電圧と実電圧の比がずれ、校正で測るモータ定数も狂う。
void BLDC_Init(bool do_set_encoder, volatile uint16_t* encoder_val, float supply_volt);

void BLDC_SetSupplyVolt(float supply_volt);

// 上位から指定される制限値。指令フレームごと (1kHz) に呼ばれ、次の外側ループから効く。
//   torque_limit_nm : トルク (= Iq) 指令の飽和値 [N・m]。全モードに効く唯一の制限。
//                     速度/位置ループでは出力制限 (アンチワインドアップの基準) になり、
//                     トルク/ブレーキモードでは最終段のクランプになる。
// MD 側の絶対上限 (Kt × MAX_CURRENT) でクランプされ、負の値は 0 に丸める。
// 0 は「制限なし」ではなく「動かない」。起動直後は 0 で、受信するまでモータは回らない。
// 速度の制限は持たない (MAX_ANGULAR_SPEED は上位から変えられないハード保護)。
void BLDC_SetLimits(float torque_limit_nm);

// 実際に適用している制限値 (上位へのエコーバックと表示用)
float BLDC_GetTorqueLimitNm(void);  // [N・m]
float BLDC_GetMaxTorqueNm(void);    // MD 側の絶対上限 = Kt × MAX_CURRENT [N・m]

// app から呼ぶ指令
void BLDC_Stop(void);
void BLDC_AngularSpeedControl(float target_angular_speed);  // [rad/s]
void BLDC_PositionControl(float target_position);           // [rad]
void BLDC_TorqueControl(float target_current);              // Iq指令 [A]
void BLDC_TorqueControlNm(float torque_nm);                 // トルク指令 [N・m] (Ktで換算してIq指令へ)
void BLDC_BrakeControl(float brake_current);                // 制動電流の大きさ [A]
void BLDC_BrakeControlNm(float brake_torque_nm);            // 制動トルクの大きさ [N・m] (Ktで換算)

// トルク定数 Kt = 1.5 × POLE_PAIRS × ψm [N・m/A] (ψm は校正値、磁気飽和なしの近似)
float BLDC_GetTorqueConstant(void);

// 状態の取得
float BLDC_GetMechTheta(void);              // 機械角 [rad]
float BLDC_GetElecTheta(void);              // 電気角 [rad]
float BLDC_GetAngularSpeed(void);           // 角速度 [rad/s]
float BLDC_GetId(void);                     // d軸電流 [A]
float BLDC_GetIq(void);                     // q軸電流 [A]
float BLDC_GetTargetIq(void);               // q軸電流指令 [A]
float BLDC_GetVd(void);                     // d軸電圧 [V]
float BLDC_GetVq(void);                     // q軸電圧 [V]
float BLDC_GetVqFF(void);                   // Vqのうちフィードフォワード分 [V]
bool BLDC_IsOvercurrent(void);              // 過電流保護が働いたか
void BLDC_GetTripInfo(BLDCTripInfo* info);  // 保護が働いた瞬間の状態を取得
void BLDC_ClearOvercurrent(void);           // 過電流保護の解除

// 相電流ピークの記録
float BLDC_GetPeakCurrent(void);
void BLDC_ResetPeakCurrent(void);

// 角度追従オブザーバの調査用データ
typedef struct {
  uint32_t saturated;    // 飽和で棄却したサンプル数 (AS5600の出力がレールに張り付いた)
  uint32_t glitch;       // イノベーション過大で棄却したサンプル数 (遷移グリッチ等)
  float max_innovation;  // 採用したイノベーションのピーク [rad]
} BLDCEncoderStats;

// max_innovation は継ぎ目の段差の大きさ。0.01rad 程度なら校正は良好、
// 0.05rad を超えるなら encoder_scale (min/max) の精度不足。
void BLDC_GetEncoderStats(BLDCEncoderStats* stats);
void BLDC_ResetEncoderStats(void);

// 20kHz制御ループの実行時間プロファイル (config.h の PROFILE_ISR で有効化)
#if PROFILE_ISR
// 割り込みハンドラ全体 (HALのディスパッチ込み) と呼び出し周期。
// stm32f3xx_it.c の DMA1_Channel1_IRQHandler から直接叩くため extern で公開する。
extern Profile bldc_prof_irq;
extern ProfilePeriod bldc_prof_period;

// 集計を表示して最大値と平均をリセットする。elapsed_s は前回表示からの経過時間 [s]。
void BLDC_PrintIsrProfile(float elapsed_s);

// 集計を捨てて測定開始点を揃える
void BLDC_ResetIsrProfile(void);
#endif

// モータ定数の測定 (制御ループが回り始めてから呼ぶ)。
// スイッチ校正と config.h の MEASURE_* から呼ばれる。成功で true、
// 失敗は理由を printf して false (出力値には触らない)。
#if MEASURE_MOTOR_RL || MOTOR_AUTO_CALIBRATION
// PIをバイパスして Vd を直接印加し、L と R を同定する (ロータは動かないが電流は流れる)

bool BLDC_MeasureMotorRL(float* out_r, float* out_l);
#endif
#if MEASURE_CURRENT_STEP
// 電流指令にステップを入れて、閉ループが1次系になっているか確認する (表示のみ)
void BLDC_MeasureCurrentStep(void);
#endif
#if MEASURE_MOTOR_PSI || MOTOR_AUTO_CALIBRATION
// 鎖交磁束 ψm を同定する (ロータが回る)
bool BLDC_MeasureMotorPsi(float* out_psi);
// 電気角の実効遅れを同定する (ロータが回る)。ψm の同定より後に呼ぶこと。
bool BLDC_MeasureAngleDelay(float* out_delay);
#endif

#endif  // BLDC_H_
