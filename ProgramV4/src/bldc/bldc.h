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

#define POSITION_DEADBAND_RAD 0.01f        // 位置制御の停止判定誤差 [rad]
#define POSITION_SETTLE_SPEED_RAD_S 0.02f  // 位置制御の停止判定角速度 [rad/s]
#define POSITION_INTEGRAL_STOP_RAD 0.01f   // 位置制御で積分を止める誤差 [rad]

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

// フラッシュに保存する校正値。スイッチ校正が**全部成功したときだけ**1回で書く。
// 項目を追加/変更したら BLDC_FLASH_MAGIC も変えること (旧データを新フォーマットで読むと誤動作する)。
// BLD3 で電気的パラメータ、BLD4 で慣性 J、BLD5 で補正テーブル・回転方向・J/Kt・摩擦・電流帯域を追加し、
// 既定値を廃止した (未校正なら駆動しない)。
#define BLDC_FLASH_MAGIC 0x424C4435UL  // "BLD5"

typedef struct {
  uint32_t magic;              // BLDC_FLASH_MAGIC。フォーマット判定用
  uint16_t max_encoder_val;    // エンコーダの上側レール [LSB]
  uint16_t min_encoder_val;    // エンコーダの下側レール [LSB]
  uint16_t encoder_margin;     // レール判定の幅 [LSB] (静止ノイズから決める)
  int16_t encoder_dir;         // +1: ADC値の増加 = 電気角の正方向 / -1: 逆
  float encoder_offset_theta;  // 電気角0に対する機械角のオフセット [rad]
  float encoder_dead_zone;     // レール飽和で角度が読めない区間の幅 [rad]
  float motor_r;               // 相抵抗 [Ω]
  float motor_l;               // 相インダクタンス [H]
  float motor_psi;             // 永久磁石の鎖交磁束 [Wb]
  float angle_delay;           // 電気角の実効遅れ [s]
  float j_per_kt;              // J/Kt [A·s²/rad] (速度ループのゲインとFF)
  float friction_current;      // 摩擦トルクを打ち消す電流 [A]
  float current_bw_hz;         // 電流PIの設計帯域 [Hz] (実測が CURRENT_BW_TARGET_HZ になる値)
  int16_t encoder_lut[ENCODER_LUT_SIZE];  // 角度の補正テーブル [2π/65536 rad]
  uint32_t checksum;           // 上の全ワードの和 (補正テーブルは範囲検査できないため)
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

// encoder_buf は ADC2 の DMA が非同期に書き換えるバッファの先頭 (エンコーダ ENCODER_ADC_SAMPLES 個)。
// supply_volt は実測した母線電圧を渡すこと。SVPWM の電圧→デューティ変換に使うので、
// 実際と違うと指令電圧と実電圧の比がずれ、校正で測るモータ定数も狂う。
// do_calibrate が真ならスイッチ校正 (config.h) を行い、成功すればフラッシュに保存する。
void BLDC_Init(bool do_calibrate, volatile uint16_t* encoder_buf, float supply_volt);

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
float BLDC_GetMechTheta(void);     // 機械角 [rad]
float BLDC_GetElecTheta(void);     // 電気角 [rad]
float BLDC_GetAngularSpeed(void);  // 角速度 [rad/s]
float BLDC_GetId(void);            // d軸電流 [A]
float BLDC_GetIq(void);            // q軸電流 [A]
float BLDC_GetTargetIq(void);      // q軸電流指令 [A]
float BLDC_GetVd(void);            // d軸電圧 [V]
float BLDC_GetVq(void);            // q軸電圧 [V]
float BLDC_GetVqFF(void);          // Vqのうちフィードフォワード分 [V]
bool BLDC_IsOvercurrent(void);     // 過電流保護が働いたか
// エンコーダとモータ定数の校正値がそろっているか。偽のときは電気角もゲインも決まっていない
// ので、app は上位の制限値を受け付けない (= 動かさない)。既定値は持たない
bool BLDC_IsCalibrated(void);
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
// 0.05rad を超えるなら補正テーブル・盲点幅の精度不足。
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

#endif  // BLDC_H_
