#ifndef CURRENT_SENSE_H_
#define CURRENT_SENSE_H_

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

// PWM同期のローサイドシャント相電流計測 (INA181A1)
//
// 基板の配線 (CurcuitV4):
//   TIM1_CH1(PA8/PA7)  → INHC/INLC → OUTC : ソフト上の U相 (u_pwm)
//   TIM1_CH2(PA9/PB0)  → INHB/INLB → OUTB : ソフト上の V相 (v_pwm)
//   TIM1_CH3(PA10/PB1) → INHA/INLA → OUTA : ソフト上の W相 (w_pwm)
// シャントは OUTB と OUTC の下側にしか入っていない。
//   PA0/ADC1_IN1 = SENSEB = OUTB の下側シャント → V相電流
//   PA1/ADC1_IN2 = SENSEC = OUTC の下側シャント → U相電流
// W相(OUTA)はシャント無しなのでキルヒホッフ則 Iw = -(Iu + Iv) で求める。

void CurrentSense_Init(void);       // ADC1をTIM1同期トリガ + DMAで起動する
void CurrentSense_EnableInterrupt(void);   // 変換完了割り込み(制御ループ)を有効化
void CurrentSense_DisableInterrupt(void);  // 変換完了割り込みを無効化

// 無電流状態でゼロ点(オフセット)を校正する。
// 呼ぶ前に必ず全相のデューティを 0.5 に揃えて電流を止めておくこと。
void CurrentSense_Calibrate(void);

// 最新の相電流を取得する [A]
void CurrentSense_Read(float* iu, float* iv, float* iw);

// 生のADC値を取得する (原因調査用)
void CurrentSense_GetRaw(uint16_t* raw_u, uint16_t* raw_v);

void CurrentSense_GetOffset(float* offset_u, float* offset_v);
bool CurrentSense_IsOffsetValid(void);

#endif  // CURRENT_SENSE_H_
