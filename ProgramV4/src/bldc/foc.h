#ifndef FOC_H_
#define FOC_H_

#include <stdint.h>

#include "mymath.h"

#define INV_SQRT3 0.5773502691896258f  // 1/√3
#define SQRT3 1.7320508075688772f

// mymath.h の PI / TWO_PI は double リテラルで、単精度FPUのCortex-M4ではソフト倍精度演算になる。
// 20kHz割り込み内では float 版を使うこと。
#define PI_F 3.14159265f
#define TWO_PI_F 6.28318531f

// VSQRT.F32 を直接使う平方根 (libm の sqrtf は errno 処理で関数呼び出しが2段挟まる)
static inline float FOC_Sqrt(float x) {
  float result;
  __asm volatile("vsqrt.f32 %0, %1" : "=t"(result) : "t"(x));
  return result;
}

// 角度を 0〜2π に正規化する
static inline float FOC_NormalizeRadians(float rad) {
  while (rad < 0.0f) rad += TWO_PI_F;
  while (rad >= TWO_PI_F) rad -= TWO_PI_F;
  return rad;
}

// 角度差を -π〜+π に正規化する (0と2πの境目を跨いでも正しく求まる)
static inline float FOC_AngleDiff(float a, float b) {
  float diff = a - b;
  while (diff > PI_F) diff -= TWO_PI_F;
  while (diff < -PI_F) diff += TWO_PI_F;
  return diff;
}

// CMSIS-DSP の sin テーブル (512点 + 端の1点)。mymath.h の Sin/Cos は1度刻みで粗すぎる。
// arm_math.h は巨大なので宣言だけここに書く。
#define FOC_SIN_TABLE_SIZE 512
extern const float sinTable_f32[FOC_SIN_TABLE_SIZE + 1];

// 電流PI制御器 (微分項なし)
typedef struct {
  float kp;            // 比例ゲイン [V/A]
  float ki;            // 積分ゲイン [V/(A·s)]
  float integral;      // 積分項 [V]
  float output_limit;  // 出力制限 [V]
} PIController;

static inline void FOC_PI_Reset(PIController* pi) {
  pi->integral = 0.0f;
}

static inline float FOC_PI_Update(PIController* pi, float error, float dt) {
  pi->integral += pi->ki * error * dt;
  pi->integral = Constrain(pi->integral, -pi->output_limit, pi->output_limit);

  float output = pi->kp * error + pi->integral;
  return Constrain(output, -pi->output_limit, pi->output_limit);
}

// 電気角の sin と cos をまとめて計算する (範囲縮約と補間係数を共有できる)。
// 入力の範囲は任意。内部で小数部を取るので、事前に正規化しないこと (while ループの無駄になる)。
static inline void FOC_SinCos(float theta, float* sin_t, float* cos_t) {
  float in = theta * (1.0f / TWO_PI_F);
  in -= (float)(int32_t)in;   // 小数部 (負のまま)
  if (in < 0.0f) in += 1.0f;  // [0,1) に折り返す

  float findex = in * (float)FOC_SIN_TABLE_SIZE;
  uint32_t index = (uint32_t)findex;
  float frac = findex - (float)index;
  index &= (FOC_SIN_TABLE_SIZE - 1);

  // 線形補間
  float s0 = sinTable_f32[index];
  float s1 = sinTable_f32[index + 1];
  *sin_t = s0 + frac * (s1 - s0);

  // cos(θ) = sin(θ + π/2)。128点ずらすだけなので frac を使い回せる
  uint32_t ci = (index + (FOC_SIN_TABLE_SIZE / 4)) & (FOC_SIN_TABLE_SIZE - 1);
  float c0 = sinTable_f32[ci];
  float c1 = sinTable_f32[ci + 1];
  *cos_t = c0 + frac * (c1 - c0);
}

// Clarke変換 (振幅一定型): 3相電流 → 静止直交2軸(α, β)
//   Iu + Iv + Iw = 0 を前提にすると
//     Iα = Iu
//     Iβ = (Iv - Iw)/√3 = (Iu + 2*Iv)/√3
static inline void FOC_Clarke(float iu, float iv, float* i_alpha, float* i_beta) {
  *i_alpha = iu;
  *i_beta = (iu + 2.0f * iv) * INV_SQRT3;
}

// Park変換: 静止(α, β) → ロータと同期して回る(d, q)
static inline void FOC_Park(float i_alpha, float i_beta, float sin_t, float cos_t,
                            float* id, float* iq) {
  *id = i_alpha * cos_t + i_beta * sin_t;
  *iq = -i_alpha * sin_t + i_beta * cos_t;
}

// 逆Park変換: (d, q) → 静止(α, β)
static inline void FOC_InvPark(float vd, float vq, float sin_t, float cos_t,
                               float* v_alpha, float* v_beta) {
  *v_alpha = vd * cos_t - vq * sin_t;
  *v_beta = vd * sin_t + vq * cos_t;
}

// 電圧ベクトル (vd, vq) の大きさを v_max の円内に制限する。
// 戻り値は掛けた縮小率 (制限がかからなければ 1.0)。積分項のアンチワインドアップに使う。
float FOC_LimitVoltageVector(float* vd, float* vq, float v_max);

// 空間ベクトル変調 (中点電圧を重畳する min-max 方式 = 三次高調波重畳)。
// 単純な正弦波変調に比べて母線電圧の利用率が 2/√3 (約15%) 高くなる。
void FOC_SVPWM(float v_alpha, float v_beta, float v_dc, float* du, float* dv, float* dw);

#endif  // FOC_H_
