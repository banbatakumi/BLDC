#ifndef FOC_H_
#define FOC_H_

#include "mymath.h"

#define INV_SQRT3 0.5773502691896258f  // 1/√3
#define SQRT3 1.7320508075688772f

// mymath.h の PI / TWO_PI は double リテラルなので、単精度FPUしか持たない
// Cortex-M4 では式全体がソフトウェアの倍精度演算に落ちてしまう。
// 20kHzの割り込みの中ではこちらの float 版を使うこと。
#define PI_F 3.14159265f
#define TWO_PI_F 6.28318531f

// Cortex-M4F の VSQRT.F32 を直接使う平方根。
// libm の sqrtf は errno と NaN の面倒を見るぶん関数呼び出しが2段挟まるので、
// 20kHzの制御ループでは1命令で済むこちらを使う。
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

// CMSIS-DSPの高速三角関数 (512点テーブル + 線形補間)。
// mymath.h の Sin/Cos は1度刻みのテーブルなのでFOCのPark変換には粗すぎる。
// arm_math.h はヘッダが巨大なのでプロトタイプだけ宣言する (float32_t == float)。
float arm_sin_f32(float x);
float arm_cos_f32(float x);

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

// 電気角のsin/cosをまとめて計算する
static inline void FOC_SinCos(float theta, float* sin_t, float* cos_t) {
  *sin_t = arm_sin_f32(theta);
  *cos_t = arm_cos_f32(theta);
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
