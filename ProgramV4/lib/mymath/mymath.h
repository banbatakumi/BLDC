#ifndef MYMATH_H_
#define MYMATH_H_

// 汎用の小さな数学ヘルパ。
//
// **三角関数と角度の正規化はここには置かない。** FOC で使うものは
// src/bldc/foc.h の FOC_SinCos / FOC_NormalizeRadians / FOC_AngleDiff にある。
// 以前はここにも1度刻みテーブルの Sin/Cos と double 定数を使う
// NormalizeRadians/GapRadians があったが、
//   - 精度が Park 変換には粗い (1度刻み = 最大0.5°の誤差)
//   - double リテラル経由なので単精度FPUでは倍精度演算に落ちる
//   - foc.h と同じ役割の関数が2組あり、どちらを使うべきか分からない
// という理由で消した。角度を扱うコードは foc.h 側を使うこと。

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// 絶対値。float は VABS.F32 の1命令 (1サイクル) で出る。
//
// ((x) > 0 ? (x) : -(x)) と書くと、float の場合コンパイラは
//   VCMP.F32 → VMRS APSR_nzcv, FPSCR → 条件分岐
// を出す。この VMRS は「FPUのフラグをコアのフラグに転送する」命令で、
// FPUのパイプラインが揃うまで待つため1命令なのに実質4〜6サイクルかかる。
// 分岐まで含めると1回の絶対値で8〜10サイクル、VABS のおよそ10倍。
// 20kHzの制御ループの中で何度も使うので、型に応じて正しい命令へ振り分ける。
// (この書き換えだけで割り込み1回あたり40サイクル前後変わる)
//
// マクロだと引数が2回展開される問題も消える。Abs(*p++) のような書き方が
// 静かに壊れるのを防げる。
static inline float AbsF(float x) { return __builtin_fabsf(x); }
static inline double AbsD(double x) { return __builtin_fabs(x); }
static inline long AbsL(long x) { return (x < 0) ? -x : x; }

#define Abs(x)     \
  _Generic((x),    \
      float: AbsF, \
      double: AbsD,\
      default: AbsL)(x)

#define Constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define Radians(deg) ((deg) * DEG_TO_RAD)
#define Degrees(rad) ((rad) * RAD_TO_DEG)

#endif  // MYMATH_H_
