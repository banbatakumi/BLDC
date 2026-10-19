#ifndef CALIB_MATH_H_
#define CALIB_MATH_H_

// 校正の計算部分 (最小二乗・エンコーダ補正テーブル)。HAL にもハードにも依存しないので、
// ファームウェア (bldc_calibration.c) とホストのテスト (tests/test_calibration.c) が
// **同じこのファイルを include する**。foc.h はインラインアセンブラを含みホストで通らないので使わない。
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define CM_PI 3.14159265f
#define CM_TWO_PI 6.28318531f

// 角度差を -π〜+π に畳む (foc.h の FOC_AngleDiff と同じ)
static inline float CalibMath_Wrap(float a) {
  while (a > CM_PI) a -= CM_TWO_PI;
  while (a < -CM_PI) a += CM_TWO_PI;
  return a;
}

// y = slope·x + intercept の最小二乗。x が NULL なら x = 0,1,2,...。点が足りなければ false
static inline bool CalibMath_LineFit(const float* x, const float* y, uint16_t n, float* slope,
                                     float* intercept) {
  if (n < 2) return false;
  // 平均を引いてから積和を取る (float でも桁落ちしない)
  float mx = 0.0f, my = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    mx += x ? x[i] : (float)i;
    my += y[i];
  }
  mx /= (float)n;
  my /= (float)n;
  float sxx = 0.0f, sxy = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    float dx = (x ? x[i] : (float)i) - mx;
    sxx += dx * dx;
    sxy += dx * (y[i] - my);
  }
  if (!(sxx > 1e-12f)) return false;
  *slope = sxy / sxx;
  if (intercept) *intercept = my - *slope * mx;
  return true;
}

// ステップ応答 i[0..n) (i[0] = ステップ直前) から時定数 [サンプル] を求める。
// 記録は電流LPF (y += a·(x − y)) を通っているので、まず x_k = y_{k−1} + (y_k − y_{k−1})/a で
// LPF を逆算し、1 − x/i_ss が 0.6〜0.1 の区間で ln(1 − x/i_ss) を直線近似して傾き −1/τ から τ を出す。
// 半周期のむだ時間は応答をずらすだけで傾きには効かない。LPF を逆算しないと、τ が LPF の時定数
// (a=0.5 で 72µs) に近いとき大きく読む (τ=100µs で +21%)。逆算はノイズを (2/a − 1) 倍にするが、
// 同期加算と多点の当てはめで均される。
static inline float CalibMath_FitTau(const float* i, uint16_t n, float i_ss, float lpf_a) {
  float xs[64], ys[64];
  uint16_t m = 0;
  if (!(i_ss > 1e-6f) || !(lpf_a > 0.0f)) return 0.0f;
  for (uint16_t k = 1; k < n && m < 64; k++) {
    float x = i[k - 1] + (i[k] - i[k - 1]) / lpf_a;
    float r = 1.0f - x / i_ss;
    if (r < 0.1f && k > 2) break;  // 尾はノイズが勝つので使わない
    if (r > 0.6f || r <= 0.0f) continue;
    xs[m] = (float)k;
    ys[m] = logf(r);
    m++;
  }
  float slope;
  if (m < 2 || !CalibMath_LineFit(xs, ys, m, &slope, 0) || !(slope < 0.0f)) return 0.0f;
  return -1.0f / slope;
}

// 速度の記録 w[0..n) (一定周期) のうち、dir·w が [lo, hi] にある連続区間の傾き [単位/サンプル]。
// 加速と減速で同じ速度帯を使うと、速度に依存する摩擦が差で消える。使った点数を返す。
static inline uint16_t CalibMath_BandSlope(const float* w, uint16_t n, float dir, float lo,
                                           float hi, float* slope) {
  uint16_t first = 0, count = 0;
  for (uint16_t k = 0; k < n; k++) {
    float v = dir * w[k];
    if (v >= lo && v <= hi) {
      if (count == 0) first = k;
      count++;
    } else if (count > 0) {
      break;  // 帯を一度抜けたら終わり (加速と減速を混ぜない)
    }
  }
  if (count < 3 || !CalibMath_LineFit(0, &w[first], count, slope, 0)) return 0;
  return count;
}

// y = psi·x + v0·s の最小二乗 (切片なし、s = ±1 は回転方向)。
// ψm の同定で、回転方向で符号が変わるデッドタイムの電圧誤差 v0 を分離する。
static inline bool CalibMath_FitPsi(const float* x, const float* s, const float* y, uint16_t n,
                                    float* psi, float* v0) {
  float sxx = 0, sxs = 0, sss = 0, sxy = 0, ssy = 0;
  for (uint16_t k = 0; k < n; k++) {
    sxx += x[k] * x[k];
    sxs += x[k] * s[k];
    sss += s[k] * s[k];
    sxy += x[k] * y[k];
    ssy += s[k] * y[k];
  }
  float det = sxx * sss - sxs * sxs;
  if (!(det > 1e-9f * sxx * sss)) return false;
  *psi = (sxy * sss - ssy * sxs) / det;
  *v0 = (ssy * sxx - sxy * sxs) / det;
  return true;
}

// ---------------------------------------------------------------------------
// エンコーダ補正テーブル
// ---------------------------------------------------------------------------
// 入力: 真の機械角 φ_b = (b + 0.5)·2π/N のビンごとに、線形写像した角度 θ_lin の平均 (正転と逆転の平均)。
//       valid[b] が偽のビン (盲点など) は前後から補間する。
// 出力: offset = θ_lin − φ の円平均 [rad] と、θ_lin の等間隔格子 k·2π/N 上の補正量
//       lut[k] = (θ_lin − offset − φ) を 2π/65536 単位で。運転時は φ = θ_lin − offset − lut(θ_lin)。
// 戻り値は隣接ビンの補正量の最大差 [rad] (ロータの滑り・極対数の不一致があると大きくなる)。負ならビン不足。
#define CALIB_LUT_SCALE (CM_TWO_PI / 65536.0f)  // lut の1LSB [rad]

// 等間隔 (ビン中心 (b+0.5)·2π/N) の周期関数 r を φ で線形補間する
static inline float CalibMath_InterpBins(const float* r, uint16_t n, float phi) {
  float f = phi * ((float)n / CM_TWO_PI) - 0.5f;
  while (f < 0.0f) f += (float)n;  // [0, n) (φ は ±数周の範囲にしか来ない)
  while (f >= (float)n) f -= (float)n;
  uint16_t i0 = (uint16_t)f;
  if (i0 >= n) i0 = n - 1;
  float frac = f - (float)i0;
  uint16_t i1 = (uint16_t)((i0 + 1) % n);
  return r[i0] + frac * (r[i1] - r[i0]);
}

// r は作業用 (N 要素)。theta_lin も上書きしない
static inline float CalibMath_BuildLut(const float* theta_lin, const bool* valid, uint16_t n,
                                       float* work_r, float* out_offset, int16_t* lut) {
  // 円平均: 最初の有効ビンを基準にした差の平均
  int16_t ref = -1;
  for (uint16_t b = 0; b < n; b++) {
    if (valid[b]) {
      ref = (int16_t)b;
      break;
    }
  }
  if (ref < 0) return -1.0f;
  const float base = CalibMath_Wrap(theta_lin[ref] - ((float)ref + 0.5f) * (CM_TWO_PI / (float)n));
  float sum = 0.0f;
  uint16_t cnt = 0;
  for (uint16_t b = 0; b < n; b++) {
    if (!valid[b]) continue;
    float err = theta_lin[b] - ((float)b + 0.5f) * (CM_TWO_PI / (float)n);
    work_r[b] = CalibMath_Wrap(err - base);  // base からの差 (小さい)
    sum += work_r[b];
    cnt++;
  }
  if (cnt < n / 2) return -1.0f;
  float mean = sum / (float)cnt;
  float offset = CalibMath_Wrap(base + mean);
  for (uint16_t b = 0; b < n; b++) {
    if (valid[b]) work_r[b] -= mean;
  }

  // 無効ビンを前後の有効ビンから直線補間する (周期境界をまたいでよい)
  for (uint16_t b = 0; b < n; b++) {
    if (valid[b]) continue;
    uint16_t lo = 1, hi = 1;
    while (!valid[(b + n - lo) % n]) lo++;
    while (!valid[(b + hi) % n]) hi++;
    float r_lo = work_r[(b + n - lo) % n];
    float r_hi = work_r[(b + hi) % n];
    work_r[b] = r_lo + (r_hi - r_lo) * (float)lo / (float)(lo + hi);
  }
  // 補間で埋めた値が次の無効ビンの補間元にならないよう、上のループは valid だけを見ている

  // 滑らかさの検査 (補間したビンも含む)
  float max_step = 0.0f;
  for (uint16_t b = 0; b < n; b++) {
    float d = fabsf(work_r[(b + 1) % n] - work_r[b]);
    if (d > max_step) max_step = d;
  }

  // φ の格子の r から θ_lin の格子の補正量へ写し替える。θ_lin = φ + offset + r(φ) を φ について
  // 解くのに固定点反復 c ← r(θ_lin − offset − c) を使う (|dr/dφ| < 1 なら収束、数回で十分)
  for (uint16_t k = 0; k < n; k++) {
    float th = (float)k * (CM_TWO_PI / (float)n);
    float c = 0.0f;
    for (uint8_t it = 0; it < 4; it++) c = CalibMath_InterpBins(work_r, n, th - offset - c);
    float q = c / CALIB_LUT_SCALE;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    lut[k] = (int16_t)(q + (q >= 0.0f ? 0.5f : -0.5f));
  }
  *out_offset = offset < 0.0f ? offset + CM_TWO_PI : offset;
  return max_step;
}

// 運転時の補正量 [rad]。θ_lin ∈ [0, 2π) の格子 k·2π/N を線形補間する (n は2のべき乗)
static inline float CalibMath_LutCorrection(const int16_t* lut, uint16_t n, float theta_lin) {
  float f = theta_lin * ((float)n / CM_TWO_PI);
  int32_t i0 = (int32_t)f;
  float frac = f - (float)i0;
  int32_t a = lut[i0 & (n - 1)];
  int32_t b = lut[(i0 + 1) & (n - 1)];
  return ((float)a + frac * (float)(b - a)) * CALIB_LUT_SCALE;
}

#endif  // CALIB_MATH_H_
