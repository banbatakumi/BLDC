// 校正の計算 (src/bldc/calib_math.h) の検証。ファームウェアと**同じヘッダ**を include する。
// 実機の代わりに合成データ (エンコーダの歪み・1次遅れ・摩擦のある加減速・デッドタイム) を作り、
// 既知の真値を取り戻せるかを見る。
//
//   make -C tests
#include <stdio.h>
#include <stdlib.h>

#include "../src/bldc/calib_math.h"

static int failures = 0;

static void Check(const char* name, double value, double expect, double tol) {
  double err = value - expect;
  bool ok = (err <= tol && err >= -tol);
  printf("  %-36s %12.6g (期待 %12.6g ±%g) %s\n", name, value, expect, tol, ok ? "OK" : "NG");
  if (!ok) failures++;
}

// 決定的な一様乱数 [-1, 1]
static float Noise(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

// --- [A] エンコーダ補正テーブル ---------------------------------------------
// 磁石の偏心 (1次) と INL (2次) で歪んだセンサ。盲点のビンは無効にする。
#define N 128
static float Distorted(float phi, float offset) {
  return phi + offset + 0.03f * sinf(phi + 0.7f) + 0.012f * sinf(2.0f * phi + 2.1f);
}

static void TestLut(void) {
  printf("[A] 補正テーブル (歪み: 1次 0.03rad + 2次 0.012rad、盲点 6ビン)\n");
  const float offset = 5.9f;  // 2π の近くで円平均の折り返しも試す
  float theta_bin[N], work[N];
  bool valid[N];
  int16_t lut[N];

  for (int b = 0; b < N; b++) {
    // ビン内を細かくサンプリングして平均する (ファームの ISR 集計と同じ)
    float sum = 0.0f;
    float base = 0.0f;
    for (int j = 0; j < 32; j++) {
      float phi = ((float)b + ((float)j + 0.5f) / 32.0f) * (CM_TWO_PI / N);
      float th = CalibMath_Wrap(Distorted(phi, offset));
      if (j == 0) base = th;
      sum += CalibMath_Wrap(th - base);
    }
    float th = base + sum / 32.0f;
    if (th < 0.0f) th += CM_TWO_PI;
    if (th >= CM_TWO_PI) th -= CM_TWO_PI;
    theta_bin[b] = th;
    valid[b] = !(b >= 60 && b < 66);
  }
  float off_est;
  float max_step = CalibMath_BuildLut(theta_bin, valid, N, work, &off_est, lut);
  Check("offset", off_est, offset, 2e-3);
  Check("隣接差 (小さいこと)", max_step, 0.0, 0.01);

  // 細かい φ で、補正後の角度の誤差を見る (盲点の区間は補間なので別に数える)
  float max_err = 0.0f, max_err_dead = 0.0f, max_raw = 0.0f;
  for (int k = 0; k < 4096; k++) {
    float phi = (float)k * (CM_TWO_PI / 4096.0f);
    float th = CalibMath_Wrap(Distorted(phi, offset));
    if (th < 0.0f) th += CM_TWO_PI;
    float phi_hat = th - off_est - CalibMath_LutCorrection(lut, N, th);
    float e = fabsf(CalibMath_Wrap(phi_hat - phi));
    float raw = fabsf(CalibMath_Wrap(th - offset - phi));
    bool dead = (phi >= 60.0f * CM_TWO_PI / N && phi < 66.0f * CM_TWO_PI / N);
    if (dead) {
      if (e > max_err_dead) max_err_dead = e;
    } else {
      if (e > max_err) max_err = e;
      if (raw > max_raw) max_raw = raw;
    }
  }
  printf("  補正前の最大誤差 %.4f rad → 補正後 %.5f rad (電気角×7 で %.2f°)\n", max_raw, max_err,
         max_err * 7.0f * 57.3f);
  Check("補正後の最大誤差 [rad]", max_err, 0.0, 1e-3);
  Check("盲点区間の最大誤差 [rad]", max_err_dead, 0.0, 5e-3);
}

// --- [B] ステップ応答の時定数 ---------------------------------------------
// 1次系 (τ) を 50µs でサンプリングし、半周期のむだ時間と電流LPF (a=0.5) を通したもの
static void TestTau(float tau_us) {
  const float ts = 50e-6f, tau = tau_us * 1e-6f, a = 0.5f;
  float i[200], y = 0.0f;
  i[0] = 0.0f;
  for (int k = 1; k < 200; k++) {
    float t = ((float)k - 0.5f) * ts;
    float x = 3.0f * (1.0f - expf(-t / tau)) + 0.002f * Noise();
    y += a * (x - y);
    i[k] = y;
  }
  float tau_est = CalibMath_FitTau(i, 200, 3.0f, a) * ts;
  char name[48];
  snprintf(name, sizeof(name), "τ=%.0fµs の推定 [µs]", tau_us);
  Check(name, tau_est * 1e6f, tau_us, tau_us * 0.05f);
}

// --- [C] J/Kt と摩擦 (加速→減速) ------------------------------------------
// (J/Kt)·dω/dt = Iq − (ic + b·ω)  (ω > 0)。1kHz で記録する
static void TestInertia(void) {
  printf("[C] J/Kt (真値 1.0e-4 A·s²/rad、摩擦 0.15A + 0.002A/(rad/s))\n");
  const float jkt = 1.0e-4f, ic = 0.15f, b = 0.002f, current = 0.4f, top = 75.0f, dt = 0.001f;
  float w[400];
  uint16_t n = 0, sw = 0;
  float omega = 0.0f, iq = current;
  // 20kHz で積分し、1kHz で記録する (ファームの記録と同じ間引き)
  while (n < 400) {
    for (int k = 0; k < 20; k++) {
      float fric = (omega > 0.0f) ? ic + b * omega : 0.0f;
      omega += (iq - fric) / jkt * (dt / 20.0f);
    }
    w[n++] = omega + 0.05f * Noise();
    if (sw == 0 && omega >= top) {
      sw = n;
      iq = -current;
    } else if (sw != 0 && omega <= 0.0f) {
      break;
    }
  }
  float s_acc = 0.0f, s_dec = 0.0f;
  uint16_t na = CalibMath_BandSlope(w, sw, 1.0f, 0.2f * top, 0.8f * top, &s_acc);
  uint16_t nd = CalibMath_BandSlope(&w[sw], n - sw, 1.0f, 0.2f * top, 0.8f * top, &s_dec);
  float a_acc = s_acc / dt, a_dec = s_dec / dt;
  float jkt_est = 2.0f * current / (a_acc - a_dec);
  float fric_est = -jkt_est * (a_acc + a_dec) * 0.5f;
  printf("  加速 %.0f (%u点) 減速 %.0f (%u点) rad/s²\n", a_acc, na, a_dec, nd);
  Check("J/Kt", jkt_est, jkt, jkt * 0.03);
  // 摩擦は速度帯の平均 ic + b·(帯の中央 37.5) 付近になる
  Check("摩擦電流 [A]", fric_est, ic + b * 0.5f * top, 0.02);
}

// --- [D] ψm とデッドタイムの分離 -------------------------------------------
static void TestPsi(void) {
  printf("[D] ψm (真値 0.00185Wb、V0 0.26V)\n");
  float x[8], s[8], y[8];
  for (int k = 0; k < 8; k++) {
    s[k] = (k < 4) ? 1.0f : -1.0f;
    x[k] = s[k] * 7.0f * 33.0f * (float)(k % 4 + 1);
    y[k] = 0.00185f * x[k] + 0.26f * s[k] + 0.005f * Noise();
  }
  float psi, v0;
  bool ok = CalibMath_FitPsi(x, s, y, 8, &psi, &v0);
  Check("FitPsi 成功", ok, 1, 0);
  Check("ψm [Wb]", psi, 0.00185, 2e-5);
  Check("V0 [V]", v0, 0.26, 0.01);
}

int main(void) {
  srand(1);
  TestLut();
  printf("[B] 時定数 (半周期のむだ時間 + LPF a=0.5 込み)\n");
  TestTau(100.0f);
  TestTau(200.0f);
  TestTau(400.0f);
  TestInertia();
  TestPsi();
  if (failures) {
    printf("\n%d 件 NG\n", failures);
    return 1;
  }
  printf("\nすべて OK\n");
  return 0;
}
