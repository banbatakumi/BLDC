// 校正の計算 (src/bldc/calib_math.h) の検証。ファームウェアと**同じヘッダ**を include する。
// 実機の代わりに合成データ (エンコーダの歪み・1次遅れ・摩擦のある加減速・デッドタイム) を作り、
// 既知の真値を取り戻せるかを見る。
//
//   make -C tests
#include <stdio.h>
#include <stdlib.h>

#include "../src/bldc/calib_math.h"
#include "../src/app/fixfmt.c"  // ファームと同じ実装 (関数定義ごと取り込む)
#include <string.h>

static int failures = 0;

static void Check(const char* name, double value, double expect, double tol) {
  double err = value - expect;
  bool ok = (err <= tol && err >= -tol);
  printf("  %-36s %12.6g (期待 %12.6g ±%g) %s\n", name, value, expect, tol, ok ? "OK" : "NG");
  if (!ok) failures++;
}

// 決定的な一様乱数 [-1, 1]
static float Noise(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

// --- [A] エンコーダの d軸オフセット (強制転流のビン平均) ------------------------
// 磁石の偏心 (1次) と INL (2次) で歪んだセンサ。盲点のビンは無効にする。
#define N 128
static float Distorted(float phi, float offset) {
  return phi + offset + 0.03f * sinf(phi + 0.7f) + 0.012f * sinf(2.0f * phi + 2.1f);
}

static void TestOffset(void) {
  printf("[A] d軸オフセット (歪み: 1次 0.03rad + 2次 0.012rad、盲点 6ビン)\n");
  const float offset = 5.9f;  // 2π の近くで円平均の折り返しも試す
  float th[N];
  bool valid[N];
  for (int b = 0; b < N; b++) {
    float x = CalibMath_Wrap(Distorted(((float)b + 0.5f) * (CM_TWO_PI / N), offset));
    th[b] = x < 0.0f ? x + CM_TWO_PI : x;
    valid[b] = !(b >= 60 && b < 66);
  }
  float max_step;
  float off_est = CalibMath_BinOffset(th, valid, N, &max_step);
  // 歪みの平均は全周では0だが、盲点のビンを抜いたぶん少しずれる
  Check("offset", off_est, offset, 2e-3);
  Check("隣接差 (小さいこと)", max_step, 0.0, 0.01);
}

// --- [A2] 補正テーブル (惰性回転) ----------------------------------------------
// センサの誤差 s(φ): 偏心 + INL + 14回/周 (モータの磁石の漏れ磁束などセンサ自身の誤差として)。
// 盲点 0.71%、飽和直後 4 サンプルは捨てる、AS5600 の遅れ (むだ時間 0.6ms + 1次遅れ 0.5ms)、
// ADC のノイズ σ 0.001rad、クーロン + 粘性の摩擦で 60 → 18 rad/s まで減速、を正転2回・逆転2回。
static float SensorErr(float phi) {
  return 0.03f * sinf(phi + 0.7f) + 0.012f * sinf(2.0f * phi + 2.1f) +
         0.004f * sinf(14.0f * phi + 0.3f);
}

static float Gauss(void) {
  float s = 0.0f;
  for (int i = 0; i < 12; i++) s += (float)rand() / (float)RAND_MAX;
  return s - 6.0f;
}

static void TestCoast(void) {
  printf("[A2] 補正テーブル (惰性回転 4回、盲点 0.71%%、遅れ 1.1ms、ノイズ 0.001rad)\n");
  const float dt = 50e-6f, dead = 0.0071f * CM_TWO_PI, span = CM_TWO_PI - dead;
  const float offset = 1.3f, tau_lpf = 0.5e-3f;
  const int delay = 12;  // 0.6ms のむだ時間 [サンプル]
  static uint16_t buf[1024];
  CoastSeg seg[4];
  uint16_t used = 0;
  for (int r = 0; r < 4; r++) {
    float dir = (r & 1) ? -1.0f : 1.0f;
    float w = 60.0f * dir, phi = (float)r * 1.7f + 0.4f;
    // むだ時間ぶんの過去の真の角度を覚えておく
    float hist[64];
    for (int i = 0; i < 64; i++) hist[i] = phi - w * dt * (float)(63 - i);
    float sens = phi + SensorErr(phi);  // 1次遅れの状態 (unwrap)
    CoastRec rec = {0};
    rec.buf = &buf[used];
    rec.cap = (uint16_t)((1024 - used) / (4 - r));
    rec.dir = (int8_t)dir;
    int since_sat = 100;
    while (fabsf(w) > 18.0f && !rec.full) {
      float a = -(100.0f + 1.0f * fabsf(w)) * dir;
      w += a * dt;
      phi += w * dt;
      memmove(hist, hist + 1, sizeof(float) * 63);
      hist[63] = phi;
      float p = hist[63 - delay];
      float target = p + SensorErr(p);
      sens += (target - sens) * (dt / tau_lpf);
      float x = fmodf(sens + offset, CM_TWO_PI);
      if (x < 0.0f) x += CM_TWO_PI;
      bool sat = (x >= span);
      since_sat = sat ? 0 : since_sat + 1;
      bool ok = !sat && since_sat > 4;
      float th = (x + 0.001f * Gauss()) * ((float)N / CM_TWO_PI);
      if (th < 0.0f) th += (float)N;
      if (th >= (float)N) th -= (float)N;
      CalibMath_CoastSample(&rec, th, ok, N);
    }
    seg[r].base = used;
    seg[r].n = rec.n;
    seg[r].k0 = rec.k0;
    seg[r].dir = rec.dir;
    used += rec.n;
  }
  printf("  記録 %u 点\n", used);

  float e[N], sum[2 * N], fb;
  uint8_t cnt[2 * N];
  uint16_t missing;
  float rms = CalibMath_CoastFit(buf, seg, 4, N, e, sum, cnt, &fb, &missing);
  int16_t lut[N];
  for (int k = 0; k < N; k++) lut[k] = (int16_t)lrintf(e[k] / CALIB_LUT_SCALE);
  printf("  残差 RMS %.5f rad  正逆差 %.5f rad  補間 %u 本\n", rms, fb, missing);

  // 補正後の角度 − 真の角度 (定数を除く) を細かい φ で見る。盲点の区間は運転中も使わないので除く
  float err[4096], mean = 0.0f, max_raw = 0.0f;
  int m = 0;
  for (int k = 0; k < 4096; k++) {
    float phi = (float)k * (CM_TWO_PI / 4096.0f);
    float x = fmodf(phi + SensorErr(phi) + offset, CM_TWO_PI);
    if (x >= span) continue;
    float c = x - CalibMath_LutCorrection(lut, N, x);
    err[m] = CalibMath_Wrap(c - phi - offset);
    mean += err[m];
    float raw = fabsf(SensorErr(phi));
    if (raw > max_raw) max_raw = raw;
    m++;
  }
  mean /= (float)m;
  float max_err = 0.0f;
  for (int k = 0; k < m; k++) {
    if (fabsf(err[k] - mean) > max_err) max_err = fabsf(err[k] - mean);
  }
  printf("  補正前の最大誤差 %.4f rad → 補正後 %.5f rad\n", max_raw, max_err);
  Check("当てはめ成功 (RMS ≥ 0)", rms >= 0.0f, 1, 0);
  // 乱数の種を変えると 0.0008〜0.0019rad (格子線ごとの記録が正逆それぞれ約2回なので、ノイズが残る)
  Check("補正後の最大誤差 [rad]", max_err, 0.0, 3e-3);
  Check("補間した格子線", missing, 1, 2);
}

// --- [A3] レールの推定 (ヒストグラムの最頻値) ---------------------------------
static void TestRail(float spike) {
  // レールに張り付いたサンプル 400 個 (σ 0.7) + 離れていく途中のサンプル (1 LSB あたり 4 個)。
  // spike: 単発ノイズで極値がレールより外側にずれた量 [LSB]
  uint16_t hist[64] = {0};
  const float rail = 1000.0f, base = rail - 2.0f - spike;  // base = 1周目の極値
  for (int k = 0; k < 400; k++) {
    float v = rail + 0.7f * (Noise() + Noise() + Noise());  // 近似正規分布
    float d = v - base;
    if (d >= 0.0f && d < 64.0f) hist[(int)d]++;
  }
  for (float v = rail + 3.0f; v < base + 64.0f; v += 0.25f) {
    float d = v - base;
    if (d >= 0.0f && d < 64.0f) hist[(int)d]++;
  }
  char name[64];
  snprintf(name, sizeof(name), "レール (極値のずれ %.0f LSB)", spike);
  Check(name, base + CalibMath_RailOffset(hist, 64), rail, 0.5);
}

// --- [F] FixFmt (printf の %f の代わり) -------------------------------------
static void CheckStr(const char* got, const char* want) {
  bool ok = strcmp(got, want) == 0;
  printf("  %-12s → \"%s\" (期待 \"%s\") %s\n", "FixFmt", got, want, ok ? "OK" : "NG");
  if (!ok) failures++;
}

static void TestFixFmt(void) {
  printf("[F] FixFmt\n");
  CheckStr(FixFmt(0.14072f, 4), "0.1407");
  CheckStr(FixFmt(35.46f, 1), "35.5");
  CheckStr(FixFmt(-1.004f, 2), "-1.00");
  CheckStr(FixFmt(-0.004f, 2), "0.00");  // 丸めて0になったら符号を付けない
  CheckStr(FixFmtSigned(1.25f, 2), "+1.25");
  CheckStr(FixFmtSigned(-0.5f, 1), "-0.5");
  CheckStr(FixFmt(0.001980f, 6), "0.001980");
  CheckStr(FixFmt(702.4f, 0), "702");
  CheckStr(FixFmt(1e10f, 0), "ovf");
  CheckStr(FixFmt(NAN, 2), "nan");
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
// (J/Kt)·dω/dt = Iq − (ic + b·ω)  (ω > 0)。20kHz で積分して 1kHz で記録する (ファームと同じ間引き)。
// 記録には白色ノイズと、コギング的な位置周期の揺れ (振幅 ripple、機械角1周に 42 回) を足す。
static void TestInertia(const char* label, float jkt, float ic, float b, float current,
                        float noise, float ripple, float tol) {
  printf("[C] J/Kt %s (真値 %.1e、摩擦 %.2fA + %.3fA/(rad/s)、I=%.2fA、揺れ ±%.0f+%.0f rad/s)\n",
         label, jkt, ic, b, current, noise, ripple);
  const float top = 75.0f, dt = 0.001f;
  float w[400];
  uint16_t n = 0, sw = 0;
  float omega = 0.0f, theta = 0.0f, iq = current;
  while (n < 400) {
    for (int k = 0; k < 20; k++) {
      float fric = (omega > 0.0f) ? ic + b * omega : 0.0f;
      omega += (iq - fric) / jkt * (dt / 20.0f);
      theta += omega * (dt / 20.0f);
    }
    float meas = omega + noise * Noise() + ripple * sinf(42.0f * theta);
    w[n++] = meas;
    // ファームと同じく、記録した (揺れを含む) 値で切り替える
    if (sw == 0 && meas >= top) {
      sw = n;
      iq = -current;
    } else if (sw != 0 && meas <= 0.0f) {
      break;
    }
  }
  float s_acc = 0.0f, s_dec = 0.0f;
  uint16_t na = CalibMath_BandSlope(w, sw, 1.0f, 0.2f * top, 0.8f * top, true, &s_acc);
  uint16_t nd = CalibMath_BandSlope(&w[sw], n - sw, 1.0f, 0.2f * top, 0.8f * top, false, &s_dec);
  float a_acc = s_acc / dt, a_dec = s_dec / dt;
  float jkt_est = 2.0f * current / (a_acc - a_dec);
  float fric_est = -jkt_est * (a_acc + a_dec) * 0.5f;
  printf("  加速 %.0f (%u点) 減速 %.0f (%u点) rad/s²\n", a_acc, na, a_dec, nd);
  Check("J/Kt", jkt_est, jkt, jkt * tol);
  // 摩擦は速度帯の平均 ic + b·(帯の中央 37.5) 付近になる
  Check("摩擦電流 [A]", fric_est, ic + b * 0.5f * top, 0.03 + 0.05 * ic);
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
  TestOffset();
  TestCoast();
  srand(1);  // 以降の試験の乱数列を TestCoast の有無で変えない
  printf("[A3] レールの推定\n");
  TestRail(0.0f);
  TestRail(20.0f);
  TestFixFmt();
  printf("[B] 時定数 (半周期のむだ時間 + LPF a=0.5 込み)\n");
  TestTau(100.0f);
  TestTau(200.0f);
  TestTau(400.0f);
  TestInertia("理想", 1.0e-4f, 0.15f, 0.002f, 0.4f, 0.05f, 0.0f, 0.05f);
  // 2026-10-19 の実機 (J/Kt 約1e-3、摩擦 0.43A)。旧版は帯の連続区間しか使わず3〜6点で失敗した
  TestInertia("実機相当", 1.0e-3f, 0.43f, 0.0f, 1.0f, 3.0f, 3.0f, 0.05f);
  TestInertia("実機相当 I=0.7A (摩擦に近い)", 1.0e-3f, 0.43f, 0.0f, 0.7f, 3.0f, 3.0f, 0.05f);
  // 揺れが大きいと切り替え・帯の入口の判定が早まり数%ずれる (速度ゲインの誤差としては許容)
  TestInertia("揺れ大", 1.0e-3f, 0.43f, 0.0f, 1.0f, 8.0f, 8.0f, 0.10f);
  TestPsi();
  if (failures) {
    printf("\n%d 件 NG\n", failures);
    return 1;
  }
  printf("\nすべて OK\n");
  return 0;
}
