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

// 速度の記録 w[0..n) (一定周期) が速度帯 [lo, hi] を通過する区間の傾き [単位/サンプル]。
// rising: 加速 (dir·w が lo を最初に越えた点 〜 hi 未満だった最後の点) /
// 偽: 減速 (hi を最初に下回った点 〜 lo より上だった最後の点)。w は加速だけ・減速だけの記録を渡すこと。
// 区間内は途中で帯から外れた点も含めて全部使う。速度推定やコギングによる数 rad/s の揺れで
// 帯を出入りしても区間が切れず、揺れは最小二乗で平均される (「帯の中の連続区間」だと揺れのたびに
// 打ち切られ、数点の傾きになって符号まで狂った)。終わりを「最後の点」で取るのは、揺れで境界を
// 早く越えた1点で区間が切れないようにするため。
// 加速と減速で同じ速度帯を使うと、速度に依存する摩擦が差で消える。使った点数を返す。
static inline uint16_t CalibMath_BandSlope(const float* w, uint16_t n, float dir, float lo,
                                           float hi, bool rising, float* slope) {
  uint16_t first = n, last = 0;
  for (uint16_t k = 0; k < n; k++) {
    float v = dir * w[k];
    bool past_enter = rising ? (v >= lo) : (v <= hi);
    bool before_leave = rising ? (v < hi) : (v > lo);
    if (first == n && past_enter) first = k;
    if (first != n && before_leave) last = k;
  }
  if (first >= n || last <= first) return 0;
  uint16_t count = (uint16_t)(last - first + 1);
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
// エンコーダのレール (飽和したときの出力値)
// ---------------------------------------------------------------------------
// hist[k] = 「基準値から k 〜 k+1 LSB 離れた位置にあったサンプル数」(k = 0..n-1)。基準値は1周目の極値。
// レールに張り付いたサンプルは数百個が ±σ に集まり、レールから離れていく途中のサンプルは
// 1 LSB あたり数個しかないので、最頻のビンがレールになる。最頻ビン ±2 の重心を返す [基準値からの LSB]。
// 単発のノイズで極値がずれても、最頻値は動かない (ビン平均の最小/最大だと、ビンの境目と継ぎ目の
// 位置関係で 10〜20 LSB 内側に偏り、盲点幅が校正ごとに 1.1〜1.9% とばらついた)。
// 1つもサンプルが無ければ負を返す。
static inline float CalibMath_RailOffset(const uint16_t* hist, uint16_t n) {
  uint16_t mode = 0;
  for (uint16_t k = 1; k < n; k++) {
    if (hist[k] > hist[mode]) mode = k;
  }
  if (hist[mode] == 0) return -1.0f;
  float sum = 0.0f, cnt = 0.0f;
  for (int16_t k = (int16_t)mode - 2; k <= (int16_t)mode + 2; k++) {
    if (k < 0 || k >= (int16_t)n) continue;
    sum += ((float)k + 0.5f) * (float)hist[k];
    cnt += (float)hist[k];
  }
  return sum / cnt;
}

// ---------------------------------------------------------------------------
// エンコーダの d軸オフセット (強制転流)
// ---------------------------------------------------------------------------
// 入力: 真の機械角 φ_b = (b + 0.5)·2π/N のビンごとの、線形写像した角度 θ_lin の平均 (正転と逆転の平均)。
// 出力: offset = θ_lin − φ の円平均 [0, 2π)。max_step には隣接ビンの (θ_lin − φ) の最大差 [rad] を入れる
//       (ロータの滑り・極対数の不一致があると大きくなる)。有効ビンが半分未満なら負を返す。
// 補正テーブル (センサの非線形) はここでは作らない。強制転流ではロータの位置ずれ (コギング・
// 電流の不揃いによるトルクの脈動) と区別できないため (CalibMath_CoastFit を参照)。
static inline float CalibMath_BinOffset(const float* theta_lin, const bool* valid, uint16_t n,
                                        float* max_step) {
  int16_t ref = -1;
  for (uint16_t b = 0; b < n; b++) {
    if (valid[b]) {
      ref = (int16_t)b;
      break;
    }
  }
  if (ref < 0) return -1.0f;
  const float bin = CM_TWO_PI / (float)n;
  const float base = CalibMath_Wrap(theta_lin[ref] - ((float)ref + 0.5f) * bin);
  float sum = 0.0f, prev = 0.0f;
  uint16_t cnt = 0;
  *max_step = 0.0f;
  bool prev_ok = false;
  for (uint16_t b = 0; b < n; b++) {
    if (!valid[b]) {
      prev_ok = false;
      continue;
    }
    float r = CalibMath_Wrap(theta_lin[b] - ((float)b + 0.5f) * bin - base);  // base からの差 (小さい)
    if (prev_ok && fabsf(r - prev) > *max_step) *max_step = fabsf(r - prev);
    prev = r;
    prev_ok = true;
    sum += r;
    cnt++;
  }
  if (cnt < n / 2) return -1.0f;
  float offset = CalibMath_Wrap(base + sum / (float)cnt);
  return offset < 0.0f ? offset + CM_TWO_PI : offset;
}

// ---------------------------------------------------------------------------
// エンコーダ補正テーブル (惰性回転)
// ---------------------------------------------------------------------------
// 出力を切って惰性で回っている間に、θ_lin が格子線 k·2π/N を横切った時刻を記録する。
// 電流が流れていないので、電流センサ・相抵抗の不揃い・デッドタイムによるトルクの脈動が入らない。
// コギングによる速度の揺れも慣性で小さくなる (位置のずれ ≈ T/(J·Ω²)、Ω は脈動の角周波数)。
// 真の角度は時間の滑らかな関数 (摩擦による減速) なので、各回の記録に3次式を当てはめた残りが
// センサの誤差 e(θ_lin) になる。強制転流と違い、極対数の倍数の成分もセンサ由来のものだけが残る。
// 運転時と同じ θ_lin・飽和判定で記録するので、継ぎ目付近の扱いも校正と運転で一致する。
#define CALIB_LUT_SCALE (CM_TWO_PI / 65536.0f)  // lut の1LSB [rad]
#define COAST_T_SCALE 8.0f                      // 記録時刻の単位 = 1/8 サンプル
#define COAST_MISSING 0xFFFFu                   // 横切った時刻が取れなかった格子線 (盲点など)

// 1回の惰性回転の記録 (ISR で1サンプルずつ CalibMath_CoastSample を呼ぶ)
typedef struct {
  uint16_t* buf;  // 横切った時刻 [1/8 サンプル]。格子線 k0, k0+dir, k0+2·dir, … の順
  uint16_t cap;   // buf の容量
  uint16_t n;     // 記録した数
  uint32_t t;     // 開始からのサンプル数
  float prev;     // 前のサンプルの θ_lin [格子単位]
  int16_t next;   // 次に横切る格子線
  int16_t k0;     // 最初の格子線
  int8_t dir;     // 回転の向き (θ_lin が増える向きが +1)
  bool prev_ok;   // prev が有効 (直前のサンプルが飽和していない)
  bool started;
  bool full;      // buf が埋まった / 時刻が uint16 を超えた
} CoastRec;

// th: θ_lin [格子単位 0..n)、ok: 飽和とその直後でない。n は2のべき乗
static inline void CalibMath_CoastSample(CoastRec* r, float th, bool ok, uint16_t n) {
  if (r->full) return;
  r->t++;
  if ((float)r->t * COAST_T_SCALE >= (float)COAST_MISSING) {
    r->full = true;
    return;
  }
  if (!ok) {
    r->prev_ok = false;
    return;
  }
  const float half = 0.5f * (float)n;
  if (!r->started) {
    int16_t k = (int16_t)th;  // floor (th ≥ 0)
    r->next = (int16_t)((r->dir > 0 ? k + 1 : k) & (n - 1));
    r->k0 = r->next;
    r->started = true;
  } else {
    // 今のサンプルまでに過ぎた格子線を順に記録する。1サンプルで進むのは格子の数分の1なので、
    // 何本も過ぎるのは盲点を抜けた直後だけ (そこは prev が無いので時刻なし)
    for (uint8_t guard = 0; guard < 8; guard++) {
      float dn = th - (float)r->next;
      if (dn > half) dn -= (float)n;
      if (dn <= -half) dn += (float)n;
      if (dn * (float)r->dir < 0.0f) break;  // まだ手前
      uint16_t v = COAST_MISSING;
      if (r->prev_ok) {
        float dp = th - r->prev, dl = (float)r->next - r->prev;
        if (dp > half) dp -= (float)n;
        if (dp <= -half) dp += (float)n;
        if (dl > half) dl -= (float)n;
        if (dl <= -half) dl += (float)n;
        float s = (float)r->dir;
        if (dp * s > 0.0f && dl * s >= 0.0f && dl * s <= dp * s) {
          float t = ((float)(r->t - 1) + dl / dp) * COAST_T_SCALE;
          v = (uint16_t)(t + 0.5f);
        }
      }
      r->buf[r->n++] = v;
      r->next = (int16_t)((r->next + r->dir) & (n - 1));
      if (r->n >= r->cap) {
        r->full = true;
        break;
      }
    }
  }
  r->prev = th;
  r->prev_ok = true;
}

// 記録の区切り (buf の中の位置と向き)
typedef struct {
  uint16_t base, n;
  int16_t k0;
  int8_t dir;
} CoastSeg;

// m 元の連立1次方程式 (拡大係数行列 a[m][m+1]) を部分ピボットのガウス消去で解き、解を a[i][m] に残す
static inline bool CalibMath_Solve4(float a[4][5]) {
  for (uint8_t c = 0; c < 4; c++) {
    uint8_t p = c;
    for (uint8_t r = c + 1; r < 4; r++) {
      if (fabsf(a[r][c]) > fabsf(a[p][c])) p = r;
    }
    if (!(fabsf(a[p][c]) > 1e-12f)) return false;
    for (uint8_t k = 0; k < 5; k++) {
      float t = a[c][k];
      a[c][k] = a[p][k];
      a[p][k] = t;
    }
    for (uint8_t r = 0; r < 4; r++) {
      if (r == c) continue;
      float f = a[r][c] / a[c][c];
      for (uint8_t k = c; k < 5; k++) a[r][k] -= f * a[c][k];
    }
  }
  for (uint8_t r = 0; r < 4; r++) a[r][4] /= a[r][r];
  return true;
}

#define COAST_FIT_ITER 40  // 当てはめと誤差の推定を交互に繰り返す回数

// 惰性回転の記録から補正量 e[k] (θ_lin の格子線 k での θ_lin − 真の角度、平均0) [rad] を求める。
//   各回: 真の角度 = 時刻の3次式。格子線の位置 (unwrap) − e = 3次式 を最小二乗
//   全体: e[k] = 各回の残差の平均 (正転と逆転を合わせる)
// を交互に繰り返す (3次式とセンサの誤差を同時に最小二乗するのと同じ解に収束する)。
// 正転と逆転を平均すると、センサのフィルタによる位相の遅れが打ち消される。
// sum[2n] / cnt[2n] は作業用 (正転 / 逆転ごとの残差の和と数)。
// 横切った記録が無い格子線 (盲点) は前後から補間する。
// 戻り値: 当てはめの残差 RMS [rad] (負なら失敗)。fb_max: 正転と逆転の差の最大 [rad]、missing: 補間した格子線の数
static inline float CalibMath_CoastFit(const uint16_t* buf, const CoastSeg* seg, uint8_t nseg,
                                       uint16_t n, float* e, float* sum, uint8_t* cnt,
                                       float* fb_max, uint16_t* missing) {
  const float step = CM_TWO_PI / (float)n;
  for (uint16_t k = 0; k < n; k++) e[k] = 0.0f;
  float rms = 0.0f;
  for (uint8_t it = 0; it <= COAST_FIT_ITER; it++) {
    for (uint16_t k = 0; k < 2u * n; k++) {
      sum[k] = 0.0f;
      cnt[k] = 0;
    }
    float sq = 0.0f;
    uint32_t sq_n = 0;
    for (uint8_t s = 0; s < nseg; s++) {
      const uint16_t* t8 = &buf[seg[s].base];
      // 時刻を [-1, 1] に正規化して条件を良くする
      float t_lo = 1e9f, t_hi = -1e9f;
      for (uint16_t j = 0; j < seg[s].n; j++) {
        if (t8[j] == COAST_MISSING) continue;
        if ((float)t8[j] < t_lo) t_lo = (float)t8[j];
        if ((float)t8[j] > t_hi) t_hi = (float)t8[j];
      }
      if (!(t_hi > t_lo)) return -1.0f;
      const float tm = 0.5f * (t_lo + t_hi), th = 0.5f * (t_hi - t_lo);
      float a[4][5] = {{0}};
      for (uint16_t j = 0; j < seg[s].n; j++) {
        if (t8[j] == COAST_MISSING) continue;
        uint16_t k = (uint16_t)((seg[s].k0 + seg[s].dir * (int32_t)j) & (n - 1));
        float u = ((float)t8[j] - tm) / th;
        float y = (float)seg[s].dir * (float)j * step - e[k];
        float p[4] = {1.0f, u, u * u, u * u * u};
        for (uint8_t r = 0; r < 4; r++) {
          for (uint8_t c = 0; c < 4; c++) a[r][c] += p[r] * p[c];
          a[r][4] += p[r] * y;
        }
      }
      if (!CalibMath_Solve4(a)) return -1.0f;
      const uint16_t side = (seg[s].dir > 0) ? 0 : n;
      for (uint16_t j = 0; j < seg[s].n; j++) {
        if (t8[j] == COAST_MISSING) continue;
        uint16_t k = (uint16_t)((seg[s].k0 + seg[s].dir * (int32_t)j) & (n - 1));
        float u = ((float)t8[j] - tm) / th;
        float fit = a[0][4] + u * (a[1][4] + u * (a[2][4] + u * a[3][4]));
        float r = (float)seg[s].dir * (float)j * step - fit;  // = e[k] + 定数 + ノイズ
        float d = r - e[k];
        sq += d * d;
        sq_n++;
        sum[side + k] += r;
        if (cnt[side + k] < 255) cnt[side + k]++;
      }
    }
    rms = sqrtf(sq / (float)(sq_n ? sq_n : 1));
    // e を更新して平均を0にする (定数は各回の3次式の定数項と区別できない)
    float mean = 0.0f;
    uint16_t m = 0;
    for (uint16_t k = 0; k < n; k++) {
      uint16_t c = cnt[k] + cnt[n + k];
      if (c == 0) continue;
      e[k] = (sum[k] + sum[n + k]) / (float)c;
      mean += e[k];
      m++;
    }
    if (m < n / 2) return -1.0f;
    mean /= (float)m;
    for (uint16_t k = 0; k < n; k++) {
      if (cnt[k] + cnt[n + k] > 0) e[k] -= mean;
    }
  }

  // 正転と逆転の差 (3次式の定数項の違いを除くため、それぞれ平均を引いてから比べる)
  float mf = 0.0f, mb = 0.0f;
  uint16_t nb = 0;
  for (uint16_t k = 0; k < n; k++) {
    if (cnt[k] && cnt[n + k]) {
      mf += sum[k] / (float)cnt[k];
      mb += sum[n + k] / (float)cnt[n + k];
      nb++;
    }
  }
  *fb_max = 0.0f;
  if (nb > 0) {
    mf /= (float)nb;
    mb /= (float)nb;
    for (uint16_t k = 0; k < n; k++) {
      if (!(cnt[k] && cnt[n + k])) continue;
      float d = fabsf((sum[k] / (float)cnt[k] - mf) - (sum[n + k] / (float)cnt[n + k] - mb));
      if (d > *fb_max) *fb_max = d;
    }
  }

  // 記録の無い格子線を前後から直線補間する (周期境界をまたいでよい)
  *missing = 0;
  for (uint16_t k = 0; k < n; k++) {
    if (cnt[k] + cnt[n + k] > 0) continue;
    (*missing)++;
    uint16_t lo = 1, hi = 1;
    while (cnt[(k + n - lo) % n] + cnt[n + (k + n - lo) % n] == 0) lo++;
    while (cnt[(k + hi) % n] + cnt[n + (k + hi) % n] == 0) hi++;
    float e_lo = e[(k + n - lo) % n], e_hi = e[(k + hi) % n];
    e[k] = e_lo + (e_hi - e_lo) * (float)lo / (float)(lo + hi);
  }
  return rms;
}

// 角度 x ∈ [0, 2π) の格子 k·2π/N に置いた int16 の表を線形補間する (n は2のべき乗。単位は表のまま)
static inline float CalibMath_TableInterp(const int16_t* table, uint16_t n, float x) {
  float f = x * ((float)n / CM_TWO_PI);
  int32_t i0 = (int32_t)f;
  float frac = f - (float)i0;
  int32_t a = table[i0 & (n - 1)];
  int32_t b = table[(i0 + 1) & (n - 1)];
  return (float)a + frac * (float)(b - a);
}

// 運転時の補正量 [rad]。θ_lin ∈ [0, 2π) の格子 k·2π/N を線形補間する (n は2のべき乗)
static inline float CalibMath_LutCorrection(const int16_t* lut, uint16_t n, float theta_lin) {
  return CalibMath_TableInterp(lut, n, theta_lin) * CALIB_LUT_SCALE;
}


#endif  // CALIB_MATH_H_
