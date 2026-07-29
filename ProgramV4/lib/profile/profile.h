#ifndef PROFILE_H_
#define PROFILE_H_

#include <stdbool.h>

#include "main.h"

// ---------------------------------------------------------------------------
// DWTサイクルカウンタによる実行時間プロファイラ
// ---------------------------------------------------------------------------
// Cortex-M4 のデバッグユニット(DWT)には、コアクロックを数え続ける32bitの
// フリーランカウンタ CYCCNT がある。これを区間の前後で読んで引き算すれば、
// 追加のタイマも割り込みも使わずに実行時間がサイクル単位で測れる。
//
//   72MHz なので 1サイクル = 13.9ns、1周 = 2^32 / 72MHz = 59.6秒
//   引き算は uint32_t のまま行うこと。ラップアラウンドしても
//   (now - start) は2の補数演算で正しい差になる。
//
// GPIOをトグルしてオシロで見る方法と違い、最大値と平均を長時間ぶん
// 積算できるのが利点。「たまに1回だけ長い」を捕まえられる。
//
// 注意: CYCCNT はコアが停止していると止まる (WFIやデバッガのブレーク中)。
// この制御ではどちらも起きないので問題ない。

// 計測のオーバーヘッド [サイクル]。
// Profile_Begin/End のペアで CYCCNT読み2回 + 比較 + 64bit加算 + ストア。
// 実測で引き算するのが確実だが、目安としてこの程度。
#define PROFILE_OVERHEAD_CYCLES 15

typedef struct {
  uint32_t start;  // Profile_Begin 時点の CYCCNT
  uint32_t last;   // 直近の実行サイクル数
  uint32_t max;    // 区間内の最大サイクル数
  uint64_t sum;    // 平均を出すための合計 (20kHzで1000サイクルなら32bitは3.6分で溢れる)
  uint32_t count;  // 平均を出すためのサンプル数
} Profile;

// 呼び出し間隔(周期)の計測。制御ループのジッタや取りこぼしの検出に使う。
typedef struct {
  uint32_t prev;    // 前回 Mark したときの CYCCNT
  uint32_t last;    // 直近の間隔 [サイクル]
  uint32_t min;     // 区間内の最小
  uint32_t max;     // 区間内の最大
  bool primed;      // 1回目は差が取れないので捨てる
} ProfilePeriod;

// 割り込みを止めてスナップショットした集計結果
typedef struct {
  uint32_t last;
  uint32_t max;
  uint32_t min;
  uint32_t count;
  uint32_t total;  // 区間内の合計サイクル数。CPU使用率はこれと実経過時間から出す
                   // (72MHzなら約59秒ぶんまで溢れない)
  float mean;      // サンプルが無ければ0
} ProfileResult;

// ---------------------------------------------------------------------------
// 初期化
// ---------------------------------------------------------------------------
// CYCCNT を動かす。デバッガが繋がっていなくても、TRCENA をソフトで立てれば動く。
// 制御ループを回し始める前に1回だけ呼ぶこと。
static inline void Profile_EnableDWT(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// CYCCNT が実際に進んでいるか確かめる。
// 進んでいない状態に気づかずに測ると、全部0という結果を「速い」と誤読する。
static inline bool Profile_IsDWTRunning(void) {
  uint32_t t0 = DWT->CYCCNT;
  for (volatile int i = 0; i < 10; i++) {
  }
  return (DWT->CYCCNT != t0);
}

// ---------------------------------------------------------------------------
// 計測 (割り込みの中から呼ぶ)
// ---------------------------------------------------------------------------
static inline void Profile_Begin(Profile* p) {
  p->start = DWT->CYCCNT;
}

static inline void Profile_End(Profile* p) {
  uint32_t cycles = DWT->CYCCNT - p->start;
  p->last = cycles;
  if (cycles > p->max) p->max = cycles;
  p->sum += cycles;
  p->count++;
}

static inline void ProfilePeriod_Mark(ProfilePeriod* p) {
  uint32_t now = DWT->CYCCNT;
  if (p->primed) {
    uint32_t delta = now - p->prev;
    p->last = delta;
    if (delta > p->max) p->max = delta;
    if (delta < p->min) p->min = delta;
  } else {
    p->primed = true;
    p->min = 0xFFFFFFFFUL;
    p->max = 0;
  }
  p->prev = now;
}

// Profile_Begin/End 1組あたりのオーバーヘッドを実測する [サイクル]。
//
// 「測るという行為そのものが結果に乗る」ぶんを知らないと、出てきた数字の
// どこまでが本当の処理時間なのか判断できない。区間を細かく分けるほど効く。
//
// 外側の CYCCNT 読み2回ぶんを別途測って差し引く。SysTickに割り込まれた回は
// 大きく出るので、最小値を採用して弾く。
static inline uint32_t Profile_MeasureOverhead(void) {
  static Profile dummy;
  uint32_t best_pair = 0xFFFFFFFFUL;
  uint32_t best_base = 0xFFFFFFFFUL;

  for (int i = 0; i < 32; i++) {
    uint32_t a = DWT->CYCCNT;
    Profile_Begin(&dummy);
    Profile_End(&dummy);
    uint32_t b = DWT->CYCCNT;
    uint32_t d = b - a;
    if (d < best_pair) best_pair = d;
  }
  for (int i = 0; i < 32; i++) {
    uint32_t a = DWT->CYCCNT;
    uint32_t b = DWT->CYCCNT;
    uint32_t d = b - a;
    if (d < best_base) best_base = d;
  }

  return (best_pair > best_base) ? (best_pair - best_base) : 0;
}

// ---------------------------------------------------------------------------
// 集計の取り出し (メインループから呼ぶ)
// ---------------------------------------------------------------------------
// sum は64bitなので、割り込みに割り込まれると上位と下位で別の値を読む
// (ティアリング) 可能性がある。読んでいる間だけ割り込みを止める。
// 割り込みの中で計算までやると20kHzループを止めてしまうので、
// 平均の割り算はクリティカルセクションの外で行う。
static inline void Profile_Snapshot(Profile* p, ProfileResult* out, bool reset) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t last = p->last;
  uint32_t max = p->max;
  uint64_t sum = p->sum;
  uint32_t count = p->count;
  if (reset) {
    p->max = 0;
    p->sum = 0;
    p->count = 0;
  }

  __set_PRIMASK(primask);

  out->last = last;
  out->max = max;
  out->min = 0;
  out->count = count;
  out->total = (uint32_t)sum;
  out->mean = (count > 0) ? ((float)(uint32_t)(sum / count)) : 0.0f;
}

static inline void ProfilePeriod_Snapshot(ProfilePeriod* p, ProfileResult* out, bool reset) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  out->last = p->last;
  out->max = p->max;
  out->min = (p->min == 0xFFFFFFFFUL) ? 0 : p->min;
  if (reset) {
    p->min = 0xFFFFFFFFUL;
    p->max = 0;
  }

  __set_PRIMASK(primask);

  out->count = 0;
  out->total = 0;
  out->mean = 0.0f;
}

// ---------------------------------------------------------------------------
// 単位変換
// ---------------------------------------------------------------------------
// SystemCoreClock は HAL が実クロックに合わせて更新するので、
// PLL設定を変えてもここは直さなくてよい。
static inline float Profile_CyclesToUs(float cycles) {
  return cycles * (1000000.0f / (float)SystemCoreClock);
}

// CPU使用率 [%] = 区間内の合計サイクル数 / 実経過時間ぶんのサイクル数。
//
// 「1回あたりの時間 × 想定周波数」で出してはいけない。想定が間違っていると
// そのまま間違った答えが出る。実際この方式で最初 20kHz を仮定して測ったところ、
// DMAのハーフ転送完了割り込みで**倍のレートで割り込んでいた**ことに気づけず、
// CPU使用率が半分に見えていた。実測した合計と実経過時間だけで出せば嘘をつかない。
static inline float Profile_CpuPercent(uint32_t total_cycles, float elapsed_s) {
  if (elapsed_s <= 0.0f) return 0.0f;
  return (float)total_cycles * (100.0f / (elapsed_s * (float)SystemCoreClock));
}

// 実測した呼び出し頻度 [Hz]。想定と違っていないか必ず確認すること。
static inline float Profile_RateHz(uint32_t count, float elapsed_s) {
  if (elapsed_s <= 0.0f) return 0.0f;
  return (float)count / elapsed_s;
}

#endif  // PROFILE_H_
