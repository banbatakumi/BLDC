#ifndef LPF_H_
#define LPF_H_

// 1次ローパスフィルタ (指数移動平均)
//   y ← k·y + (1-k)·x     k が大きいほど強くかかる
typedef struct {
  float value;      // 直近の出力 = 次回の入力にもなる内部状態
  float k_lpf;      // ローパスフィルタ係数
  float k_lpf_inv;  // (1.0 - k_lpf) 事前計算値
} LPF;

static inline void LPF_Init(LPF* lpf, float k_lpf, float initial_val) {
  lpf->k_lpf = k_lpf;
  lpf->k_lpf_inv = 1.0f - k_lpf;  // 初期化時に事前計算
  lpf->value = initial_val;
}

static inline float LPF_Update(LPF* lpf, float new_val) {
  lpf->value = lpf->k_lpf * lpf->value + lpf->k_lpf_inv * new_val;
  return lpf->value;
}

#endif  // LPF_H_
