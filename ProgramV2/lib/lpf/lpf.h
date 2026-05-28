#ifndef LPH_H_
#define LPH_H_

typedef struct {
  float current_val;
  float prev_val;
  float k_lpf;      // ローパスフィルタ係数
  float k_lpf_inv;  // (1.0 - k_lpf) 事前計算値
} LPF;

static inline void LPF_Init(LPF* lpf, float k_lpf, float initial_val) {
  lpf->k_lpf = k_lpf;
  lpf->k_lpf_inv = 1.0f - k_lpf;  // 初期化時に事前計算
  lpf->prev_val = initial_val;
}

static inline float LPF_Update(LPF* lpf, float new_val) {
  // ローパスフィルタの更新 (最適化版)
  lpf->current_val = lpf->k_lpf * lpf->prev_val + lpf->k_lpf_inv * new_val;
  lpf->prev_val = lpf->current_val;
  return lpf->current_val;
}

#endif  // LPH_H_