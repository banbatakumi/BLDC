#ifndef LPH_H_
#define LPH_H_

typedef struct {
      double current_val;
      double prev_val;
      double k_lpf;  // ローパスフィルタ係数
} LPF;

static inline void LPF_Init(LPF *lpf, double k_lpf, double initial_val) {
      lpf->k_lpf = k_lpf;
      lpf->prev_val = initial_val;
}

static inline double LPF_Update(LPF *lpf, double new_val) {
      // ローパスフィルタの更新
      lpf->current_val = lpf->k_lpf * lpf->prev_val + (1.0 - lpf->k_lpf) * new_val;
      lpf->prev_val = lpf->current_val;
      return lpf->current_val;
}

#endif  // LPH_H_