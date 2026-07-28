#include "foc.h"

// 電圧ベクトルの大きさを変調限界の円内に制限する
float FOC_LimitVoltageVector(float* vd, float* vq, float v_max) {
  float mag_sq = (*vd) * (*vd) + (*vq) * (*vq);
  float max_sq = v_max * v_max;
  if (mag_sq <= max_sq || mag_sq <= 0.0f) return 1.0f;

  float scale = v_max / FOC_Sqrt(mag_sq);
  *vd *= scale;
  *vq *= scale;
  return scale;
}

// 空間ベクトル変調 (min-max方式)
//   1. 逆Clarke変換で各相の電圧指令をつくる
//   2. 3相の最大値と最小値の中点を全相から引く (= 三次高調波の重畳)
//      線間電圧は変わらないままピーク電圧が √3/2 倍に下がるので、その分だけ
//      大きな相電圧振幅 (最大 Vdc/√3) を出せるようになる
//   3. デューティ 0.5 を中心に正規化する
void FOC_SVPWM(float v_alpha, float v_beta, float v_dc, float* du, float* dv, float* dw) {
  if (v_dc < 1.0f) v_dc = 1.0f;  // 0除算防止
  float inv_v_dc = 1.0f / v_dc;

  // 逆Clarke変換 (振幅一定型)
  float vu = v_alpha;
  float vv = -0.5f * v_alpha + (SQRT3 * 0.5f) * v_beta;
  float vw = -0.5f * v_alpha - (SQRT3 * 0.5f) * v_beta;

  // 3相の中点
  float v_max = vu;
  if (vv > v_max) v_max = vv;
  if (vw > v_max) v_max = vw;
  float v_min = vu;
  if (vv < v_min) v_min = vv;
  if (vw < v_min) v_min = vw;
  float v_com = 0.5f * (v_max + v_min);

  *du = 0.5f + (vu - v_com) * inv_v_dc;
  *dv = 0.5f + (vv - v_com) * inv_v_dc;
  *dw = 0.5f + (vw - v_com) * inv_v_dc;
}
