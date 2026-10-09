#include "bldc_internal.h"

// エンコーダ/モータ定数の校正・同定。起動時に1回だけ (スイッチ校正時、または
// config.h の MEASURE_* を有効にしたとき) 実行される。共有状態は bldc_internal.h。

#if BLDC_NEED_RL || BLDC_NEED_PSI
#include <math.h>  // logf / asinf (起動時の同定のみ。ISRからは呼ばない)
#endif

// 強制転流(オープンループ駆動)。エンコーダ校正でのみ使う。
// sin/cos を1回だけ引き、加法定理 cos(φ ∓ 2π/3) = -0.5·cosφ ± (√3/2)·sinφ で3相に展開する。
// FOC_SinCos は範囲を問わないので位相の正規化は不要。
static void BLDC_OpenLoopDrive(float amp, float phase) {
  float sin_p, cos_p;
  FOC_SinCos(phase, &sin_p, &cos_p);

  float half_amp = 0.5f * amp;
  float a = half_amp * cos_p;                   // U相の振れ幅
  float b = half_amp * (SQRT3 * 0.5f) * sin_p;  // V/W相の直交成分

  BLDC_WritePwm(0.5f + a, 0.5f - 0.5f * a + b, 0.5f - 0.5f * a - b);
}

#if BLDC_MEASURING
// --- ステップ応答測定の共通部分 ---
// 測定前の状態づくり。q軸指令0のトルクモードで出力を有効にし、電流が整定するのを待つ
// (ステップ直前の値 = サンプル0 = 基準を揃えるため)。
static void BLDC_BeginMeasurement(void) {
  svc.target_id_override = 0.0f;
  svc.target_current = 0.0f;
  svc.mode = BLDC_MODE_TORQUE;
  svc.enable = true;
  HAL_Delay(200);
}

// 記録したIdの定常値 [A]。後ろ20%の平均
static float BLDC_CaptureSteadyState(void) {
  const uint16_t tail = BLDC_CAPTURE_SAMPLES / 5;
  int32_t sum = 0;
  for (uint16_t i = BLDC_CAPTURE_SAMPLES - tail; i < BLDC_CAPTURE_SAMPLES; i++) {
    sum += bldc_capture[i];
  }
  return (float)sum / (float)tail * 0.001f;
}

// 63.2%到達点をサンプル単位で返す (0 なら見つからなかった)。
// またぐ2点を線形補間する (しないと時定数を長く読み、1000Hzの1次系が 796Hz と出る。補間ありは 992Hz)。
static float BLDC_CaptureTimeTo63(float i_ss, uint16_t* out_index) {
  float thr = i_ss * 0.632f;
  for (uint16_t i = 1; i < BLDC_CAPTURE_SAMPLES; i++) {
    float cur = (float)bldc_capture[i] * 0.001f;
    if (cur >= thr) {
      float prev = (float)bldc_capture[i - 1] * 0.001f;
      float d = cur - prev;
      *out_index = i;
      return (d > 1e-6f) ? ((float)(i - 1) + (thr - prev) / d) : (float)i;
    }
  }
  *out_index = 0;
  return 0.0f;
}

// 波形を表示する (前半は1サンプルずつ、後半は間引く)
static void BLDC_CapturePrint(float i_ss) {
  printf("   t[ms]   Id[A]\n");
  for (uint16_t i = 0; i < BLDC_CAPTURE_SAMPLES; i += (i < 20) ? 1 : 10) {
    float v = (float)bldc_capture[i] * 0.001f;
    int bars = (i_ss > 0.001f) ? (int)(v / i_ss * 40.0f) : 0;
    if (bars < 0) bars = 0;
    if (bars > 50) bars = 50;
    printf("  %6.2f  %+6.3f |", (double)(i * CURRENT_LOOP_DT * 1000.0f), (double)v);
    for (int b = 0; b < bars; b++) printf("#");
    printf("\n");
  }
}

// ステップを1回打って記録が埋まるまで待つ。成功したら true。
static bool BLDC_RunCapture(float step, bool is_voltage) {
  svc.capture_step = step;
  svc.capture_is_voltage = is_voltage;
  svc.capture_state = BLDC_CAPTURE_ARMED;

  // 記録は10msで終わるので、500msで打ち切る
  uint32_t t0 = HAL_GetTick();
  while (svc.capture_state != BLDC_CAPTURE_DONE && (HAL_GetTick() - t0) < 500) {
  }
  bool ok = (svc.capture_state == BLDC_CAPTURE_DONE);
  svc.capture_state = BLDC_CAPTURE_IDLE;
  if (!ok) {
    printf("  記録が完了しなかった (%u/%u)。制御ループが回っているか確認すること。\n",
           svc.capture_index, (unsigned)BLDC_CAPTURE_SAMPLES);
  }
  return ok;
}
#endif  // BLDC_MEASURING

#if BLDC_NEED_RL
// ---------------------------------------------------------------------------
// L と R の同定 (PIをバイパスした電圧ステップ)
// ---------------------------------------------------------------------------
// 応答は素の1次系: i(t) = (V/R)(1 - e^(-t/τ)), τ = L/R。定常値から R、63%到達から L = τ·R。
// 電圧を2点 (V と 2V) 振って R = (V2 - V1) / (I2 - I1) とし、ほぼ一定のデッドタイム誤差
// (278ns, 12Vで約0.067V) を消す。1点だと R に10%近い誤差が乗る。
bool BLDC_MeasureMotorRL(float* out_r, float* out_l) {
  const float v1 = MOTOR_RL_STEP_VOLTS;
  const float v2 = MOTOR_RL_STEP_VOLTS * 2.0f;
  float i1 = 0.0f, i2 = 0.0f, tau = 0.0f;

  printf("[RL] L と R の同定 (Vd を直接印加、PIはバイパス)\n");

  BLDC_BeginMeasurement();

  svc.rl_aborted = false;
  svc.vd_override = 0.0f;
  svc.vd_override_active = true;

  for (uint8_t pass = 0; pass < 2; pass++) {
    float v = (pass == 0) ? v1 : v2;
    if (!BLDC_RunCapture(v, true)) break;

    float i_ss = BLDC_CaptureSteadyState();
    uint16_t k;
    float k63 = BLDC_CaptureTimeTo63(i_ss, &k);
    printf("  V=%.3fV → 定常 %.3fA, 63%%到達 %.2f サンプル (%.3fms)\n",
           (double)v, (double)i_ss, (double)k63, (double)(k63 * CURRENT_LOOP_DT * 1000.0f));

    if (pass == 0) {
      i1 = i_ss;
    } else {
      i2 = i_ss;
      tau = k63 * CURRENT_LOOP_DT;  // 電流が大きいほうがSNRが良い
      BLDC_CapturePrint(i_ss);
    }

    // 熱を溜めないよう電流を落とす
    svc.vd_override = 0.0f;
    HAL_Delay(100);
    if (svc.rl_aborted) break;
  }

  svc.vd_override = 0.0f;
  svc.vd_override_active = false;
  BLDC_Stop();

  if (svc.rl_aborted) {
    printf("  中断: 電流が %.1fA を超えた。MOTOR_RL_STEP_VOLTS を下げること。\n",
           (double)MOTOR_RL_ABORT_AMPS);
    return false;
  }
  if (i2 - i1 < 0.05f) {
    printf(
        "  電流差が小さすぎて R を求められない (I1=%.3f I2=%.3f)。\n"
        "  MOTOR_RL_STEP_VOLTS を上げること。\n",
        (double)i1, (double)i2);
    return false;
  }

  float r = (v2 - v1) / (i2 - i1);
  float r_naive = v2 / i2;  // 1点だけの場合 (比較用)

  printf("  → **MOTOR_R = %.4f** [Ω]  (1点だけなら %.4f。差がデッドタイム誤差)\n",
         (double)r, (double)r_naive);
  if (tau <= 0.0f) {
    printf("  τ を検出できなかった。L は求められない。\n");
    return false;
  }

  // 63%到達時間には L/R 以外の遅れが2つ混ざっており、引かないと L を4割ほど過大評価する。
  //   (a) 電流LPFの時定数 τ = -Ts / ln(1-a)  (a=0.5, Ts=50µs で 72µs)
  //   (b) 電圧を出してからサンプリングするまでの半周期 (CCRはプリロード付きで、電流を拾うのは
  //       低側ON区間の中央なので、サンプル n の経過時間は (n - 0.5)·Ts)
  // 1次系の縦続なので63%点は時定数の和にほぼ等しく、差し引きの誤差は数%。
  const float tau_lpf = -CURRENT_LOOP_DT / logf(1.0f - CURRENT_LPF_COEF);
  const float tau_delay = 0.5f * CURRENT_LOOP_DT;
  float tau_lr = tau - tau_lpf - tau_delay;

  printf("  → τ(実測) %.3fms − LPF %.3fms − 半周期 %.3fms = **τ(L/R) %.3fms**\n",
         (double)(tau * 1000.0f), (double)(tau_lpf * 1000.0f),
         (double)(tau_delay * 1000.0f), (double)(tau_lr * 1000.0f));

  if (tau_lr <= 0.0f) {
    printf(
        "  補正後のτが0以下。L/R がフィルタの時定数より速く、この方法では測れない。\n"
        "  CURRENT_LPF_COEF を大きくして測り直すこと。\n");
    return false;
  }

  float l = tau_lr * r;
  printf("  → **MOTOR_L = %.6f** [H] (= %.1f µH)  ※補正なしなら %.1f µH\n",
         (double)l, (double)(l * 1e6f), (double)(tau * r * 1e6f));
  printf("  → R/L = %.0f rad/s  (PIのゼロ点 Ki/Kp をここに合わせる)\n", (double)(r / l));
  printf("  → この値で %.0fHz にすると Kp=%.4f Ki=%.1f\n",
         (double)CURRENT_BW_HZ, (double)(l * 6.28318531f * CURRENT_BW_HZ),
         (double)(r * 6.28318531f * CURRENT_BW_HZ));
  if (tau_lr < 3.0f * CURRENT_LOOP_DT) {
    printf(
        "  注意 補正後のτが %.1f サンプルしかない。50µsサンプリングに対して速すぎるので\n"
        "       L の値は誤差が大きい。\n",
        (double)(tau_lr / CURRENT_LOOP_DT));
  }
  *out_r = r;
  *out_l = l;
  return true;
}
#endif  // BLDC_NEED_RL

#if BLDC_NEED_PSI
// ---------------------------------------------------------------------------
// 鎖交磁束 ψm の同定
// ---------------------------------------------------------------------------
// 定常状態では dIq/dt = 0 なので Vq = R·Iq + ω_e·ψm。2速度の差を取り、デッドタイム等の
// 一定オフセットを消す: ψm = ((Vq2 - Vq1) - R(Iq2 - Iq1)) / (ω_e2 - ω_e1)
// フィードフォワードが有効でも成立する (必要な Vq の総量は ψm の真値だけで決まり、PIとFFの内訳は無関係)。

// 指定速度まで回して整定させ、定常点の ω_e / Vq / Iq の平均を返す (ψm と角度遅れの同定で共用)。
static bool BLDC_MeasureSteadyPoint(float target_w, float* out_we, float* out_vq, float* out_iq) {
  // 校正は通信開始前 (BLDC_Init 内) に走り、制限値が既定の 0 だとモータが回らないので、
  // ここで MD 側の絶対上限まで開ける。閉じるのは BLDC_Init の最後。
  BLDC_SetLimits(BLDC_GetMaxTorqueNm());

  BLDC_AngularSpeedControl(target_w);

  // 加速度制限でランプするので、到達と整定を待つ
  HAL_Delay(2500);

  // 500ms 平均 (リプルや速度のゆらぎを均す)
  const uint16_t n = 500;
  float sum_w = 0.0f, sum_vq = 0.0f, sum_iq = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    sum_w += svc.angular_speed;
    sum_vq += svc.vq;
    sum_iq += svc.iq;
    HAL_Delay(1);
  }
  float w = sum_w / (float)n;
  *out_we = w * POLE_PAIRS;
  *out_vq = sum_vq / (float)n;
  *out_iq = sum_iq / (float)n;

  printf("  %.0f rad/s 指令 → 実測 %.1f rad/s (ω_e %.0f), Vq %.3fV, Iq %.3fA\n",
         (double)target_w, (double)w, (double)*out_we, (double)*out_vq, (double)*out_iq);

  if (Abs(w - target_w) > target_w * 0.2f) {
    printf("  指令速度に届いていない。負荷がかかっているか電圧が足りない。\n");
    return false;
  }
  return true;
}

bool BLDC_MeasureMotorPsi(float* out_psi) {
  printf("[PSI] 逆起電力定数 ψm の同定\n");
  printf("  **モータが回ります。** 危険なら電源を切ってください。\n");
  for (int8_t i = 3; i > 0; i--) {
    printf("  %d...\n", i);
    HAL_Delay(1000);
  }

  float we1, vq1, iq1, we2, vq2, iq2;
  bool ok = BLDC_MeasureSteadyPoint(MOTOR_PSI_SPEED1, &we1, &vq1, &iq1) &&
            BLDC_MeasureSteadyPoint(MOTOR_PSI_SPEED2, &we2, &vq2, &iq2);
  BLDC_Stop();

  if (!ok) return false;
  if (we2 - we1 < 1.0f) {
    printf("  2点の速度差が小さすぎる。MOTOR_PSI_SPEED1/2 を離すこと。\n");
    return false;
  }

  float psi = ((vq2 - vq1) - svc.motor_r * (iq2 - iq1)) / (we2 - we1);
  float psi_1pt = (vq2 - svc.motor_r * iq2) / we2;  // 1点だけの場合 (比較用)

  printf("  → **MOTOR_PSI = %.6f** [Wb]  (1点だけなら %.6f。差が一定オフセット分)\n",
         (double)psi, (double)psi_1pt);

  // 2点法は「オフセットが2点で同じ」ことが前提なので、各点から逆算して確かめる
  //   Vq = R·Iq + ω_e·ψm + V0  →  V0 = Vq − R·Iq − ω_e·ψm
  float v0_1 = vq1 - svc.motor_r * iq1 - we1 * psi;
  float v0_2 = vq2 - svc.motor_r * iq2 - we2 * psi;
  printf("  逆算したオフセット: %.3fV / %.3fV\n", (double)v0_1, (double)v0_2);
  if (Abs(v0_1 - v0_2) > 0.05f) {
    printf(
        "  警告 2点でオフセットが揃っていない。2点法の前提が崩れているので\n"
        "       ψm は当てにならない。MOTOR_PSI_SPEED1/2 を上げて測り直すこと\n"
        "       (速度が高いほど ω_e·ψm が大きくなり、オフセットの影響が減る)。\n");
  }

  if (psi > 1e-6f) {
    // 無負荷の到達速度の目安 (実際と大きく違えば符号・極対数・電圧を疑う)
    float v_limit = svc.supply_volt * MAX_MODULATION_RATIO;
    printf("  → 逆起電力定数 %.4f V/(rad/s) [電気角]\n", (double)psi);
    printf("  → 電圧上限 %.2fV から無負荷の到達速度は約 %.0f rad/s (機械角) の見込み\n",
           (double)v_limit, (double)(v_limit / (psi * POLE_PAIRS)));
    *out_psi = psi;
    return true;
  }
  printf("  ψm が負またはゼロ。Vq の符号か POLE_PAIRS を疑うこと。\n");
  return false;
}

// ---------------------------------------------------------------------------
// 電気角の実効遅れの同定
// ---------------------------------------------------------------------------
// 推定角が δ ずれると逆起電力が推定d軸に漏れる: Vd = R·Id − ω_e·L·Iq + ω_e·ψm·sin δ
//   sin δ = (Vd − R·Id + ω_e·L·Iq) / (ω_e·ψm)
// δ は現在の補償値で回したときの残りなので、真の遅れは t_true = t_いま − δ / ω_e。
// 1回の測定で決まる (検算: 補償前 1.074ms、補償後 1.077ms)。ψm が必要なので ψm の同定より後に呼ぶ。
bool BLDC_MeasureAngleDelay(float* out_delay) {
  const float target_w = ANGLE_DELAY_MEASURE_SPEED;
  printf("[DLY] 電気角の実効遅れの同定 (%.0f rad/s で回す)\n", (double)target_w);

  float we, vq, iq;
  if (!BLDC_MeasureSteadyPoint(target_w, &we, &vq, &iq)) {
    BLDC_Stop();
    return false;
  }

  // Vd と Id は BLDC_MeasureSteadyPoint が拾わないのでここで平均する
  const uint16_t n = 300;
  float sum_vd = 0.0f, sum_id = 0.0f;
  for (uint16_t i = 0; i < n; i++) {
    sum_vd += svc.vd;
    sum_id += svc.id;
    HAL_Delay(1);
  }
  float vd = sum_vd / (float)n;
  float id = sum_id / (float)n;
  BLDC_Stop();

  float denom = we * svc.motor_psi;
  if (denom < 1e-6f) {
    printf("  ω_e·ψm が小さすぎて解けない。ψm の同定を先に済ませること。\n");
    return false;
  }

  float sin_d = (vd - svc.motor_r * id + we * svc.motor_l * iq) / denom;
  printf("  Vd %.3fV, Id %.3fA, ω_e %.0f, いまの補償 %.3fms → sinδ = %.4f\n",
         (double)vd, (double)id, (double)we, (double)(svc.angle_delay * 1000.0f),
         (double)sin_d);

  if (sin_d > 0.9f || sin_d < -0.9f) {
    printf(
        "  δ が90°に近く、この式では精度が出ない。\n"
        "  ANGLE_DELAY_MEASURE_SPEED を下げるか、既定値を実際に近づけること。\n");
    return false;
  }

  float delta = asinf(sin_d);
  float delay = svc.angle_delay - delta / we;
  printf("  → δ = %+.1f° → **ANGLE_DELAY = %.5f** [s] (= %.3f ms)\n",
         (double)(delta * 57.29578f), (double)delay, (double)(delay * 1000.0f));

  // 負や極端に大きい値は測定失敗。既定値のほうがまだ安全。
  if (!(delay > 0.0f && delay < 0.01f)) {
    printf("  値が範囲外。採用しない。\n");
    return false;
  }
  *out_delay = delay;
  return true;
}
#endif  // BLDC_NEED_PSI

#if MEASURE_CURRENT_STEP
// ---------------------------------------------------------------------------
// 電流ループ(閉ループ)のステップ応答測定
// ---------------------------------------------------------------------------
// d軸に電流ステップを入れて Id の応答を記録し、帯域を求める。
// d軸ならトルクが出ず逆起電力も混ざらないので電気的な応答だけが取れ、SPM (Ld = Lq) なので
// 結果はq軸にそのまま使える。
static void BLDC_AnalyzeCurrentStep(float step_amps) {
  float i_ss = BLDC_CaptureSteadyState();

  printf("[STEP] 電流ループのステップ応答 (Id指令 %.2fA)\n", (double)step_amps);

  // 定常値が指令から大きく外れていたら電流が流せていない (電源電圧不足・出力無効・過電流ラッチなど)
  if (i_ss < step_amps * 0.5f) {
    printf(
        "  定常値 %.3fA が指令 %.2fA に届いていない。測定は無効。\n"
        "  電源電圧・過電流ラッチ・エンコーダ校正を確認すること。\n",
        (double)i_ss, (double)step_amps);
    return;
  }

  uint16_t k63;
  float k63f = BLDC_CaptureTimeTo63(i_ss, &k63);

  // オーバーシュート (15%超ならゲインが高すぎる)
  int16_t peak = bldc_capture[0];
  for (uint16_t i = 0; i < BLDC_CAPTURE_SAMPLES; i++) {
    if (bldc_capture[i] > peak) peak = bldc_capture[i];
  }
  float overshoot = ((float)peak * 0.001f - i_ss) / i_ss * 100.0f;

  printf("  定常値 %.3fA  63%%到達 %.2f サンプル (%.3fms)  オーバーシュート %.1f%%\n",
         (double)i_ss, (double)k63f, (double)(k63f * CURRENT_LOOP_DT * 1000.0f),
         (double)overshoot);

  if (k63 == 0 || k63f <= 0.0f) {
    printf("  1サンプル(50µs)以内に立ち上がっている。帯域は測定限界の外。\n");
    BLDC_CapturePrint(i_ss);
    return;
  }

  float tau = k63f * CURRENT_LOOP_DT;

  // 本当に1次系か検証する (1次系なら 2τ で 86.5%、3τ で 95.0%)。極零相殺できていないと
  // 速い成分と遅い成分に割れて長い尾を引き、63%点だけでは帯域を過大評価する。
  // τ は整数サンプルとは限らないので、ここも線形補間する (切り捨てると判定が不安定になる)。
  const float frac[3] = {0.632f, 0.865f, 0.950f};
  float dev[3] = {0.0f, 0.0f, 0.0f};
  printf("  1次系との比較:");
  for (uint8_t m = 1; m <= 3; m++) {
    float fidx = k63f * (float)m;
    uint16_t i0 = (uint16_t)fidx;
    if (i0 > BLDC_CAPTURE_SAMPLES - 2) i0 = BLDC_CAPTURE_SAMPLES - 2;
    float f = fidx - (float)i0;
    float v0 = (float)bldc_capture[i0] * 0.001f;
    float v1 = (float)bldc_capture[i0 + 1] * 0.001f;
    float actual = (v0 + f * (v1 - v0)) / i_ss;
    dev[m - 1] = (actual - frac[m - 1]) * 100.0f;
    printf("  %uτ %.0f%%(理想%.0f%%)", m, (double)(actual * 100.0f),
           (double)(frac[m - 1] * 100.0f));
  }
  printf("\n");

  // 判定の主役は 3τ。極零相殺が合っていないと 3τ でも 95% に届かない尾が残る。
  // 2τ が高めに出るのはループ内の遅れ (電流LPFと演算遅れ) で τ を長く読むためで、異常ではない。
  // 実測: 壊れていたとき 2τ +17% / 3τ -13%、直したあと 2τ +9% / 3τ +4%。
  if (dev[2] < -8.0f) {
    printf(
        "  **遅い尾が残っている (3τ で %+.0f%%)。** PIのゼロ点 Ki/Kp が\n"
        "  プラントの極 R/L と合っていないと、応答が速い成分と遅い成分に割れる。\n"
        "  MEASURE_MOTOR_RL で L と R を実測して MOTOR_L / MOTOR_R を直すこと。\n"
        "  この状態では下の帯域の値は当てにならない。\n",
        (double)dev[2]);
  } else if (dev[1] > 20.0f) {
    printf(
        "  **応答が1次系から外れている (2τ で %+.0f%%)。**\n"
        "  ゲインが高すぎて振動しているか、ループ内の遅れが想定より大きい。\n",
        (double)dev[1]);
  } else {
    // L の同定と同じく半周期を引く (引かないと帯域を1割ほど低く読む)
    float tau_corr = tau - 0.5f * CURRENT_LOOP_DT;
    if (tau_corr <= 0.0f) tau_corr = tau;
    float f_bw = 1.0f / (TWO_PI_F * tau_corr);
    printf("  → 1次系とみなせる。**帯域 約%.0fHz** (狙い %.0fHz, 補正前 %.0fHz)\n",
           (double)f_bw, (double)CURRENT_BW_TARGET_HZ, (double)(1.0f / (TWO_PI_F * tau)));
    // 基準は CURRENT_BW_TARGET_HZ (config.h 参照)
    printf("  → 狙いに合わせるなら CURRENT_BW_HZ = %.0f (いまは %.0f)\n",
           (double)(CURRENT_BW_HZ * CURRENT_BW_TARGET_HZ / f_bw), (double)CURRENT_BW_HZ);
  }

  if (k63 < 3) {
    printf(
        "  注意 63%%到達が %u サンプルしかない。50µsサンプリングに対して速すぎるので\n"
        "       帯域の値は誤差が大きい。\n",
        k63);
  }
  if (overshoot > 15.0f) {
    printf("  警告 オーバーシュートが大きい。CURRENT_BW_HZ を下げること。\n");
  }

  BLDC_CapturePrint(i_ss);
}

// 注意: モータに電流が流れる。d軸なのでトルクは出ないはずだが、エンコーダ校正がずれていると
// ロータが動く。校正済みで、ロータが自由に回っても安全な状態で使うこと。
void BLDC_MeasureCurrentStep(void) {
  const float step_amps = Constrain(CURRENT_STEP_AMPS, 0.0f, MAX_CURRENT);

  BLDC_BeginMeasurement();  // q軸は0のまま、d軸だけを動かす

  bool ok = BLDC_RunCapture(step_amps, false);

  svc.target_id_override = 0.0f;
  BLDC_Stop();

  if (ok) BLDC_AnalyzeCurrentStep(step_amps);
}
#endif  // MEASURE_CURRENT_STEP

// エンコーダの最大/最小値・盲点幅・電気角のオフセットを実測する (保存は BLDC_SaveCalibration)
void BLDC_SetEncoder(volatile uint16_t* encoder_val) {
  svc.encoder_offset_theta = 0;
  svc.max_encoder_val = 0;
  svc.min_encoder_val = MAX_ADC_VAL;

  // 強制転流で等速に回しながら測る。盲点幅は「時間の割合＝機械角の割合」として求めるので、
  // 2周とも同じ速度・同じ駆動条件で回すこと。
  const uint16_t SWEEP_SAMPLES = 3000;  // HAL_Delay(1) なので3秒 = 約13回転
  const float DRIVE_AMP = 0.15f;        // 強制転流の変調振幅 (回すのに使う)
  const float HOLD_AMP = 0.3f;          // 位置を引き込んで保持するときの振幅
  const float SWEEP_STEP = 0.2f;        // 1サンプルあたりの電気角の進み [rad]

  // 1周目: 最大値・最小値を取得する
  float phase = 0;
  for (uint16_t i = 0; i < SWEEP_SAMPLES; i++) {
    phase += SWEEP_STEP;
    if (svc.max_encoder_val < *encoder_val) svc.max_encoder_val = *encoder_val;
    if (svc.min_encoder_val > *encoder_val) svc.min_encoder_val = *encoder_val;
    BLDC_OpenLoopDrive(DRIVE_AMP, phase);
    HAL_Delay(1);
  }

  // 2周目: 盲点(レール飽和で角度が読めない区間)の幅を、飽和していたサンプルの割合から求める。
  // 無視すると継ぎ目に段差ができる (BLDC_UpdateEncoderScale 参照)。約13回転なので端数誤差は 1/13 程度。
  uint32_t sat_samples = 0;
  for (uint16_t i = 0; i < SWEEP_SAMPLES; i++) {
    phase += SWEEP_STEP;
    if (BLDC_IsEncoderSaturated(*encoder_val)) sat_samples++;
    BLDC_OpenLoopDrive(DRIVE_AMP, phase);
    HAL_Delay(1);
  }
  svc.encoder_dead_zone = TWO_PI_F * (float)sat_samples / (float)SWEEP_SAMPLES;

  // 異常値 (盲点が1周の1/4以上 = エンコーダか磁石の取り付けを疑う) は補正なしにする
  if (!(svc.encoder_dead_zone >= 0.0f && svc.encoder_dead_zone < TWO_PI_F * 0.25f)) {
    svc.encoder_dead_zone = 0.0f;
  }

  BLDC_UpdateEncoderScale();  // これ以降 BLDC_UpdateEncoder が使える

  // 電気角0の位置にロータを引き込み、そのときの機械角をオフセットとする。
  // 電気角1回転ぶんずつ位置をずらしながら POLE_PAIRS 回くり返して平均する。
  // 測定点は機械角で 2π/P ずつ離れるが、平均は θ0 と 2π/P の整数倍しか違わず、
  // 電気角では 2π/P のずれ = 2π のずれ = 同一なので、そのまま平均してよい
  // (各測定値が等間隔で同じ誤差を持つことが前提)。
  float offset_sum = 0;
  for (uint8_t i = 0; i < POLE_PAIRS; i++) {
    BLDC_OpenLoopDrive(DRIVE_AMP, 0);
    HAL_Delay(200);
    BLDC_OpenLoopDrive(HOLD_AMP, 0);
    HAL_Delay(100);

    // ロータは直前の位相送りで約0.9rad動いている。オブザーバは50µs前提で、この校正ループ
    // (HAL_Delay(1)) では追従が20倍遅く、イノベーション上限も超えて棄却され続ける。
    // 張り直さないと追従中のランプが平均に入り、オフセットが約70°(電気角) ずれる。
    svc.encoder_primed = false;
    BLDC_UpdateEncoder(*encoder_val);

    // 0/2π の境目で壊れないよう、1点目からの差分で平均する
    float base = svc.mech_theta;
    float diff_sum = 0;
    for (uint16_t j = 0; j < 200; j++) {
      BLDC_UpdateEncoder(*encoder_val);
      diff_sum += FOC_AngleDiff(svc.mech_theta, base);
      HAL_Delay(1);
    }
    offset_sum += FOC_NormalizeRadians(base + diff_sum * 0.005f);

    // 電気角を約1回転送って次の測定点へ進める (次のループ頭で phase=0 ≡ 2π に引き込まれるので端数は問題ない)
    phase = 0;
    for (uint16_t j = 0; j < (uint16_t)(TWO_PI_F * 10.0f); j++) {
      phase += 0.1f;
      BLDC_OpenLoopDrive(DRIVE_AMP, phase);
      HAL_Delay(1);
    }
  }
  svc.encoder_offset_theta = offset_sum / POLE_PAIRS;

  printf("(Measure)max: %lu, min: %lu, offset: %.6f rad, dead_zone: %.4f rad (%.2f°, 1周の%.1f%%)\n",
         (unsigned long)svc.max_encoder_val,
         (unsigned long)svc.min_encoder_val,
         svc.encoder_offset_theta,
         svc.encoder_dead_zone,
         (double)(svc.encoder_dead_zone * 57.29578f),
         (double)(svc.encoder_dead_zone * 100.0f / TWO_PI_F));
}

#if MOTOR_AUTO_CALIBRATION
// 校正値をまとめてフラッシュに保存する。ページ消去を伴うので、必ず1回で全項目を書くこと。
void BLDC_SaveCalibration(void) {
  BLDCFlashData d = {BLDC_FLASH_MAGIC,
                     svc.max_encoder_val,
                     svc.min_encoder_val,
                     svc.encoder_offset_theta,
                     svc.encoder_dead_zone,
                     svc.motor_r,
                     svc.motor_l,
                     svc.motor_psi,
                     svc.angle_delay};
  if (Flash_WriteData(FLASH_USER_START_ADDR, &d, sizeof(d)) == HAL_OK) {
    printf("[CAL] フラッシュに保存した\n");
  } else {
    printf("[CAL] フラッシュ書き込みに失敗した\n");
  }
}
#endif  // MOTOR_AUTO_CALIBRATION

// 電流PIのゲインを、いまの L と R から計算し直す (L,R を実測したら必ず呼ぶ)
void BLDC_UpdateCurrentGains(void) {
  svc.id_pi.kp = CURRENT_PI_KP_FROM(svc.motor_l);
  svc.id_pi.ki = CURRENT_PI_KI_FROM(svc.motor_r);
  svc.id_pi.output_limit = svc.supply_volt * MAX_MODULATION_RATIO;
  FOC_PI_Reset(&svc.id_pi);
  svc.iq_pi = svc.id_pi;
  printf("[CAL] 電流PI: Kp=%.4f Ki=%.1f (Ki/Kp=%.0f = R/L, 目標帯域 %.0fHz)\n",
         (double)svc.id_pi.kp, (double)svc.id_pi.ki,
         (double)(svc.motor_r / svc.motor_l), (double)CURRENT_BW_HZ);
}

#if MOTOR_AUTO_CALIBRATION
// モータ定数の自動測定 (制御ループが回り始めてから呼ぶ)。順序の理由は config.h の MOTOR_AUTO_CALIBRATION。
// 失敗した項目は既定値のまま続行する。
void BLDC_CalibrateMotor(void) {
  float r, l, psi, delay;

  printf("[CAL] モータ定数の自動測定を開始\n");

  if (BLDC_MeasureMotorRL(&r, &l)) {
    svc.motor_r = r;
    svc.motor_l = l;
    BLDC_UpdateCurrentGains();
  } else {
    printf("[CAL] R,L の測定に失敗。既定値のまま続行する\n");
  }

  if (BLDC_MeasureMotorPsi(&psi)) {
    svc.motor_psi = psi;
  } else {
    printf("[CAL] ψm の測定に失敗。既定値のまま続行する\n");
  }

  if (BLDC_MeasureAngleDelay(&delay)) {
    svc.angle_delay = delay;
  } else {
    printf("[CAL] 角度遅れの測定に失敗。既定値のまま続行する\n");
  }

  printf("[CAL] 結果: R=%.4fΩ L=%.6fH(%.1fµH) ψm=%.6fWb 遅れ=%.3fms\n",
         (double)svc.motor_r, (double)svc.motor_l, (double)(svc.motor_l * 1e6f),
         (double)svc.motor_psi, (double)(svc.angle_delay * 1000.0f));
}
#endif  // MOTOR_AUTO_CALIBRATION

// フラッシュの校正値を検証して適用する。未校正 (全ビット1) や旧フォーマットを
// マジックナンバーと範囲チェックで弾く (NaN も落とせるよう不等号は肯定形で書く)。
// エンコーダとモータ定数は別々に検証し、「もっともらしいゴミ」を避けるためモータ定数は範囲を厳しめにする。
void BLDC_LoadFlashCalibration(void) {
  BLDCFlashData d;
  Flash_ReadData(FLASH_USER_START_ADDR, &d, sizeof(d));

  bool enc_ok = (d.magic == BLDC_FLASH_MAGIC) &&
                (d.max_encoder_val > d.min_encoder_val) && (d.max_encoder_val <= MAX_ADC_VAL) &&
                (d.encoder_offset_theta > -TWO_PI_F && d.encoder_offset_theta < TWO_PI_F) &&
                (d.encoder_dead_zone >= 0.0f && d.encoder_dead_zone < TWO_PI_F * 0.25f);

  bool motor_ok = (d.magic == BLDC_FLASH_MAGIC) &&
                  (d.motor_r > 0.001f && d.motor_r < 100.0f) &&
                  (d.motor_l > 1e-6f && d.motor_l < 0.1f) &&
                  (d.motor_psi > 1e-5f && d.motor_psi < 1.0f) &&
                  (d.angle_delay >= 0.0f && d.angle_delay < 0.01f);

  svc.encoder_calibrated = enc_ok;
  if (enc_ok) {
    svc.max_encoder_val = d.max_encoder_val;
    svc.min_encoder_val = d.min_encoder_val;
    svc.encoder_offset_theta = d.encoder_offset_theta;
    svc.encoder_dead_zone = d.encoder_dead_zone;
    printf("(From Flash)max: %lu, min: %lu, offset: %.6f rad, dead_zone: %.4f rad\n",
           (unsigned long)d.max_encoder_val, (unsigned long)d.min_encoder_val,
           d.encoder_offset_theta, d.encoder_dead_zone);
  } else {
    printf("BLDC: エンコーダ校正値が不正です。スイッチを押しながら再起動して校正してください。\n");
    svc.min_encoder_val = 0;
    svc.max_encoder_val = MAX_ADC_VAL;
    svc.encoder_offset_theta = 0.0f;
    svc.encoder_dead_zone = 0.0f;
  }

  if (motor_ok) {
    svc.motor_r = d.motor_r;
    svc.motor_l = d.motor_l;
    svc.motor_psi = d.motor_psi;
    svc.angle_delay = d.angle_delay;
    printf("(From Flash)R: %.4fΩ, L: %.1fµH, ψm: %.6fWb, 遅れ: %.3fms\n",
           d.motor_r, (double)(d.motor_l * 1e6f), d.motor_psi,
           (double)(d.angle_delay * 1000.0f));
  } else {
    printf("BLDC: モータ定数が未校正です。config.h の既定値を使います。\n");
  }
}

// 起動時の診断 (config.h の MEASURE_*)。制御ループが回り始めてからでないと測れない。
void BLDC_RunStartupMeasurements(void) {
#if MEASURE_MOTOR_RL
  {
    float r, l;
    BLDC_MeasureMotorRL(&r, &l);
  }
#endif
#if MEASURE_CURRENT_STEP
  BLDC_MeasureCurrentStep();
#endif
#if MEASURE_MOTOR_PSI
  {
    float psi;
    BLDC_MeasureMotorPsi(&psi);
  }
#endif
}
