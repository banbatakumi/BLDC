// 位置制御のトルク上限 / アンチワインドアップのシミュレーション。
//
// bldc.c の BLDC_PIDControl と BLDC_OuterLoop (BLDC_MODE_POSITION) を
// **そのまま書き写して**、1kHz の外側ループだけを回す。
// 電流ループとモータの電気系は理想 (Iq指令 = 実Iq) とみなす。ここで確かめたいのは
// 制限とアンチワインドアップの振る舞いなので、その近似で十分。
//
// **速度制限は撤廃済み。** MD が上位から受け取る制限はトルクだけになったので、
// 位置制御も「目標角との偏差をそのまま PID に入れる」素の構成になっている。
// 以前あった位置指令のランプと引き戻し (POSITION_RAMP_LEAD_RAD) は、
// 速度制限が無いと毎周期 引き戻しが効いて出力が kp×引き戻し幅 の定数に化け、
// torque_limit が効かなくなるため撤廃した。[C] がその回帰テスト。
//
//   make -C tests
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define Constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define Abs(x) fabsf(x)

#define PI_F 3.14159265f
#define TWO_PI_F 6.28318531f

#define OUTER_LOOP_DT 0.001f
#define POSITION_DEADBAND_RAD 0.05f
#define POSITION_SETTLE_SPEED_RAD_S 0.05f
#define POSITION_INTEGRAL_STOP_RAD 0.05f

static float FOC_NormalizeRadians(float rad) {
  while (rad < 0.0f) rad += TWO_PI_F;
  while (rad >= TWO_PI_F) rad -= TWO_PI_F;
  return rad;
}
static float FOC_AngleDiff(float a, float b) {
  float diff = a - b;
  while (diff > PI_F) diff -= TWO_PI_F;
  while (diff < -PI_F) diff += TWO_PI_F;
  return diff;
}

// --- bldc.c から書き写し ---------------------------------------------------
typedef struct {
  float kp, ki, kd;
  float integral, prev_error, d_term, d_lpf;
  float output_limit;
} PIDController;

static float BLDC_PIDControl(PIDController* pid, float error, float dt, bool enable_integral) {
  float p_term = pid->kp * error;
  if (enable_integral) pid->integral += pid->ki * error * dt;

  float raw_d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->d_term = pid->d_term * pid->d_lpf + raw_d_term * (1.0f - pid->d_lpf);
  pid->prev_error = error;

  float raw_output = p_term + pid->integral + pid->d_term;
  float output = Constrain(raw_output, -pid->output_limit, pid->output_limit);
  if (enable_integral) pid->integral += output - raw_output;
  pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);
  return output;
}
// ---------------------------------------------------------------------------

typedef struct {
  PIDController pid;
  float target_position;
  float mech_theta;
  float angular_speed;
  float target_iq;
  float iq_limit;
  bool blocked;  // 軸を機械的に固定しているか
} Sim;

// bldc.c の BLDC_OuterLoop の BLDC_MODE_POSITION 分岐 + 最終クランプを書き写し
static void OuterLoopPosition(Sim* s) {
  const float iq_limit = s->iq_limit;
  s->pid.output_limit = iq_limit;

  float error = FOC_AngleDiff(s->target_position, s->mech_theta);
  float abs_error = Abs(error);

  if (abs_error < POSITION_INTEGRAL_STOP_RAD) s->pid.integral = 0;

  if (abs_error < POSITION_DEADBAND_RAD && Abs(s->angular_speed) < POSITION_SETTLE_SPEED_RAD_S) {
    s->pid.prev_error = error;
    s->pid.d_term = 0;
    s->target_iq = 0;
  } else {
    s->target_iq = BLDC_PIDControl(&s->pid, error, OUTER_LOOP_DT,
                                   abs_error >= POSITION_INTEGRAL_STOP_RAD);
  }

  s->target_iq = Constrain(s->target_iq, -iq_limit, iq_limit);
}

// 機械系。T = Kt·Iq、慣性と粘性のみ。拘束中は一切動かない。
#define KT 0.0195f         // [N・m/A]
#define MAX_CURRENT 10.0f  // config.h と同じ値にすること (このテストは単体でビルドする)

// 慣性と粘性は「この基板で実測した値」ではなく置いた仮定なので、1組の数字で
// 良し悪しを判定しない。**掃引して、どの組でも成り立つ性質だけを検証する。**
static float g_inertia = 3e-5f;    // [kg・m^2]
static float g_damping = 2.0e-4f;  // [N・m/(rad/s)]

static void Plant(Sim* s) {
  if (s->blocked) {
    s->angular_speed = 0.0f;
    return;
  }
  float torque = KT * s->target_iq - g_damping * s->angular_speed;
  s->angular_speed += torque / g_inertia * OUTER_LOOP_DT;
  s->mech_theta = FOC_NormalizeRadians(s->mech_theta + s->angular_speed * OUTER_LOOP_DT);
}

static void SimInit(Sim* s, float torque_limit_nm) {
  Sim z = {{7.5f, 15.0f, 0.05f, 0, 0, 0, 0.8f, 0}, 0, 0, 0, 0, 0, false};
  *s = z;
  s->iq_limit = Constrain(torque_limit_nm / KT, 0.0f, MAX_CURRENT);
}

static int failures = 0;
static void Check(int ok, const char* name) {
  printf("%s %s\n", ok ? "  ok  " : "  FAIL", name);
  if (!ok) failures++;
}

// ---------------------------------------------------------------------------
static void TestZeroLimitDoesNotMove(void) {
  printf("[A] 起動直後 (トルク上限 0) はモータが動かない\n");
  Sim s;
  SimInit(&s, 0.0f);
  s.target_position = 3.0f;

  float peak_iq = 0;
  for (int i = 0; i < 2000; i++) {
    OuterLoopPosition(&s);
    if (Abs(s.target_iq) > peak_iq) peak_iq = Abs(s.target_iq);
    Plant(&s);
  }
  printf("       2秒後: theta=%.4f rad, Iq指令のピーク=%.4f A\n", s.mech_theta, peak_iq);
  Check(peak_iq < 1e-6f, "Iq指令が一切出ない (0 を無制限と解釈していない)");
  Check(Abs(s.mech_theta) < 1e-6f, "軸が動かない");
}

// トルク上限が全域で効くこと。上限を振って、どの値でも Iq が超えないことを見る。
static void TestTorqueLimitHolds(void) {
  printf("[B] トルク上限で Iq が頭打ちになる (上限を振って確認)\n");
  const float limits[4] = {0.02f, 0.05f, 0.10f, 0.19f};

  for (int trial = 0; trial < 4; trial++) {
    Sim s;
    SimInit(&s, limits[trial]);
    s.target_position = 3.0f;

    float peak_iq = 0;
    for (int i = 0; i < 5000; i++) {
      OuterLoopPosition(&s);
      if (Abs(s.target_iq) > peak_iq) peak_iq = Abs(s.target_iq);
      Plant(&s);
    }
    printf("       torque_limit=%.2f N・m (= %.3f A) → Iq指令のピーク %.3f A\n", limits[trial],
           s.iq_limit, peak_iq);
    Check(peak_iq <= s.iq_limit + 1e-5f, "Iq指令が torque_limit を超えない");
    Check(Abs(FOC_AngleDiff(s.target_position, s.mech_theta)) < POSITION_DEADBAND_RAD,
          "目標位置には到達する");
  }
}

// **この改修で一番大事な検証。**
// 軸を固定したまま保持させたとき、流れる電流が torque_limit に比例すること。
//
// 旧実装 (位置指令のランプ + 引き戻し) では、拘束中の保持トルクを決めるのは
// torque_limit ではなく kp × POSITION_RAMP_LEAD_RAD = 7.5 × 0.2 = 1.5 A の定数だった。
// 速度制限を撤廃するとこれが常時成立してしまい、torque_limit が全く効かなくなる。
// ここが比例していれば、その退行が起きていないことの証明になる。
static void TestHoldingTorqueFollowsLimit(void) {
  printf("[C] 拘束中の保持トルクが torque_limit そのものになる\n");
  const float limits[4] = {0.02f, 0.05f, 0.10f, 0.19f};
  int proportional = 1;

  for (int trial = 0; trial < 4; trial++) {
    Sim s;
    SimInit(&s, limits[trial]);
    s.target_position = 3.0f;
    s.blocked = true;  // ラックエンドに当たったまま

    for (int i = 0; i < 3000; i++) {
      OuterLoopPosition(&s);
      Plant(&s);
    }
    float hold = Abs(s.target_iq);
    printf("       torque_limit=%.2f N・m (= %.3f A) → 保持電流 %.3f A%s\n", limits[trial],
           s.iq_limit, hold, (hold < 1.6f && s.iq_limit > 1.6f) ? "  ← 旧実装ならここで頭打ち" : "");
    if (hold < s.iq_limit - 1e-3f) proportional = 0;
  }
  Check(proportional, "保持電流が torque_limit まで出る (定数に化けていない)");
}

// 目標位置まで動かすとき、
//   (1) 途中で軸を固定する → 拘束が解けたあと
//   (2) 何も邪魔しない普通の移動
// の2つを比べる。**(1) の行き過ぎが (2) と同程度なら「飛び出していない」。**
// 位置ループそのものの行き過ぎと、ワインドアップによる飛び出しを混同しないため。
static float RunMove(float inertia, float damping, int block_ms, float* out_peak_speed,
                     float* out_peak_iq, float* out_peak_integral) {
  g_inertia = inertia;
  g_damping = damping;

  Sim s;
  SimInit(&s, 0.10f);  // 0.10 N・m (= 5.13 A)
  s.target_position = 3.0f;

  float peak_speed = 0, peak_iq = 0, peak_integral = 0, max_overshoot = 0;
  for (int i = 0; i < 8000; i++) {
    s.blocked = (i < block_ms);
    OuterLoopPosition(&s);
    if (Abs(s.target_iq) > peak_iq) peak_iq = Abs(s.target_iq);
    if (Abs(s.pid.integral) > peak_integral) peak_integral = Abs(s.pid.integral);
    Plant(&s);
    if (Abs(s.angular_speed) > peak_speed) peak_speed = Abs(s.angular_speed);
    float past = FOC_AngleDiff(s.mech_theta, s.target_position);
    if (past > max_overshoot) max_overshoot = past;
  }
  *out_peak_speed = peak_speed;
  *out_peak_iq = peak_iq;
  *out_peak_integral = peak_integral;
  return max_overshoot;
}

static void TestStallAntiWindup(void) {
  printf("[D] 据え切り (軸を固定) → 拘束解除。アンチワインドアップの確認\n");

  const float IQ_LIMIT = 0.10f / KT;  // 5.128 A

  float sp, iq, integ;
  float ov_free = RunMove(3e-5f, 2.0e-4f, 0, &sp, &iq, &integ);
  printf("       邪魔なし    : 行き過ぎ %.4f rad, 速度ピーク %.2f rad/s\n", ov_free, sp);

  float ov_block = RunMove(3e-5f, 2.0e-4f, 5000, &sp, &iq, &integ);
  printf("       5秒拘束→解除: 行き過ぎ %.4f rad, 速度ピーク %.2f rad/s\n", ov_block, sp);
  printf("       拘束中      : Iq指令ピーク %.3f A (上限 %.3f), 積分項ピーク %.3f\n", iq,
         IQ_LIMIT, integ);

  Check(iq <= IQ_LIMIT + 1e-5f, "拘束中の Iq が torque_limit を超えない");
  Check(integ <= IQ_LIMIT + 1e-5f, "積分項が torque_limit を超えて育たない");

  // **機械系を掃引して、どの慣性・粘性でも「拘束後の行き過ぎ ≒ 普通の移動の行き過ぎ」**
  // であることを確かめる。1組の数字で判断すると、たまたま良かっただけかもしれない。
  const float inertias[4] = {1e-5f, 3e-5f, 1e-4f, 3e-4f};
  const float dampings[3] = {5e-5f, 2e-4f, 1e-3f};
  float worst_ratio = 0.0f, worst_j = 0, worst_b = 0;
  int cases = 0;
  for (int a = 0; a < 4; a++) {
    for (int b = 0; b < 3; b++) {
      float f = RunMove(inertias[a], dampings[b], 0, &sp, &iq, &integ);
      float k = RunMove(inertias[a], dampings[b], 5000, &sp, &iq, &integ);
      cases++;
      // 行き過ぎがどちらもデッドバンド以下なら比は問わない (どちらも十分小さい)
      float ratio = (f > POSITION_DEADBAND_RAD)   ? (k / f)
                    : (k > POSITION_DEADBAND_RAD) ? 99.0f
                                                  : 1.0f;
      if (ratio > worst_ratio) {
        worst_ratio = ratio;
        worst_j = inertias[a];
        worst_b = dampings[b];
      }
      if (iq > IQ_LIMIT + 1e-5f) {
        printf("       J=%.0e B=%.0e で制限が破れた! Iq=%.3f\n", inertias[a], dampings[b], iq);
        Check(0, "掃引中に制限が破れた");
      }
    }
  }
  printf("       機械系 %d 通りを掃引: 「拘束後の行き過ぎ / 普通の移動の行き過ぎ」の最悪値 "
         "%.2f 倍 (J=%.0e B=%.0e)\n",
         cases, worst_ratio, worst_j, worst_b);
  Check(worst_ratio <= 1.5f,
        "どの機械系でも、拘束を解いた後の行き過ぎが普通の移動と同程度 (= 飛び出さない)");

  g_inertia = 3e-5f;
  g_damping = 2.0e-4f;
}

// 速度制限が無くなったので、移動速度は「上位が指定した値」ではなく
// **torque_limit と負荷と PIDゲインの合成**で決まるようになった。
// つまみが1つ減ったのではなく、意味の違うつまみに変わっている。
//
// 加速側は torque_limit が支配する (a = Kt·iq_limit / J)。
// 減速側は、出力が飽和から抜けたあと kp·error − kd·ω ≒ 0 に沿うので
// ω ≒ (kp/kd)·error の包絡線に乗る。**どちらが効くかは移動量による。**
// 下の実測値がその境目を示している (小さい移動ほど包絡線側で頭打ち)。
//
// **速すぎるときに戻すのは速度制限ではなく kd。** ここを上げると減速の包絡線が
// 寝るので、大きい移動ほど効く。torque_limit を下げると加速側だけが鈍る。
static void TestSpeedProfile(void) {
  printf("[E] 参考: 速度制限が無い今、移動速度は torque_limit と kp/kd の合成で決まる\n");

  const float targets[3] = {0.5f, 1.5f, 3.0f};
  g_inertia = 3e-5f;
  g_damping = 2.0e-4f;

  for (int trial = 0; trial < 3; trial++) {
    Sim s;
    SimInit(&s, 0.10f);
    s.target_position = targets[trial];

    float peak_speed = 0;
    for (int i = 0; i < 8000; i++) {
      OuterLoopPosition(&s);
      Plant(&s);
      if (Abs(s.angular_speed) > peak_speed) peak_speed = Abs(s.angular_speed);
    }
    // 2つの上界: トルク律速 (三角速度プロファイル) と kp/kd の包絡線
    float torque_bound = sqrtf(KT * s.iq_limit / g_inertia * targets[trial]);
    float gain_bound = (7.5f / 0.05f) * targets[trial];
    printf("       移動量 %.1f rad → 速度ピーク %6.2f rad/s "
           "(トルク律速の上界 %.1f / kp/kd 包絡線 %.1f)\n",
           targets[trial], peak_speed, torque_bound, gain_bound);
    Check(peak_speed <= torque_bound + 1.0f, "速度ピークがトルク律速の上界を超えない");
    Check(Abs(FOC_AngleDiff(s.target_position, s.mech_theta)) < POSITION_DEADBAND_RAD,
          "目標位置には到達する");
  }
}

int main(void) {
  TestZeroLimitDoesNotMove();
  TestTorqueLimitHolds();
  TestHoldingTorqueFollowsLimit();
  TestStallAntiWindup();
  TestSpeedProfile();
  printf("\n%s (失敗 %d 件)\n", failures ? "=== 失敗 ===" : "=== すべて成功 ===", failures);
  return failures != 0;
}
