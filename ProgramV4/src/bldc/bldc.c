#include "bldc_internal.h"

// このファイルは20kHz制御ループ本体 (PWM出力・角度追従オブザーバ・電流PI・
// SVPWM・保護・app向け公開API) だけを持つ。
// エンコーダ/モータ定数の校正・同定は bldc_calibration.c を参照。
// 共有する内部状態 (svc) とヘルパーは bldc_internal.h にある。

#if BLDC_MEASURING
// ステップ応答の記録バッファの実体。20kHz ISR (BLDC_CurrentLoop) が書き、
// bldc_calibration.c 側が読む。
int16_t bldc_capture[BLDC_CAPTURE_SAMPLES];
#endif

SensoredVectorControl svc;

// ---------------------------------------------------------------------------
// PWM出力の有効/無効
// ---------------------------------------------------------------------------
// 出力の有効/無効。無効側は「フリーラン(コースト)」= 上下FETともOFF。
//
// 全相デューティ0.5は線間電圧こそ0だが、3相の端子が常に同電位で振られるため
// モータの3相を短絡しているのと同じ状態になる。止まっていれば無害でも、回転中は
// 逆起電力が短絡電流 I ≈ E / |R + jωL| を流すので短絡制動になってしまう。
// 過電流保護が働いた直後にこれをやると、一番電流を流したくない場面で
// 一番電流が流れる状態に入ることになる。
//
// MOE=0 にすると全チャンネルの出力が止まる。BDTR の OSSI=1 と
// OISx/OISxN=0 (tim.c で TIM_OCIDLESTATE_RESET) の組み合わせにより、出力ピンは
// Hi-Z ではなくアイドルレベル(Low)に固定される。DRV8300 の INH/INL が
// どちらも Low になるので上下FETともOFF、つまりコーストになる。
static inline void BLDC_SetOutputEnable(bool on) {
  if (on) {
    __HAL_TIM_MOE_ENABLE(&htim1);
  } else {
    // __HAL_TIM_MOE_DISABLE は「全チャンネルが無効なとき」しか効かないマクロなので使わない
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }
}

// ---------------------------------------------------------------------------
// センサ処理
// ---------------------------------------------------------------------------
// エンコーダのレンジからラジアン変換係数を作り直す。max/min/盲点幅を変えたら必ず呼ぶこと。
//
// 素朴に 2π / (max - min) としてはいけない。AS5600のアナログ出力はレール付近で
// 飽和しており、ADC値 [min, max] が実際にカバーしているのは 2π ではなく
// 「2π - 盲点幅」だから。2π を割り当てると θ_meas が 2π/(2π-dead) 倍に引き伸ばされ、
//   - 観測できる区間では ω̂ がその比率ぶん高く出る
//   - 盲点を外挿で渡ると θ̂ が dead × その比率 だけ進み、観測再開時に段差になる
// という形で、1回転ごとの速度スパイクの原因になる。
// (実測: 盲点 0.108rad のとき比率 1.0173、段差 0.11rad ≒ emax の実測値 0.09rad)
void BLDC_UpdateEncoderScale(void) {
  float range = (float)(svc.max_encoder_val - svc.min_encoder_val);
  float span = TWO_PI_F - svc.encoder_dead_zone;  // ADCレンジが実際にカバーする機械角
  svc.encoder_scale = (range > 0.0f) ? (span / range) : 0.0f;
}

// ---------------------------------------------------------------------------
// 制御器
// ---------------------------------------------------------------------------
// 外側ループ (速度・位置) のPID。出力は Iq指令 [A]、上限は MAX_CURRENT。
//
// --- アンチワインドアップ (バックカリキュレーション) ---
// **積分項を ±output_limit で単独にクランプするだけでは足りない。**
// それだと出力が上限に張り付いている間も積分は MAX_CURRENT まで伸び続け、
// 誤差の符号が反転しても「積分を吐き出しきる」まで出力が上限に居座る。
// 行き過ぎてから戻り始めるので、飽和のたびに大きなオーバーシュートが出る。
//
// ここでやっているのは、飽和で捨てられたぶん (raw_output - output) を
// そのまま積分項から引き戻すこと。結果として積分項は
//   integral = output_limit - p_term - d_term
// つまり「P項とD項を出したあとに残っている余裕」ちょうどに落ち着く。
//   - 飽和していないとき: raw_output == output なので補正はきっかり0。
//     通常のPIDと完全に同じ動きをする (浮動小数の誤差も入らない)。
//   - 飽和しているとき  : 積分は使える範囲を超えて伸びない。誤差が減って
//     P項が下がれば、その場で積分が余裕を埋め直すので復帰に遅れが出ない。
//
// 電流ループ側 (FOC_LimitVoltageVector で scale を積分に掛け戻す) と考え方は
// 同じ。あちらはベクトルの大きさ制限なので比率、こちらはスカラなので差分。
//
// enable_integral が false のときは積分を凍結しているのでワインドアップ自体が
// 起きない。ここで引き戻すと、位置制御が目標近傍で積分を0に固定している意図
// (POSITION_INTEGRAL_STOP_RAD) を壊すので、補正もしない。
static inline float BLDC_PIDControl(PIDController* pid, float error, float dt, bool enable_integral) {
  // 比例項
  float p_term = pid->kp * error;

  // 積分項
  if (enable_integral) {
    pid->integral += pid->ki * error * dt;
  }

  // 微分項
  float raw_d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->d_term = pid->d_term * pid->d_lpf + raw_d_term * (1.0f - pid->d_lpf);
  pid->prev_error = error;

  // 出力の計算
  float raw_output = p_term + pid->integral + pid->d_term;
  float output = Constrain(raw_output, -pid->output_limit, pid->output_limit);

  // 飽和で捨てたぶんを積分項に返す
  if (enable_integral) {
    pid->integral += output - raw_output;
  }
  // 最後の砦。ゲインや output_limit を実行時に変えても積分が暴れないようにする。
  pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);

  return output;
}

// 外側ループ (1kHz)。各モードに応じてq軸電流指令を作る。
static void BLDC_OuterLoop(void) {
  // 上位から指定された制限値をローカルに取る。
  // **volatile を Constrain に直接渡してはいけない。** マクロが引数を3回展開するので
  // 3回別々にメモリを読みにいき、その間にメインループが書き換えると
  // low > high の状態でクランプが走る (下限が上限を上回り、値が下限に張り付く)。
  const float iq_limit = svc.iq_limit;

  // 外側ループの出力制限 = 上位が指定したトルク上限。
  // **BLDC_PIDControl のアンチワインドアップはこの output_limit を基準に働く**ので、
  // ここを毎周期入れ替えるだけで「飽和中は積分が伸びない」が自動的に成立する。
  // (据え切りでラックエンドに当たり続けても積分が育たない = 電流を流しっぱなしにしない)
  svc.speed_pid.output_limit = iq_limit;
  svc.position_pid.output_limit = iq_limit;

  switch (svc.mode) {
    case BLDC_MODE_SPEED: {
      // 速度指令は上位がそのまま決める。ここでかけるのは MD 固定のハード保護
      // (MAX_ANGULAR_SPEED) と、最大角加速度によるスルーレート制限だけ。
      float target =
          Constrain(svc.target_angular_speed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      float accel = Constrain((target - svc.speed_ramp) * (1.0f / OUTER_LOOP_DT),
                              -MAX_ANGULAR_ACCEL, MAX_ANGULAR_ACCEL);
      svc.speed_ramp += accel * OUTER_LOOP_DT;

      svc.target_iq = BLDC_PIDControl(&svc.speed_pid, svc.speed_ramp - svc.angular_speed,
                                      OUTER_LOOP_DT, true);
      break;
    }

    case BLDC_MODE_POSITION: {
      // 目標角との偏差をそのまま PID に入れる素の構成。
      // 内部の位置指令 (ランプ) は持たない — 経緯は bldc.h のコメントを参照。
      //
      // 偏差は FOC_AngleDiff が ±π に畳むので、拘束されても青天井には育たない。
      // ワインドアップは BLDC_PIDControl のバックカリキュレーションが抑える。
      float error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
      float abs_error = Abs(error);

      if (abs_error < POSITION_INTEGRAL_STOP_RAD) {
        svc.position_pid.integral = 0;
      }

      if (abs_error < POSITION_DEADBAND_RAD && Abs(svc.angular_speed) < POSITION_SETTLE_SPEED_RAD_S) {
        // 目標位置に整定した。無駄な電流を流さない。
        svc.position_pid.prev_error = error;
        svc.position_pid.d_term = 0;
        svc.target_iq = 0;
        break;
      }

      svc.target_iq = BLDC_PIDControl(&svc.position_pid, error, OUTER_LOOP_DT,
                                      abs_error >= POSITION_INTEGRAL_STOP_RAD);
      break;
    }

    case BLDC_MODE_TORQUE:
      svc.target_iq = svc.target_current;
      break;

    case BLDC_MODE_BRAKE:
      // 回転方向と逆向きの電流を流す。速度が落ちるほど電流も減らす。
      svc.target_iq = svc.brake_current * Constrain(svc.angular_speed * -0.05f, -1.0f, 1.0f);
      break;

    case BLDC_MODE_STOP:
    default:
      svc.target_iq = 0;
      break;
  }

  // 最終的なIq指令の飽和。PIDを通らないトルクモード・制動モードにも効かせるため、
  // モードによらずここで必ず通す。iq_limit は MAX_CURRENT でクランプ済み。
  svc.target_iq = Constrain(svc.target_iq, -iq_limit, iq_limit);
  // target_id は電流ループ側で決める (1kHzのここで決めると、ステップ応答測定の
  // ステップ位置が最大1msぶれてしまう)
}

// 過電流保護の発動。原因調査のため、発動した瞬間の状態を丸ごと記録して止める。
static void BLDC_Trip(float peak, float iu, float iv, float iw) {
  svc.trip.peak_current = peak;
  svc.trip.iu = iu;
  svc.trip.iv = iv;
  svc.trip.iw = iw;
  svc.trip.id = svc.id;
  svc.trip.iq = svc.iq;
  svc.trip.target_iq = svc.target_iq;
  svc.trip.vd = svc.vd;
  svc.trip.vq = svc.vq;
  svc.trip.mech_theta = svc.mech_theta;
  svc.trip.angular_speed = svc.angular_speed;
  CurrentSense_GetRaw(&svc.trip.raw_u, &svc.trip.raw_v);

  svc.is_overcurrent = true;
  svc.enable = false;
  svc.mode = BLDC_MODE_STOP;
}

// 出力を切ってフリーランさせ、制御器を再開できる状態に戻す。
// ここで全相デューティ0.5にすると3相短絡になり、回転中は制動電流が流れてしまう。
static void BLDC_Coast(void) {
  BLDC_SetOutputEnable(false);
  svc.vd = 0;
  svc.vq = 0;
  svc.id = 0;
  svc.iq = 0;
  svc.target_iq = 0;
  svc.speed_ramp = 0;
  // 位置PIDの微分項を、再開時に蹴らないよう「いまの偏差」で初期化しておく。
  // 0 のままにすると、再開1周期目に (error - 0)/dt がそのまま微分項に化けて
  // 出力が飛ぶ (mech_theta は BLDC_UpdateEncoder が enable に関係なく毎周期
  // 更新しているので、ここで読んで問題ない)。
  svc.position_pid.prev_error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
  svc.position_pid.d_term = 0;
  FOC_PI_Reset(&svc.id_pi);
  FOC_PI_Reset(&svc.iq_pi);
  svc.speed_pid.integral = 0;
  svc.position_pid.integral = 0;
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);  // 再開時に中性から始まるようCCRは戻しておく
}

// 電流ループ (20kHz)。ADCの変換完了ごとに呼ばれる。
static void BLDC_CurrentLoop(void) {
  // --- 相電流の取得 ---
  float iu, iv, iw;
  CurrentSense_Read(&iu, &iv, &iw);

  // --- 過電流保護 ---
  // 検出が遅れないよう、フィルタをかける前の生値で判定する
  float abs_iu = Abs(iu), abs_iv = Abs(iv), abs_iw = Abs(iw);
  float peak = abs_iu;
  if (abs_iv > peak) peak = abs_iv;
  if (abs_iw > peak) peak = abs_iw;

  if (peak > svc.peak_current) svc.peak_current = peak;  // ピーク保持(調査用)

  // 制御にはノイズを抑えたフィルタ後の値を使う。
  // W相はキルヒホッフ則で決まる従属変数で、Clarke変換も Iu/Iv しか使わないので持たない。
  svc.iu += CURRENT_LPF_COEF * (iu - svc.iu);
  svc.iv += CURRENT_LPF_COEF * (iv - svc.iv);

  // 1サンプルだけのノイズで誤検出しないよう、連続して超えたときだけ保護を発動する。
  // (この基板にはハードウェアの過電流保護が無いので、短絡のような急峻な故障は
  //  どのみちソフトでは間に合わない。ここで守るのは持続的な過電流。)
  if (peak > OVERCURRENT_LIMIT) {
    if (++svc.overcurrent_count >= OVERCURRENT_TRIP_COUNT && !svc.is_overcurrent) {
      BLDC_Trip(peak, iu, iv, iw);
    }
  } else {
    svc.overcurrent_count = 0;
  }

  if (!svc.enable) {
    BLDC_Coast();
    return;
  }

  // --- 電気角 ---
  // エンコーダ校正 (BLDC_SetEncoder) により、ロータのd軸は mech_theta * POLE_PAIRS に一致する。
  //
  // ここで 0〜2π に正規化しないこと。FOC_SinCos は内部で x/2π の小数部を取るので
  // 範囲を問わない。一方 FOC_NormalizeRadians は while ループなので、
  // mech_theta × POLE_PAIRS (最大 44rad) を渡すと平均3〜4回まわり、
  // 1周につき VCMP+VMRS を余分に消費するだけの無駄になる。
  // 表示用の正規化は BLDC_GetElecTheta 側で行う。
  //
  // **角度の遅れを補償する。** AS5600の内部フィルタ(約1ms)と演算遅れ(0.075ms)で
  // 推定角は実際のロータ位置より遅れている。一定の時間遅れは回転中に
  // 「速度に比例した角度のずれ」になるので、ω̂ ぶん先に進めれば打ち消せる。
  // 補償しないと 120 rad/s で電気角が 49° ずれ、逆起電力がd軸に漏れて
  // Vd が本来の40倍出る (実測)。詳細と合わせ方は config.h の ANGLE_DELAY_S。
  float mech_theta_now = svc.mech_theta + svc.angular_speed * svc.angle_delay;
  svc.elec_theta = mech_theta_now * POLE_PAIRS + ELEC_THETA_OFFSET;
  float sin_t, cos_t;
  FOC_SinCos(svc.elec_theta, &sin_t, &cos_t);

  // --- Clarke変換 → Park変換 ---
  float i_alpha, i_beta;
  FOC_Clarke(svc.iu, svc.iv, &i_alpha, &i_beta);
  FOC_Park(i_alpha, i_beta, sin_t, cos_t, &svc.id, &svc.iq);

  // d軸指令。表面磁石型(SPM)なので弱め界磁はせず、通常は0。
  // ステップ応答測定のときだけ target_id_override が非0になる。
  // この周期の中だけで使い切る値なので、構造体には持たない。
  float target_id = svc.target_id_override;

#if BLDC_MEASURING
  // ステップ応答の記録。**Idを記録してからステップを入れる**ので、
  // サンプル0はステップ直前の値 = 基準になる。
  // サンプルnの時刻は「ステップから n × 50µs 後」。
  if (svc.capture_state == BLDC_CAPTURE_ARMED) {
    svc.capture_state = BLDC_CAPTURE_RUNNING;
    svc.capture_index = 0;
  }
  if (svc.capture_state == BLDC_CAPTURE_RUNNING) {
    bldc_capture[svc.capture_index] = (int16_t)(svc.id * 1000.0f);  // [mA]
    if (++svc.capture_index >= BLDC_CAPTURE_SAMPLES) {
      svc.capture_state = BLDC_CAPTURE_DONE;
    } else if (svc.capture_index == 1) {
      // ここでステップを入れる
#if BLDC_NEED_RL
      if (svc.capture_is_voltage) {
        svc.vd_override = svc.capture_step;
      } else
#endif
      {
        target_id = svc.capture_step;               // この周期からステップを反映する
        svc.target_id_override = svc.capture_step;  // 次の周期以降も保持する
      }
    }
  }
#endif

  // --- 電流PI制御 ---
#if BLDC_NEED_RL
  if (svc.vd_override_active) {
    // L と R の同定中は PI を通さず Vd を直接与える (開ループ)。
    // 制御器が挟まらないので、応答はモデルの仮定が要らない素の1次系
    // (時定数 L/R) になる。閉ループのステップ応答から極を逆算するのと違い、
    // リンギングやゼロ点の影響を受けない。
    svc.vd = svc.vd_override;
    svc.vq = 0.0f;

    // 安全弁。R が想定より小さいと I = V/R が伸びるので、超えたら即座に切る。
    // 開ループなので電流を制限してくれるものが他に無い。
    if (Abs(svc.id) > MOTOR_RL_ABORT_AMPS) {
      svc.vd = 0.0f;
      svc.vd_override = 0.0f;
      svc.rl_aborted = true;
    }
  } else
#endif
  {
    svc.vd = FOC_PI_Update(&svc.id_pi, target_id - svc.id, CURRENT_LOOP_DT);
    svc.vq = FOC_PI_Update(&svc.iq_pi, svc.target_iq - svc.iq, CURRENT_LOOP_DT);

#if CURRENT_FF_ENABLE
    // --- 逆起電力とdq干渉のフィードフォワード ---
    // 電圧方程式のうち速度に比例する項を先回りして足す (config.h 参照)。
    //   Vd ← −ω_e·L·Iq            (dq干渉)
    //   Vq ← +ω_e·L·Id + ω_e·ψm   (dq干渉 + 逆起電力)
    //
    // PIから見るとこれらは「速度に比例して大きくなる外乱」で、積分が誤差を
    // 見てから追いかける形になっていた。先に足しておけばPIは残差だけを相手に
    // すればよく、高速域ほど追従が良くなる。
    //
    // ω̂ はPLLの出力をそのまま使う。帯域32Hzで平滑されているのでノイズは乗らない。
    float w_e = svc.angular_speed * POLE_PAIRS;
    svc.vq_ff = w_e * (svc.motor_l * svc.id + svc.motor_psi);

    // **FFに電圧の全予算を使わせない。** FF単独で変調限界を超えると、PIは
    // 「FFの出しすぎを打ち消す」ためだけに使われ、電流を制御する権限を失う。
    // ψm を過大に見積もったときに実際にそうなり、指令+0.50Aに対し Iq が -0.24A、
    // 相電流ピークが8Aに達した。逆起電力が本当に電圧を超える速度ではモータは
    // それ以上回れないのが物理で、FFを頭打ちにしてもその物理は変わらない。
    // 変わるのは「PIが常に一定の権限を持つ」ことだけ。
    float ff_limit = svc.supply_volt * (MAX_MODULATION_RATIO * CURRENT_FF_MAX_RATIO);
    svc.vq_ff = Constrain(svc.vq_ff, -ff_limit, ff_limit);

    svc.vd -= w_e * svc.motor_l * svc.iq;
    svc.vq += svc.vq_ff;
#endif
  }

  // 電圧ベクトルを変調限界の円内に収める。
  // 制限がかかったぶんだけ積分項も縮めてワインドアップを防ぐ。
  float scale = FOC_LimitVoltageVector(&svc.vd, &svc.vq, svc.supply_volt * MAX_MODULATION_RATIO);
  if (scale < 1.0f) {
    svc.id_pi.integral *= scale;
    svc.iq_pi.integral *= scale;
  }

  // --- 逆Park変換 → SVPWM ---
  float v_alpha, v_beta;
  FOC_InvPark(svc.vd, svc.vq, sin_t, cos_t, &v_alpha, &v_beta);

  float du, dv, dw;
  FOC_SVPWM(v_alpha, v_beta, svc.supply_volt, &du, &dv, &dw);
  BLDC_WritePwm(du, dv, dw);
  BLDC_SetOutputEnable(true);  // コーストから復帰する
}

// ---------------------------------------------------------------------------
// 実行時間プロファイル
// ---------------------------------------------------------------------------
#if PROFILE_ISR
// 割り込みハンドラ全体と呼び出し周期。stm32f3xx_it.c から更新される。
Profile bldc_prof_irq;
ProfilePeriod bldc_prof_period;

// このコールバックの中身だけ。bldc_prof_irq との差がHALのディスパッチ分。
static Profile prof_callback;
#endif  // PROFILE_ISR

// 内訳の計測 (PROFILE_ISR = 2 のときだけ)。
// 無効なら ((void)0) に展開されるので、呼び出し側に #if を撒かなくてよい。
#if PROFILE_ISR >= 2
static Profile prof_encoder;  // 角度追従オブザーバ
static Profile prof_current;  // 電流ループ (Clarke/Park/PI/SVPWM)
static Profile prof_outer;    // 外側ループ (1kHzのときだけカウントされる)
#define PROF2_BEGIN(p) Profile_Begin(&(p))
#define PROF2_END(p) Profile_End(&(p))
#else
#define PROF2_BEGIN(p) ((void)0)
#define PROF2_END(p) ((void)0)
#endif

// ADC1(PWM同期の電流計測)の変換完了割り込み。これが20kHzの制御ループ本体。
// ADC2も同じコールバックを共有するのでインスタンスの判定が必要。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) return;

#if PROFILE_ISR
  Profile_Begin(&prof_callback);
#endif

  static uint16_t outer_cnt = 0;

  // 機械角と角速度はオブザーバが20kHzで同時に更新する (分周しない)
  PROF2_BEGIN(prof_encoder);
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);
  PROF2_END(prof_encoder);

  if (++outer_cnt >= OUTER_LOOP_DIV) {
    outer_cnt = 0;
    PROF2_BEGIN(prof_outer);
    BLDC_OuterLoop();
    PROF2_END(prof_outer);
  }

  PROF2_BEGIN(prof_current);
  BLDC_CurrentLoop();
  PROF2_END(prof_current);

#if PROFILE_ISR
  Profile_End(&prof_callback);
#endif
}

#if PROFILE_ISR
// 実行時間の集計を1行で表示して、最大値と平均をリセットする。
// elapsed_s には前回の表示からの実経過時間を渡すこと。
//
// 見かた:
//   irq   : 割り込みハンドラ全体 (HALのディスパッチ込み) の平均/最大。
//   CPU   : 合計サイクル数 ÷ 実経過時間。想定周波数を使わないので嘘をつかない。
//   rate  : 実測した割り込み頻度。20.0kHz でなければ何かがおかしい。
//   cb    : このファイルのコールバック本体。irq - cb がHALのディスパッチ分で、
//           ここが大きいならベアメタルのハンドラに置き換える価値がある。
//   周期  : 50.00µs から外れていたらADCトリガか優先度設定を疑う。
//           max が 50µs を大きく超えていたら1回取りこぼしている。
//   max   : 外側ループが回る20回に1回が最悪ケースになるはず。
//           これが50µsに近づいたら、これ以上処理を足す余裕は無い。
// 集計を捨てて、ここから測り直す。
// 起動直後は BLDC_Init 中に溜まったぶんが最初の1回に混ざり、実測レートが
// 実際より高く出てしまう。前回の失敗 (レートの異常を見落とす) を繰り返さないよう、
// 表示用タイマを張るのと同時にこれを呼んで測定開始点を揃える。
void BLDC_ResetIsrProfile(void) {
  ProfileResult dummy;
  Profile_Snapshot(&bldc_prof_irq, &dummy, true);
  Profile_Snapshot(&prof_callback, &dummy, true);
  ProfilePeriod_Snapshot(&bldc_prof_period, &dummy, true);
#if PROFILE_ISR >= 2
  Profile_Snapshot(&prof_encoder, &dummy, true);
  Profile_Snapshot(&prof_current, &dummy, true);
  Profile_Snapshot(&prof_outer, &dummy, true);
#endif
}

void BLDC_PrintIsrProfile(float elapsed_s) {
  ProfileResult irq, cb, period;
  Profile_Snapshot(&bldc_prof_irq, &irq, true);
  Profile_Snapshot(&prof_callback, &cb, true);
  ProfilePeriod_Snapshot(&bldc_prof_period, &period, true);

  printf("[PROF] irq %5.2f/%5.2fus  CPU %4.1f%%  rate %5.2fkHz  cb %5.2fus  周期 %5.2f-%5.2fus\n",
         (double)Profile_CyclesToUs(irq.mean), (double)Profile_CyclesToUs((float)irq.max),
         (double)Profile_CpuPercent(irq.total, elapsed_s),
         (double)(Profile_RateHz(irq.count, elapsed_s) * 0.001f),
         (double)Profile_CyclesToUs(cb.mean),
         (double)Profile_CyclesToUs((float)period.min),
         (double)Profile_CyclesToUs((float)period.max));

  // センター揃えではデューティ書き込みの期限がある (config.h の ISR_DEADLINE_US)。
  // 超えると書き込みが谷を過ぎてから届き、次の「頂点」で反映されて低側ON区間が
  // 左右非対称になる。電流のサンプリング点がずれるので、症状は
  // 「なぜか電流値がおかしい」という切り分けの難しい形で出る。必ず気づけるようにする。
  float irq_max_us = Profile_CyclesToUs((float)irq.max);
  if (irq_max_us > ISR_DEADLINE_US) {
    printf(
        "       警告 割り込みが期限 %.1fus を超えた (最大 %.2fus)。"
        "デューティの反映が1周期ずれる恐れがある\n",
        (double)ISR_DEADLINE_US, (double)irq_max_us);
  }

#if PROFILE_ISR >= 2
  ProfileResult enc, cur, out;
  Profile_Snapshot(&prof_encoder, &enc, true);
  Profile_Snapshot(&prof_current, &cur, true);
  Profile_Snapshot(&prof_outer, &out, true);

  printf("       内訳: PLL %4.2fus  電流ループ %4.2f/%4.2fus  外側 %4.2fus (%4.2fkHz)\n",
         (double)Profile_CyclesToUs(enc.mean),
         (double)Profile_CyclesToUs(cur.mean), (double)Profile_CyclesToUs((float)cur.max),
         (double)Profile_CyclesToUs(out.mean),
         (double)(Profile_RateHz(out.count, elapsed_s) * 0.001f));
#endif
}
#endif  // PROFILE_ISR

// ---------------------------------------------------------------------------
// 初期化
// ---------------------------------------------------------------------------
// TIM1 をセンター揃えに切り替えて3相PWMを出し始める。
//
// エッジ揃えだと低側FETのON区間 [CCR, ARR] の**後ろ側しか使えず**、
// サンプリング点より手前のデューティしか出せなかった (MAX_DUTY = 0.80 →
// 変調率 0.346 で、SVPWMの理論上限 0.577 の6割しか母線電圧を使えていなかった)。
// センター揃えなら低側ON区間がカウンタ頂点を中心に左右へ分散するので、
// 同じ絶対時間の余裕で 0.88 まで引ける (変調率 0.439, +27%)。
// おまけに3相の立ち上がりが同時でなくなるのでリプルとEMIも下がる。
//
// tim.c は CubeMX の再生成で上書きされるので、BDTR の OSSI と同じくここで設定する。
// (current_sense.c が ADC1 を再初期化しているのと同じ考え方)
static void BLDC_InitPwmTimer(void) {
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_ARR;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    printf("TIM1 センター揃えへの再初期化に失敗\n");
  }

  // PWM出力を開始する (CH1 = U相, CH2 = V相, CH3 = W相)。
  // 以降のデューティ書き込みは CCR 直書き (BLDC_WritePwm) なので、
  // lib/pwm_out の PwmOut ハンドルは保持しない。ここで必要なのは
  // 「相補出力つきでPWMを走らせる」ことだけなので HAL を直接呼ぶ。
  const uint32_t channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
  for (uint8_t i = 0; i < 3; i++) {
    HAL_TIM_PWM_Start(&htim1, channels[i]);
    HAL_TIMEx_PWMN_Start(&htim1, channels[i]);
  }
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);

  // MOE=0 のときに出力ピンをHi-Zではなくアイドルレベル(Low)に固定する。
  // CubeMXの既定 (TIM_OSSI_DISABLE) のままだとピンが浮いてゲートドライバの入力が
  // 不定になり、コーストにならない。tim.c は再生成で上書きされるのでここで設定する。
  TIM1->BDTR |= TIM_BDTR_OSSI;
}

// 外側ループ (速度・位置) のゲイン。出力は「Iq指令 [A]」。
// output_limit は上位から来たトルク上限で毎周期入れ替わる (BLDC_OuterLoop)。
// 初期値は 0 = 出力できない。上位から制限値を受け取るまで動かないのが正しい。
static void BLDC_InitOuterGains(void) {
  svc.speed_pid.kp = 0.1f;
  svc.speed_pid.ki = 0.25f;
  svc.speed_pid.kd = 0;
  svc.speed_pid.d_term = 0;
  svc.speed_pid.d_lpf = 0.0f;
  svc.speed_pid.output_limit = 0.0f;

  svc.position_pid.kp = 15.0f;
  svc.position_pid.ki = 20.0f;
  svc.position_pid.kd = 0.05f;
  svc.position_pid.d_term = 0;
  svc.position_pid.d_lpf = 0.8f;
  svc.position_pid.output_limit = 0.0f;
}

#if PROFILE_ISR
// サイクルカウンタを動かして、計測自身のコストを実測して表示する。
// この値を知らないと、出てきた数字のうちどこまでが本当の処理時間か判断できない。
// (lib/timer の Timer_Init も DWT を有効にするが、あちらが呼ばれるのは
//  BLDC_Init より後なので、制御ループが回り出す前にここで自前で有効化する)
static void BLDC_InitProfiler(void) {
  Profile_EnableDWT();
  if (!Profile_IsDWTRunning()) {
    printf("BLDC: 警告 DWT->CYCCNT が動いていません。プロファイル値は無効です。\n");
    return;
  }
  uint32_t ov = Profile_MeasureOverhead();
  printf("BLDC: プロファイル有効 (PROFILE_ISR=%d) 計測オーバーヘッド %lu cycle/区間 (%.3fus)\n",
         PROFILE_ISR, (unsigned long)ov, (double)Profile_CyclesToUs((float)ov));
}
#endif

void BLDC_Init(bool do_set_encoder, volatile uint16_t* encoder_val, float supply_volt) {
  printf("BLDC_Init\n");

  svc.encoder_val_ptr = encoder_val;
  svc.mode = BLDC_MODE_STOP;
  svc.enable = false;
  // **母線電圧は最初に確定させること。**
  // SVPWM は「電圧指令 → デューティ」の変換にこの値を使うので、実際の母線と
  // 食い違っていると、指令した電圧と実際に加わる電圧の比がそのままずれる。
  //
  // 以前は 12.0V の暫定値のまま校正まで走っていた。実測が 8.6V だったので、
  // 実際に加わる電圧は指令の 8.6/12 = 0.717 倍。電流ループは閉じているので
  // PIが 1/0.717 = 1.395 倍の電圧を指令して埋め合わせ、その水増しされた Vq から
  // ψm を計算していた (実測 0.002663 に対し真値 0.001908)。
  //
  // R と L は同じ倍率で狂っても、ゲイン (Kp=L·ω, Ki=R·ω) の水増しと印加電圧の
  // 目減りが打ち消し合うので実害が出ず、**ψm だけが表に出た**。
  // 測定時と使用時で母線の扱いが違う量は、この手の誤差が打ち消されない。
  BLDC_SetSupplyVolt(supply_volt);

  BLDC_InitPwmTimer();

  // 電流センシング(TIM1同期ADC)を開始。TIM1が動いてから呼ぶ必要がある。
  CurrentSense_Init();

  // --- モータ定数の初期値 ---
  // フラッシュに有効な値があれば下で上書きされる。校正するときも、R,L を測るまでは
  // この値で電流ループを回す必要があるので、先に入れておく。
  svc.motor_r = MOTOR_R_DEFAULT;
  svc.motor_l = MOTOR_L_DEFAULT;
  svc.motor_psi = MOTOR_PSI_DEFAULT;
  svc.angle_delay = ANGLE_DELAY_DEFAULT;

  // 校正値をどこから取るかは do_set_encoder で決まる。
  //   スイッチ押下あり: いま測る。フラッシュの古い値で上書きしないこと。
  //   スイッチ押下なし: フラッシュから読む (検証に通ったものだけ採用)。
  // フラッシュ書き込みは、この後のモータ定数測定まで終わってから1回でまとめて行う。
  if (do_set_encoder) {
    printf("BLDC_SetEncoder\n");
    BLDC_SetEncoder(encoder_val);
    printf("(Measured) このセッションの校正値を使う\n");
  } else {
    BLDC_LoadFlashCalibration();
  }
  BLDC_UpdateEncoderScale();

  // トルク指令[N・m]の目安をここで出す。ψmは校正ごとに変わるので、
  // 「Kt = 1.5×P×ψm」も「MAX_CURRENTで出せる最大トルク」もその都度違う。
  printf("トルク定数 Kt: %.5f N・m/A, 最大トルク(MAX_CURRENT時): %.4f N・m\n",
         (double)BLDC_GetTorqueConstant(), (double)(BLDC_GetTorqueConstant() * MAX_CURRENT));

  // --- 制御器のゲイン ---
  BLDC_InitOuterGains();      // 外側ループの出力は「Iq指令 [A]」
  BLDC_UpdateCurrentGains();  // 電流PIの出力は電圧 [V]。ゲインは L と R から導く

  // 電流センサのゼロ点校正。全相デューティ0.5で電流が流れない状態にして行う。
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);
  HAL_Delay(200);
  CurrentSense_Calibrate();

  // 制御ループを回す前に、機械角をいまのエンコーダ実測値で確定させておく。
  // (フィルタで0から実角度まで移動する間、電気角が高速に回ってしまうのを防ぐ)
  svc.encoder_primed = false;
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);
  printf("BLDC: mech_theta = %.3f rad で開始\n", svc.mech_theta);

  // 指令が来るまでは出力を切っておく (enable = false と状態を合わせる)。
  // 電流センサのゼロ点校正まではPWMを出しておく必要があるので、ここまで来てから切る。
  BLDC_SetOutputEnable(false);

#if PROFILE_ISR
  BLDC_InitProfiler();  // 制御ループが回り出す前にサイクルカウンタを動かしておく
#endif

  // ここまで来たら20kHzの制御ループ(ADC変換完了割り込み)を回し始める
  CurrentSense_EnableInterrupt();
  printf("BLDC control loop start (%.0f Hz)\n", (double)PWM_FREQ);

  // 測定は制御ループが回っていないとできないので、必ずここから下で行う。
#if MOTOR_AUTO_CALIBRATION
  // スイッチを押しながら起動したときだけ、モータ定数も測ってまとめて保存する。
  if (do_set_encoder) {
    BLDC_CalibrateMotor();
    BLDC_SaveCalibration();
  }
#endif
  BLDC_RunStartupMeasurements();

  // --- 制限値を閉じた状態で運転開始 ---
  // 校正 (BLDC_MeasureSteadyPoint) が一時的に開けているので、必ずここで戻す。
  // これ以降、上位から指令フレームで制限値を受け取るまでモータは動かない。
  BLDC_SetLimits(0.0f);
  printf(
      "BLDC: トルク上限 0 で開始 (上位からの指令待ち)。MD側の上限は %.3f N・m"
      " / 速度は %.0f rad/s で頭打ち\n",
      (double)BLDC_GetMaxTorqueNm(), (double)MAX_ANGULAR_SPEED);
}

// ---------------------------------------------------------------------------
// app から呼ぶAPI
// ---------------------------------------------------------------------------
void BLDC_SetSupplyVolt(float supply_volt) {
  if (supply_volt < 1.0f) supply_volt = 1.0f;  // 0除算防止
  svc.supply_volt = supply_volt;

  float v_limit = supply_volt * MAX_MODULATION_RATIO;
  svc.id_pi.output_limit = v_limit;
  svc.iq_pi.output_limit = v_limit;
}

// 表面磁石型PMSMのトルク式 T = 1.5×P×ψm×Iq より、トルク定数 Kt [N・m/A]。
// ψm は校正で決まるので実行時に計算する。
float BLDC_GetTorqueConstant(void) { return 1.5f * POLE_PAIRS * svc.motor_psi; }

// MD 側の絶対上限。上位が何を送ってきてもこれを超えたトルクは出さない。
float BLDC_GetMaxTorqueNm(void) { return BLDC_GetTorqueConstant() * MAX_CURRENT; }

// 上位から指定された制限値を適用する。詳細は bldc.h を参照。
//
// **0 を「制限なし」と解釈しない。** 0 は文字どおり上限0で、モータは動かない。
// 通信が確立していない状態で全力回転するのが最悪の失敗なので、
// 制限値を受け取れていない = 何もできない、が正しいフェイルセーフになる。
void BLDC_SetLimits(float torque_limit_nm) {
  // トルク [N・m] → Iq指令 [A]。Kt は校正した ψm から決まるので実行時に計算する。
  // ψm が未校正で 0 だと 0除算になるので、その場合は 0 (= 動かない) に倒す。
  float kt = BLDC_GetTorqueConstant();
  float iq_limit = (kt > 1e-6f) ? (torque_limit_nm / kt) : 0.0f;
  svc.iq_limit = Constrain(iq_limit, 0.0f, MAX_CURRENT);
}

float BLDC_GetTorqueLimitNm(void) { return svc.iq_limit * BLDC_GetTorqueConstant(); }

void BLDC_Stop(void) {
  svc.mode = BLDC_MODE_STOP;
  svc.enable = false;
}

// 過電流保護がラッチされている間は起動させない
static inline void BLDC_Enable(BLDCMode mode) {
  if (svc.is_overcurrent) return;
  // モードが切り替わった瞬間に位置PIDの微分項を初期化する。
  // 別のモードで回っている間 prev_error は更新されないので、そのまま位置制御へ
  // 入ると1周期目に「その間に動いたぶん」がまとめて微分項に化けて出力が飛ぶ。
  if (svc.mode != mode) {
    svc.position_pid.prev_error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
    svc.position_pid.d_term = 0;
  }
  svc.mode = mode;
  svc.enable = true;
}

void BLDC_AngularSpeedControl(float target_angular_speed) {
  svc.target_angular_speed = target_angular_speed;
  BLDC_Enable(BLDC_MODE_SPEED);
}

void BLDC_PositionControl(float target_position) {
  svc.target_position = target_position;
  BLDC_Enable(BLDC_MODE_POSITION);
}

void BLDC_TorqueControl(float target_current) {
  svc.target_current = Constrain(target_current, -MAX_CURRENT, MAX_CURRENT);
  BLDC_Enable(BLDC_MODE_TORQUE);
}

// トルク指令 [N・m] を Kt で割って Iq指令 [A] に直す。
void BLDC_TorqueControlNm(float torque_nm) {
  BLDC_TorqueControl(torque_nm / BLDC_GetTorqueConstant());
}

void BLDC_BrakeControl(float brake_current) {
  svc.brake_current = Constrain(brake_current, 0.0f, MAX_CURRENT);
  BLDC_Enable(BLDC_MODE_BRAKE);
}

void BLDC_BrakeControlNm(float brake_torque_nm) {
  BLDC_BrakeControl(brake_torque_nm / BLDC_GetTorqueConstant());
}

float BLDC_GetMechTheta(void) { return svc.mech_theta; }

// 割り込みの中では電気角を正規化していない (FOC_SinCos が範囲を問わないため)。
// 表示・シリアル送信用にここで 0〜2π に畳む。
float BLDC_GetElecTheta(void) { return FOC_NormalizeRadians(svc.elec_theta); }
float BLDC_GetAngularSpeed(void) { return svc.angular_speed; }
float BLDC_GetId(void) { return svc.id; }
float BLDC_GetIq(void) { return svc.iq; }
float BLDC_GetTargetIq(void) { return svc.target_iq; }
float BLDC_GetVd(void) { return svc.vd; }
float BLDC_GetVq(void) { return svc.vq; }

// Vq のうちフィードフォワードで作った分。
// FFが正しく効いていれば、速度が上がるほど Vq に占めるこの割合が大きくなり、
// PIは残差だけを相手にする。**符号が逆だと Vq と逆向きに出る**ので、
// 実装ミスの確認にはこれを見るのが一番早い。
float BLDC_GetVqFF(void) { return svc.vq_ff; }
bool BLDC_IsOvercurrent(void) { return svc.is_overcurrent; }
void BLDC_GetTripInfo(BLDCTripInfo* info) { *info = svc.trip; }

void BLDC_ClearOvercurrent(void) {
  svc.overcurrent_count = 0;
  svc.is_overcurrent = false;
}

float BLDC_GetPeakCurrent(void) { return svc.peak_current; }
void BLDC_ResetPeakCurrent(void) { svc.peak_current = 0.0f; }

void BLDC_GetEncoderStats(BLDCEncoderStats* stats) {
  stats->saturated = svc.saturated_total;
  stats->glitch = svc.glitch_total;
  stats->max_innovation = svc.max_innovation;
}

void BLDC_ResetEncoderStats(void) {
  svc.saturated_total = 0;
  svc.glitch_total = 0;
  svc.max_innovation = 0.0f;
}
