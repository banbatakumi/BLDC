#include <math.h>  // tanhf (ブレーキの境界層)

#include "bldc_internal.h"

// 20kHz制御ループ本体 (PWM出力・電流PI・SVPWM・保護・公開API)。
// 校正・同定は bldc_calibration.c、共有状態 (svc) は bldc_internal.h。

#if BLDC_MEASURING
// ステップ応答の記録バッファ (BLDC_CurrentLoop が書き、bldc_calibration.c が読む)
int16_t bldc_capture[BLDC_CAPTURE_SAMPLES];
#endif

SensoredVectorControl svc;

// PWM出力の有効/無効。無効側は上下FETともOFFのコースト。
// 全相デューティ0.5は3相短絡と同じで、回転中は逆起電力による短絡制動電流が流れるため使わない。
// MOE=0 では BDTR の OSSI=1 と OISx/OISxN=0 (tim.c) により出力ピンがアイドルレベル(Low)に固定され、
// DRV8300 の INH/INL が両方Lowになってコーストになる。
static inline void BLDC_SetOutputEnable(bool on) {
  if (on) {
    __HAL_TIM_MOE_ENABLE(&htim1);
  } else {
    // __HAL_TIM_MOE_DISABLE は全チャンネル無効のときしか効かないので使わない
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }
}

// エンコーダのレンジからラジアン変換係数を作り直す。max/min/盲点幅を変えたら必ず呼ぶこと。
// AS5600はレール付近で飽和するため、ADC値 [min, max] がカバーするのは 2π ではなく
// 2π - 盲点幅。2π を割り当てると θ_meas が引き伸ばされ、ω̂ が比率ぶん高く出たり、
// 盲点を外挿で渡った後に段差が出て1回転ごとの速度スパイクになる。
void BLDC_UpdateEncoderScale(void) {
  float range = (float)(svc.max_encoder_val - svc.min_encoder_val);
  float span = TWO_PI_F - svc.encoder_dead_zone;  // ADCレンジが実際にカバーする機械角
  svc.encoder_scale = (range > 0.0f) ? (span / range) : 0.0f;
}

// 外側ループ (速度・位置) のPID。出力は Iq指令 [A]。
// アンチワインドアップはバックカリキュレーション: 飽和で捨てたぶん (raw_output - output) を
// 積分項から引き戻し、積分が使える範囲を超えて伸びないようにする (クランプだけだと
// 飽和のたびに大きなオーバーシュートが出る)。飽和していなければ補正は0で通常のPIDと同じ。
// enable_integral が false のときは積分を凍結しており、引き戻すと位置制御が目標近傍で
// 積分を0に固定する意図 (POSITION_INTEGRAL_STOP_RAD) を壊すので補正しない。
static inline float BLDC_PIDControl(PIDController* pid, float error, float dt, bool enable_integral) {
  float p_term = pid->kp * error;

  if (enable_integral) {
    pid->integral += pid->ki * error * dt;
  }

  float raw_d_term = pid->kd * (error - pid->prev_error) / dt;
  pid->d_term = pid->d_term * pid->d_lpf + raw_d_term * (1.0f - pid->d_lpf);
  pid->prev_error = error;

  float raw_output = p_term + pid->integral + pid->d_term;
  float output = Constrain(raw_output, -pid->output_limit, pid->output_limit);

  if (enable_integral) {
    pid->integral += output - raw_output;
  }
  // ゲインや output_limit を実行時に変えても積分が暴れないようにする
  pid->integral = Constrain(pid->integral, -pid->output_limit, pid->output_limit);

  return output;
}

// 制動電流に掛ける係数 (-1〜1)。回転と逆向きに流す。
// sign(ω) だと v=0 付近でチャタリングするので tanh の境界層 (BRAKE_BOUNDARY_SPEED_RAD_S) で
// 滑らかにし、さらに不感帯未満 (BRAKE_DEADBAND_SPEED_RAD_S) は 0 にしてディザを防ぐ。
static float BLDC_BrakeDirection(float angular_speed) {
  if (Abs(angular_speed) < BRAKE_DEADBAND_SPEED_RAD_S) return 0.0f;
  return -tanhf(angular_speed / BRAKE_BOUNDARY_SPEED_RAD_S);
}

// PIDを通らないモード (トルク・制動・停止) のIq指令を作り、上限で飽和させる。
// 指令を素通しするだけなので、反映が1ms遅れないよう割り込みから20kHzで毎周期呼ぶ。
static void BLDC_DirectIqLoop(float iq_limit) {
  float target_iq;
  switch (svc.mode) {
    case BLDC_MODE_TORQUE:
      target_iq = svc.target_current;
      break;
    case BLDC_MODE_BRAKE:
      target_iq = svc.brake_current * svc.brake_direction;
      break;
    case BLDC_MODE_STOP:
    default:
      target_iq = 0.0f;
      break;
  }
  svc.target_iq = Constrain(target_iq, -iq_limit, iq_limit);
}

static bool BLDC_IsDirectIqMode(BLDCMode mode) {
  return mode != BLDC_MODE_SPEED && mode != BLDC_MODE_POSITION;
}

// 外側ループ (1kHz)。各モードに応じてq軸電流指令を作る。
static void BLDC_OuterLoop(void) {
  // volatile を Constrain に直接渡さないこと (マクロが引数を3回展開し、途中で書き換わると low > high になる)
  const float iq_limit = svc.iq_limit;

  // 出力制限 = 上位指定のトルク上限。アンチワインドアップがこれを基準に働くので、
  // 毎周期入れ替えるだけで飽和中に積分が伸びなくなる。
  svc.speed_pid.output_limit = iq_limit;
  svc.position_pid.output_limit = iq_limit;

  // tanhf を20kHzで回さないよう制動係数はここ (1kHz) で更新する。モードによらず更新しておく。
  svc.brake_direction = BLDC_BrakeDirection(svc.angular_speed);

  switch (svc.mode) {
    case BLDC_MODE_SPEED: {
      // MD 固定のハード保護 (MAX_ANGULAR_SPEED) と最大角加速度のスルーレート制限だけをかける
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
      // 偏差は FOC_AngleDiff が ±π に畳むので、拘束されても青天井には育たない
      float error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
      float abs_error = Abs(error);

      if (abs_error < POSITION_INTEGRAL_STOP_RAD) {
        svc.position_pid.integral = 0;
      }

      if (abs_error < POSITION_DEADBAND_RAD && Abs(svc.angular_speed) < POSITION_SETTLE_SPEED_RAD_S) {
        // 整定した。無駄な電流を流さない
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
    case BLDC_MODE_BRAKE:
    case BLDC_MODE_STOP:
    default:
      BLDC_DirectIqLoop(iq_limit);
      return;
  }

  svc.target_iq = Constrain(svc.target_iq, -iq_limit, iq_limit);
}

// 過電流保護の発動。原因調査のため、発動時の状態を記録して止める。
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

// 出力を切ってコーストさせ、制御器を再開できる状態に戻す。
static void BLDC_Coast(void) {
  BLDC_SetOutputEnable(false);
  svc.vd = 0;
  svc.vq = 0;
  svc.id = 0;
  svc.iq = 0;
  svc.target_iq = 0;
  // 速度指令のランプは現在の角速度から始める (0 から始めると、回っている状態で速度モードに
  // 入った瞬間に「目標0」との偏差で逆向きのトルクが出る)
  svc.speed_ramp = svc.angular_speed;
  // 再開1周期目に微分項が飛ばないよう、位置PIDの prev_error をいまの偏差で初期化する
  svc.position_pid.prev_error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
  svc.position_pid.d_term = 0;
  FOC_PI_Reset(&svc.id_pi);
  FOC_PI_Reset(&svc.iq_pi);
  svc.speed_pid.integral = 0;
  svc.position_pid.integral = 0;
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);  // 再開時に中性から始まるようCCRを戻す
}

// 電流ループ (20kHz)。ADCの変換完了ごとに呼ばれる。
static void BLDC_CurrentLoop(void) {
  float iu, iv, iw;
  CurrentSense_Read(&iu, &iv, &iw);

  // 過電流判定は検出が遅れないようフィルタ前の生値で行う
  float abs_iu = Abs(iu), abs_iv = Abs(iv), abs_iw = Abs(iw);
  float peak = abs_iu;
  if (abs_iv > peak) peak = abs_iv;
  if (abs_iw > peak) peak = abs_iw;

  if (peak > svc.peak_current) svc.peak_current = peak;  // ピーク保持(調査用)

  // 制御にはフィルタ後の値を使う (W相は Iw = -(Iu+Iv) なので持たない)
  svc.iu += CURRENT_LPF_COEF * (iu - svc.iu);
  svc.iv += CURRENT_LPF_COEF * (iv - svc.iv);

  // 連続して超えたときだけ発動する。この基板にはハードの過電流保護が無く、
  // 短絡のような急峻な故障はソフトでは間に合わない。ここで守るのは持続的な過電流。
  if (peak > OVERCURRENT_LIMIT) {
    if (++svc.overcurrent_count >= OVERCURRENT_TRIP_COUNT && !svc.is_overcurrent) {
      BLDC_Trip(peak, iu, iv, iw);
    }
  } else {
    svc.overcurrent_count = 0;
  }

  // 過電流ラッチ中は enable に関わらず必ず出力を切る。BLDC_Enable (メインループ) が
  // is_overcurrent を確認してから enable = true を書くまでの間にここのトリップが割り込むと
  // enable が真に戻るが、トリップは一度しか発動しないので、enable だけを見ていると
  // ラッチ表示のまま駆動が続く
  if (!svc.enable || svc.is_overcurrent) {
    BLDC_Coast();
    return;
  }

  // 電気角。エンコーダ校正によりd軸は mech_theta × POLE_PAIRS に一致する。
  // FOC_SinCos は範囲を問わないので正規化しない (正規化は表示用の BLDC_GetElecTheta で行う)。
  // 角度の遅れ (AS5600のフィルタ等) を ω̂ ぶん進めて補償する。しないと 120 rad/s で
  // 電気角が 49° ずれる (実測)。詳細は config.h の ANGLE_DELAY_DEFAULT。
  float mech_theta_now = svc.mech_theta + svc.angular_speed * svc.angle_delay;
  svc.elec_theta = mech_theta_now * POLE_PAIRS + ELEC_THETA_OFFSET;
  float sin_t, cos_t;
  FOC_SinCos(svc.elec_theta, &sin_t, &cos_t);

  float i_alpha, i_beta;
  FOC_Clarke(svc.iu, svc.iv, &i_alpha, &i_beta);
  FOC_Park(i_alpha, i_beta, sin_t, cos_t, &svc.id, &svc.iq);

  // d軸指令。SPMなので弱め界磁はせず通常は0。ステップ応答測定のときだけ target_id_override が非0。
  float target_id = svc.target_id_override;

#if BLDC_MEASURING
  // ステップ応答の記録。Idを記録してからステップを入れるので、サンプルnは「ステップから n × 50µs 後」
  // (サンプル0はステップ直前の基準値)。
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
        target_id = svc.capture_step;               // この周期から反映
        svc.target_id_override = svc.capture_step;  // 次周期以降も保持
      }
    }
  }
#endif

#if BLDC_NEED_RL
  if (svc.vd_override_active) {
    // L・R同定中は PI を通さず Vd を直接与える (応答が素の1次系になる)
    svc.vd = svc.vd_override;
    svc.vq = 0.0f;

    // 開ループで電流を制限するものが無いので、超えたら即座に切る
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
    // 逆起電力とdq干渉のフィードフォワード (config.h 参照)
    //   Vd ← −ω_e·L·Iq,  Vq ← +ω_e·L·Id + ω_e·ψm
    float w_e = svc.angular_speed * POLE_PAIRS;
    svc.vq_ff = w_e * (svc.motor_l * svc.id + svc.motor_psi);

    // FFが電圧の全予算を使うとPIが制御権限を失うので頭打ちにする (config.h の CURRENT_FF_MAX_RATIO)
    float ff_limit = svc.supply_volt * (MAX_MODULATION_RATIO * CURRENT_FF_MAX_RATIO);
    svc.vq_ff = Constrain(svc.vq_ff, -ff_limit, ff_limit);

    svc.vd -= w_e * svc.motor_l * svc.iq;
    svc.vq += svc.vq_ff;
#endif
  }

  // 電圧ベクトルを変調限界の円内に収め、制限したぶん積分項も縮める (ワインドアップ防止)
  float scale = FOC_LimitVoltageVector(&svc.vd, &svc.vq, svc.supply_volt * MAX_MODULATION_RATIO);
  if (scale < 1.0f) {
    svc.id_pi.integral *= scale;
    svc.iq_pi.integral *= scale;
  }

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
// 割り込みハンドラ全体と呼び出し周期 (stm32f3xx_it.c から更新される)
Profile bldc_prof_irq;
ProfilePeriod bldc_prof_period;

// コールバック本体だけ。bldc_prof_irq との差がHALのディスパッチ分。
static Profile prof_callback;
#endif  // PROFILE_ISR

// 内訳の計測 (PROFILE_ISR = 2 のみ)。無効時は ((void)0) になる。
#if PROFILE_ISR >= 2
static Profile prof_encoder;  // 角度追従オブザーバ
static Profile prof_current;  // 電流ループ (Clarke/Park/PI/SVPWM)
static Profile prof_outer;    // 外側ループ (1kHzのときだけカウント)
#define PROF2_BEGIN(p) Profile_Begin(&(p))
#define PROF2_END(p) Profile_End(&(p))
#else
#define PROF2_BEGIN(p) ((void)0)
#define PROF2_END(p) ((void)0)
#endif

// ADC1(PWM同期の電流計測)の変換完了割り込み = 20kHz制御ループ本体。ADC2とコールバックを共有する。
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance != ADC1) return;

#if PROFILE_ISR
  Profile_Begin(&prof_callback);
#endif

  static uint16_t outer_cnt = 0;

  // 機械角と角速度はオブザーバが20kHzで更新する
  PROF2_BEGIN(prof_encoder);
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);
  PROF2_END(prof_encoder);

  if (++outer_cnt >= OUTER_LOOP_DIV) {
    outer_cnt = 0;
    PROF2_BEGIN(prof_outer);
    BLDC_OuterLoop();
    PROF2_END(prof_outer);
  } else if (BLDC_IsDirectIqMode(svc.mode)) {
    // トルク・制動・停止は分周せず毎周期反映する
    BLDC_DirectIqLoop(svc.iq_limit);
  }

  PROF2_BEGIN(prof_current);
  BLDC_CurrentLoop();
  PROF2_END(prof_current);

#if PROFILE_ISR
  Profile_End(&prof_callback);
#endif
}

#if PROFILE_ISR
// 集計を捨てて測り直す (起動直後に BLDC_Init 中の分が混ざってレートが高く出るのを防ぐ)
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

// 集計を1行で表示して最大値と平均をリセットする。elapsed_s は前回表示からの実経過時間。
//   irq  : 割り込み全体 (HALのディスパッチ込み) の平均/最大
//   CPU  : 合計サイクル ÷ 実経過時間
//   rate : 実測の割り込み頻度。20.0kHz でなければ異常
//   cb   : コールバック本体。irq - cb がHALのディスパッチ分
//   周期 : 50µs から外れていたらADCトリガか優先度を疑う
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

  // デューティ書き込みの期限 (config.h の ISR_DEADLINE_US)。超えると電流サンプリング点がずれる
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
// センター揃えにするとMAX_DUTYを 0.80 → 0.88 に上げられる (docs/OPTIMIZATION.md)。
// tim.c は CubeMX の再生成で上書きされるので、センター揃えと BDTR の OSSI はここで設定する。
static void BLDC_InitPwmTimer(void) {
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_ARR;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    printf("TIM1 センター揃えへの再初期化に失敗\n");
  }

  // 相補出力つきでPWM開始 (CH1 = U相, CH2 = V相, CH3 = W相)。以降は CCR 直書き。
  const uint32_t channels[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
  for (uint8_t i = 0; i < 3; i++) {
    HAL_TIM_PWM_Start(&htim1, channels[i]);
    HAL_TIMEx_PWMN_Start(&htim1, channels[i]);
  }
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);

  // MOE=0 のとき出力ピンをHi-Zでなくアイドルレベル(Low)に固定する (CubeMX既定だと入力が不定になる)
  TIM1->BDTR |= TIM_BDTR_OSSI;
}

// 外側ループ (速度・位置) のゲイン。出力は Iq指令 [A]。
// output_limit は BLDC_OuterLoop が毎周期入れ替える。初期値 0 = 制限値を受け取るまで動かない。
static void BLDC_InitOuterGains(void) {
  svc.speed_pid.kp = 0.1f;
  svc.speed_pid.ki = 0.25f;
  svc.speed_pid.kd = 0;
  svc.speed_pid.d_term = 0;
  svc.speed_pid.d_lpf = 0.0f;
  svc.speed_pid.output_limit = 0.0f;

  svc.position_pid.kp = 15.0f;
  svc.position_pid.ki = 25.0f;
  svc.position_pid.kd = 0.05f;
  svc.position_pid.d_term = 0;
  svc.position_pid.d_lpf = 0.8f;
  svc.position_pid.output_limit = 0.0f;
}

#if PROFILE_ISR
// サイクルカウンタを動かし、計測自身のコストを表示する (Timer_Init は BLDC_Init より後なのでここで有効化)
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

// 電流センサのゼロ点校正。全相デューティ0.5 は3相短絡なので、ロータが回っていると
// (惰行中に駆動電源を入れ直したときなど) 逆起電力による電流が流れ、その平均がゼロ点に混ざる。
// 校正の前後でエンコーダの生値が動いていたらやり直す (短絡制動で減速するので待てば止まる)。
// 上限まで繰り返しても止まらなければ、最後の測定値を警告つきで採用する
// (理論値 CURRENT_REF_ADC は個体差が大きく、回転中の測定値よりも外れうるため使わない)
static void BLDC_CalibrateCurrentZero(void) {
  for (uint8_t attempt = 1;; attempt++) {
    int32_t before = *svc.encoder_val_ptr;
    CurrentSense_Calibrate();
    int32_t moved = (int32_t)*svc.encoder_val_ptr - before;
    if (moved < 0) moved = -moved;
    if (moved > MAX_ADC_VAL / 2) moved = MAX_ADC_VAL - moved;  // 0/2π の継ぎ目をまたいだ
    if (moved <= CURRENT_CALIB_MAX_ENCODER_MOVE_LSB) return;

    if (attempt >= CURRENT_CALIB_MAX_ATTEMPTS) {
      printf("CurrentSense: 警告 ロータが止まらないままゼロ点を校正した (エンコーダ %ld LSB 移動)\n",
             (long)moved);
      return;
    }
    printf("CurrentSense: ロータが回っている (エンコーダ %ld LSB 移動)。ゼロ点校正をやり直す\n",
           (long)moved);
  }
}

void BLDC_Init(bool do_set_encoder, volatile uint16_t* encoder_val, float supply_volt) {
  printf("BLDC_Init\n");

  svc.encoder_val_ptr = encoder_val;
  svc.mode = BLDC_MODE_STOP;
  svc.enable = false;
  // 母線電圧は最初に確定させる。暫定値のまま校正すると、電圧の水増しで ψm だけが
  // 狂う (12V暫定・実測8.6Vのとき ψm が 0.002663 対 真値 0.001908)。
  BLDC_SetSupplyVolt(supply_volt);

  BLDC_InitPwmTimer();

  // 電流センシング(TIM1同期ADC)を開始 (TIM1が動いてから)
  CurrentSense_Init();

  // モータ定数の初期値 (フラッシュに有効な値があれば上書きされる。校正中もR,L測定まではこれで回す)
  svc.motor_r = MOTOR_R_DEFAULT;
  svc.motor_l = MOTOR_L_DEFAULT;
  svc.motor_psi = MOTOR_PSI_DEFAULT;
  svc.angle_delay = ANGLE_DELAY_DEFAULT;

  // スイッチ押下あり: いま測る / なし: フラッシュから読む (検証に通ったものだけ採用)。
  // フラッシュ書き込みは、モータ定数測定まで終わってから1回でまとめて行う。
  if (do_set_encoder) {
    printf("BLDC_SetEncoder\n");
    BLDC_SetEncoder(encoder_val);
    svc.encoder_calibrated = true;
    printf("(Measured) このセッションの校正値を使う\n");
  } else {
    BLDC_LoadFlashCalibration();
  }
  BLDC_UpdateEncoderScale();

  printf("トルク定数 Kt: %.5f N・m/A, 最大トルク(MAX_CURRENT時): %.4f N・m\n",
         (double)BLDC_GetTorqueConstant(), (double)(BLDC_GetTorqueConstant() * MAX_CURRENT));

  BLDC_InitOuterGains();
  BLDC_UpdateCurrentGains();  // L と R から導く

  // 電流センサのゼロ点校正 (全相デューティ0.5で電流が流れない状態で行う)
  BLDC_WritePwm(0.5f, 0.5f, 0.5f);
  HAL_Delay(200);
  BLDC_CalibrateCurrentZero();

  // 制御ループを回す前に機械角を実測値で確定させる
  svc.encoder_primed = false;
  BLDC_UpdateEncoder(*svc.encoder_val_ptr);
  printf("BLDC: mech_theta = %.3f rad で開始\n", svc.mech_theta);

  // 指令が来るまで出力を切る (ゼロ点校正が終わってから)
  BLDC_SetOutputEnable(false);

#if PROFILE_ISR
  BLDC_InitProfiler();
#endif

  // 20kHz制御ループを開始
  CurrentSense_EnableInterrupt();
  printf("BLDC control loop start (%.0f Hz)\n", (double)PWM_FREQ);

  // 測定は制御ループが回っている必要があるのでここから下で行う
#if MOTOR_AUTO_CALIBRATION
  // スイッチ押下で起動したときだけモータ定数も測って保存する
  if (do_set_encoder) {
    BLDC_CalibrateMotor();
    BLDC_SaveCalibration();
  }
#endif
  BLDC_RunStartupMeasurements();

  // 校正が一時的に開けた制限値を閉じ、上位から制限値を受け取るまで動かないようにする
  BLDC_SetLimits(0.0f);
  printf(
      "BLDC: トルク上限 0 で開始 (上位からの指令待ち)。MD側の上限は %.3f N・m"
      " / 速度は %.0f rad/s で頭打ち\n",
      (double)BLDC_GetMaxTorqueNm(), (double)MAX_ANGULAR_SPEED);
}

// ---------------------------------------------------------------------------
// 公開API
// ---------------------------------------------------------------------------
void BLDC_SetSupplyVolt(float supply_volt) {
  if (supply_volt < 1.0f) supply_volt = 1.0f;  // 0除算防止
  svc.supply_volt = supply_volt;

  float v_limit = supply_volt * MAX_MODULATION_RATIO;
  svc.id_pi.output_limit = v_limit;
  svc.iq_pi.output_limit = v_limit;
}

// 表面磁石型PMSMのトルク式 T = 1.5×P×ψm×Iq より Kt [N・m/A]
float BLDC_GetTorqueConstant(void) { return 1.5f * POLE_PAIRS * svc.motor_psi; }

// MD 側の絶対上限
float BLDC_GetMaxTorqueNm(void) { return BLDC_GetTorqueConstant() * MAX_CURRENT; }

// 上位から指定された制限値を適用する。0 は「制限なし」ではなく「動かない」(詳細は bldc.h)。
void BLDC_SetLimits(float torque_limit_nm) {
  // トルク [N・m] → Iq指令 [A]。ψm が未校正で Kt が 0 のときは 0 (動かない) に倒す
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
  // モード切替時に位置PIDの微分項を初期化する (他モード中は prev_error が更新されず、出力が飛ぶため)
  if (svc.mode != mode) {
    svc.position_pid.prev_error = FOC_AngleDiff(svc.target_position, svc.mech_theta);
    svc.position_pid.d_term = 0;
    // 速度モードも現在の角速度から始める (ランプ・積分を他モードから持ち越さない)
    svc.speed_ramp = svc.angular_speed;
    svc.speed_pid.integral = 0;
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

void BLDC_TorqueControlNm(float torque_nm) {
  float kt = BLDC_GetTorqueConstant();
  BLDC_TorqueControl((kt > 1e-6f) ? (torque_nm / kt) : 0.0f);
}

void BLDC_BrakeControl(float brake_current) {
  svc.brake_current = Constrain(brake_current, 0.0f, MAX_CURRENT);
  BLDC_Enable(BLDC_MODE_BRAKE);
}

void BLDC_BrakeControlNm(float brake_torque_nm) {
  float kt = BLDC_GetTorqueConstant();
  BLDC_BrakeControl((kt > 1e-6f) ? (brake_torque_nm / kt) : 0.0f);
}

float BLDC_GetMechTheta(void) { return svc.mech_theta; }

// 割り込み内では正規化していないので、表示・送信用にここで 0〜2π に畳む
float BLDC_GetElecTheta(void) { return FOC_NormalizeRadians(svc.elec_theta); }
float BLDC_GetAngularSpeed(void) { return svc.angular_speed; }
float BLDC_GetId(void) { return svc.id; }
float BLDC_GetIq(void) { return svc.iq; }
float BLDC_GetTargetIq(void) { return svc.target_iq; }
float BLDC_GetVd(void) { return svc.vd; }
float BLDC_GetVq(void) { return svc.vq; }

// Vq のうちフィードフォワード分。符号が逆だと Vq と逆向きに出るので、実装ミスの確認に使う
float BLDC_GetVqFF(void) { return svc.vq_ff; }
bool BLDC_IsOvercurrent(void) { return svc.is_overcurrent; }
bool BLDC_IsEncoderCalibrated(void) { return svc.encoder_calibrated; }
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
