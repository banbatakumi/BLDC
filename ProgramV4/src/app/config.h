#ifndef CONFIG_H_
#define CONFIG_H_

// ===========================================================================
// 保護
// ===========================================================================
#define TEMP_LIMIT 60  // 温度制限 [°C]

#define SUPPLY_VOLTAGE_MIN_LIMIT 7
#define SUPPLY_VOLTAGE_MAX_LIMIT 20

// ===========================================================================
// 通信
// ===========================================================================
#define SERIAL_SEND_INTERVAL_US 1000  // シリアル送信間隔 [μs]

// printf での状態表示の間隔 [s]。0 で無効 (コンパイル時に丸ごと削除される)。
// PIDゲインの調整時に 0.5f 程度にすると Iq の追従と電流ピークを確認できる。
#define STATUS_PRINT_INTERVAL_S 0.0f

// ===========================================================================
// PWM / 制御周期
// ===========================================================================
#define PWM_ARR 3599       // TIM1のARR。72MHz / (3599+1) = 20kHz
#define PWM_FREQ 20000.0f  // PWM周波数 = 電流ループ周波数 [Hz]

#define CURRENT_LOOP_DT (1.0f / PWM_FREQ)  // 電流ループ周期 [s] (50µs)

// 外側ループはADC完了割り込みを分周して回す
#define OUTER_LOOP_DIV 20   // 速度/位置制御 → 1kHz
#define SPEED_CALC_DIV 40   // 角速度計算 → 500Hz
#define ACCEL_CALC_DIV 400  // 角加速度計算 → 50Hz

#define OUTER_LOOP_DT (CURRENT_LOOP_DT * OUTER_LOOP_DIV)
#define SPEED_CALC_DT (CURRENT_LOOP_DT * SPEED_CALC_DIV)
#define ACCEL_CALC_DT (CURRENT_LOOP_DT * ACCEL_CALC_DIV)

// デューティの上限。
// ローサイドシャントは低側FETがONの区間しか相電流が流れないので、ADCのサンプリング点
// (CNT = CURRENT_SENSE_TRIG_POINT) で低側FETがONになっている必要がある。
//   デューティ上限の理論値 = CURRENT_SENSE_TRIG_POINT / (PWM_ARR + 1) = 0.889
// 実際にはデッドタイムとINA181の整定時間ぶんの余裕が要るので 0.80 とする。
// (0.80なら低側ON後 3200-2880=320count = 4.4µs 経ってからサンプリングされる)
#define MAX_DUTY 0.80f
#define MIN_DUTY 0.02f

// 電圧ベクトルの大きさの上限 (対 母線電圧)。
// SVPWM(三次高調波重畳)なので相電圧振幅は (MAX_DUTY - 0.5) * 2/√3 まで出せる。
#define MAX_MODULATION_RATIO ((MAX_DUTY - 0.5f) * 1.1547005f)

// ===========================================================================
// 電流センシング (ローサイドシャント + 双方向電流センスアンプ INA181A1)
// ===========================================================================
//   回路: CurcuitV4  R16/R24 = 5mΩ,  U4/U6 = INA181A1
//   REF は R28-R31 の 1k/1k 分圧で 3.3V/2 = 1.65V に固定されているので、
//   無電流時の出力は約1.65V = ADC 2048 になり、正負どちらの電流も読める。
#define SHUNT_RESISTANCE 0.005f  // シャント抵抗 [Ω]
#define CURRENT_AMP_GAIN 20.0f   // 電流センスアンプ INA181A1 のゲイン [V/V]

#define CURRENT_REF_ADC 2048.0f       // 無電流時の出力(理論値) [ADC]
#define CURRENT_OFFSET_TOLERANCE 250  // オフセット校正値の許容ずれ [ADC]

// ADCのトリガ位置。TIM1_CH4のCCR4に設定し、OC4REF→TRGO2→ADC1外部トリガとする。
#define CURRENT_SENSE_TRIG_POINT 3200

// 電流の符号
//   INA181の IN+(pin3) が下側FETのソース(シャント上側)、IN-(pin4) が GNDPWR に繋がっている。
//   相電流がインバータからモータへ流れ出す向き(正)のとき、電流は
//   GNDPWR → シャント → 下側FET → モータ と流れるのでシャント上側は GNDPWR より
//   低電位になり、アンプ出力は REF より下がる。
//   よって「(ADC値 - オフセット) × -1」が相電流になる。
#define CURRENT_SIGN (-1.0f)

#define CURRENT_LPF_COEF 0.25f  // 測定電流のローパス係数 (1.0で無フィルタ)

// ===========================================================================
// BLDCのパラメーター
// ===========================================================================
#define POLE_PAIRS 7              // 極対数 (磁石の数/2)
#define MAX_ANGULAR_SPEED 100.0f  // 最大角速度 [rad/s]
#define MAX_ANGULAR_ACCEL 50.0f   // 最大角加速度 [rad/s^2]

// エンコーダの1制御周期(50µs)あたりの機械角の変化量の上限 [rad]。
// ロータには慣性があるので物理的に動ける量には限りがある。AS5600のアナログ出力が
// 0/2πを跨ぐ瞬間に拾うグリッチはこれを大きく超えるので、ここで頭打ちにして
// 電気角(機械角の極対数倍)が飛ぶのを防ぐ。
// MAX_ANGULAR_SPEED の4倍の速度まで追従できる余裕を持たせている。
#define ENCODER_MAX_STEP_RAD (MAX_ANGULAR_SPEED * 4.0f * CURRENT_LOOP_DT)

#define MAX_CURRENT 7.0f         // Id/Iq指令の上限 [A]
#define OVERCURRENT_LIMIT 10.0f  // 過電流保護のしきい値 [A]

// 連続して何サンプルしきい値を超えたら保護を発動するか。
// 1サンプル(50µs)だけのノイズで誤検出しないためのデバウンス。
// 3 なら 150µs 継続した過電流で発動する。
#define OVERCURRENT_TRIP_COUNT 3

// 電流PI制御のゲイン
//   理論値は  Kp = L * 2π * f_bw,  Ki = R * 2π * f_bw
//   (L: 相インダクタンス [H], R: 相抵抗 [Ω], f_bw: 目標帯域 [Hz])
//   例: L=0.5mH, R=1Ω, f_bw=300Hz → Kp=0.94, Ki=1885
//   まずは低めから始めて、電流が振動しない範囲で上げていくこと。
#define CURRENT_PI_KP 0.5f    // [V/A]
#define CURRENT_PI_KI 500.0f  // [V/(A·s)]

// 電気角のオフセット [rad]
//   エンコーダ校正 (BLDC_SetEncoder) 後は elec_theta = mech_theta * POLE_PAIRS で
//   d軸(ロータ磁束方向)に一致するので通常は0。
//   Iq指令を入れたときにIdが大きく出る場合はここを調整する。
#define ELEC_THETA_OFFSET 0.0f

#endif  // CONFIG_H_
