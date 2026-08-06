#include "app.h"

// 基板上のLEDの色と役割
//   LED1/LED2 (青)  : 起動シーケンスの進捗 → 運転中は負荷(Iq)のバーグラフ
//   LED3      (緑)  : 正常。初期化完了と、シリアルでコマンドを受け取っている間の点灯
//   LED4      (赤)  : 異常。点滅回数で種別 (1回:過熱 2回:電圧異常 3回:過電流)
static PwmOut LED1;
static PwmOut LED2;
static PwmOut LED3;
static PwmOut LED4;
static DigitalIn SW;

// 赤LEDの点滅回数でエラー種別を知らせる。異常時は他の色を消して赤だけにする。
#define ERROR_BLINK_OVERHEAT 1
#define ERROR_BLINK_VOLTAGE 2
#define ERROR_BLINK_OVERCURRENT 3

static Timer serial_send_timer;
static Timer serial_recv_timer;
static Timer status_print_timer;
#if PROFILE_ISR
static Timer profile_print_timer;
#endif

static Serial uart2;

static LPF supply_volt_lpf;
static LPF temp_lpf;

// ADC2の値を格納する配列 (エンコーダ, 電圧, 温度)。
// DMAが非同期に書き換えるので volatile 必須。これが無いと、下の
// 「値が入るまで待つ」ループが最適化で無限ループになりうる。
static volatile uint16_t adc_val[3];
#define ADC_ENCODER 0
#define ADC_SUPPLY_VOLT 1
#define ADC_TEMP 2
#define ADC_CHANNEL_COUNT 3

static float supply_volt;
static float temp;

static bool sw_state;

static bool is_overheat;
static bool is_voltage_out_of_range;

// 実際に適用しているトルク制限 (バイト表現のまま持つ)。状態フレームでエコーバックする。
// **起動時は 0 = 上限0。** 制限値を受け取るまでモータは動かない。
static uint8_t applied_torque_limit;

// 破棄したフレームの数 (通信品質の確認用)。printf の状態表示に出る。
static uint32_t rx_crc_error_count;     // CRC不一致で捨てた数
static uint32_t rx_unknown_mode_count;  // CRCは合ったがモードヘッダが未知だった数

void Setup(void) {
  printf("Hello World\n");
  printf("SystemCoreClock = %ld\n", SystemCoreClock);

  // LEDの初期化
  PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);
  PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);
  PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);
  PwmOut_Write(&LED1, 1);  // 青2つが点いている間は初期化中
  PwmOut_Write(&LED2, 1);
  PwmOut_Write(&LED3, 0);  // 緑は初期化完了で点ける
  PwmOut_Write(&LED4, 0);  // 赤は異常時のみ

  DigitalIn_Init(&SW, GPIOA, GPIO_PIN_12);

  // ADC2(エンコーダ・電圧・温度)のDMA開始
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_val, ADC_CHANNEL_COUNT);
  for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
    while (!(adc_val[i] > 0));  // ADCの値が代入されるまで待つ
  }

  printf("ADC_DMA start\n");
  PwmOut_Write(&LED1, 0);

  // ローパスフィルタの初期化。電源電圧は実測値を初期値にする。
  supply_volt = adc_val[ADC_SUPPLY_VOLT] * ADC2VOLT * 10.0f;
  LPF_Init(&supply_volt_lpf, 0.9, supply_volt);
  LPF_Init(&temp_lpf, 0.99, 30);

  // BLDCの初期化。PWM出力・電流センシング・20kHzの制御ループがここで立ち上がる。
  // スイッチを押しながら起動するとエンコーダの校正を行う。
  // 母線電圧は BLDC_Init に渡す。校正もこの中で走るので、実際の電圧を
  // 知らないまま測ると モータ定数が母線のずれの比だけ狂う (bldc.c 参照)。
  BLDC_Init(DigitalIn_Read(&SW), &adc_val[ADC_ENCODER], supply_volt);
  PwmOut_Write(&LED2, 0);

  // Serialの初期化
  Serial_Init(&uart2, &huart2, 256);

  PwmOut_Write(&LED3, 1);  // 初期化完了 (緑)
  HAL_Delay(200);
  PwmOut_Write(&LED3, 0);

  Timer_Init(&serial_send_timer);
  Timer_Init(&serial_recv_timer);
  Timer_Init(&status_print_timer);
#if PROFILE_ISR
  Timer_Init(&profile_print_timer);
  BLDC_ResetIsrProfile();  // BLDC_Init 中に溜まったぶんを捨てて測定開始点を揃える
#endif
}

// ---------------------------------------------------------------------------
// 状態表示 (printf)
// ---------------------------------------------------------------------------
// 電流の実測値を定期表示する。ゲイン調整と過電流のしきい値決めに使う。
// STATUS_PRINT_INTERVAL_S を 0 にすると無効。
static void PrintStatus(void) {
  if (STATUS_PRINT_INTERVAL_S <= 0) return;
  if (Timer_Read(&status_print_timer) < STATUS_PRINT_INTERVAL_S) return;
  Timer_Reset(&status_print_timer);

  BLDCEncoderStats enc;
  BLDC_GetEncoderStats(&enc);

  // Vq の内訳 (FF分) も出す。フィードフォワードが正しく効いていれば、速度が
  // 上がるほど FF が Vq の大半を占め、PIは残差だけを相手にする。
  // **符号を間違えると FF が Vq と逆向きに出る**ので、ここを見れば一目で分かる。
  //
  // **Vd は電気角が合っているかの判定に使う。**
  // 角度が合っていれば、d軸に必要な電圧は干渉項だけなので Vd ≈ −ω_e·L·Iq で
  // ほぼゼロになる。推定角が真の角から δ ずれていると、逆起電力がd軸に漏れて
  // Vd ≈ −ω_e·ψm·sin δ が現れる。つまり **Vd の大きさがそのまま角度ずれの証拠**。
  printf(
      "Iq:%+6.2f/%+6.2fA  Id:%+6.2fA  Vd:%+5.2fV  Vq:%+5.2f(FF%+5.2f)/%5.2fV"
      "  peak:%5.2fA  %6.1frad/s"
      "  sat:%4lu glt:%4lu emax:%.4frad  %4.1fV %2.0fC"
      "  lim:%.3fN・m  crcerr:%lu mode?:%lu\n",
      BLDC_GetIq(), BLDC_GetTargetIq(), BLDC_GetId(), BLDC_GetVd(),
      BLDC_GetVq(), BLDC_GetVqFF(), supply_volt * MAX_MODULATION_RATIO,
      BLDC_GetPeakCurrent(), BLDC_GetAngularSpeed(),
      (unsigned long)enc.saturated, (unsigned long)enc.glitch, enc.max_innovation,
      supply_volt, temp,
      (double)(applied_torque_limit * SERIAL_TORQUE_LIMIT_SCALE),
      (unsigned long)rx_crc_error_count, (unsigned long)rx_unknown_mode_count);
  BLDC_ResetPeakCurrent();   // 次の区間のピークを測るためリセット
  BLDC_ResetEncoderStats();  // sat/glt/emax はこの表示区間あたりの値
}

// 20kHz制御ループの実行時間を定期表示する。
// 制御周期を上げる/処理を足す前の余裕の確認に使う。
// PROFILE_ISR / PROFILE_PRINT_INTERVAL_S を 0 にすると丸ごと消える。
static void PrintProfile(void) {
#if PROFILE_ISR
  if (PROFILE_PRINT_INTERVAL_S <= 0) return;

  // CPU使用率と実測レートは「合計サイクル数 ÷ 実経過時間」で出すので、
  // 設定値ではなく実際に経過した時間を渡す。
  float elapsed = Timer_Read(&profile_print_timer);
  if (elapsed < PROFILE_PRINT_INTERVAL_S) return;
  Timer_Reset(&profile_print_timer);

  BLDC_PrintIsrProfile(elapsed);
#endif
}

// ---------------------------------------------------------------------------
// センサ
// ---------------------------------------------------------------------------
static void GetSensors(void) {
  // 電源電圧の変換(分圧で1/10にしている)
  supply_volt = adc_val[ADC_SUPPLY_VOLT] * ADC2VOLT * 10.0f;
  supply_volt = LPF_Update(&supply_volt_lpf, supply_volt);  // ローパスフィルタを適用
  BLDC_SetSupplyVolt(supply_volt);                          // 変調率の計算に使うのでBLDCへ渡す

  // MCP9700T/HTT温度センサの変換 (0.5V が 0°C、10mV/°C)
  float temp_voltage = adc_val[ADC_TEMP] * ADC2VOLT;
  temp = (temp_voltage - 0.5f) * 100.0f;
  temp = LPF_Update(&temp_lpf, temp);  // ローパスフィルタを適用

  // スイッチ
  sw_state = DigitalIn_Read(&SW);
}

// ---------------------------------------------------------------------------
// シリアル受信
// ---------------------------------------------------------------------------
// フレームの並びは serial_protocol.h を参照 (6バイト固定長 + CRC-8/AUTOSAR)。
//
// モードごとに「どのヘッダか」「どんな単位か」「どのBLDC APIを呼ぶか」を
// 別々の if 連鎖で何度も書いていたので、表にまとめた。
// モードを増やすときはこの表に1行足すだけでよい。
// **スケールは上位側と対で決まっている。勝手に変えないこと。**
typedef struct {
  uint8_t header;        // モードヘッダ
  float scale;           // 受信した int16 に掛ける係数
  void (*apply)(float);  // 指令を渡す BLDC 側のAPI
} SerialCommand;

static const SerialCommand SERIAL_COMMANDS[] = {
    {0xBA, 0.01f, BLDC_AngularSpeedControl},  // 角速度 [rad/s]
    {0xBB, 0.001f, BLDC_PositionControl},     // 位置 [rad]
    {0xBC, 0.001f, BLDC_TorqueControl},       // トルク(Iq指令) [A]
    {0xBD, 0.0001f, BLDC_TorqueControlNm},    // トルク指令 [N・m] (Ktで換算)
    {0xBE, 0.001f, BLDC_BrakeControl},        // 制動電流 [A]
    {0xBF, 0.0001f, BLDC_BrakeControlNm},     // 制動トルク [N・m] (Ktで換算)
};
#define SERIAL_COMMAND_COUNT (sizeof(SERIAL_COMMANDS) / sizeof(SERIAL_COMMANDS[0]))

// いま実行中の指令。**NULL = 停止モード** (起動直後・通信断・過電流ラッチ)。
// 目標値を「モードごとの変数」ではなく1つに持てるのは、受信フレームが常に
// 「モード + その1つの指令値」で完結しているため。モードが変われば前の値は
// もう使わないので、取っておく理由がない。
static const SerialCommand* active_command;
static float target_value;

// 受信したトルク制限を「MD が実際に適用する値」に落として BLDC へ渡す。
//
// **比較をバイトのまま行うのが肝。** 一度 float にしてから min を取り、
// エコーバックのためにバイトへ戻すと、丸めで受信値と1LSBずれることがある。
// 上位から見ると「20 を送ったのに 19 が返ってきた」となり、制限が効いているのか
// 通信が化けたのか区別できなくなる。バイト領域なら往復が必ず完全一致する。
//
// MD 側の絶対上限は Kt×MAX_CURRENT。ψm (校正値) 次第で変わるので実行時に計算する。
// 速度に上位から指定する制限は無い (MAX_ANGULAR_SPEED は bldc.c 側の固定保護)。
static void ApplyLimits(uint8_t rx_torque_limit) {
  float hw_torque = BLDC_GetMaxTorqueNm() / SERIAL_TORQUE_LIMIT_SCALE;

  // 切り捨て。上限を1LSBでも上回る値をエコーしないため。
  uint8_t hw_torque_byte = (hw_torque >= 255.0f) ? 255 : (uint8_t)hw_torque;

  applied_torque_limit = (rx_torque_limit < hw_torque_byte) ? rx_torque_limit : hw_torque_byte;

  BLDC_SetLimits(applied_torque_limit * SERIAL_TORQUE_LIMIT_SCALE);
}

// 受け取った6バイトの解釈結果。
// **「CRCが合わない」と「CRCは合ったが知らないモード」を必ず分けること。**
// 前者は同期がずれている可能性があるので詰め直しが要るが、後者はフレーム境界は
// 正しく取れているので詰め直してはいけない。カウンタも別にしないと、
// 通信品質 (CRCエラー) と上位の設定ミス (未対応モード) の区別がつかなくなる。
typedef enum {
  FRAME_ACCEPTED,      // 受理して指令に反映した
  FRAME_BAD_CRC,       // ヘッダかCRCが不一致 → 同期がずれているかもしれない
  FRAME_UNKNOWN_MODE,  // CRCは通ったが、この MD が知らないモードヘッダ
} FrameResult;

static FrameResult AcceptFrame(const uint8_t* frame) {
  SerialRxFrame f;
  if (!SerialProtocol_Decode(frame, &f)) return FRAME_BAD_CRC;

  // モードヘッダを表から引く。CRCが通っているので化けではなく、
  // 上位が未対応のモードを送ってきたということ。
  const SerialCommand* command = NULL;
  for (uint8_t i = 0; i < SERIAL_COMMAND_COUNT; i++) {
    if (SERIAL_COMMANDS[i].header == f.mode) {
      command = &SERIAL_COMMANDS[i];
      break;
    }
  }
  if (command == NULL) return FRAME_UNKNOWN_MODE;

  active_command = command;
  target_value = f.setpoint * command->scale;

  // **制限値は指令と同じフレームに載っている (CANopen の RxPDO 相当)。**
  // 1フレームが常に完結した状態を表すので、取りこぼしてもMDがリセットしても
  // 次のフレームで復元される。設定値の同期という問題自体が起きない。
  ApplyLimits(f.torque_limit);

  PwmOut_Write(&LED3, 0.25);  // 緑点灯 = コマンドを受信できている
  Timer_Reset(&serial_recv_timer);
  return FRAME_ACCEPTED;
}

// 停止モードへ落とす。制限値も 0 に戻す。
// 通信が切れた後で再開するときは、必ず新しい制限値を受け取ってから
// 動き出してほしいため (起動直後と同じ状態にそろえる)。
static void EnterStopMode(void) {
  active_command = NULL;
  applied_torque_limit = 0;
  BLDC_SetLimits(0.0f);
}

static void RecvSerial(void) {
  static uint8_t frame[SERIAL_RX_FRAME_SIZE];
  static uint8_t len = 0;

  for (uint8_t budget = SERIAL_RX_BYTES_PER_CALL; budget > 0 && Serial_Available(&uart2); budget--) {
    frame[len++] = Serial_Read(&uart2);

    // 先頭は必ず 0xAA。違えばそのバイトは捨てて次を待つ。
    if (len == 1) {
      if (frame[0] != SERIAL_HEADER) len = 0;
      continue;
    }
    if (len < SERIAL_RX_FRAME_SIZE) continue;

    FrameResult result = AcceptFrame(frame);
    if (result != FRAME_BAD_CRC) {
      // 受理できた、または「CRCは通ったが知らないモード」。どちらもフレーム境界は
      // 正しく取れているので、次の6バイトをそのまま次のフレームとして読めばよい。
      if (result == FRAME_UNKNOWN_MODE) rx_unknown_mode_count++;
      len = 0;
      continue;
    }

    // **破棄する。前回の指令値と制限値をそのまま保持する。**
    // 誤った値で制御するより、1周期古い正しい値を使う方が安全。
    rx_crc_error_count++;

    // 再同期。バイト落ちで同期がずれると、フレームの途中に居るのに
    // 「6バイト読んだ」と判定してしまう。捨てたバッファの中の次の 0xAA を
    // 先頭に詰め直せば、本物のフレーム境界を1バイトも読み飛ばさずに拾い直せる。
    // (単に len = 0 に戻すと、この中にあった本物のヘッダごと捨ててしまい、
    //  復帰が1フレーム余計に遅れる)
    uint8_t next = 1;
    while (next < SERIAL_RX_FRAME_SIZE && frame[next] != SERIAL_HEADER) next++;
    len = (uint8_t)(SERIAL_RX_FRAME_SIZE - next);
    memmove(frame, &frame[next], len);
  }

  // 一定時間まともなフレームが来ない場合は停止モードにする。
  // **上位はこれを停止手段として使っている。**
  if (Timer_Read(&serial_recv_timer) > SERIAL_TIMEOUT_S) {
    EnterStopMode();
    PwmOut_Write(&LED3, 0);  // 通信が途切れたので緑を消す
    Serial_Reset(&uart2);
    len = 0;  // 途中まで溜まっていたフレームも捨てる
    Timer_Reset(&serial_recv_timer);
  }
}

// ---------------------------------------------------------------------------
// シリアル送信
// ---------------------------------------------------------------------------
static void SendSerial(void) {
  if (Timer_ReadUs(&serial_send_timer) <= SERIAL_SEND_INTERVAL_US) return;

  static uint8_t data[SERIAL_TX_FRAME_SIZE];

  // **1つの値につき取得は1回だけ。** これらは20kHzの割り込みが書き換えるので、
  // 上位バイトと下位バイトで別々に取得すると違う瞬間の値が混ざる。
  SerialTxFrame f;
  f.status = (BLDC_IsOvercurrent() << 3) | (is_overheat << 2) |
             (is_voltage_out_of_range << 1) | (active_command != NULL);
  f.temperature = (uint8_t)Constrain(temp, 0.0f, 255.0f);  // u8 の範囲外を折り返さない
  f.theta = (uint16_t)(BLDC_GetMechTheta() * 10000);       // 機械角 [0.1mrad]
  f.speed = (int16_t)(BLDC_GetAngularSpeed() * 100);       // 角速度 [0.01rad/s]
  f.iq = (int16_t)(BLDC_GetIq() * 1000);                   // q軸電流 [mA]

  // **受信値そのままではなく、MD が実際に適用している値**を返す。
  // 上位から「本当に効いている制限」が見えることに意味がある。
  f.torque_limit = applied_torque_limit;

  SerialProtocol_Encode(&f, data);
  Serial_Write(&uart2, data, sizeof(data));  // シリアル送信
  Timer_Reset(&serial_send_timer);
}

// ---------------------------------------------------------------------------
// 異常処理
// ---------------------------------------------------------------------------
// 異常を赤LED(LED4)の点滅回数で知らせる。点滅の間は青・緑を消して
// 「赤だけが点滅している = 異常」と一目で分かるようにする。
// MainAppのループから毎周期呼ばれるので、1回の呼び出しで1パターンだけ出す。
static void BlinkError(uint8_t blink_count) {
  PwmOut_Write(&LED1, 0);
  PwmOut_Write(&LED2, 0);
  PwmOut_Write(&LED3, 0);

  for (uint8_t i = 0; i < blink_count; i++) {
    PwmOut_Write(&LED4, 1);
    HAL_Delay(100);
    PwmOut_Write(&LED4, 0);
    HAL_Delay(100);
  }
  HAL_Delay(400);  // パターンの区切り (点滅回数を数えやすくする)
}

// ラッチ式の異常処理。復帰条件を満たすまでモータを止め、赤LEDを点滅させ続ける。
//
// 復帰条件にヒステリシスを持たせているのは、しきい値ちょうどで
// 停止と復帰を往復させないため (過熱なら -5°C、電圧なら ±0.5V)。
// 過熱と電圧異常でまったく同じ形だったのでまとめた。
static void LatchFault(bool* latched, bool recovered, uint8_t blink_count) {
  BLDC_Stop();  // モーターストップ

  if (recovered) {
    *latched = false;
    PwmOut_Write(&LED4, 0);  // 復帰したので赤を消す
  } else {
    BlinkError(blink_count);
  }
}

// 過電流保護が働いた瞬間の状態を一度だけ表示する (原因の切り分け用)。
static void ReportOvercurrentTrip(void) {
  BLDCTripInfo t;
  BLDC_GetTripInfo(&t);
  printf("=== Overcurrent (limit %.1fA) ===\n", (double)OVERCURRENT_LIMIT);
  printf("  peak:%.2fA  Iu:%+.2f Iv:%+.2f Iw:%+.2f A\n", t.peak_current, t.iu, t.iv, t.iw);
  printf("  raw  U:%4u V:%4u  (中点%.0f)\n", t.raw_u, t.raw_v, (double)CURRENT_REF_ADC);
  printf("  Id:%+.2f Iq:%+.2f A  (指令 Iq:%+.2f A)\n", t.id, t.iq, t.target_iq);
  printf("  Vd:%+.2f Vq:%+.2f V  (上限%.2f V)\n", t.vd, t.vq,
         supply_volt * MAX_MODULATION_RATIO);
  printf("  theta:%.3f rad  speed:%.1f rad/s  Vdc:%.2f V\n",
         t.mech_theta, t.angular_speed, supply_volt);
}

// 過電流ラッチ。制御ループ側で既にモーターは止まっている。
// スイッチを押すまで解除しない。
static void HandleOvercurrent(void) {
  static bool trip_reported = false;

  if (!trip_reported) {
    trip_reported = true;
    ReportOvercurrentTrip();
  }
  active_command = NULL;
  BlinkError(ERROR_BLINK_OVERCURRENT);

  if (sw_state) {  // スイッチを押すと復帰
    trip_reported = false;
    BLDC_ResetPeakCurrent();
    BLDC_ClearOvercurrent();
    Serial_Reset(&uart2);    // 停止中に溜まった受信データを捨てる
    PwmOut_Write(&LED4, 0);  // 復帰したので赤を消す
  }
}

// 異常の検出とラッチ処理をまとめる。異常が続いている間は true を返す
// (モータはこの中で止めてあるので、呼び出し側は指令の処理を丸ごと飛ばす)。
static bool HandleFaults(void) {
  if (temp > TEMP_LIMIT || is_overheat) {
    printf("Overheat! Temperature: %.2f°C, Supply Voltage: %.2fV\n", temp, supply_volt);
    is_overheat = true;
    LatchFault(&is_overheat, temp < (TEMP_LIMIT - 5), ERROR_BLINK_OVERHEAT);
    return true;
  }

  if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT ||
      is_voltage_out_of_range) {
    printf("Supply voltage out of range: %.2fV, Temperature: %.2f°C\n", supply_volt, temp);
    is_voltage_out_of_range = true;
    LatchFault(&is_voltage_out_of_range,
               supply_volt > (SUPPLY_VOLTAGE_MIN_LIMIT + 0.5f) &&
                   supply_volt < (SUPPLY_VOLTAGE_MAX_LIMIT - 0.5f),
               ERROR_BLINK_VOLTAGE);
    return true;
  }

  if (BLDC_IsOvercurrent()) {
    HandleOvercurrent();
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
// メインループ
// ---------------------------------------------------------------------------
// 正常時の状態表示。赤は消し、青2つでq軸電流の大きさをバーグラフにする
// (LED1が0→100%、そこから先をLED2が0→100%で引き継ぐ)。
static void UpdateLoadLeds(void) {
  PwmOut_Write(&LED4, 0);
  float iq_ratio = Abs(BLDC_GetIq()) / MAX_CURRENT;
  PwmOut_Write(&LED1, iq_ratio * 2.0f);
  PwmOut_Write(&LED2, iq_ratio * 2.0f - 1.0f);
}

void MainApp(void) {
  while (1) {
    GetSensors();
    SendSerial();
    PrintStatus();
    PrintProfile();

    if (HandleFaults()) continue;

    RecvSerial();

    // 表から引いた BLDC 側のAPIをそのまま呼ぶ。
    // active_command == NULL が停止モード。
    if (active_command != NULL) {
      active_command->apply(target_value);
    } else {
      BLDC_Stop();
    }

    UpdateLoadLeds();
  }
}
