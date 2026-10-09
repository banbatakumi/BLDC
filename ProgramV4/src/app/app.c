// serial.h の HAL_UART_RxCpltCallback をこの.cファイルでのみ実体化する (多重定義の回避。serial.h 参照)
#define SERIAL_DEFINE_DMA_CALLBACKS
#include "app.h"

// LED1/LED2 (青): 起動の進捗 → 運転中は負荷(Iq)のバーグラフ
// LED3 (緑): 初期化完了と、コマンド受信中の点灯
// LED4 (赤): 異常。点滅回数で種別 (1回:過熱 2回:電圧異常 3回:過電流)
static PwmOut LED1;
static PwmOut LED2;
static PwmOut LED3;
static PwmOut LED4;
static DigitalIn SW;

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

// ADC2の値 (エンコーダ, 電圧, 温度)。DMAが書き換えるので volatile 必須。
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
static bool is_voltage_pending;       // 範囲外を検出して猶予時間を計測中
static uint32_t voltage_out_since_ms;  // 範囲外になった時刻 (HAL_GetTick)

static bool fault_active;             // 前の周に異常 (過熱・電圧・過電流) が続いていたか
static uint32_t fault_since_ms;       // 異常に入った時刻 (点滅パターンの起点)
static uint32_t fault_print_ms;       // 異常の内容を最後に printf した時刻

// 実際に適用しているトルク制限 (バイト表現)。状態フレームでエコーバックする。起動時は 0 = 動かない。
static uint8_t applied_torque_limit;

// 破棄したフレームの数 (printf の状態表示用)
static uint32_t rx_crc_error_count;     // CRC不一致
static uint32_t rx_unknown_mode_count;  // CRCは合ったがモードヘッダが未知

void Setup(void) {
  printf("Hello World\n");
  printf("SystemCoreClock = %ld\n", SystemCoreClock);

  PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);
  PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);
  PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);
  PwmOut_Write(&LED1, 1);  // 青2つが点いている間は初期化中
  PwmOut_Write(&LED2, 1);
  PwmOut_Write(&LED3, 0);  // 緑は初期化完了で点ける
  PwmOut_Write(&LED4, 0);  // 赤は異常時のみ

  DigitalIn_Init(&SW, GPIOA, GPIO_PIN_12);

  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_val, ADC_CHANNEL_COUNT);
  for (uint8_t i = 0; i < ADC_CHANNEL_COUNT; i++) {
    while (!(adc_val[i] > 0));  // 値が入るまで待つ
  }

  printf("ADC_DMA start\n");
  PwmOut_Write(&LED1, 0);

  supply_volt = adc_val[ADC_SUPPLY_VOLT] * ADC2VOLT * 10.0f;
  LPF_Init(&supply_volt_lpf, 0.9, supply_volt);
  LPF_Init(&temp_lpf, 0.99, 30);

  // PWM出力・電流センシング・20kHz制御ループが立ち上がる。スイッチ押下で起動すると校正を行う。
  // 校正中の電圧換算に使うので、実測の母線電圧を渡す。
  BLDC_Init(DigitalIn_Read(&SW), &adc_val[ADC_ENCODER], supply_volt);
  PwmOut_Write(&LED2, 0);

  Serial_Init(&uart2, &huart2, 256);

  PwmOut_Write(&LED3, 1);  // 初期化完了 (緑)
  HAL_Delay(200);
  PwmOut_Write(&LED3, 0);

  Timer_Init(&serial_send_timer);
  Timer_Init(&serial_recv_timer);
  Timer_Init(&status_print_timer);
#if PROFILE_ISR
  Timer_Init(&profile_print_timer);
  BLDC_ResetIsrProfile();
#endif
}

// ---------------------------------------------------------------------------
// 状態表示 (printf)
// ---------------------------------------------------------------------------
// 電流などを定期表示する (ゲイン調整・過電流しきい値の決定用)。STATUS_PRINT_INTERVAL_S = 0 で無効。
static void PrintStatus(void) {
  if (STATUS_PRINT_INTERVAL_S <= 0) return;
  if (Timer_Read(&status_print_timer) < STATUS_PRINT_INTERVAL_S) return;
  Timer_Reset(&status_print_timer);

  BLDCEncoderStats enc;
  BLDC_GetEncoderStats(&enc);

  // Vq(FF...) はFF分。符号が正しければ FF は Vq と同符号で、速度とともに大半を占める。
  // Vd は電気角が合っているかの指標。合っていれば Vd ≈ −ω_e·L·Iq でほぼ0、
  // δ ずれていると Vd ≈ −ω_e·ψm·sin δ が現れる。
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
  BLDC_ResetPeakCurrent();   // peak, sat/glt/emax は表示区間ごとの値
  BLDC_ResetEncoderStats();
}

// 20kHz制御ループの実行時間を定期表示する (PROFILE_ISR / PROFILE_PRINT_INTERVAL_S = 0 で削除)
static void PrintProfile(void) {
#if PROFILE_ISR
  if (PROFILE_PRINT_INTERVAL_S <= 0) return;

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
  supply_volt = LPF_Update(&supply_volt_lpf, supply_volt);
  BLDC_SetSupplyVolt(supply_volt);

  // MCP9700T/HTT温度センサの変換 (0.5V が 0°C、10mV/°C)
  float temp_voltage = adc_val[ADC_TEMP] * ADC2VOLT;
  temp = (temp_voltage - 0.5f) * 100.0f;
  temp = LPF_Update(&temp_lpf, temp);

  sw_state = DigitalIn_Read(&SW);
}

// ---------------------------------------------------------------------------
// シリアル受信
// ---------------------------------------------------------------------------
// フレームの並びは serial_protocol.h を参照 (6バイト固定長 + CRC-8/AUTOSAR)。
// モードごとのヘッダ・単位・呼ぶBLDC APIを表にまとめてある。モードを増やすときは1行足す。
// スケールは上位側と対で決まっているので勝手に変えないこと。
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

// いま実行中の指令。NULL = 停止モード (起動直後・通信断・異常中)。
// 受信フレームが常に「モード + 1つの指令値」で完結しているので、目標値は1つでよい。
static const SerialCommand* active_command;
static float target_value;

// 受信したトルク制限を MD が実際に適用する値に落として BLDC へ渡す。
// 比較はバイトのまま行う (floatで min を取ると丸めで1LSBずれ、エコーバックと一致しなくなる)。
// MD 側の絶対上限は Kt×MAX_CURRENT で、ψm (校正値) に依存するので実行時に計算する。
static void ApplyLimits(uint8_t rx_torque_limit) {
  float hw_torque = BLDC_GetMaxTorqueNm() / SERIAL_TORQUE_LIMIT_SCALE;

  // 切り捨て (上限を1LSBでも上回る値をエコーしないため)
  uint8_t hw_torque_byte = (hw_torque >= 255.0f) ? 255 : (uint8_t)hw_torque;

  // エンコーダ未校正のまま電流を流すと電気角が合わず、指令と無関係な向きに回る。
  // 上限 0 のまま動かさない (エコーバックも 0 になるので上位は制限値の不一致として検知できる)
  if (!BLDC_IsEncoderCalibrated()) hw_torque_byte = 0;

  applied_torque_limit = (rx_torque_limit < hw_torque_byte) ? rx_torque_limit : hw_torque_byte;

  BLDC_SetLimits(applied_torque_limit * SERIAL_TORQUE_LIMIT_SCALE);
}

// 受け取った6バイトの解釈結果。「CRC不一致」(同期ずれの可能性があり詰め直しが要る) と
// 「CRCは合ったが未知のモード」(境界は正しいので詰め直さない) は分けて数える。
typedef enum {
  FRAME_ACCEPTED,
  FRAME_BAD_CRC,
  FRAME_UNKNOWN_MODE,
} FrameResult;

static FrameResult AcceptFrame(const uint8_t* frame) {
  SerialRxFrame f;
  if (!SerialProtocol_Decode(frame, &f)) return FRAME_BAD_CRC;

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

  // 制限値は指令と同じフレームに載っているので、取りこぼしやMDのリセットも次のフレームで復元される
  ApplyLimits(f.torque_limit);

  PwmOut_Write(&LED3, 0.25);  // 緑点灯 = 受信できている
  Timer_Reset(&serial_recv_timer);
  return FRAME_ACCEPTED;
}

// 停止モードへ落とす。再開時に新しい制限値を受け取ってから動くよう、制限値も 0 に戻す。
static void EnterStopMode(void) {
  active_command = NULL;
  applied_torque_limit = 0;
  BLDC_SetLimits(0.0f);
}

// 組み立て中の受信フレーム (異常からの復帰時に捨てられるよう、関数の外に置く)
static uint8_t rx_frame[SERIAL_RX_FRAME_SIZE];
static uint8_t rx_len = 0;

// 受信をやり直す。溜まっているバイトと組み立て途中のフレームを捨て、無通信タイムアウトも測り直す
static void RestartReceive(void) {
  Serial_Reset(&uart2);
  rx_len = 0;
  Timer_Reset(&serial_recv_timer);
}

static void RecvSerial(void) {
  for (uint8_t budget = SERIAL_RX_BYTES_PER_CALL; budget > 0 && Serial_Available(&uart2); budget--) {
    rx_frame[rx_len++] = Serial_Read(&uart2);

    // 先頭は 0xAA。違えば捨てる
    if (rx_len == 1) {
      if (rx_frame[0] != SERIAL_HEADER) rx_len = 0;
      continue;
    }
    if (rx_len < SERIAL_RX_FRAME_SIZE) continue;

    FrameResult result = AcceptFrame(rx_frame);
    if (result != FRAME_BAD_CRC) {
      // 受理、または未知モード。どちらも境界は正しいので次の6バイトをそのまま読む
      if (result == FRAME_UNKNOWN_MODE) rx_unknown_mode_count++;
      rx_len = 0;
      continue;
    }

    // 破棄して前回の指令値と制限値を保持する (誤った値より1周期古い正しい値の方が安全)
    rx_crc_error_count++;

    // 再同期: バッファ内の次の 0xAA を先頭に詰め直す (rx_len = 0 だと中にある本物のヘッダごと捨て、復帰が遅れる)
    uint8_t next = 1;
    while (next < SERIAL_RX_FRAME_SIZE && rx_frame[next] != SERIAL_HEADER) next++;
    rx_len = (uint8_t)(SERIAL_RX_FRAME_SIZE - next);
    memmove(rx_frame, &rx_frame[next], rx_len);
  }

  // 一定時間フレームが来なければ停止モードにする (通信断の保険)
  if (Timer_Read(&serial_recv_timer) > SERIAL_TIMEOUT_S) {
    EnterStopMode();
    PwmOut_Write(&LED3, 0);
    RestartReceive();
  }
}

// ---------------------------------------------------------------------------
// シリアル送信
// ---------------------------------------------------------------------------
static void SendSerial(void) {
  if (Timer_ReadUs(&serial_send_timer) <= SERIAL_SEND_INTERVAL_US) return;

  static uint8_t data[SERIAL_TX_FRAME_SIZE];

  // 20kHz割り込みが書き換える値なので、1つの値につき取得は1回だけにする
  SerialTxFrame f;
  f.status = (!BLDC_IsEncoderCalibrated() << SERIAL_STATUS_BIT_UNCALIBRATED) |
             (BLDC_IsOvercurrent() << SERIAL_STATUS_BIT_OVERCURRENT) |
             (is_overheat << SERIAL_STATUS_BIT_OVERHEAT) |
             (is_voltage_out_of_range << SERIAL_STATUS_BIT_VOLTAGE) |
             ((active_command != NULL) << SERIAL_STATUS_BIT_COMMANDED);
  f.temperature = (uint8_t)Constrain(temp, 0.0f, 255.0f);
  f.theta = (uint16_t)(BLDC_GetMechTheta() * 10000);  // 機械角 [0.1mrad]
  // 角速度 [0.01rad/s]。クランプは100倍後の値なので i16 のレンジで指定する
  f.speed = (int16_t)Constrain(BLDC_GetAngularSpeed() * 100, -32767.0f, 32767.0f);
  f.iq = (int16_t)(BLDC_GetIq() * 1000);  // q軸電流 [mA]

  // 受信値ではなく MD が実際に適用している値を返す
  f.torque_limit = applied_torque_limit;

  SerialProtocol_Encode(&f, data);
  Serial_Write(&uart2, data, sizeof(data));
  Timer_Reset(&serial_send_timer);
}

// ---------------------------------------------------------------------------
// 異常処理
// ---------------------------------------------------------------------------
// 異常を赤LED(LED4)の点滅回数で知らせる (青・緑は消す)。100ms 点灯 + 100ms 消灯を
// blink_count 回、そのあと 400ms 空ける。
// 待たずに、異常に入ってからの経過時間で点灯/消灯を決める。以前は HAL_Delay で待っていたため
// 異常中は状態フレームが約1Hzに落ち、上位が異常ビットを知るのが 0.6〜1.0秒遅れていた
static void BlinkError(uint8_t blink_count) {
  PwmOut_Write(&LED1, 0);
  PwmOut_Write(&LED2, 0);
  PwmOut_Write(&LED3, 0);

  uint32_t blink_ms = (uint32_t)blink_count * 200u;
  uint32_t t = (HAL_GetTick() - fault_since_ms) % (blink_ms + 400u);
  PwmOut_Write(&LED4, (t < blink_ms && (t % 200u) < 100u) ? 1 : 0);
}

// 異常が続いている間の printf を間引く (ブロッキングなので毎周出すと状態フレームが遅れる)。
// 異常に入った周と、以後 FAULT_PRINT_INTERVAL_MS ごとに true を返す
static bool FaultPrintDue(void) {
  uint32_t now = HAL_GetTick();
  if (fault_active && now - fault_print_ms < FAULT_PRINT_INTERVAL_MS) return false;
  fault_print_ms = now;
  return true;
}

// 異常中に共通の処置。モータを止め、受理済みの指令と制限値も捨てる (状態フレームの
// 「指令あり」ビットと制限値のエコーが落ちるので、上位からも止まっていることが分かる)
static void StopForFault(void) {
  BLDC_Stop();
  EnterStopMode();
}

// ラッチ式の異常処理。復帰条件を満たすまでモータを止め、赤LEDを点滅させ続ける。
// 復帰条件にはヒステリシスを持たせる (過熱なら -5°C、電圧なら SUPPLY_VOLTAGE_RECOVER_HYST)。
static void LatchFault(bool* latched, bool recovered, uint8_t blink_count) {
  StopForFault();

  if (recovered) {
    *latched = false;
    PwmOut_Write(&LED4, 0);
  } else {
    BlinkError(blink_count);
  }
}

// 過電流保護が働いた瞬間の状態を一度だけ表示する
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

// 過電流ラッチ。スイッチを押すまで解除しない。
// 制御ループ側もラッチ中は出力を切るが、こちらでも明示的に止める (enable を偽に戻す)
static void HandleOvercurrent(void) {
  static bool trip_reported = false;

  StopForFault();
  if (!trip_reported) {
    trip_reported = true;
    ReportOvercurrentTrip();
  }
  BlinkError(ERROR_BLINK_OVERCURRENT);

  if (sw_state) {  // スイッチを押すと復帰 (受信のやり直しは MainApp の復帰処理で行う)
    trip_reported = false;
    BLDC_ResetPeakCurrent();
    BLDC_ClearOvercurrent();
    PwmOut_Write(&LED4, 0);
  }
}

// 異常の検出とラッチ処理。異常が続いている間は true を返す (モータは停止済み)。
static bool HandleFaults(void) {
  if (temp > TEMP_LIMIT || is_overheat) {
    if (FaultPrintDue()) {
      printf("Overheat! Temperature: %.2f°C, Supply Voltage: %.2fV\n", temp, supply_volt);
    }
    is_overheat = true;
    LatchFault(&is_overheat, temp < (TEMP_LIMIT - 5), ERROR_BLINK_OVERHEAT);
    return true;
  }

  // 電圧は範囲外が SUPPLY_VOLTAGE_TRIP_DELAY_MS 連続したときだけ停止する (範囲内に戻れば計測をやり直す)
  bool volt_out = supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT;
  if (!is_voltage_out_of_range) {
    if (!volt_out) {
      is_voltage_pending = false;
    } else if (!is_voltage_pending) {
      is_voltage_pending = true;
      voltage_out_since_ms = HAL_GetTick();
    } else if (HAL_GetTick() - voltage_out_since_ms >= SUPPLY_VOLTAGE_TRIP_DELAY_MS) {
      is_voltage_pending = false;
      is_voltage_out_of_range = true;
    }
  }

  if (is_voltage_out_of_range) {
    if (FaultPrintDue()) {
      printf("Supply voltage out of range: %.2fV, Temperature: %.2f°C\n", supply_volt, temp);
    }
    LatchFault(&is_voltage_out_of_range,
               supply_volt > (SUPPLY_VOLTAGE_MIN_LIMIT + SUPPLY_VOLTAGE_RECOVER_HYST) &&
                   supply_volt < (SUPPLY_VOLTAGE_MAX_LIMIT - SUPPLY_VOLTAGE_RECOVER_HYST),
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
// 正常時の表示。青2つでq軸電流のバーグラフ (LED1が0→100%、続きをLED2が引き継ぐ)
static void UpdateLoadLeds(void) {
  PwmOut_Write(&LED4, 0);
  float iq_ratio = Abs(BLDC_GetIq()) / MAX_CURRENT;
  PwmOut_Write(&LED1, iq_ratio * 2.0f);
  PwmOut_Write(&LED2, iq_ratio * 2.0f - 1.0f);
}

void MainApp(void) {
  // ブロッキングする初期化・校正 (Setup) がすべて終わってから起動する
  Watchdog_Start(WATCHDOG_TIMEOUT_MS);

  while (1) {
    Watchdog_Refresh();

    GetSensors();
    SendSerial();
    PrintStatus();
    PrintProfile();

    // 異常中も上の SendSerial は回り続け、状態フレーム (異常ビット) を通常の周期で返す。
    // 指令は受け取らない (受信バッファは溜まるに任せ、復帰時に捨てる)
    if (!fault_active) fault_since_ms = HAL_GetTick();
    bool faulted = HandleFaults();
    if (faulted) {
      fault_active = true;
      continue;
    }
    if (fault_active) {
      // 異常から復帰した。異常の間に届いていた指令や、異常の前に受理していた指令で
      // 動き出さないよう、すべて捨てて新しいフレームを待つ
      fault_active = false;
      EnterStopMode();
      RestartReceive();
    }

    RecvSerial();

    if (active_command != NULL) {
      active_command->apply(target_value);
    } else {
      BLDC_Stop();
    }

    UpdateLoadLeds();
  }
}
