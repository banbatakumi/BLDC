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

static float supply_volt;
static float temp;

static bool sw_state;

static bool is_overheat;
static bool is_voltage_out_of_range;

// 制御モード。シリアルのヘッダバイトで切り替わる。
typedef enum {
  APP_MODE_STOP = 0,
  APP_MODE_SPEED,
  APP_MODE_POSITION,
  APP_MODE_TORQUE,
  APP_MODE_BRAKE,
} AppMode;

static AppMode mode = APP_MODE_STOP;

static float target_angular_speed, target_current, target_position, brake_current;

void Setup() {
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
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_val, 3);
  for (uint8_t i = 0; i < 3; i++) {
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
static void HandleFault(bool* latched, bool recovered, uint8_t blink_count) {
  BLDC_Stop();  // モーターストップ

  if (recovered) {
    *latched = false;
    PwmOut_Write(&LED4, 0);  // 復帰したので赤を消す
  } else {
    BlinkError(blink_count);
  }
}

// 電流の実測値を定期表示する。ゲイン調整と過電流のしきい値決めに使う。
// STATUS_PRINT_INTERVAL_S を 0 にすると無効。
static void PrintStatus() {
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
      "  sat:%4lu glt:%4lu emax:%.4frad  %4.1fV %2.0fC\n",
      BLDC_GetIq(), BLDC_GetTargetIq(), BLDC_GetId(), BLDC_GetVd(),
      BLDC_GetVq(), BLDC_GetVqFF(), supply_volt * MAX_MODULATION_RATIO,
      BLDC_GetPeakCurrent(), BLDC_GetAngularSpeed(),
      (unsigned long)enc.saturated, (unsigned long)enc.glitch, enc.max_innovation,
      supply_volt, temp);
  BLDC_ResetPeakCurrent();   // 次の区間のピークを測るためリセット
  BLDC_ResetEncoderStats();  // sat/glt/emax はこの表示区間あたりの値
}

// 20kHz制御ループの実行時間を定期表示する。
// 制御周期を上げる/処理を足す前の余裕の確認に使う。
// PROFILE_ISR / PROFILE_PRINT_INTERVAL_S を 0 にすると丸ごと消える。
static void PrintProfile() {
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

static void GetSensors() {
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
// パケット形式: [0xFF][モードヘッダ][データ上位][データ下位][0xAA]
//
// モードごとに「どのヘッダか」「どの変数へ入れるか」「どんな単位か」を
// 別々の if 連鎖で2回書いていたので、表にまとめた。
// モードを増やすときはこの表に1行足すだけでよい。
typedef struct {
  uint8_t header;
  AppMode mode;
  float scale;    // 受信した int16 に掛ける係数
  float* target;  // 格納先
} SerialCommand;

static const SerialCommand SERIAL_COMMANDS[] = {
    {0xFE, APP_MODE_SPEED, 0.01f, &target_angular_speed},  // 角速度 [rad/s]
    {0xFD, APP_MODE_POSITION, 0.001f, &target_position},   // 位置 [rad]
    {0xFC, APP_MODE_TORQUE, 0.001f, &target_current},      // トルク(Iq指令) [A]
    {0xFB, APP_MODE_BRAKE, 0.001f, &brake_current},        // 制動電流 [A]
};
#define SERIAL_COMMAND_COUNT (sizeof(SERIAL_COMMANDS) / sizeof(SERIAL_COMMANDS[0]))

static void RecvSerial() {
  static const uint8_t HEADER = 0xFF;
  static const uint8_t FOOTER = 0xAA;
  static const uint8_t DATA_SIZE = 2;
  static uint8_t recv_data[2];
  static uint8_t index = 0;
  static const SerialCommand* command = NULL;

  if (Serial_Available(&uart2)) {
    uint8_t recv_byte = Serial_Read(&uart2);
    if (index == 0) {
      index = (recv_byte == HEADER) ? 1 : 0;
    } else if (index == 1) {
      // モードヘッダを表から引く。見つからなければ先頭から取り直す。
      command = NULL;
      for (uint8_t i = 0; i < SERIAL_COMMAND_COUNT; i++) {
        if (SERIAL_COMMANDS[i].header == recv_byte) {
          command = &SERIAL_COMMANDS[i];
          break;
        }
      }
      if (command != NULL) {
        mode = command->mode;
        index++;
      } else {
        index = 0;
      }
    } else if (index == (DATA_SIZE + 2)) {
      if (recv_byte == FOOTER && command != NULL) {
        PwmOut_Write(&LED3, 1);  // 緑点灯 = コマンドを受信できている
        int16_t raw = (int16_t)((recv_data[0] << 8) | recv_data[1]);
        *command->target = raw * command->scale;

        Timer_Reset(&serial_recv_timer);
      }
      index = 0;
    } else {
      recv_data[index - 2] = recv_byte;
      index++;
    }
  } else if (Timer_Read(&serial_recv_timer) > 0.5) {
    mode = APP_MODE_STOP;    // 一定時間データが受信されない場合は停止モードにする
    PwmOut_Write(&LED3, 0);  // 通信が途切れたので緑を消す
    Serial_Reset(&uart2);
    Timer_Reset(&serial_recv_timer);
  }
}

static void SendSerial() {
  if (Timer_ReadUs(&serial_send_timer) <= SERIAL_SEND_INTERVAL_US) return;

  static const uint8_t HEADER = 0xFF;
  static const uint8_t FOOTER = 0xAA;
  static uint8_t data[12];

  // **1つの値につき取得は1回だけ。** これらは20kHzの割り込みが書き換えるので、
  // 上位バイトと下位バイトで別々に取得すると違う瞬間の値が混ざる。
  uint16_t theta = (uint16_t)(BLDC_GetMechTheta() * 10000);  // 機械角 [0.1mrad]
  int16_t speed = (int16_t)(BLDC_GetAngularSpeed() * 100);   // 角速度 [0.01rad/s]
  int16_t accel = (int16_t)(BLDC_GetAngularAccel() * 10);    // 角加速度 [0.1rad/s^2]
  int16_t iq = (int16_t)(BLDC_GetIq() * 1000);               // q軸電流 [mA]

  data[0] = HEADER;
  data[1] = (BLDC_IsOvercurrent() << 3) | (is_overheat << 2) |
            (is_voltage_out_of_range << 1) | (mode != APP_MODE_STOP);
  data[2] = (uint8_t)temp;
  data[3] = (theta >> 8) & 0xFF;
  data[4] = theta & 0xFF;
  data[5] = (speed >> 8) & 0xFF;
  data[6] = speed & 0xFF;
  data[7] = (accel >> 8) & 0xFF;
  data[8] = accel & 0xFF;
  data[9] = (iq >> 8) & 0xFF;
  data[10] = iq & 0xFF;
  data[11] = FOOTER;

  Serial_Write(&uart2, data, sizeof(data));  // シリアル送信
  Timer_Reset(&serial_send_timer);
}

// 過電流保護が働いた瞬間の状態を一度だけ表示する (原因の切り分け用)。
static void ReportOvercurrentTrip() {
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

void MainApp() {
  while (1) {
    GetSensors();
    SendSerial();
    PrintStatus();
    PrintProfile();

    if (temp > TEMP_LIMIT || is_overheat) {
      printf("Overheat! Temperature: %.2f°C, Supply Voltage: %.2fV\n", temp, supply_volt);
      is_overheat = true;
      HandleFault(&is_overheat, temp < (TEMP_LIMIT - 5), ERROR_BLINK_OVERHEAT);
    } else if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT ||
               supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT || is_voltage_out_of_range) {
      printf("Supply voltage out of range: %.2fV, Temperature: %.2f°C\n", supply_volt, temp);
      is_voltage_out_of_range = true;
      HandleFault(&is_voltage_out_of_range,
                  supply_volt > (SUPPLY_VOLTAGE_MIN_LIMIT + 0.5f) &&
                      supply_volt < (SUPPLY_VOLTAGE_MAX_LIMIT - 0.5f),
                  ERROR_BLINK_VOLTAGE);
    } else if (BLDC_IsOvercurrent()) {
      // 過電流保護。制御ループ側で既にモーターは止まっている。
      static bool trip_reported = false;
      if (!trip_reported) {
        trip_reported = true;
        ReportOvercurrentTrip();
      }
      mode = APP_MODE_STOP;
      BlinkError(ERROR_BLINK_OVERCURRENT);
      if (sw_state) {  // スイッチを押すと復帰
        trip_reported = false;
        BLDC_ResetPeakCurrent();
        BLDC_ClearOvercurrent();
        Serial_Reset(&uart2);    // 停止中に溜まった受信データを捨てる
        PwmOut_Write(&LED4, 0);  // 復帰したので赤を消す
      }
    } else {
      RecvSerial();

      switch (mode) {
        case APP_MODE_SPEED:
          BLDC_AngularSpeedControl(target_angular_speed);
          break;
        case APP_MODE_POSITION:
          BLDC_PositionControl(target_position);
          break;
        case APP_MODE_TORQUE:
          BLDC_TorqueControl(target_current);
          break;
        case APP_MODE_BRAKE:
          BLDC_BrakeControl(brake_current);
          break;
        case APP_MODE_STOP:
        default:
          BLDC_Stop();
          break;
      }

      // 状態の表示。正常なので赤は消し、青2つでq軸電流の大きさをバーグラフにする
      // (LED1が0→100%、そこから先をLED2が0→100%で引き継ぐ)。
      PwmOut_Write(&LED4, 0);
      float iq_ratio = Abs(BLDC_GetIq()) / MAX_CURRENT;
      PwmOut_Write(&LED1, iq_ratio * 2.0f);
      PwmOut_Write(&LED2, iq_ratio * 2.0f - 1.0f);
    }
  }
}
