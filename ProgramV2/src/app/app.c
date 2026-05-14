#include "app.h"

#define ADC2VOLT 0.0008058608059

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
DigitalIn SW;

Timer serial_send_timer;
Timer serial_recv_timer;

Serial uart2;

LPF supply_volt_lpf;
LPF temp_lpf;

uint16_t adc_val[3];  // ADCの値を格納する配列

uint16_t encoder_val, supply_volt_val, temp_val;
float supply_volt;
float temp;

bool sw_state;

bool is_overheat;
bool is_voltage_out_of_range;

float target_angular_speed, target_voltage, target_position, brake_volt;

uint8_t mode = 0;  // 制御モード(0: 停止, 1: 角速度制御, 2: 位置制御, 3: トルク制御)

void Setup() {
  printf("Hello World\n");
  printf("SystemCoreClock = %ld\n", SystemCoreClock);

  // LEDの初期化
  PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);
  PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);
  PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);
  PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);
  PwmOut_Write(&LED1, 1);
  PwmOut_Write(&LED2, 1);
  PwmOut_Write(&LED3, 1);

  DigitalIn_Init(&SW, GPIOA, GPIO_PIN_0);

  // ADCの初期化
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_val, 3);
  for (uint8_t i = 0; i < 3; i++) {
    while (!(adc_val[i] > 0));  // ADCの値が代入されるまで待つ
  }
  printf("ADC_DMA start\n");
  PwmOut_Write(&LED1, 0);

  // BLDCの初期化
  BLDC_Init(DigitalIn_Read(&SW), &adc_val[0]);
  PwmOut_Write(&LED2, 0);

  // Serialの初期化
  Serial_Init(&uart2, &huart2, 256);

  // ローパスフィルタの初期化
  LPF_Init(&supply_volt_lpf, 0.9, 12);  // 電圧
  LPF_Init(&temp_lpf, 0.99, 30);        // 温度

  PwmOut_Write(&LED3, 0);

  Timer_Init(&serial_send_timer);
  Timer_Reset(&serial_send_timer);
  Timer_Init(&serial_recv_timer);
  Timer_Reset(&serial_recv_timer);
}

void GetSensors() {
  encoder_val = adc_val[0];      // エンコーダの値
  supply_volt_val = adc_val[1];  // 電圧の値
  temp_val = adc_val[2];         // 温度の値

  // 電源電圧の変換(分圧で1/10にしている)
  supply_volt = supply_volt_val * ADC2VOLT * 10.0f;
  supply_volt = LPF_Update(&supply_volt_lpf, supply_volt);  // ローパスフィルタを適用

  // MCP9700T/HTT温度センサの変換
  float temp_voltage = temp_val * ADC2VOLT;  // ADC値 → 電圧変換

  temp = (temp_voltage - 0.5) * 100;   // 電圧 → 温度変換
  temp = LPF_Update(&temp_lpf, temp);  // ローパスフィルタを適用

  // スイッチ
  sw_state = DigitalIn_Read(&SW);
}

void RecvSerial() {
  const static uint8_t HEADER = 0xFF;
  const static uint8_t ANGULAR_SPEED_HEADER = 0xFE;
  const static uint8_t POSITION_HEADER = 0xFD;
  const static uint8_t TORQUE_HEADER = 0xFC;
  const static uint8_t BRAKE_HEADER = 0xFB;
  const static uint8_t FOOTER = 0xAA;
  const static uint8_t DATA_SIZE = 2;
  static uint8_t recv_data[2];
  static uint8_t index = 0;

  if (Serial_Available(&uart2)) {
    uint8_t recv_byte = Serial_Read(&uart2);
    if (index == 0) {
      if (recv_byte == HEADER) {
        index++;
      } else {
        index = 0;
      }
    } else if (index == 1) {
      if (recv_byte == ANGULAR_SPEED_HEADER) {
        mode = 1;  // 角速度制御モード
        index++;
      } else if (recv_byte == POSITION_HEADER) {
        mode = 2;  // 位置制御モード
        index++;
      } else if (recv_byte == TORQUE_HEADER) {
        mode = 3;  // トルク制御モード
        index++;
      } else if (recv_byte == BRAKE_HEADER) {
        mode = 4;  // ブレーキモード
        index++;
      } else {
        index = 0;
      }
    } else if (index == (DATA_SIZE + 2)) {
      if (recv_byte == FOOTER) {
        PwmOut_Write(&LED3, 1);
        if (mode == 1) {
          target_angular_speed = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.01;  // 角速度制御
        } else if (mode == 2) {
          target_position = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.001;  // 位置制御
        } else if (mode == 3) {
          target_voltage = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.0001;  // 電圧制御
        } else if (mode == 4) {
          brake_volt = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.0001;  // ブレーキ制御
        }

        Timer_Reset(&serial_recv_timer);
      }
      index = 0;
    } else {
      recv_data[index - 2] = recv_byte;
      index++;
    }
  } else if (Timer_Read(&serial_recv_timer) > 0.5) {
    mode = 0;  // 一定時間データが受信されない場合は停止モードにする
    PwmOut_Write(&LED3, 0);
    Serial_Reset(&uart2);
    Timer_Reset(&serial_recv_timer);
  }
}

void SendSerial() {
  if (Timer_ReadUs(&serial_send_timer) > SERIAL_SEND_INTERVAL_US) {  // 指定された間隔ごとにシリアル送信
    const static uint8_t HEADER = 0xFF;
    const static uint8_t FOOTER = 0xAA;
    static uint8_t data[9];

    data[0] = HEADER;
    data[1] = (is_overheat << 2) | (is_voltage_out_of_range << 1) | (mode != 0);
    data[2] = ((uint16_t)(BLDC_GetMechTheta() * 10000) >> 8) & 0xFF;
    data[3] = (uint16_t)(BLDC_GetMechTheta() * 10000) & 0xFF;
    data[4] = ((int16_t)(BLDC_GetAngularSpeed() * 100) >> 8) & 0xFF;
    data[5] = (int16_t)(BLDC_GetAngularSpeed() * 100) & 0xFF;
    data[6] = ((int16_t)(BLDC_GetAngularAccel() * 10) >> 8) & 0xFF;
    data[7] = (int16_t)(BLDC_GetAngularAccel() * 10) & 0xFF;
    data[8] = FOOTER;

    Serial_Write(&uart2, data, sizeof(data));  // シリアル送信
    Timer_Reset(&serial_send_timer);
  }
}

void MainApp() {
  while (1) {
    GetSensors();
    SendSerial();

    if (temp > TEMP_LIMIT || is_overheat == true) {
      printf("Overheat! Temperature: %.2f°C, Supply Voltage: %.2fV\n", temp, supply_volt);
      is_overheat = true;
      BLDC_Stop(false);  // モーターストップ

      if (is_overheat == true && temp < (TEMP_LIMIT - 5)) {
        is_overheat = false;
        PwmOut_Write(&LED4, 0);
      } else {
        PwmOut_Write(&LED1, 0);
        PwmOut_Write(&LED2, 0);
        PwmOut_Write(&LED4, 0);
        PwmOut_Write(&LED3, 1);
        HAL_Delay(100);
        PwmOut_Write(&LED3, 0);
        HAL_Delay(100);
      }
    } else if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT || is_voltage_out_of_range == true) {
      printf("Supply voltage out of range: %.2fV, Temperature: %.2f°C\n", supply_volt, temp);
      is_voltage_out_of_range = true;
      BLDC_Stop(false);  // モーターストップ

      if (is_voltage_out_of_range == true && supply_volt > (SUPPLY_VOLTAGE_MIN_LIMIT + 0.5) && supply_volt < (SUPPLY_VOLTAGE_MAX_LIMIT - 0.5)) {
        is_voltage_out_of_range = false;
        PwmOut_Write(&LED2, 0);
      } else {
        PwmOut_Write(&LED1, 1);
        PwmOut_Write(&LED2, 0);
        PwmOut_Write(&LED3, 0);
        PwmOut_Write(&LED4, 0);
        HAL_Delay(100);
        PwmOut_Write(&LED1, 0);
        HAL_Delay(100);
      }
    } else {
      RecvSerial();
      if (mode == 0) {
        BLDC_Stop(false);  // モーターストップ

        PwmOut_Write(&LED1, 0);
        PwmOut_Write(&LED2, 0);
      } else {
        BLDC_SensoredVectorControlDrive(encoder_val, supply_volt);
        if (mode == 1) {
          BLDC_AngularSpeedControl(target_angular_speed);  // 角速度制御
        } else if (mode == 2) {
          BLDC_PositionControl(target_position);  // 位置制御
        } else if (mode == 3) {
          BLDC_VoltageControl(target_voltage);  // 電圧制御
        } else if (mode == 4) {
          BLDC_VoltageControl(brake_volt * Constrain(BLDC_GetAngularSpeed() * 0.05, -1, 1));  // ブレーキ
        }

        // 状態の表示
        PwmOut_Write(&LED1, Abs(BLDC_GetAmpVolt()) * 0.4);
        PwmOut_Write(&LED2, Abs(BLDC_GetAmpVolt()) * 0.4 - 1);
      }
    }
  }
}
