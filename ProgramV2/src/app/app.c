#include "app.h"

#define ADC2VOLT 0.0008058608059

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
DigitalIn SW;

SensoredVectorControl svc;

Timer serial_send_timer;
Timer serial_recv_timer;

Serial pc;
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

bool enable = false;
bool done_setup = false;

float target_speed, target_torque, target_position, brake_torque;

uint8_t mode = 0;  // 制御モード(0: 停止, 1: 速度制御, 2: 位置制御, 3: トルク制御)

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
  BLDC_Init(&svc, DigitalIn_Read(&SW), &adc_val[0]);
  PwmOut_Write(&LED2, 0);

  // Serialの初期化
  Serial_Init(&pc, &huart1, 256);
  Serial_Init(&uart2, &huart2, 256);

  // ローパスフィルタの初期化
  LPF_Init(&supply_volt_lpf, 0.9, 12);  // 電圧
  LPF_Init(&temp_lpf, 0.99, 30);        // 温度

  PwmOut_Write(&LED3, 0);

  Timer_Init(&serial_send_timer);
  Timer_Reset(&serial_send_timer);
  Timer_Init(&serial_recv_timer);
  Timer_Reset(&serial_recv_timer);

  done_setup = true;
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

void TimerInterrupt() {
  if (enable == true) {
    if (mode == 0) {
      BLDC_Stop(false);  // モーターストップ
    } else if (mode == 1) {
      BLDC_SpeedControl(&svc, target_speed);  // 速度制御
    } else if (mode == 2) {
      BLDC_PositionControl(&svc, target_position);  // 位置制御
    } else if (mode == 3) {
      BLDC_TorqueControl(&svc, target_torque);  // トルク制御
    } else if (mode == 4) {
      BLDC_TorqueControl(&svc, brake_torque * Constrain(svc.speed * 0.05, -1, 1));  // ブレーキ
    }
    BLDC_SensoredVectorControlDrive(&svc, encoder_val, supply_volt);
  } else {
    if (done_setup == true) BLDC_Stop(false);  // モーターストップ
  }
}

void MainApp() {
  while (1) {
    GetSensors();
    if (temp > TEMP_LIMIT || is_overheat == true) {
      printf("Overheat! Temperature: %.2f°C\n", temp);
      enable = false;
      is_overheat = true;
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
      BLDC_OpenLoopDrive(0, 0);  // モーターフリー状態
    } else if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT || is_voltage_out_of_range == true) {
      printf("Supply voltage out of range: %.2fV\n", supply_volt);
      enable = false;
      is_voltage_out_of_range = true;
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
      BLDC_OpenLoopDrive(0, 0);  // モーターフリー状態
    } else {
      const static uint8_t HEADER = 0xFF;
      const static uint8_t SPEED_HEADER = 0xFE;
      const static uint8_t POSITION_HEADER = 0xFD;
      const static uint8_t TORQUE_HEADER = 0xFC;
      const static uint8_t BRAKE_HEADER = 0xFB;
      const static uint8_t FOOTER = 0xAA;
      const static uint8_t DATA_SIZE = 2;
      static uint8_t recv_data[2];
      static uint8_t index = 0;

      while (Serial_Available(&uart2)) {
        uint8_t recv_byte = Serial_Read(&uart2);
        enable = true;
        if (index == 0) {
          if (recv_byte == HEADER) {
            index++;
          } else {
            index = 0;
          }
        } else if (index == 1) {
          if (recv_byte == SPEED_HEADER) {
            mode = 1;  // 速度制御モード
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
              target_speed = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.01;  // 速度制御
            } else if (mode == 2) {
              target_position = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.001;  // 位置制御
            } else if (mode == 3) {
              target_torque = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.01;  // トルク制御
            } else if (mode == 4) {
              brake_torque = (int16_t)((recv_data[0] << 8) | recv_data[1]) * 0.01;  // ブレーキ制御
            }

            Timer_Reset(&serial_recv_timer);
          }
          index = 0;
        } else {
          recv_data[index - 2] = recv_byte;
          index++;
        }
      }
      if (Timer_Read(&serial_recv_timer) > 0.5) {
        enable = false;
        PwmOut_Write(&LED3, 0);
        Serial_Reset(&uart2);
        Timer_Reset(&serial_recv_timer);
      }

      // 状態の表示
      PwmOut_Write(&LED1, Abs(svc.amp_volt) * 0.4);
      PwmOut_Write(&LED2, Abs(svc.amp_volt) * 0.4 - 1);
    }
    if (Timer_Read(&serial_send_timer) > 0.001) {  // 1msごとにシリアル送信
      const static uint8_t HEADER = 0xFF;
      const static uint8_t FOOTER = 0xAA;
      static uint8_t data[8];

      data[0] = HEADER;
      data[1] = (is_overheat << 2) | (is_voltage_out_of_range << 1) | enable;
      data[2] = ((int16_t)(svc.speed * 100) >> 8) & 0xFF;
      data[3] = (int16_t)(svc.speed * 100) & 0xFF;
      data[4] = ((int16_t)(svc.mech_theta * 1000) >> 8) & 0xFF;
      data[5] = (int16_t)(svc.mech_theta * 1000) & 0xFF;
      data[6] = (uint8_t)(abs(svc.amp_volt) * 10);
      data[7] = FOOTER;

      Serial_Write(&uart2, data, sizeof(data));  // シリアル送信
      Timer_Reset(&serial_send_timer);
    }
  }
}