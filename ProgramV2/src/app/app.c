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
double supply_volt;
double temp;

bool sw_state;

bool is_overheat;

bool enable = false;
bool done_setup = false;

int16_t target_speed, target_torque;
float target_position;

uint8_t mode = 0;  // 制御モード(0: 停止, 1: 速度制御, 2: 位置制御)

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
      HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&adc_val, 3);
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
                  // トルク制御の実装は省略
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
                  enable = false;
                  printf("Overheat! Temperature: %.2f°C\n", temp);
                  is_overheat = true;
                  if (is_overheat == true && temp < (TEMP_LIMIT - 5)) {
                        is_overheat = false;
                        PwmOut_Write(&LED4, 0);
                  } else {
                        PwmOut_Write(&LED1, 0);
                        PwmOut_Write(&LED2, 0);
                        PwmOut_Write(&LED4, 1);
                        PwmOut_Write(&LED3, 1);
                        HAL_Delay(100);
                        PwmOut_Write(&LED3, 0);
                        HAL_Delay(100);
                  }
                  BLDC_OpenLoopDrive(0, 0);  // モーターフリー状態
            } else if (supply_volt > SUPPLY_VOLTAGE_MAX_LIMIT || supply_volt < SUPPLY_VOLTAGE_MIN_LIMIT) {
                  enable = false;
                  printf("Supply voltage out of range: %.2fV\n", supply_volt);
                  BLDC_OpenLoopDrive(0, 0);  // モーターフリー状態
                  PwmOut_Write(&LED1, 0);
                  PwmOut_Write(&LED2, 0);
                  PwmOut_Write(&LED3, 0);
                  PwmOut_Write(&LED4, 1);
                  HAL_Delay(250);
                  PwmOut_Write(&LED4, 0);
                  HAL_Delay(250);
            } else {
                  const static uint8_t SPEED_HEADER = 0xFF;
                  const static uint8_t POSITION_HEADER = 0xFE;
                  const static uint8_t TORQUE_HEADER = 0xFD;
                  const static uint8_t FOOTER = 0xAA;
                  const uint8_t data_size = 2;
                  static uint8_t recv_data[2];
                  static uint8_t index = 0;
                  uint8_t recv_byte = Serial_Read(&pc);

                  if (Serial_Available(&pc)) {
                        enable = true;
                        PwmOut_Write(&LED3, 1);
                        if (index == 0) {
                              if (recv_byte == SPEED_HEADER) {
                                    mode = 1;  // 速度制御モード
                                    index++;
                              } else if (recv_byte == POSITION_HEADER) {
                                    mode = 2;  // 位置制御モード
                                    index++;
                              } else if (recv_byte == TORQUE_HEADER) {
                                    mode = 3;  // トルク制御モード
                                    index++;
                              } else {
                                    index = 0;
                              }
                        } else if (index == (data_size + 1)) {
                              if (recv_byte == FOOTER) {
                                    if (mode == 1) {
                                          target_speed = (recv_data[0] << 8) | recv_data[1];  // 速度制御
                                          printf("Target Speed: %d\n", target_speed);
                                    } else if (mode == 2) {
                                          target_position = ((recv_data[0] << 8) | recv_data[1]) * 0.001;  // 位置制御
                                          printf("Target Position: %.2f rad\n", target_position);
                                    } else if (mode == 3) {
                                          target_torque = (recv_data[0] << 8) | recv_data[1];  // トルク制御
                                          printf("Target Torque: %d\n", target_torque);
                                    }
                              }
                              index = 0;
                        } else {
                              recv_data[index - 1] = recv_byte;
                              index++;
                        }
                        Timer_Reset(&serial_recv_timer);
                  } else if (Timer_Read(&serial_recv_timer) > 0.5) {  // 100msごとにシリアル受信
                        enable = false;
                        PwmOut_Write(&LED3, 0);
                        Serial_Reset(&uart2);
                        Timer_Reset(&serial_recv_timer);
                  }

                  // if (Serial_Available(&uart2)) {
                  //       enable = true;
                  //       // target_rad = Serial_Read(&uart2) * (TWO_PI / 255.0f);  // 0〜255の値を0〜2πのラジアンに変換
                  //       target_rad = Serial_Read(&uart2);
                  //       Timer_Reset(&serial_recv_timer);
                  //       PwmOut_Write(&LED3, 1);
                  // } else if (Timer_Read(&serial_recv_timer) > 0.5) {  // 100msごとにシリアル受信
                  //       enable = false;
                  //       PwmOut_Write(&LED3, 0);
                  //       Serial_Reset(&uart2);
                  //       Timer_Reset(&serial_recv_timer);
                  // }

                  // if (Timer_Read(&serial_send_timer) > 0.01) {                                          // 100msごとにシリアル送信
                  //       uint8_t rad = (svc.mech_theta + svc.encoder_offset_theta) * (255.0f / TWO_PI);  // ラジアンを0〜255の値に変換
                  //       Serial_Write(&uart2, (uint8_t *)&rad, 1);  // シリアルに送信
                  //       Timer_Reset(&serial_send_timer);
                  // }

                  // 状態の表示
                  PwmOut_Write(&LED1, Abs(svc.amp) * 5);
                  PwmOut_Write(&LED2, Abs(svc.amp) * 5 - 1);
            }
      }
}