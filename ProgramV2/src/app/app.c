#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
DigitalIn SW;

Timer control_timer;

SensoredVectorControl svc;

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
      HAL_Delay(100);
      PwmOut_Write(&LED1, 0);

      // BLDCの初期化
      BLDC_Init(&svc);
      if (DigitalIn_Read(&SW)) {
            while (BLDC_SetEncoder(&svc, adc_val[0]) == false);
      }
      HAL_Delay(100);
      PwmOut_Write(&LED2, 0);

      // Serialの初期化
      Serial_Init(&pc, &huart1, 256, true);
      Serial_Init(&uart2, &huart2, 1024, true);

      // ローパスフィルタの初期化
      LPF_Init(&supply_volt_lpf, 0.9, 12);  // 電圧
      LPF_Init(&temp_lpf, 0.9, 30);         // 温度

      Timer_Init(&control_timer);
      Timer_Reset(&control_timer);

      PwmOut_Write(&LED3, 0);
}

void GetSensors() {
      encoder_val = adc_val[0];      // エンコーダの値
      supply_volt_val = adc_val[1];  // 電圧の値
      temp_val = adc_val[2];         // 温度の値

      // 電源電圧の変換(分圧で1/10にしている)
      supply_volt = supply_volt_val * (3.3f / 4095.0f) * 10.0f;
      supply_volt = LPF_Update(&supply_volt_lpf, supply_volt);  // ローパスフィルタを適用

      // MCP9700T/HTT温度センサの変換
      float temp_voltage = temp_val * (3.3f / 4095.0f);  // ADC値 → 電圧変換

      temp = (temp_voltage - 0.5) / 0.01f;  // 電圧 → 温度変換
      temp = LPF_Update(&temp_lpf, temp);   // ローパスフィルタを適用

      // スイッチ
      sw_state = DigitalIn_Read(&SW);
}

void MainApp() {
      while (1) {
            GetSensors();
            if (temp > TEMP_LIMIT || is_overheat == true) {
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
                  float target_rad = 0;
                  if (Serial_Available(&uart2)) {
                        target_rad = Serial_Read(&uart2) * (TWO_PI / 255.0f);  // 0〜255の値を0〜2πのラジアンに変換
                  }
                  // printf("Target rad: %.6f, Supply volt: %.2fV, Temp: %.2f°C\n", target_rad, supply_volt, temp);
                  // uint8_t rad = (svc.mech_theta + svc.encoder_offset_theta) * (255.0f / TWO_PI);  // ラジアンを0〜255の値に変換
                  // Serial_Write(&uart2, (uint8_t *)&rad, 1);                                       // シリアルに送信

                  // BLDC_SpeedControl(&svc, 50);  // 速度制御
                  // BLDC_PositionControl(&svc, svc.mech_theta + svc.encoder_offset_theta);  // 位置制御
                  BLDC_PositionControl(&svc, target_rad);  // 速度制御

                  BLDC_SensoredVectorControlDrive(&svc, encoder_val, supply_volt);

                  // 状態の表示
                  PwmOut_Write(&LED1, Abs(svc.amp) * 5);
                  PwmOut_Write(&LED2, Abs(svc.amp) * 5 - 1);
                  PwmOut_Write(&LED3, Abs(svc.amp) * 5 - 2);

                  // 制御周期の一定化
                  // while (Timer_Read(&control_timer) <= CONTROL_PERIOD);
                  // Timer_Reset(&control_timer);
            }
      }
}