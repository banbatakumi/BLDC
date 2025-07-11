#include "app.h"

PwmOut LED1;
PwmOut LED2;
PwmOut LED3;
PwmOut LED4;
DigitalIn SW;

Timer control_timer;

SensoredVectorControl svc;

Serial pc;

uint16_t adc_val[3];  // ADCの値を格納する配列

uint16_t encoder_val, supply_volt_val, temp_val;
float supply_volt;
float temp;

void Setup() {
      printf("Hello World\n");
      printf("SystemCoreClock = %ld\n", SystemCoreClock);

      // ADCの初期化
      HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&adc_val, 3);
      for (uint8_t i = 0; i < 3; i++) {
            while (!(adc_val[i] > 0));  // ADCの値が代入されるまで待つ
      }
      printf("ADC_DMA start\n");
      HAL_Delay(500);

      // LEDの初期化
      PwmOut_Init(&LED1, &htim2, TIM_CHANNEL_1);
      PwmOut_Init(&LED2, &htim2, TIM_CHANNEL_2);
      PwmOut_Init(&LED3, &htim3, TIM_CHANNEL_1);
      PwmOut_Init(&LED4, &htim3, TIM_CHANNEL_2);

      DigitalIn_Init(&SW, GPIOA, GPIO_PIN_0);

      // BLDCの初期化
      BLDC_Init(&svc);
      // while (BLDC_SetEncoder(&svc, adc_val[0]) == false);

      // Serialの初期化
      Serial_Init(&pc, &huart1, 256, true);

      Timer_Init(&control_timer);
      Timer_Reset(&control_timer);
}

void GetSensors() {
      encoder_val = adc_val[0];      // エンコーダの値
      supply_volt_val = adc_val[1];  // 電圧の値
      temp_val = adc_val[2];         // 温度の値

      // 電源電圧の変換(分圧で1/10にしている)
      supply_volt = supply_volt_val * (3.3f / 4095.0f) * 10.0f;

      // MCP9700T/HTT温度センサの変換
      float temp_voltage = temp_val * (3.3f / 4095.0f);  // ADC値 → 電圧変換

      temp = (temp_voltage - 0.5) / 0.01f;  // 電圧 → 温度変換
}

void MainApp() {
      while (1) {
            if (temp > TEMP_LIMIT) {
                  printf("Overheat! Temperature: %.2f°C\n", temp);
                  BLDC_OpenLoopDrive(0, 0);  // モーター停止
            } else {
                  static uint8_t speed = 0;
                  if (Serial_Available(&pc)) {
                        speed = Serial_Read(&pc);
                  }
                  BLDC_SpeedControl(&svc, 50);  // 速度制御
                  // BLDC_PositionControl(&svc, speed);  // 位置制御

                  BLDC_SensoredVectorControlDrive(&svc, encoder_val);

                  // 制御周期の一定化
                  while (Timer_Read(&control_timer) <= CONTROL_PERIOD);
                  Timer_Reset(&control_timer);
            }
      }
}