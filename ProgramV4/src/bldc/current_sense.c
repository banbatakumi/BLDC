#include "current_sense.h"

#include <stdio.h>

#include "adc.h"
#include "main.h"
#include "mymath.h"
#include "tim.h"

// ADC値 → 電流 [A] : I = (Vout - REF) / (Rshunt * Gain)。CURRENT_SIGN は配線上の向き (config.h 参照)
#define ADC2CURRENT_ABS (ADC2VOLT / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN))
#define ADC2CURRENT (CURRENT_SIGN * ADC2CURRENT_ABS)

// DMAの転送先。[0] = ADC1_IN1(V相), [1] = ADC1_IN2(U相)
static volatile uint16_t adc_val[2];

static float offset_v = CURRENT_REF_ADC;
static float offset_u = CURRENT_REF_ADC;

// 電流センシングADC1をPWM同期トリガで初期化する。
// ローサイドシャントは低側FETがONの区間しか電流が流れない。TIM1はセンター揃えなので
// 低側ON区間はカウンタ頂点 (CNT=PWM_ARR) を中心に広がり、頂点付近でサンプリングすると
// その周期の平均電流が取れる。
// TIM1_CH4 を PWM2 モードにして CCR4 = PWM_ARR - CURRENT_SENSE_TRIG_ADVANCE に置くと、
// 上りの通過時だけ OC4REF が立ち上がる (1周期に1回)。これを TRGO2 → ADC1外部トリガにする。
// この変換完了割り込みが20kHz電流制御ループのタイミングになる。
void CurrentSense_Init(void) {
  // TIM1_CH4 コンペア設定 (OC4REF生成用。出力ピンは無いが内部REFは生成される)
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM2;  // CNT>=CCR4 でOC4REF=High(立ち上がりでトリガ)
  oc.Pulse = PWM_ARR - CURRENT_SENSE_TRIG_ADVANCE;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK) {
    printf("TIM1 CH4 config failed!\n");
  }
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // TRGO2 = OC4REF
  TIM_MasterConfigTypeDef mc = {0};
  mc.MasterOutputTrigger = TIM_TRGO_RESET;
  mc.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
  mc.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &mc);

  // ADC1 を外部トリガ(TIM1_TRGO2立ち上がり)・単発変換に再設定
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    printf("ADC1 re-init failed!\n");
  }
  // 変換チャンネル列 (IN1=SENSEB(V相) rank1, IN2=SENSEC(U相) rank2)
  ADC_ChannelConfTypeDef sc = {0};
  sc.SingleDiff = ADC_SINGLE_ENDED;
  sc.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sc.OffsetNumber = ADC_OFFSET_NONE;
  sc.Channel = ADC_CHANNEL_1;
  sc.Rank = ADC_REGULAR_RANK_1;
  HAL_ADC_ConfigChannel(&hadc1, &sc);
  sc.Channel = ADC_CHANNEL_2;
  sc.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc1, &sc);

  // セルフキャリブレーション
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // 制御ループの準備が整うまで割り込みは止めておく
  CurrentSense_DisableInterrupt();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 2) != HAL_OK) {
    printf("ADC1 DMA start failed!\n");
  }

  // ハーフ転送完了(HT)割り込みを止める。2変換なので1変換目でHTが立ち、空のコールバックのために
  // 毎周期 約1µs (CPU 2%) を使い、制御ループの起動も遅らせていた。TC側には影響しない。
  // (hdma_adc1 は adc.h で公開されていないので hadc1.DMA_Handle 経由で触る)
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);

  // ADC2(エンコーダ・電圧・温度)は連続変換で完了割り込みが高頻度に入る。値はDMAが更新するので止める
  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);

  // 制御ループ(ADC1のDMA完了)を最優先にし、SysTickとUARTのDMAは下げてジッタを抑える
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);  // USART1_TX
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);  // USART1_RX
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);  // USART2_RX
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 2, 0);  // USART2_TX
}

void CurrentSense_EnableInterrupt(void) {
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void CurrentSense_DisableInterrupt(void) {
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}

void CurrentSense_Calibrate(void) {
  const uint16_t CALIB_SAMPLES = 500;
  uint32_t v_sum = 0, u_sum = 0;
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    v_sum += adc_val[0];
    u_sum += adc_val[1];
    HAL_Delay(1);
  }
  offset_v = (float)v_sum / CALIB_SAMPLES;
  offset_u = (float)u_sum / CALIB_SAMPLES;

  float dev_u = offset_u - CURRENT_REF_ADC;
  float dev_v = offset_v - CURRENT_REF_ADC;
  bool offset_valid =
      (Abs(dev_u) <= CURRENT_OFFSET_TOLERANCE) && (Abs(dev_v) <= CURRENT_OFFSET_TOLERANCE);

  printf("CurrentSense: offset Iu:%.1f Iv:%.1f [ADC] (中点 %.0f), %.4f A/LSB, レンジ 約±%.1f A\n",
         offset_u, offset_v, (double)CURRENT_REF_ADC,
         (double)ADC2CURRENT_ABS, (double)(CURRENT_REF_ADC * ADC2CURRENT_ABS));
  if (!offset_valid) {
    printf("CurrentSense: 警告 オフセットが中点から大きくずれています (Iu:%+.0f Iv:%+.0f LSB)\n",
           dev_u, dev_v);
  }
}

void CurrentSense_Read(float* iu, float* iv, float* iw) {
  *iv = ((float)adc_val[0] - offset_v) * ADC2CURRENT;  // PA0 = SENSEB = OUTB
  *iu = ((float)adc_val[1] - offset_u) * ADC2CURRENT;  // PA1 = SENSEC = OUTC
  *iw = -(*iu + *iv);                                  // キルヒホッフの電流則より
}

void CurrentSense_GetRaw(uint16_t* raw_u, uint16_t* raw_v) {
  *raw_v = adc_val[0];
  *raw_u = adc_val[1];
}
