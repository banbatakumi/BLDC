#include "current_sense.h"

#include <stdio.h>

#include "adc.h"
#include "main.h"
#include "mymath.h"
#include "tim.h"

#define ADC2VOLT 0.0008058608059f  // ADC値 → 電圧 [V] (3.3V / 4095)

// ADC値 → 電流 [A] : Vout = REF + I * Rshunt * Gain より I = (Vout - REF) / (Rshunt * Gain)
// CURRENT_SIGN は配線上の向きを合わせるための符号 (config.h 参照)
#define ADC2CURRENT_ABS (ADC2VOLT / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN))
#define ADC2CURRENT (CURRENT_SIGN * ADC2CURRENT_ABS)

// DMAの転送先。[0] = ADC1_IN1(V相), [1] = ADC1_IN2(U相)
static volatile uint16_t adc_val[2];

static float offset_v = CURRENT_REF_ADC;
static float offset_u = CURRENT_REF_ADC;
static bool offset_valid = false;

// 電流センシングADC1をPWM同期トリガで初期化する
// ローサイドシャントは「低側FETがONの区間」しか正しい電流が流れないため、
// 低側FETがONになっているPWM周期の末尾(CNTがARRに近い区間)でADCをトリガする。
//   エッジ揃えUPカウントPWM1: CNT<CCRで高側ON、CNT>=CCRで低側ON
//   → 低側ONの区間 = [CCR, ARR]
//   TIM1_CH4をPWM2モードにしCCR4をARR手前に置くと、その位置でOC4REFが立ち上がる。
//   これをTRGO2に出し、ADC1の外部トリガ(TIM1_TRGO2)として使う。
// 平均ではなくPWM周期ごとの1点サンプリングであり、この変換完了割り込みが
// そのまま20kHzの電流制御ループのタイミングになる。
void CurrentSense_Init(void) {
  // TIM1_CH4 コンペア設定 (OC4REF生成用。CH4に出力ピンは無いが内部REFは生成される)
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM2;  // CNT>=CCR4 でOC4REF=High(立ち上がりでトリガ)
  oc.Pulse = CURRENT_SENSE_TRIG_POINT;
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
  // 変換チャンネル列を再設定 (IN1=SENSEB(V相) rank1, IN2=SENSEC(U相) rank2)
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

  // ADC2(エンコーダ・電圧・温度)は連続変換なのでDMA完了割り込みが高頻度で入る。
  // 値はDMAが勝手に更新してくれるので割り込みは不要。20kHzの制御ループを
  // 邪魔しないように止めておく。
  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);

  // 制御ループ(ADC1のDMA完了)を最優先にする。SysTickとUARTのDMAは1段下げて、
  // 制御周期のジッタを抑える。
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
  offset_valid = (Abs(dev_u) <= CURRENT_OFFSET_TOLERANCE) && (Abs(dev_v) <= CURRENT_OFFSET_TOLERANCE);

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

void CurrentSense_GetOffset(float* out_offset_u, float* out_offset_v) {
  *out_offset_u = offset_u;
  *out_offset_v = offset_v;
}

bool CurrentSense_IsOffsetValid(void) {
  return offset_valid;
}
