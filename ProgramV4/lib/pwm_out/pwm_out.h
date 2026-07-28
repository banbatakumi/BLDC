#ifndef PWM_H_
#define PWM_H_

#include "mymath.h"
#include "tim.h"

typedef struct {
      TIM_HandleTypeDef *htim;
      uint32_t channel;
      uint32_t maxValue;
      int usePwmPin_t;
} PwmOut;

static inline void PwmOut_Init(PwmOut *obj, TIM_HandleTypeDef *htim, uint32_t channel) {
      obj->htim = htim;
      obj->channel = channel;
      obj->usePwmPin_t = 0;

      HAL_TIM_PWM_Start(obj->htim, obj->channel);
      HAL_TIMEx_PWMN_Start(obj->htim, obj->channel);
      obj->maxValue = obj->htim->Init.Period;
}

static inline void PwmOut_Write(PwmOut *obj, float duty) {
      int val = (int)(Constrain(duty, 0.0f, 1.0f) * obj->maxValue);
      __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, val);
}

#endif