#ifndef TIMER_H_
#define TIMER_H_

#include "main.h"

typedef struct {
      uint32_t start_time;
} Timer;

static inline void Timer_Init(Timer *timer) {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void Timer_Reset(Timer *timer) {
      timer->start_time = DWT->CYCCNT;
}

static inline float Timer_Read(Timer *timer) {
      return (float)(DWT->CYCCNT - timer->start_time) / HAL_RCC_GetSysClockFreq();
}

static inline uint32_t Timer_ReadMs(Timer *timer) {
      return (uint32_t)(((float)(DWT->CYCCNT - timer->start_time) / HAL_RCC_GetSysClockFreq()) * 1000.0f);
}

static inline uint32_t Timer_ReadUs(Timer *timer) {
      return (uint32_t)(((float)(DWT->CYCCNT - timer->start_time) / HAL_RCC_GetSysClockFreq()) * 1000000.0f);
}

static inline void WaitUs(uint32_t micros) {
      uint32_t startTick = DWT->CYCCNT;
      uint32_t requiredTicks = micros * (SystemCoreClock / 1000000);
      while ((DWT->CYCCNT - startTick) < requiredTicks) {
      }
}

static inline void WaitMs(uint32_t millis) {
      uint32_t startTick = DWT->CYCCNT;
      uint32_t requiredTicks = millis * (SystemCoreClock / 1000);
      while ((DWT->CYCCNT - startTick) < requiredTicks) {
      }
}

static inline void Wait(uint32_t seconds) {
      uint32_t startTick = DWT->CYCCNT;
      uint32_t requiredTicks = seconds * SystemCoreClock;
      while ((DWT->CYCCNT - startTick) < requiredTicks) {
      }
}

#endif  // TIMER_H_