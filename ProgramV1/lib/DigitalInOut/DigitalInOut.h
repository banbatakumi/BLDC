#ifndef DIGITALINOUT_H_
#define DIGITALINOUT_H_

#include "main.h"

typedef struct {
      GPIO_TypeDef *port;
      uint16_t pin;
} DigitalOut;

typedef struct {
      GPIO_TypeDef *port;
      uint16_t pin;
} DigitalIn;

// Out
static inline void DigitalOut_Init(DigitalOut *obj, GPIO_TypeDef *port, uint16_t pin) {
      obj->port = port;
      obj->pin = pin;
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static inline void DigitalOut_Write(DigitalOut *obj, int value) {
      HAL_GPIO_WritePin(obj->port, obj->pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// In
static inline void DigitalIn_Init(DigitalIn *obj, GPIO_TypeDef *port, uint16_t pin) {
      obj->port = port;
      obj->pin = pin;
}

static inline int DigitalIn_Read(DigitalIn *obj) {
      return HAL_GPIO_ReadPin(obj->port, obj->pin) == GPIO_PIN_SET;
}

#endif