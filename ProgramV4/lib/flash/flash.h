#ifndef FLASH_H_
#define FLASH_H_

#include <string.h>

#include "stm32f3xx_hal.h"

#define FLASH_USER_START_ADDR ((uint32_t)0x0800F800)
#define FLASH_USER_END_ADDR ((uint32_t)0x0800FFFF)

static inline HAL_StatusTypeDef Flash_WriteData(uint32_t address, const void *data, size_t size) {
      if (address < FLASH_USER_START_ADDR || (address + size) > FLASH_USER_END_ADDR) {
            return HAL_ERROR;
      }

      HAL_FLASH_Unlock();

      // ページ消去
      FLASH_EraseInitTypeDef eraseInit;
      uint32_t pageError = 0;
      eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
      eraseInit.PageAddress = FLASH_USER_START_ADDR;
      eraseInit.NbPages = 1;

      if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
      }

      // 4バイトずつ書き込み。size が4の倍数でない端数分は、data の先を読み過ぎない
      // よう一旦ローカルバッファへコピーしてから渡す。
      const uint8_t *src = (const uint8_t *)data;
      size_t wordCount = (size + 3) / 4;
      for (size_t i = 0; i < wordCount; i++) {
            uint32_t word;
            size_t offset = i * 4;
            size_t remain = size - offset;
            if (remain >= 4) {
                  memcpy(&word, src + offset, 4);
            } else {
                  uint8_t tail[4] = {0};
                  memcpy(tail, src + offset, remain);
                  memcpy(&word, tail, 4);
            }
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word) != HAL_OK) {
                  HAL_FLASH_Lock();
                  return HAL_ERROR;
            }
      }

      HAL_FLASH_Lock();
      return HAL_OK;
}

static inline void Flash_ReadData(uint32_t address, void *buffer, size_t size) {
      uint32_t *p = (uint32_t *)buffer;
      for (size_t i = 0; i < (size + 3) / 4; i++) {
            p[i] = *(volatile uint32_t *)(address + i * 4);
      }
}

#endif  // FLASH_H_