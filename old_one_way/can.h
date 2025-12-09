#pragma once

#include "pin_config.h"
#include <Arduino.h>
#include <CircularBuffer.hpp>

#if defined (STM32C0xx)
  #include "stm32c0xx_hal_fdcan.h"
#endif
#if defined (STM32U5xx)
  #include "stm32u5xx_hal_fdcan.h"
#endif

#define MAX_LENGTH 64

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} canRec;

extern FDCAN_HandleTypeDef hfdcan1;
extern CircularBuffer<canRec, MAX_LENGTH> rxBuf;   // LoRa -> CAN
extern CircularBuffer<canRec, MAX_LENGTH> txBuf;   // CAN  -> LoRa

void initCan();
void pollCanRx();
void processCanTx();
