/*
CAN layer

Used for low level control of the CAN bus and handles link buffers

Buffers:
- rxBuf: stores data received by the radio -> to be transmitted with CAN
- txBuf: stores data received by CAN -> to be transmitted with radio

Filter:
- Accept all id's
*/

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

#define MAX_LENGTH 32 // # of can records

typedef struct __attribute__((packed)){
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} canRec;

extern FDCAN_HandleTypeDef hfdcan1;
extern CircularBuffer<canRec, MAX_LENGTH> rxBuf;   // radio -> CAN
extern CircularBuffer<canRec, MAX_LENGTH> txBuf;   // CAN  -> radio

void initCan();
void pollCanRx();
void processCanTx();
