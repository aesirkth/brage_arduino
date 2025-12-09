#pragma once

#include "pin_config.h"
#include <Arduino.h>
#include <RadioLib.h>

#include "can.h"

#define MAX_PAYLOAD_LENGTH  RADIOLIB_SX128X_MAX_PACKET_LENGTH

extern volatile bool radioFlag;

void initRadio();
void configRadio();
void startRx();
void handleRadioIrq();
void radioTransmit();
