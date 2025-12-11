/*
Radio layer

Used to communicate with the RadioLib api

Configuration settings:
- Bitrate
- Coding rate

Modes:
- transmit
- receive
- idle / standby
*/

#pragma once

#include <RadioLib.h>

#define MAX_PAYLOAD_LENGTH  127

extern volatile bool radioFlag;

void initRadio();
void configRadio();     // sets modulation parameters and flrc settings
void startRx();         // puts radio in rx mode
void handleRadioIrq();  // handles dio1 interrupt (check if rx or tx irq)
void radioTransmit(const uint8_t *buf, size_t len);   // transmit whatever is in txBuf
void radioIdle();       // enter standby mode

