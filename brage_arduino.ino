#include "pin_config.h"
#include "can.h"
#include "radio.h"
#include "tdma.h"

// #define ROLE TDMA_MASTER
#define ROLE TDMA_FOLLOWER

canRec testUplinkData = {.id = 0x720, .dlc = 8, .data = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}};

int lastTransmit = 0;

void setup() {
  Serial.begin(230400);

  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);

  initCan();
  initRadio();
  configRadio();
  tdmaInit(ROLE);

  delay(500);
}


void loop() {

  if (millis() - lastTransmit >= 1000) {
    txBuf.push(testUplinkData);
    lastTransmit = millis();
  }

  pollCanRx();
  handleRadioIrq();

  if (tdmaIsSynced()) {
    tdmaUpdate();
  }

  processCanTx();
}
