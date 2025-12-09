#include "pin_config.h"
#include "can.h"
#include "radio.h"

uint32_t lastTransmit = 0;

const canRec testFrames[] = {
  {
    .id = 0x720,
    .dlc = 8,
    .data = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77},
  },
  {
    .id = 0x321,
    .dlc = 6,
    .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
  },
  {
    .id = 0x123,
    .dlc = 2,
    .data = {0x10, 0x20},
  }
};

const size_t TEST_FRAME_COUNT = sizeof(testFrames) / sizeof(testFrames[0]);
const uint32_t TEST_FRAME_INTERVAL_MS = 1000;

uint32_t lastTestFrameEnqueue = 0;

void enqueueTestFrames() {
  uint32_t now = millis();
  // if (now - lastTestFrameEnqueue < TEST_FRAME_INTERVAL_MS) {
  //   return;
  // }

  lastTestFrameEnqueue = now;

  for (size_t i = 0; i < TEST_FRAME_COUNT; i++) {
    txBuf.push(testFrames[i]);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);

  initCan();
  initRadio();
  startRx();

  // enqueueTestFrames();
  delay(500);
}

void loop() {

  pollCanRx();
  handleRadioIrq();

  if (!txBuf.isEmpty()) {
    radioTransmit();
  }

  processCanTx();
}
