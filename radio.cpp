#include <Arduino.h>
#include "radio.h"
#include "can.h"
#include "tdma.h"
#include "pin_config.h"

SX1280 radio = new Module(SX_CS, SX_DIO1, SX_RESET, SX_BUSY);

static const uint32_t rfswitch_pins[] = {PA_EN, RX_EN, TX_EN, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {Module::MODE_IDLE, {LOW, LOW, LOW}},
  {Module::MODE_RX,   {LOW, HIGH, LOW}},
  {Module::MODE_TX,   {HIGH, LOW, HIGH}},
  Module::MODE_END_OF_TABLE,
};

volatile bool radioFlag;
static volatile bool radioBusy = false;
static volatile uint32_t lastRxDoneUs = 0;

static void setFlag() {
  radioFlag = true;
}

void initRadio() {
  Serial.println("[SX1280] Initializing...");

  // int state = radio.beginFLRC();
  int state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Init failed: %d\n", state);
    while (true) {
      digitalWrite(LED_RX, HIGH);
      delay(100);
      digitalWrite(LED_RX, LOW);
      delay(100);
    }
  }

  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  radio.setDio1Action(setFlag);
  Serial.println("[SX1280] Initialized");
}

void configRadio() {

  radio.setSpreadingFactor(6);
  radio.setBandwidth(812.5);
  radio.setCodingRate(5);
  radio.setPreambleLength(8);
  radio.setOutputPower(13);

  radio.variablePacketLengthMode(MAX_PAYLOAD_LENGTH);
  radio.setCRC(2);
}

void startRx() {
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Start RX failed: %d\n", state);
  }
}

static void handleRadioRx() {
  uint8_t buf[MAX_PAYLOAD_LENGTH] = {0};
  uint32_t rx_time_us = lastRxDoneUs;

  int16_t len = radio.getPacketLength();
  if (len < 0) {
    Serial.printf("[SX1280] getPacketLength failed: %d\n", len);
    return;
  }

  if (len > MAX_PAYLOAD_LENGTH) {
    len = MAX_PAYLOAD_LENGTH;
  }
  
  int state = radio.readData(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    // Adjust RX timestamp to packet midpoint
    uint32_t toa_us = (uint32_t)radio.getTimeOnAir(len);
    uint32_t midpoint_offset = toa_us / 2;
    if (midpoint_offset < rx_time_us) {
      rx_time_us -= midpoint_offset;
    }

    tdmaProcessRx(buf, (size_t)len, rx_time_us);
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[SX1280] CRC error");
  } else {
    Serial.printf("[SX1280] RX error: %d\n", state);
  }

  radio.finishReceive();
  startRx();
}

void radioTransmit(const uint8_t *buf, size_t len) {
  noInterrupts();
  if (radioBusy) {
    interrupts();
    Serial.println("[SX1280] TX blocked: already transmitting");
    return;
  }

  int state = radio.startTransmit(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    radioBusy = true;
    interrupts();

    // Serial.printf("[SX1280] TX len=%u\n", (unsigned int)len);
    // Serial.printf("[SX1280] TX HEX: ");
    // for (size_t i = 0; i < len; i++) {
    //   Serial.printf("0x%x ", buf[i]);
    // }
    // Serial.println();
  } else {
    interrupts();
    Serial.printf("[SX1280] TX failed: %d\n", state);
    startRx();
  }
}

void handleRadioIrq() {
  noInterrupts();
  bool flag = radioFlag;
  radioFlag = false;
  interrupts();

  if (!flag) {
    return;
  }

  uint32_t irqStatus = radio.getIrqStatus();

  if (irqStatus & RADIOLIB_SX128X_IRQ_RX_DONE) {
    lastRxDoneUs = micros();
    handleRadioRx();
  }

  if (irqStatus & RADIOLIB_SX128X_IRQ_TX_DONE) {
    radio.finishTransmit();
    radioBusy = false;
    // Serial.println("[SX1280] TX done");
    startRx();
  }

  if (irqStatus & RADIOLIB_SX128X_IRQ_RX_TX_TIMEOUT) {
    if (radioBusy) {
      radio.finishTransmit();
      radioBusy = false;
    } else {
      radio.finishReceive();
    }
    startRx();
  }
}

void radioIdle() {
  radioBusy = false;
  radio.standby();
}
