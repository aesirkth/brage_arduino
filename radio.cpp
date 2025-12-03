#include <Arduino.h>

#include "radio.h"
#include "can.h"
#include "tdma.h"
#include "pin_config.h"

#define FLRC_BIT_RATE 325 // 325 kb/s
#define FLRC_CR 3 // 3/4 coding rate

SX1280 radio = new Module(SX_CS, SX_DIO1, SX_RESET, SX_BUSY);

static const uint32_t rfswitch_pins[] = {PA_EN, RX_EN, TX_EN, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] =
{
  {Module::MODE_IDLE, {LOW, LOW, LOW}},
  {Module::MODE_RX,   {HIGH, HIGH, LOW}},
  {Module::MODE_TX,   {HIGH, LOW, HIGH}},
  Module::MODE_END_OF_TABLE,
};

volatile bool radioFlag;
static volatile bool radioBusy = false;

static void setFlag() {
  radioFlag = true;
}

void initRadio() {
  Serial.println("[SX1280] Initializing ...");

  int state;
  state = radio.beginFLRC();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Success\n");
  } else {
    Serial.printf("[SX1280] Failed, code: %d\n", state);
    // should use retry instead
    while (true) {
      digitalWrite(LED_RX, HIGH);
      delay(100);
      digitalWrite(LED_RX, LOW);
      delay(100);
    }
  }

  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  radio.setDio1Action(setFlag);
}

// configure radio settings and modulation parameters
void configRadio() {
  radio.setBitRate(FLRC_BIT_RATE);
  radio.setCodingRate(FLRC_CR);
  // Add frequency config
  uint8_t syncword[] = {0x41, 0x45, 0x53, 0x52}; // AESR :)
  radio.setSyncWord(syncword, 4);
}

// set radio in rx mode
void startRx() {
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Start RX failed (%d)\n", state);
    delay(100);
    return;
  }
  // (toggle led rx) 
  // Serial.printf("[SX1280] Start RX (%d)\n", state);
}

// handle received radio packet
static void handleRadioRx() {

  uint8_t buf[MAX_PAYLOAD_LENGTH];

  int16_t len = radio.getPacketLength();
  if (len < 0) {
    Serial.printf("[SX1280] getPacketLength failed (%d)\n", len);
    startRx();
    return;
  }

  if (len > MAX_PAYLOAD_LENGTH) {
    len = MAX_PAYLOAD_LENGTH;
  }

  int state = radio.readData(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Received payload of length %d\n", len);
    tdmaProcessRx(buf, (size_t)len); // forward to tdma protocol
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[SX1280] CRC Error");
  } else {
    Serial.printf("[SX1280] Reading received data failed (%d)\n", state);
  }

  startRx();
}

void radioTransmit(const uint8_t *buf, size_t len) {
  if (radioBusy) {
    return;  // already transmitting
  }

  digitalWrite(LED_TX, HIGH);

  Serial.printf("[SX1280] Transmitting payload of length %d\n", len);
  int state = radio.startTransmit(buf, len);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Transmit failed (%d)\n", state);
    startRx(); // return to rx on failure
  } else {
    radioBusy = true;
  }

  digitalWrite(LED_TX, LOW);
}

void handleRadioIrq() {
  noInterrupts(); // avoid flag being set while this function is running
  bool flag = radioFlag;
  radioFlag = false;
  interrupts();

  if(!flag) {
    return;
  }

  uint32_t irqStatus = radio.getIrqStatus();

  if (irqStatus & RADIOLIB_SX128X_IRQ_RX_DONE){
    handleRadioRx();
  }

  if (irqStatus & RADIOLIB_SX128X_IRQ_TX_DONE){
    radioBusy = false;
    startRx();
  }
}

void radioIdle() {
  if (radio.standby() != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Failed to enter idle mode");
  }
}
