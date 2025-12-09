#include "WSerial.h"
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
  {Module::MODE_RX,   {LOW, HIGH, LOW}},
  {Module::MODE_TX,   {HIGH, LOW, HIGH}},
  Module::MODE_END_OF_TABLE,
};

volatile bool radioFlag;
static volatile bool radioBusy = false;
static volatile uint32_t radioBusySinceUs = 0;
static volatile uint32_t radioTxDeadlineUs = 0;
static volatile uint32_t lastRxDoneUs = 0; // captured at IRQ

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
  radioBusy = false;
  radioBusySinceUs = 0;
  radioTxDeadlineUs = 0;
  radio.standby();
  radio.clearIrqFlags(0xFFFF);
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
  uint32_t rx_time_us = lastRxDoneUs; // RX_DONE time captured in IRQ (end of packet)

  // float rssi = radio.getRSSI();
  // float snr = radio.getSNR();
  // Serial.print("[SX1280] RSSI: ");
  // Serial.print(rssi);
  // Serial.print(", SNR: ");
  // Serial.println(snr);

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

    // Adjust timestamp toward packet mid-air to reduce offset error
    uint32_t toa_us = (uint32_t)(radio.getTimeOnAir(len) * 1000.0f); // getTimeOnAir returns ms
    if (toa_us / 2 < rx_time_us) {
      rx_time_us -= toa_us / 2;
    }
    Serial.printf("[SX1280] RX len = %d\n", len);
    Serial.printf("[SX1280] RX HEX DUMP: ");
    for (int i = 0; i < len; i++){
      Serial.printf("0x%x ", buf[i]);
    }
    Serial.println();
    tdmaProcessRx(buf, (size_t)len, rx_time_us); // forward to tdma protocol
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // drop silently
  } else {
    // drop silently
  }

  startRx();
}

void radioTransmit(const uint8_t *buf, size_t len) {
  uint32_t now = micros();

  noInterrupts();
  if (radioBusy) {
    if (radioTxDeadlineUs != 0 && (int32_t)(now - radioTxDeadlineUs) >= 0) {
      // Clear any stale IRQ state and reset busy tracking.
      radio.finishTransmit();
      radioBusy = false;
      radioBusySinceUs = 0;
      radioTxDeadlineUs = 0;
      interrupts();
      Serial.println("[SX1280] TX busy cleared (deadline)");
    } else {
      interrupts();
      Serial.printf("[SX1280] TX blocked: busy (age=%lu us)\n", (unsigned long)(now - radioBusySinceUs));
      return;  // already transmitting
    }
  } else {
    interrupts();
  }

  digitalWrite(LED_TX, HIGH);

  noInterrupts();
  int state = radio.startTransmit(buf, len);
  if (state != RADIOLIB_ERR_NONE) {
    interrupts();
    Serial.printf("[SX1280] Transmit failed (%d)\n", state);
    startRx(); // return to rx on failure
  } else {
    radioBusy = true;
    radioBusySinceUs = micros();
    float toa_ms = radio.getTimeOnAir(len); // returns ms
    uint32_t toa_us = (uint32_t)(toa_ms * 1000.0f);
    radioTxDeadlineUs = radioBusySinceUs + toa_us + 5000; // guard margin
    interrupts();
    Serial.printf("[SX1280] TX start len=%u\n", (unsigned int)len);
    Serial.printf("[SX1280] TX HEX DUMP: ");
    for (int i = 0; i < len; i++){
      Serial.printf("0x%x ", buf[i]);
    }
    Serial.println();
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
    lastRxDoneUs = micros();
    handleRadioRx();
  }

  if (irqStatus & RADIOLIB_SX128X_IRQ_TX_DONE){
    radio.finishTransmit(); // clears IRQ status and returns to standby
    radioBusy = false;
    radioBusySinceUs = 0;
    radioTxDeadlineUs = 0;
    Serial.println("[SX1280] TX done");
    radio.standby();
    delayMicroseconds(1000); // add small delay to prevent hearing own transmission
    startRx();
  }

  if (irqStatus & RADIOLIB_SX128X_IRQ_RX_TX_TIMEOUT) {
    radio.finishTransmit();
    radioBusy = false;
    radioBusySinceUs = 0;
    radioTxDeadlineUs = 0;
    Serial.println("[SX1280] TX timeout");
    startRx();
  }

}

void radioIdle() {
  radioBusy = false;
  radioBusySinceUs = 0;
  radioTxDeadlineUs = 0;

  if (radio.standby() != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Failed to enter idle mode");
  }
}
