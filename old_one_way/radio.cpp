#include "radio.h"

#include <string.h>

SX1280 radio = new Module(SX_CS, SX_DIO1, SX_RESET, SX_BUSY);

// fix for stm32u5 later

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

// Batching window configuration
static const uint32_t BATCH_WINDOW_MS = 20;  // Wait up to 20ms to accumulate frames
static const uint8_t MIN_BATCH_SIZE = 3;     // Or transmit when we have 3+ frames
static uint32_t lastBatchStart = 0;

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

void configRadio() {
  /* configures modulation parameters and radio settings */
  radio.setBitRate(325);
  radio.setCodingRate(3);
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

  Serial.printf("[SX1280] Start RX (%d)\n", state);
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

  digitalWrite(LED_RX, HIGH);
  int state = radio.readData(buf, len);
  if (state == RADIOLIB_ERR_NONE) {

    Serial.printf("[SX1280] Received payload of length %d\n", len);
    // push received payload to rxBuf
    size_t offset = 0;
    // Serial.println("[DEBUG] Records in payload:");
    while (offset + sizeof(canRec) <= (size_t)len) {
      canRec rec;
      memcpy(&rec, &buf[offset], sizeof(rec));
      // Serial.printf("\tID: 0x%x, DLC: %d\n", rec.id, rec.dlc);
      offset += sizeof(rec);

      rxBuf.push(rec);
    }
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[SX1280] CRC Error");
  } else {
    Serial.printf("[SX1280] Reading received data failed (%d)\n", state);
  }

  digitalWrite(LED_RX, LOW);

  startRx();
}

void radioTransmit() {
  if (txBuf.isEmpty()) {
    return;
  }

  uint32_t now = millis();
  size_t txBufSize = txBuf.size();

  // Start batching window when first frame arrives
  if (lastBatchStart == 0) {
    lastBatchStart = now;
  }

  uint32_t elapsed = now - lastBatchStart;

  // Transmit if:
  // 1. Batch window has elapsed, OR
  // 2. We have accumulated enough frames, OR
  // 3. Buffer is getting full (>50%)
  bool shouldTransmit = (elapsed >= BATCH_WINDOW_MS) ||
                        (txBufSize >= MIN_BATCH_SIZE) ||
                        (txBufSize > (MAX_LENGTH / 2));

  if (!shouldTransmit) {
    return;  // Wait for more frames
  }

  // Atomically check and set radioBusy to prevent race condition
  noInterrupts();
  if (radioBusy) {
    interrupts();
    return;  // already transmitting
  }
  radioBusy = true;  // Set flag before enabling interrupts
  interrupts();

  // Reset batch timer
  lastBatchStart = 0;

  uint8_t payload[MAX_PAYLOAD_LENGTH];
  size_t  len = 0;

  // fill payload with CAN records from txBuf up to 255 bytes
  while (!txBuf.isEmpty() && (len + sizeof(canRec)) < MAX_PAYLOAD_LENGTH) {
    canRec rec = txBuf.shift();
    memcpy(&payload[len], &rec, sizeof(rec));
    len += sizeof(rec);
  }

  if (len == 0) {
    radioBusy = false;  // Clear flag if no data to send
    return;
  }

  digitalWrite(LED_TX, HIGH);

  Serial.printf("[SX1280] Transmitting payload of length %d\n", len);
  int state = radio.startTransmit(payload, len);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[SX1280] Transmit failed (%d)\n", state);
    radioBusy = false;  // Clear flag on failure
    // Back to RX on failure
    startRx();
  }
  // Note: radioBusy will be cleared by TX_DONE interrupt on success

  digitalWrite(LED_TX, LOW);
}

void handleRadioIrq() {
  if (!radioFlag) {
    return;
  }

  radioFlag = false;

  uint32_t irqStatus = radio.getIrqStatus();

  if (irqStatus & RADIOLIB_SX128X_IRQ_RX_DONE){
    handleRadioRx();
  }

  if (irqStatus & RADIOLIB_SX128X_IRQ_TX_DONE){
    radioBusy = false;
    startRx();
  }
}
