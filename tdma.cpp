#include <stdint.h>
#include "tdma.h"
#include "can.h"
#include "radio.h"
#include <Arduino.h>

#ifndef TDMA_ENABLE_DEBUG
#define TDMA_ENABLE_DEBUG 0
#endif

#define TDMA_LOGF(...) do { if (TDMA_ENABLE_DEBUG) Serial.printf(__VA_ARGS__); } while (0)

static struct tdmaState state;
static void tdmaEnterSlot(SlotId next_slot);
static void tdmaTransmit();

struct SlotWindow {
  SlotId id;
  uint32_t startUs;
  uint32_t endUs;
};

static const SlotWindow kFrameSlots[] = {
    {GUARD, 0, GUARD_TIME_US},
    {DOWNLINK, GUARD_TIME_US, GUARD_TIME_US + DOWNLINK_TIME_US},
    {GUARD, GUARD_TIME_US + DOWNLINK_TIME_US,
     GUARD_TIME_US + DOWNLINK_TIME_US + GUARD_TIME_US},
    {UPLINK, GUARD_TIME_US + DOWNLINK_TIME_US + GUARD_TIME_US, FRAME_LEN_US},
};

void tdmaInit(TdmaRole role) {
  state.role = role;
  state.currentSlot = GUARD;
  state.frameSeq = 0;
  state.clockOffsetUs = 0;

  Serial.printf("[TDMA] Initializing %s\n",
                (role == TDMA_MASTER) ? "MASTER" : "FOLLOWER");

  if (role == TDMA_MASTER) {
    state.frameStartUs = micros();
    state.synced = true;
    radioIdle();  // Start in idle, will enter DOWNLINK soon
  } else {
    state.frameStartUs = 0;
    state.synced = false;
    state.lastSyncUs = micros();
    startRx();  // Follower starts listening immediately
  }
}

void tdmaUpdate() {
  int64_t now = (int64_t)micros() + (int64_t)state.clockOffsetUs;
  int64_t elapsed = now - (int64_t)state.frameStartUs;

  if (elapsed < 0) {
    elapsed = 0;
  }

  // Follower sync timeout check
  if (state.role == TDMA_FOLLOWER && state.synced) {
    if (micros() - state.lastSyncUs >= FRAME_LEN_US * 10) {
      Serial.println("[TDMA] Lost sync");
      state.synced = false;
      state.clockOffsetUs = 0;
      state.frameStartUs = micros();
      startRx();
      return;
    }
  }

  // Handle frame rollover
  if (elapsed >= FRAME_LEN_US) {
    state.frameStartUs += FRAME_LEN_US;
    state.frameSeq++;
    elapsed = now - (int64_t)state.frameStartUs;
    tdmaEnterSlot(GUARD);
  }

  // Find current slot
  SlotId slot = state.currentSlot;
  for (const SlotWindow &frameSlot : kFrameSlots) {
    if (elapsed >= frameSlot.startUs && elapsed < frameSlot.endUs) {
      slot = frameSlot.id;
      break;
    }
  }

  if (slot != state.currentSlot) {
    tdmaEnterSlot(slot);
  }
}

static void tdmaBuildHeader(struct tdmaHeader &header, uint8_t num_records) {
  header.slot_id = state.currentSlot;
  header.frame_seq = state.frameSeq;
  header.epoch_us = state.frameStartUs;
  header.num_records = num_records;
}

static void tdmaEnterSlot(SlotId next_slot) {
  state.currentSlot = next_slot;

  switch (next_slot) {
  case GUARD:
    // Do nothing - radio is already in correct mode
    // Master: in RX from previous UPLINK
    // Follower: stays in RX
    break;

  case DOWNLINK:
    if (state.role == TDMA_MASTER) {
      tdmaTransmit();  // startTransmit() handles mode switch
    } else {
      startRx();  // Follower listens for master
    }
    break;

  case UPLINK:
    if (state.role == TDMA_FOLLOWER) {
      if (state.synced && !txBuf.isEmpty()) {
        tdmaTransmit();
      }
      // else stay in RX (already there)
    }
    // Master is already in RX from TX_DONE, do nothing
    break;
  }
}

static bool processHeader(const uint8_t *buf, uint32_t rx_time){
  tdmaHeader h;
  
  memcpy(&h, buf, sizeof(tdmaHeader));

  if (h.slot_id != DOWNLINK && h.slot_id != UPLINK && h.slot_id != GUARD) {
    TDMA_LOGF("[TDMA] Invalid slot ID\n");
    return false;
  }

  TDMA_LOGF("[TDMA] RX header: slot=%d frame=%d epoch=%lu records=%d\n",
              h.slot_id, h.frame_seq, h.epoch_us, h.num_records);


  if (h.slot_id == DOWNLINK) {
    uint32_t rx_est = rx_time;
    uint32_t downlink_mid = GUARD_TIME_US + (DOWNLINK_TIME_US / 2);
    if (rx_est > downlink_mid) {
      rx_est -= downlink_mid;
    }
    state.clockOffsetUs = (int32_t)(h.epoch_us - rx_est);
    state.frameSeq = h.frame_seq;
    state.frameStartUs = h.epoch_us;
    state.synced = true;
    state.lastSyncUs = rx_time;
  }

  return true;
}

void tdmaProcessRx(const uint8_t *buf, size_t len, uint32_t rx_time) {
  size_t offset = 0;

  if (state.role == TDMA_FOLLOWER) {
    if (len < sizeof(tdmaHeader)) {
      return;
    } 
    if (!processHeader(buf, rx_time)) {
      return;
    }
    offset = sizeof(tdmaHeader);
  }

  // Extract CAN records
  while (offset + sizeof(canRec) <= len) {
    canRec rec;
    memcpy(&rec, &buf[offset], sizeof(rec));
    offset += sizeof(rec);
    rxBuf.push(rec);
    TDMA_LOGF("  RX CAN id=0x%lx dlc=%u\n", rec.id, rec.dlc);
  }
  
  // Serial.printf("[SX1280] RX len=%d\n", len);
  // Serial.printf("[SX1280] RX HEX: ");
  // for (int i = 0; i < len; i++) {
  //   Serial.printf("0x%x ", buf[i]);
  // }
  // Serial.println();
}

static void tdmaTransmit() {
  size_t offset = 0;
  uint8_t num_records = 0;

  // Select payload limit and max records based on role
  const size_t max_payload = (state.role == TDMA_MASTER) ? MASTER_PAYLOAD_LEN : FOLLOWER_PAYLOAD_LEN;
  const uint8_t max_records = (state.role == TDMA_MASTER) ? MASTER_MAX_CAN_RECORDS : FOLLOWER_MAX_CAN_RECORDS;

  uint8_t payload[max_payload];

  // Master builds header first
  if (state.role == TDMA_MASTER) {
    offset = sizeof(tdmaHeader);
  }

  // Pack CAN records up to role-specific limit
  while (!txBuf.isEmpty() && num_records < max_records && (offset + sizeof(canRec)) <= max_payload) {
    canRec rec = txBuf.shift();
    memcpy(&payload[offset], &rec, sizeof(rec));
    offset += sizeof(rec);
    num_records++;
  }

  // Write header for master
  if (state.role == TDMA_MASTER) {
    tdmaHeader header;
    tdmaBuildHeader(header, num_records);
    memcpy(&payload[0], &header, sizeof(header));
    TDMA_LOGF("[TDMA] TX DOWNLINK: frame=%u records=%u\n", header.frame_seq, num_records);
  } else {
    TDMA_LOGF("[TDMA] TX UPLINK: records=%u\n", num_records);
  }

  radioTransmit(payload, offset);
}

bool tdmaIsSynced() {
  return state.synced;
}
