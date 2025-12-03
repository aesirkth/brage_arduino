#include <Arduino.h>
#include "radio.h"
#include "tdma.h"
#include "can.h"

static struct tdmaState state;

struct SlotWindow { 
  SlotId id; 
  uint32_t startUs;
  uint32_t endUs; 
};

static const SlotWindow kFrameSlots[] = {
  {GUARD, 0, GUARD_TIME_US},
  {DOWNLINK, GUARD_TIME_US, GUARD_TIME_US + DOWNLINK_TIME_US},
  {GUARD, GUARD_TIME_US + DOWNLINK_TIME_US, GUARD_TIME_US + DOWNLINK_TIME_US + GUARD_TIME_US},
  {UPLINK, GUARD_TIME_US + DOWNLINK_TIME_US + GUARD_TIME_US, FRAME_LEN_US},
};

void tdmaInit(TdmaRole role) {
  state.role = role;
  Serial.printf("[TDMA] Initializing %s\n", (role == TDMA_MASTER) ? "MASTER" : "FOLLOWER");
  if (role == TDMA_MASTER) {
    state.currentSlot = GUARD;
    state.frameSeq = 0;
    state.frameStartUs = micros();
    state.clockOffsetUs = 0;
    state.synced = true;

    tdmaEnterSlot(GUARD);

  } else if (role == TDMA_FOLLOWER) {
    state.currentSlot = GUARD;
    state.frameSeq = 0;
    state.frameStartUs = 0;
    state.clockOffsetUs = 0;
    state.synced = 0;
    state.lastSyncUs = micros();
    startRx(); // start searching rx for sync
  } else {
    Serial.printf("[TDMA] Invalid role\n");
  }

}
void tdmaUpdate() {
  uint32_t now = micros() + state.clockOffsetUs;
  uint32_t elapsed = now - state.frameStartUs;

  if (state.role == TDMA_FOLLOWER) {
    if (micros() - state.lastSyncUs >= FRAME_LEN_US * 10 && state.synced) {
      state.synced = false;
      state.clockOffsetUs = 0;
      state.frameStartUs = micros();
      Serial.printf("[TDMA] Sync lost\n");
      startRx();
    }
  }

  while(elapsed >= FRAME_LEN_US) {
    elapsed -= FRAME_LEN_US;
    state.frameStartUs += FRAME_LEN_US;
    state.frameSeq++;
    tdmaEnterSlot(GUARD);
  }

  SlotId slot = state.currentSlot;
  for (SlotWindow frameSlot : kFrameSlots) {
    if (elapsed >= frameSlot.startUs && elapsed < frameSlot.endUs) {
      slot = frameSlot.id;
      break;
    }
  }

  if (slot != state.currentSlot) {
    tdmaEnterSlot(slot);
  }
}                            

void tdmaBuildHeader(struct tdmaHeader &header){
  header.slot_id = state.currentSlot;
  header.frame_seq = state.frameSeq;
  header.epoch_us = state.frameStartUs; 
} 

void tdmaEnterSlot(SlotId next_slot) {

  state.currentSlot = next_slot;

  switch (next_slot) {
    case GUARD:
      startRx();
      break;
    case DOWNLINK:
      // enter tx for master, rx for follower
      if (state.role == TDMA_MASTER) {
        tdmaTransmit();
      } else {
        startRx();
      }
      break;
    case UPLINK:
      // rx master, tx follower
      if (state.role == TDMA_FOLLOWER && !txBuf.isEmpty() && state.synced) {
        tdmaTransmit();
      } else {
        startRx();
      }
      break;
    default:
      break; 
  }
} 

void tdmaProcessRx(const uint8_t *buf, size_t len) {

  // walk through payload and extract header and can records
  if (len < sizeof(tdmaHeader)) {
    return;
  }

  size_t offset = 0;

  uint8_t slot_id_raw;
  uint16_t frame_seq;
  uint32_t epoch_us;

  // parse header fields seperately
  memcpy(&slot_id_raw, &buf[offset], sizeof(slot_id_raw));
  offset += sizeof(slot_id_raw);

  if (slot_id_raw != GUARD && slot_id_raw != UPLINK && slot_id_raw != DOWNLINK){
    return;
  }

  memcpy(&frame_seq, &buf[offset], sizeof(frame_seq));
  offset += sizeof(frame_seq);
  memcpy(&epoch_us, &buf[offset], sizeof(epoch_us));
  offset += sizeof(epoch_us);

  uint32_t rx_time = micros();

  if (state.role == TDMA_FOLLOWER && (SlotId)slot_id_raw == DOWNLINK) {
    uint32_t master_frame_start = epoch_us;
    state.clockOffsetUs = (int32_t)(epoch_us - rx_time);
    state.frameSeq = frame_seq;
    state.frameStartUs = master_frame_start;
    state.synced = true;
    state.lastSyncUs = rx_time;
    Serial.printf("[TDMA] offset = %d\n", state.clockOffsetUs);
  }
  
  while (offset + sizeof(canRec) <= len) {
    canRec rec;
    memcpy(&rec, &buf[offset], sizeof(rec));
    offset += sizeof(rec);

    rxBuf.push(rec);
  }
  
}

void tdmaTransmit() {
  struct tdmaHeader header;
  tdmaBuildHeader(header);

  uint8_t payload[MAX_PAYLOAD_LENGTH];
  
  size_t offset = 0;

  memcpy(&payload[offset], &header, sizeof(header));

  offset += sizeof(header);

  while (!txBuf.isEmpty() && (offset + sizeof(canRec)) < MAX_PAYLOAD_LENGTH) {
    canRec rec = txBuf.shift();
    memcpy(&payload[offset], &rec, sizeof(rec));
    offset += sizeof(rec);
  }

  radioTransmit(payload, offset);
}

bool tdmaIsSynced(){
  return state.synced;
}
