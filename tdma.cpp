#include <Arduino.h>
#include "radio.h"
#include "tdma.h"
#include "can.h"

#ifndef TDMA_ENABLE_DEBUG
#define TDMA_ENABLE_DEBUG 1
#endif

#define TDMA_LOGF(...)    do { if (TDMA_ENABLE_DEBUG) { Serial.printf(__VA_ARGS__); } } while (0)
#define TDMA_LOGLN(msg)   do { if (TDMA_ENABLE_DEBUG) { Serial.println(msg); } } while (0)

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
  int64_t now = (int64_t)micros() + (int64_t)state.clockOffsetUs;
  int64_t elapsed = now - (int64_t)state.frameStartUs;

  if (elapsed < 0) {
    elapsed = 0;
  }

  if (state.role == TDMA_FOLLOWER) {
    if (micros() - state.lastSyncUs >= FRAME_LEN_US * 10 && state.synced) {
      state.synced = false;
      state.clockOffsetUs = 0;
      state.frameStartUs = micros();
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
    // if (state.role == TDMA_FOLLOWER) {
    //   Serial.printf("[TDMA] Slot change -> %u frame=%u elapsed=%ld\n",
    //                 (unsigned int)slot,
    //                 (unsigned int)state.frameSeq,
    //                 (long)elapsed);
    // } else if (state.role == TDMA_MASTER) {
    //   Serial.printf("[TDMA] M Slot change -> %u frame=%u elapsed=%ld\n",
    //                 (unsigned int)slot,
    //                 (unsigned int)state.frameSeq,
    //                 (long)elapsed);
    // }
    tdmaEnterSlot(slot);
  }
}                            

static void tdmaBuildHeader(struct tdmaHeader &header, uint8_t num_records){
  header.slot_id = state.currentSlot;
  header.frame_seq = state.frameSeq;
  header.epoch_us = state.frameStartUs; 
  header.num_records = num_records;
} 

static void tdmaEnterSlot(SlotId next_slot) {

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
      if (state.role == TDMA_FOLLOWER) {
        bool hasData = !txBuf.isEmpty();
        bool canTx = state.synced && hasData;
        TDMA_LOGF("[TDMA] UPLINK entry: synced=%d empty=%d tx=%d frame=%u slot=%u\n",
                  (int)state.synced,
                  (int)!hasData ? 1 : 0,
                  (int)canTx,
                  (unsigned int)state.frameSeq,
                  (unsigned int)state.currentSlot);
        if (canTx) {
          tdmaTransmit();
        } else {
          startRx();
        }
      } else {
        startRx();
      }
      break;
    default:
      break; 
  }
} 

void tdmaProcessRx(const uint8_t *buf, size_t len, uint32_t rx_time) {

  size_t offset = 0;

  uint8_t slot_id_raw;
  uint16_t frame_seq;
  uint32_t epoch_us;
  uint8_t num_records;

  // Master receiving UPLINK has no header, follower receiving DOWNLINK has header
  bool parseHeader = (state.role == TDMA_FOLLOWER);

  if (parseHeader) {
    // Follower expects DOWNLINK with header
    if (len < sizeof(tdmaHeader)) {
      return;
    }

    // Parse header fields separately
    memcpy(&slot_id_raw, &buf[offset], sizeof(slot_id_raw));
    offset += sizeof(slot_id_raw);

    if (slot_id_raw != GUARD && slot_id_raw != UPLINK && slot_id_raw != DOWNLINK){
      return;
    }

    memcpy(&frame_seq, &buf[offset], sizeof(frame_seq));
    offset += sizeof(frame_seq);
    memcpy(&epoch_us, &buf[offset], sizeof(epoch_us));
    offset += sizeof(epoch_us);
    memcpy(&num_records, &buf[offset], sizeof(num_records));
    offset += sizeof(num_records);

    TDMA_LOGF("[TDMA] RX header: slot = %d, frame = %d, epoch = %lld, num_records = %d\n", slot_id_raw, frame_seq, epoch_us, num_records);

    // Follower synchronization from DOWNLINK header
    if ((SlotId)slot_id_raw == DOWNLINK) {
      // Estimate when the master started the frame by backing out guard + half slot
      uint32_t rx_est = rx_time;
      uint32_t downlink_mid = GUARD_TIME_US + (DOWNLINK_TIME_US / 2);
      if (rx_est > downlink_mid) {
        rx_est -= downlink_mid;
      }
      state.clockOffsetUs = (int32_t)(epoch_us - rx_est);
      state.frameSeq = frame_seq;
      state.frameStartUs = epoch_us;
      state.synced = true;
      state.lastSyncUs = rx_time;
    }

    TDMA_LOGF("[TDMA] RX frame=%u records=%u\n",
              (unsigned int)frame_seq,
              (unsigned int)num_records);
  } else {
    // Master receiving headerless UPLINK
    // No header to parse, offset stays at 0
    num_records = len / sizeof(canRec);  // Calculate from packet size
    TDMA_LOGF("[TDMA] RX UPLINK (no header) len=%u records=%u\n", (unsigned int)len, (unsigned int)num_records);
  }

  // Extract CAN records using while loop - more robust than relying on num_records
  uint8_t rec_count = 0;
  while (offset + sizeof(canRec) <= len) {
    canRec rec;
    memcpy(&rec, &buf[offset], sizeof(rec));
    offset += sizeof(rec);
    rxBuf.push(rec);
    TDMA_LOGF("  RX rec %u: id=0x%lx dlc=%u\n",
              (unsigned int)rec_count,
              (unsigned long)rec.id,
              (unsigned int)rec.dlc);
    rec_count++;
  }

}

static void tdmaTransmit() {

  struct tdmaHeader header;
  uint8_t payload[MAX_PAYLOAD_LENGTH];
  uint8_t num_records = 0;

  // Master sends header, follower does not
  size_t offset = (state.role == TDMA_MASTER) ? sizeof(tdmaHeader) : 0;

  while (!txBuf.isEmpty() && (offset + sizeof(canRec)) < MAX_PAYLOAD_LENGTH) {
    canRec rec = txBuf.shift();
    memcpy(&payload[offset], &rec, sizeof(rec));
    offset += sizeof(rec);
    num_records++;
    TDMA_LOGF("  TX rec %u: id=0x%lx dlc=%u\n",
              (unsigned int)(num_records - 1),
              (unsigned long)rec.id,
              (unsigned int)rec.dlc);
  }

  // Only master builds and sends header (DOWNLINK)
  if (state.role == TDMA_MASTER) {
    tdmaBuildHeader(header, num_records);
    TDMA_LOGF("[TDMA] TX hdr slot=%u frame=%u records=%u\n",
              (unsigned int)header.slot_id,
              (unsigned int)header.frame_seq,
              (unsigned int)header.num_records);
    memcpy(&payload[0], &header, sizeof(header));

    TDMA_LOGF("[TDMA] TX frame=%u slot=%u records=%u (currentSlot=%u)\n",
              (unsigned int)header.frame_seq,
              (unsigned int)header.slot_id,
              (unsigned int)header.num_records,
              (unsigned int)state.currentSlot);
  } else {
    // Follower sends headerless UPLINK
    TDMA_LOGF("[TDMA] TX UPLINK (no header) records=%u\n", (unsigned int)num_records);
  }

  radioTransmit(payload, offset);
}

bool tdmaIsSynced(){
  return state.synced;
}
