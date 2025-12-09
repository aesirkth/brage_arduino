#include <sys/_stdint.h>
/*
TDMA Protocol Layer

Allows duplex communication between rocket and gcs

Frame structure [20 ms]:
  [GUARD][DOWNLINK][GUARD][UPLINK]
  - GUARD - 1 ms
  - DOWNLINK/UPLINK - 9 ms

- Master (rocket) transmits during DOWNLINK, receives during UPLINK
- Follower (gcs) receives during DOWNLINK, transmits during UPLINK
- Follower syncs clock using header information from master

Packet format:
  [tdmaHeader 7 bytes][canRec][canRec]...
*/

#pragma once

#define FRAME_LEN_US 30000 // 30 ms
#define DOWNLINK_TIME_US 13000 // 13 ms
#define UPLINK_TIME_US 13000 // 13 ms
#define GUARD_TIME_US 2000 // 2 ms

enum TdmaRole: uint8_t {
  TDMA_MASTER,
  TDMA_FOLLOWER
};

enum SlotId: uint8_t {
  DOWNLINK,
  UPLINK,
  GUARD
};

struct tdmaState {
  TdmaRole role;
  SlotId currentSlot;

  uint16_t frameSeq;     // current frame sequence
  uint32_t frameStartUs; // start time of frame in microseconds
  int32_t clockOffsetUs; // offset between follower and master
  
  uint32_t lastSyncUs;
  bool synced;           // follower only transmits when true
};

struct tdmaHeader {
  SlotId slot_id;
  uint16_t frame_seq; 
  uint32_t epoch_us;  // micros() captured at tx start
  uint8_t num_records; // # of CAN records in payload
} __attribute__((packed));

void tdmaInit(TdmaRole role);
void tdmaUpdate(); // run every loop iteration: check micros(), advance slots, control radio actions
void tdmaProcessRx(const uint8_t *buf, size_t len, uint32_t rx_time_us); // process received message: decode header, update clockOffset (follower), push CAN payloads

bool tdmaIsSynced(); 
