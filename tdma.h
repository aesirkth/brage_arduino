/*
TDMA Protocol Layer

Allows duplex communication between rocket and gcs

Frame structure [100 ms]:
  [GUARD][DOWNLINK][GUARD][UPLINK]
  - GUARD - 10 ms
  - DOWNLINK - 60 ms (master TX, follower RX)
  - UPLINK - 20 ms (follower TX, master RX)

- Master (GCS) transmits during DOWNLINK, receives during UPLINK
- Follower (Rocket) receives during DOWNLINK, transmits during UPLINK
- Follower syncs clock using header information from master

Packet format:
  [tdmaHeader 8 bytes][canRec 13 bytes][canRec]...
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

// Frame timing
#define FRAME_LEN_US (100 * 1000) // 100 ms
#define DOWNLINK_TIME_US (60 * 1000) // 60 ms
#define UPLINK_TIME_US (20 * 1000) // 20 ms
#define GUARD_TIME_US (10 * 1000) // 10 ms

// Payload limits
#define CAN_REC_SIZE 13  // sizeof(canRec): 4 + 1 + 8
#define TDMA_HEADER_SIZE 8  // sizeof(tdmaHeader): 1 + 2 + 4 + 1

#define MASTER_MAX_CAN_RECORDS 2
#define FOLLOWER_MAX_CAN_RECORDS 16

// Master (GCS): header + 2 CAN records for commands
#define MASTER_PAYLOAD_LEN (TDMA_HEADER_SIZE + (MASTER_MAX_CAN_RECORDS * CAN_REC_SIZE))  // 34 bytes

// Follower (Rocket): 16 CAN records for telemetry (no header)
#define FOLLOWER_PAYLOAD_LEN (FOLLOWER_MAX_CAN_RECORDS * CAN_REC_SIZE)  // 208 bytes

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
