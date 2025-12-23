# Brage Arduino Firmware

Bridges the rocket CAN bus to a 2.4 GHz SX1280 radio link using a simple TDMA schedule. The GCS runs as TDMA master (downlink slot sender); the rocket runs as TDMA follower (uplink slot sender).

## Overview
- 100 ms TDMA frame: 10 ms guard, 60 ms downlink, 10 ms guard, 20 ms uplink (`tdma.h`).
- CAN frames are buffered into `txBuf` (CAN→radio) and `rxBuf` (radio→CAN) and carried inside each TDMA packet (`can.cpp`, `tdma.cpp`).
- Radio layer uses LoRa modulation (SF6, 812.5 kHz BW), SX1280 + power amplifier with RF switch table, and DIO1 IRQ polling from the main loop (`radio.cpp`).
- Designed for STM32C0xx (Nucleo C092RC) or STM32U5xx (brage) with an external CAN transceiver and SX1280 IC with RF front-end (`pin_config.h`).

Prerequisites / dependencies
- Arduino CLI/IDE with STM32 core installed (provides HAL for CAN enabled in `hal_conf_extra.h`) 
- Libraries: RadioLib, CircularBuffer
- Hardware: Nucleo C092RC + LAMBDA80 24S or Brage custom board
- Keep loop non-blocking so TDMA timing stays accurate; avoid `delay` in the main loop.

Best practices
- Match master/follower roles across devices; a follower transmits only when synced to the master header.
- Always call `handleRadioIrq()` and `tdmaUpdate()` every loop iteration; blocking code breaks slot timing.
- If you change radio parameters (bit rate, coding rate, sync word), update both ends.
- Use short logging in ISR-adjacent paths; heavy `Serial` use can jitter TDMA timing.

## Configuration
- Role selection (`brage_arduino.ino`): set `#define ROLE TDMA_MASTER` or `TDMA_FOLLOWER`. Follower will only transmit when synced.
- Radio settings (`radio.cpp`):
  - LoRa: SF6, 812.5 kHz bandwidth, CR 5, 13 dBm output power via `configRadio()`.
  - RF switch pins / DIO1 / RESET / BUSY from `pin_config.h`.
- TDMA timing (`tdma.h`): `FRAME_LEN_US=100000`, `DOWNLINK_TIME_US=60000`, `UPLINK_TIME_US=20000`, `GUARD_TIME_US=10000`.
- Payload limits (`tdma.h`): Master (GCS) sends up to 2 CAN records (34 bytes), Follower (Rocket) sends up to 16 CAN records (208 bytes).
- CAN layer (`can.cpp`):
  - Nominal/data bit timing set for 500 kbps with 48 MHz CAN clock.
  - Filters accept all standard IDs.
  - `MAX_LENGTH=32` records per circular buffer (`rxBuf`, `txBuf`).
- Pin mapping (`pin_config.h`):
  - STM32C0xx and STM32U5xx variants define SPI, RF switch, LED, and FDCAN pins/alternate functions.
- HAL extras (`hal_conf_extra.h`): `HAL_FDCAN_MODULE_ENABLED` required for linking HAL FDCAN symbols.

## Function reference
- CAN layer (`can.h`, `can.cpp`)
  - `initCan()`: configure GPIO, clock, filters, bit timing, start FDCAN1.
  - `pollCanRx()`: move received CAN frames into `txBuf` (for radio uplink/downlink).
  - `processCanTx()`: drain `rxBuf` (from radio) into FDCAN TX FIFO.
  - Buffers: `CircularBuffer<canRec, MAX_LENGTH> rxBuf` (radio→CAN), `txBuf` (CAN→radio); `canRec` holds `id`, `dlc`, `data[8]`.
- Radio layer (`radio.h`, `radio.cpp`)
  - `initRadio()`: bring up SX1280 in LoRa mode, configure RF switch table, attach DIO1 ISR.
  - `configRadio()`: apply spreading factor, bandwidth, coding rate, output power.
  - `startRx()`: enter receive mode; called at slot boundaries and after TX/RX.
  - `handleRadioIrq()`: poll DIO1 flag, dispatch RX_DONE / TX_DONE, restart RX.
  - `radioTransmit(const uint8_t* buf, size_t len)`: start TX if not busy; falls back to RX on error.
  - `radioIdle()`: place radio in standby.
- TDMA protocol (`tdma.h`, `tdma.cpp`)
  - `tdmaInit(TdmaRole role)`: initialize state; master starts in guard/tx, follower waits for sync and listens.
  - `tdmaUpdate()`: run every loop; advances slots based on `micros()`, handles frame rollover, loss-of-sync.
  - `tdmaProcessRx(const uint8_t* buf, size_t len, uint32_t rx_time_us)`: parse TDMA header, update follower clock offset, push embedded `canRec` payloads into `rxBuf`. `rx_time_us` should be captured as close to the radio RX_DONE interrupt as possible.
  - `tdmaIsSynced()`: follower sync status; use to gate uplink transmissions.
  - Internals: `tdmaTransmit()` builds `[tdmaHeader][canRec]*` payload from `txBuf` respecting role-specific payload limits.
