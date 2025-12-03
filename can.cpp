#include "can.h"

#include <Arduino.h>

static uint32_t txBufOverwrites = 0; // track overwritten CAN records

FDCAN_HandleTypeDef hfdcan1;

CircularBuffer<canRec, MAX_LENGTH> rxBuf;   // radio -> CAN
CircularBuffer<canRec, MAX_LENGTH> txBuf;   // CAN  -> radio

static uint8_t dlcToBytes(uint32_t dlc) {
  switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    default: return 8;
  }
}

static uint32_t bytesToDlc(uint8_t len) {
  switch (len) {
    case 0: return FDCAN_DLC_BYTES_0;
    case 1: return FDCAN_DLC_BYTES_1;
    case 2: return FDCAN_DLC_BYTES_2;
    case 3: return FDCAN_DLC_BYTES_3;
    case 4: return FDCAN_DLC_BYTES_4;
    case 5: return FDCAN_DLC_BYTES_5;
    case 6: return FDCAN_DLC_BYTES_6;
    case 7: return FDCAN_DLC_BYTES_7;
    default: return FDCAN_DLC_BYTES_8;
  }
}

static void EnableFdcanGpioClock() {
  #if defined (STM32C0xx)
    __HAL_RCC_GPIOD_CLK_ENABLE();
  #elif defined (STM32U5xx)
    __HAL_RCC_GPIOB_CLK_ENABLE();
  #endif
}

// FDCAN clock configuration and gpio initialization
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
  if (hfdcan->Instance == FDCAN1) {
    __HAL_RCC_FDCAN1_CLK_ENABLE();
    EnableFdcanGpioClock();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin       = FDCAN_TX_GPIO;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = FDCAN_ALTERNATE;
    HAL_GPIO_Init(FDCAN_GPIOS, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = FDCAN_RX_GPIO;
    HAL_GPIO_Init(FDCAN_GPIOS, &GPIO_InitStruct);
  }
}

void initCan() {
  Serial.println("[CAN] Initializing...");

  hfdcan1.Instance = FDCAN1;

  // Basic config
  hfdcan1.Init.ClockDivider       = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat        = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode               = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause      = DISABLE;
  hfdcan1.Init.ProtocolException  = DISABLE;

  // Nominal bit timing for 500 kbps at 48 MHz CAN clock
  hfdcan1.Init.NominalPrescaler     = 6;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1      = 13;
  hfdcan1.Init.NominalTimeSeg2      = 2;

  // Data bit timing (same as nominal)
  hfdcan1.Init.DataPrescaler        = 6;
  hfdcan1.Init.DataSyncJumpWidth    = 1;
  hfdcan1.Init.DataTimeSeg1         = 13;
  hfdcan1.Init.DataTimeSeg2         = 2;

  // Filters and Tx FIFO/queue
  hfdcan1.Init.StdFiltersNbr        = 1;
  hfdcan1.Init.ExtFiltersNbr        = 0;
  hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  int ret = HAL_FDCAN_Init(&hfdcan1);
  Serial.printf("[CAN] Init returned (%d), ErrorCode=0x%x\n", ret, hfdcan1.ErrorCode);
  if (ret != HAL_OK) {
    Serial.println("[CAN] Init FAILED");
    return;
  }

  // configure filter
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0x0;
  sFilterConfig.FilterID2    = 0x0; // match all

  ret = HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  Serial.printf("[CAN] Filter config returned (%d), ErrorCode=0x%x\n", ret, hfdcan1.ErrorCode);
  if (ret != HAL_OK) {
    Serial.println("[CAN] Filter config FAILED");
    return;
  }

  ret = HAL_FDCAN_Start(&hfdcan1);
  Serial.printf("[CAN] Start returned (%d), ErrorCode=0x%x\n", ret, hfdcan1.ErrorCode);
}

// Check for received CAN messages and push them to txBuf
void pollCanRx() {

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) {
    return; // no frames received
  }

  uint8_t data[8];
  FDCAN_RxHeaderTypeDef rxHeader;

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, data) == HAL_OK) {
    uint8_t len = dlcToBytes(rxHeader.DataLength);
    // Serial.printf("[CAN] Frame received (ID=0x%x DLC=%d)\n",
    //               rxHeader.Identifier,
    //               rxHeader.DataLength);

    canRec rec;
    rec.id  = rxHeader.Identifier;
    rec.dlc = len;

    memset(rec.data, 0, sizeof(rec.data));
    memcpy(rec.data, data, len);

    if(!txBuf.push(rec)) {
      txBufOverwrites += 1; // push returns 0 on overwrite
    }

  } else {
    Serial.printf("[CAN] GetRxMessage failed, ErrorCode=0x%08lx\n", hfdcan1.ErrorCode);
  }
}

// empty rxBuf and transmit via CAN
void processCanTx() {

  while (!rxBuf.isEmpty() && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {

    canRec rec = rxBuf.shift();

    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier          = rec.id;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = bytesToDlc(rec.dlc);
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, rec.data) == HAL_OK) {
      // Serial.printf("[CAN] Frame sent (ID:0x%x, DLC:%d)\n", rec.id, rec.dlc);
    } else {
      Serial.printf("[CAN] AddMessageToTxFifoQ failed, Status=0x%x, ErrorCode=0x%x\n",
                    hfdcan1.ErrorCode, hfdcan1.ErrorCode);
      break;
    }
  }
}
