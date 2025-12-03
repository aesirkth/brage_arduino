#pragma once

#include <RadioLib.h>

#if defined (STM32C0xx)

  #define SX_CS 10
  #define SX_RESET 2
  #define SX_BUSY 3
  #define SX_DIO1 4

  #define PA_EN RADIOLIB_NC // unused
  #define TX_EN 6
  #define RX_EN 5

  #define LED_TX LED_GREEN
  #define LED_RX LED_BLUE

  #define FDCAN_GPIOS GPIOD 
  #define FDCAN_RX_GPIO GPIO_PIN_0
  #define FDCAN_TX_GPIO GPIO_PIN_1
  #define FDCAN_ALTERNATE GPIO_AF4_FDCAN1

#endif

#if defined (STM32U5xx)

  #define SX_CS PB0
  #define SX_DIO1 PB3
  #define SX_RESET PB6
  #define SX_BUSY PA15

  #define VREF_EN PA9 // 2V8_EN
  #define PA_EN PC13
  #define TX_EN PC14
  #define RX_EN PC15

  #define LED_TX PB12
  #define LED_RX PB13

  #define FDCAN_GPIOS GPIOB
  #define FDCAN_RX_GPIO GPIO_PIN_8
  #define FDCAN_TX_GPIO GPIO_PIN_9
  #define FDCAN_ALTERNATE GPIO_AF9_FDCAN1

#endif