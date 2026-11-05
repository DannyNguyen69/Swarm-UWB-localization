/* DWM1001-DEV pin map (see dwm1001-examples/boards/dw1001_dev.h). */
#ifndef BOARD_H
#define BOARD_H

#define PIN_DW_SCK    16
#define PIN_DW_MOSI   20
#define PIN_DW_MISO   18
#define PIN_DW_CS     17
#define PIN_DW_IRQ    19
#define PIN_DW_RST    24

#define PIN_UART_TX    5   /* to J-Link VCOM */
#define PIN_UART_RX   11

/* LEDs are active low. */
#define PIN_LED_GREEN 30
#define PIN_LED_BLUE  31
#define PIN_LED_RED   22

#endif
