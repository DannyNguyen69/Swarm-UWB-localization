#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

void uart_init(uint32_t baud);

/* Queues one packet for DMA transmission. Waits only if the previous packet is still going out. */
void uart_write(const uint8_t *data, uint32_t len);

bool uart_busy(void);

/* Received bytes go into a ring buffer from the UARTE interrupt. Returns -1 if empty. */
int uart_getc(void);

#endif
