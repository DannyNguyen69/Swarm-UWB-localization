#include "uart.h"
#include "board.h"
#include "nrf.h"
#include <string.h>

#define UART_BUF_SIZE 128
#define RX_RING_SIZE 64

static uint8_t tx_buf[UART_BUF_SIZE];
static bool tx_active;

/* RX: one-byte EasyDMA transfers restarted by the ENDRX_STARTRX shortcut; the interrupt moves
 * each byte into the ring before the next one (87 us apart at 115200) can land. */
static volatile uint8_t rx_byte;
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head, rx_tail;

/* BAUDRATE = round(baud * 2^32 / 16 MHz), low 12 bits cleared. Unlike the
 * predefined constants this gives ~921600 instead of 941176 for 921600. */
static uint32_t baud_reg(uint32_t baud)
{
    uint64_t reg = (((uint64_t)baud << 32) + 8000000u) / 16000000u;
    return (uint32_t)reg & 0xFFFFF000u;
}

void uart_init(uint32_t baud)
{
    NRF_P0->OUTSET = 1u << PIN_UART_TX;
    NRF_P0->PIN_CNF[PIN_UART_TX] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;

    NRF_P0->PIN_CNF[PIN_UART_RX] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                   (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    NRF_UARTE0->PSEL.TXD = PIN_UART_TX;
    NRF_UARTE0->PSEL.RXD = PIN_UART_RX;
    NRF_UARTE0->PSEL.RTS = 0xFFFFFFFFu;
    NRF_UARTE0->PSEL.CTS = 0xFFFFFFFFu;
    NRF_UARTE0->CONFIG = 0;
    NRF_UARTE0->BAUDRATE = baud_reg(baud);
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;
    tx_active = false;

    rx_head = rx_tail = 0;
    NRF_UARTE0->RXD.PTR = (uint32_t)&rx_byte;
    NRF_UARTE0->RXD.MAXCNT = 1;
    NRF_UARTE0->SHORTS = UARTE_SHORTS_ENDRX_STARTRX_Msk;
    NRF_UARTE0->EVENTS_ENDRX = 0;
    NRF_UARTE0->INTENSET = UARTE_INTENSET_ENDRX_Msk;
    NVIC_SetPriority(UARTE0_UART0_IRQn, 3);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);
    NRF_UARTE0->TASKS_STARTRX = 1;
}

void UARTE0_UART0_IRQHandler(void)
{
    if (NRF_UARTE0->EVENTS_ENDRX) {
        NRF_UARTE0->EVENTS_ENDRX = 0;
        uint32_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = rx_byte;
            rx_head = next;
        }
    }
}

int uart_getc(void)
{
    if (rx_tail == rx_head)
        return -1;
    uint8_t c = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % RX_RING_SIZE;
    return c;
}

bool uart_busy(void)
{
    if (tx_active && NRF_UARTE0->EVENTS_ENDTX) {
        NRF_UARTE0->EVENTS_ENDTX = 0;
        NRF_UARTE0->TASKS_STOPTX = 1;
        tx_active = false;
    }
    return tx_active;
}

void uart_write(const uint8_t *data, uint32_t len)
{
    if (len > UART_BUF_SIZE)
        len = UART_BUF_SIZE;
    while (uart_busy()) {
    }
    memcpy(tx_buf, data, len);
    NRF_UARTE0->EVENTS_ENDTX = 0;
    NRF_UARTE0->TXD.PTR = (uint32_t)tx_buf;
    NRF_UARTE0->TXD.MAXCNT = len;
    NRF_UARTE0->TASKS_STARTTX = 1;
    tx_active = true;
}
