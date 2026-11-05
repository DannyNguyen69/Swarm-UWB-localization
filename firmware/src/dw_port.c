/* Platform glue required by the Decawave DW1000 driver: SPI, mutex and sleep. */
#include "dw_port.h"
#include "board.h"
#include "hw.h"
#include "nrf.h"
#include "deca_device_api.h"
#include <string.h>

#define SPI_CHUNK 255 /* SPIM EasyDMA MAXCNT is 8 bits on nRF52832 */

static uint8_t spi_tx[SPI_CHUNK];
static uint8_t spi_rx[SPI_CHUNK];

static void cs_low(void)  { NRF_P0->OUTCLR = 1u << PIN_DW_CS; }
static void cs_high(void) { NRF_P0->OUTSET = 1u << PIN_DW_CS; }

static void spim_xfer(uint32_t tx_len, uint32_t rx_len)
{
    NRF_SPIM1->TXD.PTR = (uint32_t)spi_tx;
    NRF_SPIM1->TXD.MAXCNT = tx_len;
    NRF_SPIM1->RXD.PTR = (uint32_t)spi_rx;
    NRF_SPIM1->RXD.MAXCNT = rx_len;
    NRF_SPIM1->EVENTS_END = 0;
    NRF_SPIM1->TASKS_START = 1;
    while (!NRF_SPIM1->EVENTS_END) {
    }
}

void dw_port_init(void)
{
    NRF_P0->OUTSET = 1u << PIN_DW_CS;
    NRF_P0->PIN_CNF[PIN_DW_CS] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;
    NRF_P0->OUTCLR = 1u << PIN_DW_SCK;
    NRF_P0->PIN_CNF[PIN_DW_SCK] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;
    NRF_P0->PIN_CNF[PIN_DW_MOSI] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;
    NRF_P0->PIN_CNF[PIN_DW_MISO] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                   (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P0->PIN_CNF[PIN_DW_RST] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    NRF_SPIM1->PSEL.SCK = PIN_DW_SCK;
    NRF_SPIM1->PSEL.MOSI = PIN_DW_MOSI;
    NRF_SPIM1->PSEL.MISO = PIN_DW_MISO;
    NRF_SPIM1->CONFIG = 0; /* mode 0, MSB first */
    NRF_SPIM1->ORC = 0x00;
    dw_port_spi_slow();
    NRF_SPIM1->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
}

void dw_port_spi_slow(void)
{
    NRF_SPIM1->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M2;
}

void dw_port_spi_fast(void)
{
    NRF_SPIM1->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
}

/* RSTn is open drain: pull it low briefly, then release it and let the DW1000 drive it. */
void dw_port_reset(void)
{
    NRF_P0->OUTCLR = 1u << PIN_DW_RST;
    NRF_P0->PIN_CNF[PIN_DW_RST] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;
    hw_delay_ms(2);
    NRF_P0->PIN_CNF[PIN_DW_RST] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    hw_delay_ms(5);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    cs_low();
    uint32_t n = headerLength;
    memcpy(spi_tx, headerBuffer, headerLength);
    while (bodylength > 0) {
        uint32_t take = SPI_CHUNK - n;
        if (take > bodylength)
            take = bodylength;
        memcpy(spi_tx + n, bodyBuffer, take);
        spim_xfer(n + take, 0);
        bodyBuffer += take;
        bodylength -= take;
        n = 0;
    }
    if (n > 0)
        spim_xfer(n, 0);
    cs_high();
    return 0;
}

/* The header is sent and the reply clocked in within one DMA transfer, so RXD.MAXCNT is
 * always >= 2 (avoids nRF52832 anomaly 58 with single-byte transfers). */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    cs_low();
    memcpy(spi_tx, headerBuffer, headerLength);
    uint32_t skip = headerLength;
    uint32_t tx_len = headerLength;
    while (readlength > 0) {
        uint32_t take = SPI_CHUNK - skip;
        if (take > readlength)
            take = readlength;
        spim_xfer(tx_len, skip + take);
        memcpy(readBuffer, spi_rx + skip, take);
        readBuffer += take;
        readlength -= take;
        skip = 0;
        tx_len = 0;
    }
    cs_high();
    return 0;
}

/* The firmware polls the DW1000 and does not use its IRQ line, so there is nothing to mask. */
decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
    (void)s;
}

void deca_sleep(unsigned int time_ms)
{
    hw_delay_ms(time_ms);
}
