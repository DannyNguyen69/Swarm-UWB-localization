#ifndef DW_PORT_H
#define DW_PORT_H

void dw_port_init(void);
void dw_port_reset(void);
void dw_port_spi_slow(void); /* 2 MHz, required until dwt_initialise() has run */
void dw_port_spi_fast(void); /* 8 MHz, nRF52832 SPIM maximum */

#endif
