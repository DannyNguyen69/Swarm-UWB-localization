#ifndef HW_H
#define HW_H

#include <stdint.h>
#include <stdbool.h>

void hw_init(void);

/* Free-running 1 MHz counter (TIMER1), wraps every ~71 minutes. */
uint32_t hw_micros(void);
void hw_delay_us(uint32_t us);
void hw_delay_ms(uint32_t ms);

void led_set(uint32_t pin, bool on);
void led_toggle(uint32_t pin);

#endif
