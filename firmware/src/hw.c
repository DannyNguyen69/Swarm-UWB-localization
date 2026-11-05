#include "hw.h"
#include "board.h"
#include "nrf.h"

static void gpio_output(uint32_t pin, bool high)
{
    if (high)
        NRF_P0->OUTSET = 1u << pin;
    else
        NRF_P0->OUTCLR = 1u << pin;
    NRF_P0->PIN_CNF[pin] = GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos;
}

void hw_init(void)
{
    /* The crystal keeps the UART baud rate and the microsecond timer accurate. */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_HFCLKSTARTED) {
    }

    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER1->PRESCALER = 4; /* 16 MHz / 2^4 = 1 MHz */
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->TASKS_START = 1;

    gpio_output(PIN_LED_GREEN, true);
    gpio_output(PIN_LED_BLUE, true);
    gpio_output(PIN_LED_RED, true);
}

uint32_t hw_micros(void)
{
    NRF_TIMER1->TASKS_CAPTURE[0] = 1;
    return NRF_TIMER1->CC[0];
}

void hw_delay_us(uint32_t us)
{
    uint32_t start = hw_micros();
    while ((uint32_t)(hw_micros() - start) < us) {
    }
}

void hw_delay_ms(uint32_t ms)
{
    hw_delay_us(ms * 1000u);
}

void led_set(uint32_t pin, bool on)
{
    if (on)
        NRF_P0->OUTCLR = 1u << pin;
    else
        NRF_P0->OUTSET = 1u << pin;
}

void led_toggle(uint32_t pin)
{
    NRF_P0->OUT ^= 1u << pin;
}
