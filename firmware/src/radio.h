/* DW1000 setup, over-the-air frame format and DS-TWR timing shared by anchor and tag. */
#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* IEEE 802.15.4 data frame, PAN ID compression, 16-bit addresses. */
#define FRAME_FC     0x8841u
#define FRAME_PAN    0xDECAu
#define FRAME_FCS    2
#define ADDR_ANCHOR(id) ((uint16_t)(0xA000u | (id)))
#define ADDR_TAG(id)    ((uint16_t)(0x7000u | (id)))

enum frame_fn {
    FN_POLL = 0x20,
    FN_RESP = 0x21,
    FN_FINAL = 0x22,
    FN_REPORT = 0x23,
    FN_SURVEY_REQ = 0x30,   /* tag -> anchor a: range anchor b n times */
    FN_PAIR_RESULT = 0x31,  /* anchor a -> tag: result of SURVEY_REQ */
};

struct __attribute__((packed)) frame_hdr {
    uint16_t fc;
    uint8_t seq;
    uint16_t pan;
    uint16_t dst;
    uint16_t src;
    uint8_t fn;
};

struct __attribute__((packed)) frame_resp {
    struct frame_hdr h;
    uint32_t t2; /* anchor: POLL received (low 32 bits of DW1000 time) */
    uint32_t t3; /* anchor: RESP sent */
};

struct __attribute__((packed)) frame_report {
    struct frame_hdr h;
    uint32_t t6;        /* anchor: FINAL received */
    uint32_t uptime_ms; /* lets the initiator notice that this node rebooted */
};

struct __attribute__((packed)) frame_survey_req {
    struct frame_hdr h;
    uint8_t target_id;
    uint8_t n_samples;
};

struct __attribute__((packed)) frame_pair_result {
    struct frame_hdr h;
    uint8_t id_a;
    uint8_t id_b;
    uint8_t n_ok;
    int32_t median_mm;
    uint16_t std_mm;
};

_Static_assert(sizeof(struct frame_hdr) == 10, "frame_hdr size");

/* DS-TWR timing in UWB microseconds (1 UUS = 512/499.2 MHz ~ 1.026 us). Starting values,
 * to be tightened once turnaround times are measured on hardware (PLAN.md phase 2). */
#define UUS_TO_DWT_TIME              65536ull
#define POLL_RX_TO_RESP_TX_DLY_UUS   1000u  /* anchor */
#define RESP_RX_TO_FINAL_TX_DLY_UUS  1000u  /* tag */
#define RESP_RX_TIMEOUT_UUS          2500u  /* tag, RX on right after POLL */
#define FINAL_RX_TIMEOUT_UUS         2500u  /* anchor, RX on right after RESP */
#define REPORT_RX_TIMEOUT_UUS        2500u  /* tag, RX on right after FINAL */

#define SPEED_OF_LIGHT 299702547.0 /* in air */

/* Returns the DW1000 device id, or 0 if the chip did not come up. Fills in the antenna delay
 * and TX power actually used (factory OTP values unless UICR overrides the delay). */
uint32_t radio_init(struct node_cfg *cfg);

/* Busy-waits until one of the SYS_STATUS bits in mask is set and returns SYS_STATUS. */
uint32_t radio_wait(uint32_t mask);
void radio_clear(uint32_t bits);
void radio_rx_recover(void);

/* Writes a frame (FCS added by hardware) and starts TX. Returns DWT_ERROR if a delayed TX is late. */
int radio_send(void *frame, uint16_t len, uint8_t mode);

/* Reads the received frame into buf; returns its length without FCS, or 0 if it does not fit. */
uint16_t radio_read(void *buf, uint16_t max);

uint64_t radio_rx_ts(void);
uint64_t radio_tx_ts(void);

/* Delayed TX at (ref + delay); returns the exact RMARKER time the frame will carry. */
uint64_t radio_schedule_tx(uint64_t ref_ts, uint32_t delay_uus, uint16_t tx_ant_dly);

/* Receive power and first-path power of the last frame, in dBm * 100 (DW1000 UM 4.7). */
void radio_rx_power(int16_t *rx_cdbm, int16_t *fp_cdbm);

void frame_hdr_init(struct frame_hdr *h, uint8_t seq, uint16_t dst, uint16_t src, uint8_t fn);

#endif
