/* Asymmetric DS-TWR, used by the tag (to anchors) and by anchors (to each other during survey).
 *
 *   initiator                 responder
 *   t1 POLL ───────────────>  t2
 *   t4      <─────────────── RESP {t2, t3}       Ra = t4 - t1   Da = t3 - t2
 *   t5 FINAL ──────────────>  t6                 Rb = t6 - t3   Db = t5 - t4
 *           <─────────────── REPORT {t6, uptime}
 *
 *   ToF = (Ra*Rb - Da*Db) / (Ra + Rb + Da + Db)
 *
 * Differences use the low 32 bits of the 40-bit DW1000 clock; every interval is far below
 * the 67 ms wrap of those bits.
 */
#ifndef TWR_H
#define TWR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "radio.h"

struct twr_result {
    uint8_t flags;      /* RANGE_F_* from proto.h */
    int32_t range_mm;
    int16_t rx_cdbm;
    int16_t fp_cdbm;
    uint32_t peer_uptime_ms;
};

struct twr_result twr_initiate(const struct node_cfg *cfg, uint16_t me, uint16_t peer, uint8_t *seq);

/* Answers a POLL that has just been received (and read into poll). Returns true on success. */
bool twr_respond(const struct node_cfg *cfg, uint16_t me, const struct frame_hdr *poll, uint8_t *seq);

/* Waits for any good frame; returns its length (0 on timeout or error) and leaves RX off. */
uint16_t twr_receive(void *buf, uint16_t max, uint16_t timeout_uus);

#endif
