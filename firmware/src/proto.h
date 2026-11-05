/* UART protocol, node -> host. Must match host/uwbproto.py and PLAN.md section 8.
 *
 *   | 0xAA | 0x55 | len | type | payload[len] | crc16 (LE) |
 *
 * CRC-16/CCITT-FALSE over len, type and payload. All fields little-endian.
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

#define PKT_SYNC0 0xAA
#define PKT_SYNC1 0x55

enum pkt_type {
    PKT_RANGE = 0x01,
    PKT_ANCHOR_PAIR = 0x02,
    PKT_STATUS = 0x03,
    /* host -> node commands, same framing */
    CMD_RESURVEY = 0x80, /* tag: forget the anchor table and survey every pair again */
    CMD_SURVEY_MORE = 0x81, /* tag: survey only the pairs still missing, keeping the measured ones */
};

enum range_flags {
    RANGE_F_TIMEOUT = 1u << 0, /* no RESP or no REPORT */
    RANGE_F_RX_ERR = 1u << 1,  /* frame error / unexpected frame */
    RANGE_F_NLOS = 1u << 2,    /* rx_power - fp_power > 6 dB */
    RANGE_F_INVALID = 1u << 3, /* range not computed */
    RANGE_F_TX_LATE = 1u << 4, /* delayed FINAL could not be scheduled in time */
};

struct __attribute__((packed)) pkt_range {
    uint16_t seq;
    uint16_t tag_id;
    uint16_t anchor_id;
    uint32_t t_us;
    int32_t range_mm;
    int16_t rx_power_cdbm;
    int16_t fp_power_cdbm;
    uint8_t flags;
};

enum survey_state {
    SURVEY_IDLE = 0,       /* table complete, or node is an anchor */
    SURVEY_WAITING = 1,    /* tag: not all anchors alive yet */
    SURVEY_RUNNING = 2,    /* tag: measuring anchor pairs, no RANGE packets meanwhile */
    SURVEY_INCOMPLETE = 3, /* tag: too few pairs for a layout, retrying later */
    SURVEY_PARTIAL = 4,    /* tag: some pairs missing but the layout is solvable; no automatic retries */
    SURVEY_GAVE_UP = 5,    /* tag: still not solvable after SURVEY_MAX_ROUNDS; waits for CMD_SURVEY_MORE or CMD_RESURVEY */
};

struct __attribute__((packed)) pkt_status {
    uint8_t role;
    uint8_t node_id;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t n_anchors;
    uint32_t uptime_ms;
    uint16_t n_ok;   /* in the last status period */
    uint16_t n_fail;
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint32_t dw_dev_id;
    /* since firmware 0.2 */
    uint8_t ant_src;    /* enum ant_src */
    uint32_t tx_power;  /* 0 = chip default */
    uint8_t alive_mask; /* tag: bit (id-1) set if anchor id answered in the last second */
    uint8_t pairs_valid;
    uint8_t survey_state;
};

/* Distance between two anchors, measured by the anchors themselves during the survey. */
struct __attribute__((packed)) pkt_anchor_pair {
    uint16_t table_version; /* increments whenever any pair changes */
    uint16_t id_a;
    uint16_t id_b;
    int32_t dist_mm;        /* median */
    uint16_t std_mm;
    uint16_t n_samples;     /* successful exchanges behind the median */
};

/* Sizes are part of the wire format (host/uwbproto.py). */
_Static_assert(sizeof(struct pkt_range) == 19, "pkt_range size");
_Static_assert(sizeof(struct pkt_status) == 29, "pkt_status size");
_Static_assert(sizeof(struct pkt_anchor_pair) == 14, "pkt_anchor_pair size");

uint16_t proto_crc16(const uint8_t *data, uint32_t len);
void proto_send(uint8_t type, const void *payload, uint8_t len);

/* Consumes received UART bytes; returns the type of a complete, CRC-valid command, or -1. */
int proto_poll_command(void);

#endif
