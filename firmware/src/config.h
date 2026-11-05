#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* Node configuration lives in UICR CUSTOMER[0..3] (0x10001080) and is written by flash.py:
 *   [0] magic 'UWB1'
 *   [1] role (bits 0-7) | node_id (8-15) | n_anchors (16-23)
 *   [2] tx_ant_dly (0-15) | rx_ant_dly (16-31), 0xFFFF = default
 *   [3] uart baud, 0xFFFFFFFF = default
 */
#define CFG_MAGIC 0x31425755u

enum role {
    ROLE_NONE = 0,
    ROLE_ANCHOR = 1,
    ROLE_TAG = 2,
};

#define DEFAULT_ANT_DLY   16436
#define DEFAULT_BAUD      115200u  /* 921600 lost ~18% of packets through the J-Link OB VCOM */
#define DEFAULT_N_ANCHORS 4
#define MAX_ANCHORS       8

/* Where the antenna delay in use came from (reported in STATUS). */
enum ant_src {
    ANT_SRC_DEFAULT = 0,
    ANT_SRC_OTP = 1,  /* factory calibration in DW1000 OTP 0x1C */
    ANT_SRC_UICR = 2, /* per-board calibration written by flash.py (PLAN.md phase 3) */
};

struct node_cfg {
    bool valid;
    uint8_t role;
    uint8_t node_id;   /* 1..N, same numbering as the PANS labels on the map in PLAN.md */
    uint8_t n_anchors; /* tag only: ranges anchors 1..n_anchors in turn */
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint8_t ant_src;   /* UICR or DEFAULT after config_load(); radio_init() may switch to OTP */
    uint32_t tx_power; /* TX_POWER register in use, 0 = chip default (set by radio_init) */
    uint32_t baud;
};

void config_load(struct node_cfg *cfg);

#endif
