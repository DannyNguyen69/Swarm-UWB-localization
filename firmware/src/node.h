#ifndef NODE_H
#define NODE_H

#include <stdint.h>
#include "config.h"

#define STATUS_PERIOD_US 1000000u

void anchor_run(const struct node_cfg *cfg, uint32_t dev_id);
void tag_run(const struct node_cfg *cfg, uint32_t dev_id);

struct survey_info {
    uint8_t alive_mask;
    uint8_t pairs_valid;
    uint8_t state;
};

/* Sends a STATUS packet; counters are per status period. survey may be NULL (anchor). */
void node_send_status(const struct node_cfg *cfg, uint32_t dev_id, uint16_t n_ok, uint16_t n_fail,
                      const struct survey_info *survey);

#endif
