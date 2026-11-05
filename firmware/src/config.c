#include "config.h"
#include "nrf.h"

void config_load(struct node_cfg *cfg)
{
    uint32_t w0 = NRF_UICR->CUSTOMER[0];
    uint32_t w1 = NRF_UICR->CUSTOMER[1];
    uint32_t w2 = NRF_UICR->CUSTOMER[2];
    uint32_t w3 = NRF_UICR->CUSTOMER[3];

    cfg->valid = (w0 == CFG_MAGIC);
    cfg->role = cfg->valid ? (uint8_t)(w1 & 0xFF) : ROLE_NONE;
    cfg->node_id = cfg->valid ? (uint8_t)((w1 >> 8) & 0xFF) : 0;
    cfg->n_anchors = cfg->valid ? (uint8_t)((w1 >> 16) & 0xFF) : DEFAULT_N_ANCHORS;
    if (cfg->n_anchors == 0 || cfg->n_anchors > MAX_ANCHORS)
        cfg->n_anchors = DEFAULT_N_ANCHORS;
    if (cfg->role != ROLE_ANCHOR && cfg->role != ROLE_TAG)
        cfg->role = ROLE_NONE;

    uint16_t tx = (uint16_t)(w2 & 0xFFFF);
    uint16_t rx = (uint16_t)(w2 >> 16);
    bool uicr_ant = cfg->valid && tx != 0xFFFF && rx != 0xFFFF;
    cfg->tx_ant_dly = uicr_ant ? tx : DEFAULT_ANT_DLY;
    cfg->rx_ant_dly = uicr_ant ? rx : DEFAULT_ANT_DLY;
    cfg->ant_src = uicr_ant ? ANT_SRC_UICR : ANT_SRC_DEFAULT;
    cfg->tx_power = 0;
    cfg->baud = (cfg->valid && w3 != 0xFFFFFFFFu && w3 != 0) ? w3 : DEFAULT_BAUD;
}
