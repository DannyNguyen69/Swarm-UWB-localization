#include <stddef.h>
#include "config.h"
#include "hw.h"
#include "uart.h"
#include "proto.h"
#include "radio.h"
#include "node.h"
#include "board.h"

void node_send_status(const struct node_cfg *cfg, uint32_t dev_id, uint16_t n_ok, uint16_t n_fail,
                      const struct survey_info *survey)
{
    struct pkt_status s = {
        .ant_src = cfg->ant_src,
        .tx_power = cfg->tx_power,
        .alive_mask = survey ? survey->alive_mask : 0,
        .pairs_valid = survey ? survey->pairs_valid : 0,
        .survey_state = survey ? survey->state : SURVEY_IDLE,
        .role = cfg->role,
        .node_id = cfg->node_id,
        .fw_major = FW_MAJOR,
        .fw_minor = FW_MINOR,
        .n_anchors = cfg->n_anchors,
        .uptime_ms = hw_micros() / 1000u,
        .n_ok = n_ok,
        .n_fail = n_fail,
        .tx_ant_dly = cfg->tx_ant_dly,
        .rx_ant_dly = cfg->rx_ant_dly,
        .dw_dev_id = dev_id,
    };
    proto_send(PKT_STATUS, &s, sizeof s);
}

/* Unconfigured board or dead radio: keep reporting so flash.py can still identify it. */
static void idle_forever(const struct node_cfg *cfg, uint32_t dev_id)
{
    for (;;) {
        node_send_status(cfg, dev_id, 0, 0, NULL);
        led_toggle(PIN_LED_RED);
        hw_delay_ms(500);
    }
}

int main(void)
{
    struct node_cfg cfg;
    config_load(&cfg);

    hw_init();
    uart_init(cfg.baud);

    uint32_t dev_id = radio_init(&cfg);
    if (dev_id == 0 || cfg.role == ROLE_NONE)
        idle_forever(&cfg, dev_id);

    if (cfg.role == ROLE_ANCHOR)
        anchor_run(&cfg, dev_id);
    else
        tag_run(&cfg, dev_id);
}
