/* Anchor: answers DS-TWR exchanges started by tags or other anchors, and on request from a tag
 * measures the distance to another anchor (survey, PLAN.md section 9). */
#include "node.h"
#include "twr.h"
#include "proto.h"
#include "hw.h"
#include "board.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <math.h>

#define RX_BUF 32
#define LISTEN_TIMEOUT_UUS 50000  /* short windows so STATUS still goes out with no tag around */
#define SURVEY_MAX_SAMPLES 200
#define SURVEY_GAP_US 300

static void sort_i32(int32_t *v, int n)
{
    for (int i = 1; i < n; i++) {
        int32_t x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
}

/* Ranges anchor target_id n times and reports median / std back to the requesting tag. */
static void run_survey(const struct node_cfg *cfg, const struct frame_survey_req *req, uint8_t *seq)
{
    static int32_t samples[SURVEY_MAX_SAMPLES];
    uint16_t me = ADDR_ANCHOR(cfg->node_id);
    uint16_t peer = ADDR_ANCHOR(req->target_id);
    int n = req->n_samples > SURVEY_MAX_SAMPLES ? SURVEY_MAX_SAMPLES : req->n_samples;
    int ok = 0;

    for (int i = 0; i < n; i++) {
        struct twr_result r = twr_initiate(cfg, me, peer, seq);
        if (!(r.flags & RANGE_F_INVALID))
            samples[ok++] = r.range_mm;
        hw_delay_us(SURVEY_GAP_US);
    }

    struct frame_pair_result res;
    frame_hdr_init(&res.h, (*seq)++, req->h.src, me, FN_PAIR_RESULT);
    res.id_a = cfg->node_id;
    res.id_b = req->target_id;
    res.n_ok = (uint8_t)ok;
    res.median_mm = 0;
    res.std_mm = 0;
    if (ok > 0) {
        sort_i32(samples, ok);
        res.median_mm = samples[ok / 2];
        double mean = 0, var = 0;
        for (int i = 0; i < ok; i++)
            mean += samples[i];
        mean /= ok;
        for (int i = 0; i < ok; i++)
            var += (samples[i] - mean) * (samples[i] - mean);
        double sd = ok > 1 ? sqrt(var / ok) : 0;
        res.std_mm = sd > 65535 ? 65535 : (uint16_t)sd;
    }
    radio_send(&res, sizeof res, DWT_START_TX_IMMEDIATE);
    radio_wait(SYS_STATUS_TXFRS);
    radio_clear(SYS_STATUS_TXFRS);
}

void anchor_run(const struct node_cfg *cfg, uint32_t dev_id)
{
    uint16_t me = ADDR_ANCHOR(cfg->node_id);
    uint8_t seq = 0;
    uint16_t n_ok = 0, n_fail = 0;
    uint32_t next_status = hw_micros();

    for (;;) {
        if ((int32_t)(hw_micros() - next_status) >= 0) {
            node_send_status(cfg, dev_id, n_ok, n_fail, NULL);
            n_ok = n_fail = 0;
            next_status += STATUS_PERIOD_US;
            led_toggle(PIN_LED_GREEN); /* heartbeat */
        }

        uint8_t buf[RX_BUF];
        uint16_t len = twr_receive(buf, sizeof buf, LISTEN_TIMEOUT_UUS);
        const struct frame_hdr *h = (const struct frame_hdr *)buf;
        if (len == 0 || h->dst != me)
            continue;

        if (h->fn == FN_POLL) {
            struct frame_hdr poll = *h;
            if (twr_respond(cfg, me, &poll, &seq)) {
                n_ok++;
                led_toggle(PIN_LED_BLUE);
            } else {
                n_fail++;
            }
        } else if (h->fn == FN_SURVEY_REQ && len >= sizeof(struct frame_survey_req)) {
            struct frame_survey_req req = *(const struct frame_survey_req *)buf;
            led_set(PIN_LED_RED, true);
            run_survey(cfg, &req, &seq);
            led_set(PIN_LED_RED, false);
        }
    }
}
