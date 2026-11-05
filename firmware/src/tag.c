/* Tag: every 10 ms ranges one anchor (1..n_anchors in turn) and sends one RANGE packet per
 * attempt, successful or not.
 *
 * It also coordinates the anchor survey (PLAN.md section 9): once every anchor 1..n_anchors
 * answers, any anchor pair without a measured distance is surveyed by asking anchor a to range
 * anchor b. A survey runs only at boot and on CMD_RESURVEY / CMD_SURVEY_MORE from the host. An
 * anchor reboot (its uptime in REPORT goes backwards) only invalidates its pairs. At power-up at most SURVEY_MAX_ROUNDS rounds
 * run; after that the host asks for more (only while disarmed), so ranging is never paused in flight.
 * The table goes to the host as ANCHOR_PAIR packets, repeated every 2 s.
 */
#include "node.h"
#include "twr.h"
#include "proto.h"
#include "hw.h"
#include "board.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#define RANGE_PERIOD_US       10000u
#define ALIVE_US              1000000u  /* anchor counts as alive if it answered this recently */
#define SURVEY_SAMPLES        100
#define SURVEY_TRIES          3
#define SURVEY_RETRY_US       30000000u /* between rounds while the layout is not usable yet */
#define SURVEY_MAX_ROUNDS     2         /* after power-up only; each round pauses ranging for several seconds */
#define SURVEY_MIN_OK         20        /* a median of 20 is still solid; weak links rarely reach 50 */
#define SURVEY_RX_SLICE_UUS   60000u
#define SURVEY_WAIT_PER_SAMPLE_US 6000u
#define TABLE_REPEAT_US       2000000u

struct pair {
    bool valid;
    int32_t dist_mm;
    uint16_t std_mm;
    uint16_t n;
};

static struct pair table[MAX_ANCHORS + 1][MAX_ANCHORS + 1]; /* [a][b], a < b */
static uint16_t table_version;
static uint32_t last_ok_us[MAX_ANCHORS + 1];
static bool seen[MAX_ANCHORS + 1];
static uint32_t last_uptime_ms[MAX_ANCHORS + 1];

static void invalidate_anchor(int id)
{
    for (int a = 1; a <= MAX_ANCHORS; a++)
        for (int b = a + 1; b <= MAX_ANCHORS; b++)
            if ((a == id || b == id) && table[a][b].valid) {
                table[a][b].valid = false;
                table_version++;
            }
}

static uint8_t alive_mask(const struct node_cfg *cfg, uint32_t now)
{
    uint8_t m = 0;
    for (int id = 1; id <= cfg->n_anchors; id++)
        if (seen[id] && now - last_ok_us[id] < ALIVE_US)
            m |= (uint8_t)(1u << (id - 1));
    return m;
}

static int count_valid(const struct node_cfg *cfg)
{
    int n = 0;
    for (int a = 1; a <= cfg->n_anchors; a++)
        for (int b = a + 1; b <= cfg->n_anchors; b++)
            n += table[a][b].valid;
    return n;
}

static void send_pair(int a, int b)
{
    struct pkt_anchor_pair p = {
        .table_version = table_version,
        .id_a = (uint16_t)a,
        .id_b = (uint16_t)b,
        .dist_mm = table[a][b].dist_mm,
        .std_mm = table[a][b].std_mm,
        .n_samples = table[a][b].n,
    };
    proto_send(PKT_ANCHOR_PAIR, &p, sizeof p);
}

/* Asks anchor a to range anchor b and waits for its PAIR_RESULT. */
static bool survey_pair(const struct node_cfg *cfg, int a, int b, uint8_t *seq)
{
    uint16_t me = ADDR_TAG(cfg->node_id);
    struct frame_survey_req req;
    frame_hdr_init(&req.h, (*seq)++, ADDR_ANCHOR(a), me, FN_SURVEY_REQ);
    req.target_id = (uint8_t)b;
    req.n_samples = SURVEY_SAMPLES;
    radio_send(&req, sizeof req, DWT_START_TX_IMMEDIATE);
    radio_wait(SYS_STATUS_TXFRS);
    radio_clear(SYS_STATUS_TXFRS);

    uint32_t start = hw_micros();
    uint32_t budget = SURVEY_SAMPLES * SURVEY_WAIT_PER_SAMPLE_US + 200000u;
    while (hw_micros() - start < budget) {
        uint8_t buf[32];
        uint16_t len = twr_receive(buf, sizeof buf, SURVEY_RX_SLICE_UUS);
        const struct frame_pair_result *res = (const struct frame_pair_result *)buf;
        if (len < sizeof *res || res->h.fn != FN_PAIR_RESULT || res->h.dst != me || res->h.src != ADDR_ANCHOR(a))
            continue;
        if (res->id_b != b || res->n_ok < SURVEY_MIN_OK)
            return false;
        table[a][b] = (struct pair){true, res->median_mm, res->std_mm, res->n_ok};
        table_version++;
        return true;
    }
    return false;
}

/* Surveys every missing pair; ranging pauses meanwhile (a few seconds for a full table). */
static bool survey_missing(const struct node_cfg *cfg, uint32_t dev_id, uint8_t *seq)
{
    struct survey_info info = {alive_mask(cfg, hw_micros()), (uint8_t)count_valid(cfg), SURVEY_RUNNING};
    node_send_status(cfg, dev_id, 0, 0, &info);
    led_set(PIN_LED_RED, true);

    bool complete = true;
    for (int a = 1; a <= cfg->n_anchors; a++) {
        for (int b = a + 1; b <= cfg->n_anchors; b++) {
            if (table[a][b].valid)
                continue;
            bool ok = false;
            for (int t = 0; t < SURVEY_TRIES && !ok; t++)
                ok = survey_pair(cfg, a, b, seq);
            if (ok)
                send_pair(a, b);
            complete &= ok;
        }
    }

    led_set(PIN_LED_RED, false);
    /* Ranging was paused: do not let that count as anchors going silent. */
    uint32_t now = hw_micros();
    for (int id = 1; id <= cfg->n_anchors; id++)
        if (seen[id])
            last_ok_us[id] = now;
    return complete;
}

void tag_run(const struct node_cfg *cfg, uint32_t dev_id)
{
    uint16_t me = ADDR_TAG(cfg->node_id);
    uint8_t seq = 0;
    uint16_t pkt_seq = 0;
    uint16_t n_ok = 0, n_fail = 0;
    uint32_t k = 0;
    uint32_t next = hw_micros() + RANGE_PERIOD_US;
    uint32_t next_status = hw_micros();
    uint32_t next_table = hw_micros() + TABLE_REPEAT_US;
    uint32_t survey_retry_at = hw_micros();
    bool survey_pending = true; /* set at boot and on CMD_RESURVEY */
    int survey_rounds = 0;
    int table_emit = -1; /* index of the next pair to repeat, -1 = idle */
    const uint8_t all_alive = (uint8_t)((1u << cfg->n_anchors) - 1u);
    const int n_pairs = cfg->n_anchors * (cfg->n_anchors - 1) / 2;
    /* 2n - 3 pairs fix a rigid 2D layout (5 of 6 for four anchors): good enough to stop retrying. */
    const int usable_pairs = cfg->n_anchors >= 3 ? 2 * cfg->n_anchors - 3 : n_pairs;

    for (;;) {
        while ((int32_t)(hw_micros() - next) < 0) {
        }
        uint32_t t_us = next;
        next += RANGE_PERIOD_US;
        /* If we fell behind (survey, debugger halt), skip ahead instead of bursting. */
        if ((int32_t)(hw_micros() - next) > 0)
            next = hw_micros() + RANGE_PERIOD_US;

        uint8_t anchor_id = (uint8_t)(1u + (k++ % cfg->n_anchors));
        struct twr_result r = twr_initiate(cfg, me, ADDR_ANCHOR(anchor_id), &seq);
        uint32_t now = hw_micros();

        struct pkt_range p = {
            .seq = pkt_seq++,
            .tag_id = cfg->node_id,
            .anchor_id = anchor_id,
            .t_us = t_us,
            .range_mm = r.range_mm,
            .rx_power_cdbm = r.rx_cdbm,
            .fp_power_cdbm = r.fp_cdbm,
            .flags = r.flags,
        };
        proto_send(PKT_RANGE, &p, sizeof p);

        if (r.flags & RANGE_F_INVALID) {
            n_fail++;
        } else {
            n_ok++;
            led_toggle(PIN_LED_BLUE);
            if (seen[anchor_id] && r.peer_uptime_ms < last_uptime_ms[anchor_id]) {
                /* Rebooted: it may have been moved, so its pairs are dropped. No automatic survey though:
                 * a weak anchor can brown out in flight and a survey would blank ranging for seconds.
                 * Re-measure on the ground with CMD_RESURVEY (scripts/resurvey.sh) or a tag power cycle. */
                invalidate_anchor(anchor_id);
            }
            seen[anchor_id] = true;
            last_uptime_ms[anchor_id] = r.peer_uptime_ms;
            last_ok_us[anchor_id] = now;
        }

        int cmd = proto_poll_command();
        if (cmd == CMD_RESURVEY) {
            for (int id = 1; id <= MAX_ANCHORS; id++)
                invalidate_anchor(id);
        }
        if (cmd == CMD_RESURVEY || cmd == CMD_SURVEY_MORE) {
            /* CMD_SURVEY_MORE keeps the pairs already measured, so weak pairs accumulate over rounds.
             * The host sends it only while the vehicle is disarmed. */
            survey_pending = true;
            survey_rounds = 0;
            survey_retry_at = now; /* runs as soon as every anchor answers */
        }

        uint8_t alive = alive_mask(cfg, now);
        int valid = count_valid(cfg);
        bool missing = valid < n_pairs;
        if (!missing)
            survey_pending = false;
        if (survey_pending && alive == all_alive && cfg->n_anchors >= 2 &&
            (int32_t)(now - survey_retry_at) >= 0) {
            bool complete = survey_missing(cfg, dev_id, &seq);
            valid = count_valid(cfg);
            survey_rounds++;
            /* No periodic re-surveys once the layout is usable: each one pauses ranging. Retry
             * only while the layout cannot be solved yet, and give up after a few rounds. */
            if (complete || valid >= usable_pairs || survey_rounds >= SURVEY_MAX_ROUNDS)
                survey_pending = false;
            survey_retry_at = hw_micros() + SURVEY_RETRY_US;
            next = hw_micros() + RANGE_PERIOD_US;
            next_table = hw_micros();
            continue;
        }

        if ((int32_t)(now - next_status) >= 0) {
            uint8_t state = !missing ? SURVEY_IDLE
                          : alive != all_alive && survey_pending ? SURVEY_WAITING
                          : survey_pending ? (survey_rounds ? SURVEY_INCOMPLETE : SURVEY_RUNNING)
                          : valid >= usable_pairs ? SURVEY_PARTIAL : SURVEY_GAVE_UP;
            struct survey_info info = {alive, (uint8_t)valid, state};
            node_send_status(cfg, dev_id, n_ok, n_fail, &info);
            n_ok = n_fail = 0;
            next_status += STATUS_PERIOD_US;
            led_toggle(PIN_LED_GREEN);
            continue; /* one extra packet per cycle keeps the UART well below capacity */
        }

        /* Repeat the table, one pair per cycle, so a host that connects late still gets it. */
        if (table_emit < 0 && valid > 0 && (int32_t)(now - next_table) >= 0)
            table_emit = 0;
        if (table_emit >= 0) {
            int idx = 0;
            bool sent = false;
            for (int a = 1; a <= cfg->n_anchors && !sent; a++)
                for (int b = a + 1; b <= cfg->n_anchors && !sent; b++, idx++)
                    if (idx >= table_emit && table[a][b].valid) {
                        send_pair(a, b);
                        table_emit = idx + 1;
                        sent = true;
                    }
            if (!sent) {
                table_emit = -1;
                next_table = now + TABLE_REPEAT_US;
            }
        }
    }
}
