#include "twr.h"
#include "proto.h"
#include "hw.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <math.h>

#define RX_BUF 32
#define NLOS_DB_CDB 600 /* rx - fp power above 6 dB hints at a blocked first path */

static uint32_t wait_rx(void)
{
    return radio_wait(SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
}

/* Waits for a frame of type fn from peer to me. Returns the length, or 0 and sets flags. */
static uint16_t expect(uint8_t *buf, uint16_t max, uint8_t fn, uint16_t peer, uint16_t me, uint8_t *flags)
{
    uint32_t status = wait_rx();
    radio_clear(SYS_STATUS_TXFRS);
    if (!(status & SYS_STATUS_RXFCG)) {
        *flags |= (status & SYS_STATUS_ALL_RX_TO) ? RANGE_F_TIMEOUT : RANGE_F_RX_ERR;
        radio_rx_recover();
        return 0;
    }
    radio_clear(SYS_STATUS_RXFCG);
    uint16_t len = radio_read(buf, max);
    const struct frame_hdr *h = (const struct frame_hdr *)buf;
    if (len < sizeof *h || h->fn != fn || h->src != peer || h->dst != me) {
        *flags |= RANGE_F_RX_ERR;
        return 0;
    }
    return len;
}

struct twr_result twr_initiate(const struct node_cfg *cfg, uint16_t me, uint16_t peer, uint8_t *seq)
{
    struct twr_result r = {.flags = RANGE_F_INVALID};
    uint8_t buf[RX_BUF];

    struct frame_hdr poll;
    frame_hdr_init(&poll, (*seq)++, peer, me, FN_POLL);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    radio_send(&poll, sizeof poll, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (expect(buf, sizeof buf, FN_RESP, peer, me, &r.flags) < sizeof(struct frame_resp))
        return r;
    const struct frame_resp *resp = (const struct frame_resp *)buf;
    uint32_t t2 = resp->t2, t3 = resp->t3;
    uint64_t t1 = radio_tx_ts();
    uint64_t t4 = radio_rx_ts();
    radio_rx_power(&r.rx_cdbm, &r.fp_cdbm);

    struct frame_hdr fin;
    frame_hdr_init(&fin, (*seq)++, peer, me, FN_FINAL);
    uint64_t t5 = radio_schedule_tx(t4, RESP_RX_TO_FINAL_TX_DLY_UUS, cfg->tx_ant_dly);
    dwt_setrxtimeout(REPORT_RX_TIMEOUT_UUS);
    if (radio_send(&fin, sizeof fin, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        r.flags |= RANGE_F_TX_LATE;
        return r;
    }

    if (expect(buf, sizeof buf, FN_REPORT, peer, me, &r.flags) < sizeof(struct frame_report))
        return r;
    const struct frame_report *rep = (const struct frame_report *)buf;
    uint32_t t6 = rep->t6;
    r.peer_uptime_ms = rep->uptime_ms;

    int64_t ra = (uint32_t)((uint32_t)t4 - (uint32_t)t1);
    int64_t db = (uint32_t)((uint32_t)t5 - (uint32_t)t4);
    int64_t da = (uint32_t)(t3 - t2);
    int64_t rb = (uint32_t)(t6 - t3);
    int64_t den = ra + rb + da + db;
    if (den <= 0)
        return r;
    double tof = (double)(ra * rb - da * db) / (double)den * DWT_TIME_UNITS;
    r.range_mm = (int32_t)lrint(tof * SPEED_OF_LIGHT * 1000.0);
    r.flags &= (uint8_t)~RANGE_F_INVALID;
    if (r.rx_cdbm - r.fp_cdbm > NLOS_DB_CDB)
        r.flags |= RANGE_F_NLOS;
    return r;
}

bool twr_respond(const struct node_cfg *cfg, uint16_t me, const struct frame_hdr *poll, uint8_t *seq)
{
    uint16_t initiator = poll->src;
    uint64_t t2 = radio_rx_ts();

    struct frame_resp resp;
    frame_hdr_init(&resp.h, (*seq)++, initiator, me, FN_RESP);
    uint64_t t3 = radio_schedule_tx(t2, POLL_RX_TO_RESP_TX_DLY_UUS, cfg->tx_ant_dly);
    resp.t2 = (uint32_t)t2;
    resp.t3 = (uint32_t)t3;

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
    if (radio_send(&resp, sizeof resp, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS)
        return false;

    uint8_t buf[RX_BUF];
    uint8_t flags = 0;
    if (expect(buf, sizeof buf, FN_FINAL, initiator, me, &flags) < sizeof(struct frame_hdr))
        return false;
    uint64_t t6 = radio_rx_ts();

    struct frame_report rep;
    frame_hdr_init(&rep.h, (*seq)++, initiator, me, FN_REPORT);
    rep.t6 = (uint32_t)t6;
    rep.uptime_ms = hw_micros() / 1000u;
    dwt_setrxtimeout(0);
    radio_send(&rep, sizeof rep, DWT_START_TX_IMMEDIATE);
    radio_wait(SYS_STATUS_TXFRS);
    radio_clear(SYS_STATUS_TXFRS);
    return true;
}

uint16_t twr_receive(void *buf, uint16_t max, uint16_t timeout_uus)
{
    dwt_setrxtimeout(timeout_uus);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    uint32_t status = wait_rx();
    if (!(status & SYS_STATUS_RXFCG)) {
        radio_rx_recover();
        return 0;
    }
    radio_clear(SYS_STATUS_RXFCG);
    uint16_t len = radio_read(buf, max);
    const struct frame_hdr *h = (const struct frame_hdr *)buf;
    if (len < sizeof *h || h->fc != FRAME_FC || h->pan != FRAME_PAN)
        return 0;
    return len;
}
