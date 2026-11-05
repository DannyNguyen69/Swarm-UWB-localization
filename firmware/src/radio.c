#include "radio.h"
#include "dw_port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <math.h>
#include <string.h>

/* Fastest standard configuration: channel 5, 6.8 Mbps, PRF 64 MHz, 128-symbol preamble. */
static dwt_config_t dw_config = {
    5,
    DWT_PRF_64M,
    DWT_PLEN_128,
    DWT_PAC8,
    10,
    10,
    0,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    (129 + 8 - 8),
};

/* Factory calibration of the DWM1001C (datasheet 2.1.4, OTP memory map). XTRIM and LDOTUNE
 * are already applied by dwt_initialise(); the antenna delay and TX power are not. */
#define OTP_TXCFG   0x19 /* CH5 PRF64 TX power (smart power format) */
#define OTP_ANTDLY  0x1C /* [15:0] antenna delay, [31:16] same for sleep mode */
#define OTP_XTRIM   0x1E /* [7:0] XTRIM, [23:16] PGDLY */
#define DEFAULT_PGDLY_CH5 0xC0

static bool otp_valid(uint32_t v)
{
    return v != 0 && v != 0xFFFFFFFFu;
}

/* Must run while the SPI is still slow: OTP reads at 8 MHz return 0xFFFFFFFF. */
static void apply_otp_calibration(struct node_cfg *cfg)
{
    uint32_t antdly = 0, txcfg = 0, xtrim = 0;
    dwt_otpread(OTP_ANTDLY, &antdly, 1);
    dwt_otpread(OTP_TXCFG, &txcfg, 1);
    dwt_otpread(OTP_XTRIM, &xtrim, 1);

    uint16_t otp_dly = (uint16_t)(antdly & 0xFFFF);
    if (cfg->ant_src != ANT_SRC_UICR && otp_dly != 0 && otp_dly != 0xFFFF) {
        /* Qorvo: the factory value is split evenly, the same for TX and RX. */
        cfg->tx_ant_dly = otp_dly;
        cfg->rx_ant_dly = otp_dly;
        cfg->ant_src = ANT_SRC_OTP;
    }

    if (otp_valid(txcfg)) {
        uint8_t pgdly = (uint8_t)((xtrim >> 16) & 0xFF);
        dwt_txconfig_t tx = {
            .PGdly = pgdly ? pgdly : DEFAULT_PGDLY_CH5,
            .power = txcfg,
        };
        dwt_configuretxrf(&tx);
        cfg->tx_power = txcfg;
    }
}

uint32_t radio_init(struct node_cfg *cfg)
{
    dw_port_init();
    dw_port_reset();

    dw_port_spi_slow();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
        return 0;

    uint32_t dev_id = dwt_readdevid();
    if (dev_id != DWT_DEVICE_ID)
        return 0;

    dwt_configure(&dw_config);
    apply_otp_calibration(cfg);
    dw_port_spi_fast();

    dwt_settxantennadelay(cfg->tx_ant_dly);
    dwt_setrxantennadelay(cfg->rx_ant_dly);
    return dev_id;
}

uint32_t radio_wait(uint32_t mask)
{
    uint32_t status;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & mask)) {
    }
    return status;
}

void radio_clear(uint32_t bits)
{
    dwt_write32bitreg(SYS_STATUS_ID, bits);
}

void radio_rx_recover(void)
{
    radio_clear(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXFCG);
    dwt_rxreset();
}

int radio_send(void *frame, uint16_t len, uint8_t mode)
{
    uint16_t total = (uint16_t)(len + FRAME_FCS);
    dwt_writetxdata(total, (uint8_t *)frame, 0);
    dwt_writetxfctrl(total, 0, 1);
    return dwt_starttx(mode);
}

uint16_t radio_read(void *buf, uint16_t max)
{
    uint16_t len = (uint16_t)(dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023);
    if (len < FRAME_FCS)
        return 0;
    len -= FRAME_FCS;
    if (len > max)
        return 0;
    dwt_readrxdata((uint8_t *)buf, len, 0);
    return len;
}

static uint64_t ts40(const uint8_t b[5])
{
    uint64_t t = 0;
    for (int i = 4; i >= 0; i--)
        t = (t << 8) | b[i];
    return t;
}

uint64_t radio_rx_ts(void)
{
    uint8_t b[5];
    dwt_readrxtimestamp(b);
    return ts40(b);
}

uint64_t radio_tx_ts(void)
{
    uint8_t b[5];
    dwt_readtxtimestamp(b);
    return ts40(b);
}

/* The delayed TX register ignores the low 9 bits, and the transmitted timestamp then gets
 * the TX antenna delay added, so the actual RMARKER time is computable in advance. */
uint64_t radio_schedule_tx(uint64_t ref_ts, uint32_t delay_uus, uint16_t tx_ant_dly)
{
    uint32_t tx_time = (uint32_t)((ref_ts + (uint64_t)delay_uus * UUS_TO_DWT_TIME) >> 8);
    dwt_setdelayedtrxtime(tx_time);
    return ((((uint64_t)(tx_time & 0xFFFFFFFEu)) << 8) + tx_ant_dly) & 0xFFFFFFFFFFull;
}

void radio_rx_power(int16_t *rx_cdbm, int16_t *fp_cdbm)
{
    dwt_rxdiag_t d;
    dwt_readdiagnostics(&d);
    const float A = 121.74f; /* PRF 64 MHz */
    float n2 = (float)d.rxPreamCount * (float)d.rxPreamCount;
    if (n2 <= 0.0f) {
        *rx_cdbm = INT16_MIN;
        *fp_cdbm = INT16_MIN;
        return;
    }
    float f1 = d.firstPathAmp1, f2 = d.firstPathAmp2, f3 = d.firstPathAmp3;
    float fp = 10.0f * log10f((f1 * f1 + f2 * f2 + f3 * f3) / n2) - A;
    float rx = 10.0f * log10f((float)d.maxGrowthCIR * 131072.0f / n2) - A;
    *rx_cdbm = (int16_t)lrintf(rx * 100.0f);
    *fp_cdbm = (int16_t)lrintf(fp * 100.0f);
}

void frame_hdr_init(struct frame_hdr *h, uint8_t seq, uint16_t dst, uint16_t src, uint8_t fn)
{
    h->fc = FRAME_FC;
    h->seq = seq;
    h->pan = FRAME_PAN;
    h->dst = dst;
    h->src = src;
    h->fn = fn;
}
