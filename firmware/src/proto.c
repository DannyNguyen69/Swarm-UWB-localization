#include "proto.h"
#include "uart.h"
#include <string.h>

uint16_t proto_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

void proto_send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t buf[4 + 255 + 2];
    buf[0] = PKT_SYNC0;
    buf[1] = PKT_SYNC1;
    buf[2] = len;
    buf[3] = type;
    memcpy(&buf[4], payload, len);
    uint16_t crc = proto_crc16(&buf[2], 2u + len);
    buf[4 + len] = (uint8_t)(crc & 0xFF);
    buf[5 + len] = (uint8_t)(crc >> 8);
    uart_write(buf, 6u + len);
}

int proto_poll_command(void)
{
    static uint8_t frame[4 + 32 + 2];
    static uint32_t n;
    int c;
    while ((c = uart_getc()) >= 0) {
        if (n == 0 && c != PKT_SYNC0)
            continue;
        if (n == 1 && c != PKT_SYNC1) {
            n = (c == PKT_SYNC0) ? 1 : 0;
            continue;
        }
        frame[n++] = (uint8_t)c;
        if (n == 3 && frame[2] > 32) {
            n = 0; /* commands are short; anything longer is noise */
            continue;
        }
        if (n >= 4 && n == 6u + frame[2]) {
            uint32_t len = frame[2];
            uint16_t crc = (uint16_t)(frame[4 + len] | (frame[5 + len] << 8));
            n = 0;
            if (crc == proto_crc16(&frame[2], 2u + len))
                return frame[3];
        }
    }
    return -1;
}
