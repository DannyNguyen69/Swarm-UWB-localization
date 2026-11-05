// UART protocol of the UWB tag firmware (firmware/src/proto.h, host/uwbproto.py).
//   | 0xAA | 0x55 | len | type | payload[len] | crc16 LE |    CRC-16/CCITT-FALSE over len, type, payload
#pragma once
#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

namespace uwb {

enum PktType : uint8_t { PKT_RANGE = 0x01, PKT_ANCHOR_PAIR = 0x02, PKT_STATUS = 0x03, CMD_RESURVEY = 0x80, CMD_SURVEY_MORE = 0x81 };
enum RangeFlags : uint8_t { F_TIMEOUT = 1, F_RX_ERR = 2, F_NLOS = 4, F_INVALID = 8, F_TX_LATE = 16 };

struct Range {
    uint16_t seq = 0, tag_id = 0, anchor_id = 0;
    uint32_t t_us = 0;
    int32_t range_mm = 0;
    double rx_dbm = 0, fp_dbm = 0;
    uint8_t flags = 0;
    bool ok() const { return !(flags & F_INVALID); }
};

struct Status {
    uint8_t role = 0, node_id = 0, fw_major = 0, fw_minor = 0, n_anchors = 0;
    uint32_t uptime_ms = 0;
    uint16_t n_ok = 0, n_fail = 0, tx_ant_dly = 0, rx_ant_dly = 0;
    uint32_t dw_dev_id = 0;
    bool has_v02 = false;  // firmware >= 0.2 appends the fields below
    uint8_t ant_src = 0;
    uint32_t tx_power = 0;
    uint8_t alive_mask = 0, pairs_valid = 0, survey_state = 0;
};

struct AnchorPair {
    uint16_t table_version = 0, id_a = 0, id_b = 0;
    int32_t dist_mm = 0;
    uint16_t std_mm = 0, n_samples = 0;
};

using Packet = std::variant<Range, Status, AnchorPair>;

uint16_t crc16(const uint8_t* data, size_t len);
std::vector<uint8_t> encode_frame(uint8_t type, const std::vector<uint8_t>& payload = {});
const char* survey_state_name(uint8_t state);

// Incremental decoder: feed() raw bytes, get back the complete, CRC-valid packets.
class Parser {
public:
    std::vector<Packet> feed(const uint8_t* data, size_t len);
    size_t crc_errors() const { return crc_errors_; }

private:
    std::vector<uint8_t> buf_;
    size_t crc_errors_ = 0;
};

}  // namespace uwb
