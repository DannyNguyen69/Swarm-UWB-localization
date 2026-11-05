#include "uwb_proto.hpp"

#include <algorithm>
#include <cstring>

namespace uwb {
namespace {

template <class T>
T rd(const uint8_t* p)  // little-endian host (aarch64, x86)
{
    T v;
    std::memcpy(&v, p, sizeof v);
    return v;
}

constexpr size_t kRangeLen = 19, kStatusLen = 21, kStatusV02Len = 29, kPairLen = 14;

Range decode_range(const uint8_t* p)
{
    Range r;
    r.seq = rd<uint16_t>(p + 0);
    r.tag_id = rd<uint16_t>(p + 2);
    r.anchor_id = rd<uint16_t>(p + 4);
    r.t_us = rd<uint32_t>(p + 6);
    r.range_mm = rd<int32_t>(p + 10);
    r.rx_dbm = rd<int16_t>(p + 14) / 100.0;
    r.fp_dbm = rd<int16_t>(p + 16) / 100.0;
    r.flags = p[18];
    return r;
}

Status decode_status(const uint8_t* p, size_t len)
{
    Status s;
    s.role = p[0];
    s.node_id = p[1];
    s.fw_major = p[2];
    s.fw_minor = p[3];
    s.n_anchors = p[4];
    s.uptime_ms = rd<uint32_t>(p + 5);
    s.n_ok = rd<uint16_t>(p + 9);
    s.n_fail = rd<uint16_t>(p + 11);
    s.tx_ant_dly = rd<uint16_t>(p + 13);
    s.rx_ant_dly = rd<uint16_t>(p + 15);
    s.dw_dev_id = rd<uint32_t>(p + 17);
    if (len >= kStatusV02Len) {
        s.has_v02 = true;
        s.ant_src = p[21];
        s.tx_power = rd<uint32_t>(p + 22);
        s.alive_mask = p[26];
        s.pairs_valid = p[27];
        s.survey_state = p[28];
    }
    return s;
}

AnchorPair decode_pair(const uint8_t* p)
{
    AnchorPair a;
    a.table_version = rd<uint16_t>(p + 0);
    a.id_a = rd<uint16_t>(p + 2);
    a.id_b = rd<uint16_t>(p + 4);
    a.dist_mm = rd<int32_t>(p + 6);
    a.std_mm = rd<uint16_t>(p + 10);
    a.n_samples = rd<uint16_t>(p + 12);
    return a;
}

}  // namespace

uint16_t crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

std::vector<uint8_t> encode_frame(uint8_t type, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> f{0xAA, 0x55, static_cast<uint8_t>(payload.size()), type};
    f.insert(f.end(), payload.begin(), payload.end());
    const uint16_t crc = crc16(f.data() + 2, f.size() - 2);
    f.push_back(static_cast<uint8_t>(crc & 0xFF));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    return f;
}

const char* survey_state_name(uint8_t s)
{
    switch (s) {
        case 0: return "xong";
        case 1: return "chờ đủ anchor";
        case 2: return "đang đo";
        case 3: return "chưa đủ cặp để giải, sẽ thử lại";
        case 4: return "thiếu cặp nhưng giải được";
        case 5: return "chưa đủ cặp, tag dừng đo (chờ lệnh đo tiếp)";
        default: return "?";
    }
}

std::vector<Packet> Parser::feed(const uint8_t* data, size_t len)
{
    buf_.insert(buf_.end(), data, data + len);
    std::vector<Packet> out;
    for (;;) {
        // find the sync bytes
        size_t i = 0;
        bool found = false;
        for (; i + 1 < buf_.size(); ++i) {
            if (buf_[i] == 0xAA && buf_[i + 1] == 0x55) {
                found = true;
                break;
            }
        }
        if (!found) {
            if (buf_.size() > 1) buf_.erase(buf_.begin(), buf_.end() - 1);  // keep a possible lone 0xAA
            return out;
        }
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i));
        if (buf_.size() < 4) return out;
        const size_t n = buf_[2];
        if (buf_.size() < 6 + n) return out;
        const uint16_t want = static_cast<uint16_t>(buf_[4 + n] | (buf_[5 + n] << 8));
        if (crc16(buf_.data() + 2, 2 + n) != want) {
            ++crc_errors_;
            buf_.erase(buf_.begin(), buf_.begin() + 2);
            continue;
        }
        const uint8_t type = buf_[3];
        const uint8_t* p = buf_.data() + 4;
        if (type == PKT_RANGE && n >= kRangeLen) out.emplace_back(decode_range(p));
        else if (type == PKT_STATUS && n >= kStatusLen) out.emplace_back(decode_status(p, n));
        else if (type == PKT_ANCHOR_PAIR && n >= kPairLen) out.emplace_back(decode_pair(p));
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(6 + n));
    }
}

}  // namespace uwb
