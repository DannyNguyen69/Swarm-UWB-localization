"""UART protocol of the UWB firmware (firmware/src/proto.h) and board detection.

Frame: | 0xAA | 0x55 | len | type | payload[len] | crc16 LE |
CRC-16/CCITT-FALSE over len, type and payload.
"""

import glob
import os
import re
import struct
import time
from dataclasses import dataclass

import serial

SYNC = b"\xAA\x55"
PKT_RANGE = 0x01
PKT_ANCHOR_PAIR = 0x02
PKT_STATUS = 0x03
CMD_RESURVEY = 0x80   # host -> tag (firmware >= 0.3): forget the anchor table, survey again
CMD_SURVEY_MORE = 0x81  # host -> tag (firmware >= 0.7): survey only the missing pairs, keep the measured ones

ROLE_NONE, ROLE_ANCHOR, ROLE_TAG = 0, 1, 2
ROLE_NAMES = {ROLE_NONE: "chưa cấu hình", ROLE_ANCHOR: "anchor", ROLE_TAG: "tag"}

F_TIMEOUT = 1 << 0
F_RX_ERR = 1 << 1
F_NLOS = 1 << 2
F_INVALID = 1 << 3
F_TX_LATE = 1 << 4

DEFAULT_BAUD = 115200   # 921600 lost ~18% of packets through the J-Link OB VCOM
PANS_BAUD = 115200
PROBE_BAUDS = (115200, 921600)  # 921600: boards flashed before the default changed
DW1000_DEV_ID = 0xDECA0130

ANT_SRC_NAMES = {0: "mặc định", 1: "OTP nhà máy", 2: "UICR (calib riêng)"}
SURVEY_IDLE, SURVEY_WAITING, SURVEY_RUNNING, SURVEY_INCOMPLETE, SURVEY_PARTIAL, SURVEY_GAVE_UP = range(6)
SURVEY_NAMES = {SURVEY_IDLE: "xong", SURVEY_WAITING: "chờ đủ anchor",
                SURVEY_RUNNING: "đang đo", SURVEY_INCOMPLETE: "chưa đủ cặp để giải, sẽ thử lại",
                SURVEY_PARTIAL: "thiếu cặp nhưng giải được, không tự thử lại (bấm Calib lại)",
                SURVEY_GAVE_UP: "chưa đủ cặp để giải, đã thử đủ 10 lượt (bấm Calib lại)"}

_RANGE = struct.Struct("<HHHIihhB")
_STATUS = struct.Struct("<BBBBBIHHHHI")
_STATUS_V02 = struct.Struct("<BIBBB")   # appended in firmware 0.2
_ANCHOR_PAIR = struct.Struct("<HHHiHH")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(ptype: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), ptype]) + payload
    crc = crc16(body)
    return SYNC + body + bytes([crc & 0xFF, crc >> 8])


@dataclass
class Range:
    seq: int
    tag_id: int
    anchor_id: int
    t_us: int
    range_mm: int
    rx_power_dbm: float
    fp_power_dbm: float
    flags: int

    @property
    def ok(self) -> bool:
        return not self.flags & F_INVALID

    @classmethod
    def decode(cls, p: bytes) -> "Range":
        seq, tag, anc, t_us, rng, rx, fp, flags = _RANGE.unpack(p[: _RANGE.size])
        return cls(seq, tag, anc, t_us, rng, rx / 100.0, fp / 100.0, flags)


@dataclass
class Status:
    role: int
    node_id: int
    fw: str
    n_anchors: int
    uptime_ms: int
    n_ok: int
    n_fail: int
    tx_ant_dly: int
    rx_ant_dly: int
    dw_dev_id: int
    ant_src: int = None       # None: firmware older than 0.2
    tx_power: int = None
    alive_mask: int = 0
    pairs_valid: int = 0
    survey_state: int = SURVEY_IDLE

    @property
    def role_name(self) -> str:
        return ROLE_NAMES.get(self.role, f"? ({self.role})")

    @property
    def radio_ok(self) -> bool:
        return self.dw_dev_id == DW1000_DEV_ID

    @property
    def calib_text(self) -> str:
        if self.ant_src is None:
            return f"trễ anten {self.tx_ant_dly}"
        power = f"0x{self.tx_power:08X}" if self.tx_power else "mặc định chip"
        return (f"trễ anten {self.tx_ant_dly} ({ANT_SRC_NAMES.get(self.ant_src, '?')}), "
                f"công suất {power}")

    @classmethod
    def decode(cls, p: bytes) -> "Status":
        f = _STATUS.unpack(p[: _STATUS.size])
        s = cls(f[0], f[1], f"{f[2]}.{f[3]}", f[4], f[5], f[6], f[7], f[8], f[9], f[10])
        if len(p) >= _STATUS.size + _STATUS_V02.size:
            s.ant_src, s.tx_power, s.alive_mask, s.pairs_valid, s.survey_state = \
                _STATUS_V02.unpack(p[_STATUS.size:_STATUS.size + _STATUS_V02.size])
        return s


@dataclass
class AnchorPair:
    """Distance between two anchors from the survey (median of n samples)."""
    table_version: int
    id_a: int
    id_b: int
    dist_mm: int
    std_mm: int
    n_samples: int

    @classmethod
    def decode(cls, p: bytes) -> "AnchorPair":
        return cls(*_ANCHOR_PAIR.unpack(p[: _ANCHOR_PAIR.size]))


def flag_names(flags: int) -> str:
    names = [(F_TIMEOUT, "timeout"), (F_RX_ERR, "rx_err"), (F_NLOS, "nlos"),
             (F_INVALID, "invalid"), (F_TX_LATE, "tx_late")]
    return ",".join(n for bit, n in names if flags & bit) or "-"


class Parser:
    """Incremental decoder: feed() raw bytes, get back decoded packets."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]
                return out
            del self.buf[:i]
            if len(self.buf) < 4:
                return out
            n = self.buf[2]
            if len(self.buf) < 6 + n:
                return out
            body = bytes(self.buf[2:4 + n])
            crc = self.buf[4 + n] | (self.buf[5 + n] << 8)
            if crc16(body) != crc:
                self.crc_errors += 1
                del self.buf[:2]
                continue
            ptype, payload = body[1], body[2:]
            del self.buf[:6 + n]
            if ptype == PKT_RANGE and len(payload) >= _RANGE.size:
                out.append(Range.decode(payload))
            elif ptype == PKT_STATUS and len(payload) >= _STATUS.size:
                out.append(Status.decode(payload))
            elif ptype == PKT_ANCHOR_PAIR and len(payload) >= _ANCHOR_PAIR.size:
                out.append(AnchorPair.decode(payload))
            else:
                out.append((ptype, payload))


# ---------------------------------------------------------------- board detection

@dataclass
class Board:
    sn: str
    port: str
    kind: str = "unknown"  # "uwb" (our firmware), "pans", "unknown"
    status: Status = None
    baud: int = None
    pans_label: str = ""
    pans_mode: str = ""

    def describe(self) -> str:
        if self.kind == "uwb":
            s = self.status
            who = f"{s.role_name} {s.node_id}" if s.role != ROLE_NONE else s.role_name
            extra = f", {s.n_anchors or '?'} anchor" if s.role == ROLE_TAG else ""
            radio = "" if s.radio_ok else ", LỖI DW1000"
            calib = f", {s.calib_text}" if s.fw != "?" else ""
            return f"firmware UWB v{s.fw}: {who}{extra}{radio}{calib} ({self.baud} baud)"
        if self.kind == "pans":
            return f"PANS: label {self.pans_label}, mode {self.pans_mode}"
        if self.kind == "busy":
            return "cổng serial đang bị chương trình khác mở (uwb_viewer?), vẫn nạp được"
        return "không nhận diện được"


def list_ports():
    """J-Link VCOM ports of DWM1001-DEV boards, as (serial_number, port)."""
    found = []
    for link in sorted(glob.glob("/dev/serial/by-id/usb-SEGGER_J-Link_*-if00")):
        m = re.search(r"J-Link_(\d+)-if00", link)
        if m:
            found.append((m.group(1), link))
    return found


def _listen_status(port: str, baud: int, timeout: float):
    """Waits for a STATUS packet. STATUS comes once per second and can be lost on a noisy
    link, so a valid RANGE packet also identifies a tag (firmware version then unknown)."""
    seen_range = None
    with serial.Serial(port, baud, timeout=0.1) as s:
        s.reset_input_buffer()
        parser = Parser()
        t0 = time.time()
        while time.time() - t0 < timeout:
            for pkt in parser.feed(s.read(512)):
                if isinstance(pkt, Status):
                    return pkt
                if isinstance(pkt, Range):
                    seen_range = pkt
            if seen_range is None and parser.crc_errors == 0 and time.time() - t0 > 1.3:
                break  # silent at this baud rate: do not wait for the full timeout
    if seen_range:
        return Status(ROLE_TAG, seen_range.tag_id, "?", 0, 0, 0, 0, 0, 0, DW1000_DEV_ID)
    return None


def _probe_pans(port: str):
    with serial.Serial(port, PANS_BAUD, timeout=0.2) as s:
        s.reset_input_buffer()
        s.write(b"\r\r")
        time.sleep(1.0)
        s.read(4096)
        s.write(b"si\r")
        time.sleep(1.0)
        text = s.read(4096).decode(errors="ignore")
        s.write(b"quit\r")
    label = re.search(r"label=(\S+)", text)
    mode = re.search(r"mode: (\S+)", text)
    if label or mode:
        return (label.group(1) if label else "?"), (mode.group(1) if mode else "?")
    return None


def probe(sn: str, port: str) -> Board:
    try:
        return _probe(sn, port)
    except (serial.SerialException, OSError):
        # Typically uwb_viewer.py or another reader has the port open; flashing (over SWD) still works.
        return Board(sn=sn, port=port, kind="busy")


def _probe(sn: str, port: str) -> Board:
    b = Board(sn=sn, port=port)
    for baud in PROBE_BAUDS:
        st = _listen_status(port, baud, 2.5)
        if st:
            b.kind, b.status, b.baud = "uwb", st, baud
            return b
    pans = _probe_pans(port)
    if pans:
        b.kind, b.baud = "pans", PANS_BAUD
        b.pans_label, b.pans_mode = pans
    return b


def detect():
    return [probe(sn, port) for sn, port in list_ports()]


def resolve(port: str) -> str:
    return os.path.realpath(port)
