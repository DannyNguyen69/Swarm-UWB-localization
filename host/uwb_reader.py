#!/usr/bin/env python3
"""Read RANGE/STATUS packets from a tag and print per-second statistics.

    host/uwb_reader.py                  # auto-detect the tag
    host/uwb_reader.py --csv log.csv    # also log every RANGE packet
    host/uwb_reader.py --raw            # print every packet
"""

import argparse
import csv
import os
import statistics
import sys
import time
from collections import defaultdict

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import uwbproto as up  # noqa: E402


def pick_port(args):
    if args.port:
        return args.port, args.baud
    boards = [b for b in up.detect() if b.kind == "uwb"]
    tags = [b for b in boards if b.status.role == up.ROLE_TAG]
    if not boards:
        sys.exit("Không thấy board nào chạy firmware UWB. Dùng --port để chỉ định.")
    b = tags[0] if tags else boards[0]
    print(f"Dùng S/N {b.sn} ({up.resolve(b.port)}): {b.describe()}")
    return b.port, b.baud


class Stats:
    def __init__(self):
        self.reset()
        self.last_seq = None
        self.lost_total = 0

    def reset(self):
        self.per_anchor = defaultdict(lambda: {"ok": [], "fail": 0, "rx": [], "fp": [], "nlos": 0})
        self.t_us = []
        self.n = 0
        self.lost = 0

    def add(self, r: up.Range):
        if self.last_seq is not None:
            gap = (r.seq - self.last_seq - 1) & 0xFFFF
            if gap < 1000:
                self.lost += gap
                self.lost_total += gap
        self.last_seq = r.seq
        self.n += 1
        self.t_us.append(r.t_us)
        a = self.per_anchor[r.anchor_id]
        if r.ok:
            a["ok"].append(r.range_mm)
            a["rx"].append(r.rx_power_dbm)
            a["fp"].append(r.fp_power_dbm)
            if r.flags & up.F_NLOS:
                a["nlos"] += 1
        else:
            a["fail"] += 1

    def report(self, crc_errors):
        dts = [(b - a) & 0xFFFFFFFF for a, b in zip(self.t_us, self.t_us[1:])]
        jitter = statistics.pstdev(dts) if len(dts) > 1 else 0.0
        period = statistics.mean(dts) if dts else 0.0
        print(f"\n{time.strftime('%H:%M:%S')}  {self.n} gói/s  chu kỳ {period / 1000:.2f} ms "
              f"(jitter {jitter:.0f} us)  mất {self.lost} (tổng {self.lost_total})  lỗi CRC {crc_errors}")
        for aid in sorted(self.per_anchor):
            a = self.per_anchor[aid]
            ok = a["ok"]
            tot = len(ok) + a["fail"]
            if ok:
                mean = statistics.mean(ok)
                std = statistics.pstdev(ok) if len(ok) > 1 else 0.0
                print(f"  A{aid}: {len(ok):3d}/{tot:3d} ok  {mean / 1000:7.3f} m  ±{std:5.1f} mm  "
                      f"rx {statistics.mean(a['rx']):6.1f} dBm  fp {statistics.mean(a['fp']):6.1f} dBm  "
                      f"nlos {a['nlos']}")
            else:
                print(f"  A{aid}:   0/{tot:3d} ok  (không có phản hồi)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--csv", help="ghi từng gói RANGE ra file CSV")
    ap.add_argument("--raw", action="store_true", help="in từng gói")
    ap.add_argument("--duration", type=float, default=0, help="dừng sau N giây (0 = chạy mãi)")
    args = ap.parse_args()

    port, baud = pick_port(args)
    parser = up.Parser()
    stats = Stats()
    writer = None
    if args.csv:
        f = open(args.csv, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["host_time", "seq", "tag_id", "anchor_id", "t_us", "range_mm",
                         "rx_power_dbm", "fp_power_dbm", "flags"])

    t_start = time.time()
    next_report = t_start + 1.0
    with serial.Serial(port, baud, timeout=0.05) as s:
        s.reset_input_buffer()
        last_survey, pairs_seen = None, set()
        while not args.duration or time.time() - t_start < args.duration:
            for pkt in parser.feed(s.read(4096)):
                if isinstance(pkt, up.Range):
                    stats.add(pkt)
                    if writer:
                        writer.writerow([f"{time.time():.6f}", pkt.seq, pkt.tag_id, pkt.anchor_id, pkt.t_us,
                                         pkt.range_mm, pkt.rx_power_dbm, pkt.fp_power_dbm, pkt.flags])
                    if args.raw:
                        print(f"RANGE seq={pkt.seq} A{pkt.anchor_id} t={pkt.t_us} "
                              f"{pkt.range_mm} mm rx={pkt.rx_power_dbm} fp={pkt.fp_power_dbm} "
                              f"[{up.flag_names(pkt.flags)}]")
                elif isinstance(pkt, up.Status):
                    survey = up.SURVEY_NAMES.get(pkt.survey_state, "?") if pkt.ant_src is not None else None
                    if args.raw or pkt.role != up.ROLE_TAG or survey != last_survey:
                        print(f"STATUS {pkt.role_name} {pkt.node_id} v{pkt.fw} up {pkt.uptime_ms / 1000:.0f}s "
                              f"ok {pkt.n_ok} fail {pkt.n_fail} | {pkt.calib_text}"
                              + (f" | tự calib: {survey}, {pkt.pairs_valid} cặp, anchor sống "
                                 f"{bin(pkt.alive_mask)[2:].zfill(pkt.n_anchors)[::-1]}" if survey else ""))
                        last_survey = survey
                elif isinstance(pkt, up.AnchorPair):
                    key = (pkt.id_a, pkt.id_b, pkt.table_version)
                    if args.raw or key not in pairs_seen:
                        pairs_seen.add(key)
                        print(f"ANCHOR_PAIR v{pkt.table_version} A{pkt.id_a}–A{pkt.id_b}: "
                              f"{pkt.dist_mm / 1000:.3f} m ± {pkt.std_mm} mm ({pkt.n_samples} mẫu)")
            if time.time() >= next_report:
                if stats.n:
                    stats.report(parser.crc_errors)
                stats.reset()
                next_report += 1.0


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
