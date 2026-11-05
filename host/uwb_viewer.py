#!/usr/bin/env python3
"""Live map of the tag in the browser. Runs on the laptop.

    host/uwb_viewer.py                       # tag plugged into this machine (auto-detect)
    host/uwb_viewer.py --tcp 192.168.1.50:7777   # tag on the Pi, via host/uwb_bridge.py
    then open http://localhost:8000

Decodes RANGE/STATUS packets, solves the 2D position from the latest range to each anchor
(coordinates from anchors.toml, reloaded when the file changes) and streams a snapshot to the
page with Server-Sent Events. Standard library + pyserial only.
"""

import argparse
import json
import math
import os
import socket
import statistics
import sys
import threading
import time
import tomllib
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import uwbproto as up  # noqa: E402
from survey_solver import format_report, solve_layout  # noqa: E402

WEB_DIR = os.path.join(HERE, "web")
FRESH_S = 0.3        # a range older than this is not used for the position fix
STATS_WINDOW_S = 1.0
PUSH_HZ = 20


# ---------------------------------------------------------------- anchors

class AnchorMap:
    """Anchor coordinates: from the tag's survey (source = "auto", the default) once a complete
    table has arrived, otherwise from the [anchors] table in anchors.toml."""

    def __init__(self, path):
        self.path = path
        self.mtime = None
        self.file_anchors = {}
        self.source = "auto"
        self.anchor_height = 0.0
        self.tag_height = 0.0
        self.error = None
        self.layout = None          # survey_solver.Layout from the tag's survey
        self.layout_version = None
        self.reload()

    @property
    def anchors(self):
        if self.source == "auto" and self.layout:
            return self.layout.anchors
        return self.file_anchors

    @property
    def using_survey(self):
        return self.source == "auto" and self.layout is not None

    def reload(self):
        try:
            mtime = os.path.getmtime(self.path)
            if mtime == self.mtime:
                return
            with open(self.path, "rb") as f:
                cfg = tomllib.load(f)
            self.file_anchors = {int(k): (float(v[0]), float(v[1])) for k, v in cfg.get("anchors", {}).items()}
            self.source = str(cfg.get("source", "auto"))
            self.anchor_height = float(cfg.get("anchor_height", 0.0))
            self.tag_height = float(cfg.get("tag_height", 0.0))
            self.mtime = mtime
            self.error = None
            print(f"anchors.toml: source {self.source}, {self.file_anchors}  "
                  f"(anchor_height {self.anchor_height} m, tag_height {self.tag_height} m)")
        except (OSError, KeyError, ValueError, tomllib.TOMLDecodeError) as e:
            self.error = f"{os.path.basename(self.path)}: {e}"

    def update_survey(self, pairs, version, n_expected):
        """pairs: {(a, b): AnchorPair}. Uses only pairs of the newest table version, which the tag
        repeats every 2 s, so pairs invalidated by an anchor reboot drop out."""
        current = {k: p.dist_mm / 1000.0 for k, p in pairs.items()
                   if p.table_version == version and p.n_samples > 0}
        # 2n - 3 pairs fix a rigid 2D layout of n anchors (5 of 6 for four); solve_layout says
        # which pair is missing and that nothing is left to cross-check.
        need = max(2 * n_expected - 3, 3) if n_expected else 3
        # Re-solve on a new table version, and also when more pairs of the same version arrive
        # (the tag repeats the table one pair at a time, so the 6th can come after the 5th).
        same = version == self.layout_version and self.layout and len(current) <= self.layout.n_pairs
        if same or len(current) < need:
            return
        lay = solve_layout(current)
        if lay:
            self.layout, self.layout_version = lay, version
            print(format_report(lay, pairs))

    def horizontal(self, d):
        dz = self.anchor_height - self.tag_height
        return math.sqrt(max(d * d - dz * dz, 0.0))


def solve_2d(anchors, ranges, guess):
    """Gauss-Newton least squares. anchors/ranges: lists of (x, y) and horizontal distances."""
    if len(anchors) < 3:
        return None
    x, y = guess
    for _ in range(15):
        a11 = a12 = a22 = b1 = b2 = 0.0
        for (ax, ay), d in zip(anchors, ranges):
            dx, dy = x - ax, y - ay
            r = math.hypot(dx, dy) or 1e-6
            jx, jy, res = dx / r, dy / r, r - d
            a11 += jx * jx
            a12 += jx * jy
            a22 += jy * jy
            b1 -= jx * res
            b2 -= jy * res
        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-9:
            return None
        sx = (a22 * b1 - a12 * b2) / det
        sy = (a11 * b2 - a12 * b1) / det
        x, y = x + sx, y + sy
        if abs(sx) + abs(sy) < 1e-4:
            break
    rms = math.sqrt(sum((math.hypot(x - ax, y - ay) - d) ** 2 for (ax, ay), d in zip(anchors, ranges))
                    / len(ranges))
    return x, y, rms


# ---------------------------------------------------------------- state

class State:
    def __init__(self, amap):
        self.amap = amap
        self.lock = threading.Lock()
        self.parser = up.Parser()
        self.latest = {}                   # anchor_id -> (host_time, Range)
        self.window = {}                   # anchor_id -> deque[(host_time, Range)]
        self.pkt_times = deque()
        self.last_seq = None
        self.lost_total = 0
        self.status = None
        self.status_time = 0.0
        self.position = None
        self.source = ""
        self.connected = False
        self.pairs = {}                    # (a, b) -> AnchorPair
        self.pair_version = None
        self.writer = None                 # sends bytes to the tag (serial write / socket sendall)
        self.last_command = None

    def send_command(self, ptype):
        """Sends a host -> tag command. Returns an error message, or None on success."""
        w = self.writer
        if not w:
            return "chưa kết nối tới tag"
        try:
            w(up.encode_frame(ptype))
        except OSError as e:
            return str(e)
        self.last_command = (ptype, time.time())
        return None

    def feed(self, data):
        now = time.time()
        with self.lock:
            for pkt in self.parser.feed(data):
                if isinstance(pkt, up.Range):
                    self._add_range(now, pkt)
                elif isinstance(pkt, up.Status):
                    self.status, self.status_time = pkt, now
                elif isinstance(pkt, up.AnchorPair):
                    self.pairs[(pkt.id_a, pkt.id_b)] = pkt
                    self.pair_version = pkt.table_version
                    n = self.status.n_anchors if self.status else 0
                    self.amap.update_survey(self.pairs, self.pair_version, n)

    def _add_range(self, now, r):
        if self.last_seq is not None:
            gap = (r.seq - self.last_seq - 1) & 0xFFFF
            if gap < 1000:
                self.lost_total += gap
        self.last_seq = r.seq
        self.pkt_times.append(now)
        self.window.setdefault(r.anchor_id, deque()).append((now, r))
        if r.ok:
            self.latest[r.anchor_id] = (now, r)
            self._solve(now)

    def _solve(self, now):
        pts, dists = [], []
        for aid, (t, r) in self.latest.items():
            if now - t <= FRESH_S and aid in self.amap.anchors:
                pts.append(self.amap.anchors[aid])
                dists.append(self.amap.horizontal(r.range_mm / 1000.0))
        if len(pts) < 3:
            self.position = None
            return
        guess = self.position[:2] if self.position else (
            sum(p[0] for p in pts) / len(pts) + 0.1, sum(p[1] for p in pts) / len(pts) + 0.1)
        sol = solve_2d(pts, dists, guess)
        self.position = (sol[0], sol[1], sol[2], len(pts)) if sol else None

    def _layout_info(self):
        lay = self.amap.layout
        info = {"source": "survey" if self.amap.using_survey else "file",
                "pref": self.amap.source, "version": self.amap.layout_version}
        if lay:
            info.update(rms_mm=lay.rms_mm, warnings=lay.warnings, redundancy=lay.redundancy,
                        pairs=[{"a": a, "b": b, "dist": self.pairs[(a, b)].dist_mm / 1000.0,
                                "std": self.pairs[(a, b)].std_mm, "resid_mm": r}
                               for (a, b), r in sorted(lay.residuals.items()) if (a, b) in self.pairs])
        return info

    def snapshot(self):
        now = time.time()
        self.amap.reload()
        with self.lock:
            while self.pkt_times and now - self.pkt_times[0] > STATS_WINDOW_S:
                self.pkt_times.popleft()
            anchors = {}
            ids = set(self.amap.anchors) | set(self.window)
            for aid in sorted(ids):
                w = self.window.get(aid, deque())
                while w and now - w[0][0] > STATS_WINDOW_S:
                    w.popleft()
                ok = [r for _, r in w if r.ok]
                lt = self.latest.get(aid)
                rng = lt[1].range_mm / 1000.0 if lt else None
                anchors[aid] = {
                    "xy": self.amap.anchors.get(aid),
                    "range": rng,
                    "hrange": self.amap.horizontal(rng) if rng is not None else None,
                    "age": (now - lt[0]) if lt else None,
                    "n_ok": len(ok),
                    "n_fail": len(w) - len(ok),
                    "std_mm": statistics.pstdev([r.range_mm for r in ok]) if len(ok) > 1 else None,
                    "rx": statistics.mean([r.rx_power_dbm for r in ok]) if ok else None,
                    "fp": statistics.mean([r.fp_power_dbm for r in ok]) if ok else None,
                    "nlos": sum(1 for r in ok if r.flags & up.F_NLOS),
                }
            st = self.status
            fresh_pos = self.position if self.latest and now - max(t for t, _ in self.latest.values()) < FRESH_S else None
            return {
                "t": now,
                "source": self.source,
                "connected": self.connected,
                "config_error": self.amap.error,
                "anchor_height": self.amap.anchor_height,
                "tag_height": self.amap.tag_height,
                "rate": len(self.pkt_times),
                "lost_total": self.lost_total,
                "crc_errors": self.parser.crc_errors,
                "anchors": anchors,
                "tag": None if not fresh_pos else {
                    "x": fresh_pos[0], "y": fresh_pos[1], "rms": fresh_pos[2], "n": fresh_pos[3]},
                "status": None if not st or now - self.status_time > 3 else {
                    "role": st.role_name, "id": st.node_id, "fw": st.fw, "n_anchors": st.n_anchors,
                    "uptime_s": st.uptime_ms / 1000.0, "radio_ok": st.radio_ok, "calib": st.calib_text,
                    "survey": None if st.ant_src is None else up.SURVEY_NAMES.get(st.survey_state, "?"),
                    "pairs_valid": st.pairs_valid, "alive_mask": st.alive_mask},
                "layout": self._layout_info(),
            }


# ---------------------------------------------------------------- data sources

def serial_reader(state, port, baud):
    import serial
    state.source = f"serial {up.resolve(port)} @ {baud}"
    while True:
        try:
            with serial.Serial(port, baud, timeout=0.02) as s:
                state.connected = True
                state.writer = s.write
                while True:
                    state.feed(s.read(4096))
        except Exception as e:  # unplugged, permission, ...
            state.connected = False
            state.writer = None
            print(f"serial: {e}; retrying")
            time.sleep(1)


def tcp_reader(state, host, port):
    state.source = f"tcp {host}:{port}"
    while True:
        try:
            with socket.create_connection((host, port), timeout=5) as s:
                s.settimeout(2)
                state.connected = True
                state.writer = s.sendall
                print(f"connected to bridge {host}:{port}")
                while True:
                    data = s.recv(4096)
                    if not data:
                        raise ConnectionError("bridge closed the connection")
                    state.feed(data)
        except (OSError, ConnectionError) as e:
            state.connected = False
            state.writer = None
            print(f"tcp: {e}; retrying")
            time.sleep(1)


# ---------------------------------------------------------------- web

def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._file("index.html", "text/html; charset=utf-8")
            elif self.path == "/events":
                self._events()
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/api/resurvey":
                self.send_error(404)
                return
            err = state.send_command(up.CMD_RESURVEY)
            print("resurvey requested from the web page" + (f": {err}" if err else ""))
            body = json.dumps({"ok": err is None, "error": err}).encode()
            self.send_response(200 if err is None else 503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _file(self, name, ctype):
            with open(os.path.join(WEB_DIR, name), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    msg = "data: " + json.dumps(state.snapshot()) + "\n\n"
                    self.wfile.write(msg.encode())
                    self.wfile.flush()
                    time.sleep(1.0 / PUSH_HZ)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--serial", help="serial port of a tag plugged into this machine")
    src.add_argument("--tcp", help="HOST:PORT of host/uwb_bridge.py on the Pi")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--anchors", default=os.path.join(HERE, "anchors.toml"))
    ap.add_argument("--http", type=int, default=8000, help="web port")
    args = ap.parse_args()

    state = State(AnchorMap(args.anchors))

    if args.tcp:
        host, _, port = args.tcp.rpartition(":")
        target, targs = tcp_reader, (state, host, int(port))
    else:
        port, baud = args.serial, args.baud
        if not port:
            tags = [b for b in up.detect() if b.kind == "uwb" and b.status.role == up.ROLE_TAG]
            if not tags:
                sys.exit("no tag found on this machine; plug one in, or use --serial / --tcp")
            port, baud = tags[0].port, tags[0].baud
            print(f"tag: S/N {tags[0].sn}, {tags[0].describe()}")
        target, targs = serial_reader, (state, port, baud)
    threading.Thread(target=target, args=targs, daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.http), make_handler(state))
    server.daemon_threads = True
    print(f"open http://localhost:{args.http}  (or http://<this-machine-ip>:{args.http} from a phone)")
    server.serve_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
