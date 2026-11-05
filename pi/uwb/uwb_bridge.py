#!/usr/bin/env python3
"""UWB tag (USB) -> 2D position -> MAVLink ODOMETRY -> ArduPilot (or PX4), through fc_router.

Runs on the Raspberry Pi next to dark-landing/build/fc_router, which owns the FC serial port
(/dev/ttyAMA5) and relays telemetry to QGC. This program only talks to the router's local UDP port:

    scripts/run_router.sh                       # terminal 1 (or scripts/run_all.sh starts both)
    scripts/run_uwb.sh                          # terminal 2
    scripts/measure_heading.sh                  # once per anchor layout: drone nose along A1 -> A2

Direct use (all options in --help):
    uwb_bridge.py --heading 87 --origin 21.0285,105.8542,10 --tag-z 0.10
    uwb_bridge.py --measure-heading

What it does:
  - reads RANGE / ANCHOR_PAIR / STATUS from the tag, solves a 2D least-squares fix on every new
    range (>= 3 anchors answered within 0.3 s), averages the fixes into --rate Hz windows
  - anchor layout: the tag's survey (anchors.toml source = "auto"), or anchors.toml; it can only
    change while the vehicle is DISARMED, never in flight
  - converts the UWB frame (A1 origin, x towards A2, y towards A4) to NED with --heading, the
    compass bearing of the A1 -> A2 edge
  - sends ODOMETRY with the position covariance and its own HEARTBEAT
  - sends nothing when fewer than 3 anchors answer, so the autopilot sees the loss and fails safe

ArduPilot (Copter-4.7.0 source) vs PX4 (1.18):
  ArduPilot: only MAV_FRAME_LOCAL_FRD is accepted (handled as NED, no rotation with VISO_TYPE=1);
             one error value sqrt(cov[0]+cov[6]+cov[11]) for all axes, so the unused z variance is 0;
             a valid attitude quaternion is expected, so the FC's own attitude is echoed back;
             needs an EKF origin, sent with SET_GPS_GLOBAL_ORIGIN until the FC reports one;
             EKF3 drops samples closer than 20 ms, hence the 30 Hz default
  PX4:       frame MAV_FRAME_LOCAL_NED; per-axis variance; x, y, z must all be finite or the sample
             is dropped; no quaternion needed (EV yaw not fused)
"""

import argparse
import math
import os
import sys
import threading
import time

import numpy as np
import serial

os.environ.setdefault("MAVLINK20", "1")  # ODOMETRY only exists in MAVLink 2
from pymavlink import mavutil  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# On the Pi, sync.sh copies the shared modules into uwb/lib; in the repo they live in host/.
for _p in (os.path.join(HERE, "lib"), os.path.join(os.path.dirname(ROOT), "host")):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
import uwbproto as up  # noqa: E402
from uwb_viewer import AnchorMap, solve_2d  # noqa: E402

FRESH_S = 0.3
DEFAULT_FC = "udpout:127.0.0.1:14540"   # fc_router --local udpin://127.0.0.1:14540


def default_anchors():
    for p in (os.path.join(ROOT, "config", "anchors.toml"), os.path.join(os.path.dirname(ROOT), "host", "anchors.toml")):
        if os.path.isfile(p):
            return p
    return os.path.join(ROOT, "config", "anchors.toml")


class Fixes:
    """Thread-safe accumulation of UWB fixes between two ODOMETRY messages."""

    def __init__(self):
        self.lock = threading.Lock()
        self.window = []
        self.total = 0
        self.anchors_fresh = 0

    def add(self, t, x, y, vx, vy, n):
        with self.lock:
            self.window.append((t, x, y, vx, vy))
            self.total += 1
            self.anchors_fresh = n

    def take(self):
        with self.lock:
            w, self.window = self.window, []
        if not w:
            return None
        a = np.array(w)
        n = len(w)
        # stamped at the mean fix time, not at send time: the window would otherwise add ~1/2 period of delay
        return a[:, 0].mean(), a[:, 1].mean(), a[:, 2].mean(), a[:, 3].mean() / n, a[:, 4].mean() / n


class Tag(threading.Thread):
    def __init__(self, port, baud, amap, fixes, sigma_range, range_offset):
        super().__init__(daemon=True)
        self.port, self.baud, self.amap, self.fixes = port, baud, amap, fixes
        self.sigma, self.offset = sigma_range, range_offset
        self.allow_layout_change = True  # cleared while armed
        self.latest = {}
        self.pairs = {}
        self.status = None
        self.ok = 0
        self.fail = 0

    def run(self):
        parser = up.Parser()
        while True:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.02) as s:
                    s.reset_input_buffer()
                    while True:
                        for pkt in parser.feed(s.read(4096)):
                            self.handle(pkt)
            except serial.SerialException as e:
                print(f"[tag] {e}; thử lại sau 1 s")
                time.sleep(1)

    def handle(self, pkt):
        if isinstance(pkt, up.Status):
            self.status = pkt
        elif isinstance(pkt, up.AnchorPair):
            self.pairs[(pkt.id_a, pkt.id_b)] = pkt
            if self.allow_layout_change:
                self.amap.update_survey(self.pairs, pkt.table_version, self.status.n_anchors if self.status else 0)
        elif isinstance(pkt, up.Range):
            if not pkt.ok:
                self.fail += 1
                return
            self.ok += 1
            t = time.monotonic()
            self.latest[pkt.anchor_id] = (t, pkt.range_mm / 1000.0 + self.offset)
            anchors = self.amap.anchors
            fresh = [(a, v[1]) for a, v in self.latest.items() if t - v[0] <= FRESH_S and a in anchors]
            if len(fresh) < 3:
                return
            pts = [anchors[a] for a, _ in fresh]
            hd = [self.amap.horizontal(d) for _, d in fresh]
            guess = (float(np.mean([p[0] for p in pts])) + 0.1, float(np.mean([p[1] for p in pts])) + 0.1)
            sol = solve_2d(pts, hd, guess)
            if not sol:
                return
            x, y = sol[0], sol[1]
            H = np.array([np.array([x - ax, y - ay]) / max(math.hypot(x - ax, y - ay), 1e-6) for ax, ay in pts])
            try:
                cov = self.sigma ** 2 * np.linalg.inv(H.T @ H)
            except np.linalg.LinAlgError:
                return
            self.fixes.add(t, x, y, cov[0, 0], cov[1, 1], len(fresh))


class Fc:
    """MAVLink link to the flight controller (through fc_router's UDP port or a serial port)."""

    ARDU, PX4 = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavutil.mavlink.MAV_AUTOPILOT_PX4

    def __init__(self, url, baud, fw, origin):
        self.fw = fw
        self.origin = origin            # (lat, lon, alt_m) for ArduPilot, None for PX4
        self.origin_set = False
        self.att_q = None               # FC attitude as (w, x, y, z), echoed back to ArduPilot
        self.yaw = None
        self.m = mavutil.mavlink_connection(url, baud=baud, dialect="common", source_system=1,
                                            source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        self.lock = threading.Lock()    # pymavlink's seq counter is not thread safe
        self.t0 = time.monotonic()
        self.armed = False
        self.connected = False
        self.height = None              # m above the EKF origin (ground), from LOCAL_POSITION_NED
        self.height_time = 0.0
        self.seen = {}                  # msgid -> last time received, to ask for streams only when missing
        self.connected_at = None
        self.pitch_roll = (0.0, 0.0)
        self.params = {}
        # The z in ODOMETRY is a constant 0 (it must be finite or EKF3 drops the sample). It is harmless only
        # while no EKF source set takes height from ExternalNav, so ArduPilot is checked before sending.
        self.z_state = True if fw == "px4" else None   # None = not verified yet, True = safe, False = blocked
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def us(self):
        return int((time.monotonic() - self.t0) * 1e6)

    def _want(self):
        """(message id, Hz) this program needs from the FC."""
        mm = mavutil.mavlink
        return ((mm.MAVLINK_MSG_ID_ATTITUDE, 20), (mm.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10))

    def request_interval(self, msg_id, hz):
        with self.lock:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    def _ensure_streams(self):
        """Ask only for messages that are not arriving. QGC sets its own rates on the same link and a
        blind re-request every second would keep overriding them."""
        now = time.monotonic()
        for msg_id, hz in self._want():
            if now - self.seen.get(msg_id, 0.0) > 3.0:
                self.request_interval(msg_id, hz)

    def _ensure_origin(self):
        if self.origin_set or self.fw != "ardupilot" or not self.origin:
            return
        mm = mavutil.mavlink
        with self.lock:
            # first see whether the FC already has one (e.g. QGC or an earlier run set it) ...
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mm.MAV_CMD_REQUEST_MESSAGE, 0, mm.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
        if time.monotonic() - self.connected_at > 3.0:
            lat, lon, alt = self.origin
            with self.lock:  # ... and set it if not. ArduPilot ignores this once an origin exists.
                self.m.mav.set_gps_global_origin_send(self.m.target_system, int(lat * 1e7), int(lon * 1e7),
                                                      int(alt * 1000), self.us())

    Z_PARAMS = ("EK3_SRC1_POSZ", "EK3_SRC2_POSZ", "EK3_SRC3_POSZ")

    def _check_z(self):
        if self.z_state is not None:
            return
        missing = [n for n in self.Z_PARAMS if n not in self.params]
        if missing:
            with self.lock:
                for n in missing:
                    self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, n.encode(), -1)
            return
        bad = [n for n in self.Z_PARAMS if int(self.params[n]) == 6]  # 6 = ExternalNav
        self.z_state = not bad
        vals = ",".join(str(int(self.params[n])) for n in self.Z_PARAMS)
        if bad:
            print(f"[fc] ⚠ {', '.join(bad)} = ExternalNav: EKF3 sẽ lấy độ cao từ ODOMETRY (luôn 0). "
                  f"KHÔNG GỬI. Đặt về 1 (baro) rồi chạy lại.")
        else:
            print(f"[fc] độ cao không lấy từ UWB (EK3_SRC1..3_POSZ = {vals})")

    def _heartbeat(self):
        while True:
            with self.lock:
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            if self.connected:
                self._ensure_streams()
                self._ensure_origin()
                if self.fw == "ardupilot":
                    self._check_z()
            time.sleep(1)

    def _rx(self):
        want_ap = self.ARDU if self.fw == "ardupilot" else self.PX4
        while True:
            try:
                msg = self.m.recv_match(blocking=True, timeout=1)
            except Exception as e:  # serial unplugged etc.
                print(f"[fc] {e}")
                time.sleep(1)
                continue
            if not msg:
                continue
            t = msg.get_type()
            self.seen[msg.get_msgId()] = time.monotonic()
            if t == "HEARTBEAT":
                if msg.autopilot == want_ap:
                    if not self.connected:
                        self.connected_at = time.monotonic()
                    # explicit: other heartbeats (QGC, the router itself) must not change the target
                    self.m.target_system, self.m.target_component = msg.get_srcSystem(), msg.get_srcComponent()
                    self.connected = True
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "ATTITUDE":
                self.yaw = msg.yaw
                cr, sr = math.cos(msg.roll / 2), math.sin(msg.roll / 2)
                cp, sp = math.cos(msg.pitch / 2), math.sin(msg.pitch / 2)
                cy, sy = math.cos(msg.yaw / 2), math.sin(msg.yaw / 2)
                self.att_q = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                              cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
                self.pitch_roll = (msg.pitch, msg.roll)
            elif t == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif t == "GPS_GLOBAL_ORIGIN":
                if not self.origin_set:
                    print(f"[fc] EKF origin: {msg.latitude / 1e7:.6f}, {msg.longitude / 1e7:.6f}")
                self.origin_set = True
            elif t == "LOCAL_POSITION_NED":
                self.height, self.height_time = -msg.z, time.monotonic()
            elif t == "TIMESYNC" and msg.tc1 == 0:
                with self.lock:
                    self.m.mav.timesync_send(self.us() * 1000, msg.ts1)
            elif t == "STATUSTEXT":
                print(f"[{self.fw}] {msg.text}")

    def ready(self):
        """ArduPilot needs the echoed attitude before a sample can be built."""
        return self.connected and self.z_state is True and (self.fw == "px4" or self.att_q is not None)

    def send(self, t_mono, n, e, var_n, var_e):
        nan = float("nan")
        cov = [nan] * 21
        if self.fw == "px4":
            # z must be finite or EKF2 drops the whole sample; it is not fused (EKF2_EV_CTRL = 1).
            frame, q = mavutil.mavlink.MAV_FRAME_LOCAL_NED, [nan] * 4
            cov[0], cov[6], cov[11] = var_n, var_e, 100.0
        else:
            # ArduPilot: LOCAL_FRD only; one error sqrt(cov0+cov6+cov11) for every axis, so z adds
            # nothing; angle covariance finite; attitude echoed (EK3_SRC1_YAW = compass, so unused).
            frame, q = mavutil.mavlink.MAV_FRAME_LOCAL_FRD, list(self.att_q)
            cov = [0.0] * 21
            cov[0], cov[6], cov[11] = var_n, var_e, 0.0
            cov[15] = cov[18] = cov[20] = 0.01
        with self.lock:
            self.m.mav.odometry_send(int((t_mono - self.t0) * 1e6), frame, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                     float(n), float(e), 0.0, q, nan, nan, nan, nan, nan, nan, cov, [nan] * 21,
                                     reset_counter=0, estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
                                     quality=100)


def find_tag():
    # One J-Link on the USB: use it without probing. Probing waits for packets and fails while the
    # tag is silent during its anchor survey.
    ports = up.list_ports()
    if len(ports) == 1:
        return ports[0][1], up.DEFAULT_BAUD
    for b in up.detect():
        if b.kind == "uwb" and b.status.role == up.ROLE_TAG:
            return b.port, b.baud
    sys.exit("Không thấy tag UWB trên USB.")


def measure_heading(fc, amap, nose, seconds=4.0):
    """Mean FC yaw while the drone sits with its nose along the line anchor a -> anchor b (--nose a:b).
    With numbering counter-clockwise seen from above, the line's direction in the UWB frame is theta
    (counter-clockwise from +x), its compass bearing is yaw = heading - theta, so
    heading (bearing of the UWB x axis) = yaw + theta. a:b = 1:2 is the x axis itself (theta = 0)."""
    a, b = (int(v) for v in nose.split(":"))
    for k in (a, b):
        if k not in amap.anchors:
            sys.exit(f"--nose {nose}: anchor {k} không có trong anchors.toml")
    (xa, ya), (xb, yb) = amap.anchors[a], amap.anchors[b]
    theta = math.degrees(math.atan2(yb - ya, xb - xa))
    print("Chờ FC (heartbeat qua router)...")
    t0 = time.time()
    while not fc.connected:
        if time.time() - t0 > 20:
            sys.exit("Không thấy FC. fc_router đã chạy chưa (scripts/run_router.sh)?")
        time.sleep(0.2)
    time.sleep(1.5)  # let _ensure_streams ask for ATTITUDE
    ys, last = [], None
    t0 = time.time()
    print(f"Đọc yaw của FC trong {seconds:.0f} s. Giữ drone đứng yên, mũi hướng từ A{a} sang A{b}...")
    while time.time() - t0 < seconds:
        if fc.yaw is not None and fc.yaw != last:
            ys.append(fc.yaw)
            last = fc.yaw
        time.sleep(0.02)
    if len(ys) < 5:
        sys.exit(f"Chỉ nhận {len(ys)} mẫu yaw từ FC (ATTITUDE chưa chảy).")
    c, s = np.mean(np.cos(ys)), np.mean(np.sin(ys))
    mean = math.degrees(math.atan2(s, c)) % 360
    yaw_nose = mean
    mean = (yaw_nose + theta) % 360
    spread = math.degrees(math.sqrt(max(-2 * math.log(max(math.hypot(c, s), 1e-9)), 0)))
    pitch, roll = (math.degrees(v) for v in fc.pitch_roll)
    print(f"\nyaw trung bình {yaw_nose:.1f}° (dao động {spread:.1f}°, {len(ys)} mẫu), pitch {pitch:.1f}° roll {roll:.1f}°")
    print(f"cạnh A{a}→A{b} lệch {theta:.1f}° so với trục x (A1→A2) nên hướng A1→A2 = yaw {yaw_nose:.1f}° + {theta:.1f}° = {mean:.1f}°")
    if spread > 3:
        print("⚠ Yaw dao động nhiều: la bàn bị nhiễu (kim loại, dây điện) hoặc drone đang bị chạm.")
    if abs(pitch) > 10 or abs(roll) > 10:
        print("⚠ Drone đang nghiêng nhiều, kết quả có thể lệch.")
    print(f"\n→ đặt  HEADING={mean:.0f}  trong config/uwb.env")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fw", choices=["px4", "ardupilot"], default="ardupilot", help="firmware của flight controller")
    ap.add_argument("--fc", default=DEFAULT_FC,
                    help=f"đường tới FC (mặc định {DEFAULT_FC}, tức cổng UDP nội bộ của fc_router). Cũng nhận "
                         "/dev/ttyAMA5 (khi không chạy router), udpin:0.0.0.0:14540 (SITL)")
    ap.add_argument("--fc-baud", type=int, default=921600, help="chỉ dùng khi --fc là cổng serial")
    ap.add_argument("--origin", help="ArduPilot: vĩ độ,kinh độ,độ cao(m) của chỗ bay, làm EKF origin "
                                     "(ArduPilot dùng nó để tính độ lệch từ của la bàn)")
    ap.add_argument("--tag-port", help="mặc định: tự tìm")
    ap.add_argument("--tag-baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--heading", type=float, help="hướng la bàn của cạnh A1→A2, độ (0 = Bắc, 90 = Đông). "
                                                  "Đo bằng --measure-heading")
    ap.add_argument("--resurvey", action="store_true",
                    help="ra lệnh cho tag đo lại khoảng cách giữa các anchor (như nút 'Calib lại toàn hệ' trên viewer), "
                         "rồi thoát. Chạy được khi bridge đang chạy. Chỉ làm khi drone nằm trên mặt đất")
    ap.add_argument("--nose", default="1:2", metavar="A:B",
                    help="dùng với --measure-heading: mũi drone đang hướng từ anchor A sang anchor B (mặc định 1:2). "
                         "Ví dụ 2:1 (ngược), 2:3 hoặc 1:4 (dọc theo trục y)")
    ap.add_argument("--measure-heading", action="store_true",
                    help="đọc yaw của FC khi mũi drone hướng từ A1 sang A2 rồi in ra giá trị --heading")
    ap.add_argument("--rate", type=float, default=None,
                    help="tần số gửi ODOMETRY (Hz). Mặc định 50 cho PX4, 30 cho ArduPilot: EKF3 bỏ mẫu cách "
                         "mẫu trước dưới 20 ms (extNavIntervalMin_ms), gửi sát 50 Hz thì jitter làm rơi mẫu")
    ap.add_argument("--sigma-range", type=float, default=0.05, help="nhiễu một khoảng cách (m) để tính covariance")
    ap.add_argument("--range-offset", type=float, default=0.0,
                    help="cộng vào mọi khoảng cách của tag (m), bù độ trễ anten tag chưa calib")
    ap.add_argument("--tag-z", type=float, default=0.0,
                    help="độ cao của tag so với sàn khi drone nằm trên mặt đất (m). Độ cao tag lúc bay = độ cao "
                         "EKF của FC + giá trị này")
    ap.add_argument("--anchors", default=default_anchors())
    args = ap.parse_args()

    if args.resurvey:
        port, baud = (args.tag_port, args.tag_baud) if args.tag_port else find_tag()
        with serial.Serial(port, baud, timeout=0.1) as s:
            s.write(up.encode_frame(up.CMD_RESURVEY))
        print(f"Đã gửi lệnh calib lại tới tag ({up.resolve(port)}). Tag ngừng đo vài giây rồi gửi bảng khoảng cách mới.")
        return

    origin = None
    if args.fw == "ardupilot" and not args.measure_heading:
        if not args.origin:
            sys.exit("ArduPilot cần --origin vĩ_độ,kinh_độ,độ_cao của chỗ bay (lấy từ điện thoại).")
        origin = tuple(float(v) for v in args.origin.split(","))

    fc = Fc(args.fc, args.fc_baud, args.fw, origin)
    if args.measure_heading:
        measure_heading(fc, AnchorMap(args.anchors), args.nose)
        return
    if args.heading is None:
        sys.exit("Thiếu --heading. Đo bằng: uwb_bridge.py --measure-heading (drone nằm yên, mũi hướng A1 → A2).")
    if args.rate is None:
        args.rate = 30 if args.fw == "ardupilot" else 50

    tag_port, tag_baud = (args.tag_port, args.tag_baud) if args.tag_port else find_tag()
    amap = AnchorMap(args.anchors)
    # Until the FC reports its height the drone is assumed to sit on the ground; the tag_height in the
    # file is for the NUC tools (a tag on a test stand), not for the drone.
    amap.tag_height = args.tag_z
    fixes = Fixes()
    tag = Tag(tag_port, tag_baud, amap, fixes, args.sigma_range, args.range_offset)
    tag.start()

    psi = math.radians(args.heading)
    c, s = math.cos(psi), math.sin(psi)
    period = 1.0 / args.rate
    next_tx = time.time()
    next_print = time.time() + 2
    sent = 0
    last_ne = None
    h0 = None  # EKF height on the ground (see below)
    fcname = {"px4": "PX4", "ardupilot": "ArduPilot"}[args.fw]
    print(f"tag {up.resolve(tag_port)} → {fcname} {args.fc}, {args.rate:g} Hz, hướng A1→A2 {args.heading:g}°, "
          f"anchors {args.anchors}")

    while True:
        now = time.time()
        if now < next_tx:
            time.sleep(min(next_tx - now, 0.005))
            continue
        next_tx += period
        if now - next_tx > 1:  # fell far behind (suspend etc.)
            next_tx = now + period

        # The anchor layout may only change on the ground.
        tag.allow_layout_change = not fc.armed
        # 3D ranges -> horizontal distances need the tag height above the floor: EKF height from the FC
        # plus the tag's height when the drone sits on the ground; before the FC reports one, tag_height
        # from anchors.toml.
        # The baro height drifts by tens of cm while the FC warms up, and a height error of dh moves the
        # horizontal distance by about dz/d_h * dh (large on the ground, where dz is biggest). So the
        # EKF height is taken relative to its average while disarmed on the ground, frozen once armed.
        if fc.height is not None and time.monotonic() - fc.height_time < 2.0:
            if not fc.armed:
                h0 = fc.height if h0 is None else 0.98 * h0 + 0.02 * fc.height
            amap.tag_height = max(fc.height - (h0 or 0.0), 0.0) + args.tag_z
            height_src = "FC" if h0 is not None else "FC, tuyệt đối"
        else:
            height_src = "anchors.toml"

        f = fixes.take()
        if f and fc.ready():
            t_fix, x, y, vx, vy = f
            # UWB (x to A2, y to A4, counter-clockwise) -> NED, origin at A1
            n, e = x * c + y * s, x * s - y * c
            var_n = vx * c * c + vy * s * s
            var_e = vx * s * s + vy * c * c
            fc.send(t_fix, n, e, var_n, var_e)
            last_ne = (n, e)
            sent += 1

        if now >= next_print:
            next_print = now + 2
            st = tag.status
            lay = "tự calib" if amap.using_survey else \
                ("anchors.toml" if amap.source == "manual" else "anchors.toml (tag chưa calib xong, dùng tọa độ trong file)")
            pos = f"N {last_ne[0]:+.2f} E {last_ne[1]:+.2f} m" if last_ne else "chưa có vị trí"
            print(f"[{time.strftime('%H:%M:%S')}] gửi {sent / 2:4.0f} Hz | {pos} | đo ok {tag.ok / 2:4.0f}/s lỗi {tag.fail / 2:3.0f}/s | "
                  f"anchor dùng {fixes.anchors_fresh} | tag cao {amap.tag_height:.2f} m ({height_src}) | bản đồ {lay}{' (khóa: đang bay)' if fc.armed else ''} | "
                  f"{fcname} {'ARMED' if fc.armed else 'disarmed' if fc.connected else 'chưa kết nối'}"
                  + (f" | origin {'OK' if fc.origin_set else 'chưa có'}" if args.fw == "ardupilot" else "")
                  + (f" | độ cao: {'không lấy từ UWB' if fc.z_state else 'ĐANG CHẶN GỬI' if fc.z_state is False else 'chưa kiểm tra'}"
                     if args.fw == "ardupilot" else "")
                  + (f" | tự calib tag: {up.SURVEY_NAMES.get(st.survey_state, '?')}" if st and st.ant_src is not None else ""))
            sent = tag.ok = tag.fail = 0


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
