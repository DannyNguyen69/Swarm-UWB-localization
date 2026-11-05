#!/usr/bin/env python3
"""Real-to-sim hover test: the real UWB error, live from the tag, on a simulated x500 in Gazebo.

    sim/run_sitl.sh                                     # terminal 1 (HEADLESS=1 for no window)
    sim/.venv/bin/python sim/uwb_real2sim.py --setup    # once: PX4 params for EV + range finder
    # restart sim/run_sitl.sh so the EKF starts with the new params, then:
    sim/.venv/bin/python sim/uwb_real2sim.py            # live tag, 30 s hover
    sim/.venv/bin/python sim/uwb_real2sim.py --replay data/ev_run20.xlsx

The tag stays still on the table, so its position is not the drone's. What is real is its error:

    EV sent to PX4 = Gazebo ground truth + (UWB fix - mean UWB fix while static)

at --rate Hz, each EV being the mean of the UWB fixes in its window; windows without a fix send
nothing, so real dropouts reach PX4 too. EKF2 fuses it as external vision (horizontal position,
EKF2_EV_CTRL = 1), height comes from the simulated range finder, yaw from the simulated mag.
The drone is armed in offboard mode, held at --alt for --hover s, landed, and the hold error
(ground truth vs setpoint) and the EKF error (estimate vs ground truth) are plotted.
"""

import argparse
import csv
import math
import os
import struct
import sys
import threading
import time

import numpy as np
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "host"))
sys.path.insert(0, os.path.join(ROOT, "ekf"))
import uwbproto as up  # noqa: E402
from uwb_viewer import AnchorMap, solve_2d  # noqa: E402

MODEL = "x500_lidar_down_0"
DATA_DIR = os.path.join(ROOT, "data")
FRESH_S = 0.3

EV_PARAMS = {  # name: (value, MAV_PARAM_TYPE)
    "EKF2_GPS_CTRL": (0, 6),     # no GNSS: UWB is the only horizontal position source
    "EKF2_EV_CTRL": (1, 6),      # EV horizontal position only
    "EKF2_HGT_REF": (2, 6),      # height reference: range finder
    "EKF2_RNG_CTRL": (2, 6),     # range finder always on
    "EKF2_EVP_NOISE": (0.1, 9),
    "EKF2_EV_DELAY": (0.0, 9),
    "COM_RC_IN_MODE": (4, 6),    # no RC in this test
    "COM_RCL_EXCEPT": (4, 6),    # no RC-loss failsafe in offboard
    "NAV_RCL_ACT": (0, 6),
    "NAV_DLL_ACT": (0, 6),
}


# ---------------------------------------------------------------- ground truth (Gazebo)

class GroundTruth:
    def __init__(self):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        self.lock = threading.Lock()
        self.ned = None
        self.node = Node()
        if not self.node.subscribe(Pose_V, "/world/default/dynamic_pose/info", self._cb):
            sys.exit("Không đăng ký được topic tư thế của Gazebo. SITL đã chạy chưa?")

    def _cb(self, msg):
        for p in msg.pose:
            if p.name == MODEL:
                with self.lock:  # Gazebo ENU -> NED
                    self.ned = np.array([p.position.y, p.position.x, -p.position.z])

    def get(self):
        with self.lock:
            return None if self.ned is None else self.ned.copy()


# ---------------------------------------------------------------- UWB error source

class UwbError:
    """Collects UWB fixes and turns them into errors around the static mean."""

    def __init__(self, warmup_s):
        self.lock = threading.Lock()
        self.window = []          # fixes since the last EV: (x, y, var_x, var_y)
        self.warm = []
        self.ref = None
        self.warmup_s = warmup_s
        self.t0 = None
        self.fixes = 0
        self.info = ""
        self.layout_ready = False  # set once the anchor layout to fly with is known
        self.frozen = False        # layout and reference fixed for the rest of the run

    def reset(self):
        """The anchor layout changed before take-off: start the static average again."""
        with self.lock:
            self.warm, self.window, self.ref, self.t0 = [], [], None, None

    def add_fix(self, x, y, vx, vy):
        now = time.time()
        with self.lock:
            self.fixes += 1
            if self.ref is None:
                if not self.layout_ready:
                    return
                self.t0 = self.t0 or now
                self.warm.append((x, y))
                if now - self.t0 >= self.warmup_s and len(self.warm) > 20:
                    self.ref = np.mean(self.warm, axis=0)
                    self.frozen = True
                return
            self.window.append((x - self.ref[0], y - self.ref[1], vx, vy))

    def ready(self):
        return self.ref is not None

    def take(self):
        """Mean error of the fixes since the last call, or None if there was none."""
        with self.lock:
            w, self.window = self.window, []
        if not w:
            return None
        a = np.array(w)
        return a[:, 0].mean(), a[:, 1].mean(), a[:, 2].mean() / len(w), a[:, 3].mean() / len(w)


def fix_from(latest, anchors, amap, t):
    fresh = [(a, v[1]) for a, v in latest.items() if t - v[0] <= FRESH_S and a in anchors]
    if len(fresh) < 3:
        return None
    pts = [anchors[a] for a, _ in fresh]
    hd = [amap.horizontal(d) for _, d in fresh]
    guess = (float(np.mean([p[0] for p in pts])) + 0.1, float(np.mean([p[1] for p in pts])) + 0.1)
    sol = solve_2d(pts, hd, guess)
    if not sol:
        return None
    x, y = sol[0], sol[1]
    H = np.array([np.array([x - ax, y - ay]) / max(math.hypot(x - ax, y - ay), 1e-6) for ax, ay in pts])
    cov = 0.05 ** 2 * np.linalg.inv(H.T @ H)
    return x, y, cov[0, 0], cov[1, 1]


def live_tag(src, port, baud):
    import serial
    amap = AnchorMap(os.path.join(ROOT, "host", "anchors.toml"))
    parser, pairs, status, latest = up.Parser(), {}, None, {}
    start = time.time()
    with serial.Serial(port, baud, timeout=0.02) as s:
        s.reset_input_buffer()
        while True:
            # Fly with the survey layout if one arrives within a few seconds (the tag repeats the
            # table every 2 s), else with anchors.toml; never change it once the reference is set.
            if not src.layout_ready and (amap.using_survey or time.time() - start > 6):
                src.layout_ready = True
            for pkt in parser.feed(s.read(4096)):
                if isinstance(pkt, up.Status):
                    status = pkt
                elif isinstance(pkt, up.AnchorPair) and not src.frozen:
                    pairs[(pkt.id_a, pkt.id_b)] = pkt
                    before = (amap.layout_version, amap.layout.n_pairs if amap.layout else 0)
                    amap.update_survey(pairs, pkt.table_version, status.n_anchors if status else 0)
                    if (amap.layout_version, amap.layout.n_pairs if amap.layout else 0) != before:
                        src.reset()
                elif isinstance(pkt, up.Range) and pkt.ok:
                    t = time.time()
                    latest[pkt.anchor_id] = (t, pkt.range_mm / 1000.0)
                    f = fix_from(latest, amap.anchors, amap, t)
                    if f:
                        src.add_fix(*f)
            src.info = f"tag live, anchor {'tự calib' if amap.using_survey else 'anchors.toml'}"


def replay(src, path):
    from px4_ekf_sim import anchor_layout, load
    from px4_ev_sim import ev_samples
    amap = AnchorMap(os.path.join(ROOT, "host", "anchors.toml"))
    ranges, pairs = load(path)
    anchors, layout = anchor_layout(pairs, amap, False)
    ev = ev_samples(ranges, anchors, amap.anchor_height, amap.tag_height)
    src.info = f"replay {os.path.basename(path)} ({layout})"
    mean = np.mean([(e[1], e[2]) for e in ev], axis=0)
    src.warm, src.ref = [], mean  # the whole file is static: use its mean
    src.layout_ready = src.frozen = True
    dur = ev[-1][0]
    start = time.time()
    while True:
        for t, x, y, vx, vy, _ in ev:
            while time.time() - start < t:
                time.sleep(0.001)
            src.add_fix(x, y, vx, vy)
        start += dur


# ---------------------------------------------------------------- MAVLink

class Px4:
    def __init__(self):
        self.m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245, source_component=191)
        self.lock = threading.Lock()
        self.local = None      # (n, e, d)
        self.yaw = 0.0
        self.armed = False
        self.mode = None
        self.texts = []
        self.t0 = time.monotonic()
        print("Chờ PX4 SITL (UDP 14540)...")
        self.m.wait_heartbeat(timeout=30)
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()

    def us(self):
        return int((time.monotonic() - self.t0) * 1e6)

    def _heartbeat(self):
        while True:  # a GCS heartbeat satisfies the "no connection to the GCS" preflight check
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1)

    def _rx(self):
        while True:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if not msg:
                continue
            t = msg.get_type()
            if t == "TIMESYNC" and msg.tc1 == 0:
                self.m.mav.timesync_send(self.us() * 1000, msg.ts1)
            elif t == "LOCAL_POSITION_NED":
                with self.lock:
                    self.local = np.array([msg.x, msg.y, msg.z])
            elif t == "ATTITUDE":
                self.yaw = msg.yaw
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != 6:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = msg.custom_mode
            elif t == "STATUSTEXT":
                self.texts.append(msg.text)
                print(f"  PX4: {msg.text}")

    def set_param(self, name, value, ptype):
        # PX4 takes integer parameters bytewise: send the int32's bits in the float field.
        if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            value = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value, ptype)

    def command(self, cmd, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

    def send_ev(self, ned, var_xy):
        nan = float("nan")
        cov = [nan] * 21
        cov[0], cov[6], cov[11] = var_xy[0], var_xy[1], 1.0
        self.m.mav.odometry_send(self.us(), mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                 float(ned[0]), float(ned[1]), float(ned[2]), [nan, nan, nan, nan],
                                 nan, nan, nan, nan, nan, nan, cov, [nan] * 21,
                                 reset_counter=0, estimator_type=mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
                                 quality=100)

    def setpoint(self, ned, yaw):
        mask = 0b0000_1001_1111_1000  # position + yaw
        self.m.mav.set_position_target_local_ned_send(int((time.monotonic() - self.t0) * 1000), self.m.target_system,
                                                       self.m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                       mask, *map(float, ned), 0, 0, 0, 0, 0, 0, float(yaw), 0)


def custom_mode(main, sub=0):
    return (main << 16) | (sub << 24)


# ---------------------------------------------------------------- test

def run(args):
    gt = GroundTruth()
    px4 = Px4()
    src = UwbError(args.warmup)

    if args.replay:
        threading.Thread(target=replay, args=(src, args.replay), daemon=True).start()
    else:
        port, baud = args.port, args.baud
        if not port:
            tags = [b for b in up.detect() if b.kind == "uwb" and b.status.role == up.ROLE_TAG]
            if not tags:
                sys.exit("Không thấy tag (tắt uwb_viewer.py nếu đang chạy), hoặc dùng --replay.")
            port, baud = tags[0].port, tags[0].baud
        threading.Thread(target=live_tag, args=(src, port, baud), daemon=True).start()

    print(f"Lấy mẫu nhiễu UWB lúc đứng yên ({args.warmup:.0f} s)...")
    deadline = time.time() + args.warmup + 15
    while not src.ready():
        if time.time() > deadline:
            sys.exit(f"Không nhận được vị trí UWB ({src.fixes} lần giải). Tag có chạy và đủ 3 anchor không?")
        time.sleep(0.1)
    print(f"Nguồn nhiễu: {src.info}")

    period = 1.0 / args.rate
    log, phase = [], "ground"
    hold, t_start = None, time.time()
    t_hover_start = t_land = None
    next_ev = next_sp = next_cmd = time.time()
    ev_sent, last_ev = 0, None

    while True:
        now = time.time()
        truth = gt.get()
        if truth is None:
            time.sleep(0.01)
            continue

        if now >= next_ev:
            next_ev += period
            e = src.take()
            if e:
                ev = truth + np.array([e[0], e[1], 0.0])
                px4.send_ev(ev, (e[2], e[3]))
                ev_sent += 1
                last_ev = (now, ev)

        local = px4.local
        t = now - t_start
        if phase == "ground" and t > 4 and local is not None:
            hold = np.array([local[0], local[1], -args.alt])
            phase = "arming"
            print(f"Giữ vị trí ({hold[0]:.2f}, {hold[1]:.2f}) ở độ cao {args.alt} m. Offboard + arm...")
        if phase in ("arming", "hover") and now >= next_sp:
            next_sp += 0.05
            px4.setpoint(hold, px4.yaw if phase == "arming" else hold_yaw)
        if phase == "arming":
            hold_yaw = px4.yaw
            if t > 5 and now >= next_cmd:
                next_cmd = now + 1.0
                px4.command(176, 1, 6)  # MAV_CMD_DO_SET_MODE: custom, offboard
                px4.command(400, 1)     # arm
            if px4.armed and px4.mode == custom_mode(6):
                phase, t_hover_start = "hover", now
                print("Đã arm, đang cất cánh và giữ vị trí...")
            elif t > 20:
                sys.exit("Không arm được trong offboard. Xem thông báo PX4 ở trên.")
        if phase == "hover" and now - t_hover_start > args.hover:
            phase, t_land = "land", now
            px4.command(176, 1, 4, 6)  # AUTO.LAND
            print("Hạ cánh...")
        if phase == "land" and (not px4.armed or now - t_land > 15):
            break

        log.append((t, phase, *truth, *(local if local is not None else [np.nan] * 3),
                    *(hold if hold is not None else [np.nan] * 3),
                    *(last_ev[1][:2] if last_ev and now - last_ev[0] < 0.05 else [np.nan, np.nan])))
        time.sleep(0.005)

    print(f"Đã gửi {ev_sent} gói EV trong {time.time() - t_start:.0f} s")
    return log, src.info, t_hover_start - t_start


def analyse(log, info, t_hover, args):
    os.makedirs(DATA_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA_DIR, f"real2sim_{stamp}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", "true_n", "true_e", "true_d", "ekf_n", "ekf_e", "ekf_d",
                    "sp_n", "sp_e", "sp_d", "ev_n", "ev_e"])
        w.writerows(log)

    a = np.array([[r[0], *r[2:]] for r in log], dtype=float)
    ph = np.array([r[1] for r in log])
    t, tru, ekf, sp, ev = a[:, 0], a[:, 1:4], a[:, 4:7], a[:, 7:10], a[:, 10:12]
    # steady hover: from 8 s after the hover command until landing
    steady = (ph == "hover") & (t > t_hover + 8)
    hold_err = np.hypot(*(tru[steady, :2] - sp[steady, :2]).T) * 100
    ekf_err = np.hypot(*(ekf[steady, :2] - tru[steady, :2]).T) * 100
    z_true = -tru[steady, 2] * 100
    ev_t = t[~np.isnan(ev[:, 0])]
    gaps = np.diff(ev_t) * 1000 if len(ev_t) > 1 else np.array([0])

    print(f"\n=== Hover ổn định {t[steady][-1] - t[steady][0]:.0f} s (bỏ 8 s đầu sau khi cất cánh) ===")
    print(f"Nguồn nhiễu: {info}, EV {args.rate} Hz, EKF2_EVP_NOISE 0.1")
    print(f"Sai số giữ vị trí ngang (thật − điểm đặt): RMS {np.sqrt(np.mean(hold_err ** 2)):.1f} cm, "
          f"95% {np.percentile(hold_err, 95):.1f} cm, max {hold_err.max():.1f} cm")
    print(f"Sai số ước lượng EKF (EKF − thật):          RMS {np.sqrt(np.mean(ekf_err ** 2)):.1f} cm, "
          f"95% {np.percentile(ekf_err, 95):.1f} cm, max {ekf_err.max():.1f} cm")
    print(f"Độ cao thật khi hover: σ {z_true.std():.1f} cm (giá trị tuyệt đối lệch điểm đặt "
          f"{np.mean(z_true) - (-sp[steady, 2].mean() * 100):+.0f} cm do gốc độ cao của SITL, không liên quan UWB)")
    print(f"Log: {path}")
    plot(path, t, ph, tru, ekf, sp, ev, steady, hold_err, ekf_err, info, args)


def plot(path, t, ph, tru, ekf, sp, ev, steady, hold_err, ekf_err, info, args):
    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.suptitle(f"Real-to-sim: nhiễu UWB thật → PX4 SITL x500 (Gazebo) – {info}, EV {args.rate} Hz", fontsize=12)
    c = sp[steady][0, :2]

    a = ax[0][0]
    evs = steady & ~np.isnan(ev[:, 0])
    a.scatter((ev[evs, 1] - c[1]) * 100, (ev[evs, 0] - c[0]) * 100, s=3, color="#d1d5db", label="EV gửi PX4")
    a.plot((tru[steady, 1] - c[1]) * 100, (tru[steady, 0] - c[0]) * 100, color="#e5484d", lw=1, label="drone thật (Gazebo)")
    a.plot((ekf[steady, 1] - c[1]) * 100, (ekf[steady, 0] - c[0]) * 100, color="#1d4ed8", lw=0.8, alpha=0.7, label="EKF ước lượng")
    for rad, ls in ((5, "--"), (10, ":")):
        a.add_patch(plt.Circle((0, 0), rad, fill=False, ls=ls, color="#555"))
    a.plot(0, 0, "k+", ms=12)
    a.set_aspect("equal", adjustable="box")
    lim = max(12, float(np.nanmax(np.abs(np.r_[(tru[steady, :2] - c) * 100]))) * 1.2)
    a.set_xlim(-lim, lim)
    a.set_ylim(-lim, lim)
    a.set_xlabel("Đông (cm)")
    a.set_ylabel("Bắc (cm)")
    a.set_title("Nhìn từ trên khi hover (quanh điểm đặt), vòng 5 / 10 cm")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    b = ax[0][1]
    hz = np.hypot(*(tru[:, :2] - sp[:, :2]).T) * 100
    ez = np.hypot(*(ekf[:, :2] - tru[:, :2]).T) * 100
    b.plot(t, hz, color="#e5484d", lw=1, label="giữ vị trí: |thật − điểm đặt|")
    b.plot(t, ez, color="#1d4ed8", lw=1, label="ước lượng: |EKF − thật|")
    b.axvspan(t[steady][0], t[steady][-1], color="#16a34a", alpha=0.06, label="đoạn tính thống kê")
    b.set_ylim(0, max(20, np.percentile(hz[steady], 99) * 1.5))
    b.set_xlabel("t (s)")
    b.set_ylabel("cm")
    b.set_title("Sai số ngang theo thời gian")
    b.legend(fontsize=8)
    b.grid(alpha=0.3)

    d = ax[1][0]
    for i, (name, col) in enumerate((("Bắc", "#1d4ed8"), ("Đông", "#c2410c"))):
        d.plot(t[steady], (tru[steady, i] - sp[steady, i]) * 100, color=col, lw=1, label=f"{name} thật − điểm đặt")
    d.set_xlabel("t (s)")
    d.set_ylabel("cm")
    d.set_title("Lệch khỏi điểm đặt theo trục khi hover")
    d.legend(fontsize=8)
    d.grid(alpha=0.3)

    e = ax[1][1]
    e.plot(t, -tru[:, 2], color="#e5484d", lw=1.2, label="độ cao thật")
    e.plot(t, -ekf[:, 2], color="#1d4ed8", lw=1, ls="--", label="độ cao EKF")
    e.plot(t, -sp[:, 2], color="#555", lw=0.8, label="điểm đặt")
    e.set_xlabel("t (s)")
    e.set_ylabel("m")
    e.set_title("Độ cao (rangefinder)")
    e.legend(fontsize=8)
    e.grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.splitext(path)[0] + ".png"
    fig.savefig(png, dpi=110)
    print(f"Đồ thị: {png}")
    if not args.no_show:
        plt.show()


def setup():
    px4 = Px4()
    for name, (value, ptype) in EV_PARAMS.items():
        px4.set_param(name, value, ptype)
        time.sleep(0.05)
    time.sleep(1)
    print("Đã đặt tham số (PX4 tự lưu vào sim/rootfs). Khởi động lại sim/run_sitl.sh rồi chạy test.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--setup", action="store_true", help="chỉ đặt tham số PX4 cho EV + rangefinder")
    ap.add_argument("--replay", help="dùng nhiễu từ file .xlsx thay cho tag live")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--rate", type=float, default=50, help="tần số gửi EV (Hz)")
    ap.add_argument("--alt", type=float, default=1.5)
    ap.add_argument("--hover", type=float, default=30, help="thời gian giữ vị trí (s)")
    ap.add_argument("--warmup", type=float, default=3, help="giây lấy trung bình vị trí tag")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()
    if args.setup:
        setup()
        return
    log, info, t_hover = run(args)
    analyse(log, info, t_hover, args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nĐã dừng.")
