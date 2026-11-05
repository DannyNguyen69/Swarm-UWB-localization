#!/usr/bin/env python3
"""Offline replay of recorded UWB ranges through a model of PX4 EKF2 ranging-beacon fusion.

    ekf/px4_ekf_sim.py                         # newest recording in data/
    ekf/px4_ekf_sim.py data/ekf_run.xlsx --no-show

What follows PX4 (release/1.18, src/modules/ekf2/EKF/aid_sources/ranging_beacon/ and
estimator_interface.cpp) exactly:
  - filter update every EKF2_PREDICT_US (10 ms); one beacon sample fused per update at most
  - observation buffer: a sample is dropped if it is closer than _min_obs_interval_us to the
    previous one (= EKF2_DELAY_MAX window / (buffer length - 1), 10 ms with defaults)
  - sample time shifted by -EKF2_RNGBC_DELAY - dt_ekf/2
  - R = max(range_accuracy^2, EKF2_RNGBC_NOISE^2); our tag sends no accuracy, so R = NOISE^2
  - innovation = predicted - measured range (3D, beacon at its altitude); H = d|b - p|/dp
  - test_ratio = innov^2 / (GATE^2 * S); rejected if > 1
  - Kalman gain on vertical position forced to 0 (height comes from the height reference)

What is a stand-in, because the recording has no IMU:
  - prediction is a constant-velocity model whose acceleration noise is EKF2_ACC_NOISE,
    instead of integrating IMU accelerations into the 24-state filter
  - the height reference is a range-finder-like measurement of the known tag height at 10 Hz
    with EKF2_RNG_NOISE (UWB cannot observe height with coplanar anchors)
So this shows how the beacon fusion, gating and noise settings behave on our real ranges; it
does not reproduce attitude, biases or IMU-driven dynamics.
"""

import argparse
import glob
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
import openpyxl

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "host"))
from survey_solver import solve_layout  # noqa: E402
from uwb_viewer import AnchorMap, solve_2d  # noqa: E402

DATA_DIR = os.path.join(ROOT, "data")


@dataclass
class Params:
    """PX4 parameter names; defaults are PX4's unless noted."""
    name: str
    EKF2_PREDICT_US: int = 10000
    EKF2_DELAY_MAX: float = 200.0       # ms
    EKF2_RNGBC_NOISE: float = 30.0      # m  (PX4 default, sized for LoRa beacons)
    EKF2_RNGBC_GATE: float = 5.0        # SD
    EKF2_RNGBC_DELAY: float = 0.0       # ms
    EKF2_ACC_NOISE: float = 0.35        # m/s^2
    EKF2_RNG_NOISE: float = 0.1         # m, height reference stand-in
    height_rate_hz: float = 10.0        # not a PX4 parameter


@dataclass
class Result:
    params: Params
    t: list = field(default_factory=list)          # filter time (s) at each update
    pos: list = field(default_factory=list)        # (x, y, z)
    std: list = field(default_factory=list)        # (sx, sy)
    innov: dict = field(default_factory=dict)      # anchor -> list of (t, innov, gate_bound, rejected)
    fused: dict = field(default_factory=dict)
    rejected: dict = field(default_factory=dict)
    dropped: int = 0                               # lost to the observation buffer rate limit


# ---------------------------------------------------------------- data

def load(path):
    wb = openpyxl.load_workbook(path, read_only=True)
    ranges = []
    for r in wb["Khoang cach"].iter_rows(min_row=2, values_only=True):
        _, _, _, aid, t_us, rng, _, _, _, ok = r[:10]
        if ok:
            ranges.append((t_us, aid, rng / 1000.0))
    pairs = {}
    if "Anchor calib" in wb.sheetnames:
        for r in wb["Anchor calib"].iter_rows(min_row=2, values_only=True):
            if r and isinstance(r[0], str) and "–" in r[0] and r[1]:
                a, b = (int(s.strip()[1:]) for s in r[0].split("–"))
                pairs[(a, b)] = float(r[1])
    # unwrap the 32-bit microsecond counter and start at 0
    t0, out, prev, wrap = ranges[0][0], [], ranges[0][0], 0
    for t_us, aid, d in ranges:
        if t_us < prev:
            wrap += 1 << 32
        prev = t_us
        out.append(((t_us + wrap - t0) / 1e6, aid, d))
    return out, pairs


def anchor_layout(pairs, amap, force_rect):
    """Survey layout if solvable, else a rectangle fitted to the measured pairs, else anchors.toml."""
    if pairs and not force_rect:
        lay = solve_layout(pairs)
        if lay:
            return lay.anchors, f"tự calib ({len(pairs)} cặp)"
    if pairs:
        sides_x = [pairs[k] for k in ((1, 2), (3, 4)) if k in pairs]
        sides_y = [pairs[k] for k in ((1, 4), (2, 3)) if k in pairs]
        if sides_x and sides_y:
            w, h = float(np.mean(sides_x)), float(np.mean(sides_y))
            return ({1: (0.0, 0.0), 2: (w, 0.0), 3: (w, h), 4: (0.0, h)},
                    f"hình chữ nhật {w:.2f} × {h:.2f} m khớp từ các cạnh đo được")
    return amap.anchors, "anchors.toml"


# ---------------------------------------------------------------- filter

def run(ranges, anchors, anchor_h, tag_h, p: Params):
    res = Result(p)
    dt = p.EKF2_PREDICT_US / 1e6
    imu_buf_len = max(2, math.ceil(p.EKF2_DELAY_MAX / (dt * 1e3)))
    obs_buf_len = min(round(p.EKF2_DELAY_MAX * 1.5 / (dt * 1e3)), imu_buf_len)
    min_obs_interval = (imu_buf_len - 1) * dt / (obs_buf_len - 1)

    # observation buffer: rate-limited exactly like setRangingBeaconData()
    buf, newest = [], -1e9
    for t, aid, d in ranges:
        if aid not in anchors:
            continue
        ts = t - p.EKF2_RNGBC_DELAY / 1e3 - dt / 2
        if ts >= newest + min_obs_interval - 1e-9:
            buf.append((ts, aid, d))
            newest = ts
        else:
            res.dropped += 1

    # initial state from a least-squares fix on the first ranges to every anchor
    first = {}
    for _, aid, d in ranges:
        first.setdefault(aid, d)
    dz = anchor_h - tag_h
    pts = [anchors[a] for a in first if a in anchors]
    hd = [math.sqrt(max(first[a] ** 2 - dz * dz, 0)) for a in first if a in anchors]
    guess = (float(np.mean([q[0] for q in pts])) + 0.1, float(np.mean([q[1] for q in pts])) + 0.1)
    fix = solve_2d(pts, hd, guess) or (*guess, 0)
    x = np.array([fix[0], fix[1], tag_h, 0.0, 0.0, 0.0])
    P = np.diag([1.0, 1.0, 0.1, 0.25, 0.25, 0.25])

    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    q = p.EKF2_ACC_NOISE ** 2
    Q1 = q * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
    Q = np.zeros((6, 6))
    for i in range(3):
        Q[np.ix_([i, i + 3], [i, i + 3])] = Q1

    t_end = buf[-1][0] if buf else 0
    k, n_steps = 0, int(t_end / dt) + 1
    next_height = 0.0
    for step in range(n_steps):
        t = step * dt
        x = F @ x
        P = F @ P @ F.T + Q

        # height reference (range-finder stand-in): z = tag_h
        if t >= next_height:
            H = np.zeros((1, 6))
            H[0, 2] = 1.0
            S = (H @ P @ H.T)[0, 0] + p.EKF2_RNG_NOISE ** 2
            K = (P @ H.T / S).ravel()
            x = x + K * (tag_h - x[2])
            P = P - np.outer(K, H @ P)
            next_height += 1.0 / p.height_rate_hz

        # pop_first_older_than: at most one beacon sample per filter update
        if k < len(buf) and buf[k][0] <= t:
            ts, aid, d = buf[k]
            k += 1
            bx, by = anchors[aid]
            delta = np.array([bx, by, anchor_h]) - x[0:3]
            pred = float(np.linalg.norm(delta))
            H = np.zeros((1, 6))
            H[0, 0:3] = -delta / max(pred, 1e-6)
            R = p.EKF2_RNGBC_NOISE ** 2
            S = (H @ P @ H.T)[0, 0] + R
            innov = pred - d
            test_ratio = innov ** 2 / (p.EKF2_RNGBC_GATE ** 2 * S)
            rejected = test_ratio > 1.0
            res.innov.setdefault(aid, []).append((t, innov, p.EKF2_RNGBC_GATE * math.sqrt(S), rejected))
            if rejected:
                res.rejected[aid] = res.rejected.get(aid, 0) + 1
            else:
                K = (P @ H.T / S).ravel()
                K[2] = 0.0  # PX4: altitude is handled by the height reference
                x = x - K * innov
                P = P - np.outer(K, H @ P)
                P = 0.5 * (P + P.T)
                res.fused[aid] = res.fused.get(aid, 0) + 1

        res.t.append(t)
        res.pos.append(x[0:3].copy())
        res.std.append((math.sqrt(P[0, 0]), math.sqrt(P[1, 1])))
    return res


def raw_fixes(ranges, anchors, anchor_h, tag_h):
    """Unfiltered 2D least-squares fixes, the same way the viewer computes them."""
    dz = anchor_h - tag_h
    latest, out, guess = {}, [], None
    for t, aid, d in ranges:
        if aid not in anchors:
            continue
        latest[aid] = (t, d)
        fresh = [(a, v[1]) for a, v in latest.items() if t - v[0] <= 0.3]
        if len(fresh) < 3:
            continue
        pts = [anchors[a] for a, _ in fresh]
        hd = [math.sqrt(max(v * v - dz * dz, 0)) for _, v in fresh]
        guess = guess or (float(np.mean([q[0] for q in pts])) + 0.1, float(np.mean([q[1] for q in pts])) + 0.1)
        sol = solve_2d(pts, hd, guess)
        if sol:
            guess = sol[:2]
            out.append((t, sol[0], sol[1]))
    return out


def spread(xy):
    a = np.asarray(xy)
    c = a.mean(axis=0)
    r = np.hypot(*(a - c).T) * 1000
    return {"sx": a[:, 0].std() * 1000, "sy": a[:, 1].std() * 1000,
            "cep50": float(np.percentile(r, 50)), "r95": float(np.percentile(r, 95)),
            "max": float(r.max()), "center": c}


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?")
    ap.add_argument("--anchors", default=os.path.join(ROOT, "host", "anchors.toml"))
    ap.add_argument("--rect", action="store_true", help="luôn khớp hình chữ nhật từ các cạnh đo được")
    ap.add_argument("--noise", type=float, default=0.1, help="EKF2_RNGBC_NOISE cho cấu hình đã chỉnh (m)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    path = args.file or max(glob.glob(os.path.join(DATA_DIR, "*.xlsx")), key=os.path.getmtime)
    ranges, pairs = load(path)
    amap = AnchorMap(args.anchors)
    anchors, layout_src = anchor_layout(pairs, amap, args.rect)
    ah, th = amap.anchor_height, amap.tag_height

    configs = [Params("PX4 mặc định (NOISE 30 m)"),
               Params(f"đã chỉnh (NOISE {args.noise} m)", EKF2_RNGBC_NOISE=args.noise)]
    results = [run(ranges, anchors, ah, th, p) for p in configs]
    raw = raw_fixes(ranges, anchors, ah, th)

    print(f"File: {os.path.relpath(path, ROOT)}  ({len(ranges)} khoảng cách hợp lệ, "
          f"{ranges[-1][0]:.1f} s)")
    print(f"Tọa độ anchor: {layout_src}")
    for a, (ax, ay) in sorted(anchors.items()):
        print(f"  A{a}: ({ax:.3f}, {ay:.3f})")
    print(f"Độ cao anchor {ah} m, tag {th} m\n")

    settle = 1.0  # s, skip convergence
    rs = spread([(x, y) for t, x, y in raw if t > settle])
    print(f"{'':32s} {'σx':>6s} {'σy':>6s} {'CEP50':>6s} {'R95':>6s} {'max':>6s}  (mm)   tâm")
    print(f"{'Thô (giải từng lần, như viewer)':32s} {rs['sx']:6.1f} {rs['sy']:6.1f} {rs['cep50']:6.1f} "
          f"{rs['r95']:6.1f} {rs['max']:6.1f}         ({rs['center'][0]:.3f}, {rs['center'][1]:.3f})")
    for r in results:
        s = spread([(p[0], p[1]) for t, p in zip(r.t, r.pos) if t > settle])
        fused = sum(r.fused.values())
        rej = sum(r.rejected.values())
        print(f"{'EKF ' + r.params.name:32s} {s['sx']:6.1f} {s['sy']:6.1f} {s['cep50']:6.1f} {s['r95']:6.1f} "
              f"{s['max']:6.1f}         ({s['center'][0]:.3f}, {s['center'][1]:.3f})")
        per = ", ".join(f"A{a} {r.fused.get(a, 0)}/{r.fused.get(a, 0) + r.rejected.get(a, 0)}"
                        for a in sorted(set(r.fused) | set(r.rejected)))
        print(f"{'':32s} fuse {fused}, loại {rej}, rơi ở bộ đệm {r.dropped} | {per} | "
              f"σ ước lượng cuối {r.std[-1][0] * 1000:.0f}/{r.std[-1][1] * 1000:.0f} mm")

    plot(path, ranges, anchors, raw, results, layout_src, args.no_show)


def plot(path, ranges, anchors, raw, results, layout_src, no_show):
    import matplotlib
    if no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {1: "#3b82f6", 2: "#a855f7", 3: "#14b8a6", 4: "#f59e0b"}
    cfg_colors = ["#9ca3af", "#e5484d"]
    tuned = results[-1]
    fig, ax = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.suptitle(f"Mô phỏng EKF2 ranging beacon – {os.path.basename(path)} – anchor: {layout_src}", fontsize=12)

    # 1. map
    a = ax[0][0]
    rx = np.array([(x, y) for _, x, y in raw])
    a.scatter(rx[:, 0], rx[:, 1], s=4, color="#c7cbd3", label="thô (từng lần giải)")
    for r, c in zip(results, cfg_colors):
        p = np.array(r.pos)
        a.plot(p[:, 0], p[:, 1], lw=1.2, color=c, label="EKF " + r.params.name)
    for k, (x, y) in anchors.items():
        a.plot(x, y, "s", color=colors.get(k), ms=9)
        a.annotate(f"A{k}", (x, y), xytext=(6, 6), textcoords="offset points", weight="bold")
    a.set_aspect("equal", adjustable="datalim")
    a.set_title("Bản đồ (m)")
    a.legend(fontsize=8, loc="best")
    a.grid(alpha=0.3)

    # 2. zoom: position around the tuned EKF mean, mm
    b = ax[0][1]
    pt = np.array(tuned.pos)
    c = pt[len(pt) // 10:, 0:2].mean(axis=0)
    b.scatter((rx[:, 0] - c[0]) * 1000, (rx[:, 1] - c[1]) * 1000, s=5, color="#c7cbd3", label="thô")
    for r, col in zip(results, cfg_colors):
        p = np.array(r.pos)[len(r.pos) // 10:]
        b.plot((p[:, 0] - c[0]) * 1000, (p[:, 1] - c[1]) * 1000, lw=1, color=col, label="EKF " + r.params.name)
    b.set_aspect("equal", adjustable="box")
    b.set_xlim(-300, 300)
    b.set_ylim(-300, 300)
    b.set_title("Phóng to quanh tâm EKF đã chỉnh (mm)")
    b.legend(fontsize=8)
    b.grid(alpha=0.3)

    # 3. x, y over time
    d = ax[1][0]
    rt = np.array([t for t, _, _ in raw])
    d.plot(rt, (rx[:, 0] - c[0]) * 1000, ".", ms=2, color="#93c5fd", label="x thô")
    d.plot(rt, (rx[:, 1] - c[1]) * 1000, ".", ms=2, color="#fdba74", label="y thô")
    t = np.array(tuned.t)
    d.plot(t, (pt[:, 0] - c[0]) * 1000, color="#1d4ed8", lw=1.3, label="x EKF")
    d.plot(t, (pt[:, 1] - c[1]) * 1000, color="#c2410c", lw=1.3, label="y EKF")
    sd = np.array(tuned.std) * 1000
    d.fill_between(t, -sd[:, 0], sd[:, 0], color="#1d4ed8", alpha=0.08, label="±1σ x EKF ước lượng")
    d.set_ylim(-300, 300)
    d.set_title(f"Theo thời gian, EKF {tuned.params.name} (mm, quanh tâm)")
    d.set_xlabel("t (s)")
    d.legend(fontsize=8, ncol=3)
    d.grid(alpha=0.3)

    # 4. innovations with the gate
    e = ax[1][1]
    for aid, rows in sorted(tuned.innov.items()):
        arr = np.array([(t, i, g) for t, i, g, _ in rows])
        rej = np.array([(t, i) for t, i, g, rj in rows if rj])
        e.plot(arr[:, 0], arr[:, 1] * 1000, ".", ms=2.5, color=colors.get(aid), label=f"A{aid}")
        if len(rej):
            e.plot(rej[:, 0], rej[:, 1] * 1000, "x", ms=6, color=colors.get(aid))
    g = np.array([(t, gb) for rows in tuned.innov.values() for t, _, gb, _ in rows])
    g = g[np.argsort(g[:, 0])]
    e.plot(g[:, 0], g[:, 1] * 1000, color="#555", lw=0.8, label=f"cổng ±{tuned.params.EKF2_RNGBC_GATE:g}σ")
    e.plot(g[:, 0], -g[:, 1] * 1000, color="#555", lw=0.8)
    e.set_ylim(-max(300, np.percentile(g[:, 1], 50) * 1300), max(300, np.percentile(g[:, 1], 50) * 1300))
    e.set_title("Innovation khoảng cách (mm), x = bị loại")
    e.set_xlabel("t (s)")
    e.legend(fontsize=8, ncol=3)
    e.grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.splitext(path)[0] + "_ekf.png"
    fig.savefig(png, dpi=110)
    print(f"\nĐồ thị: {png}")
    if not no_show:
        plt.show()


if __name__ == "__main__":
    main()
