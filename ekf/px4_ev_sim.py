#!/usr/bin/env python3
"""Offline replay of recorded UWB ranges through the External Vision (EV) path of PX4 EKF2.

    ekf/px4_ev_sim.py                       # newest recording in data/
    ekf/px4_ev_sim.py data/ev_run.xlsx --no-show

Pipeline modelled (PLAN.md section 10, option B "External Vision"):

  tag ──UART──> Pi: 2D least-squares fix on every new range (>= 3 anchors fresh within 0.3 s),
                    covariance sigma_r^2 (H^T H)^-1 ──MAVLink ODOMETRY──> PX4 EKF2

EKF2 side follows release/1.18 aid_sources/external_vision/ev_pos_control.cpp and EKF2.cpp:
  - EKF2_EV_NOISE_MD = 0: variance = max(message variance, EKF2_EVP_NOISE^2, 0.01^2) per axis
  - innovation gate EKF2_EVP_GATE on each axis (test_ratio > 1 rejects the sample)
  - x and y fused one after the other; observation buffer and one sample per 10 ms update as
    in px4_ekf_sim.py; EV frame assumed already rotated to NED on the Pi (no EV yaw)
Stand-ins, because the recording has no IMU: constant-velocity prediction driven by
EKF2_ACC_NOISE, and a range-finder height reference (EKF2_RNG_NOISE) at the known tag height.
"""

import argparse
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from px4_ekf_sim import DATA_DIR, ROOT, anchor_layout, load, spread  # noqa: E402
from uwb_viewer import AnchorMap, solve_2d  # noqa: E402

FRESH_S = 0.3
SIGMA_RANGE = 0.05   # m, per-range noise the Pi assumes when it builds the ODOMETRY covariance


def ev_samples(ranges, anchors, anchor_h, tag_h):
    """What the Pi would send: (t, x, y, var_x, var_y, n_anchors) per new range."""
    dz = anchor_h - tag_h
    latest, out, guess = {}, [], None
    for t, aid, d in ranges:
        if aid not in anchors:
            continue
        latest[aid] = (t, d)
        fresh = [(a, v[1]) for a, v in latest.items() if t - v[0] <= FRESH_S]
        if len(fresh) < 3:
            continue
        pts = [anchors[a] for a, _ in fresh]
        hd = [math.sqrt(max(v * v - dz * dz, 0)) for _, v in fresh]
        guess = guess or (float(np.mean([q[0] for q in pts])) + 0.1, float(np.mean([q[1] for q in pts])) + 0.1)
        sol = solve_2d(pts, hd, guess)
        if not sol:
            continue
        x, y = sol[0], sol[1]
        guess = (x, y)
        H = np.array([[(x - ax) / max(math.hypot(x - ax, y - ay), 1e-6),
                       (y - ay) / max(math.hypot(x - ax, y - ay), 1e-6)] for ax, ay in pts])
        cov = SIGMA_RANGE ** 2 * np.linalg.inv(H.T @ H)
        out.append((t, x, y, cov[0, 0], cov[1, 1], len(fresh)))
    return out


def run(ev, tag_h, evp_noise, gate=5.0, acc_noise=0.35, rng_noise=0.1, predict_us=10000):
    dt = predict_us / 1e6
    # observation buffer rate limit (EKF2_DELAY_MAX 200 ms -> 10 ms with 10 ms updates)
    buf, newest, dropped = [], -1e9, 0
    for s in ev:
        ts = s[0] - dt / 2
        if ts >= newest + dt - 1e-9:
            buf.append((ts,) + s[1:])
            newest = ts
        else:
            dropped += 1

    x = np.array([buf[0][1], buf[0][2], tag_h, 0, 0, 0], dtype=float)
    P = np.diag([0.5, 0.5, 0.1, 0.25, 0.25, 0.25])
    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    q = acc_noise ** 2
    Q1 = q * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
    Q = np.zeros((6, 6))
    for i in range(3):
        Q[np.ix_([i, i + 3], [i, i + 3])] = Q1

    out = {"t": [], "pos": [], "vel": [], "std": [], "innov": [], "rejected": 0, "fused": 0, "dropped": dropped}
    k, next_h = 0, 0.0
    for step in range(int(buf[-1][0] / dt) + 1):
        t = step * dt
        x = F @ x
        P = F @ P @ F.T + Q
        if t >= next_h:  # range-finder height reference, 10 Hz
            K = P[:, 2] / (P[2, 2] + rng_noise ** 2)
            x = x + K * (tag_h - x[2])
            P = P - np.outer(K, P[2, :])
            next_h += 0.1
        if k < len(buf) and buf[k][0] <= t:
            _, mx, my, vx, vy, _ = buf[k]
            k += 1
            var = [max(vx, evp_noise ** 2, 0.01 ** 2), max(vy, evp_noise ** 2, 0.01 ** 2)]
            innov = [x[0] - mx, x[1] - my]
            S = [P[0, 0] + var[0], P[1, 1] + var[1]]
            ratios = [innov[i] ** 2 / (gate ** 2 * S[i]) for i in range(2)]
            out["innov"].append((t, innov[0], innov[1], gate * math.sqrt(S[0]), gate * math.sqrt(S[1])))
            if max(ratios) > 1.0:
                out["rejected"] += 1
            else:
                for i in range(2):  # fuseHorizontalPosition: one axis at a time
                    s = P[i, i] + var[i]
                    K = P[:, i] / s
                    x = x - K * (x[i] - (mx if i == 0 else my))
                    P = P - np.outer(K, P[i, :])
                P = 0.5 * (P + P.T)
                out["fused"] += 1
        out["t"].append(t)
        out["pos"].append(x[0:2].copy())
        out["vel"].append(x[3:5].copy())
        out["std"].append((math.sqrt(P[0, 0]), math.sqrt(P[1, 1])))
    return out


ODOMETRY_BYTES = 244  # MAVLink 2 ODOMETRY, full payload + framing (zero-trimming can make it shorter)


def decimate(ev, rate):
    """What the Pi sends at a fixed rate: the mean of the fixes in each 1/rate window, stamped at
    the mean sample time. rate 0 sends every fix as it is computed."""
    if not rate:
        return ev
    out, win, period = [], [], 1.0 / rate
    edge = ev[0][0] + period
    for s in ev:
        while s[0] >= edge:
            if win:
                a = np.array(win)
                n = len(win)
                out.append((a[:, 0].mean(), a[:, 1].mean(), a[:, 2].mean(),
                            a[:, 3].mean() / n, a[:, 4].mean() / n, int(a[:, 5].min())))
                win = []
            edge += period
        win.append(s)
    return out


def compare_rates(path, ev, anchors, th, layout_src, rates, no_show):
    settle = 1.0
    dur = ev[-1][0] - ev[0][0]
    print(f"{'Pi gửi':>8s} {'thực tế':>8s} {'trống max':>9s} {'băng thông':>11s} | "
          f"{'σx':>5s} {'σy':>5s} {'CEP50':>6s} {'R95':>5s} {'max':>5s} mm | σv mm/s | fuse/loại")
    runs = []
    for rate in rates:
        evr = decimate(ev, rate)
        r = run(evr, th, 0.1)
        pos = [p for t, p in zip(r["t"], r["pos"]) if t > settle]
        vel = np.array([v for t, v in zip(r["t"], r["vel"]) if t > settle])
        s = spread(pos)
        hz = len(evr) / dur
        gap = np.diff([e[0] for e in evr]).max() * 1000
        label = "mọi lần" if not rate else f"{rate} Hz"
        print(f"{label:>8s} {hz:6.1f}Hz {gap:7.0f}ms {hz * ODOMETRY_BYTES / 1000:8.1f}KB/s | "
              f"{s['sx']:5.1f} {s['sy']:5.1f} {s['cep50']:6.1f} {s['r95']:5.1f} {s['max']:5.1f}    | "
              f"{vel[:, 0].std() * 1000:3.0f}/{vel[:, 1].std() * 1000:3.0f} | {r['fused']}/{r['rejected']}")
        runs.append((label, hz, evr, r))
    print("Băng thông cổng UART (8N1): 115200 ≈ 11.5 KB/s, 460800 ≈ 46 KB/s, 921600 ≈ 92 KB/s")
    plot_rates(path, runs, layout_src, no_show)


def plot_rates(path, runs, layout_src, no_show):
    import matplotlib
    if no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cols = ["#9ca3af", "#e5484d", "#1d4ed8", "#16a34a"]
    ref = np.array(runs[0][3]["pos"])
    c = ref[np.array(runs[0][3]["t"]) > 1.0].mean(axis=0)
    fig, ax = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.suptitle(f"EV vào EKF2 theo tần số gửi – {os.path.basename(path)} – anchor: {layout_src}", fontsize=12)

    a = ax[0][0]
    ev0 = np.array([(e[1], e[2]) for e in runs[0][2]])
    a.scatter((ev0[:, 0] - c[0]) * 100, (ev0[:, 1] - c[1]) * 100, s=3, color="#d1d5db", label="mỗi lần giải (thô)")
    for (label, hz, evr, r), col in zip(runs, cols):
        p = np.array(r["pos"])[np.array(r["t"]) > 1.0]
        a.plot((p[:, 0] - c[0]) * 100, (p[:, 1] - c[1]) * 100, lw=1, color=col, label=f"EKF, gửi {label}")
    for rad, ls in ((5, "--"), (10, ":")):
        a.add_patch(plt.Circle((0, 0), rad, fill=False, ls=ls, color="#555"))
    a.set_aspect("equal", adjustable="box")
    a.set_xlim(-12, 12)
    a.set_ylim(-12, 12)
    a.set_title("Sai số vị trí EKF quanh tâm (cm), vòng 5 và 10 cm")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    for idx, (axis, name) in enumerate(((ax[0][1], "x"), (ax[1][0], "y"))):
        for (label, hz, evr, r), col in zip(runs, cols):
            t = np.array(r["t"])
            p = np.array(r["pos"])
            axis.plot(t, (p[:, idx] - c[idx]) * 100, lw=1, color=col, label=f"gửi {label}")
        axis.set_ylim(-6, 6)
        axis.set_xlabel("t (s)")
        axis.set_title(f"{name} của EKF theo thời gian (cm)")
        axis.legend(fontsize=8, ncol=len(runs))
        axis.grid(alpha=0.3)

    b = ax[1][1]
    labels = [lbl for lbl, *_ in runs]
    sx = [spread([p for t, p in zip(r["t"], r["pos"]) if t > 1.0]) for *_, r in runs]
    xs = np.arange(len(runs))
    b.bar(xs - 0.2, [s["cep50"] / 10 for s in sx], 0.4, label="CEP50", color="#60a5fa")
    b.bar(xs + 0.2, [s["r95"] / 10 for s in sx], 0.4, label="R95", color="#f87171")
    b2 = b.twinx()
    b2.plot(xs, [hz * ODOMETRY_BYTES / 1000 for _, hz, *_ in runs], "ko-", label="băng thông ODOMETRY")
    b2.axhline(11.5, color="#b45309", ls="--", lw=1)
    b2.annotate("giới hạn 115200 baud", (xs[-1], 11.8), fontsize=8, color="#b45309", ha="right")
    b2.set_ylabel("KB/s")
    b.set_xticks(xs, [f"{lbl}\n({hz:.0f} Hz)" for lbl, hz, *_ in runs])
    b.set_ylabel("cm")
    b.set_title("Sai số EKF và băng thông theo tần số gửi")
    b.legend(fontsize=8, loc="upper left")
    b2.legend(fontsize=8, loc="upper right")
    b.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    png = os.path.splitext(path)[0] + "_ev_rates.png"
    fig.savefig(png, dpi=110)
    print(f"\nĐồ thị: {png}")
    if not no_show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?")
    ap.add_argument("--anchors", default=os.path.join(ROOT, "host", "anchors.toml"))
    ap.add_argument("--rates", help="so sánh tần số gửi, ví dụ 0,70,50 (0 = gửi mọi lần giải)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    path = args.file or max(glob.glob(os.path.join(DATA_DIR, "*.xlsx")), key=os.path.getmtime)
    ranges, pairs = load(path)
    amap = AnchorMap(args.anchors)
    anchors, layout_src = anchor_layout(pairs, amap, False)
    ah, th = amap.anchor_height, amap.tag_height
    ev = ev_samples(ranges, anchors, ah, th)

    ev_t = np.array([s[0] for s in ev])
    gaps = np.diff(ev_t)
    print(f"File: {os.path.relpath(path, ROOT)}  |  anchor: {layout_src}")
    for a, (ax, ay) in sorted(anchors.items()):
        print(f"  A{a}: ({ax:.3f}, {ay:.3f})")
    print(f"Pi → PX4: {len(ev)} gói ODOMETRY trong {ranges[-1][0]:.1f} s ({len(ev) / ranges[-1][0]:.0f} Hz), "
          f"khoảng trống lớn nhất {gaps.max() * 1000:.0f} ms, "
          f"σ ODOMETRY TB {np.mean([math.sqrt(s[3]) for s in ev]) * 1000:.0f}/"
          f"{np.mean([math.sqrt(s[4]) for s in ev]) * 1000:.0f} mm\n")

    if args.rates:
        compare_rates(path, ev, anchors, th, layout_src, [int(r) for r in args.rates.split(",")], args.no_show)
        return

    settle = 1.0
    rows = [("Pi gửi (thô)", spread([(s[1], s[2]) for s in ev if s[0] > settle]), None)]
    results = {}
    for noise in (0.1, 0.05):
        r = run(ev, th, noise)
        results[noise] = r
        pos = [p for t, p in zip(r["t"], r["pos"]) if t > settle]
        vel = np.array([v for t, v in zip(r["t"], r["vel"]) if t > settle])
        rows.append((f"EKF EV, EVP_NOISE {noise} m", spread(pos), (r, vel)))

    print(f"{'':28s} {'σx':>6s} {'σy':>6s} {'CEP50':>6s} {'R95':>6s} {'max':>6s} mm | σv (mm/s) | fuse/loại")
    for name, s, extra in rows:
        line = f"{name:28s} {s['sx']:6.1f} {s['sy']:6.1f} {s['cep50']:6.1f} {s['r95']:6.1f} {s['max']:6.1f}"
        if extra:
            r, vel = extra
            line += f"    | {vel[:, 0].std() * 1000:4.0f} / {vel[:, 1].std() * 1000:4.0f} | {r['fused']}/{r['rejected']}"
        print(line)

    plot(path, anchors, ev, results, layout_src, args.no_show)


def plot(path, anchors, ev, results, layout_src, no_show):
    import matplotlib
    if no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {1: "#3b82f6", 2: "#a855f7", 3: "#14b8a6", 4: "#f59e0b"}
    r = results[0.1]
    pos = np.array(r["pos"])
    t = np.array(r["t"])
    c = pos[t > 1.0].mean(axis=0)
    evx = np.array([(s[0], s[1], s[2]) for s in ev])

    fig, ax = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.suptitle(f"Mô phỏng PX4 EKF2 đường External Vision – {os.path.basename(path)} – anchor: {layout_src}",
                 fontsize=12)

    a = ax[0][0]
    a.scatter(evx[:, 1], evx[:, 2], s=4, color="#c7cbd3", label="Pi gửi (ODOMETRY)")
    a.plot(pos[:, 0], pos[:, 1], color="#e5484d", lw=1.2, label="EKF (EVP_NOISE 0.1)")
    for k, (x, y) in anchors.items():
        a.plot(x, y, "s", color=colors.get(k), ms=9)
        a.annotate(f"A{k}", (x, y), xytext=(6, 6), textcoords="offset points", weight="bold")
    a.set_aspect("equal", adjustable="datalim")
    a.set_title("Bản đồ (m)")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    b = ax[0][1]
    b.scatter((evx[:, 1] - c[0]) * 100, (evx[:, 2] - c[1]) * 100, s=5, color="#c7cbd3", label="Pi gửi")
    for noise, col in ((0.1, "#e5484d"), (0.05, "#1d4ed8")):
        p = np.array(results[noise]["pos"])[np.array(results[noise]["t"]) > 1.0]
        b.plot((p[:, 0] - c[0]) * 100, (p[:, 1] - c[1]) * 100, lw=1, color=col, label=f"EKF EVP_NOISE {noise}")
    for rad, ls in ((5, "--"), (10, ":")):
        b.add_patch(plt.Circle((0, 0), rad, fill=False, ls=ls, color="#555"))
        b.annotate(f"{rad} cm", (rad * 0.71, rad * 0.71), fontsize=8)
    b.set_aspect("equal", adjustable="box")
    b.set_xlim(-15, 15)
    b.set_ylim(-15, 15)
    b.set_title("Sai số quanh vị trí trung bình (cm)")
    b.legend(fontsize=8)
    b.grid(alpha=0.3)

    d = ax[1][0]
    d.plot(evx[:, 0], (evx[:, 1] - c[0]) * 100, ".", ms=2, color="#93c5fd", label="x Pi")
    d.plot(evx[:, 0], (evx[:, 2] - c[1]) * 100, ".", ms=2, color="#fdba74", label="y Pi")
    d.plot(t, (pos[:, 0] - c[0]) * 100, color="#1d4ed8", lw=1.3, label="x EKF")
    d.plot(t, (pos[:, 1] - c[1]) * 100, color="#c2410c", lw=1.3, label="y EKF")
    sd = np.array(r["std"]) * 100
    d.fill_between(t, -sd[:, 0], sd[:, 0], color="#1d4ed8", alpha=0.1, label="±1σ EKF")
    d.set_ylim(-15, 15)
    d.set_xlabel("t (s)")
    d.set_title("Sai số vị trí theo thời gian (cm), EVP_NOISE 0.1")
    d.legend(fontsize=8, ncol=3)
    d.grid(alpha=0.3)

    e = ax[1][1]
    vel = np.array(r["vel"]) * 100
    e.plot(t, vel[:, 0], color="#1d4ed8", lw=1, label="vx EKF")
    e.plot(t, vel[:, 1], color="#c2410c", lw=1, label="vy EKF")
    e2 = e.twinx()
    e2.plot(evx[1:, 0], np.diff(evx[:, 0]) * 1000, color="#9ca3af", lw=0.8, label="khoảng cách giữa 2 gói EV (ms)")
    e2.set_ylabel("ms")
    e.set_ylim(-20, 20)
    e.set_xlabel("t (s)")
    e.set_ylabel("cm/s")
    e.set_title("Vận tốc EKF (cm/s) và nhịp gói EV (ms, xám)")
    e.legend(fontsize=8, loc="upper left")
    e2.legend(fontsize=8, loc="upper right")
    e.grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.splitext(path)[0] + "_ev.png"
    fig.savefig(png, dpi=110)
    print(f"\nĐồ thị: {png}")
    if not no_show:
        plt.show()


if __name__ == "__main__":
    main()
