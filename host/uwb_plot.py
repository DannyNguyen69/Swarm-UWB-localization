#!/usr/bin/env python3
"""Plot the noise of a recording made by uwb_record.py.

    host/uwb_plot.py                              # newest file in data/
    host/uwb_plot.py data/uwb_20261101_173821.xlsx
    host/uwb_plot.py --no-show                    # only save the PNG next to the .xlsx

Four panels: position scatter with CEP50/R95 circles, position deviation over time,
range deviation per anchor over time, and range deviation histograms.
"""

import argparse
import glob
import math
import os
import statistics

import openpyxl

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(os.path.dirname(HERE), "data")
ANCHOR_COLORS = {1: "#3b82f6", 2: "#a855f7", 3: "#14b8a6", 4: "#f59e0b"}


def load(path):
    wb = openpyxl.load_workbook(path, read_only=True)
    ranges = {}
    for r in wb["Khoang cach"].iter_rows(min_row=2, values_only=True):
        t, _, _, aid, _, rng, _, _, _, ok = r[:10]
        if ok:
            ranges.setdefault(aid, []).append((t, rng))
    fixes = []
    if "Vi tri" in wb.sheetnames:
        fixes = [r[:3] for r in wb["Vi tri"].iter_rows(min_row=2, values_only=True)]
    return ranges, fixes[1:]  # first fix is solved before every anchor has reported


def percentile(values, p):
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="file .xlsx (mặc định: file mới nhất trong data/)")
    ap.add_argument("--no-show", action="store_true", help="chỉ lưu PNG, không mở cửa sổ")
    args = ap.parse_args()

    path = args.file or max(glob.glob(os.path.join(DATA_DIR, "*.xlsx")), key=os.path.getmtime, default=None)
    if not path:
        raise SystemExit("Không có file nào trong data/. Ghi bằng host/uwb_record.py trước.")

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ranges, fixes = load(path)
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"UWB – {os.path.basename(path)}", fontsize=13)

    # 1. position scatter, in mm around the mean
    a = ax[0][0]
    if fixes:
        mx = statistics.mean(f[1] for f in fixes)
        my = statistics.mean(f[2] for f in fixes)
        dx = [(f[1] - mx) * 1000 for f in fixes]
        dy = [(f[2] - my) * 1000 for f in fixes]
        dr = [math.hypot(x, y) for x, y in zip(dx, dy)]
        cep, r95 = percentile(dr, 50), percentile(dr, 95)
        a.scatter(dx, dy, s=6, alpha=0.5, color="#e5484d")
        for r, name, style in ((cep, "CEP50", "-"), (r95, "R95", "--")):
            a.add_patch(plt.Circle((0, 0), r, fill=False, ls=style, color="#333"))
            a.annotate(f"{name} {r:.0f} mm", (r * 0.71, r * 0.71), fontsize=9)
        a.plot(0, 0, "k+", ms=12)
        a.set_title(f"Vị trí quanh tâm ({mx:.3f}, {my:.3f}) m\n"
                    f"σx {statistics.pstdev(dx):.0f} mm, σy {statistics.pstdev(dy):.0f} mm, {len(fixes)} điểm")
        a.set_xlabel("dx (mm)")
        a.set_ylabel("dy (mm)")
        a.set_aspect("equal", adjustable="datalim")
        a.grid(alpha=0.3)

        # 2. position deviation over time
        b = ax[0][1]
        t = [f[0] for f in fixes]
        b.plot(t, dx, lw=0.8, label="dx")
        b.plot(t, dy, lw=0.8, label="dy")
        b.set_title("Độ lệch vị trí theo thời gian")
        b.set_xlabel("t (s)")
        b.set_ylabel("mm")
        b.legend()
        b.grid(alpha=0.3)
    else:
        ax[0][0].text(0.5, 0.5, "không có vị trí\n(cần ≥ 3 anchor)", ha="center", va="center")

    # 3. range deviation per anchor over time
    c = ax[1][0]
    d = ax[1][1]
    for aid in sorted(ranges):
        t = [x[0] for x in ranges[aid]]
        mm = [x[1] for x in ranges[aid]]
        m = statistics.mean(mm)
        dev = [v - m for v in mm]
        sd = statistics.pstdev(mm)
        col = ANCHOR_COLORS.get(aid)
        c.plot(t, dev, lw=0.7, color=col, label=f"A{aid}  {m / 1000:.3f} m  σ {sd:.0f} mm")
        d.hist(dev, bins=30, range=(-200, 200), histtype="step", lw=1.5, color=col, label=f"A{aid} σ {sd:.0f} mm")
    c.set_title("Khoảng cách – độ lệch so với trung bình")
    c.set_xlabel("t (s)")
    c.set_ylabel("mm")
    c.legend(fontsize=8, loc="upper right")
    c.grid(alpha=0.3)
    d.set_title("Phân bố độ lệch khoảng cách")
    d.set_xlabel("mm")
    d.set_ylabel("số lần đo")
    d.legend(fontsize=8)
    d.grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.splitext(path)[0] + ".png"
    fig.savefig(png, dpi=110)
    print(f"Đã lưu: {png}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
