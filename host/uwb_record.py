#!/usr/bin/env python3
"""Record the tag for a few seconds, save an Excel workbook and print the noise analysis.

    host/uwb_record.py                    # 10 s, auto-detect the tag, data/uwb_<time>.xlsx
    host/uwb_record.py --duration 30 --out data/static_test.xlsx

The tag must be standing still for the noise numbers to mean anything. Positions are solved
offline exactly like uwb_viewer.py (latest range per anchor, fresh within 0.3 s of tag time),
using the coordinates in anchors.toml. Stop uwb_viewer.py first: two programs reading the same
serial port each get only part of the bytes.
"""

import argparse
import math
import os
import statistics
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import uwbproto as up  # noqa: E402
from uwb_viewer import AnchorMap, FRESH_S, solve_2d  # noqa: E402

DATA_DIR = os.path.join(os.path.dirname(HERE), "data")


def record(port, baud, duration, amap):
    """Returns the RANGE packets with host time, the CRC error count, and the anchor pairs.
    Anchor pairs (repeated by the tag every 2 s) also update amap's survey layout."""
    parser = up.Parser()
    ranges, pairs, status, version = [], {}, None, None
    with serial.Serial(port, baud, timeout=0.05) as s:
        s.reset_input_buffer()
        time.sleep(0.1)
        s.reset_input_buffer()  # drop bytes buffered before we opened the port
        t0 = time.time()
        while time.time() - t0 < duration:
            now = time.time()
            for pkt in parser.feed(s.read(4096)):
                if isinstance(pkt, up.Range):
                    ranges.append((now - t0, pkt))
                elif isinstance(pkt, up.Status):
                    status = pkt
                elif isinstance(pkt, up.AnchorPair):
                    pairs[(pkt.id_a, pkt.id_b)] = pkt
                    version = pkt.table_version
                    amap.update_survey(pairs, version, status.n_anchors if status else 0)
            left = duration - (time.time() - t0)
            print(f"\r  ghi... {len(ranges)} gói, còn {max(left, 0):4.1f} s", end="", flush=True)
    print()
    return ranges, parser.crc_errors, {k: p for k, p in pairs.items() if p.table_version == version}


def solve_positions(ranges, amap):
    """Replays the stream: one position fix per successful range once >= 3 anchors are fresh."""
    latest, fixes, guess = {}, [], None
    for _, r in ranges:
        if not r.ok or r.anchor_id not in amap.anchors:
            continue
        latest[r.anchor_id] = r
        pts, dists = [], []
        for aid, lr in latest.items():
            if ((r.t_us - lr.t_us) & 0xFFFFFFFF) <= FRESH_S * 1e6:
                pts.append(amap.anchors[aid])
                dists.append(amap.horizontal(lr.range_mm / 1000.0))
        if len(pts) < 3:
            continue
        if guess is None:
            guess = (sum(p[0] for p in pts) / len(pts) + 0.1, sum(p[1] for p in pts) / len(pts) + 0.1)
        sol = solve_2d(pts, dists, guess)
        if sol:
            guess = sol[:2]
            fixes.append({"t_us": r.t_us, "x": sol[0], "y": sol[1], "rms": sol[2], "n": len(pts)})
    return fixes


def pct(values, p):
    s = sorted(values)
    if not s:
        return float("nan")
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def analyse(ranges, fixes, duration):
    per_anchor = {}
    for aid in sorted({r.anchor_id for _, r in ranges}):
        rs = [r for _, r in ranges if r.anchor_id == aid]
        ok = [r for r in rs if r.ok]
        mm = [r.range_mm for r in ok]
        per_anchor[aid] = {
            "attempts": len(rs),
            "ok": len(ok),
            "success": len(ok) / len(rs) * 100 if rs else 0,
            "timeout": sum(1 for r in rs if r.flags & up.F_TIMEOUT),
            "rx_err": sum(1 for r in rs if r.flags & up.F_RX_ERR),
            "mean_m": statistics.mean(mm) / 1000 if mm else None,
            "std_mm": statistics.pstdev(mm) if len(mm) > 1 else None,
            "min_m": min(mm) / 1000 if mm else None,
            "max_m": max(mm) / 1000 if mm else None,
            "p2p_mm": (max(mm) - min(mm)) if mm else None,
            "nlos": sum(1 for r in ok if r.flags & up.F_NLOS),
            "rx_dbm": statistics.mean(r.rx_power_dbm for r in ok) if ok else None,
            "fp_dbm": statistics.mean(r.fp_power_dbm for r in ok) if ok else None,
        }

    pos = None
    if len(fixes) > 1:
        mx = statistics.mean(f["x"] for f in fixes)
        my = statistics.mean(f["y"] for f in fixes)
        for f in fixes:
            f["dx_mm"] = (f["x"] - mx) * 1000
            f["dy_mm"] = (f["y"] - my) * 1000
            f["dr_mm"] = math.hypot(f["dx_mm"], f["dy_mm"])
        dr = [f["dr_mm"] for f in fixes]
        pos = {
            "n": len(fixes),
            "rate": len(fixes) / duration,
            "mean_x": mx,
            "mean_y": my,
            "std_x_mm": statistics.pstdev(f["dx_mm"] for f in fixes),
            "std_y_mm": statistics.pstdev(f["dy_mm"] for f in fixes),
            "drms_mm": math.sqrt(sum(d * d for d in dr) / len(dr)),
            "cep50_mm": pct(dr, 50),
            "r95_mm": pct(dr, 95),
            "max_mm": max(dr),
            "resid_mean_cm": statistics.mean(f["rms"] for f in fixes) * 100,
        }
    return per_anchor, pos


def print_report(per_anchor, pos, n_ranges, crc, duration, amap):
    print(f"\n=== {n_ranges} gói trong {duration:.1f} s ({n_ranges / duration:.1f} /s), lỗi CRC {crc}")
    print("\nKhoảng cách theo anchor:")
    print("  anchor  thành công   trung bình    ±σ      min–max (đỉnh-đỉnh)        RX / FP dBm   nlos")
    for aid, a in per_anchor.items():
        if a["mean_m"] is None:
            print(f"  A{aid}     {a['ok']:3d}/{a['attempts']:3d}      (không đo được)")
            continue
        print(f"  A{aid}     {a['ok']:3d}/{a['attempts']:3d} {a['success']:5.1f}%  {a['mean_m']:7.3f} m  "
              f"{a['std_mm']:5.1f} mm  {a['min_m']:.3f}–{a['max_m']:.3f} m ({a['p2p_mm']:4.0f} mm)  "
              f"{a['rx_dbm']:6.1f} / {a['fp_dbm']:6.1f}   {a['nlos']}")
    if not pos:
        print("\nKhông tính được vị trí (cần ≥ 3 anchor có tọa độ trong anchors.toml).")
        return
    print(f"\nVị trí ({pos['n']} lần giải, {pos['rate']:.0f} /s), tag cao {amap.tag_height} m, anchor cao {amap.anchor_height} m:")
    print(f"  trung bình       x = {pos['mean_x']:.3f} m   y = {pos['mean_y']:.3f} m")
    print(f"  độ lệch chuẩn    σx = {pos['std_x_mm']:.1f} mm   σy = {pos['std_y_mm']:.1f} mm")
    print(f"  DRMS             {pos['drms_mm']:.1f} mm   (căn trung bình bình phương độ lệch so với tâm)")
    print(f"  CEP50            {pos['cep50_mm']:.1f} mm   (50% điểm nằm trong bán kính này)")
    print(f"  R95              {pos['r95_mm']:.1f} mm   (95% điểm nằm trong bán kính này)")
    print(f"  lệch xa nhất     {pos['max_mm']:.1f} mm")
    print(f"  sai số dư TB     {pos['resid_mean_cm']:.1f} cm   (lớn = tọa độ anchor hoặc độ cao chưa khớp thực tế)")


def write_xlsx(path, ranges, fixes, per_anchor, pos, meta, pairs=None, layout=None):
    from openpyxl import Workbook
    from openpyxl.chart import Reference, ScatterChart, Series
    from openpyxl.styles import Font

    wb = Workbook()
    bold = Font(bold=True)

    ws = wb.active
    ws.title = "Tom tat"
    ws.append(["UWB recording"]); ws["A1"].font = Font(bold=True, size=14)
    for k, v in meta.items():
        ws.append([k, v])
    ws.append([])
    ws.append(["Anchor", "Lượt đo", "Thành công", "%", "Timeout", "Lỗi frame", "TB (m)", "σ (mm)",
               "Min (m)", "Max (m)", "Đỉnh-đỉnh (mm)", "NLOS", "RX (dBm)", "FP (dBm)"])
    for c in ws[ws.max_row]:
        c.font = bold
    for aid, a in per_anchor.items():
        ws.append([f"A{aid}", a["attempts"], a["ok"], round(a["success"], 1), a["timeout"], a["rx_err"],
                   a["mean_m"], a["std_mm"], a["min_m"], a["max_m"], a["p2p_mm"], a["nlos"],
                   a["rx_dbm"], a["fp_dbm"]])
    if pos:
        ws.append([])
        ws.append(["Vị trí", "Giá trị"]); ws.cell(ws.max_row, 1).font = bold
        rows = [("Số lần giải", pos["n"]), ("Tần số (/s)", round(pos["rate"], 1)),
                ("x trung bình (m)", pos["mean_x"]), ("y trung bình (m)", pos["mean_y"]),
                ("σx (mm)", pos["std_x_mm"]), ("σy (mm)", pos["std_y_mm"]), ("DRMS (mm)", pos["drms_mm"]),
                ("CEP50 (mm)", pos["cep50_mm"]), ("R95 (mm)", pos["r95_mm"]), ("Lệch xa nhất (mm)", pos["max_mm"]),
                ("Sai số dư TB (cm)", pos["resid_mean_cm"])]
        for r in rows:
            ws.append(list(r))
    ws.column_dimensions["A"].width = 22

    wr = wb.create_sheet("Khoang cach")
    wr.append(["t_host (s)", "seq", "tag", "anchor", "t_us", "range (mm)", "RX (dBm)", "FP (dBm)", "flags", "ok"])
    for c in wr[1]:
        c.font = bold
    for t, r in ranges:
        wr.append([round(t, 4), r.seq, r.tag_id, r.anchor_id, r.t_us, r.range_mm if r.ok else None,
                   r.rx_power_dbm if r.ok else None, r.fp_power_dbm if r.ok else None,
                   up.flag_names(r.flags), int(r.ok)])
    wr.freeze_panes = "A2"

    if fixes:
        wp = wb.create_sheet("Vi tri")
        wp.append(["t (s)", "x (m)", "y (m)", "dx (mm)", "dy (mm)", "lệch (mm)", "sai số dư (cm)", "số anchor"])
        for c in wp[1]:
            c.font = bold
        t0 = fixes[0]["t_us"]
        for f in fixes:
            wp.append([((f["t_us"] - t0) & 0xFFFFFFFF) / 1e6, f["x"], f["y"], f.get("dx_mm"), f.get("dy_mm"),
                       f.get("dr_mm"), f["rms"] * 100, f["n"]])
        wp.freeze_panes = "A2"
        n = len(fixes) + 1

        ch = ScatterChart()
        ch.title = "Vị trí tag (m)"
        ch.x_axis.title, ch.y_axis.title = "x (m)", "y (m)"
        s = Series(Reference(wp, min_col=3, min_row=2, max_row=n), Reference(wp, min_col=2, min_row=2, max_row=n),
                   title="tag")
        s.marker.symbol, s.marker.size, s.graphicalProperties.line.noFill = "circle", 3, True
        ch.series.append(s)
        ch.height, ch.width = 12, 14
        wp.add_chart(ch, "J2")

        ch2 = ScatterChart()
        ch2.title = "Độ lệch so với tâm theo thời gian (mm)"
        ch2.x_axis.title, ch2.y_axis.title = "t (s)", "mm"
        for col, name in ((4, "dx"), (5, "dy")):
            s = Series(Reference(wp, min_col=col, min_row=2, max_row=n), Reference(wp, min_col=1, min_row=2, max_row=n),
                       title=name)
            s.marker.symbol = "none"
            ch2.series.append(s)
        ch2.height, ch2.width = 9, 22
        wp.add_chart(ch2, "J27")

    if pairs:
        wa = wb.create_sheet("Anchor calib")
        wa.append(["Cặp", "Đo (m)", "σ (mm)", "Số mẫu", "Đo − giải (mm)"])
        for c in wa[1]:
            c.font = bold
        for (a, b), p in sorted(pairs.items()):
            resid = layout.residuals.get((a, b)) if layout else None
            wa.append([f"A{a}–A{b}", p.dist_mm / 1000, p.std_mm, p.n_samples, resid])
        if layout:
            wa.append([])
            wa.append(["Anchor", "x (m)", "y (m)"])
            for c in wa[wa.max_row]:
                c.font = bold
            for k, (x, y) in sorted(layout.anchors.items()):
                wa.append([f"A{k}", x, y])
            wa.append([])
            wa.append(["Sai số dư RMS (mm)", layout.rms_mm])
            for w in layout.warnings:
                wa.append(["Cảnh báo", w])
        wa.column_dimensions["A"].width = 20

    os.makedirs(os.path.dirname(path), exist_ok=True)
    wb.save(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--out", help="file .xlsx (mặc định data/uwb_<thời gian>.xlsx)")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--anchors", default=os.path.join(HERE, "anchors.toml"))
    args = ap.parse_args()

    port, baud = args.port, args.baud
    if not port:
        tags = [b for b in up.detect() if b.kind == "uwb" and b.status.role == up.ROLE_TAG]
        if not tags:
            sys.exit("Không thấy tag. Tắt uwb_viewer.py nếu nó đang chạy, hoặc dùng --port.")
        port, baud = tags[0].port, tags[0].baud
        print(f"tag: S/N {tags[0].sn}, {tags[0].describe()}")

    amap = AnchorMap(args.anchors)
    ranges, crc, pairs = record(port, baud, args.duration, amap)
    src = "tự calib" if amap.using_survey else "anchors.toml"
    print(f"Tọa độ anchor: {src}")
    fixes = solve_positions(ranges, amap)
    per_anchor, pos = analyse(ranges, fixes, args.duration)
    print_report(per_anchor, pos, len(ranges), crc, args.duration, amap)

    out = args.out or os.path.join(DATA_DIR, time.strftime("uwb_%Y%m%d_%H%M%S.xlsx"))
    meta = {"Thời điểm": time.strftime("%Y-%m-%d %H:%M:%S"), "Thời lượng (s)": args.duration,
            "Cổng": up.resolve(port), "Số gói": len(ranges), "Lỗi CRC": crc,
            "Độ cao anchor (m)": amap.anchor_height, "Độ cao tag (m)": amap.tag_height,
            "Nguồn tọa độ anchor": src,
            "Tọa độ anchor": str({k: (round(x, 3), round(y, 3)) for k, (x, y) in amap.anchors.items()})}
    write_xlsx(out, ranges, fixes, per_anchor, pos, meta, pairs, amap.layout if amap.using_survey else None)
    print(f"\nĐã lưu: {out}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nĐã hủy.")
