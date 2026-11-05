#!/usr/bin/env python3
"""Anchor self-positioning from the anchor-to-anchor distances measured by the survey.

    host/survey_solver.py            # listen to the tag, print the layout when the table is complete
    host/survey_solver.py --write    # ... and store the coordinates in anchors.toml

Coordinates follow the map in PLAN.md section 4: A1 = (0, 0), A2 on the +x axis, the other
anchors on the +y side (numbering counter-clockwise seen from above). All anchors hang at the
same height, so the problem is 2D. Also usable as a module (solve_layout / format_report).
"""

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


@dataclass
class Layout:
    anchors: dict                      # id -> (x, y) in metres
    rms_mm: float = 0.0
    residuals: dict = field(default_factory=dict)  # (a, b) -> measured - solved, mm
    warnings: list = field(default_factory=list)
    n_pairs: int = 0
    redundancy: int = 0                # pairs minus unknowns; 0 means no consistency check possible


def _solve_linear(a, b):
    """Gaussian elimination with partial pivoting; a is n x n, b has n entries."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def solve_layout(pairs):
    """pairs: {(a, b): distance_m} with a < b. Returns Layout, or None if it cannot be solved."""
    ids = sorted({i for p in pairs for i in p})
    if len(ids) < 3:
        return None
    a1, a2 = ids[0], ids[1]
    d = {}
    for (a, b), v in pairs.items():
        d[(a, b)] = d[(b, a)] = v
    if (a1, a2) not in d or d[(a1, a2)] <= 0:
        return None

    # Initial guess: A1 at the origin, A2 on +x, then each further anchor trilaterated from any
    # two already placed anchors it has distances to, taking the solution on the +y side. This
    # still works when one pair is missing (5 of 6 pairs for 4 anchors).
    pos = {a1: (0.0, 0.0), a2: (d[(a1, a2)], 0.0)}
    pending = ids[2:]
    while pending:
        progressed = False
        for k in list(pending):
            refs = [p for p in pos if (p, k) in d]
            if len(refs) < 2:
                continue
            (p, q) = refs[:2]
            (xp, yp), (xq, yq) = pos[p], pos[q]
            base = math.hypot(xq - xp, yq - yp)
            if base < 1e-6:
                continue
            ex, ey = (xq - xp) / base, (yq - yp) / base
            along = (d[(p, k)] ** 2 - d[(q, k)] ** 2 + base ** 2) / (2 * base)
            off = math.sqrt(max(d[(p, k)] ** 2 - along ** 2, 0.0)) or 1e-3
            cands = [(xp + along * ex - s * off * ey, yp + along * ey + s * off * ex) for s in (1, -1)]
            pos[k] = max(cands, key=lambda c: c[1])
            pending.remove(k)
            progressed = True
        if not progressed:
            return None

    # Gauss-Newton over every measured pair. Unknowns: x of A2, then (x, y) of the others.
    params = [(a2, 0)] + [(k, c) for k in ids[2:] for c in (0, 1)]
    index = {p: i for i, p in enumerate(params)}
    edges = [(a, b) for (a, b) in pairs]
    for _ in range(30):
        jtj = [[0.0] * len(params) for _ in params]
        jtr = [0.0] * len(params)
        for a, b in edges:
            (xa, ya), (xb, yb) = pos[a], pos[b]
            r = math.hypot(xa - xb, ya - yb) or 1e-9
            res = r - pairs[(a, b)]
            grad = {}
            for node, sign in ((a, 1.0), (b, -1.0)):
                ux, uy = sign * (pos[a][0] - pos[b][0]) / r, sign * (pos[a][1] - pos[b][1]) / r
                if (node, 0) in index:
                    grad[index[(node, 0)]] = ux
                if (node, 1) in index:
                    grad[index[(node, 1)]] = uy
            for i, gi in grad.items():
                jtr[i] -= gi * res
                for j, gj in grad.items():
                    jtj[i][j] += gi * gj
        step = _solve_linear(jtj, jtr)
        if step is None:
            break
        for (node, c), s in zip(params, step):
            x, y = pos[node]
            pos[node] = (x + s, y) if c == 0 else (x, y + s)
        if max(abs(s) for s in step) < 1e-6:
            break

    lay = Layout(anchors=pos, n_pairs=len(edges), redundancy=len(edges) - len(params))
    for a, b in edges:
        solved = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
        lay.residuals[(a, b)] = (pairs[(a, b)] - solved) * 1000
    lay.rms_mm = math.sqrt(sum(v * v for v in lay.residuals.values()) / len(edges))
    _check(lay, pairs, ids)
    return lay


def _check(lay, pairs, ids):
    w = lay.warnings
    n_all = len(ids) * (len(ids) - 1) // 2
    if len(pairs) < n_all:
        have = set(pairs)
        missing = [f"A{a}–A{b}" for a in ids for b in ids if a < b and (a, b) not in have]
        w.append(f"Thiếu cặp {', '.join(missing)}: tọa độ giải từ {len(pairs)}/{n_all} cặp.")
    if lay.redundancy <= 0:
        w.append("Không có phép đo dư nên không kiểm tra được sai số.")
    elif lay.rms_mm > 15:  # with one redundant pair, a 25 cm error in one pair shows as ~25 mm RMS
        worst = max(lay.residuals, key=lambda k: abs(lay.residuals[k]))
        w.append(f"Sai số dư lớn ({lay.rms_mm:.0f} mm): các khoảng cách không khớp một hình phẳng. "
                 f"Nghi bị che hoặc đa đường, lớn nhất ở A{worst[0]}–A{worst[1]} "
                 f"({lay.residuals[worst]:+.0f} mm).")

    if len(ids) == 4 and len(pairs) == n_all:  # the diagonal test needs both diagonals measured
        a1, a2, a3, a4 = ids
        diag = {(a1, a3), (a2, a4)}
        longest = set(sorted(pairs, key=pairs.get)[-2:])
        if longest != diag:
            w.append(f"Hai khoảng cách dài nhất không phải hai đường chéo A{a1}–A{a3}, A{a2}–A{a4}: "
                     f"có thể treo sai thứ tự (phải đi {a1}→{a2}→{a3}→{a4} ngược chiều kim đồng hồ).")
        ring = [a1, a2, a3, a4]
        for i, k in enumerate(ring):
            p, prev, nxt = lay.anchors[k], lay.anchors[ring[i - 1]], lay.anchors[ring[(i + 1) % 4]]
            v1 = (prev[0] - p[0], prev[1] - p[1])
            v2 = (nxt[0] - p[0], nxt[1] - p[1])
            n1, n2 = math.hypot(*v1), math.hypot(*v2)
            if n1 and n2:
                ang = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))))
                if abs(ang - 90) > 10:
                    w.append(f"Góc tại A{k} là {ang:.0f}° (lệch {ang - 90:+.0f}° so với hình chữ nhật).")


def format_report(lay, pairs_info=None):
    lines = ["Tọa độ anchor (m):"]
    for k, (x, y) in sorted(lay.anchors.items()):
        lines.append(f"  A{k}: x = {x:7.3f}   y = {y:7.3f}")
    lines.append(f"Sai số dư RMS: {lay.rms_mm:.1f} mm ({lay.n_pairs} cặp, dư {lay.redundancy} phép đo)")
    lines.append("Từng cặp (đo − giải, mm):")
    for (a, b), r in sorted(lay.residuals.items()):
        extra = ""
        if pairs_info and (a, b) in pairs_info:
            pi = pairs_info[(a, b)]
            extra = f"   đo {pi.dist_mm / 1000:.3f} m ± {pi.std_mm} mm, {pi.n_samples} mẫu"
        lines.append(f"  A{a}–A{b}: {r:+6.1f}{extra}")
    for w in lay.warnings:
        lines.append("⚠ " + w)
    return "\n".join(lines)


def write_anchors_toml(path, anchors, anchor_height, tag_height):
    with open(path) as f:
        text = f.read()
    head = text.split("[anchors]")[0]
    body = "".join(f"{k} = [{x:.3f}, {y:.3f}]\n" for k, (x, y) in sorted(anchors.items()))
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(path, "w") as f:
        f.write(head + f"[anchors]\n# written by survey_solver.py {stamp}\n" + body)


def main():
    import serial
    import uwbproto as up
    from uwb_viewer import AnchorMap

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--timeout", type=float, default=20.0, help="giây chờ đủ bảng")
    ap.add_argument("--write", action="store_true", help="ghi tọa độ vào anchors.toml")
    ap.add_argument("--anchors", default=os.path.join(HERE, "anchors.toml"))
    args = ap.parse_args()

    port, baud = args.port, args.baud
    if not port:
        tags = [b for b in up.detect() if b.kind == "uwb" and b.status.role == up.ROLE_TAG]
        if not tags:
            sys.exit("Không thấy tag (tắt uwb_viewer.py nếu đang chạy), hoặc dùng --port.")
        port, baud = tags[0].port, tags[0].baud

    parser, pairs, status, version = up.Parser(), {}, None, None
    t0 = time.time()
    print("Chờ bảng khoảng cách anchor từ tag...")
    with serial.Serial(port, baud, timeout=0.05) as s:
        s.reset_input_buffer()
        while time.time() - t0 < args.timeout:
            for pkt in parser.feed(s.read(4096)):
                if isinstance(pkt, up.Status):
                    status = pkt
                elif isinstance(pkt, up.AnchorPair) and pkt.n_samples > 0:
                    if pkt.table_version != version:
                        version = pkt.table_version
                    pairs[(pkt.id_a, pkt.id_b)] = pkt
            if status and status.n_anchors:
                n = status.n_anchors
                done = {up.SURVEY_IDLE: n * (n - 1) // 2, up.SURVEY_PARTIAL: max(2 * n - 3, 3)}
                need = done.get(status.survey_state)
                current = [p for p in pairs.values() if p.table_version == version]
                if need and len(current) >= need:
                    break
    if not pairs:
        state = up.SURVEY_NAMES.get(status.survey_state, "?") if status and status.ant_src is not None else "?"
        sys.exit(f"Không nhận được cặp nào (trạng thái khảo sát của tag: {state}).")

    lay = solve_layout({k: v.dist_mm / 1000.0 for k, v in pairs.items()})
    if not lay:
        sys.exit(f"Chưa đủ cặp để giải: {sorted(pairs)}")
    print(format_report(lay, pairs))
    if args.write:
        amap = AnchorMap(args.anchors)
        write_anchors_toml(args.anchors, lay.anchors, amap.anchor_height, amap.tag_height)
        print(f"Đã ghi vào {args.anchors}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
