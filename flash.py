#!/usr/bin/env python3
"""Detect DWM1001-DEV boards on USB and flash the UWB firmware as tag or anchor.

    ./flash.py                     # interactive
    ./flash.py --list              # only show the boards that are plugged in
    ./flash.py --sn 000760163359 --role tag --id 1 --anchors 4 --yes

Role and number are stored in UICR, so the same firmware image serves every board.
Numbering follows the anchor map in PLAN.md (anchors 1-4, counter-clockwise from the origin).
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "host"))
import uwbproto as up  # noqa: E402

FW_DIR = os.path.join(ROOT, "firmware")
FW_HEX = os.path.join(FW_DIR, "build", "uwb.hex")
BACKUP_DIR = os.path.join(ROOT, "backup")

UICR_CUSTOMER = 0x10001080
CFG_MAGIC = 0x31425755
MAX_ANCHORS = 8
MAX_TAGS = 16

ANCHOR_MAP = r"""
        A4 ●─────────────────────● A3
           │                     │
           │      khu vực bay    │
           │                     │
        A1 ●─────────────────────● A2
        (0,0)      trục +x →
   (đánh số ngược chiều kim đồng hồ khi nhìn từ trên xuống)
"""


def find_tool(name, extra):
    path = shutil.which(name)
    if path:
        return path
    for p in extra:
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    sys.exit(f"Không tìm thấy {name}. Xem PLAN.md mục 2 (công cụ).")


NRFUTIL = None
JLINK = None


def run(cmd, **kw):
    print("  $ " + " ".join(cmd))
    return subprocess.run(cmd, **kw)


def ask(prompt, choices=None, default=None, cast=str):
    while True:
        suffix = f" [{default}]" if default is not None else ""
        raw = input(f"{prompt}{suffix}: ").strip()
        if not raw and default is not None:
            raw = str(default)
        try:
            val = cast(raw)
        except ValueError:
            print("  Giá trị không hợp lệ.")
            continue
        if choices is None or val in choices:
            return val
        print(f"  Chọn một trong: {', '.join(map(str, choices))}")


def show_boards(boards):
    if not boards:
        print("Không thấy board UWB nào (J-Link SEGGER) trên USB.")
        return
    print(f"Tìm thấy {len(boards)} board:")
    for i, b in enumerate(boards, 1):
        print(f"  [{i}] S/N {b.sn}  {up.resolve(b.port)}  →  {b.describe()}")


# ---------------------------------------------------------------- hex / UICR

def ihex_record(addr16, rtype, data):
    rec = bytes([len(data), (addr16 >> 8) & 0xFF, addr16 & 0xFF, rtype]) + bytes(data)
    csum = (-sum(rec)) & 0xFF
    return ":" + rec.hex().upper() + f"{csum:02X}"


def uicr_words(role, node_id, n_anchors, baud):
    return [
        CFG_MAGIC,
        role | (node_id << 8) | (n_anchors << 16),
        0xFFFFFFFF,  # antenna delays: firmware default until calibrated (PLAN.md phase 3)
        baud,
    ]


def build_combined_hex(words):
    with open(FW_HEX) as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    if lines and lines[-1] == ":00000001FF":
        lines.pop()
    data = b"".join(w.to_bytes(4, "little") for w in words)
    lines.append(ihex_record(0, 0x04, (UICR_CUSTOMER >> 16).to_bytes(2, "big")))
    lines.append(ihex_record(UICR_CUSTOMER & 0xFFFF, 0x00, data))
    lines.append(":00000001FF")
    fd, path = tempfile.mkstemp(prefix="uwb_", suffix=".hex")
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------- steps

def backup_pans(board):
    label = board.pans_label or board.sn
    dest = os.path.join(BACKUP_DIR, f"pans_flash_{label}.bin")
    if os.path.exists(dest):
        print(f"  Đã có bản sao lưu PANS: {os.path.relpath(dest, ROOT)}")
        return True
    print(f"  Board đang chạy PANS ({label}) và chưa có bản sao lưu. Đang sao lưu...")
    os.makedirs(BACKUP_DIR, exist_ok=True)
    uicr = os.path.join(BACKUP_DIR, f"pans_uicr_{label}.bin")
    script = f"connect\nsavebin {dest} 0x0 0x80000\nsavebin {uicr} 0x10001000 0x400\nexit\n"
    with tempfile.NamedTemporaryFile("w", suffix=".jlink", delete=False) as f:
        f.write(script)
        path = f.name
    r = run([JLINK, "-NoGui", "1", "-USB", board.sn, "-device", "nRF52832_xxAA", "-if", "SWD",
             "-speed", "4000", "-autoconnect", "1", "-CommandFile", path],
            capture_output=True, text=True)
    os.unlink(path)
    ok = r.returncode == 0 and os.path.exists(dest) and os.path.getsize(dest) == 0x80000
    print("  Sao lưu xong." if ok else "  Sao lưu THẤT BẠI:\n" + r.stdout[-800:])
    return ok


def build_firmware():
    print("Build firmware...")
    r = run(["make", "-C", FW_DIR, "-j8"], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:] + r.stderr[-2000:])
        sys.exit("Build lỗi.")
    print("  " + r.stdout.strip().splitlines()[-1])


def flash(board, hex_path):
    print(f"Nạp lên S/N {board.sn} (xóa toàn bộ flash + UICR)...")
    r = run([NRFUTIL, "device", "program", "--firmware", hex_path, "--serial-number", board.sn,
             "--options", "chip_erase_mode=ERASE_ALL,verify=VERIFY_READ"])
    if r.returncode != 0:
        sys.exit("Nạp lỗi.")
    r = run([NRFUTIL, "device", "reset", "--serial-number", board.sn])
    if r.returncode != 0:
        sys.exit("Reset lỗi.")


def verify(board, baud, role, node_id):
    print("Kiểm tra board sau khi nạp...")
    time.sleep(0.5)
    try:
        st = up._listen_status(board.port, baud, 4.0)
    except (up.serial.SerialException, OSError):
        print("  ? Cổng serial đang bị chương trình khác mở (uwb_viewer?), không kiểm tra được.")
        print("    Đã nạp xong; xem phiên bản firmware trên viewer.")
        return True
    if not st:
        print("  ✗ Không nhận được gói STATUS từ board.")
        return False
    ok = st.role == role and st.node_id == node_id and st.radio_ok
    mark = "✓" if ok else "✗"
    print(f"  {mark} {st.role_name} {st.node_id}, firmware v{st.fw}, "
          f"DW1000 {'OK' if st.radio_ok else 'LỖI'} (0x{st.dw_dev_id:08X})")
    print(f"    {st.calib_text}")
    return ok


def main():
    global NRFUTIL, JLINK
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="chỉ liệt kê board đang cắm")
    ap.add_argument("--sn", help="số serial J-Link của board cần nạp")
    ap.add_argument("--role", choices=["tag", "anchor"])
    ap.add_argument("--id", type=int, help="số thứ tự (anchor 1-4 theo sơ đồ, tag 1..)")
    ap.add_argument("--anchors", type=int, help="tag: số anchor trong hệ (mặc định 4)")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--yes", action="store_true", help="không hỏi xác nhận")
    args = ap.parse_args()

    print("Đang dò board UWB...")
    boards = up.detect()
    show_boards(boards)
    if args.list or not boards:
        return

    NRFUTIL = find_tool("nrfutil", [os.path.expanduser("~/tools/nrfutil")])
    JLINK = find_tool("JLinkExe", [os.path.expanduser("~/tools/jlink/JLinkExe"),
                                   "/opt/SEGGER/JLink/JLinkExe"])

    if args.sn:
        board = next((b for b in boards if b.sn.lstrip("0") == args.sn.lstrip("0")), None)
        if not board:
            sys.exit(f"Không thấy board S/N {args.sn}.")
    elif len(boards) == 1:
        board = boards[0]
    else:
        board = boards[ask("Chọn board", range(1, len(boards) + 1), cast=int) - 1]

    if args.role:
        role = up.ROLE_TAG if args.role == "tag" else up.ROLE_ANCHOR
    else:
        print("\nVai trò:  [1] Tag   [2] Anchor")
        role = up.ROLE_TAG if ask("Chọn vai trò", [1, 2], cast=int) == 1 else up.ROLE_ANCHOR

    n_anchors = 0
    if role == up.ROLE_ANCHOR:
        print(ANCHOR_MAP)
        node_id = args.id or ask("Số thứ tự anchor", range(1, MAX_ANCHORS + 1), cast=int)
    else:
        node_id = args.id or ask("Số thứ tự tag", range(1, MAX_TAGS + 1), default=1, cast=int)
        n_anchors = args.anchors or ask("Số anchor trong hệ (tag đo lần lượt anchor 1..N)",
                                        range(1, MAX_ANCHORS + 1), default=4, cast=int)

    name = f"{up.ROLE_NAMES[role]} {node_id}"
    print(f"\nSẽ nạp: S/N {board.sn} → {name}"
          + (f", đo {n_anchors} anchor" if role == up.ROLE_TAG else "")
          + f", UART {args.baud} baud")
    print(f"Hiện tại board: {board.describe()}")
    if board.kind == "pans":
        print("CẢNH BÁO: firmware PANS trên board sẽ bị xóa.")
    if not args.yes and ask("Tiếp tục? (y/n)", ["y", "n", "Y", "N"]).lower() != "y":
        return

    if board.kind == "pans" and not backup_pans(board):
        sys.exit("Dừng lại vì chưa sao lưu được PANS.")

    build_firmware()
    hex_path = build_combined_hex(uicr_words(role, node_id, n_anchors, args.baud))
    try:
        flash(board, hex_path)
    finally:
        os.unlink(hex_path)
    ok = verify(board, args.baud, role, node_id)
    print(f"\n{'Xong' if ok else 'Đã nạp nhưng kiểm tra không đạt'}: S/N {board.sn} là {name}.")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nĐã hủy.")
