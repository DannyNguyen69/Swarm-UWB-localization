#!/usr/bin/env python3
"""Runs on the Pi: forwards the tag's raw UART stream to TCP clients, and client bytes (commands
such as the viewer's re-survey button) back to the tag.

    host/uwb_bridge.py                      # auto-detect the tag, listen on port 7777
    host/uwb_bridge.py --port /dev/serial/by-id/usb-SEGGER_J-Link_...-if00

Bytes are passed through untouched (no parsing), so the Pi does almost no work; the viewer on
the laptop checks CRCs and decodes. Connect with: host/uwb_viewer.py --tcp <pi-ip>:7777
"""

import argparse
import os
import socket
import sys
import threading
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import uwbproto as up  # noqa: E402

clients = []
clients_lock = threading.Lock()
port_ref = {"serial": None}   # the open serial port, for commands coming back from clients
port_lock = threading.Lock()


def client_to_serial(conn):
    """Commands from the viewer (e.g. the re-survey button) go back down to the tag."""
    try:
        while True:
            data = conn.recv(256)
            if not data:
                return
            with port_lock:
                if port_ref["serial"]:
                    port_ref["serial"].write(data)
    except OSError:
        return


def accept_loop(server):
    while True:
        conn, addr = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with clients_lock:
            clients.append(conn)
        threading.Thread(target=client_to_serial, args=(conn,), daemon=True).start()
        print(f"client connected: {addr[0]}:{addr[1]} ({len(clients)} total)")


def broadcast(data):
    with clients_lock:
        for c in clients[:]:
            try:
                c.sendall(data)
            except OSError:
                clients.remove(c)
                c.close()
                print(f"client disconnected ({len(clients)} left)")


def pick_port(args):
    if args.port:
        return args.port, args.baud
    for b in up.detect():
        if b.kind == "uwb" and b.status.role == up.ROLE_TAG:
            print(f"tag: S/N {b.sn} ({up.resolve(b.port)}), {b.describe()}")
            return b.port, b.baud
    sys.exit("no tag running the UWB firmware found; use --port")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the tag (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=up.DEFAULT_BAUD)
    ap.add_argument("--listen", type=int, default=7777, help="TCP port")
    args = ap.parse_args()

    port, baud = pick_port(args)
    server = socket.create_server(("0.0.0.0", args.listen), reuse_port=False)
    threading.Thread(target=accept_loop, args=(server,), daemon=True).start()
    print(f"forwarding {port} @ {baud} to tcp://0.0.0.0:{args.listen}")

    while True:
        try:
            with serial.Serial(port, baud, timeout=0.02) as s:
                with port_lock:
                    port_ref["serial"] = s
                while True:
                    data = s.read(4096)
                    if data:
                        broadcast(data)
        except serial.SerialException as e:
            with port_lock:
                port_ref["serial"] = None
            print(f"serial error: {e}; retrying in 1 s")
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
