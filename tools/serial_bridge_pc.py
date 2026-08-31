#!/usr/bin/env python3
"""
UV-K5 Serial Bridge PC tool

Talks to a radio running the Serial Bridge firmware over the programming
cable (UART ~38400 8N1). While the radio is in SER BRIDGE mode, raw bytes
written here are sent over FSK RF and appear on the peer radio's UART.

Examples:
  python serial_bridge_pc.py list
  python serial_bridge_pc.py terminal /dev/cu.usbserial-XXXX
  python serial_bridge_pc.py test /dev/cu.usbserial-A /dev/cu.usbserial-B
  python serial_bridge_pc.py tunnel /dev/cu.usbserial-XXXX
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing dependency: pyserial", file=sys.stderr)
    print("  python3 -m venv .venv && .venv/bin/pip install pyserial", file=sys.stderr)
    sys.exit(1)

DEFAULT_BAUD = 38400


def open_port(path: str, baud: int = DEFAULT_BAUD) -> serial.Serial:
    """Open the Kenwood programming cable without keying the radio PTT.

    Cheap CH340/CP210x cables often wire DTR/RTS to the 2.5 mm PTT ring.
    pyserial asserts DTR on open by default, which makes the UV-K5 transmit
    and the RF/USB noise can glitch a Mac display / Terminal.
    """
    ser = serial.Serial()
    ser.port = path
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.05
    ser.write_timeout = 1.0
    ser.dsrdtr = False
    ser.rtscts = False
    ser.xonxoff = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except (AttributeError, OSError):
        pass
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


# FSK TX EMI on the programming cable: CH340 → 0xFF, PL2303 → 0x00.
_IDLE_NOISE = frozenset((0x00, 0xFF))


def sanitize_for_terminal(data: bytes) -> bytes:
    """Strip ANSI/control bytes so UART garbage cannot wreck the terminal."""
    if data and all(b in _IDLE_NOISE for b in data):
        return b""
    out = bytearray()
    for b in data:
        if b in _IDLE_NOISE:
            continue
        if b in (9, 10, 13) or 32 <= b < 127:
            out.append(b)
        else:
            out.extend(b"\\x%02x" % b)
    return bytes(out)


def cmd_list(_: argparse.Namespace) -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    for p in ports:
        print(f"{p.device:30s}  {p.description}  [{p.hwid}]")
    return 0


def cmd_terminal(args: argparse.Namespace) -> int:
    ser = open_port(args.port, args.baud)
    print(f"Connected to {args.port} @ {args.baud}. Ctrl-C to quit.", file=sys.stderr)
    stop = threading.Event()

    def reader() -> None:
        while not stop.is_set():
            data = ser.read(256)
            if data:
                sys.stdout.buffer.write(sanitize_for_terminal(data))
                sys.stdout.buffer.flush()

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
        while True:
            if sys.stdin.isatty():
                # Line-buffered interactive: send each line with newline
                line = sys.stdin.buffer.readline()
                if not line:
                    break
                ser.write(line)
            else:
                chunk = sys.stdin.buffer.read(256)
                if not chunk:
                    break
                ser.write(chunk)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()
    return 0


def cmd_tunnel(args: argparse.Namespace) -> int:
    """Raw byte tunnel: stdin -> UART, UART -> stdout (no line editing)."""
    ser = open_port(args.port, args.baud)
    print(f"Tunnel on {args.port} @ {args.baud}. Ctrl-C to quit.", file=sys.stderr)
    stop = threading.Event()

    def reader() -> None:
        while not stop.is_set():
            data = ser.read(256)
            if data:
                sys.stdout.buffer.write(sanitize_for_terminal(data))
                sys.stdout.buffer.flush()

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
        stdin_fd = sys.stdin.fileno()
        while not stop.is_set():
            r, _, _ = select.select([stdin_fd], [], [], 0.1)
            if stdin_fd in r:
                data = os.read(stdin_fd, 256)
                if not data:
                    break
                ser.write(data)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()
    return 0


def cmd_test(args: argparse.Namespace) -> int:
    """Send a known payload from port A, expect it on port B (and reverse)."""
    a = open_port(args.port_a, args.baud)
    b = open_port(args.port_b, args.baud)
    payload = (args.payload or "UVK5-SERIAL-BRIDGE-TEST\n").encode("utf-8")
    rounds = args.rounds
    ok = 0
    fail = 0
    total_bytes = 0
    t0 = time.monotonic()

    print(f"Testing {args.port_a} <-> {args.port_b}, {rounds} round-trips…")
    try:
        for i in range(rounds):
            for src, dst, label in ((a, b, "A->B"), (b, a, "B->A")):
                dst.reset_input_buffer()
                src.write(payload)
                src.flush()
                # FSK frame + TX turnaround can take a few hundred ms
                deadline = time.monotonic() + args.timeout
                got = bytearray()
                while time.monotonic() < deadline and len(got) < len(payload):
                    chunk = dst.read(len(payload) - len(got))
                    if chunk:
                        got.extend(chunk)
                    else:
                        time.sleep(0.02)
                if bytes(got) == payload:
                    ok += 1
                    total_bytes += len(payload)
                    print(f"  [{i+1}/{rounds}] {label} OK ({len(payload)} B)")
                else:
                    fail += 1
                    print(
                        f"  [{i+1}/{rounds}] {label} FAIL "
                        f"got {len(got)} B: {got[:40]!r}"
                    )
    finally:
        a.close()
        b.close()

    elapsed = max(time.monotonic() - t0, 1e-6)
    print(
        f"Done: {ok} ok, {fail} fail, "
        f"{total_bytes} bytes in {elapsed:.2f}s "
        f"({total_bytes / elapsed:.0f} B/s useful)"
    )
    return 0 if fail == 0 else 2


def cmd_send(args: argparse.Namespace) -> int:
    """Send a long payload, paced to the FSK modem (~56 B / ~1 s)."""
    if args.file:
        data = open(args.file, "rb").read()
    elif args.bytes:
        alphabet = b"abcdefghijklmnopqrstuvwxyz 0123456789\n"
        data = bytes(alphabet[i % len(alphabet)] for i in range(args.bytes))
    else:
        data = (args.text or "").encode("utf-8")
        if not data.endswith(b"\n"):
            data += b"\n"

    if not data:
        print("Nothing to send. Use --text, --file or --bytes.", file=sys.stderr)
        return 1

    chunk_size = args.chunk
    pause = args.pause
    ser = open_port(args.port, args.baud)
    print(
        f"Sending {len(data)} B on {args.port} "
        f"({chunk_size} B / {pause:.2f}s)…",
        file=sys.stderr,
    )
    sent = 0
    t0 = time.monotonic()
    try:
        while sent < len(data):
            piece = data[sent : sent + chunk_size]
            ser.write(piece)
            ser.flush()
            sent += len(piece)
            print(f"  {sent}/{len(data)} B", file=sys.stderr)
            if sent < len(data):
                time.sleep(pause)
        # Let the last FSK frame leave the radio
        time.sleep(pause)
    finally:
        ser.close()
    elapsed = max(time.monotonic() - t0, 1e-6)
    print(
        f"Done: {sent} B in {elapsed:.1f}s ({sent / elapsed:.0f} B/s).",
        file=sys.stderr,
    )
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description="UV-K5 Serial Bridge PC helper")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="UART baud (default 38400)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list", help="List serial ports")
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("terminal", help="Interactive terminal on one radio")
    sp.add_argument("port", help="Serial device path")
    sp.set_defaults(func=cmd_terminal)

    sp = sub.add_parser("tunnel", help="Raw stdin/stdout tunnel")
    sp.add_argument("port", help="Serial device path")
    sp.set_defaults(func=cmd_tunnel)

    sp = sub.add_parser("test", help="Round-trip test between two radios")
    sp.add_argument("port_a", help="Serial device of radio A")
    sp.add_argument("port_b", help="Serial device of radio B")
    sp.add_argument("--rounds", type=int, default=5)
    sp.add_argument("--timeout", type=float, default=2.0, help="Seconds to wait per direction")
    sp.add_argument("--payload", default=None, help="Custom test string")
    sp.set_defaults(func=cmd_test)

    sp = sub.add_parser("send", help="Send a long text/file, paced for FSK")
    sp.add_argument("port", help="Serial device of the TX radio")
    sp.add_argument("--text", default=None, help="UTF-8 string to send")
    sp.add_argument("--file", default=None, help="File to send as-is")
    sp.add_argument("--bytes", type=int, default=None, help="Send N generated bytes")
    sp.add_argument("--chunk", type=int, default=56, help="Bytes per FSK frame (max 56)")
    sp.add_argument("--pause", type=float, default=1.0, help="Seconds between chunks")
    sp.set_defaults(func=cmd_send)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
