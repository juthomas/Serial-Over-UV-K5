#!/usr/bin/env python3
"""
UV-K5 Serial Bridge PC tool

Talks to a radio running the Serial Bridge firmware over the programming
cable (UART ~38400 8N1). While the radio is in SER FSK / SER DTMF mode,
raw bytes written here are sent over RF (FSK or DTMF) and appear on the
peer radio's UART. Both radios must use the same air mode (* on the HT).

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
# FSK frame is 56 payload bytes (~1 s). DTMF frame is 8 bytes (~2–3 s);
# the radio XOFFs so these defaults still work (raise --timeout / --pause).
DEFAULT_CHUNK = 56
DEFAULT_PAUSE = 1.0
DEFAULT_TEST_TIMEOUT = 20.0
XON = 0x11
XOFF = 0x13
# FSK TX EMI on the programming cable: CH340 → 0xFF, PL2303 → 0x00.
_IDLE_NOISE = frozenset((0x00, 0xFF))
_FLOW = frozenset((XON, XOFF))


def usb_serial_ports() -> list:
    """Programming-cable ports only (skip Pi UART / Bluetooth / debug)."""
    found = []
    for p in list_ports.comports():
        dev = (p.device or "").lower()
        if "bluetooth" in dev or "debug-console" in dev or "ttyama" in dev:
            continue
        if "usb" in dev or p.vid is not None:
            found.append(p)
    return found


def resolve_port(path: str | None) -> str:
    """Use path if it exists; otherwise pick the only USB-serial adapter."""
    if path and os.path.exists(path):
        return path
    cands = usb_serial_ports()
    if path:
        print(
            f"{path} is gone — RF often unplugs the dongle; Linux comes back as ttyUSB1, ttyUSB2…",
            file=sys.stderr,
        )
    if len(cands) == 1:
        print(f"Using {cands[0].device}  ({cands[0].description})", file=sys.stderr)
        return cands[0].device
    if cands:
        print("USB serial ports:", file=sys.stderr)
        for p in cands:
            print(f"  {p.device:28s}  {p.description}", file=sys.stderr)
        raise SystemExit("Pass one of these, or unplug extras.")
    raise SystemExit(
        "No USB-serial cable found. Plug the Kenwood cable, then:\n"
        "  python serial_bridge_pc.py list"
    )


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


class UartCredit:
    """Software flow control. New firmware sends XON/XOFF; old firmware times out."""

    def __init__(self) -> None:
        self._can_send = threading.Event()
        self._can_send.set()

    def feed(self, data: bytes) -> bytes:
        out = bytearray()
        for b in data:
            if b == XOFF:
                self._can_send.clear()
            elif b == XON:
                self._can_send.set()
            else:
                out.append(b)
        return bytes(out)

    def wait_ready(self, timeout: float) -> None:
        if not self._can_send.wait(timeout):
            # No XON (stock firmware): the pause itself is the pacing.
            self._can_send.set()

    def consumed(self) -> None:
        self._can_send.clear()


def sanitize_for_terminal(data: bytes) -> bytes:
    """Strip ANSI/control bytes so UART garbage cannot wreck the terminal."""
    if data and all(b in _IDLE_NOISE or b in _FLOW for b in data):
        return b""
    out = bytearray()
    for b in data:
        if b in _IDLE_NOISE or b in _FLOW:
            continue
        if b in (9, 10, 13) or 32 <= b < 127:
            out.append(b)
        else:
            out.extend(b"\\x%02x" % b)
    return bytes(out)


def write_paced(
    ser: serial.Serial,
    data: bytes,
    credit: UartCredit,
    chunk: int = DEFAULT_CHUNK,
    pause: float = DEFAULT_PAUSE,
    progress: bool = False,
) -> None:
    """Write UART paced by XON/XOFF so the radio UART buffer cannot overflow."""
    if not data:
        return
    if chunk < 1:
        chunk = DEFAULT_CHUNK
    total = len(data)
    sent = 0
    if progress and total > chunk:
        print(
            f"Pacing {total} B ({chunk} B / {pause:.2f}s, wait XON or timeout)…",
            file=sys.stderr,
        )
    while sent < total:
        credit.wait_ready(pause)
        piece = data[sent : sent + chunk]
        ser.write(piece)
        ser.flush()
        sent += len(piece)
        if progress and total > chunk:
            print(f"  {sent}/{total} B", file=sys.stderr)
        # More to send, or a large blob: wait for the radio to finish this frame.
        if sent < total or total > chunk:
            credit.consumed()
    if progress and total > chunk:
        credit.wait_ready(pause)


def _uart_lost_message(port: str) -> str:
    return (
        f"\nUART {port} disconnected. RF on the programming cable often "
        "unplugs CH340/CP210x (Linux may come back as /dev/ttyUSB1). "
        "LOW power, ferrite, or unplug before a long TX. Replug and restart."
    )


def _close_port(ser: serial.Serial, stop: threading.Event | None = None) -> None:
    if stop is not None:
        stop.set()
    try:
        ser.cancel_read()
    except Exception:
        pass
    try:
        if ser.is_open:
            ser.close()
    except Exception:
        pass


def start_reader(
    ser: serial.Serial,
    stop: threading.Event,
    credit: UartCredit,
    port_label: str = "",
) -> threading.Thread:
    def reader() -> None:
        while not stop.is_set():
            try:
                data = ser.read(256)
            except (serial.SerialException, OSError, TypeError):
                if not stop.is_set():
                    print(_uart_lost_message(port_label or ser.port or "?"), file=sys.stderr)
                    stop.set()
                break
            if not data:
                continue
            data = credit.feed(data)
            shown = sanitize_for_terminal(data)
            if shown:
                sys.stdout.buffer.write(shown)
                sys.stdout.buffer.flush()

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    return t


def cmd_list(_: argparse.Namespace) -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    for p in ports:
        print(f"{p.device:30s}  {p.description}  [{p.hwid}]")
    return 0


def cmd_terminal(args: argparse.Namespace) -> int:
    args.port = resolve_port(args.port)
    ser = open_port(args.port, args.baud)
    credit = UartCredit()
    print(
        f"Connected to {args.port} @ {args.baud}. "
        f"Long pastes are paced ({args.chunk} B / {args.pause:.2f}s). Ctrl-C to quit.",
        file=sys.stderr,
    )
    stop = threading.Event()
    start_reader(ser, stop, credit, port_label=args.port)
    rc = 0
    try:
        stdin_fd = sys.stdin.fileno()
        while not stop.is_set():
            if sys.stdin.isatty():
                r, _, _ = select.select([stdin_fd], [], [], 0.2)
                if stop.is_set():
                    rc = 1
                    break
                if stdin_fd not in r:
                    continue
                line = sys.stdin.buffer.readline()
                if not line:
                    break
                write_paced(
                    ser, line, credit, chunk=args.chunk, pause=args.pause, progress=True
                )
            else:
                chunk = sys.stdin.buffer.read(4096)
                if not chunk:
                    break
                write_paced(
                    ser, chunk, credit, chunk=args.chunk, pause=args.pause, progress=True
                )
    except (KeyboardInterrupt, serial.SerialException, OSError) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print(_uart_lost_message(args.port), file=sys.stderr)
            rc = 1
    finally:
        _close_port(ser, stop)
    return rc


def cmd_tunnel(args: argparse.Namespace) -> int:
    """Raw byte tunnel: stdin -> UART, UART -> stdout (paced, no line editing)."""
    args.port = resolve_port(args.port)
    ser = open_port(args.port, args.baud)
    credit = UartCredit()
    print(
        f"Tunnel on {args.port} @ {args.baud} "
        f"(paced {args.chunk} B / {args.pause:.2f}s). Ctrl-C to quit.",
        file=sys.stderr,
    )
    stop = threading.Event()
    start_reader(ser, stop, credit, port_label=args.port)
    pending = bytearray()
    rc = 0
    try:
        stdin_fd = sys.stdin.fileno()
        while not stop.is_set():
            r, _, _ = select.select([stdin_fd], [], [], 0.1)
            if stdin_fd in r:
                data = os.read(stdin_fd, 4096)
                if not data:
                    if pending:
                        write_paced(
                            ser,
                            bytes(pending),
                            credit,
                            chunk=args.chunk,
                            pause=args.pause,
                        )
                    break
                pending.extend(data)
                if len(pending) >= args.chunk:
                    write_paced(
                        ser,
                        bytes(pending),
                        credit,
                        chunk=args.chunk,
                        pause=args.pause,
                    )
                    pending.clear()
    except (KeyboardInterrupt, serial.SerialException, OSError) as exc:
        if pending and isinstance(exc, KeyboardInterrupt):
            try:
                write_paced(
                    ser, bytes(pending), credit, chunk=args.chunk, pause=args.pause
                )
            except (serial.SerialException, OSError):
                pass
        if not isinstance(exc, KeyboardInterrupt):
            print(_uart_lost_message(args.port), file=sys.stderr)
            rc = 1
    finally:
        _close_port(ser, stop)
    return rc


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
                # FSK: a few hundred ms. DTMF: several seconds per 8-byte burst.
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
    except (serial.SerialException, OSError) as exc:
        print(_uart_lost_message(f"{args.port_a} / {args.port_b}"), file=sys.stderr)
        print(f"  ({exc})", file=sys.stderr)
        fail += 1
    finally:
        _close_port(a)
        _close_port(b)

    elapsed = max(time.monotonic() - t0, 1e-6)
    print(
        f"Done: {ok} ok, {fail} fail, "
        f"{total_bytes} bytes in {elapsed:.2f}s "
        f"({total_bytes / elapsed:.0f} B/s useful)"
    )
    return 0 if fail == 0 else 2


def cmd_send(args: argparse.Namespace) -> int:
    """Send a long payload, paced by the radio XON/XOFF (FSK or DTMF)."""
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

    args.port = resolve_port(args.port)
    ser = open_port(args.port, args.baud)
    credit = UartCredit()
    stop = threading.Event()

    def _credit_reader() -> None:
        while not stop.is_set():
            try:
                data_in = ser.read(256)
            except (serial.SerialException, OSError, TypeError):
                if not stop.is_set():
                    print(_uart_lost_message(args.port), file=sys.stderr)
                    stop.set()
                break
            if data_in:
                credit.feed(data_in)

    threading.Thread(target=_credit_reader, daemon=True).start()
    t0 = time.monotonic()
    lost = False
    try:
        write_paced(
            ser,
            data,
            credit,
            chunk=args.chunk,
            pause=args.pause,
            progress=True,
        )
        lost = stop.is_set()
    except (serial.SerialException, OSError):
        print(_uart_lost_message(args.port), file=sys.stderr)
        lost = True
    finally:
        _close_port(ser, stop)
    if lost:
        return 1
    elapsed = max(time.monotonic() - t0, 1e-6)
    print(
        f"Done: {len(data)} B in {elapsed:.1f}s ({len(data) / elapsed:.0f} B/s).",
        file=sys.stderr,
    )
    return 0


def _add_pace_args(sp: argparse.ArgumentParser) -> None:
    sp.add_argument(
        "--chunk",
        type=int,
        default=DEFAULT_CHUNK,
        help="Write chunk size (FSK frames are 56 B, DTMF 8 B; XON/XOFF paces)",
    )
    sp.add_argument(
        "--pause",
        type=float,
        default=DEFAULT_PAUSE,
        help="Seconds to wait for XON / between chunks (raise for DTMF, e.g. 4)",
    )


def main() -> int:
    p = argparse.ArgumentParser(description="UV-K5 Serial Bridge PC helper")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="UART baud (default 38400)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("list", help="List serial ports")
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("terminal", help="Interactive terminal on one radio")
    sp.add_argument(
        "port",
        nargs="?",
        default=None,
        help="Serial device (optional: auto-picks the USB cable)",
    )
    _add_pace_args(sp)
    sp.set_defaults(func=cmd_terminal)

    sp = sub.add_parser("tunnel", help="Raw stdin/stdout tunnel")
    sp.add_argument(
        "port",
        nargs="?",
        default=None,
        help="Serial device (optional: auto-picks the USB cable)",
    )
    _add_pace_args(sp)
    sp.set_defaults(func=cmd_tunnel)

    sp = sub.add_parser("test", help="Round-trip test between two radios")
    sp.add_argument("port_a", help="Serial device of radio A")
    sp.add_argument("port_b", help="Serial device of radio B")
    sp.add_argument("--rounds", type=int, default=5)
    sp.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TEST_TIMEOUT,
        help="Seconds to wait per direction (default 20; FSK is fine, DTMF needs this)",
    )
    sp.add_argument("--payload", default=None, help="Custom test string")
    sp.set_defaults(func=cmd_test)

    sp = sub.add_parser("send", help="Send a long text/file, paced by XON/XOFF")
    sp.add_argument(
        "port",
        nargs="?",
        default=None,
        help="Serial device (optional: auto-picks the USB cable)",
    )
    sp.add_argument("--text", default=None, help="UTF-8 string to send")
    sp.add_argument("--file", default=None, help="File to send as-is")
    sp.add_argument("--bytes", type=int, default=None, help="Send N generated bytes")
    _add_pace_args(sp)
    sp.set_defaults(func=cmd_send)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
