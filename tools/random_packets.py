#!/usr/bin/env python3
"""Envoie des paquets aléatoires sur le câble série du UV-K5 (Mac → radio).

Cadence XON/XOFF comme serial_bridge_pc.py. Texte imprimable pour que le
récepteur (terminal sur le Pi) affiche quelque chose de lisible.

  source .venv/bin/activate
  python tools/random_packets.py
  python tools/random_packets.py --count 20 --gap 500
"""

from __future__ import annotations

import argparse
import random
import string
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial_bridge_pc as sb  # noqa: E402

_ALPHABET = string.ascii_letters + string.digits + " "


def _packet(seq: int, lo: int, hi: int) -> bytes:
    body_max = max(1, hi - 8)
    n = random.randint(max(1, lo), max(lo, min(hi, body_max + 8)))
    prefix = f"{seq:04d} "
    room = max(1, n - len(prefix) - 1)
    body = "".join(random.choices(_ALPHABET, k=room))
    return (prefix + body + "\n").encode("ascii")


def main() -> int:
    p = argparse.ArgumentParser(description="Random Serial Bridge packets (Mac TX)")
    p.add_argument("port", nargs="?", default=None, help="ex. /dev/cu.usbserial-14310")
    p.add_argument("--count", type=int, default=0, help="0 = infini (Ctrl-C)")
    p.add_argument("--min", type=int, default=8, dest="size_min", help="taille min (o)")
    p.add_argument("--max", type=int, default=56, dest="size_max", help="taille max (o)")
    p.add_argument(
        "--gap",
        type=int,
        default=0,
        metavar="MS",
        help="espacement entre paquets, en millisecondes (défaut 0)",
    )
    p.add_argument("--pause", type=float, default=sb.DEFAULT_PAUSE)
    p.add_argument("--chunk", type=int, default=sb.DEFAULT_CHUNK)
    p.add_argument("--seed", type=int, default=None)
    args = p.parse_args()

    if args.size_min < 1 or args.size_max < args.size_min:
        print("Invalid --min/--max", file=sys.stderr)
        return 2
    if args.gap < 0:
        print("--gap must be >= 0", file=sys.stderr)
        return 2
    if args.seed is not None:
        random.seed(args.seed)

    port = sb.resolve_port(args.port)
    ser = sb.open_port(port)
    credit = sb.UartCredit()
    stop = threading.Event()

    def _reader() -> None:
        while not stop.is_set():
            try:
                data = ser.read(256)
            except (sb.serial.SerialException, OSError, TypeError):
                if not stop.is_set():
                    print(sb._uart_lost_message(port), file=sys.stderr)
                    stop.set()
                break
            if data:
                credit.feed(data)

    threading.Thread(target=_reader, daemon=True).start()
    print(
        f"Random packets on {port} ({args.size_min}–{args.size_max} B, "
        f"gap {args.gap} ms), Ctrl-C to stop.",
        file=sys.stderr,
    )
    seq = 0
    sent = 0
    t0 = time.monotonic()
    try:
        while not stop.is_set() and (args.count <= 0 or seq < args.count):
            seq += 1
            pkt = _packet(seq, args.size_min, args.size_max)
            sb.write_paced(
                ser, pkt, credit, chunk=args.chunk, pause=args.pause, progress=False
            )
            sent += len(pkt)
            print(f"TX {seq:04d}  {len(pkt):3d} B  {pkt[:-1].decode('ascii')}", flush=True)
            more = args.count <= 0 or seq < args.count
            if more and args.gap > 0 and not stop.is_set():
                time.sleep(args.gap / 1000.0)
    except (KeyboardInterrupt, sb.serial.SerialException, OSError) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print(sb._uart_lost_message(port), file=sys.stderr)
            sb._close_port(ser, stop)
            return 1
    finally:
        sb._close_port(ser, stop)
    elapsed = max(time.monotonic() - t0, 1e-6)
    print(
        f"Done: {seq} packets, {sent} B in {elapsed:.1f}s ({sent / elapsed:.0f} B/s).",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
