#!/usr/bin/env python3
"""
Probe likely GPS baud rates and score NMEA line quality.

Usage:
  python3 scripts/debug/probe_gps_baud.py /dev/gps1
  python3 scripts/debug/probe_gps_baud.py /dev/gps2 --seconds 4
"""

import argparse
import os
import re
import select
import termios
import time


BAUD_CANDIDATES = [9600, 19200, 38400, 57600, 115200]
NMEA_RE = re.compile(r"^\$[A-Z0-9,.*-]+\*[0-9A-F]{2}$")


def checksum_ok(sentence: str) -> bool:
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, checksum = sentence[1:].split("*", 1)
    if len(checksum) < 2:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        expected = int(checksum[:2], 16)
    except ValueError:
        return False
    return calc == expected


def score_port(port: str, baud: int, seconds: float) -> tuple[int, int, int]:
    total = 0
    structured = 0
    checksum_valid = 0
    end_t = time.monotonic() + seconds
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        speed_map = {
            9600: termios.B9600,
            19200: termios.B19200,
            38400: termios.B38400,
            57600: termios.B57600,
            115200: termios.B115200,
        }
        speed = speed_map[baud]
        attrs[0] = 0  # iflag
        attrs[1] = 0  # oflag
        attrs[2] = attrs[2] | termios.CREAD | termios.CLOCAL  # cflag
        attrs[3] = 0  # lflag
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        # Portable speed assignment: some Python builds omit cfsetispeed/cfsetospeed.
        attrs[4] = speed  # ispeed
        attrs[5] = speed  # ospeed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        buf = b""
        while time.monotonic() < end_t:
            ready, _, _ = select.select([fd], [], [], 0.25)
            if not ready:
                continue
            raw = os.read(fd, 512)
            if not raw:
                continue
            buf += raw
            while b"\n" in buf:
                line_raw, buf = buf.split(b"\n", 1)
                total += 1
                line = line_raw.decode(errors="ignore").strip()
                if not line:
                    continue
                if NMEA_RE.match(line):
                    structured += 1
                    if checksum_ok(line):
                        checksum_valid += 1
    finally:
        os.close(fd)
    return total, structured, checksum_valid


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial device (e.g. /dev/gps1)")
    parser.add_argument("--seconds", type=float, default=5.0, help="Sampling seconds per baud")
    args = parser.parse_args()

    print(f"Probing {args.port} for NMEA quality ({args.seconds:.1f}s per baud)")
    print("baud,total_lines,structured_nmea,valid_checksum,quality_pct")
    best = None

    for baud in BAUD_CANDIDATES:
        try:
            total, structured, valid = score_port(args.port, baud, args.seconds)
        except Exception as exc:  # pragma: no cover (tool script)
            print(f"{baud},0,0,0,0.0  # error: {exc}")
            continue

        quality = (100.0 * valid / structured) if structured else 0.0
        print(f"{baud},{total},{structured},{valid},{quality:.1f}")

        if best is None or quality > best[1]:
            best = (baud, quality, structured)

    if best is None:
        print("No usable baud found.")
        return

    baud, quality, structured = best
    print(f"\nBest candidate: {baud} (quality={quality:.1f}%, structured_lines={structured})")
    if quality < 70.0:
        print("Warning: low checksum quality across all bauds; likely wiring/USB power/noise issue.")


if __name__ == "__main__":
    main()
