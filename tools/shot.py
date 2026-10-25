#!/usr/bin/env python3
"""
shot.py — grab a screenshot from the Tab5 over its USB-C port.

The firmware answers a single 'S' byte with a 16-byte header followed by the
live 720x1280 panel framebuffer as raw RGB565 (see main/shot.h). Console logs
share the same link, so the reader:

  * searches for the magic instead of assuming the header comes first, and
  * verifies the payload checksum, retrying rather than saving a torn image
    when a log line lands inside a transfer.

Two deliberate choices about the port:

  * The fd is opened with os.open + termios, NOT pyserial. Opening a pyserial
    Serial asserts DTR/RTS, and on this board that resets the CPU — so the
    "screenshot" would be of a freshly booted device, or of the boot log.
  * Software flow control is disabled. The payload is binary and any 0x13
    (XOFF) inside it would stop the link mid-frame.

Output is a PNG written with zlib directly, so this needs no imaging library.

Usage:
    tools/shot.py [-p PORT] [-o OUT.png] [--raw]
"""

import argparse
import glob
import os
import struct
import sys
import termios
import time
import zlib

MAGIC = 0x35415354  # "TSA5"
HDR_LEN = 16
REQ = b"S"


def find_port():
    for pat in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def open_raw(path):
    """Open the port with no modem-line games and no flow control."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    # Raw: no echo, no CR/LF translation, no signal chars.
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ECHONL |
               termios.ISIG | termios.IEXTEN)
    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY |
               termios.INLCR | termios.IGNCR | termios.ICRNL |
               termios.ISTRIP | termios.BRKINT)
    oflag &= ~termios.OPOST
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL

    # Non-blocking reads with a poll loop in Python; VMIN/VTIME unused then but
    # set to sane values in case the fd ever goes blocking.
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
    return fd


def drain(fd):
    try:
        while os.read(fd, 65536):
            pass
    except (BlockingIOError, OSError):
        pass


def read_until(fd, n, deadline, buf=None):
    """Read exactly n bytes, or raise TimeoutError."""
    out = bytearray() if buf is None else buf
    start = len(out)
    while len(out) - start < n:
        if time.time() > deadline:
            raise TimeoutError("timed out (%d/%d bytes)" % (len(out) - start, n))
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        except OSError as e:
            raise TimeoutError("read failed: %s" % e)
        if chunk:
            out += chunk
    return bytes(out)


def write_all(fd, data):
    off = 0
    while off < len(data):
        try:
            off += os.write(fd, data[off:])
        except BlockingIOError:
            time.sleep(0.002)


def grab(fd, deadline_s=45):
    """Ask for one frame and return (w, h, payload)."""
    drain(fd)
    write_all(fd, REQ)

    deadline = time.time() + deadline_s

    # Resync on the magic: log lines may precede the header.
    magic = struct.pack("<I", MAGIC)
    window = b""
    while True:
        if time.time() > deadline:
            raise TimeoutError("no frame header (firmware flashed?)")
        try:
            b = os.read(fd, 1)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        if not b:
            continue
        window = (window + b)[-len(magic):]
        if window == magic:
            break

    rest = read_until(fd, HDR_LEN - 4, deadline)
    if len(rest) != HDR_LEN - 4:
        # Should be impossible after read_until(), but a short read here would
        # otherwise surface as a bare struct.error and abort the whole retry
        # loop instead of being treated as one bad attempt.
        raise ValueError("short header (%d/%d bytes)" % (len(rest), HDR_LEN - 4))
    w, h, nbytes, checksum = struct.unpack("<HHII", rest)

    if w * h * 2 != nbytes:
        raise ValueError("bad geometry %dx%d vs %d bytes" % (w, h, nbytes))

    payload = read_until(fd, nbytes, deadline + 120)

    words = len(payload) // 4
    got = sum(struct.unpack("<%dI" % words, payload[:words * 4]))
    got = (got + sum(payload[words * 4:])) & 0xFFFFFFFF
    if got != checksum:
        raise ValueError("checksum mismatch (log line spliced in?)")
    return w, h, payload


def write_png(path, w, h, rgb_rows):
    raw = b"".join(b"\x00" + row for row in rgb_rows)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def rgb565_to_rgb(px):
    """RGB565 -> 8-bit RGB, replicating the low bits so white stays white."""
    r = (px >> 11) & 0x1F
    g = (px >> 5) & 0x3F
    b = px & 0x1F
    return bytes(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-o", "--out", default="screenshot.png")
    ap.add_argument("--raw", action="store_true",
                    help="keep panel orientation instead of landscape")
    ap.add_argument("-t", "--tries", type=int, default=4)
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("no serial port found; is the Tab5 plugged into USB-C?")

    fd = open_raw(port)

    last = None
    for attempt in range(args.tries):
        try:
            w, h, payload = grab(fd)
            break
        except (TimeoutError, ValueError) as e:
            last = e
            print("attempt %d/%d: %s" % (attempt + 1, args.tries, e),
                  file=sys.stderr)
            time.sleep(0.5)
    else:
        os.close(fd)
        sys.exit("failed: %s" % last)

    os.close(fd)

    rows = []
    stride = w * 2
    for y in range(h):
        rows.append(b"".join(rgb565_to_rgb(px) for px in
                             struct.unpack_from("<%dH" % w, payload, y * stride)))

    if args.raw:
        write_png(args.out, w, h, rows)
        print("wrote %s (%dx%d)" % (args.out, w, h))
    else:
        # The UI draws in 640x360 landscape and the PPA rotates 90° onto the
        # portrait panel, so rotate back to get a readable image.
        #
        # Inverse of the firmware mapping px = ly*SCALE, py = PANEL_H - lx*SCALE:
        #   out row r (0..719)  = panel column r
        #   out col c (0..1279) = panel row (PANEL_H-1-c)
        # Getting the loop order backwards produces a 180°-rotated picture that
        # still looks plausible at a glance, so the mapping is spelled out here.
        out = []
        for r in range(w):                      # w = panel columns = out rows
            row = bytearray()
            for c in range(h):                  # h = panel rows = out columns
                p = rows[h - 1 - c][r * 3:r * 3 + 3]
                row += p
            out.append(bytes(row))
        write_png(args.out, h, w, out)
        print("wrote %s (%dx%d)" % (args.out, h, w))


if __name__ == "__main__":
    main()
