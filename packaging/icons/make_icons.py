#!/usr/bin/env python3
"""Renders the WPI Emulator application icon (original artwork) to PNG / ICO / ICNS.

Pure standard library, no dependencies. The vector source of truth is wpiemu.svg; the shapes below
are the same primitives rasterised with signed-distance anti-aliasing, so the outputs match it.

    python packaging/icons/make_icons.py          # writes next to this script
"""
import os
import struct
import sys
import zlib

DESIGN = 1024.0  # design grid; wpiemu.svg uses the same 1024 viewBox

BACKGROUND = (0x16, 0x21, 0x3E)
TEAL = (0x2E, 0xC4, 0xB6)
AMBER = (0xF4, 0xA2, 0x61)
INK = (0x0B, 0x13, 0x2B)
SILVER = (0xE8, 0xEC, 0xF1)
# (kind, params, colour) in painter's order.
SHAPES = [
    ("ring", (512, 512, 372, 34), TEAL),
    ("rrect", (512, 512, 190, 232, 58), AMBER),
    ("rrect", (280, 362, 34, 84, 20), SILVER),
    ("rrect", (744, 362, 34, 84, 20), SILVER),
    ("rrect", (280, 662, 34, 84, 20), SILVER),
    ("rrect", (744, 662, 34, 84, 20), SILVER),
    ("circle", (512, 372, 58), INK),
    ("circle", (512, 372, 26), TEAL),
    ("rrect", (512, 610, 96, 18, 9), INK),
]


def sd_rrect(x, y, cx, cy, hx, hy, r):
    qx = abs(x - cx) - (hx - r)
    qy = abs(y - cy) - (hy - r)
    ox, oy = max(qx, 0.0), max(qy, 0.0)
    return (ox * ox + oy * oy) ** 0.5 + min(max(qx, qy), 0.0) - r


def sd_circle(x, y, cx, cy, r):
    return ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5 - r


def sd_ring(x, y, cx, cy, r, w):
    return abs(sd_circle(x, y, cx, cy, r)) - w / 2.0


def distance(shape, x, y):
    kind, p = shape[0], shape[1]
    if kind == "rrect":
        return sd_rrect(x, y, *p)
    if kind == "circle":
        return sd_circle(x, y, *p)
    return sd_ring(x, y, *p)


def render(size):
    scale = size / DESIGN
    tile_r = 224.0
    rows = []
    for py in range(size):
        y = (py + 0.5) / scale
        row = bytearray()
        for px in range(size):
            x = (px + 0.5) / scale
            cov = min(max(0.5 - sd_rrect(x, y, 512, 512, 512, 512, tile_r) * scale, 0.0), 1.0)
            if cov <= 0.0:
                row += b"\x00\x00\x00\x00"
                continue
            r, g, b = BACKGROUND
            for shape in SHAPES:
                a = min(max(0.5 - distance(shape, x, y) * scale, 0.0), 1.0)
                if a > 0.0:
                    c = shape[2]
                    r, g, b = r + (c[0] - r) * a, g + (c[1] - g) * a, b + (c[2] - b) * a
            row += bytes((int(r + 0.5), int(g + 0.5), int(b + 0.5), int(cov * 255 + 0.5)))
        rows.append(bytes(row))
    return rows


def png_bytes(size, rows):
    def chunk(tag, data):
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + r for r in rows)
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9))
            + chunk(b"IEND", b""))


def write_ico(path, images):
    # PNG-compressed entries are valid for every size since Windows Vista.
    sizes = sorted(images)
    header = struct.pack("<HHH", 0, 1, len(sizes))
    offset = 6 + 16 * len(sizes)
    entries, blobs = b"", b""
    for s in sizes:
        data = images[s]
        dim = 0 if s >= 256 else s
        entries += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(data), offset)
        blobs += data
        offset += len(data)
    with open(path, "wb") as f:
        f.write(header + entries + blobs)


def write_icns(path, images):
    # OSType -> pixel size. The @2x entries reuse the matching larger bitmap.
    table = [(b"icp4", 16), (b"icp5", 32), (b"icp6", 64), (b"ic07", 128), (b"ic08", 256), (b"ic09", 512),
             (b"ic10", 1024), (b"ic11", 32), (b"ic12", 64), (b"ic13", 256), (b"ic14", 512)]
    body = b""
    for tag, s in table:
        data = images[s]
        body += tag + struct.pack(">I", len(data) + 8) + data
    with open(path, "wb") as f:
        f.write(b"icns" + struct.pack(">I", len(body) + 8) + body)


def main():
    out = os.path.dirname(os.path.abspath(__file__))
    sizes = [16, 32, 48, 64, 128, 256, 512, 1024]
    images = {}
    for s in sizes:
        images[s] = png_bytes(s, render(s))
        print(f"rendered {s}x{s}")
    for s in sizes:
        with open(os.path.join(out, f"wpiemu-{s}.png"), "wb") as f:
            f.write(images[s])
    write_ico(os.path.join(out, "wpiemu.ico"), {s: images[s] for s in (16, 32, 48, 64, 128, 256)})
    write_icns(os.path.join(out, "wpiemu.icns"), images)
    return 0


if __name__ == "__main__":
    sys.exit(main())
