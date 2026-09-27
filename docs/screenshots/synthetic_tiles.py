#!/usr/bin/env python3
"""Serves synthetic XYZ map tiles for testing the offline map and for the
screenshots: a park with a pond around the simulator's default origin
(Jamaica Pond, Boston), drawn in lat/lon through Web-Mercator tile math,
with a faint 0.001 degree grid. Nothing here is real map data.

    python3 docs/screenshots/synthetic_tiles.py --port 8765
    asep_base_station --map 'http://127.0.0.1:8765/{z}/{x}/{y}.png'

/{z}/{x}/{y}.png and .jpg are served. Needs Pillow (python3-pil).
"""

import argparse
import io
import math
import random
import re
import sys
import threading
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image, ImageDraw

R = 6371008.8  # same mean radius as basestation::geo
TILE = 256
SS = 2  # supersampling

LAT0, LON0 = 42.3160, -71.1205

# Pond outline, metres east/north of the origin.
POND = [(-230, 20), (-205, 105), (-125, 158), (-25, 172), (75, 152), (165, 125), (224, 52), (232, -38),
        (192, -118), (104, -158), (4, -166), (-88, -150), (-168, -112), (-218, -52)]
# Streets (metres), as polylines.
ROADS = [[(-420, -260), (420, -260)], [(330, -420), (330, 400)], [(-420, 300), (380, 300)]]

LAND = (88, 112, 72)
LAND2 = (78, 101, 64)
TREE = (50, 76, 44)
WATER = (52, 92, 118)
DEEP = (42, 78, 104)
SHORE = (168, 158, 128)
PATH = (186, 176, 146)
ROAD = (118, 118, 116)
GRID = (150, 160, 150)


def to_geo(e, n):
    return (LAT0 + math.degrees(n / R), LON0 + math.degrees(e / (R * math.cos(math.radians(LAT0)))))


def to_local(lat, lon):
    return (math.radians(lon - LON0) * R * math.cos(math.radians(LAT0)), math.radians(lat - LAT0) * R)


def tile_x(lon, z):
    return (lon + 180.0) / 360.0 * (1 << z)


def tile_y(lat, z):
    return (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * (1 << z)


def tile_lon(x, z):
    return x / (1 << z) * 360.0 - 180.0


def tile_lat(y, z):
    return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / (1 << z)))))


def inside(poly, e, n):
    c = False
    j = len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > n) != (yj > n) and e < (xj - xi) * (n - yi) / (yj - yi) + xi:
            c = not c
        j = i
    return c


def scaled(poly, k):
    return [(x * k, y * k) for x, y in poly]


@lru_cache(maxsize=4096)
def render(z, x, y, fmt):
    size = TILE * SS
    img = Image.new("RGB", (size, size), LAND)
    d = ImageDraw.Draw(img)

    def px(e, n):
        lat, lon = to_geo(e, n)
        return ((tile_x(lon, z) - x) * size, (tile_y(lat, z) - y) * size)

    # Metres per (supersampled) pixel here, for line widths.
    mpp = 2 * math.pi * 6378137 * math.cos(math.radians(LAT0)) / (size * (1 << z))

    def width(m):
        return max(1, int(round(m / mpp)))

    # Tile extent in local metres (with margin), for scattering features.
    w_lat, e_lat = tile_lat(y + 1, z), tile_lat(y, z)
    e0, n0 = to_local(w_lat, tile_lon(x, z))
    e1, n1 = to_local(e_lat, tile_lon(x + 1, z))

    if z >= 14:
        # Mown / rough grass patches and trees, placed on a world grid so
        # neighbouring tiles agree.
        cell = 14.0
        for gi in range(int(math.floor(e0 / cell)) - 1, int(math.ceil(e1 / cell)) + 1):
            for gj in range(int(math.floor(n0 / cell)) - 1, int(math.ceil(n1 / cell)) + 1):
                rnd = random.Random(gi * 73856093 ^ gj * 19349663)
                ce, cn = (gi + rnd.random()) * cell, (gj + rnd.random()) * cell
                if inside(scaled(POND, 1.16), ce, cn) or abs(cn + 260) < 12 or abs(ce - 330) < 12 or abs(cn - 300) < 12:
                    continue
                if rnd.random() < 0.28:
                    r = (4 + rnd.random() * 5) / mpp
                    cx, cy = px(ce, cn)
                    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=TREE)
                elif rnd.random() < 0.5:
                    r = (6 + rnd.random() * 6) / mpp
                    cx, cy = px(ce, cn)
                    d.ellipse([cx - r, cy - r * 0.7, cx + r, cy + r * 0.7], fill=LAND2)

    for road in ROADS:
        d.line([px(e, n) for e, n in road], fill=ROAD, width=width(9))
    d.line([px(e, n) for e, n in scaled(POND, 1.11) + [scaled(POND, 1.11)[0]]], fill=PATH, width=width(2.5))
    d.polygon([px(e, n) for e, n in scaled(POND, 1.03)], fill=SHORE)
    d.polygon([px(e, n) for e, n in POND], fill=WATER)
    d.polygon([px(e, n) for e, n in scaled(POND, 0.6)], fill=DEEP)

    # Faint lat/lon grid every 0.001 degree.
    step = 0.001 if z >= 14 else 0.01
    lat = math.floor(w_lat / step) * step
    while lat <= e_lat + step:
        yy = (tile_y(lat, z) - y) * size
        d.line([(0, yy), (size, yy)], fill=GRID, width=1)
        lat += step
    lon = math.floor(tile_lon(x, z) / step) * step
    while lon <= tile_lon(x + 1, z) + step:
        xx = (tile_x(lon, z) - x) * size
        d.line([(xx, 0), (xx, size)], fill=GRID, width=1)
        lon += step

    img = img.resize((TILE, TILE), Image.LANCZOS)
    out = io.BytesIO()
    if fmt == "jpg":
        img.save(out, "JPEG", quality=85)
    else:
        img.save(out, "PNG", optimize=True)
    return out.getvalue()


class Handler(BaseHTTPRequestHandler):
    counter = 0
    lock = threading.Lock()

    def do_GET(self):
        m = re.match(r"^/(\d+)/(\d+)/(\d+)\.(png|jpg)$", self.path)
        if not m:
            self.send_error(404)
            return
        z, x, y = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if z > 22 or x >= (1 << z) or y >= (1 << z):
            self.send_error(404)
            return
        body = render(z, x, y, m.group(4))
        with Handler.lock:
            Handler.counter += 1
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg" if m.group(4) == "jpg" else "image/png")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        if not self.server.quiet:
            sys.stderr.write("%s %s\n" % (self.headers.get("User-Agent", "-"), fmt % args))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--print-base", action="store_true",
                    help="print a base-station position on the south shore and exit")
    args = ap.parse_args()
    if args.print_base:
        lat, lon = to_geo(-20, -190)
        print("%.7f,%.7f" % (lat, lon))
        return
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    srv.quiet = args.quiet
    print("serving synthetic tiles on http://127.0.0.1:%d/{z}/{x}/{y}.png" % args.port, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
