"""Extract XYZ map tiles from an .mbtiles chart into the layout Meshtastic UI
expects on the T-Deck's SD card.

MUI's map panel builds a path per tile and hands it straight to LVGL's image
loader (device-ui, source/graphics/map/MapTile.cpp):

    S:<prefix>/<style>/<z>/<x>/<y>.<fmt>     e.g. S:/maps/charts/14/9319/4741.png

so the card just needs a plain slippy-tile tree. Two things bite here:

  * .mbtiles stores rows in TMS order (y counts up from the south pole), while
    the tile tree MUI reads is XYZ/OSM (y counts down from the north). Copying
    rows across verbatim gives a chart that looks plausible but is mirrored
    north-for-south, so we flip: y_xyz = 2**z - 1 - y_tms.
  * LVGL decodes images, not vector tiles. A format=pbf source cannot be used;
    only raster (png/jpg) mbtiles are.

Usage:
    python tdeck_tiles.py --mbtiles CHART.mbtiles --route route.json --dry-run
    python tdeck_tiles.py --mbtiles CHART.mbtiles --route route.json \
        --out /Volumes/TDECK/maps --style charts --min-zoom 8 --max-zoom 16
"""

import argparse
import json
import math
import sqlite3
import sys
from pathlib import Path

# Web Mercator can't represent the poles; this is the usual clamp.
MAX_LAT = 85.0511
NM_PER_DEG_LAT = 60.0


def deg2num(lat, lon, zoom):
    """Slippy-map tile containing lat/lon, in XYZ (OSM) convention."""
    lat = max(-MAX_LAT, min(MAX_LAT, lat))
    n = 2**zoom
    x = int((lon + 180.0) / 360.0 * n)
    phi = math.radians(lat)
    y = int((1.0 - math.log(math.tan(phi) + 1.0 / math.cos(phi)) / math.pi) / 2.0 * n)
    # A point exactly on the eastern/southern edge lands one tile past the grid.
    return min(x, n - 1), max(0, min(y, n - 1))


def leg_bboxes(waypoints, corridor_nm):
    """One padded bbox per leg, rather than a single bbox around the whole route.

    A long diagonal passage has a route bbox many times the area actually
    sailed, and at z16 that difference is gigabytes.
    """
    boxes = []
    pairs = zip(waypoints, waypoints[1:]) if len(waypoints) > 1 else [(waypoints[0], waypoints[0])]
    for (lat1, lon1), (lat2, lon2) in pairs:
        south, north = min(lat1, lat2), max(lat1, lat2)
        west, east = min(lon1, lon2), max(lon1, lon2)
        pad_lat = corridor_nm / NM_PER_DEG_LAT
        # Longitude degrees shrink with latitude; pad using the widest end of
        # the leg so the corridor is never narrower than asked for.
        widest = max(abs(south), abs(north))
        pad_lon = corridor_nm / NM_PER_DEG_LAT / max(math.cos(math.radians(widest)), 1e-6)
        boxes.append((west - pad_lon, south - pad_lat, east + pad_lon, north + pad_lat))
    return boxes


def tiles_for_bboxes(boxes, zoom):
    """Union of XYZ tile coords covering the boxes at one zoom level."""
    wanted = set()
    for west, south, east, north in boxes:
        x_min, y_min = deg2num(north, west, zoom)  # north edge -> smaller y
        x_max, y_max = deg2num(south, east, zoom)
        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                wanted.add((x, y))
    return wanted


def read_metadata(conn):
    return {name: value for name, value in conn.execute("SELECT name, value FROM metadata")}


def extract(conn, out_root, style, zooms, boxes, tile_format, dry_run):
    written = files = missing = 0
    per_zoom = []
    for zoom in zooms:
        wanted = tiles_for_bboxes(boxes, zoom)
        n = 2**zoom
        found_here = bytes_here = 0
        for x, y in sorted(wanted):
            # mbtiles rows are TMS; the tile tree is XYZ.
            row = conn.execute(
                "SELECT tile_data FROM tiles "
                "WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                (zoom, x, n - 1 - y),
            ).fetchone()
            if row is None:
                missing += 1
                continue
            found_here += 1
            bytes_here += len(row[0])
            if not dry_run:
                path = out_root / style / str(zoom) / str(x) / f"{y}.{tile_format}"
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(row[0])
                files += 1
        written += bytes_here
        per_zoom.append((zoom, len(wanted), found_here, bytes_here))
    return per_zoom, written, files, missing


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mbtiles", required=True, type=Path, help="source raster .mbtiles chart")
    ap.add_argument("--out", type=Path, help="destination /maps directory on the SD card")
    ap.add_argument("--style", default="charts", help="style subfolder under /maps (default: charts)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--route", type=Path, help='route json: {"waypoints": [[lat, lon], ...]}')
    src.add_argument("--bbox", help="W,S,E,N in degrees")
    ap.add_argument("--corridor-nm", type=float, default=3.0, help="pad either side of each leg (default: 3)")
    ap.add_argument("--min-zoom", type=int, default=8)
    ap.add_argument("--max-zoom", type=int, default=16)
    ap.add_argument("--dry-run", action="store_true", help="report tile counts and size, write nothing")
    args = ap.parse_args(argv)

    if not args.dry_run and args.out is None:
        ap.error("--out is required unless --dry-run")

    conn = sqlite3.connect(f"file:{args.mbtiles}?mode=ro", uri=True)
    meta = read_metadata(conn)

    tile_format = meta.get("format", "png")
    if tile_format not in ("png", "jpg", "jpeg"):
        # pbf/mvt: LVGL has no vector renderer, the tiles would never draw.
        sys.exit(f"{args.mbtiles.name} is format={tile_format}; MUI can only display raster tiles (png/jpg).")

    if args.route:
        waypoints = [tuple(p) for p in json.loads(args.route.read_text())["waypoints"]]
        if not waypoints:
            sys.exit("route has no waypoints")
        boxes = leg_bboxes(waypoints, args.corridor_nm)
        where = f"{len(waypoints)} waypoints, {args.corridor_nm} nm corridor"
    else:
        west, south, east, north = (float(v) for v in args.bbox.split(","))
        boxes = [(west, south, east, north)]
        where = f"bbox {args.bbox}"

    src_min = int(meta.get("minzoom", 0))
    src_max = int(meta.get("maxzoom", 20))
    zooms = [z for z in range(args.min_zoom, args.max_zoom + 1) if src_min <= z <= src_max]
    skipped = [z for z in range(args.min_zoom, args.max_zoom + 1) if z not in zooms]
    if not zooms:
        sys.exit(f"chart only holds z{src_min}-{src_max}, nothing in the requested z{args.min_zoom}-{args.max_zoom}")
    if skipped:
        print(f"note: chart holds z{src_min}-{src_max}, skipping requested z{skipped}")

    print(f"{meta.get('name', args.mbtiles.name)}  format={tile_format}  {where}")
    per_zoom, total_bytes, files, missing = extract(
        conn, args.out, args.style, zooms, boxes, tile_format, args.dry_run
    )

    print(f"\n{'zoom':>5} {'wanted':>9} {'in chart':>9} {'coverage':>9} {'size':>10}")
    for zoom, wanted, found, size in per_zoom:
        pct = (100.0 * found / wanted) if wanted else 0.0
        print(f"{zoom:>5} {wanted:>9} {found:>9} {pct:>8.1f}% {size / 1e6:>9.1f}MB")
    total_found = sum(f for _, _, f, _ in per_zoom)
    print(f"\ntotal {total_found} tiles, {total_bytes / 1e6:.1f} MB ({missing} not in chart)")

    if args.dry_run:
        print("dry run: nothing written")
    else:
        print(f"wrote {files} files under {args.out / args.style}")
        print(f"card layout: {args.out.name}/{args.style}/<z>/<x>/<y>.{tile_format}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
