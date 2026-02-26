#!/usr/bin/env python3
"""
PCD (FAST-LIO2 map) -> 2D Occupancy Grid (PGM + YAML) for ROS2 Nav2

- Reads .pcd
- (Optional) voxel downsample
- Filters by z range (height slice)
- Projects to XY and rasterizes into an occupancy grid
- Marks origin (0,0) on the map with a cross marker
- Writes map.pgm + map.yaml (nav2 map_server compatible)

Install deps:
  pip install open3d numpy

Usage example:
  python pcd_to_2d_map.py \
    --pcd data/0212.pcd \
    --out_prefix data/map \
    --resolution 0.02 \
    --z_min -1.4 --z_max 1.00 \
    --voxel 0.05 \
    --min_hits 1 \
    --opening_kernel 0 \
    --mark_origin
"""
import argparse
import math
import os
from typing import Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit("open3d is required. Install with: pip install open3d") from e


# Grayscale values for trinary maps (common convention)
OCCUPIED = 0
FREE = 254
UNKNOWN = 205
ORIGIN_MARKER = 128  # gray value for origin cross marker


def write_pgm_p5(path: str, img: np.ndarray) -> None:
    """Write a uint8 grayscale image as binary PGM (P5)."""
    h, w = img.shape
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(img.tobytes(order="C"))


def compute_bounds_xy(points_xy: np.ndarray) -> Tuple[float, float, float, float]:
    min_x = float(np.min(points_xy[:, 0]))
    max_x = float(np.max(points_xy[:, 0]))
    min_y = float(np.min(points_xy[:, 1]))
    max_y = float(np.max(points_xy[:, 1]))
    return min_x, max_x, min_y, max_y


def erode_binary(mask: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return mask.copy()
    pad = k // 2
    h, w = mask.shape
    m = np.pad(mask, pad_width=pad, mode="constant", constant_values=False)
    out = np.ones((h, w), dtype=bool)
    for dy in range(k):
        ys = dy
        ye = dy + h
        for dx in range(k):
            xs = dx
            xe = dx + w
            out &= m[ys:ye, xs:xe]
    return out


def dilate_binary(mask: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return mask.copy()
    pad = k // 2
    h, w = mask.shape
    m = np.pad(mask, pad_width=pad, mode="constant", constant_values=False)
    out = np.zeros((h, w), dtype=bool)
    for dy in range(k):
        ys = dy
        ye = dy + h
        for dx in range(k):
            xs = dx
            xe = dx + w
            out |= m[ys:ye, xs:xe]
    return out


def opening_binary(mask: np.ndarray, k: int) -> np.ndarray:
    return dilate_binary(erode_binary(mask, k), k)


def draw_cross(img: np.ndarray, row: int, col: int, size: int = 5, value: int = ORIGIN_MARKER) -> None:
    """Draw a cross marker on the image at (row, col)."""
    h, w = img.shape
    for d in range(-size, size + 1):
        r1, c1 = row + d, col
        r2, c2 = row, col + d
        if 0 <= r1 < h and 0 <= c1 < w:
            img[r1, c1] = value
        if 0 <= r2 < h and 0 <= c2 < w:
            img[r2, c2] = value


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", required=True, help="Input .pcd file (FAST-LIO2 map)")
    ap.add_argument(
        "--out_prefix",
        required=True,
        help="Output prefix (e.g., /tmp/map -> /tmp/map.pgm,/tmp/map.yaml)",
    )
    ap.add_argument("--resolution", type=float, default=0.05, help="Map resolution (m/pixel) (default: 0.05)")
    ap.add_argument("--z_min", type=float, default=0.10, help="Min z to keep (m) (default: 0.10)")
    ap.add_argument("--z_max", type=float, default=2.00, help="Max z to keep (m) (default: 2.00)")
    ap.add_argument("--voxel", type=float, default=0.05, help="Voxel size (m). 0 disables (default: 0.05)")
    ap.add_argument("--padding", type=float, default=1.0, help="Extra margin around bounds (m) (default: 1.0)")
    ap.add_argument("--mode", choices=["unknown", "free"], default="unknown",
                    help="Background cells set to unknown or free (default: unknown)")
    ap.add_argument("--min_hits", type=int, default=1,
                    help="Min point hits per cell to mark as occupied (default: 1)")
    ap.add_argument("--opening_kernel", type=int, default=0,
                    help="Apply morphology opening with KxK kernel on occupied mask (0 disables) (default: 0)")
    ap.add_argument("--mark_origin", action="store_true",
                    help="Draw a cross marker at origin (0,0) on the PGM image")
    ap.add_argument("--origin_size", type=int, default=10,
                    help="Size of origin cross marker in pixels (default: 10)")

    args = ap.parse_args()

    if args.min_hits < 1:
        raise SystemExit("--min_hits must be >= 1")

    k = args.opening_kernel
    if k < 0:
        raise SystemExit("--opening_kernel must be >= 0")
    if k > 0 and k % 2 == 0:
        k += 1
        print(f"[info] opening_kernel was even; using {k} instead.")

    # 1) Load PCD
    pcd = o3d.io.read_point_cloud(args.pcd)
    if len(pcd.points) == 0:
        raise SystemExit("PCD has no points.")

    # 2) Optional voxel downsample
    if args.voxel and args.voxel > 0:
        pcd = pcd.voxel_down_sample(voxel_size=args.voxel)

    pts = np.asarray(pcd.points, dtype=np.float64)

    # 3) Height slice filter
    z = pts[:, 2]
    keep = ((z >= 0.05) & (z <= args.z_min)) | ((z >= args.z_max) & (z <= 1.50))
    pts = pts[keep]
    if pts.shape[0] == 0:
        raise SystemExit("No points left after z filtering. Adjust --z_min/--z_max.")

    # 4) Compute XY bounds (with padding)
    xy = pts[:, :2]
    min_x, max_x, min_y, max_y = compute_bounds_xy(xy)

    pad = args.padding
    min_x -= pad
    max_x += pad
    min_y -= pad
    max_y += pad

    res = args.resolution
    width = int(math.ceil((max_x - min_x) / res)) + 1
    height = int(math.ceil((max_y - min_y) / res)) + 1

    # 5) Map point -> grid indices
    xs = pts[:, 0]
    ys = pts[:, 1]

    cols = np.floor((xs - min_x) / res).astype(np.int64)
    rows = np.floor((max_y - ys) / res).astype(np.int64)

    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols = cols[valid]
    rows = rows[valid]

    # 6) Count hits per cell
    counts = np.zeros((height, width), dtype=np.uint16)
    np.add.at(counts, (rows, cols), 1)

    occupied_mask = counts >= args.min_hits

    # 7) Optional morphology opening
    if k > 0:
        occupied_before = int(occupied_mask.sum())
        occupied_mask = opening_binary(occupied_mask, k)
        occupied_after = int(occupied_mask.sum())
        print(f"[info] opening K={k}: occupied pixels {occupied_before} -> {occupied_after}")

    # 8) Build final grayscale image
    if args.mode == "free":
        img = np.full((height, width), FREE, dtype=np.uint8)
    else:
        img = np.full((height, width), UNKNOWN, dtype=np.uint8)

    img[occupied_mask] = OCCUPIED

    # 9) Mark origin (0, 0) on the image
    if args.mark_origin:
        origin_col = int(math.floor((0.0 - min_x) / res))
        origin_row = int(math.floor((max_y - 0.0) / res))
        if 0 <= origin_col < width and 0 <= origin_row < height:
            draw_cross(img, origin_row, origin_col, size=args.origin_size, value=ORIGIN_MARKER)
            print(f"[origin] Marked at pixel ({origin_col}, {origin_row}) = world (0, 0)")
            print(f"         This is where FAST-LIO started (IMU position)")
        else:
            print(f"[origin] Origin (0,0) is outside the map bounds, not drawn.")
            print(f"         Origin would be at pixel ({origin_col}, {origin_row}), map is {width}x{height}")

    # 10) Write outputs
    out_pgm = args.out_prefix + ".pgm"
    out_yaml = args.out_prefix + ".yaml"
    os.makedirs(os.path.dirname(out_pgm) or ".", exist_ok=True)

    write_pgm_p5(out_pgm, img)

    yaml_text = f"""image: {os.path.basename(out_pgm)}
resolution: {res}
origin: [{min_x:.6f}, {min_y:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
"""
    with open(out_yaml, "w", encoding="utf-8") as f:
        f.write(yaml_text)

    print("Wrote:")
    print(" ", out_pgm)
    print(" ", out_yaml)
    print(f"Grid size: {width} x {height} @ {res} m/px")
    print(f"Bounds: x[{min_x:.3f}, {max_x:.3f}], y[{min_y:.3f}, {max_y:.3f}]")
    print(f"Z slice: [{args.z_min:.3f}, {args.z_max:.3f}]")
    print(f"min_hits: {args.min_hits}, opening_kernel: {k if k>0 else 0}")


if __name__ == "__main__":
    main()
