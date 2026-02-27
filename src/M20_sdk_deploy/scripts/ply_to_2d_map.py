#!/usr/bin/env python3
"""Convert a 3D PLY point cloud to a 2D Nav2-compatible occupancy grid (.pgm + .yaml)."""

import argparse
import numpy as np
import open3d as o3d
from pathlib import Path
from PIL import Image


def ply_to_occupancy_grid(ply_path: str, output_dir: str, resolution: float,
                          z_min: float, z_max: float, padding: int):
    # Load and height-slice
    pcd = o3d.io.read_point_cloud(ply_path)
    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points from {ply_path}")

    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    sliced = points[mask]
    print(f"After height slice [{z_min}, {z_max}]: {len(sliced)} points")

    if len(sliced) == 0:
        raise ValueError("No points in height range. Adjust z_min/z_max.")

    # Compute grid bounds
    x_min, y_min = sliced[:, 0].min(), sliced[:, 1].min()
    x_max, y_max = sliced[:, 0].max(), sliced[:, 1].max()

    width = int(np.ceil((x_max - x_min) / resolution)) + 2 * padding
    height = int(np.ceil((y_max - y_min) / resolution)) + 2 * padding
    print(f"Grid size: {width} x {height} pixels at {resolution} m/px")

    # Rasterize: free=254, occupied=0
    grid = np.full((height, width), 254, dtype=np.uint8)

    col = ((sliced[:, 0] - x_min) / resolution).astype(int) + padding
    row = ((sliced[:, 1] - y_min) / resolution).astype(int) + padding
    row = np.clip(row, 0, height - 1)
    col = np.clip(col, 0, width - 1)
    grid[row, col] = 0

    # Flip Y so image origin is bottom-left (Nav2 convention)
    grid = np.flipud(grid)

    # Save PGM
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    pgm_name = Path(ply_path).stem + "_2d.pgm"
    yaml_name = Path(ply_path).stem + "_2d.yaml"

    Image.fromarray(grid).save(out / pgm_name)

    # Save YAML
    origin_x = x_min - padding * resolution
    origin_y = y_min - padding * resolution
    with open(out / yaml_name, 'w') as f:
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print(f"Saved: {out / pgm_name}")
    print(f"Saved: {out / yaml_name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ply", help="Path to input .ply file")
    parser.add_argument("-o", "--output", default=None, help="Output directory (default: same as input)")
    parser.add_argument("-r", "--resolution", type=float, default=0.05, help="Grid resolution in m/pixel")
    parser.add_argument("--z-min", type=float, default=0.1, help="Min height for obstacle slice (m)")
    parser.add_argument("--z-max", type=float, default=1.5, help="Max height for obstacle slice (m)")
    parser.add_argument("--padding", type=int, default=10, help="Padding pixels around map border")
    args = parser.parse_args()

    output_dir = args.output or str(Path(args.ply).parent)
    ply_to_occupancy_grid(args.ply, output_dir, args.resolution, args.z_min, args.z_max, args.padding)
