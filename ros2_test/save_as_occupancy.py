#!/usr/bin/env python3

import open3d as o3d
import numpy as np
from PIL import Image
import yaml
import sys
import os

def generate_occupancy_map(ply_file, resolution=0.05, map_name="map"):
    # Load point cloud
    print(f"[INFO] Loading point cloud: {ply_file}")
    pcd = o3d.io.read_point_cloud(ply_file)
    points = np.asarray(pcd.points)

    if points.size == 0:
        print("[ERROR] No points found in the .ply file!")
        sys.exit(1)

    # Flatten to X-Z plane (suppress Y axis)
    xz_points = points[:, [0, 2]]

    # Shift so min values are at (0,0)
    min_vals = np.min(xz_points, axis=0)
    xz_points_shifted = xz_points - min_vals

    # Convert meters to grid coordinates
    grid_coords = (xz_points_shifted / resolution).astype(int)

    # Grid size
    max_vals = np.max(grid_coords, axis=0)
    grid_width, grid_height = max_vals + 1

    print(f"[INFO] Grid size: {grid_width} x {grid_height} cells")

    # Initialize grid (0 = free, 1 = obstacle)
    grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Mark obstacles
    for x, z in grid_coords:
        if 0 <= z < grid_height and 0 <= x < grid_width:
            grid[z, x] = 1

    # Convert to ROS occupancy format (-1=unknown, 0=free, 100=occupied)
    ros_grid = np.full((grid_height, grid_width), 0, dtype=np.uint8)
    ros_grid[grid == 1] = 100

    # Save as PGM (ROS map format expects top-left origin)
    pgm_file = f"{map_name}.pgm"
    Image.fromarray(255 - (ros_grid * 255 // 100)).save(pgm_file)
    print(f"[INFO] Saved PGM map: {pgm_file}")

    # Save YAML metadata
    yaml_data = {
        "image": os.path.basename(pgm_file),
        "resolution": resolution,
        "origin": [float(min_vals[0]), float(min_vals[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196
    }
    yaml_file = f"{map_name}.yaml"
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f)
    print(f"[INFO] Saved YAML metadata: {yaml_file}")

    # Save PNG preview (white=free, black=obstacle)
    png_file = f"{map_name}_preview.png"
    Image.fromarray((1 - grid) * 255).convert("L").save(png_file)
    print(f"[INFO] Saved PNG preview: {png_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <path_to_ply_file> [resolution_meters_per_cell] [map_name]")
        sys.exit(1)

    ply_file = sys.argv[1]
    resolution = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    map_name = sys.argv[3] if len(sys.argv) > 3 else "map"

    generate_occupancy_map(ply_file, resolution, map_name)

