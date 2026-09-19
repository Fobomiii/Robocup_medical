#!/usr/bin/env python3
"""Bake field_map.yaml into a Nav2 static map (PGM + YAML).

The field survey lives in millimetres with +X right / +Y forward; Nav2 wants
ROS metres with +X forward / +Y left, so each point is rotated 90 degrees
clockwise and scaled. Run this whenever the fixture survey changes:

    python3 scripts/make_static_map.py
"""

import argparse
import os
import sys

import yaml


def to_grid(field_x_mm: float, field_y_mm: float, origin_x: float, origin_y: float,
            resolution: float, height: int) -> tuple:
    """Field millimetres -> image column/row (row 0 is the top, i.e. max y)."""
    x_m = field_y_mm / 1000.0
    y_m = -field_x_mm / 1000.0
    col = int(round((x_m - origin_x) / resolution))
    row = height - 1 - int(round((y_m - origin_y) / resolution))
    return col, row


def fill_rect(image, col0, col1, row0, row1, value, width, height):
    for row in range(max(0, row0), min(height, row1 + 1)):
        for col in range(max(0, col0), min(width, col1 + 1)):
            image[row][col] = value


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    default_src = os.path.join(here, "..", "config", "field_map.yaml")
    default_out = os.path.join(here, "..", "config", "static_map.pgm")

    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=default_src)
    parser.add_argument("--output", default=default_out)
    parser.add_argument("--resolution", type=float, default=0.05)
    # If omitted, origin and raster size are derived from field_map.yaml.
    # This keeps the map aligned when the real venue boundary is re-surveyed.
    parser.add_argument("--origin-x", type=float, default=None)
    parser.add_argument("--origin-y", type=float, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--wall-mm", type=float, default=100.0)
    args = parser.parse_args()

    with open(args.source, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    field = config["field"]
    origin_x = (
        args.origin_x
        if args.origin_x is not None
        else float(field["y_min_mm"]) / 1000.0
    )
    origin_y = (
        args.origin_y
        if args.origin_y is not None
        else -float(field["x_max_mm"]) / 1000.0
    )
    width = (
        args.width
        if args.width is not None
        else int(round(
            (float(field["y_max_mm"]) - float(field["y_min_mm"]))
            / 1000.0 / args.resolution
        ))
    )
    height = (
        args.height
        if args.height is not None
        else int(round(
            (float(field["x_max_mm"]) - float(field["x_min_mm"]))
            / 1000.0 / args.resolution
        ))
    )
    resolution = args.resolution
    # 254 free, 0 occupied: the field boundary and every fixture become walls.
    image = [[254] * width for _ in range(height)]

    half_wall = args.wall_mm * 0.5
    walls = [
        (field["x_min_mm"], field["y_min_mm"], field["x_max_mm"], field["y_min_mm"]),
        (field["x_max_mm"], field["y_min_mm"], field["x_max_mm"], field["y_max_mm"]),
        (field["x_max_mm"], field["y_max_mm"], field["x_min_mm"], field["y_max_mm"]),
        (field["x_min_mm"], field["y_max_mm"], field["x_min_mm"], field["y_min_mm"]),
    ]
    for x0, y0, x1, y1 in walls:
        x_min, x_max = sorted((x0, x1))
        y_min, y_max = sorted((y0, y1))
        x_min -= half_wall
        x_max += half_wall
        y_min -= half_wall
        y_max += half_wall
        col_a, row_a = to_grid(
            x_min, y_min, origin_x, origin_y, resolution, height
        )
        col_b, row_b = to_grid(
            x_max, y_max, origin_x, origin_y, resolution, height
        )
        col0, col1 = sorted((col_a, col_b))
        row0, row1 = sorted((row_a, row_b))
        fill_rect(image, col0, col1, row0, row1, 0, width, height)

    for fixture in config.get("fixtures", []):
        half_x = float(fixture["width_mm"]) * 0.5
        half_y = float(fixture["height_mm"]) * 0.5
        cx, cy = float(fixture["x_mm"]), float(fixture["y_mm"])
        col_a, row_a = to_grid(
            cx - half_x, cy - half_y,
            origin_x, origin_y, resolution, height,
        )
        col_b, row_b = to_grid(
            cx + half_x, cy + half_y,
            origin_x, origin_y, resolution, height,
        )
        col0, col1 = sorted((col_a, col_b))
        row0, row1 = sorted((row_a, row_b))
        fill_rect(image, col0, col1, row0, row1, 0, width, height)
        print(f"fixture {fixture['name']}: cols {col0}..{col1} rows {row0}..{row1}")

    for name, goal in config.get("goals", {}).items():
        col, row = to_grid(
            float(goal["x_mm"]), float(goal["y_mm"]),
            origin_x, origin_y, resolution, height,
        )
        if not (0 <= col < width and 0 <= row < height):
            raise ValueError(f"goal {name} is outside the static map")
        if image[row][col] == 0:
            raise ValueError(f"goal {name} falls inside an occupied fixture")

    if all(value != 0 for row in image for value in row):
        raise ValueError("generated map has no occupied cells")

    with open(args.output, "wb") as handle:
        handle.write(b"P5\n")
        handle.write(b"# medical field static map\n")
        handle.write(f"{width} {height}\n255\n".encode())
        handle.write(bytes(value for row in image for value in row))

    yaml_path = os.path.splitext(args.output)[0] + ".yaml"
    with open(yaml_path, "w", encoding="utf-8") as handle:
        handle.write(
            f"image: {os.path.basename(args.output)}\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin_x}, {origin_y}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n"
        )
    print(f"wrote {args.output} ({width}x{height} @ {resolution} m/px)")
    print(f"wrote {yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
