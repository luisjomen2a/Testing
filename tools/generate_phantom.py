#!/usr/bin/env python3
"""Generate a synthetic CT-like volume (legacy VTK structured points) to test the application without patient data.

The phantom is an elliptic "body" of soft tissue (~40 HU) with a "spine" and "ribs" of bone (~700 HU), two "kidneys"
(~150 HU) surrounded by air (-1000 HU), stored as signed 16-bit Hounsfield units like a real CT. Its default size is
120 x 90 x 100 mm, so that it fits above a 60 mm tag at real scale.

Usage:
    python3 tools/generate_phantom.py --out data/phantom_ct.vtk
"""

import argparse

import numpy as np


def make_phantom(shape, spacing):
    nz, ny, nx = shape
    z, y, x = np.meshgrid(
        (np.arange(nz) - nz / 2 + 0.5) * spacing[2],
        (np.arange(ny) - ny / 2 + 0.5) * spacing[1],
        (np.arange(nx) - nx / 2 + 0.5) * spacing[0],
        indexing="ij",
    )
    a, b = nx * spacing[0] * 0.45, ny * spacing[1] * 0.42

    volume = np.full(shape, -1000, dtype=np.int16)

    body = (x / a) ** 2 + (y / b) ** 2 <= 1.0
    body &= np.abs(z) <= nz * spacing[2] * 0.45
    volume[body] = 40

    # Spine: a vertical cylinder at the back of the body
    spine = (x ** 2 + (y - b * 0.6) ** 2 <= (a * 0.12) ** 2) & body
    volume[spine] = 700

    # Ribs: thin elliptic shells every 12 mm
    ellipse = np.sqrt((x / (a * 0.85)) ** 2 + (y / (b * 0.85)) ** 2)
    ribs = (np.abs(ellipse - 1.0) < 0.04) & (np.mod(z, 12.0) < 4.0) & (y > -b * 0.5) & body
    volume[ribs] = 600

    # Kidneys
    for side in (-1.0, 1.0):
        kidney = ((x - side * a * 0.45) / (a * 0.15)) ** 2 + ((y - b * 0.25) / (b * 0.22)) ** 2 \
            + (z / (nz * spacing[2] * 0.2)) ** 2 <= 1.0
        volume[kidney] = 150

    noise = np.random.default_rng(0).normal(0.0, 8.0, shape)
    volume = np.where(volume > -1000, volume + noise, volume).astype(np.int16)
    return volume


def write_vtk(path, volume, spacing, origin):
    nz, ny, nx = volume.shape
    with open(path, "wb") as file:
        header = (
            "# vtk DataFile Version 3.0\n"
            "CT AR synthetic phantom (HU)\n"
            "BINARY\n"
            "DATASET STRUCTURED_POINTS\n"
            f"DIMENSIONS {nx} {ny} {nz}\n"
            f"SPACING {spacing[0]} {spacing[1]} {spacing[2]}\n"
            f"ORIGIN {origin[0]} {origin[1]} {origin[2]}\n"
            f"POINT_DATA {volume.size}\n"
            "SCALARS scalars short 1\n"
            "LOOKUP_TABLE default\n"
        )
        file.write(header.encode("ascii"))
        # Legacy VTK binary files are big-endian, x varies fastest.
        file.write(volume.astype(">i2").tobytes(order="C"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="phantom_ct.vtk")
    parser.add_argument("--spacing", type=float, default=1.0, help="isotropic voxel size in mm")
    args = parser.parse_args()

    spacing = (args.spacing,) * 3
    shape = (round(100 / args.spacing), round(90 / args.spacing), round(120 / args.spacing))
    # Non-zero origin on purpose, like a real CT in patient coordinates: ct_placement re-centers it on the tag.
    origin = (-180.0, -150.0, -850.0)
    write_vtk(args.out, make_phantom(shape, spacing), spacing, origin)
    print(f"Wrote {args.out} ({shape[2]}x{shape[1]}x{shape[0]} voxels, {args.spacing} mm)")


if __name__ == "__main__":
    main()
