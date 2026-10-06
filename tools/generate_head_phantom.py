#!/usr/bin/env python3
"""Generate a synthetic head CT (legacy VTK structured points, Hounsfield units, LPS axes) to test face tracking.

Contains skin, fat, a skull shell with orbits, a brain, eyes, a nose with cartilage and teeth, so that the "Skin",
"Bone" and "Skin + bone" presets all show something. Not anatomically accurate, just plausible proportions
(~150 x 200 x 220 mm). The face points towards -y (anterior), like a real supine head CT.

Usage:
    python3 tools/generate_head_phantom.py --out data/head_phantom.vtk
"""

import argparse

import numpy as np

from generate_phantom import write_vtk


def ellipsoid(x, y, z, center, radii):
    return ((x - center[0]) / radii[0]) ** 2 + ((y - center[1]) / radii[1]) ** 2 + ((z - center[2]) / radii[2]) ** 2


def make_head(spacing: float):
    # Grid in LPS mm, centered on the head
    xs = np.arange(-80, 80 + 1e-6, spacing)
    ys = np.arange(-115, 105 + 1e-6, spacing)
    zs = np.arange(-120, 105 + 1e-6, spacing)
    z, y, x = np.meshgrid(zs, ys, xs, indexing="ij")
    hu = np.full(z.shape, -1000.0)

    cranium = ellipsoid(x, y, z, (0, 5, 25), (72, 92, 78))
    face = ellipsoid(x, y, z, (0, -38, -35), (60, 55, 72))
    neck = (x / 52) ** 2 + ((y - 12) / 55) ** 2 <= 1.0
    neck &= (z < -60)
    head = (cranium <= 1.0) | (face <= 1.0) | neck

    # Nose: a wedge growing out of the face, tip around y = -103 mm
    nose_z = (z > -45) & (z < 5)
    depth = 12 + 16 * (1 - np.clip((z + 45) / 50, 0, 1)) ** 0.6  # deeper at the bottom
    half_width = 3 + 13 * (1 - np.clip((z + 45) / 50, 0, 1))
    nose = nose_z & (np.abs(x) < half_width * (1 + (y + 80) / depth).clip(0, 1)) & (y < -80) & (y > -80 - depth)
    head |= nose

    hu[head] = 30  # soft tissue
    # Subcutaneous fat layer and skin
    inner = (ellipsoid(x, y, z, (0, 5, 25), (66, 86, 72)) <= 1.0) | (ellipsoid(x, y, z, (0, -38, -35), (54, 49, 66)) <= 1.0)
    hu[head & ~inner] = -90
    hu[nose] = 40

    # Skull: shell between two ellipsoids, open at the orbits
    skull_out = ellipsoid(x, y, z, (0, 5, 25), (64, 84, 70)) <= 1.0
    skull_in = ellipsoid(x, y, z, (0, 5, 25), (58, 78, 64)) <= 1.0
    skull = skull_out & ~skull_in
    # Facial bones / maxilla / mandible
    jaw = (ellipsoid(x, y, z, (0, -40, -40), (50, 45, 55)) <= 1.0) & ~(ellipsoid(x, y, z, (0, -38, -38), (45, 40, 50)) <= 1.0)
    jaw &= z < -5
    bone = skull | jaw
    orbits = [(ellipsoid(x, y, z, (side * 32, -62, 2), (17, 22, 15)) <= 1.0) for side in (-1, 1)]
    for orbit in orbits:
        bone &= ~orbit
    hu[bone] = 900
    hu[skull_in] = 35  # brain
    ventricles = [ellipsoid(x, y, z, (side * 10, 10, 35), (6, 25, 10)) <= 1.0 for side in (-1, 1)]
    for v in ventricles:
        hu[v] = 5

    # Eyes in the orbits
    for side in (-1, 1):
        eye = ellipsoid(x, y, z, (side * 32, -66, 2), (12, 12, 12)) <= 1.0
        hu[eye] = 15
        lens = ellipsoid(x, y, z, (side * 32, -77, 2), (4, 2, 4)) <= 1.0
        hu[lens] = 120

    # Teeth: an arc of dense bone
    for angle in np.linspace(-1.1, 1.1, 14):
        tx, ty = 26 * np.sin(angle), -38 - 30 * np.cos(angle)
        hu[ellipsoid(x, y, z, (tx, ty, -52), (3.2, 3.2, 8)) <= 1.0] = 2200

    # Cervical spine
    for k in range(5):
        hu[ellipsoid(x, y, z, (0, 25, -70 - k * 14), (11, 11, 5)) <= 1.0] = 700

    hu += np.random.default_rng(1).normal(0, 6, hu.shape) * (hu > -1000)
    origin = (xs[0], ys[0], zs[0])
    return hu.astype(np.int16), (spacing, spacing, spacing), origin


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="head_phantom.vtk")
    parser.add_argument("--spacing", type=float, default=1.5, help="isotropic voxel size in mm")
    args = parser.parse_args()
    volume, spacing, origin = make_head(args.spacing)
    write_vtk(args.out, volume, spacing, origin)
    nz, ny, nx = volume.shape
    print(f"Wrote {args.out} ({nx}x{ny}x{nz} voxels, {args.spacing} mm)")


if __name__ == "__main__":
    main()
