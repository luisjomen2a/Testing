#!/usr/bin/env python3
"""Write an approximate (uncalibrated) pinhole camera model in Sight's calibration format.

Assumes a ~60 degree horizontal field of view, a centered principal point and no lens distortion, which is a fair guess
for most laptop/USB webcams. Good enough to try the application; calibrate your camera for an accurate overlay.

Usage:
    python3 tools/make_default_calibration.py --width 640 --height 480 --out ct_ar/rc/default_calibration.xml
"""

import argparse
import math

import numpy as np

from calibrate_camera import write_sight_calibration


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--hfov", type=float, default=60.0, help="horizontal field of view in degrees")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    focal = args.width / 2.0 / math.tan(math.radians(args.hfov) / 2.0)
    matrix = np.array([[focal, 0.0, args.width / 2.0],
                       [0.0, focal, args.height / 2.0],
                       [0.0, 0.0, 1.0]])
    write_sight_calibration(args.out, args.width, args.height, matrix, np.zeros(5),
                            f"approximate webcam model, hfov={args.hfov:g}deg, no distortion")
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
