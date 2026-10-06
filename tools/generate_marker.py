#!/usr/bin/env python3
"""Generate a printable ArUco tag for the CT AR application.

Sight's ArUco tracker uses the DICT_ARUCO_ORIGINAL dictionary. The application tracks the tag id 101 and expects it to
be 60 mm wide (the black square, white margin excluded). Print the PDF at 100% scale ("actual size", no "fit to page")
and measure the black square: if it is not 60 mm, change `pattern_width` in ct_ar/rc/plugin.xml accordingly.

Usage:
    pip install opencv-contrib-python-headless pillow
    python3 tools/generate_marker.py --id 101 --size-mm 60 --out data/aruco_101_60mm
"""

import argparse

import cv2
import numpy as np
from PIL import Image

MM_PER_INCH = 25.4


def generate_marker(marker_id: int, cell_px: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    cells = dictionary.markerSize + 2  # data bits + 1 black border bit on each side
    return cv2.aruco.generateImageMarker(dictionary, marker_id, cells * cell_px, borderBits=1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--id", type=int, default=101, help="marker id in DICT_ARUCO_ORIGINAL (default: 101)")
    parser.add_argument("--size-mm", type=float, default=60.0, help="printed marker width in mm (default: 60)")
    parser.add_argument("--margin-mm", type=float, default=10.0, help="white margin around the marker in mm")
    parser.add_argument("--dpi", type=int, default=600, help="print resolution (default: 600)")
    parser.add_argument("--out", default="aruco_101_60mm", help="output path without extension")
    args = parser.parse_args()

    # Use an integer number of pixels per marker cell, then adjust the stored DPI so that the printed size is exact.
    cells = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL).markerSize + 2
    cell_px = max(1, round(args.size_mm / MM_PER_INCH * args.dpi / cells))
    marker = generate_marker(args.id, cell_px)
    marker_px = marker.shape[0]
    dpi = marker_px / (args.size_mm / MM_PER_INCH)
    margin_px = round(args.margin_mm / MM_PER_INCH * dpi)
    page = np.full((marker_px + 2 * margin_px, marker_px + 2 * margin_px), 255, dtype=np.uint8)
    page[margin_px:margin_px + marker_px, margin_px:margin_px + marker_px] = marker

    # Small caption under the marker, inside the margin, to identify it once printed.
    caption = f"ArUco ORIGINAL id={args.id} - {args.size_mm:g} mm"
    font_scale = margin_px / 120.0
    cv2.putText(page, caption, (margin_px, page.shape[0] - margin_px // 3), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, 128, max(1, round(font_scale * 2)), cv2.LINE_AA)

    image = Image.fromarray(page)
    image.save(f"{args.out}.png", dpi=(dpi, dpi))
    image.save(f"{args.out}.pdf", resolution=dpi)
    print(f"Wrote {args.out}.png and {args.out}.pdf ({args.size_mm:g} mm marker, {marker_px} px at {dpi:.2f} dpi)")


if __name__ == "__main__":
    main()
