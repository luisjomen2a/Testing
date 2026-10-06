#!/usr/bin/env python3
"""Calibrate a webcam with a chessboard and write the result in the format read by Sight.

Sight's `sight::module::io::vision::open_cv_reader` (the "Load calibration" button of the CT AR application) reads an
OpenCV FileStorage file containing `nbCameras` and `camera_0` (`imageWidth`, `imageHeight`, `matrix`, `distortion`,
...). This script produces such a file. Sight's own `sight_calibrator` application can be used instead.

Usage:
    pip install opencv-contrib-python-headless
    # print a chessboard, e.g. https://github.com/opencv/opencv/blob/4.x/doc/pattern.png (9x6 inner corners)
    python3 tools/calibrate_camera.py --camera 0 --width 640 --height 480 --cols 9 --rows 6 --square-mm 25 \
        --out my_webcam_640x480.xml

Show the chessboard to the camera under various angles and distances. Press SPACE to capture a view (at least 10),
C to compute the calibration and save it, Q to quit.

The resolution must be the one you will select in the CT AR application: Sight refuses to grab frames at a resolution
different from the calibrated one.
"""

import argparse

import cv2
import numpy as np


def write_sight_calibration(path, width, height, matrix, distortion, description):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("nbCameras", 1)
    fs.startWriteStruct("camera_0", cv2.FILE_NODE_MAP)
    fs.write("id", "webcam")
    fs.write("description", description)
    fs.write("imageWidth", int(width))
    fs.write("imageHeight", int(height))
    fs.write("matrix", np.asarray(matrix, dtype=np.float64))
    fs.write("distortion", np.asarray(distortion, dtype=np.float64).reshape(1, -1)[:, :5])
    fs.write("scale", 1.0)
    fs.endWriteStruct()
    fs.release()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--cols", type=int, default=9, help="inner corners per chessboard row")
    parser.add_argument("--rows", type=int, default=6, help="inner corners per chessboard column")
    parser.add_argument("--square-mm", type=float, default=25.0, help="chessboard square size in mm")
    parser.add_argument("--out", default="webcam_calibration.xml")
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.camera)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    pattern = (args.cols, args.rows)
    object_points = np.zeros((args.cols * args.rows, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm

    all_object_points, all_image_points = [], []
    size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    while True:
        ok, frame = capture.read()
        if not ok:
            raise SystemExit("Cannot read from the camera")

        size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, pattern, corners, found)
        cv2.putText(display, f"views: {len(all_image_points)}  [SPACE] capture  [C] calibrate  [Q] quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") and found:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            all_object_points.append(object_points)
            all_image_points.append(corners)
        if key == ord("c"):
            if len(all_image_points) < 5:
                print("Capture at least 5 views (10+ recommended)")
                continue
            rms, matrix, distortion, _, _ = cv2.calibrateCamera(
                all_object_points, all_image_points, size, None, None)
            print(f"RMS reprojection error: {rms:.3f} px")
            print(f"Camera matrix:\n{matrix}\nDistortion: {distortion.ravel()}")
            write_sight_calibration(args.out, size[0], size[1], matrix, distortion,
                                    f"chessboard calibration, rms={rms:.3f}px")
            print(f"Wrote {args.out} ({size[0]}x{size[1]})")
            break

    capture.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
