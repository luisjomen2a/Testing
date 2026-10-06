# CT AR: a CT scan on your face (or on an ArUco tag), built with IRCAD Sight

`ct_ar` is a [Sight](https://github.com/IRCAD/sight) application (Sight is IRCAD's Surgical Image Guidance and
Healthcare Toolkit). It grabs a calibrated webcam, tracks an **anchor** and renders a CT scan on it in augmented
reality:

- **Face mode** (default): your face is detected and its 3D pose estimated every frame. The CT's nose tip is placed
  on your nose tip, with the patient axes along your head axes, so a head CT appears "inside" your head.
- **Tag mode**: an ArUco tag is tracked with Sight's own ArUco pipeline and the CT stands on it.

The CT is a **DICOM folder** or a **VTK/ITK file** (`.vtk`, `.vti`, `.mhd`, `.nii`...), shown with Sight's volume
rendering and/or 3D slices, plus any meshes you load (`.vtk`, `.vtp`, `.obj`, `.stl`).

The UI follows **Sight Viewer**: dark "flatdark" theme, collapsible parameter panel with an icon tab bar (camera and
tracking, data, rendering, placement), thumbnail cards for the loaded series, Sight's transfer function editor with its
CT presets, and the AR view with floating toolbars, tracking status indicators and a progress bar.

## Architecture

Everything is standard Sight: XML configuration of objects, services, signals and slots. Two new C++ services
are added, in the app's own module.

```
                 ┌─► aruco_tracker ─► pose_from2d ─► reprojection_error ─► marker_to_camera ─┐
webcam ─► grabber ─► synchronizer ─► frame                                                     ├─► switch_matrices (mode)
                 └─► ct_ar::face_tracker ────────────────────────────────► face_to_camera ───┘           │
                                                                                                 anchor_to_camera
                                                                                                          │
                                                       AR camera (intrinsics) ◄─ damping ◄─ inverse ◄─────┘
DICOM / VTK / ITK ─► series_set ─► ct_ar::ct_placement (mode, scale, offset) ─► transform node ─► volume / slices / meshes
```

All of it runs in one `<sequence>` per video frame on a worker thread, then the scene renders. The tracking mode is a
single integer, bound to the tag/face switch button. It drives `switch_matrices`, the face tracker (which only runs in
face mode) and the placement.

### `ct_ar::face_tracker` (face detection and head pose, in C++)

Implemented in `ct_ar/face_pose_estimator.{hpp,cpp}` (OpenCV only) and wrapped as a Sight service in
`ct_ar/face_tracker.{hpp,cpp}`:

1. **Detection**: [YuNet](https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet)
   (`cv::FaceDetectorYN`) finds the face box and the eyes. It runs only when no face is tracked.
2. **Landmarks**: MediaPipe's face mesh network (from the
   [Face Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/face_landmarker) bundle) runs through
   OpenCV's `dnn` TFLite importer. It predicts 478 landmarks in a 256×256 crop centered on the face and rotated so that
   the eyes are horizontal. The next frame's crop is computed from the current landmarks, so the face is tracked without
   re-detection. The network's face presence score tells when the track is lost.
3. **Head pose**: weighted PnP (SQPnP initialization, then Levenberg-Marquardt on the weighted reprojection error).
   The 2D landmarks are fitted to MediaPipe's **metric canonical face**, using the 33 rigid landmarks and weights
   MediaPipe uses for its own Procrustes alignment (nose bridge, eye corners, cheekbones, forehead; no lips or jaw).
   The canonical face is decoded at runtime from the bundle's `geometry_pipeline_metadata_landmarks.binarypb`, so no
   coordinates are hard-coded. Frames are undistorted first when the camera is calibrated.

The output, `face_to_camera`, has its origin at the nose tip, x towards the subject's left, y up, z out of the face.
It plays exactly the role of the tag pose in Sight's ArUco pipeline.

### `ct_ar::ct_placement`

Computes the CT-to-anchor transform:

- **face mode**: `offset * scale * lps_to_face * translate(-ct_nose_tip)`. The CT nose tip is found automatically:
  it's the most anterior skin voxel (above a HU threshold) at mid-height of the body, refined to a 2 mm centroid. It
  respects the image's origin, spacing and orientation, and is cached per image.
- **tag mode**: `lift * offset * scale * translate(-center)`. The CT is centered on the tag and stands on it.

`offset` comes from the transform editor (translation in mm and rotation in degrees, around the nose tip or the tag
center). `scale` defaults to 1, which is real size.

## Repository layout

| Path | Content |
|------|---------|
| `ct_ar/rc/plugin.xml` | The application: objects, services, UI, connections, update sequence |
| `ct_ar/face_pose_estimator.*` | Face detection, landmarks, tracking, PnP (OpenCV only) |
| `ct_ar/face_tracker.*` | Sight service wrapping it (frames, camera, overlay, signals) |
| `ct_ar/ct_placement.*`, `ct_ar/nose_tip.hpp` | CT placement service, CT nose tip search |
| `ct_ar/CMakeLists.txt` | Sight target, downloads and checks the face models (SHA-256) at configure time |
| `ct_ar/rc/default_calibration.xml` | Approximate 640×480 webcam calibration loaded at startup |
| `tools/face_tracking_check/` | Standalone (OpenCV only) program to test face tracking on your webcam, without Sight |
| `data/head_phantom.vtk` | Synthetic head CT (skin, skull, orbits, nose, teeth, spine) to try face mode |
| `data/phantom_ct.vtk` | Synthetic torso-like CT to try tag mode |
| `data/aruco_101_60mm.pdf` / `.png` | Printable tag (DICT_ARUCO_ORIGINAL, id 101, 60 mm) |
| `data/calibration_approx_*.xml` | Approximate calibrations for 640×480 and 1280×720 |
| `tools/calibrate_camera.py` | Chessboard calibration of your webcam, written in Sight's format |
| `tools/generate_*.py`, `tools/make_default_calibration.py` | Regenerate the tag and the test data |
| `scripts/build_with_sight.sh` | Clones Sight, plugs the app into it and builds it |

## Build

Requirements: Sight's dependencies (Qt6, VTK, ITK, OGRE, DCMTK, Boost...), and **OpenCV ≥ 4.8** with the `dnn` and
`objdetect` modules. Sight already needs OpenCV's contrib `aruco` module. OpenCV builds its TFLite importer by default
since 4.8, using bundled flatbuffers. See Sight's
[installation guide](https://sight.pages.ircad.fr/sight-doc/Installation/index.html).

```bash
./scripts/build_with_sight.sh            # SIGHT_DIR, BUILD_DIR, SIGHT_REF, BUILD_TYPE, INSTALL_DIR, PYTHON can be set
<build dir>/bin/ct_ar
```

The script clones Sight at the commit this app was written against and links `ct_ar/` into `sight/app/`. It then
configures Sight with tests and examples off and builds only `ct_ar` and the modules it uses. To do it by hand: link
`ct_ar/` into `sight/app/`, add `add_subdirectory(ct_ar)` to `sight/app/CMakeLists.txt`, then build the `ct_ar`
target.

At configure time, CMake downloads two models and checks their SHA-256: YuNet (232 KB, MIT license), and MediaPipe's
Face Landmarker bundle (3.7 MB, Apache 2.0), from which it extracts the face mesh network and the canonical face.

### Try face tracking first, without Sight

Building Sight takes a while. To check face tracking on your webcam in a couple of minutes (OpenCV ≥ 4.8 only):

```bash
cmake -S tools/face_tracking_check -B build-face-check && cmake --build build-face-check
./build-face-check/face_tracking_check                         # webcam 0, approximate intrinsics
./build-face-check/face_tracking_check --calibration my_webcam.xml --camera 1
```

It shows the landmarks, the head axes at your nose tip, distance/yaw/pitch/roll and the processing time.

### Troubleshooting (Ubuntu 24.04)

- `Could NOT find Python3 (missing: Development.Module)`: Ubuntu's VTK package needs the headers of the *system*
  Python. Run `sudo apt install python3-dev`. The script forces `-DPython3_EXECUTABLE=/usr/bin/python3` so that a
  pyenv/conda/uv Python on your `PATH` is not used.
- `nvplConfig.cmake` warnings are harmless. They come from CMake's `FindBLAS`, called by Ceres/SuiteSparse. Add
  `-DBLA_VENDOR=OpenBLAS` to the script to silence them.
- `CMAKE_INSTALL_PREFIX (/usr/local) isn't empty`: the script now uses an empty `sight-install/` folder.
- `ct_ar needs OpenCV >= 4.8`: Ubuntu 24.04 ships OpenCV 4.6. Build a recent OpenCV with contrib (for `aruco`) and
  point CMake to it with `-DOpenCV_DIR=...`.

## Use

1. **Start the camera** (*Camera and tracking* page): pick your webcam in the camera selector, then **Start**. An
   approximate 640×480 calibration is loaded at startup. For an accurate overlay, calibrate your webcam with
   `tools/calibrate_camera.py` (or Sight's `sight_calibrator`) and load the file with **Load camera calibration**.
   Sight only grabs at the calibrated resolution, so pick the matching resolution in the selector, or load
   `data/calibration_approx_1280x720.xml` for 720p.
2. **Choose the anchor**: **Face** (default) or **ArUco tag**. The *Face* and *Tag* indicators at the top right of
   the view turn green when tracked, and the RMS reprojection error is shown next to them.
3. **Load a CT** (*Data* page): **Load DICOM folder** (Ctrl+D) or **Load VTK/ITK file** (Ctrl+O). Try
   `data/head_phantom.vtk` with your face, or `data/phantom_ct.vtk` with the tag.
4. **Rendering** page: choose a transfer function preset in Sight's editor (e.g. `CT-Skin`, `CT-Bones`) or edit it,
   and tune the volume rendering quality. Volume, slices, meshes and axes are toggled from the floating toolbar.
5. **Placement** page: scale (1 = real size), translation and rotation around the nose tip or the tag center. For face
   mode there is also the skin threshold used to find the CT's nose tip.
6. For tag mode, print `data/aruco_101_60mm.pdf` at 100% ("actual size"). The black square must measure 60 mm;
   otherwise set `pattern_width` (two places in `ct_ar/rc/plugin.xml`) to the measured width.

## What was verified, and what was not

- **Face tracking** (`face_pose_estimator`, C++) was built against OpenCV 4.10 and run on test photos. It agrees with
  MediaPipe's own Face Landmarker within 1–1.5° on head orientation and within 2% on distance. It runs in 6–8 ms per
  frame while tracking (measured on this sandbox's 4-core CPU), and it rejects images without faces. On a synthetic moving sequence
  (shift, zoom, ±12° roll) it tracked 60/60 frames without re-detection, with the expected roll and depth.
- **CT nose tip search** found the nose of the head phantom, including with a flipped image orientation.
- **Model downloads** (URLs, SHA-256, extraction from the bundle) were run with CMake in script mode.
- **Sight integration**: `face_tracker.cpp`, `ct_placement.cpp` and `plugin.cpp` were compile-checked against Sight's
  headers (pinned commit) and OpenCV 4.10. `plugin.xml` was checked against Sight's sources: all 35 service types
  exist, and every referenced object, service, map key and icon exists. **The full application has not been built or
  run**: Sight's dependencies could not be downloaded in the environment where this was written.
- Limitations: the face is a generic average face scaled to metric size, so the CT is not *registered* to your
  anatomy. Use the scale and offset sliders to fit it. A real registration needs patient-specific landmarks.
  This is a demo, not a medical device.

## License

The application configuration is derived from Sight's tutorials (LGPL-3.0-or-later; Sight is © IRCAD France). The
downloaded models keep their licenses: YuNet (MIT) and MediaPipe Face Landmarker (Apache 2.0).
