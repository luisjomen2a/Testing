# CT AR: a CT scan on an ArUco tag, built with IRCAD Sight

`ct_ar` is a [Sight](https://github.com/IRCAD/sight) application (Sight is IRCAD's Surgical Image Guidance and
Healthcare Toolkit). It:

1. grabs your webcam (or a video file),
2. tracks an ArUco tag with Sight's ArUco tracker and estimates its 3D pose with the camera calibration,
3. displays a CT scan you load (a **DICOM folder**, or a **VTK/ITK file** such as `.vtk`, `.vti`, `.mhd`, `.nii`) on top
   of the tag, as a volume rendering and/or 3D slices, together with any meshes you load (`.vtk`, `.vtp`, `.obj`, `.stl`).

It is based on Sight's `tutorial/xml/tuto17_simple_ar` (ArUco tracking and AR rendering). The cube from that tutorial is
replaced by the medical data, attached to a transform computed by a small C++ service, `ct_ar::ct_placement`, which
centers the CT on the tag.

```
webcam ─► frame_grabber ─► synchronizer ─► aruco_tracker ─► pose_from2d ─► invert ─► damping ─► AR camera
                                                                                                  │
DICOM / VTK / ITK ─► series_set ─► ct_placement (center, scale, offset, sit on tag) ─► transform ─┴─► volume / slices / meshes
```

## Repository layout

| Path | Content |
|------|---------|
| `ct_ar/rc/plugin.xml` | The application: objects, services, UI, signal/slot connections, update loop |
| `ct_ar/ct_placement.{hpp,cpp}` | Service computing the CT-to-tag transform: `lift * offset * scale * translate(-center)` |
| `ct_ar/rc/default_calibration.xml` | Approximate 640x480 webcam calibration loaded at startup |
| `data/aruco_101_60mm.pdf` / `.png` | Printable tag (DICT_ARUCO_ORIGINAL, id 101, 60 mm) |
| `data/phantom_ct.vtk` | Synthetic CT-like phantom (HU values, 120 x 90 x 100 mm) to test without patient data |
| `data/calibration_approx_*.xml` | Approximate calibrations for 640x480 and 1280x720 |
| `tools/calibrate_camera.py` | Chessboard calibration of your webcam, written in Sight's format |
| `tools/generate_marker.py` | Generates other tags/sizes |
| `tools/generate_phantom.py`, `tools/make_default_calibration.py` | Regenerate the test data |
| `scripts/build_with_sight.sh` | Clones Sight, plugs the app into it and builds it |

## Build

Sight is a C++20 framework. It is tested on Ubuntu 22.04/24.04, Debian 12 and Windows with Visual Studio 2022. Its
applications are built inside Sight's CMake project.

1. Install Sight's dependencies (Qt6, VTK, ITK, OGRE, OpenCV with ArUco, DCMTK, Boost, ...) by following the
   [official installation guide](https://sight.pages.ircad.fr/sight-doc/Installation/index.html).
2. Build:

   ```bash
   ./scripts/build_with_sight.sh            # SIGHT_DIR, BUILD_DIR, SIGHT_REF and BUILD_TYPE can be overridden
   ```

   The script clones Sight (at the commit this app was written against), links `ct_ar/` into `sight/app/`, configures
   Sight with tests and examples disabled, and builds only the `ct_ar` target and the modules it uses.

   To do it by hand in an existing Sight checkout: copy or link `ct_ar/` into `sight/app/`, add
   `add_subdirectory(ct_ar)` to `sight/app/CMakeLists.txt`, then build the `ct_ar` target.

3. Run `<build dir>/bin/ct_ar`.

On Windows, follow the same steps with the Sight Visual Studio/vcpkg setup and run `bin\ct_ar.bat`.

## Use

1. **Print the tag**: print `data/aruco_101_60mm.pdf` at 100% scale ("actual size", not "fit to page"). The black
   square must measure 60 mm. If it does not, set `pattern_width` (two places in `ct_ar/rc/plugin.xml`) to the measured
   width in mm, or generate another tag with `tools/generate_marker.py`.
2. **Camera calibration**: an approximate 640x480 calibration is loaded at startup, which is enough to try the app.
   For an accurate overlay, calibrate your webcam with `tools/calibrate_camera.py` (or Sight's `sight_calibrator`) and
   load the file with the **Load calibration** button. Sight refuses to grab at a resolution other than the
   calibrated one, so pick the matching resolution in the camera selector (or load
   `data/calibration_approx_1280x720.xml` for 720p).
3. **Start the video**: choose your webcam (or a video file) with the camera selector in the toolbar, then **Start**.
   The axes appear on the tag once it is detected.
4. **Load a CT**: **Load DICOM folder** (Ctrl+D) or **Load VTK/ITK file** (Ctrl+O). Try `data/phantom_ct.vtk`.
   The volume rendering is shown by default. Toggle slices and meshes from the toolbar.
5. **Adjust** in the side panel (**Settings** button):
   - *CT placement*: scale (1 = real size in mm; a whole CT is much bigger than the tag, so try 0.2 to 0.3), "stand
     on the tag" or centered, and translation/rotation sliders around the tag,
   - *Transfer function*: choose a preset (e.g. `CT-Skin` or `CT-Bones`) or edit the transfer function,
   - *Volume rendering*: samples, opacity correction, ambient occlusion, etc.,
   - *Tracking*: pose damping (less jitter, more lag) and reprojection error display,
   - *ArUco*: detector parameters.
6. **Optional**: **Enable 3D rendering distortion** applies the lens distortion of the calibration to the 3D render, or
   **Enable video image undistortion** undistorts the video.

## Notes and limitations

- `ct_placement` only centers the data on the tag and lets you offset it by hand. Registering the CT to a real patient
  or phantom needs a real registration, for example fiducials visible in the CT, or a tag at a known position in the
  scan.
- The default calibration assumes a 60° horizontal field of view and no lens distortion, so the overlay can drift
  slightly from the tag until you calibrate your camera.
- I could not compile Sight in the environment where this was written. Sight's dependency packages could not be
  downloaded there. The XML was checked against Sight's sources at the pinned commit (service types, data keys, slots)
  and `ct_placement.cpp` was compile-checked against Sight's headers, but the app has not been run yet.
- This is a demo, not a medical device. Do not use it for diagnosis or surgery.

## License

The application configuration is derived from Sight's tutorials, which are LGPL-3.0-or-later. Sight is © IRCAD France.
