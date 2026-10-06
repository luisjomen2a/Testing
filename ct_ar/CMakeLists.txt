sight_add_target(ct_ar TYPE APP UNIQUE ON)

# Face tracking: YuNet (cv::FaceDetectorYN) and MediaPipe's face mesh run with cv::dnn's TFLite importer.
find_package(
    OpenCV QUIET REQUIRED COMPONENTS opencv_core opencv_imgproc opencv_calib3d opencv_dnn opencv_objdetect
)
if(OpenCV_VERSION VERSION_LESS 4.8)
    message(FATAL_ERROR "ct_ar needs OpenCV >= 4.8 for face tracking (found ${OpenCV_VERSION})")
endif()

target_link_libraries(
    ct_ar
    PUBLIC data
           geometry_data
           io_opencv
           service
           opencv_core
           opencv_imgproc
           opencv_calib3d
           opencv_dnn
           opencv_objdetect
)

# Models, downloaded next to the application resources (like tuto17_simple_ar's sample data)
set(MODULE_PATH "${CMAKE_BINARY_DIR}/${SIGHT_MODULE_RC_PREFIX}/ct_ar")

# YuNet face detector (MIT license) - https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet
file(
    DOWNLOAD
    https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
    "${MODULE_PATH}/face_detection_yunet_2023mar.onnx"
    SHOW_PROGRESS
    EXPECTED_HASH SHA256=8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4
)

# MediaPipe Face Landmarker bundle (Apache 2.0): face mesh network + canonical face geometry
set(FACE_LANDMARKER_TASK "${CMAKE_CURRENT_BINARY_DIR}/face_landmarker.task")
file(
    DOWNLOAD
    https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task
    "${FACE_LANDMARKER_TASK}"
    SHOW_PROGRESS
    EXPECTED_HASH SHA256=64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff
)
file(
    ARCHIVE_EXTRACT
    INPUT
    "${FACE_LANDMARKER_TASK}"
    DESTINATION
    "${MODULE_PATH}"
    PATTERNS
    face_landmarks_detector.tflite
    geometry_pipeline_metadata_landmarks.binarypb
)

add_dependencies(
    ct_ar
    data
    module_app
    module_data # Extracts the camera from the camera set
    module_geometry # Matrix inversion, switch and damping
    module_geometry_vision # ArUco tracking, pose estimation, reprojection error, distortion
    module_io_dicom # DICOM folder reader
    module_io_itk # NIfTI / INR image reader
    module_io_video # Webcam / video file grabber
    module_io_vision # OpenCV camera calibration reader
    module_io_vtk # VTK image and mesh reader
    module_service
    module_sync # Synchronizes the video with the processing pipeline
    module_ui
    module_ui_icons
    module_ui_qt
    module_ui_viz # Fullscreen screen selector
    module_viz_scene3d # 3D rendering: video, volume rendering, slices, meshes
    module_viz_scene3d_qt
    sightrun
)

# Dark theme, like Sight Viewer
module_param(
    module_ui_qt PARAM_LIST resource stylesheet PARAM_VALUES sight::module::ui::qt/flatdark.rcc
                                                             sight::module::ui::qt/flatdark.qss
)

# Main application's configuration to launch
module_param(module_app PARAM_LIST config PARAM_VALUES ct_ar)
