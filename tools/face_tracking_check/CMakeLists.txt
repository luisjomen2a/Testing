# Standalone check of ct_ar's face tracking, with OpenCV only (no Sight needed).
#
#   cmake -S tools/face_tracking_check -B build-face-check && cmake --build build-face-check
#   ./build-face-check/face_tracking_check                 # webcam 0
#   ./build-face-check/face_tracking_check --calibration my_webcam.xml --camera 1
#   ./build-face-check/face_tracking_check photo1.jpg photo2.jpg
cmake_minimum_required(VERSION 3.18)
project(face_tracking_check CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4.8 REQUIRED COMPONENTS core imgproc imgcodecs calib3d dnn objdetect videoio highgui)

# Same models as the Sight application (see ct_ar/CMakeLists.txt)
set(MODELS_DIR "${CMAKE_CURRENT_BINARY_DIR}/models")
file(
    DOWNLOAD
    https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
    "${MODELS_DIR}/face_detection_yunet_2023mar.onnx"
    EXPECTED_HASH SHA256=8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4
)
file(
    DOWNLOAD
    https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task
    "${CMAKE_CURRENT_BINARY_DIR}/face_landmarker.task"
    EXPECTED_HASH SHA256=64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff
)
file(
    ARCHIVE_EXTRACT
    INPUT
    "${CMAKE_CURRENT_BINARY_DIR}/face_landmarker.task"
    DESTINATION
    "${MODELS_DIR}"
    PATTERNS
    face_landmarks_detector.tflite
    geometry_pipeline_metadata_landmarks.binarypb
)

set(CT_AR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ct_ar")
add_executable(face_tracking_check main.cpp "${CT_AR_DIR}/face_pose_estimator.cpp")
target_include_directories(face_tracking_check PRIVATE "${CT_AR_DIR}")
target_compile_definitions(face_tracking_check PRIVATE CT_AR_MODELS_DIR="${MODELS_DIR}")
target_link_libraries(face_tracking_check PRIVATE ${OpenCV_LIBS})
