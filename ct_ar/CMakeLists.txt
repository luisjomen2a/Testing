sight_add_target(ct_ar TYPE APP UNIQUE ON)

add_dependencies(
    ct_ar
    data
    module_app
    module_data # Extracts the camera from the camera set
    module_geometry # Matrix inversion and damping
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
    module_viz_scene3d # 3D rendering: video, volume rendering, slices, meshes
    module_viz_scene3d_qt
    sightrun
)

target_link_libraries(ct_ar PUBLIC data geometry_data service)

# Dark theme
module_param(
    module_ui_qt PARAM_LIST resource stylesheet PARAM_VALUES sight::module::ui::qt/flatdark.rcc
                                                             sight::module::ui::qt/flatdark.qss
)

# Main application's configuration to launch
module_param(module_app PARAM_LIST config PARAM_VALUES ct_ar)
