#pragma once

#include "face_pose_estimator.hpp"

#include <core/com/signal.hpp>

#include <data/boolean.hpp>
#include <data/camera.hpp>
#include <data/image.hpp>
#include <data/integer.hpp>
#include <data/matrix4.hpp>
#include <data/real.hpp>

#include <service/filter.hpp>

#include <memory>

namespace ct_ar
{

/**
 * @brief Tracks the user's face in the video and outputs its 6-DoF pose, like the ArUco pipeline does for a tag.
 *
 * Algorithm (see face_pose_estimator): YuNet face detection, MediaPipe face mesh landmarks run with OpenCV's dnn
 * module, and weighted PnP against MediaPipe's metric canonical face, using the calibrated camera intrinsics. The
 * frame is undistorted first when the camera is calibrated.
 *
 * The output transform maps the face frame to the camera frame (mm). The face frame has its origin at the nose tip,
 * x towards the subject's left, y up and z out of the face. It plays the same role as the "marker to camera" matrix of
 * the ArUco pipeline, so the rest of the AR pipeline (inversion, damping, AR camera) is unchanged.
 *
 * @section Signals Signals
 * - \b face_detected(bool): emitted after each processed frame.
 * - \b error_computed(double): weighted RMS reprojection error of the landmarks, in pixels.
 *
 * @section XML XML configuration
 * @code{.xml}
   <service uid="..." type="ct_ar::face_tracker" worker="tracking" auto_connect="false">
        <data camera="${...}" />
        <data frame="${...}" />
        <data pose="${...}" />
        <data debug_frame="${...}" />
        <models detector="ct_ar/face_detection_yunet_2023mar.onnx"
                landmarks="ct_ar/face_landmarks_detector.tflite"
                geometry="ct_ar/geometry_pipeline_metadata_landmarks.binarypb" />
        <config mode="${...}" show_landmarks="true" presence_threshold="0.5" />
   </service>
   @endcode
 *
 * @subsection Input Input
 * - \b data.camera [sight::data::camera]: calibrated camera.
 * - \b data.frame [sight::data::image]: current video frame (RGB(A)/BGR(A)/gray).
 *
 * @subsection In-Out In-Out
 * - \b data.pose [sight::data::matrix4]: face to camera transform, updated when the face is found.
 * - \b data.debug_frame [sight::data::image] (optional): RGBA overlay with the landmarks, transparent elsewhere.
 *
 * @subsection Configuration Configuration
 * - \b models (optional): resource paths of the models, defaults shown above.
 * - \b config.mode (int, default=1): tracking mode of the application. The tracker only runs when it is 1 (face), so
 *   that the ArUco and face pipelines can share a sequence and a mode switch.
 * - \b config.show_landmarks (bool, default=true): draws the landmarks into the debug frame.
 * - \b config.presence_threshold (double, default=0.5): minimum face presence probability.
 */
class face_tracker final : public sight::service::filter
{
public:

    SIGHT_DECLARE_SERVICE(face_tracker, sight::service::filter);

    struct signals
    {
        using face_detected_t  = sight::core::com::signal<void (bool)>;
        using error_computed_t = sight::core::com::signal<void (double)>;
        static inline const signal_key_t FACE_DETECTED  = "face_detected";
        static inline const signal_key_t ERROR_COMPUTED = "error_computed";
    };

    face_tracker() noexcept;
    ~face_tracker() noexcept final;

protected:

    /// Reads the model paths.
    void configuring(const config_t& _config) final;

    /// Loads the models.
    void starting() final;

    /// Processes the current frame.
    void updating() final;

    /// Releases the models.
    void stopping() final;

private:

    face_pose_estimator::config m_estimator_config;
    std::unique_ptr<face_pose_estimator> m_estimator;

    /// Undistortion maps, recomputed when the camera changes.
    cv::Mat m_map_x;
    cv::Mat m_map_y;
    cv::Matx33d m_camera_matrix {cv::Matx33d::eye()};
    std::uint64_t m_camera_timestamp {0};
    cv::Size m_frame_size;
    bool m_was_active {false};

    sight::data::ptr<sight::data::camera, sight::data::access::in> m_camera {this, "data.camera"};
    sight::data::ptr<sight::data::image, sight::data::access::in> m_frame {this, "data.frame"};
    sight::data::ptr<sight::data::matrix4, sight::data::access::inout> m_pose {this, "data.pose"};
    sight::data::ptr<sight::data::image, sight::data::access::inout> m_debug_frame {this, "data.debug_frame", true};

    sight::data::ptr<sight::data::integer, sight::data::access::in> m_mode {this, "config.mode", 1};
    sight::data::ptr<sight::data::boolean, sight::data::access::in> m_show_landmarks {this, "config.show_landmarks",
                                                                                       true
    };
    sight::data::ptr<sight::data::real, sight::data::access::in> m_presence_threshold {this,
                                                                                       "config.presence_threshold", 0.5
    };
};

} // namespace ct_ar
