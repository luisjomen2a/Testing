#include "face_tracker.hpp"

#include <core/runtime/path.hpp>
#include <core/spy_log.hpp>

#include <geometry/data/matrix4.hpp>

#include <io/opencv/camera.hpp>
#include <io/opencv/image.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace ct_ar
{

namespace
{

/// Converts any Sight video frame to a BGR cv::Mat (copy).
cv::Mat to_bgr(const cv::Mat& _image, sight::data::image::pixel_format_t _format)
{
    cv::Mat bgr;
    switch(_format)
    {
        case sight::data::image::rgba:
            cv::cvtColor(_image, bgr, cv::COLOR_RGBA2BGR);
            break;

        case sight::data::image::bgra:
            cv::cvtColor(_image, bgr, cv::COLOR_BGRA2BGR);
            break;

        case sight::data::image::rgb:
            cv::cvtColor(_image, bgr, cv::COLOR_RGB2BGR);
            break;

        case sight::data::image::bgr:
            bgr = _image.clone();
            break;

        case sight::data::image::gray_scale:
            cv::cvtColor(_image, bgr, cv::COLOR_GRAY2BGR);
            break;

        default:
            break;
    }

    return bgr;
}

//------------------------------------------------------------------------------

std::string resource(const std::string& _path)
{
    return sight::core::runtime::get_resource_file_path(_path).string();
}

} // namespace

//-----------------------------------------------------------------------------

face_tracker::face_tracker() noexcept :
    filter(has_signals::signals())
{
    new_signal<signals::face_detected_t>(signals::FACE_DETECTED);
    new_signal<signals::error_computed_t>(signals::ERROR_COMPUTED);
}

//-----------------------------------------------------------------------------

face_tracker::~face_tracker() noexcept = default;

//-----------------------------------------------------------------------------

void face_tracker::configuring(const config_t& _config)
{
    m_estimator_config.detector_model = resource(
        _config.get<std::string>("models.<xmlattr>.detector", "ct_ar/face_detection_yunet_2023mar.onnx")
    );
    m_estimator_config.landmark_model = resource(
        _config.get<std::string>("models.<xmlattr>.landmarks", "ct_ar/face_landmarks_detector.tflite")
    );
    m_estimator_config.geometry_metadata = resource(
        _config.get<std::string>(
            "models.<xmlattr>.geometry",
            "ct_ar/geometry_pipeline_metadata_landmarks.binarypb"
        )
    );
}

//-----------------------------------------------------------------------------

void face_tracker::starting()
{
    try
    {
        m_estimator = std::make_unique<face_pose_estimator>(m_estimator_config);
    }
    catch(const std::exception& e)
    {
        SIGHT_ERROR("Face tracking is unavailable, the models could not be loaded: " << e.what());
        m_estimator.reset();
    }

    m_camera_timestamp = 0;
}

//-----------------------------------------------------------------------------

void face_tracker::stopping()
{
    m_estimator.reset();
}

//-----------------------------------------------------------------------------

void face_tracker::updating()
{
    const bool active = *m_mode == 1;
    if(!active)
    {
        if(m_was_active)
        {
            // Clear the overlay when leaving the face mode.
            if(auto debug_frame = m_debug_frame.lock(); debug_frame && debug_frame->size_in_bytes() > 0)
            {
                cv::Mat overlay = sight::io::opencv::image::move_to_cv(debug_frame.get_shared());
                overlay.setTo(cv::Scalar::all(0));
                debug_frame->async_emit(sight::data::signals::MODIFIED);
            }
        }

        m_was_active = false;
        return;
    }

    if(!m_was_active && m_estimator)
    {
        m_estimator->reset();
    }

    m_was_active = true;

    if(!m_estimator)
    {
        this->async_emit(signals::FACE_DETECTED, false);
        return;
    }

    // Grab the frame
    cv::Mat bgr;
    sight::data::image::size_t frame_size;
    {
        const auto frame = m_frame.lock();
        if(!frame || frame->size_in_bytes() == 0)
        {
            return;
        }

        frame_size = frame->size();
        const cv::Mat image = sight::io::opencv::image::move_to_cv(frame.get_shared());
        bgr = to_bgr(image, frame->pixel_format());
    }

    if(bgr.empty())
    {
        SIGHT_ERROR("Unsupported video frame format for face tracking");
        return;
    }

    // Undistortion maps and intrinsics, when the camera or the frame size changes
    {
        const auto camera = m_camera.lock();
        if(camera->last_modified() != m_camera_timestamp || bgr.size() != m_frame_size)
        {
            auto [intrinsic, size, distortion] = sight::io::opencv::camera::copy_to_cv(camera.get_shared());
            cv::Mat camera_matrix;
            intrinsic.convertTo(camera_matrix, CV_64F);
            if(size.width > 0 && size != bgr.size())
            {
                // Rescale the intrinsics to the actual video resolution (same sensor, scaled output).
                camera_matrix.row(0) *= static_cast<double>(bgr.cols) / size.width;
                camera_matrix.row(1) *= static_cast<double>(bgr.rows) / size.height;
            }

            m_camera_matrix = cv::Matx33d(camera_matrix);
            if(camera->get_is_calibrated() && cv::countNonZero(distortion) > 0)
            {
                cv::initUndistortRectifyMap(
                    camera_matrix,
                    distortion,
                    cv::noArray(),
                    camera_matrix,
                    bgr.size(),
                    CV_16SC2,
                    m_map_x,
                    m_map_y
                );
            }
            else
            {
                m_map_x.release();
                m_map_y.release();
            }

            m_camera_timestamp = camera->last_modified();
            m_frame_size       = bgr.size();
        }
    }

    if(!m_map_x.empty())
    {
        cv::Mat undistorted;
        cv::remap(bgr, undistorted, m_map_x, m_map_y, cv::INTER_LINEAR);
        bgr = undistorted;
    }

    m_estimator->set_presence_threshold(static_cast<float>(*m_presence_threshold));
    const face_estimate estimate = m_estimator->process(bgr, m_camera_matrix);

    if(estimate.found)
    {
        auto pose = m_pose.lock();
        for(std::size_t i = 0 ; i < 4 ; ++i)
        {
            for(std::size_t j = 0 ; j < 4 ; ++j)
            {
                (*pose)(i, j) = estimate.face_to_camera(static_cast<int>(i), static_cast<int>(j));
            }
        }

        pose->async_emit(sight::data::signals::MODIFIED);
        this->async_emit(signals::ERROR_COMPUTED, estimate.reprojection_error);
    }

    // Overlay: landmarks drawn into a transparent RGBA image
    if(auto debug_frame = m_debug_frame.lock(); debug_frame)
    {
        if(debug_frame->size() != frame_size)
        {
            debug_frame->resize(frame_size, sight::core::type::UINT8, sight::data::image::rgba);
        }

        cv::Mat overlay = sight::io::opencv::image::move_to_cv(debug_frame.get_shared());
        overlay.setTo(cv::Scalar::all(0));
        if(*m_show_landmarks && !estimate.landmarks.empty())
        {
            // Sight's steel blue accent when tracked, orange when the pose is rejected.
            const cv::Scalar color = estimate.found ? cv::Scalar(124, 171, 207, 220) : cv::Scalar(217, 140, 34, 220);
            for(const auto& point : estimate.landmarks)
            {
                cv::circle(overlay, point, 1, color, cv::FILLED, cv::LINE_AA);
            }
        }

        debug_frame->async_emit(sight::data::signals::MODIFIED);
    }

    this->async_emit(signals::FACE_DETECTED, estimate.found);
    this->async_emit(filter::signals::SUCCEEDED);
}

} // namespace ct_ar
