// Standalone check of ct_ar's face tracking (ct_ar/face_pose_estimator), with OpenCV only.
//
//   face_tracking_check [--calibration file.xml] [--camera N | --video file | image...]
//
// With a camera or a video, shows the landmarks, the head axes (at the nose tip) and the frame rate; press Q or Esc
// to quit. With images, prints the pose of each face and writes <image>_face.png next to it.

#include "face_pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#ifndef CT_AR_MODELS_DIR
#define CT_AR_MODELS_DIR "."
#endif

namespace
{

struct camera_model
{
    cv::Matx33d matrix;
    cv::Mat distortion;
};

/// Camera calibration in Sight's format (nbCameras / camera_0), or an approximate 60 degrees HFOV pinhole model.
camera_model load_camera(const std::string& _calibration, const cv::Size& _size)
{
    camera_model camera;
    if(!_calibration.empty())
    {
        cv::FileStorage storage(_calibration, cv::FileStorage::READ);
        const cv::FileNode node = storage["camera_0"];
        cv::Mat matrix;
        node["matrix"] >> matrix;
        node["distortion"] >> camera.distortion;
        const double sx = _size.width / static_cast<double>(node["imageWidth"].real());
        const double sy = _size.height / static_cast<double>(node["imageHeight"].real());
        matrix.row(0) *= sx;
        matrix.row(1) *= sy;
        camera.matrix = cv::Matx33d(matrix);
        return camera;
    }

    const double focal = _size.width / 2. / std::tan(30. * CV_PI / 180.);
    camera.matrix = cv::Matx33d(focal, 0, _size.width / 2., 0, focal, _size.height / 2., 0, 0, 1);
    return camera;
}

//------------------------------------------------------------------------------

void draw(cv::Mat& _image, const ct_ar::face_estimate& _estimate, const cv::Matx33d& _camera_matrix)
{
    const cv::Scalar landmarks_color = _estimate.found ? cv::Scalar(207, 171, 124) : cv::Scalar(34, 140, 217);
    for(const auto& point : _estimate.landmarks)
    {
        cv::circle(_image, point, 1, landmarks_color, cv::FILLED, cv::LINE_AA);
    }

    if(!_estimate.found)
    {
        return;
    }

    // Head axes at the nose tip: x (subject's left) red, y (up) green, z (out of the face) blue, 50 mm long
    const auto& pose = _estimate.face_to_camera;
    const cv::Matx33d rotation(pose(0, 0), pose(0, 1), pose(0, 2), pose(1, 0), pose(1, 1), pose(1, 2), pose(2, 0),
                               pose(2, 1), pose(2, 2));
    cv::Vec3d rvec;
    cv::Rodrigues(rotation, rvec);
    const cv::Vec3d tvec(pose(0, 3), pose(1, 3), pose(2, 3));
    const std::vector<cv::Point3d> axes {{0, 0, 0}, {50, 0, 0}, {0, 50, 0}, {0, 0, 50}};
    std::vector<cv::Point2d> projected;
    cv::projectPoints(axes, rvec, tvec, _camera_matrix, cv::noArray(), projected);
    cv::line(_image, projected[0], projected[1], {0, 0, 255}, 2, cv::LINE_AA);
    cv::line(_image, projected[0], projected[2], {0, 255, 0}, 2, cv::LINE_AA);
    cv::line(_image, projected[0], projected[3], {255, 0, 0}, 2, cv::LINE_AA);
}

//------------------------------------------------------------------------------

std::string describe(const ct_ar::face_estimate& _estimate)
{
    if(!_estimate.found)
    {
        return "no face";
    }

    const auto& p = _estimate.face_to_camera;
    // Yaw / pitch / roll of the head, in degrees, from the face axes expressed in the camera frame.
    const double yaw   = std::atan2(-p(0, 2), -p(2, 2)) * 180. / CV_PI;
    const double pitch = std::asin(std::clamp(-p(1, 2), -1., 1.)) * 180. / CV_PI;
    const double roll  = std::atan2(-p(0, 1), -p(1, 1)) * 180. / CV_PI;
    char text[160];
    std::snprintf(text, sizeof(text), "nose at %.0f mm | yaw %.0f pitch %.0f roll %.0f | RMS %.1f px",
                  std::sqrt(p(0, 3) * p(0, 3) + p(1, 3) * p(1, 3) + p(2, 3) * p(2, 3)), yaw, pitch, roll,
                  _estimate.reprojection_error);
    return text;
}

} // namespace

//------------------------------------------------------------------------------

int main(int _argc, char** _argv)
{
    std::string calibration;
    std::string video;
    int camera_index = -1;
    std::vector<std::string> images;
    for(int i = 1 ; i < _argc ; ++i)
    {
        const std::string arg = _argv[i];
        if(arg == "--calibration" && i + 1 < _argc)
        {
            calibration = _argv[++i];
        }
        else if(arg == "--camera" && i + 1 < _argc)
        {
            camera_index = std::stoi(_argv[++i]);
        }
        else if(arg == "--video" && i + 1 < _argc)
        {
            video = _argv[++i];
        }
        else
        {
            images.push_back(arg);
        }
    }

    if(images.empty() && video.empty() && camera_index < 0)
    {
        camera_index = 0;
    }

    const std::string models = CT_AR_MODELS_DIR;
    ct_ar::face_pose_estimator estimator({
            .detector_model    = models + "/face_detection_yunet_2023mar.onnx",
            .landmark_model    = models + "/face_landmarks_detector.tflite",
            .geometry_metadata = models + "/geometry_pipeline_metadata_landmarks.binarypb"
        });

    for(const auto& path : images)
    {
        cv::Mat image = cv::imread(path);
        if(image.empty())
        {
            std::cerr << path << ": cannot read\n";
            continue;
        }

        const auto camera = load_camera(calibration, image.size());
        estimator.reset();
        const auto estimate = estimator.process(image, camera.matrix);
        std::cout << path << ": " << describe(estimate) << "\n";
        draw(image, estimate, camera.matrix);
        const auto out = std::filesystem::path(path).replace_extension().string() + "_face.png";
        cv::imwrite(out, image);
    }

    if(images.empty())
    {
        cv::VideoCapture capture;
        if(video.empty())
        {
            capture.open(camera_index);
        }
        else
        {
            capture.open(video);
        }

        if(!capture.isOpened())
        {
            std::cerr << "Cannot open the video source\n";
            return 1;
        }

        cv::Mat frame;
        cv::Mat undistorted;
        camera_model camera;
        cv::Mat map_x;
        cv::Mat map_y;
        double fps = 0.;
        while(capture.read(frame))
        {
            if(map_x.empty() && camera.distortion.empty())
            {
                camera = load_camera(calibration, frame.size());
                if(!camera.distortion.empty() && cv::countNonZero(camera.distortion) > 0)
                {
                    cv::initUndistortRectifyMap(camera.matrix, camera.distortion, cv::noArray(), camera.matrix,
                                                frame.size(), CV_16SC2, map_x, map_y);
                }
            }

            if(!map_x.empty())
            {
                cv::remap(frame, undistorted, map_x, map_y, cv::INTER_LINEAR);
                frame = undistorted;
            }

            const auto start    = std::chrono::steady_clock::now();
            const auto estimate = estimator.process(frame, camera.matrix);
            const double ms     = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                                  .count();
            fps = 0.9 * fps + 0.1 * (1000. / std::max(ms, 1.));

            draw(frame, estimate, camera.matrix);
            cv::putText(frame, describe(estimate), {12, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1,
                        cv::LINE_AA);
            cv::putText(frame, cv::format("tracking %.1f ms (%.0f fps max)", ms, fps), {12, 48},
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, {207, 171, 124}, 1, cv::LINE_AA);
            cv::imshow("ct_ar face tracking", frame);
            const int key = cv::waitKey(1);
            if(key == 'q' || key == 27)
            {
                break;
            }
        }
    }

    return 0;
}
