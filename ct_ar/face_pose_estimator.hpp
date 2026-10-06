#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>

#include <string>
#include <vector>

namespace ct_ar
{

/**
 * @brief MediaPipe's canonical face: 468 metric vertices and the rigid landmarks used for pose estimation.
 *
 * Read from `geometry_pipeline_metadata_landmarks.binarypb`, shipped in MediaPipe's Face Landmarker bundle. Units are
 * millimeters; the frame has x towards the subject's left, y up and z out of the face.
 */
struct canonical_face
{
    std::vector<cv::Point3d> vertices;
    std::vector<int> basis_ids;        ///< Procrustes basis: landmarks that do not move with facial expressions.
    std::vector<double> basis_weights; ///< Their weights.

    static constexpr int NOSE_TIP = 1;

    /// Parses the protobuf metadata file. Throws std::runtime_error if the file is invalid.
    static canonical_face load(const std::string& _path);
};

/// Result of the face pose estimation for one frame.
struct face_estimate
{
    bool found {false};

    /// Face to camera transform (mm). The face frame has its origin at the nose tip, x towards the subject's left,
    /// y up and z out of the face. Camera frame: OpenCV convention (x right, y down, z forward).
    cv::Matx44d face_to_camera {cv::Matx44d::eye()};

    double reprojection_error {0.}; ///< Weighted RMS, pixels.
    float presence {0.F};           ///< Face presence probability returned by the landmark network.
    std::vector<cv::Point2f> landmarks;
};

/**
 * @brief Detects a face and estimates its 6-DoF pose from a single camera.
 *
 * 1. Detection: YuNet (cv::FaceDetectorYN) gives a face box and the eye positions. It only runs when the face is not
 *    tracked yet.
 * 2. Landmarks: MediaPipe's face mesh network (TFLite, run with cv::dnn) predicts 478 landmarks in a 256x256 crop
 *    centered on the face and rotated so that the eyes are horizontal. The crop of the next frame is computed from the
 *    current landmarks, so the face is tracked without re-running the detector; the network's face presence score
 *    tells when the track is lost.
 * 3. Pose: weighted PnP (Levenberg-Marquardt on the reprojection error) between the rigid landmarks and the metric
 *    canonical face, initialized with SQPnP or with the previous pose.
 *
 * Needs OpenCV >= 4.8 (TFLite importer and current YuNet model).
 */
class face_pose_estimator
{
public:

    struct config
    {
        std::string detector_model;    ///< face_detection_yunet_2023mar.onnx
        std::string landmark_model;    ///< face_landmarks_detector.tflite
        std::string geometry_metadata; ///< geometry_pipeline_metadata_landmarks.binarypb
        float detection_threshold {0.6F};
        float presence_threshold {0.5F};
        double max_reprojection_error {15.}; ///< pixels; above this, the estimate is rejected.
    };

    explicit face_pose_estimator(const config& _config);

    /// Processes an undistorted BGR frame. `_camera_matrix` are the pinhole intrinsics of that frame.
    face_estimate process(const cv::Mat& _bgr, const cv::Matx33d& _camera_matrix);

    /// Forgets the tracked face: the next frame runs the detector again.
    void reset();

    void set_presence_threshold(float _threshold)
    {
        m_config.presence_threshold = _threshold;
    }

    [[nodiscard]] const canonical_face& model() const
    {
        return m_model;
    }

    /// Weighted PnP, exposed for testing. Returns the pose of the canonical face frame and the weighted RMS error.
    static std::pair<cv::Matx44d, double> solve_pose(
        const std::vector<cv::Point3d>& _object,
        const std::vector<cv::Point2d>& _image,
        const std::vector<double>& _weights,
        const cv::Matx33d& _camera_matrix,
        const cv::Matx44d* _initial_guess
    );

private:

    /// Oriented square region of interest, in image pixels.
    struct roi
    {
        cv::Point2d center;
        double side {0.};
        double angle {0.}; ///< radians, rotation of the eyes axis
    };

    bool detect(const cv::Mat& _bgr, roi& _roi);
    std::vector<cv::Point2f> run_mesh(const cv::Mat& _bgr, const roi& _roi, float& _presence);
    static roi roi_from_landmarks(const std::vector<cv::Point2f>& _landmarks);

    config m_config;
    canonical_face m_model;
    cv::Ptr<cv::FaceDetectorYN> m_detector;
    cv::dnn::Net m_mesh;
    cv::Size m_detector_size;

    bool m_tracking {false};
    roi m_roi;
    bool m_has_pose {false};
    cv::Matx44d m_previous_pose {cv::Matx44d::eye()};
};

} // namespace ct_ar
