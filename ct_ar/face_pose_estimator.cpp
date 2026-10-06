#include "face_pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>

#if CV_VERSION_MAJOR < 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 8)
#error "Face tracking needs OpenCV >= 4.8 (TFLite importer of the dnn module, YuNet face detector)"
#endif

namespace ct_ar
{

namespace
{

constexpr int MESH_INPUT_SIZE  = 256;
constexpr int MESH_LANDMARKS   = 478;
constexpr int CANONICAL_POINTS = 468;
constexpr int RIGHT_EYE_OUTER  = 33;  // subject's right eye, outer corner
constexpr int LEFT_EYE_OUTER   = 263; // subject's left eye, outer corner

//------------------------------------------------------------------------------

/// Minimal protobuf wire-format reader, enough for geometry_pipeline_metadata_landmarks.binarypb.
class proto_reader
{
public:

    proto_reader(const std::uint8_t* _data, std::size_t _size) :
        m_data(_data),
        m_end(_data + _size)
    {
    }

    struct field
    {
        std::uint32_t number {0};
        std::uint32_t wire_type {0};
        std::uint64_t varint {0};
        const std::uint8_t* bytes {nullptr};
        std::size_t size {0};
    };

    bool next(field& _field)
    {
        if(m_data >= m_end)
        {
            return false;
        }

        const std::uint64_t key = read_varint();
        _field.number    = static_cast<std::uint32_t>(key >> 3U);
        _field.wire_type = static_cast<std::uint32_t>(key & 7U);
        switch(_field.wire_type)
        {
            case 0:
                _field.varint = read_varint();
                break;

            case 1:
                take(_field, 8);
                break;

            case 2:
                take(_field, static_cast<std::size_t>(read_varint()));
                break;

            case 5:
                take(_field, 4);
                break;

            default:
                throw std::runtime_error("Unsupported protobuf wire type");
        }

        return true;
    }

    static float to_float(const std::uint8_t* _bytes)
    {
        float value = 0.F;
        std::memcpy(&value, _bytes, sizeof(float)); // little-endian, like every platform Sight supports
        return value;
    }

private:

    std::uint64_t read_varint()
    {
        std::uint64_t result = 0;
        for(unsigned shift = 0 ; m_data < m_end && shift < 64 ; shift += 7)
        {
            const std::uint8_t byte = *m_data++;
            result |= static_cast<std::uint64_t>(byte & 0x7FU) << shift;
            if((byte & 0x80U) == 0)
            {
                return result;
            }
        }

        throw std::runtime_error("Truncated protobuf varint");
    }

    void take(field& _field, std::size_t _size)
    {
        if(static_cast<std::size_t>(m_end - m_data) < _size)
        {
            throw std::runtime_error("Truncated protobuf field");
        }

        _field.bytes = m_data;
        _field.size  = _size;
        m_data      += _size;
    }

    const std::uint8_t* m_data;
    const std::uint8_t* m_end;
};

//------------------------------------------------------------------------------

cv::Matx44d to_matrix(const cv::Vec3d& _rvec, const cv::Vec3d& _tvec)
{
    cv::Matx33d rotation;
    cv::Rodrigues(_rvec, rotation);
    cv::Matx44d result = cv::Matx44d::eye();
    for(int i = 0 ; i < 3 ; ++i)
    {
        for(int j = 0 ; j < 3 ; ++j)
        {
            result(i, j) = rotation(i, j);
        }

        result(i, 3) = _tvec[i];
    }

    return result;
}

//------------------------------------------------------------------------------

void from_matrix(const cv::Matx44d& _matrix, cv::Vec3d& _rvec, cv::Vec3d& _tvec)
{
    const cv::Matx33d rotation = _matrix.get_minor<3, 3>(0, 0);
    cv::Rodrigues(rotation, _rvec);
    _tvec = cv::Vec3d(_matrix(0, 3), _matrix(1, 3), _matrix(2, 3));
}

} // namespace

//------------------------------------------------------------------------------

canonical_face canonical_face::load(const std::string& _path)
{
    std::ifstream file(_path, std::ios::binary);
    if(!file)
    {
        throw std::runtime_error("Cannot open " + _path);
    }

    const std::vector<std::uint8_t> buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // GeometryPipelineMetadata { Mesh3d canonical_mesh = 1; repeated WeightedLandmarkRef procrustes_landmark_basis = 2; }
    // Mesh3d { vertex_type = 1; primitive_type = 2; repeated float vertex_buffer = 3; repeated uint32 index_buffer = 4; }
    // WeightedLandmarkRef { uint32 landmark_id = 1; float weight = 2; }
    // Vertex layout (VERTEX_PT): x, y, z, u, v, in centimeters.
    canonical_face face;
    std::vector<float> vertex_buffer;
    proto_reader reader(buffer.data(), buffer.size());
    proto_reader::field field;
    while(reader.next(field))
    {
        if(field.number == 1 && field.wire_type == 2)
        {
            proto_reader mesh(field.bytes, field.size);
            proto_reader::field mesh_field;
            while(mesh.next(mesh_field))
            {
                if(mesh_field.number != 3)
                {
                    continue;
                }

                if(mesh_field.wire_type == 5)
                {
                    vertex_buffer.push_back(proto_reader::to_float(mesh_field.bytes));
                }
                else if(mesh_field.wire_type == 2) // packed
                {
                    for(std::size_t i = 0 ; i + 4 <= mesh_field.size ; i += 4)
                    {
                        vertex_buffer.push_back(proto_reader::to_float(mesh_field.bytes + i));
                    }
                }
            }
        }
        else if(field.number == 2 && field.wire_type == 2)
        {
            proto_reader entry(field.bytes, field.size);
            proto_reader::field entry_field;
            int id        = 0;
            double weight = 1.;
            while(entry.next(entry_field))
            {
                if(entry_field.number == 1 && entry_field.wire_type == 0)
                {
                    id = static_cast<int>(entry_field.varint);
                }
                else if(entry_field.number == 2 && entry_field.wire_type == 5)
                {
                    weight = proto_reader::to_float(entry_field.bytes);
                }
            }

            face.basis_ids.push_back(id);
            face.basis_weights.push_back(weight);
        }
    }

    if(vertex_buffer.size() != static_cast<std::size_t>(CANONICAL_POINTS) * 5 || face.basis_ids.size() < 6)
    {
        throw std::runtime_error("Unexpected canonical face model in " + _path);
    }

    for(std::size_t i = 0 ; i < CANONICAL_POINTS ; ++i)
    {
        face.vertices.emplace_back(
            10. * vertex_buffer[i * 5],
            10. * vertex_buffer[i * 5 + 1],
            10. * vertex_buffer[i * 5 + 2]
        );
    }

    for(const int id : face.basis_ids)
    {
        if(id < 0 || id >= CANONICAL_POINTS)
        {
            throw std::runtime_error("Invalid landmark id in " + _path);
        }
    }

    return face;
}

//------------------------------------------------------------------------------

face_pose_estimator::face_pose_estimator(const config& _config) :
    m_config(_config),
    m_model(canonical_face::load(_config.geometry_metadata))
{
    m_detector = cv::FaceDetectorYN::create(_config.detector_model, "", cv::Size(320, 320), _config.detection_threshold);
#if CV_VERSION_MAJOR >= 5
    m_mesh = cv::dnn::readNetFromTFLite(_config.landmark_model, cv::dnn::ENGINE_CLASSIC);
#else
    m_mesh = cv::dnn::readNetFromTFLite(_config.landmark_model);
#endif
    if(m_mesh.empty())
    {
        throw std::runtime_error("Cannot load the face landmark model " + _config.landmark_model);
    }
}

//------------------------------------------------------------------------------

void face_pose_estimator::reset()
{
    m_tracking = false;
    m_has_pose = false;
}

//------------------------------------------------------------------------------

bool face_pose_estimator::detect(const cv::Mat& _bgr, roi& _roi)
{
    if(m_detector_size != _bgr.size())
    {
        m_detector->setInputSize(_bgr.size());
        m_detector_size = _bgr.size();
    }

    cv::Mat faces;
    m_detector->detect(_bgr, faces);
    if(faces.empty())
    {
        return false;
    }

    // Rows: x, y, w, h, right eye (x, y), left eye (x, y), nose, mouth right, mouth left, score. Keep the largest face.
    int best         = 0;
    float best_area  = 0.F;
    for(int i = 0 ; i < faces.rows ; ++i)
    {
        const float area = faces.at<float>(i, 2) * faces.at<float>(i, 3);
        if(area > best_area)
        {
            best_area = area;
            best      = i;
        }
    }

    const float* face = faces.ptr<float>(best);
    cv::Point2d eye_a(face[4], face[5]);
    cv::Point2d eye_b(face[6], face[7]);
    if(eye_a.x > eye_b.x)
    {
        std::swap(eye_a, eye_b);
    }

    const cv::Point2d eyes = eye_b - eye_a;
    _roi.center = {face[0] + face[2] / 2., face[1] + face[3] / 2.};
    _roi.side   = 1.6 * std::max(face[2], face[3]);
    _roi.angle  = std::atan2(eyes.y, eyes.x);
    return true;
}

//------------------------------------------------------------------------------

std::vector<cv::Point2f> face_pose_estimator::run_mesh(const cv::Mat& _bgr, const roi& _roi, float& _presence)
{
    // Affine transform: image -> 256x256 crop centered on the face, eyes horizontal.
    const double scale = MESH_INPUT_SIZE / _roi.side;
    const double c     = std::cos(_roi.angle);
    const double s     = std::sin(_roi.angle);
    cv::Matx23d affine(scale * c, scale * s, 0., -scale * s, scale * c, 0.);
    const double half = MESH_INPUT_SIZE / 2.;
    affine(0, 2) = half - (affine(0, 0) * _roi.center.x + affine(0, 1) * _roi.center.y);
    affine(1, 2) = half - (affine(1, 0) * _roi.center.x + affine(1, 1) * _roi.center.y);

    cv::Mat crop;
    cv::warpAffine(_bgr, crop, affine, cv::Size(MESH_INPUT_SIZE, MESH_INPUT_SIZE), cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT);

    const cv::Mat blob = cv::dnn::blobFromImage(crop, 1. / 255., crop.size(), cv::Scalar(), true, false);
    m_mesh.setInput(blob);
    std::vector<cv::Mat> outputs;
    m_mesh.forward(outputs, std::vector<cv::String> {"Identity", "Identity_1"});

    const float logit = outputs[1].ptr<float>()[0];
    _presence = 1.F / (1.F + std::exp(-logit));

    const cv::Mat landmarks = outputs[0].reshape(1, static_cast<int>(outputs[0].total() / 3));
    if(landmarks.rows < MESH_LANDMARKS)
    {
        return {};
    }

    cv::Matx23d inverse;
    cv::invertAffineTransform(affine, inverse);

    std::vector<cv::Point2f> points;
    points.reserve(MESH_LANDMARKS);
    for(int i = 0 ; i < MESH_LANDMARKS ; ++i)
    {
        const float* p  = landmarks.ptr<float>(i);
        const double px = p[0];
        const double py = p[1];
        points.emplace_back(
            static_cast<float>(inverse(0, 0) * px + inverse(0, 1) * py + inverse(0, 2)),
            static_cast<float>(inverse(1, 0) * px + inverse(1, 1) * py + inverse(1, 2))
        );
    }

    return points;
}

//------------------------------------------------------------------------------

face_pose_estimator::roi face_pose_estimator::roi_from_landmarks(const std::vector<cv::Point2f>& _landmarks)
{
    // Same rule as MediaPipe: square crop aligned with the eyes, 1.5 times the landmarks' extent.
    const cv::Point2d eyes = cv::Point2d(_landmarks[LEFT_EYE_OUTER]) - cv::Point2d(_landmarks[RIGHT_EYE_OUTER]);
    roi result;
    result.angle = std::atan2(eyes.y, eyes.x);

    cv::Point2d low(1e9, 1e9);
    cv::Point2d high(-1e9, -1e9);
    for(int i = 0 ; i < CANONICAL_POINTS ; ++i)
    {
        low.x  = std::min<double>(low.x, _landmarks[i].x);
        low.y  = std::min<double>(low.y, _landmarks[i].y);
        high.x = std::max<double>(high.x, _landmarks[i].x);
        high.y = std::max<double>(high.y, _landmarks[i].y);
    }

    result.center = (low + high) * 0.5;

    const double c = std::cos(-result.angle);
    const double s = std::sin(-result.angle);
    double min_u   = 1e9;
    double max_u   = -1e9;
    double min_v   = 1e9;
    double max_v   = -1e9;
    for(int i = 0 ; i < CANONICAL_POINTS ; ++i)
    {
        const double dx = _landmarks[i].x - result.center.x;
        const double dy = _landmarks[i].y - result.center.y;
        const double u  = c * dx - s * dy;
        const double v  = s * dx + c * dy;
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
    }

    result.side = 1.5 * std::max(max_u - min_u, max_v - min_v);
    return result;
}

//------------------------------------------------------------------------------

std::pair<cv::Matx44d, double> face_pose_estimator::solve_pose(
    const std::vector<cv::Point3d>& _object,
    const std::vector<cv::Point2d>& _image,
    const std::vector<double>& _weights,
    const cv::Matx33d& _camera_matrix,
    const cv::Matx44d* _initial_guess
)
{
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if(_initial_guess != nullptr)
    {
        from_matrix(*_initial_guess, rvec, tvec);
    }
    else if(!cv::solvePnP(_object, _image, _camera_matrix, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_SQPNP))
    {
        throw std::runtime_error("PnP failed");
    }

    // Levenberg-Marquardt on the weighted reprojection error (OpenCV's refiners do not take weights).
    const auto n = static_cast<int>(_object.size());
    double mean_weight = 0.;
    for(const double w : _weights)
    {
        mean_weight += w;
    }

    mean_weight /= static_cast<double>(_weights.size());
    cv::Mat sqrt_w(2 * n, 1, CV_64F);
    for(int i = 0 ; i < n ; ++i)
    {
        const double w = std::sqrt(_weights[static_cast<std::size_t>(i)] / mean_weight);
        sqrt_w.at<double>(2 * i)     = w;
        sqrt_w.at<double>(2 * i + 1) = w;
    }

    const auto evaluate = [&](const cv::Vec3d& _r, const cv::Vec3d& _t, cv::Mat& _residuals, cv::Mat& _jacobian)
                          {
                              std::vector<cv::Point2d> projected;
                              cv::Mat full_jacobian;
                              cv::projectPoints(_object, _r, _t, _camera_matrix, cv::noArray(), projected,
                                                full_jacobian);
                              _residuals.create(2 * n, 1, CV_64F);
                              for(int i = 0 ; i < n ; ++i)
                              {
                                  _residuals.at<double>(2 * i)     = projected[static_cast<std::size_t>(i)].x
                                                                     - _image[static_cast<std::size_t>(i)].x;
                                  _residuals.at<double>(2 * i + 1) = projected[static_cast<std::size_t>(i)].y
                                                                     - _image[static_cast<std::size_t>(i)].y;
                              }

                              _residuals = _residuals.mul(sqrt_w);
                              _jacobian  = full_jacobian.colRange(0, 6).clone();
                              for(int row = 0 ; row < 2 * n ; ++row)
                              {
                                  _jacobian.row(row) *= sqrt_w.at<double>(row);
                              }

                              return _residuals.dot(_residuals);
                          };

    cv::Mat residuals;
    cv::Mat jacobian;
    double cost    = evaluate(rvec, tvec, residuals, jacobian);
    double damping = 1e-3;
    for(int iteration = 0 ; iteration < 20 ; ++iteration)
    {
        const cv::Mat hessian = jacobian.t() * jacobian;
        cv::Mat damped        = hessian.clone();
        for(int i = 0 ; i < 6 ; ++i)
        {
            damped.at<double>(i, i) += damping * (hessian.at<double>(i, i) + 1e-9);
        }

        cv::Mat step;
        if(!cv::solve(damped, -jacobian.t() * residuals, step, cv::DECOMP_CHOLESKY))
        {
            break;
        }

        const cv::Vec3d r_new = rvec + cv::Vec3d(step.at<double>(0), step.at<double>(1), step.at<double>(2));
        const cv::Vec3d t_new = tvec + cv::Vec3d(step.at<double>(3), step.at<double>(4), step.at<double>(5));
        cv::Mat residuals_new;
        cv::Mat jacobian_new;
        const double cost_new = evaluate(r_new, t_new, residuals_new, jacobian_new);
        if(cost_new < cost)
        {
            rvec      = r_new;
            tvec      = t_new;
            residuals = residuals_new;
            jacobian  = jacobian_new;
            cost      = cost_new;
            damping  *= 0.3;
            if(cv::norm(step) < 1e-6)
            {
                break;
            }
        }
        else
        {
            damping *= 10.;
            if(damping > 1e8)
            {
                break;
            }
        }
    }

    return {to_matrix(rvec, tvec), std::sqrt(cost / n)};
}

//------------------------------------------------------------------------------

face_estimate face_pose_estimator::process(const cv::Mat& _bgr, const cv::Matx33d& _camera_matrix)
{
    face_estimate estimate;
    if(_bgr.empty())
    {
        return estimate;
    }

    // Detection, only when the face is not tracked yet. Then refine the crop once from the first landmarks.
    bool fresh = false;
    if(!m_tracking)
    {
        if(!detect(_bgr, m_roi))
        {
            m_has_pose = false;
            return estimate;
        }

        fresh = true;
    }

    std::vector<cv::Point2f> landmarks = run_mesh(_bgr, m_roi, estimate.presence);
    if(fresh && !landmarks.empty() && estimate.presence >= m_config.presence_threshold)
    {
        m_roi     = roi_from_landmarks(landmarks);
        landmarks = run_mesh(_bgr, m_roi, estimate.presence);
    }

    if(landmarks.empty() || estimate.presence < m_config.presence_threshold)
    {
        reset();
        return estimate;
    }

    // Track: next crop from the current landmarks.
    m_roi      = roi_from_landmarks(landmarks);
    m_tracking = true;
    estimate.landmarks = landmarks;

    std::vector<cv::Point3d> object;
    std::vector<cv::Point2d> image;
    object.reserve(m_model.basis_ids.size());
    image.reserve(m_model.basis_ids.size());
    for(const int id : m_model.basis_ids)
    {
        object.push_back(m_model.vertices[static_cast<std::size_t>(id)]);
        image.emplace_back(landmarks[static_cast<std::size_t>(id)]);
    }

    try
    {
        auto [pose, error] = solve_pose(object, image, m_model.basis_weights, _camera_matrix,
                                        m_has_pose ? &m_previous_pose : nullptr);
        if(m_has_pose && (error > m_config.max_reprojection_error || pose(2, 3) <= 0.))
        {
            std::tie(pose, error) = solve_pose(object, image, m_model.basis_weights, _camera_matrix, nullptr);
        }

        if(pose(2, 3) <= 0. || error > m_config.max_reprojection_error)
        {
            m_has_pose = false;
            return estimate;
        }

        m_previous_pose = pose;
        m_has_pose      = true;

        // Move the face frame origin to the nose tip.
        const cv::Point3d& nose = m_model.vertices[canonical_face::NOSE_TIP];
        cv::Matx44d nose_offset = cv::Matx44d::eye();
        nose_offset(0, 3) = nose.x;
        nose_offset(1, 3) = nose.y;
        nose_offset(2, 3) = nose.z;

        estimate.found              = true;
        estimate.face_to_camera     = pose * nose_offset;
        estimate.reprojection_error = error;
    }
    catch(const cv::Exception&)
    {
        m_has_pose = false;
    }
    catch(const std::runtime_error&)
    {
        m_has_pose = false;
    }

    return estimate;
}

} // namespace ct_ar
