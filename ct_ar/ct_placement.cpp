#include "ct_placement.hpp"

#include "nose_tip.hpp"

#include <core/spy_log.hpp>

#include <data/image_series.hpp>
#include <data/model_series.hpp>

#include <geometry/data/matrix4.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

namespace ct_ar
{

namespace
{

/// Axis-aligned bounding box accumulator.
struct bounds
{
    glm::dvec3 min {std::numeric_limits<double>::max()};
    glm::dvec3 max {std::numeric_limits<double>::lowest()};

    //------------------------------------------------------------------------------

    void expand(const glm::dvec3& _point)
    {
        min = glm::min(min, _point);
        max = glm::max(max, _point);
    }

    //------------------------------------------------------------------------------

    [[nodiscard]] bool valid() const
    {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }

    //------------------------------------------------------------------------------

    [[nodiscard]] std::array<glm::dvec3, 8> corners() const
    {
        std::array<glm::dvec3, 8> result {};
        for(std::size_t i = 0 ; i < result.size() ; ++i)
        {
            result[i] = {
                (i & 1U) != 0U ? max.x : min.x,
                (i & 2U) != 0U ? max.y : min.y,
                (i & 4U) != 0U ? max.z : min.z
            };
        }

        return result;
    }
};

//------------------------------------------------------------------------------

/// Adds the world-space corners of an image to the bounds.
void add_image(bounds& _bounds, const sight::data::image& _image)
{
    const auto& size = _image.size();
    if(size[0] == 0 || size[1] == 0)
    {
        return;
    }

    const voxel_to_world to_world(_image);
    // Voxel centers are at integer indices: the image extends half a voxel around them.
    const bounds local {
        .min = glm::dvec3(-0.5),
        .max = glm::dvec3(
            static_cast<double>(size[0]) - 0.5,
            static_cast<double>(size[1]) - 0.5,
            static_cast<double>(std::max<std::size_t>(size[2], 1)) - 0.5
        )
    };
    for(const auto& corner : local.corners())
    {
        _bounds.expand(to_world(corner));
    }
}

//------------------------------------------------------------------------------

/// Adds the bounding box of every reconstruction mesh of a model series to the bounds.
void add_model(bounds& _bounds, const sight::data::model_series& _model)
{
    for(const auto& reconstruction : _model.get_reconstruction_db())
    {
        if(!reconstruction)
        {
            continue;
        }

        const auto mesh = reconstruction->get_mesh();
        if(!mesh || mesh->num_points() == 0)
        {
            continue;
        }

        const auto& box = mesh->get_bounding_box();
        _bounds.expand({box.min[0], box.min[1], box.min[2]});
        _bounds.expand({box.max[0], box.max[1], box.max[2]});
    }
}

//------------------------------------------------------------------------------

std::optional<glm::dvec3> nose_tip_of(const sight::data::image& _image, double _threshold)
{
    const auto& size = _image.size();
    if(size[0] < 2 || size[1] < 2 || size[2] < 2 || _image.num_components() != 1)
    {
        return std::nullopt;
    }

    const auto lock   = _image.dump_lock();
    const void* data  = _image.buffer();
    const auto type   = _image.type();
    using sight::core::type;
    if(type == type::INT16)
    {
        return find_nose_tip(_image, static_cast<const std::int16_t*>(data), _threshold);
    }

    if(type == type::UINT16)
    {
        return find_nose_tip(_image, static_cast<const std::uint16_t*>(data), _threshold);
    }

    if(type == type::INT32)
    {
        return find_nose_tip(_image, static_cast<const std::int32_t*>(data), _threshold);
    }

    if(type == type::FLOAT32)
    {
        return find_nose_tip(_image, static_cast<const float*>(data), _threshold);
    }

    if(type == type::FLOAT64)
    {
        return find_nose_tip(_image, static_cast<const double*>(data), _threshold);
    }

    if(type == type::INT8)
    {
        return find_nose_tip(_image, static_cast<const std::int8_t*>(data), _threshold);
    }

    if(type == type::UINT8)
    {
        return find_nose_tip(_image, static_cast<const std::uint8_t*>(data), _threshold);
    }

    return std::nullopt;
}

/// LPS -> face frame: x = left, y = superior, z = anterior (-posterior).
const glm::dmat4 LPS_TO_FACE = glm::dmat4(
    glm::dvec4(1., 0., 0., 0.),  // column 0: L -> x
    glm::dvec4(0., 0., -1., 0.), // column 1: P -> -z
    glm::dvec4(0., 1., 0., 0.),  // column 2: S -> y
    glm::dvec4(0., 0., 0., 1.)
);

} // namespace

//-----------------------------------------------------------------------------

ct_placement::ct_placement() noexcept :
    filter(has_signals::signals())
{
}

//-----------------------------------------------------------------------------

void ct_placement::configuring(const config_t& /*_config*/)
{
}

//-----------------------------------------------------------------------------

void ct_placement::starting()
{
    this->update();
}

//-----------------------------------------------------------------------------

sight::service::connections_t ct_placement::auto_connections() const
{
    return {
        {m_series, sight::data::signals::MODIFIED, sight::service::slots::UPDATE},
        {m_series, sight::data::series_set::signals::ADDED_OBJECTS, sight::service::slots::UPDATE},
        {m_series, sight::data::series_set::signals::REMOVED_OBJECTS, sight::service::slots::UPDATE},
        {m_offset, sight::data::signals::MODIFIED, sight::service::slots::UPDATE},
        {m_mode, sight::data::signals::MODIFIED, sight::service::slots::UPDATE},
        {m_scale, sight::data::signals::MODIFIED, sight::service::slots::UPDATE},
        {m_sit_on_tag, sight::data::signals::MODIFIED, sight::service::slots::UPDATE},
        {m_skin_threshold, sight::data::signals::MODIFIED, sight::service::slots::UPDATE}
    };
}

//-----------------------------------------------------------------------------

std::optional<glm::dvec3> ct_placement::nose_tip(const sight::data::series_set& _series)
{
    std::shared_ptr<const sight::data::image_series> image;
    for(const auto& series : _series)
    {
        if(auto candidate = std::dynamic_pointer_cast<const sight::data::image_series>(series); candidate)
        {
            image = candidate;
            break;
        }
    }

    if(!image)
    {
        m_nose_image = nullptr;
        m_nose.reset();
        return std::nullopt;
    }

    const double threshold = *m_skin_threshold;
    if(image.get() != m_nose_image || image->last_modified() != m_nose_timestamp || threshold != m_nose_threshold)
    {
        m_nose           = nose_tip_of(*image, threshold);
        m_nose_image     = image.get();
        m_nose_timestamp = image->last_modified();
        m_nose_threshold = threshold;
        if(m_nose)
        {
            SIGHT_INFO("CT nose tip (LPS mm): " << m_nose->x << ", " << m_nose->y << ", " << m_nose->z);
        }
        else
        {
            SIGHT_WARN("Could not find the nose tip in the CT, the CT center is used instead");
        }
    }

    return m_nose;
}

//-----------------------------------------------------------------------------

void ct_placement::updating()
{
    bounds data_bounds;
    std::optional<glm::dvec3> nose;
    const bool face_mode = *m_mode == 1;
    {
        const auto series_set = m_series.const_lock();
        for(const auto& series : *series_set)
        {
            if(const auto image = std::dynamic_pointer_cast<const sight::data::image_series>(series); image)
            {
                add_image(data_bounds, *image);
            }
            else if(const auto model = std::dynamic_pointer_cast<const sight::data::model_series>(series); model)
            {
                add_model(data_bounds, *model);
            }
        }

        if(face_mode)
        {
            nose = this->nose_tip(*series_set);
        }
    }

    const glm::dvec3 center = data_bounds.valid() ? (data_bounds.min + data_bounds.max) * 0.5 : glm::dvec3(0.);
    const double scale      = std::max(static_cast<double>(*m_scale), 1e-6);
    const glm::dmat4 offset = sight::geometry::data::to_glm_mat(*m_offset.const_lock());
    const glm::dmat4 scaling = glm::scale(glm::dmat4(1.), glm::dvec3(scale));

    glm::dmat4 placement;
    if(face_mode)
    {
        // CT nose tip on the face nose tip (the face frame origin), patient axes on the head axes.
        const glm::dvec3 anchor = nose.value_or(center);
        placement = offset * scaling * LPS_TO_FACE * glm::translate(glm::dmat4(1.), -anchor);
    }
    else
    {
        placement = offset * scaling * glm::translate(glm::dmat4(1.), -center);

        if(data_bounds.valid() && *m_sit_on_tag)
        {
            // Move the lowest point of the transformed bounding box onto the tag plane.
            double lowest = std::numeric_limits<double>::max();
            for(const auto& corner : data_bounds.corners())
            {
                lowest = std::min(lowest, (placement * glm::dvec4(corner, 1.)).z);
            }

            placement = glm::translate(glm::dmat4(1.), glm::dvec3(0., 0., -lowest)) * placement;
        }
    }

    {
        const auto transform = m_transform.lock();
        sight::geometry::data::from_glm_mat(*transform, placement);
        transform->async_emit(sight::data::signals::MODIFIED);
    }

    this->async_emit(signals::SUCCEEDED);
}

//-----------------------------------------------------------------------------

void ct_placement::stopping()
{
}

} // namespace ct_ar
