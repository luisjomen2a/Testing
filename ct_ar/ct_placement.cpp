#include "ct_placement.hpp"

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

/// Adds the world-space corners of an image to the bounds, taking origin, spacing and orientation into account.
void add_image(bounds& _bounds, const sight::data::image& _image)
{
    const auto& size = _image.size();
    if(size[0] == 0 || size[1] == 0)
    {
        return;
    }

    const auto& spacing     = _image.spacing();
    const auto& origin      = _image.origin();
    const auto& orientation = _image.orientation(); // Row-major 3x3 matrix

    const glm::dvec3 extent {
        static_cast<double>(size[0]) * spacing[0],
        static_cast<double>(size[1]) * spacing[1],
        static_cast<double>(std::max<std::size_t>(size[2], 1)) * spacing[2]
    };

    const bounds local {.min = glm::dvec3(0.), .max = extent};
    for(const auto& corner : local.corners())
    {
        glm::dvec3 world {origin[0], origin[1], origin[2]};
        for(glm::length_t row = 0 ; row < 3 ; ++row)
        {
            for(glm::length_t col = 0 ; col < 3 ; ++col)
            {
                world[row] += orientation[static_cast<std::size_t>(row * 3 + col)] * corner[col];
            }
        }

        _bounds.expand(world);
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
        {m_scale, sight::data::signals::MODIFIED, sight::service::slots::UPDATE},
        {m_sit_on_tag, sight::data::signals::MODIFIED, sight::service::slots::UPDATE}
    };
}

//-----------------------------------------------------------------------------

void ct_placement::updating()
{
    bounds data_bounds;
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
    }

    const glm::dvec3 center = data_bounds.valid() ? (data_bounds.min + data_bounds.max) * 0.5 : glm::dvec3(0.);
    const double scale      = std::max(m_scale.const_lock()->value(), 1e-6);

    const glm::dmat4 offset = sight::geometry::data::to_glm_mat(*m_offset.const_lock());

    glm::dmat4 placement = offset
                           * glm::scale(glm::dmat4(1.), glm::dvec3(scale))
                           * glm::translate(glm::dmat4(1.), -center);

    if(data_bounds.valid() && m_sit_on_tag.const_lock()->value())
    {
        // Find the lowest point of the transformed bounding box along the tag normal (Z), and move it onto the tag.
        double lowest = std::numeric_limits<double>::max();
        for(const auto& corner : data_bounds.corners())
        {
            lowest = std::min(lowest, (placement * glm::dvec4(corner, 1.)).z);
        }

        placement = glm::translate(glm::dmat4(1.), glm::dvec3(0., 0., -lowest)) * placement;
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
