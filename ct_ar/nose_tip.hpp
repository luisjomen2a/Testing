#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

/**
 * Anatomical helpers working on any image type exposing size(), spacing(), origin() and orientation() like
 * sight::data::image (voxel centers at integer indices, row-major orientation matrix).
 */
namespace ct_ar
{

/// Voxel index (continuous) -> LPS world coordinates, taking origin, spacing and orientation into account.
struct voxel_to_world
{
    template<typename Image>
    explicit voxel_to_world(const Image& _image)
    {
        const auto& spacing     = _image.spacing();
        const auto& origin      = _image.origin();
        const auto& orientation = _image.orientation(); // Row-major 3x3 matrix
        offset = {origin[0], origin[1], origin[2]};
        for(glm::length_t row = 0 ; row < 3 ; ++row)
        {
            for(glm::length_t col = 0 ; col < 3 ; ++col)
            {
                // glm matrices are column-major: axes[col][row]
                axes[col][row] = orientation[static_cast<std::size_t>(row * 3 + col)] * spacing[
                    static_cast<std::size_t>(col)];
            }
        }
    }

    [[nodiscard]] glm::dvec3 operator()(const glm::dvec3& _index) const
    {
        return offset + axes * _index;
    }

    glm::dvec3 offset {0.};
    glm::dmat3 axes {1.};
};

//------------------------------------------------------------------------------

/// Most anterior skin point of a head CT (the nose tip when the patient lies supine), in LPS world coordinates.
template<typename Image, typename T>
std::optional<glm::dvec3> find_nose_tip(const Image& _image, const T* _voxels, double _threshold)
{
    const auto& size = _image.size();
    const voxel_to_world to_world(_image);

    // Subsample large volumes: ~160 samples per axis are plenty to find the nose, then refine at full resolution.
    std::array<std::size_t, 3> step {};
    for(std::size_t axis = 0 ; axis < 3 ; ++axis)
    {
        step[axis] = std::max<std::size_t>(1, size[axis] / 160);
    }

    const auto value = [&](std::size_t _x, std::size_t _y, std::size_t _z)
                       {
                           return static_cast<double>(_voxels[_x + size[0] * (_y + size[1] * _z)]);
                       };

    // Pass 1: superior/inferior extent of the body (world z = superior), to restrict the search to mid-height and
    // avoid the shoulders, the chin of a tilted head or the top of the skull.
    std::vector<double> heights;
    for(std::size_t z = 0 ; z < size[2] ; z += step[2])
    {
        for(std::size_t y = 0 ; y < size[1] ; y += step[1])
        {
            for(std::size_t x = 0 ; x < size[0] ; x += step[0])
            {
                if(value(x, y, z) > _threshold)
                {
                    heights.push_back(to_world({double(x), double(y), double(z)}).z);
                }
            }
        }
    }

    if(heights.size() < 100)
    {
        return std::nullopt;
    }

    const auto percentile = [&heights](double _p)
                            {
                                auto nth = heights.begin()
                                           + static_cast<std::ptrdiff_t>(_p * static_cast<double>(heights.size()
                                                                                                  - 1));
                                std::nth_element(heights.begin(), nth, heights.end());
                                return *nth;
                            };
    const double low  = percentile(0.15);
    const double high = percentile(0.75);

    // Pass 2: most anterior (smallest world y) skin voxel in that range, at full resolution.
    double most_anterior = std::numeric_limits<double>::max();
    for(std::size_t z = 0 ; z < size[2] ; ++z)
    {
        for(std::size_t y = 0 ; y < size[1] ; ++y)
        {
            for(std::size_t x = 0 ; x < size[0] ; ++x)
            {
                if(value(x, y, z) <= _threshold)
                {
                    continue;
                }

                const glm::dvec3 p = to_world({double(x), double(y), double(z)});
                if(p.z >= low && p.z <= high && p.y < most_anterior)
                {
                    most_anterior = p.y;
                }
            }
        }
    }

    // Pass 3: centroid of the skin voxels within 2 mm of that point, for a stable estimate.
    glm::dvec3 sum(0.);
    std::size_t count = 0;
    for(std::size_t z = 0 ; z < size[2] ; ++z)
    {
        for(std::size_t y = 0 ; y < size[1] ; ++y)
        {
            for(std::size_t x = 0 ; x < size[0] ; ++x)
            {
                if(value(x, y, z) <= _threshold)
                {
                    continue;
                }

                const glm::dvec3 p = to_world({double(x), double(y), double(z)});
                if(p.z >= low && p.z <= high && p.y <= most_anterior + 2.0)
                {
                    sum += p;
                    ++count;
                }
            }
        }
    }

    if(count == 0)
    {
        return std::nullopt;
    }

    return sum / static_cast<double>(count);
}

} // namespace ct_ar
