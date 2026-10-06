#pragma once

#include <data/boolean.hpp>
#include <data/matrix4.hpp>
#include <data/real.hpp>
#include <data/series_set.hpp>

#include <service/filter.hpp>

namespace ct_ar
{

/**
 * @brief Computes the transform that places a CT scan (or any loaded image/mesh series) on top of the ArUco tag.
 *
 * The marker frame computed by `sight::module::geometry::vision::pose_from2d` has its origin at the center of the tag,
 * the X/Y axes in the tag plane and Z along the tag normal. A CT scan on the other hand lives in patient (DICOM)
 * coordinates, so its center is usually hundreds of millimeters away from the origin. This service computes:
 *
 *   transform = lift * offset * scale * translate(-center)
 *
 * - `center` is the center of the axis-aligned bounding box of every image series and model series of the set,
 * - `scale` is a uniform scale factor (1.0 means real size, units are millimeters like the tag width),
 * - `offset` is a user-defined rigid transform (typically edited with a transform_editor) applied around the tag,
 * - `lift` translates the result along the tag normal so that the lowest point of the data sits on the tag plane,
 *   when `sit_on_tag` is enabled.
 *
 * @section XML XML configuration
 * @code{.xml}
   <service uid="..." type="ct_ar::ct_placement">
        <data series="${...}" />
        <data offset="${...}" />
        <data transform="${...}" />
        <config scale="1.0" sit_on_tag="true" />
   </service>
   @endcode
 *
 * @subsection Input Input
 * - \b data.series [sight::data::series_set]: loaded data (DICOM, VTK, ...), auto-connected.
 * - \b data.offset [sight::data::matrix4]: rigid transform applied around the tag origin, auto-connected.
 *
 * @subsection In-Out In-Out
 * - \b data.transform [sight::data::matrix4]: data-to-tag transform, to be used by a transform adaptor.
 *
 * @subsection Properties Properties
 * - \b config.scale (double, default=1.0): uniform scale factor applied to the data, auto-connected.
 * - \b config.sit_on_tag (bool, default=true): if true, the data is lifted so that it stands on the tag instead of
 *   being centered on it, auto-connected.
 */
class ct_placement final : public sight::service::filter
{
public:

    SIGHT_DECLARE_SERVICE(ct_placement, sight::service::filter);

    ct_placement() noexcept;
    ~ct_placement() noexcept final = default;

protected:

    /// Does nothing.
    void configuring(const config_t& _config) final;

    /// Computes the initial transform.
    void starting() final;

    /// Computes the transform.
    void updating() final;

    /// Does nothing.
    void stopping() final;

    /// Updates when the series set, the offset or the properties are modified.
    sight::service::connections_t auto_connections() const final;

private:

    sight::data::ptr<sight::data::series_set, sight::data::access::in> m_series {this, "data.series"};
    sight::data::ptr<sight::data::matrix4, sight::data::access::in> m_offset {this, "data.offset"};
    sight::data::ptr<sight::data::matrix4, sight::data::access::inout> m_transform {this, "data.transform"};

    sight::data::ptr<sight::data::real, sight::data::access::in> m_scale {this, "config.scale", 1.0};
    sight::data::ptr<sight::data::boolean, sight::data::access::in> m_sit_on_tag {this, "config.sit_on_tag", true};
};

} // namespace ct_ar
