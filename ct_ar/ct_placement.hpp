#pragma once

#include <data/boolean.hpp>
#include <data/integer.hpp>
#include <data/matrix4.hpp>
#include <data/real.hpp>
#include <data/series_set.hpp>

#include <service/filter.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>

namespace ct_ar
{

/**
 * @brief Computes the transform that places a CT scan (or any loaded image/mesh series) on the tracked anchor.
 *
 * Two modes, selected by `config.mode`:
 *
 * - **Tag (0)**: the anchor is the ArUco tag frame computed by `pose_from2d` (origin at the tag center, x/y in the tag
 *   plane, z along the tag normal). The data, in LPS patient coordinates, maps directly onto it (left -> x,
 *   posterior -> y, superior -> z), so a CT stands upright on a tag lying on a table:
 *
 *     transform = lift * offset * scale * translate(-center)
 *
 *   `center` is the center of the bounding box of every image and model series; `lift` (when `sit_on_tag` is
 *   enabled) moves the lowest point of the data onto the tag plane.
 *
 * - **Face (1)**: the anchor is the face frame computed by `ct_ar::face_tracker` (origin at the nose tip, x towards
 *   the subject's left, y up, z out of the face). The CT's nose tip is found automatically (most anterior skin point
 *   at mid-height of the head) and placed on the user's nose tip, with the patient axes aligned on the head axes:
 *
 *     transform = offset * scale * lps_to_face * translate(-ct_nose_tip)
 *
 * In both modes `offset` is a user rigid transform (typically edited with a transform_editor) applied around the
 * anchor origin, and `scale` a uniform scale factor (1 = real size, millimeters).
 *
 * @section XML XML configuration
 * @code{.xml}
   <service uid="..." type="ct_ar::ct_placement">
        <data series="${...}" />
        <data offset="${...}" />
        <data transform="${...}" />
        <config mode="${...}" scale="1.0" sit_on_tag="true" skin_threshold="-300" />
   </service>
   @endcode
 *
 * @subsection Input Input
 * - \b data.series [sight::data::series_set]: loaded data (DICOM, VTK, ...), auto-connected.
 * - \b data.offset [sight::data::matrix4]: rigid transform applied around the anchor origin, auto-connected.
 *
 * @subsection In-Out In-Out
 * - \b data.transform [sight::data::matrix4]: data-to-anchor transform, to be used by a transform adaptor.
 *
 * @subsection Properties Properties (all auto-connected)
 * - \b config.mode (int, default=0): 0 = tag, 1 = face.
 * - \b config.scale (double, default=1.0): uniform scale factor applied to the data.
 * - \b config.sit_on_tag (bool, default=true): tag mode only, lifts the data so that it stands on the tag.
 * - \b config.skin_threshold (double, default=-300): HU threshold separating air from skin, for the nose tip search.
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

    /// Nose tip of the first image series, in LPS mm, cached until the image changes.
    std::optional<glm::dvec3> nose_tip(const sight::data::series_set& _series);

    const void* m_nose_image {nullptr};
    std::uint64_t m_nose_timestamp {0};
    double m_nose_threshold {0.};
    std::optional<glm::dvec3> m_nose;

    sight::data::ptr<sight::data::series_set, sight::data::access::in> m_series {this, "data.series"};
    sight::data::ptr<sight::data::matrix4, sight::data::access::in> m_offset {this, "data.offset"};
    sight::data::ptr<sight::data::matrix4, sight::data::access::inout> m_transform {this, "data.transform"};

    sight::data::ptr<sight::data::integer, sight::data::access::in> m_mode {this, "config.mode", 0};
    sight::data::ptr<sight::data::real, sight::data::access::in> m_scale {this, "config.scale", 1.0};
    sight::data::ptr<sight::data::boolean, sight::data::access::in> m_sit_on_tag {this, "config.sit_on_tag", true};
    sight::data::ptr<sight::data::real, sight::data::access::in> m_skin_threshold {this, "config.skin_threshold",
                                                                                   -300.
    };
};

} // namespace ct_ar
