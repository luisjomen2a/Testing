#pragma once

#include "sight/ct_ar/config.hpp"

#include <core/runtime/plugin.hpp>

namespace ct_ar
{

/// This class is started when the module is loaded.
class SIGHT_CT_AR_CLASS_API plugin final : public sight::core::runtime::plugin
{
public:

    /// Destroys the plugin.
    SIGHT_CT_AR_API ~plugin() noexcept override;

    /// Starts the plugin, does nothing here.
    SIGHT_CT_AR_API void start() override;

    /// Stops the plugin, does nothing here.
    SIGHT_CT_AR_API void stop() noexcept override;
};

} // namespace ct_ar
