#pragma once

#include <optional>
#include <string_view>

#include "kracker/core/result.hpp"
#include "kracker/core/types.hpp"
#include "kracker/filters/estimate.hpp"
#include "kracker/models/base.hpp"

namespace kracker::filters
{
    class FilterBase
    {
    public:
        /// Destroys FilterBase.
        virtual ~FilterBase() = default;

        /// Predicts the next estimate.
        [[nodiscard]] virtual core::Result<GaussianEstimate> predict(
            const GaussianEstimate &estimate,
            const models::DynamicSystemModel &model,
            const models::ModelContext &context,
            std::optional<core::ControlInput> control = std::nullopt) const = 0;

        /// Corrects an estimate with a measurement.
        [[nodiscard]] virtual core::Result<GaussianEstimate> correct(
            const GaussianEstimate &estimate,
            const models::SensorModel &sensor,
            const core::Measurement &measurement,
            const models::ModelContext &context = {}) const = 0;

        /// Returns the component name.
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    };

} // namespace kracker::filters
