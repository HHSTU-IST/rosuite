#pragma once

#include <optional>
#include <string_view>

#include "rosuite/core/result.hpp"
#include "rosuite/core/types.hpp"
#include "rosuite/filters/estimate.hpp"
#include "rosuite/models/base.hpp"

namespace rosuite::filters
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

} // namespace rosuite::filters
