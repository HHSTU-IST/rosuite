#pragma once

#include <utility>

#include "kracker/filters/filter_base.hpp"
#include "kracker/filters/kalman_support.hpp"

namespace kracker::filters
{
  class ConstantGainFilter final : public FilterBase
  {
  public:
    /// Constructs ConstantGainFilter.
    explicit ConstantGainFilter(core::Matrix gain) : gain_(std::move(gain)) {}

    /// Predicts the next estimate.
    [[nodiscard]] core::Result<GaussianEstimate> predict(
        const GaussianEstimate &estimate,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const override
    {
      return detail::predict_linearized(estimate, model, context, std::move(control));
    }

    /// Corrects an estimate with a measurement.
    [[nodiscard]] core::Result<GaussianEstimate> correct(
        const GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const override
    {
      return detail::joseph_update(estimate, sensor, measurement, context, gain_);
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override
    {
      return "const_gain";
    }

  private:
    core::Matrix gain_;
  };

} // namespace kracker::filters
