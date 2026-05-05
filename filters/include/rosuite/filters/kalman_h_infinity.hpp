#pragma once

#include "rosuite/filters/filter_base.hpp"

namespace rosuite::filters
{
  class KalmanFilterHInfinity final : public FilterBase
  {
  public:
    /// Constructs KalmanFilterHInfinity.
    explicit KalmanFilterHInfinity(const core::Scalar gamma = 10.0);

    /// Predicts the next estimate.
    [[nodiscard]] core::Result<GaussianEstimate> predict(
        const GaussianEstimate &estimate,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const override;

    /// Corrects an estimate with a measurement.
    [[nodiscard]] core::Result<GaussianEstimate> correct(
        const GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const override;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override;

  private:
    core::Scalar gamma_;
  };

} // namespace rosuite::filters
