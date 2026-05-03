#pragma once

#include <cstdint>

#include "kracker/filters/filter_base.hpp"

namespace kracker::filters
{
  class KalmanFilterEnsemble final : public FilterBase
  {
  public:
    /// Constructs KalmanFilterEnsemble.
    explicit KalmanFilterEnsemble(
        const core::Index ensemble_size = 64,
        const std::uint64_t seed = 0U);

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
    core::Index ensemble_size_;
    std::uint64_t seed_;
  };

} // namespace kracker::filters
