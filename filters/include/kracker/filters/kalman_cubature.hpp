#pragma once

#include "kracker/filters/filter_base.hpp"
#include "kracker/filters/kalman_gaussian_transform.hpp"
#include "kracker/filters/sigma_points.hpp"

namespace kracker::filters
{
  class KalmanFilterCubature final : public FilterBase
  {
  public:
    /// Predicts the next estimate.
    [[nodiscard]] core::Result<GaussianEstimate> predict(
        const GaussianEstimate &estimate,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const override
    {
      const auto sigma_points = generator_.generate(estimate);
      if (!sigma_points.ok())
      {
        return sigma_points.status();
      }

      return detail::predict_from_sigma_points(
          estimate, sigma_points.value(), model, context, std::move(control));
    }

    /// Corrects an estimate with a measurement.
    [[nodiscard]] core::Result<GaussianEstimate> correct(
        const GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const override
    {
      const auto sigma_points = generator_.generate(estimate);
      if (!sigma_points.ok())
      {
        return sigma_points.status();
      }

      return detail::correct_from_sigma_points(
          estimate, sigma_points.value(), sensor, measurement, context);
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override
    {
      return "kalman_ckf";
    }

  private:
    CubaturePointGenerator generator_;
  };

} // namespace kracker::filters
