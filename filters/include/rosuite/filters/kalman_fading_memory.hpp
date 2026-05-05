#pragma once

#include "rosuite/filters/filter_base.hpp"
#include "rosuite/filters/kalman_support.hpp"

namespace rosuite::filters
{
  class KalmanFilterFadingMemory final : public FilterBase
  {
  public:
    /// Constructs KalmanFilterFadingMemory.
    explicit KalmanFilterFadingMemory(const core::Scalar fading_factor = 1.05)
        : fading_factor_(fading_factor) {}

    /// Predicts the next estimate.
    [[nodiscard]] core::Result<GaussianEstimate> predict(
        const GaussianEstimate &estimate,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const override
    {
      if (fading_factor_ < 1.0)
      {
        return core::Status::invalid_argument(
            "KalmanFilterFadingMemory requires fading_factor >= 1.");
      }

      const core::Status estimate_status = validate_estimate(estimate);
      if (!estimate_status.ok())
      {
        return estimate_status;
      }

      const core::Status model_status = detail::validate_motion_support(model);
      if (!model_status.ok())
      {
        return model_status;
      }

      const models::MotionRequest request{
          estimate.state,
          std::move(control),
          context,
      };

      const auto propagated = model.motion->propagate(request);
      if (!propagated.ok())
      {
        return propagated.status();
      }

      const auto jacobian = model.motion->state_jacobian(request);
      if (!jacobian.ok())
      {
        return jacobian.status();
      }

      const auto process_noise = model.process_noise->covariance(request);
      if (!process_noise.ok())
      {
        return process_noise.status();
      }

      if (process_noise.value().rows() != estimate.dimension() ||
          process_noise.value().cols() != estimate.dimension())
      {
        return core::Status::dimension_mismatch(
            "Process noise covariance dimension must match the state dimension.");
      }

      GaussianEstimate predicted;
      predicted.state = propagated.value().state;
      predicted.covariance = core::symmetrize(
          fading_factor_ * fading_factor_ *
              (jacobian.value() * estimate.covariance * jacobian.value().transpose()) +
          process_noise.value());
      return predicted;
    }

    /// Corrects an estimate with a measurement.
    [[nodiscard]] core::Result<GaussianEstimate> correct(
        const GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const override
    {
      const auto predicted = detail::linearized_measurement(
          estimate, sensor, measurement, context);
      if (!predicted.ok())
      {
        return predicted.status();
      }

      const auto gain = detail::kalman_gain_from_innovation(
          predicted.value().cross_covariance,
          predicted.value().covariance);
      if (!gain.ok())
      {
        return gain.status();
      }

      return detail::joseph_update(
          estimate, sensor, measurement, context, gain.value());
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override
    {
      return "kalman_fm";
    }

  private:
    core::Scalar fading_factor_;
  };

} // namespace rosuite::filters
