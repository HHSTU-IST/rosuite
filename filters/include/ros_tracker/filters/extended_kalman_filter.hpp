#pragma once

#include "ros_tracker/filters/filter_base.hpp"
#include "ros_tracker/filters/kalman_support.hpp"

namespace ros_tracker::filters {

class ExtendedKalmanFilter final : public FilterBase {
 public:
  [[nodiscard]] core::Result<GaussianEstimate> predict(
      const GaussianEstimate& estimate,
      const models::DynamicSystemModel& model,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) const override {
    return detail::predict_linearized(estimate, model, context, std::move(control));
  }

  [[nodiscard]] core::Result<GaussianEstimate> correct(
      const GaussianEstimate& estimate,
      const models::SensorModel& sensor,
      const core::Measurement& measurement,
      const models::ModelContext& context = {}) const override {
    const auto predicted = detail::linearized_measurement(
        estimate, sensor, measurement, context);
    if (!predicted.ok()) {
      return predicted.status();
    }

    const auto gain = detail::kalman_gain_from_innovation(
        predicted.value().cross_covariance,
        predicted.value().covariance);
    if (!gain.ok()) {
      return gain.status();
    }

    return detail::joseph_update(
        estimate, sensor, measurement, context, gain.value());
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "kalman_ekf";
  }
};

}  // namespace ros_tracker::filters
