#pragma once

#include <memory>
#include <optional>

#include <Eigen/Cholesky>

#include "ros_tracker/filters/estimate.hpp"
#include "ros_tracker/models/base.hpp"

namespace ros_tracker::filters::detail
{
  using core::Covariance;
  using core::Matrix;
  using core::Result;
  using core::Scalar;
  using core::Status;
  using core::Vector;

  /// Validates motion-model support for the requested operation.
  [[nodiscard]] Status validate_motion_support(
      const models::DynamicSystemModel &model);

  /// Validates measurement-model support for the requested operation.
  [[nodiscard]] Status validate_measurement_support(
      const models::SensorModel &sensor);

  /// Predicts an estimate with a linearized system model.
  [[nodiscard]] Result<GaussianEstimate> predict_linearized(
      const GaussianEstimate &estimate,
      const models::DynamicSystemModel &model,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control = std::nullopt);

  /// Builds a linearized predicted measurement.
  [[nodiscard]] Result<PredictedMeasurement> linearized_measurement(
      const GaussianEstimate &estimate,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context);

  /// Applies the Joseph-form covariance update.
  [[nodiscard]] Result<GaussianEstimate> joseph_update(
      const GaussianEstimate &estimate,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context,
      const Matrix &gain);

  /// Computes the Kalman gain from an innovation covariance.
  [[nodiscard]] Result<Matrix> kalman_gain_from_innovation(
      const Matrix &cross_covariance,
      const Covariance &innovation_covariance);

} // namespace ros_tracker::filters::detail
