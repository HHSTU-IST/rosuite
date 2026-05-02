#pragma once

#include <memory>
#include <optional>

#include <Eigen/Cholesky>

#include "ros_tracker/filters/estimate.hpp"
#include "ros_tracker/models/base.hpp"

namespace ros_tracker::filters::detail {

using core::Covariance;
using core::Matrix;
using core::Result;
using core::Scalar;
using core::Status;
using core::Vector;

[[nodiscard]] inline Status validate_motion_support(
    const models::DynamicSystemModel& model) {
  const Status validation = model.validate();
  if (!validation.ok()) {
    return validation;
  }

  return Status::ok_status();
}

[[nodiscard]] inline Status validate_measurement_support(
    const models::SensorModel& sensor) {
  const Status validation = sensor.validate();
  if (!validation.ok()) {
    return validation;
  }

  return Status::ok_status();
}

[[nodiscard]] inline Result<GaussianEstimate> predict_linearized(
    const GaussianEstimate& estimate,
    const models::DynamicSystemModel& model,
    const models::ModelContext& context,
    std::optional<core::ControlInput> control = std::nullopt) {
  const Status estimate_status = validate_estimate(estimate);
  if (!estimate_status.ok()) {
    return estimate_status;
  }

  const Status model_status = validate_motion_support(model);
  if (!model_status.ok()) {
    return model_status;
  }

  const models::MotionRequest request {
      .state = estimate.state,
      .control = std::move(control),
      .context = context,
  };

  const auto propagated = model.motion->propagate(request);
  if (!propagated.ok()) {
    return propagated.status();
  }

  const auto jacobian = model.motion->state_jacobian(request);
  if (!jacobian.ok()) {
    return jacobian.status();
  }

  const auto process_noise = model.process_noise->covariance(request);
  if (!process_noise.ok()) {
    return process_noise.status();
  }

  if (process_noise.value().rows() != estimate.dimension() ||
      process_noise.value().cols() != estimate.dimension()) {
    return Status::dimension_mismatch(
        "Process noise covariance dimension must match the state dimension.");
  }

  GaussianEstimate predicted;
  predicted.state = propagated.value().state;
  predicted.covariance = core::symmetrize(
      jacobian.value() * estimate.covariance * jacobian.value().transpose() +
      process_noise.value());
  return predicted;
}

[[nodiscard]] inline Result<PredictedMeasurement> linearized_measurement(
    const GaussianEstimate& estimate,
    const models::SensorModel& sensor,
    const core::Measurement& measurement,
    const models::ModelContext& context) {
  const Status estimate_status = validate_estimate(estimate);
  if (!estimate_status.ok()) {
    return estimate_status;
  }

  const Status sensor_status = validate_measurement_support(sensor);
  if (!sensor_status.ok()) {
    return sensor_status;
  }

  const models::MeasurementRequest request {
      .state = estimate.state,
      .context = context,
      .sensor_id = measurement.sensor_id,
  };

  const auto predicted_measurement = sensor.measurement->measure(request);
  if (!predicted_measurement.ok()) {
    return predicted_measurement.status();
  }

  const auto jacobian = sensor.measurement->state_jacobian(request);
  if (!jacobian.ok()) {
    return jacobian.status();
  }

  const auto measurement_noise = sensor.measurement_noise->covariance(request);
  if (!measurement_noise.ok()) {
    return measurement_noise.status();
  }

  const core::Index measurement_dimension =
      predicted_measurement.value().measurement.dimension();
  if (measurement.dimension() != measurement_dimension) {
    return Status::dimension_mismatch(
        "Measurement dimension must match the predicted measurement dimension.");
  }

  if (measurement_noise.value().rows() != measurement_dimension ||
      measurement_noise.value().cols() != measurement_dimension) {
    return Status::dimension_mismatch(
        "Measurement noise covariance dimension must match the measurement dimension.");
  }

  PredictedMeasurement result;
  result.measurement = predicted_measurement.value().measurement;
  result.cross_covariance =
      estimate.covariance * jacobian.value().transpose();
  result.covariance = core::symmetrize(
      jacobian.value() * estimate.covariance * jacobian.value().transpose() +
      measurement_noise.value());
  return result;
}

[[nodiscard]] inline Result<GaussianEstimate> joseph_update(
    const GaussianEstimate& estimate,
    const models::SensorModel& sensor,
    const core::Measurement& measurement,
    const models::ModelContext& context,
    const Matrix& gain) {
  const auto predicted = linearized_measurement(estimate, sensor, measurement, context);
  if (!predicted.ok()) {
    return predicted.status();
  }

  const Status sensor_status = validate_measurement_support(sensor);
  if (!sensor_status.ok()) {
    return sensor_status;
  }

  const models::MeasurementRequest request {
      .state = estimate.state,
      .context = context,
      .sensor_id = measurement.sensor_id,
  };
  const auto jacobian = sensor.measurement->state_jacobian(request);
  if (!jacobian.ok()) {
    return jacobian.status();
  }

  const auto measurement_noise = sensor.measurement_noise->covariance(request);
  if (!measurement_noise.ok()) {
    return measurement_noise.status();
  }

  if (gain.rows() != estimate.dimension() ||
      gain.cols() != measurement.dimension()) {
    return Status::dimension_mismatch(
        "Filter gain dimension must be state_dim x measurement_dim.");
  }

  GaussianEstimate updated = estimate;
  updated.state.value += gain *
                         (measurement.value - predicted.value().measurement.value);
  updated.state.timestamp = measurement.timestamp;
  if (!measurement.frame_id.empty()) {
    updated.state.frame_id = measurement.frame_id;
  }

  const Matrix identity = Matrix::Identity(estimate.dimension(), estimate.dimension());
  const Matrix correction = identity - gain * jacobian.value();
  updated.covariance = core::symmetrize(
      correction * estimate.covariance * correction.transpose() +
      gain * measurement_noise.value() * gain.transpose());
  return updated;
}

[[nodiscard]] inline Result<Matrix> kalman_gain_from_innovation(
    const Matrix& cross_covariance,
    const Covariance& innovation_covariance) {
  Eigen::LDLT<Covariance> ldlt(innovation_covariance);
  if (ldlt.info() != Eigen::Success) {
    return Status::numerical_error(
        "Failed to factorize innovation covariance.");
  }

  const Matrix identity =
      Matrix::Identity(innovation_covariance.rows(), innovation_covariance.cols());
  return Matrix(cross_covariance * ldlt.solve(identity));
}

}  // namespace ros_tracker::filters::detail
