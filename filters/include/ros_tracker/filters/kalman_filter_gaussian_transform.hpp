#pragma once

#include <optional>

#include <Eigen/Cholesky>

#include "ros_tracker/filters/estimate.hpp"
#include "ros_tracker/models/base.hpp"

namespace ros_tracker::filters::detail {

using core::Covariance;
using core::Matrix;
using core::Result;
using core::Status;
using core::Vector;

/// Validates a sigma-point set.
[[nodiscard]] inline Status validate_sigma_point_set(
    const SigmaPointSet& sigma_points,
    const core::Index dimension) {
  if (sigma_points.points.rows() != dimension) {
    return Status::dimension_mismatch(
        "Sigma point state dimension must match the estimate dimension.");
  }

  if (sigma_points.points.cols() <= 0) {
    return Status::invalid_argument(
        "Sigma point set must contain at least one sample.");
  }

  if (sigma_points.mean_weights.size() != sigma_points.points.cols() ||
      sigma_points.covariance_weights.size() != sigma_points.points.cols()) {
    return Status::dimension_mismatch(
        "Sigma point weights must match the number of sigma points.");
  }

  return Status::ok_status();
}

/// Computes a weighted mean from column samples.
[[nodiscard]] inline Result<Vector> weighted_mean_from_columns(
    const Matrix& samples,
    const Vector& weights) {
  if (samples.cols() != weights.size()) {
    return Status::dimension_mismatch(
        "Weighted mean expects one weight per sample column.");
  }

  Vector mean = Vector::Zero(samples.rows());
  for (core::Index i = 0; i < samples.cols(); ++i) {
    mean += weights[i] * samples.col(i);
  }

  return mean;
}

/// Computes a weighted covariance from column samples.
[[nodiscard]] inline Result<Covariance> weighted_covariance_from_columns(
    const Matrix& samples,
    const Vector& weights,
    const Vector& mean) {
  if (samples.cols() != weights.size()) {
    return Status::dimension_mismatch(
        "Weighted covariance expects one weight per sample column.");
  }

  if (samples.rows() != mean.size()) {
    return Status::dimension_mismatch(
        "Weighted covariance mean dimension must match the sample dimension.");
  }

  Covariance covariance = Covariance::Zero(samples.rows(), samples.rows());
  for (core::Index i = 0; i < samples.cols(); ++i) {
    const Vector delta = samples.col(i) - mean;
    covariance += weights[i] * (delta * delta.transpose());
  }

  return core::symmetrize(covariance);
}

/// Computes a weighted cross-covariance from column samples.
[[nodiscard]] inline Result<Matrix> weighted_cross_covariance_from_columns(
    const Matrix& left_samples,
    const Vector& left_mean,
    const Matrix& right_samples,
    const Vector& right_mean,
    const Vector& weights) {
  if (left_samples.cols() != right_samples.cols() ||
      left_samples.cols() != weights.size()) {
    return Status::dimension_mismatch(
        "Cross covariance expects aligned sample counts and weights.");
  }

  if (left_samples.rows() != left_mean.size() ||
      right_samples.rows() != right_mean.size()) {
    return Status::dimension_mismatch(
        "Cross covariance mean dimensions must match sample dimensions.");
  }

  Matrix covariance = Matrix::Zero(left_samples.rows(), right_samples.rows());
  for (core::Index i = 0; i < left_samples.cols(); ++i) {
    covariance += weights[i] *
                  (left_samples.col(i) - left_mean) *
                  (right_samples.col(i) - right_mean).transpose();
  }

  return covariance;
}

/// Propagates sigma points through the motion model.
[[nodiscard]] inline Result<GaussianEstimate> predict_from_sigma_points(
    const GaussianEstimate& estimate,
    const SigmaPointSet& sigma_points,
    const models::DynamicSystemModel& model,
    const models::ModelContext& context,
    std::optional<core::ControlInput> control = std::nullopt) {
  const Status estimate_status = validate_estimate(estimate);
  if (!estimate_status.ok()) {
    return estimate_status;
  }

  const Status sigma_status =
      validate_sigma_point_set(sigma_points, estimate.dimension());
  if (!sigma_status.ok()) {
    return sigma_status;
  }

  const Status model_status = model.validate();
  if (!model_status.ok()) {
    return model_status;
  }

  const models::MotionRequest noise_request {
      estimate.state,
      control,
      context,
  };
  const auto process_noise = model.process_noise->covariance(noise_request);
  if (!process_noise.ok()) {
    return process_noise.status();
  }

  Matrix propagated_samples =
      Matrix::Zero(estimate.dimension(), sigma_points.points.cols());
  for (core::Index i = 0; i < sigma_points.points.cols(); ++i) {
    models::MotionRequest request {
        estimate.state,
        control,
        context,
    };
    request.state.value = sigma_points.points.col(i);
    const auto propagated = model.motion->propagate(request);
    if (!propagated.ok()) {
      return propagated.status();
    }

    propagated_samples.col(i) = propagated.value().state.value;
  }

  const auto mean =
      weighted_mean_from_columns(propagated_samples, sigma_points.mean_weights);
  if (!mean.ok()) {
    return mean.status();
  }

  const auto covariance = weighted_covariance_from_columns(
      propagated_samples,
      sigma_points.covariance_weights,
      mean.value());
  if (!covariance.ok()) {
    return covariance.status();
  }

  GaussianEstimate predicted;
  predicted.state = estimate.state;
  predicted.state.value = mean.value();
  predicted.state.timestamp = context.timestamp;
  predicted.state.frame_id =
      context.frame_id.empty() ? estimate.state.frame_id : context.frame_id;
  predicted.covariance =
      core::symmetrize(covariance.value() + process_noise.value());
  return predicted;
}

/// Corrects an estimate from transformed sigma points.
[[nodiscard]] inline Result<GaussianEstimate> correct_from_sigma_points(
    const GaussianEstimate& estimate,
    const SigmaPointSet& sigma_points,
    const models::SensorModel& sensor,
    const core::Measurement& measurement,
    const models::ModelContext& context = {}) {
  const Status estimate_status = validate_estimate(estimate);
  if (!estimate_status.ok()) {
    return estimate_status;
  }

  const Status sigma_status =
      validate_sigma_point_set(sigma_points, estimate.dimension());
  if (!sigma_status.ok()) {
    return sigma_status;
  }

  const Status sensor_status = sensor.validate();
  if (!sensor_status.ok()) {
    return sensor_status;
  }

  Matrix state_samples = sigma_points.points;
  Matrix measurement_samples;
  bool initialized = false;
  for (core::Index i = 0; i < sigma_points.points.cols(); ++i) {
    models::MeasurementRequest request {
        estimate.state,
        context,
        measurement.sensor_id,
    };
    request.state.value = sigma_points.points.col(i);
    const auto predicted = sensor.measurement->measure(request);
    if (!predicted.ok()) {
      return predicted.status();
    }

    if (!initialized) {
      measurement_samples = Matrix::Zero(
          predicted.value().measurement.dimension(), sigma_points.points.cols());
      initialized = true;
    }

    measurement_samples.col(i) = predicted.value().measurement.value;
  }

  if (measurement.dimension() != measurement_samples.rows()) {
    return Status::dimension_mismatch(
        "Measurement dimension must match the predicted measurement dimension.");
  }

  const auto measurement_mean =
      weighted_mean_from_columns(measurement_samples, sigma_points.mean_weights);
  if (!measurement_mean.ok()) {
    return measurement_mean.status();
  }

  const auto innovation_covariance = weighted_covariance_from_columns(
      measurement_samples,
      sigma_points.covariance_weights,
      measurement_mean.value());
  if (!innovation_covariance.ok()) {
    return innovation_covariance.status();
  }

  const auto cross_covariance = weighted_cross_covariance_from_columns(
      state_samples,
      estimate.state.value,
      measurement_samples,
      measurement_mean.value(),
      sigma_points.covariance_weights);
  if (!cross_covariance.ok()) {
    return cross_covariance.status();
  }

  const models::MeasurementRequest noise_request {
      estimate.state,
      context,
      measurement.sensor_id,
  };
  const auto measurement_noise = sensor.measurement_noise->covariance(noise_request);
  if (!measurement_noise.ok()) {
    return measurement_noise.status();
  }

  if (measurement_noise.value().rows() != measurement.dimension() ||
      measurement_noise.value().cols() != measurement.dimension()) {
    return Status::dimension_mismatch(
        "Measurement noise covariance dimension must match the measurement dimension.");
  }

  const Covariance innovation =
      core::symmetrize(innovation_covariance.value() + measurement_noise.value());
  Eigen::LDLT<Covariance> ldlt(innovation);
  if (ldlt.info() != Eigen::Success) {
    return Status::numerical_error(
        "Failed to factorize sigma-point innovation covariance.");
  }

  const Matrix identity =
      Matrix::Identity(innovation.rows(), innovation.cols());
  const Matrix gain =
      Matrix(cross_covariance.value() * ldlt.solve(identity));

  GaussianEstimate updated = estimate;
  updated.state.value += gain * (measurement.value - measurement_mean.value());
  updated.state.timestamp = measurement.timestamp;
  if (!measurement.frame_id.empty()) {
    updated.state.frame_id = measurement.frame_id;
  }
  updated.covariance = core::symmetrize(
      estimate.covariance - gain * innovation * gain.transpose());
  return updated;
}

/// Builds a Gaussian estimate from an ensemble.
[[nodiscard]] inline Result<GaussianEstimate> estimate_from_ensemble(
    const Matrix& ensemble,
    const core::Scalar timestamp,
    const std::string& frame_id) {
  if (ensemble.rows() <= 0 || ensemble.cols() <= 1) {
    return Status::invalid_argument(
        "Ensemble estimate requires at least two ensemble members.");
  }

  Vector mean = ensemble.rowwise().mean();
  Covariance covariance = Covariance::Zero(ensemble.rows(), ensemble.rows());
  for (core::Index i = 0; i < ensemble.cols(); ++i) {
    const Vector delta = ensemble.col(i) - mean;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<core::Scalar>(ensemble.cols() - 1);

  GaussianEstimate estimate;
  estimate.state.value = mean;
  estimate.state.timestamp = timestamp;
  estimate.state.frame_id = frame_id;
  estimate.covariance = core::symmetrize(covariance);
  return estimate;
}

}  // namespace ros_tracker::filters::detail
