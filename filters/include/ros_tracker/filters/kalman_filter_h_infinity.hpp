#pragma once

#include <optional>

#include <Eigen/Cholesky>

#include "ros_tracker/filters/filter_base.hpp"
#include "ros_tracker/filters/kalman_filter_support.hpp"

namespace ros_tracker::filters {

class KalmanFilterHInfinity final : public FilterBase {
 public:
  explicit KalmanFilterHInfinity(const core::Scalar gamma = 10.0)
      : gamma_(gamma) {}

  [[nodiscard]] core::Result<GaussianEstimate> predict(
      const GaussianEstimate& estimate,
      const models::DynamicSystemModel& model,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) const override {
    if (gamma_ <= 0.0) {
      return core::Status::invalid_argument(
          "KalmanFilterHInfinity requires gamma > 0.");
    }

    return detail::predict_linearized(estimate, model, context, std::move(control));
  }

  [[nodiscard]] core::Result<GaussianEstimate> correct(
      const GaussianEstimate& estimate,
      const models::SensorModel& sensor,
      const core::Measurement& measurement,
      const models::ModelContext& context = {}) const override {
    using core::Covariance;
    using core::Matrix;

    if (gamma_ <= 0.0) {
      return core::Status::invalid_argument(
          "KalmanFilterHInfinity requires gamma > 0.");
    }

    const core::Status estimate_status = validate_estimate(estimate);
    if (!estimate_status.ok()) {
      return estimate_status;
    }

    const core::Status sensor_status = detail::validate_measurement_support(sensor);
    if (!sensor_status.ok()) {
      return sensor_status;
    }

    const auto predicted = detail::linearized_measurement(
        estimate, sensor, measurement, context);
    if (!predicted.ok()) {
      return predicted.status();
    }

    const models::MeasurementRequest request {
        estimate.state,
        context,
        measurement.sensor_id,
    };

    const auto jacobian = sensor.measurement->state_jacobian(request);
    if (!jacobian.ok()) {
      return jacobian.status();
    }

    const auto measurement_noise = sensor.measurement_noise->covariance(request);
    if (!measurement_noise.ok()) {
      return measurement_noise.status();
    }

    Eigen::LDLT<Covariance> prior_ldlt(estimate.covariance);
    if (prior_ldlt.info() != Eigen::Success) {
      return core::Status::numerical_error(
          "Failed to factorize prior covariance for H-infinity correction.");
    }

    Eigen::LDLT<Covariance> noise_ldlt(measurement_noise.value());
    if (noise_ldlt.info() != Eigen::Success) {
      return core::Status::numerical_error(
          "Failed to factorize measurement noise for H-infinity correction.");
    }

    const Matrix state_identity =
        Matrix::Identity(estimate.dimension(), estimate.dimension());
    const Matrix measurement_identity = Matrix::Identity(
        measurement_noise.value().rows(),
        measurement_noise.value().cols());

    const Matrix prior_information = prior_ldlt.solve(state_identity);
    const Matrix noise_information = noise_ldlt.solve(measurement_identity);
    const Matrix robust_information =
        prior_information +
        jacobian.value().transpose() * noise_information * jacobian.value() -
        (1.0 / (gamma_ * gamma_)) * state_identity;

    Eigen::LDLT<Covariance> posterior_ldlt(
        core::symmetrize(robust_information));
    if (posterior_ldlt.info() != Eigen::Success) {
      return core::Status::numerical_error(
          "Failed to factorize H-infinity posterior information matrix. "
          "Try a larger gamma.");
    }

    GaussianEstimate updated = estimate;
    updated.covariance = core::symmetrize(
        posterior_ldlt.solve(state_identity));
    updated.state.value +=
        updated.covariance * jacobian.value().transpose() * noise_information *
        (measurement.value - predicted.value().measurement.value);
    updated.state.timestamp = measurement.timestamp;
    if (!measurement.frame_id.empty()) {
      updated.state.frame_id = measurement.frame_id;
    }

    return updated;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "kalman_hinf";
  }

 private:
  core::Scalar gamma_;
};

}  // namespace ros_tracker::filters
