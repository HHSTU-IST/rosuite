#include "kracker/filters/kalman_ensemble.hpp"

#include <optional>

#include <Eigen/Cholesky>

#include "kracker/core/math/random.hpp"
#include "kracker/filters/kalman_gaussian_transform.hpp"

namespace kracker::filters
{
  namespace
  {
    [[nodiscard]] core::Status validate_ensemble_size(
        const core::Index ensemble_size)
    {
      if (ensemble_size < 2)
      {
        return core::Status::invalid_argument(
            "KalmanFilterEnsemble requires at least two ensemble members.");
      }

      return core::Status::ok_status();
    }
  } // namespace

  KalmanFilterEnsemble::KalmanFilterEnsemble(
      const core::Index ensemble_size,
      const std::uint64_t seed)
      : ensemble_size_(ensemble_size),
        seed_(seed) {}

  core::Result<GaussianEstimate> KalmanFilterEnsemble::predict(
      const GaussianEstimate &estimate,
      const models::DynamicSystemModel &model,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control) const
  {
    using core::Matrix;
    using core::Vector;

    const core::Status ensemble_status = validate_ensemble_size(ensemble_size_);
    if (!ensemble_status.ok())
    {
      return ensemble_status;
    }

    const core::Status estimate_status = validate_estimate(estimate);
    if (!estimate_status.ok())
    {
      return estimate_status;
    }

    const core::Status model_status = model.validate();
    if (!model_status.ok())
    {
      return model_status;
    }

    core::stats::RandomEngine rng(seed_);
    Matrix ensemble = Matrix::Zero(estimate.dimension(), ensemble_size_);
    for (core::Index i = 0; i < ensemble_size_; ++i)
    {
      const auto sample =
          rng.sample_multivariate_normal(estimate.state.value, estimate.covariance);
      if (!sample.ok())
      {
        return sample.status();
      }
      ensemble.col(i) = sample.value();
    }

    const models::MotionRequest noise_request{
        estimate.state,
        control,
        context,
    };
    const auto process_noise = model.process_noise->covariance(noise_request);
    if (!process_noise.ok())
    {
      return process_noise.status();
    }

    for (core::Index i = 0; i < ensemble_size_; ++i)
    {
      models::MotionRequest request{
          estimate.state,
          control,
          context,
      };
      request.state.value = ensemble.col(i);

      const auto propagated = model.motion->propagate(request);
      if (!propagated.ok())
      {
        return propagated.status();
      }

      const auto noise =
          rng.sample_multivariate_normal(
              Vector::Zero(estimate.dimension()),
              process_noise.value());
      if (!noise.ok())
      {
        return noise.status();
      }

      ensemble.col(i) = propagated.value().state.value + noise.value();
    }

    return detail::estimate_from_ensemble(
        ensemble,
        context.timestamp,
        context.frame_id.empty() ? estimate.state.frame_id : context.frame_id);
  }

  core::Result<GaussianEstimate> KalmanFilterEnsemble::correct(
      const GaussianEstimate &estimate,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context) const
  {
    using core::Covariance;
    using core::Matrix;
    using core::Vector;

    const core::Status ensemble_status = validate_ensemble_size(ensemble_size_);
    if (!ensemble_status.ok())
    {
      return ensemble_status;
    }

    const core::Status estimate_status = validate_estimate(estimate);
    if (!estimate_status.ok())
    {
      return estimate_status;
    }

    const core::Status sensor_status = sensor.validate();
    if (!sensor_status.ok())
    {
      return sensor_status;
    }

    core::stats::RandomEngine rng(seed_);
    Matrix state_ensemble = Matrix::Zero(estimate.dimension(), ensemble_size_);
    for (core::Index i = 0; i < ensemble_size_; ++i)
    {
      const auto sample =
          rng.sample_multivariate_normal(estimate.state.value, estimate.covariance);
      if (!sample.ok())
      {
        return sample.status();
      }
      state_ensemble.col(i) = sample.value();
    }

    Matrix measurement_ensemble;
    bool initialized = false;
    for (core::Index i = 0; i < ensemble_size_; ++i)
    {
      models::MeasurementRequest request{
          estimate.state,
          context,
          measurement.sensor_id,
      };
      request.state.value = state_ensemble.col(i);

      const auto predicted = sensor.measurement->measure(request);
      if (!predicted.ok())
      {
        return predicted.status();
      }

      if (!initialized)
      {
        measurement_ensemble = Matrix::Zero(
            predicted.value().measurement.dimension(),
            ensemble_size_);
        initialized = true;
      }

      measurement_ensemble.col(i) = predicted.value().measurement.value;
    }

    if (measurement.dimension() != measurement_ensemble.rows())
    {
      return core::Status::dimension_mismatch(
          "Measurement dimension must match the predicted measurement dimension.");
    }

    const models::MeasurementRequest noise_request{
        estimate.state,
        context,
        measurement.sensor_id,
    };
    const auto measurement_noise = sensor.measurement_noise->covariance(noise_request);
    if (!measurement_noise.ok())
    {
      return measurement_noise.status();
    }

    Vector state_mean = state_ensemble.rowwise().mean();
    Vector measurement_mean = measurement_ensemble.rowwise().mean();

    Matrix state_anomalies = state_ensemble.colwise() - state_mean;
    Matrix measurement_anomalies = measurement_ensemble.colwise() - measurement_mean;

    const Matrix cross_covariance =
        (state_anomalies * measurement_anomalies.transpose()) /
        static_cast<core::Scalar>(ensemble_size_ - 1);
    const Covariance innovation_covariance = core::symmetrize(
        (measurement_anomalies * measurement_anomalies.transpose()) /
            static_cast<core::Scalar>(ensemble_size_ - 1) +
        measurement_noise.value());

    Eigen::LDLT<Covariance> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success)
    {
      return core::Status::numerical_error(
          "Failed to factorize EnKF innovation covariance.");
    }

    const Matrix gain = Matrix(
        cross_covariance *
        ldlt.solve(
            Matrix::Identity(
                innovation_covariance.rows(),
                innovation_covariance.cols())));

    for (core::Index i = 0; i < ensemble_size_; ++i)
    {
      const auto perturbation = rng.sample_multivariate_normal(
          Vector::Zero(measurement.dimension()),
          measurement_noise.value());
      if (!perturbation.ok())
      {
        return perturbation.status();
      }

      state_ensemble.col(i) +=
          gain *
          (measurement.value + perturbation.value() - measurement_ensemble.col(i));
    }

    return detail::estimate_from_ensemble(
        state_ensemble,
        measurement.timestamp,
        measurement.frame_id.empty() ? estimate.state.frame_id : measurement.frame_id);
  }

  std::string_view KalmanFilterEnsemble::name() const noexcept
  {
    return "kalman_enkf";
  }

} // namespace kracker::filters
