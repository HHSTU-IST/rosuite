#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "ros_tracker/core/stats/random.hpp"
#include "ros_tracker/core/stats/statistics.hpp"
#include "ros_tracker/core/utils/resampling.hpp"
#include "ros_tracker/filters/filter_base.hpp"

namespace ros_tracker::filters {

class ParticleFilter final : public FilterBase {
 public:
  explicit ParticleFilter(
      const core::Index particle_count = 256,
      const std::uint64_t seed = 0U,
      const core::Scalar resampling_offset = 0.5)
      : particle_count_(particle_count),
        seed_(seed),
        resampling_offset_(resampling_offset) {}

  [[nodiscard]] core::Result<GaussianEstimate> predict(
      const GaussianEstimate& estimate,
      const models::DynamicSystemModel& model,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) const override {
    const auto prior_particles = sample_prior_particles(estimate);
    if (!prior_particles.ok()) {
      return prior_particles.status();
    }

    const auto propagated = propagate_particles(
        prior_particles.value(), model, context, std::move(control));
    if (!propagated.ok()) {
      return propagated.status();
    }

    return particle_set_to_estimate(
        propagated.value(),
        context.timestamp,
        context.frame_id.empty() ? estimate.state.frame_id : context.frame_id);
  }

  [[nodiscard]] core::Result<GaussianEstimate> correct(
      const GaussianEstimate& estimate,
      const models::SensorModel& sensor,
      const core::Measurement& measurement,
      const models::ModelContext& context = {}) const override {
    const auto prior_particles = sample_prior_particles(estimate);
    if (!prior_particles.ok()) {
      return prior_particles.status();
    }

    const auto weighted = weight_particles(
        prior_particles.value(), sensor, measurement, context);
    if (!weighted.ok()) {
      return weighted.status();
    }

    const auto resampled = resample_particles(weighted.value());
    if (!resampled.ok()) {
      return resampled.status();
    }

    return particle_set_to_estimate(
        resampled.value(),
        measurement.timestamp,
        measurement.frame_id.empty() ? estimate.state.frame_id : measurement.frame_id);
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "particle";
  }

 private:
  [[nodiscard]] core::Result<ParticleSet> sample_prior_particles(
      const GaussianEstimate& estimate) const {
    const core::Status estimate_status = validate_estimate(estimate);
    if (!estimate_status.ok()) {
      return estimate_status;
    }

    if (particle_count_ <= 0) {
      return core::Status::invalid_argument(
          "ParticleFilter requires at least one particle.");
    }

    core::stats::RandomEngine rng(seed_);
    ParticleSet particle_set;
    particle_set.particles = core::Matrix::Zero(
        estimate.dimension(), particle_count_);
    particle_set.weights.assign(
        static_cast<std::size_t>(particle_count_),
        1.0 / static_cast<core::Scalar>(particle_count_));

    for (core::Index i = 0; i < particle_count_; ++i) {
      const auto sample = rng.sample_multivariate_normal(
          estimate.state.value, estimate.covariance);
      if (!sample.ok()) {
        return sample.status();
      }

      particle_set.particles.col(i) = sample.value();
    }

    return particle_set;
  }

  [[nodiscard]] core::Result<ParticleSet> propagate_particles(
      const ParticleSet& particles,
      const models::DynamicSystemModel& model,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) const {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok()) {
      return particle_status;
    }

    const core::Status model_status = model.validate();
    if (!model_status.ok()) {
      return model_status;
    }

    core::stats::RandomEngine rng(seed_);
    models::MotionRequest noise_request {
        .state = {},
        .control = control,
        .context = context,
    };
    noise_request.state.value = particles.particles.col(0);
    const auto process_noise = model.process_noise->covariance(noise_request);
    if (!process_noise.ok()) {
      return process_noise.status();
    }

    ParticleSet propagated = particles;
    for (core::Index i = 0; i < particles.size(); ++i) {
      models::MotionRequest request {
          .state = {},
          .control = control,
          .context = context,
      };
      request.state.value = particles.particles.col(i);
      const auto prediction = model.motion->propagate(request);
      if (!prediction.ok()) {
        return prediction.status();
      }

      const auto noise = rng.sample_multivariate_normal(
          core::Vector::Zero(particles.dimension()),
          process_noise.value());
      if (!noise.ok()) {
        return noise.status();
      }

      propagated.particles.col(i) = prediction.value().state.value + noise.value();
    }

    return propagated;
  }

  [[nodiscard]] core::Result<ParticleSet> weight_particles(
      const ParticleSet& particles,
      const models::SensorModel& sensor,
      const core::Measurement& measurement,
      const models::ModelContext& context) const {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok()) {
      return particle_status;
    }

    const core::Status sensor_status = sensor.validate();
    if (!sensor_status.ok()) {
      return sensor_status;
    }

    ParticleSet weighted = particles;
    std::vector<core::Scalar> log_weights;
    log_weights.reserve(static_cast<std::size_t>(particles.size()));

    for (core::Index i = 0; i < particles.size(); ++i) {
      models::MeasurementRequest request {
          .state = {},
          .context = context,
          .sensor_id = measurement.sensor_id,
      };
      request.state.value = particles.particles.col(i);
      const auto predicted = sensor.measurement->measure(request);
      if (!predicted.ok()) {
        return predicted.status();
      }

      const auto noise = sensor.measurement_noise->covariance(request);
      if (!noise.ok()) {
        return noise.status();
      }

      const auto log_likelihood = core::stats::gaussian_log_likelihood(
          measurement.value,
          predicted.value().measurement.value,
          noise.value());
      if (!log_likelihood.ok()) {
        return log_likelihood.status();
      }

      log_weights.push_back(log_likelihood.value());
    }

    const auto normalized =
        core::stats::normalize_log_weights(log_weights);
    if (!normalized.ok()) {
      return normalized.status();
    }

    weighted.weights = normalized.value();
    return weighted;
  }

  [[nodiscard]] core::Result<ParticleSet> resample_particles(
      const ParticleSet& particles) const {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok()) {
      return particle_status;
    }

    const auto indices =
        core::utils::systematic_resample(particles.weights, resampling_offset_);
    if (!indices.ok()) {
      return indices.status();
    }

    ParticleSet resampled;
    resampled.particles =
        core::Matrix::Zero(particles.dimension(), particles.size());
    resampled.weights.assign(
        static_cast<std::size_t>(particles.size()),
        1.0 / static_cast<core::Scalar>(particles.size()));

    for (core::Index i = 0; i < particles.size(); ++i) {
      const std::size_t source = indices.value()[static_cast<std::size_t>(i)];
      resampled.particles.col(i) =
          particles.particles.col(static_cast<core::Index>(source));
    }

    return resampled;
  }

  [[nodiscard]] core::Result<GaussianEstimate> particle_set_to_estimate(
      const ParticleSet& particles,
      const core::Scalar timestamp,
      const std::string& frame_id) const {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok()) {
      return particle_status;
    }

    std::vector<core::Vector> samples;
    samples.reserve(static_cast<std::size_t>(particles.size()));
    for (core::Index i = 0; i < particles.size(); ++i) {
      samples.push_back(particles.particles.col(i));
    }

    const auto mean = core::stats::weighted_mean(samples, particles.weights);
    if (!mean.ok()) {
      return mean.status();
    }

    const auto covariance =
        core::stats::weighted_covariance(samples, particles.weights);
    if (!covariance.ok()) {
      return covariance.status();
    }

    GaussianEstimate estimate;
    estimate.state.value = mean.value();
    estimate.state.timestamp = timestamp;
    estimate.state.frame_id = frame_id;
    estimate.covariance = covariance.value();
    return estimate;
  }

  core::Index particle_count_;
  std::uint64_t seed_;
  core::Scalar resampling_offset_;
};

}  // namespace ros_tracker::filters
