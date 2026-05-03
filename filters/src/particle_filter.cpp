#include "kracker/filters/particle_filter.hpp"

namespace kracker::filters
{
  ParticleFilter::ParticleFilter(
      const core::Index particle_count,
      const std::uint64_t seed,
      const core::Scalar resampling_offset)
      : particle_count_(particle_count),
        rng_(seed),
        resampling_offset_(resampling_offset) {}

  core::Result<GaussianEstimate> ParticleFilter::predict(
      const GaussianEstimate &estimate,
      const models::DynamicSystemModel &model,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control) const
  {
    const auto prior_particles = prior_particle_set(estimate);
    if (!prior_particles.ok())
    {
      return prior_particles.status();
    }

    const auto propagated = propagate_particles(
        prior_particles.value(), model, context, std::move(control));
    if (!propagated.ok())
    {
      return propagated.status();
    }

    return particle_set_to_estimate(
        propagated.value(),
        context.timestamp,
        context.frame_id.empty() ? estimate.state.frame_id : context.frame_id);
  }

  core::Result<GaussianEstimate> ParticleFilter::correct(
      const GaussianEstimate &estimate,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context) const
  {
    const auto prior_particles = prior_particle_set(estimate);
    if (!prior_particles.ok())
    {
      return prior_particles.status();
    }

    const auto weighted = weight_particles(
        prior_particles.value(), sensor, measurement, context);
    if (!weighted.ok())
    {
      return weighted.status();
    }

    const auto resampled = resample_particles(weighted.value());
    if (!resampled.ok())
    {
      return resampled.status();
    }

    return particle_set_to_estimate(
        resampled.value(),
        measurement.timestamp,
        measurement.frame_id.empty() ? estimate.state.frame_id : measurement.frame_id);
  }

  std::string_view ParticleFilter::name() const noexcept
  {
    return "particle";
  }

  core::Result<ParticleSet> ParticleFilter::prior_particle_set(
      const GaussianEstimate &estimate) const
  {
    const core::Status estimate_status = validate_estimate(estimate);
    if (!estimate_status.ok())
    {
      return estimate_status;
    }

    if (estimate.particle_set.has_value())
    {
      return *estimate.particle_set;
    }

    return sample_prior_particles(estimate);
  }

  core::Result<ParticleSet> ParticleFilter::sample_prior_particles(
      const GaussianEstimate &estimate) const
  {
    if (particle_count_ <= 0)
    {
      return core::Status::invalid_argument(
          "ParticleFilter requires at least one particle.");
    }

    ParticleSet particle_set;
    particle_set.particles = core::Matrix::Zero(
        estimate.dimension(), particle_count_);
    particle_set.weights.assign(
        static_cast<std::size_t>(particle_count_),
        1.0 / static_cast<core::Scalar>(particle_count_));

    for (core::Index i = 0; i < particle_count_; ++i)
    {
      const auto sample = rng_.sample_multivariate_normal(
          estimate.state.value, estimate.covariance);
      if (!sample.ok())
      {
        return sample.status();
      }

      particle_set.particles.col(i) = sample.value();
    }

    return particle_set;
  }

  core::Result<ParticleSet> ParticleFilter::propagate_particles(
      const ParticleSet &particles,
      const models::DynamicSystemModel &model,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control) const
  {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok())
    {
      return particle_status;
    }

    const core::Status model_status = model.validate();
    if (!model_status.ok())
    {
      return model_status;
    }

    ParticleSet propagated = particles;
    for (core::Index i = 0; i < particles.size(); ++i)
    {
      models::MotionRequest request{
          core::State{},
          control,
          context,
      };
      request.state.value = particles.particles.col(i);
      const auto prediction = model.motion->propagate(request);
      if (!prediction.ok())
      {
        return prediction.status();
      }

      const auto process_noise = model.process_noise->covariance(request);
      if (!process_noise.ok())
      {
        return process_noise.status();
      }

      const auto noise = rng_.sample_multivariate_normal(
          core::Vector::Zero(particles.dimension()),
          process_noise.value());
      if (!noise.ok())
      {
        return noise.status();
      }

      propagated.particles.col(i) = prediction.value().state.value + noise.value();
    }

    return propagated;
  }

  core::Result<ParticleSet> ParticleFilter::weight_particles(
      const ParticleSet &particles,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context) const
  {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok())
    {
      return particle_status;
    }

    const core::Status sensor_status = sensor.validate();
    if (!sensor_status.ok())
    {
      return sensor_status;
    }

    ParticleSet weighted = particles;
    std::vector<core::Scalar> log_weights;
    log_weights.reserve(static_cast<std::size_t>(particles.size()));

    for (core::Index i = 0; i < particles.size(); ++i)
    {
      models::MeasurementRequest request{
          core::State{},
          context,
          measurement.sensor_id,
      };
      request.state.value = particles.particles.col(i);
      const auto predicted = sensor.measurement->measure(request);
      if (!predicted.ok())
      {
        return predicted.status();
      }

      const auto noise = sensor.measurement_noise->covariance(request);
      if (!noise.ok())
      {
        return noise.status();
      }

      const auto log_likelihood = core::stats::gaussian_log_likelihood(
          measurement.value,
          predicted.value().measurement.value,
          noise.value());
      if (!log_likelihood.ok())
      {
        return log_likelihood.status();
      }

      log_weights.push_back(log_likelihood.value());
    }

    const auto normalized = core::stats::normalize_log_weights(log_weights);
    if (!normalized.ok())
    {
      return normalized.status();
    }

    weighted.weights = normalized.value();
    return weighted;
  }

  core::Result<ParticleSet> ParticleFilter::resample_particles(
      const ParticleSet &particles) const
  {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok())
    {
      return particle_status;
    }

    const auto indices =
        core::utils::systematic_resample(particles.weights, resampling_offset_);
    if (!indices.ok())
    {
      return indices.status();
    }

    ParticleSet resampled;
    resampled.particles =
        core::Matrix::Zero(particles.dimension(), particles.size());
    resampled.weights.assign(
        static_cast<std::size_t>(particles.size()),
        1.0 / static_cast<core::Scalar>(particles.size()));

    for (core::Index i = 0; i < particles.size(); ++i)
    {
      const std::size_t source = indices.value()[static_cast<std::size_t>(i)];
      resampled.particles.col(i) =
          particles.particles.col(static_cast<core::Index>(source));
    }

    return resampled;
  }

  core::Result<GaussianEstimate> ParticleFilter::particle_set_to_estimate(
      const ParticleSet &particles,
      const core::Scalar timestamp,
      const std::string &frame_id) const
  {
    const core::Status particle_status = validate_particle_set(particles);
    if (!particle_status.ok())
    {
      return particle_status;
    }

    std::vector<core::Vector> samples;
    samples.reserve(static_cast<std::size_t>(particles.size()));
    for (core::Index i = 0; i < particles.size(); ++i)
    {
      samples.push_back(particles.particles.col(i));
    }

    const auto mean = core::stats::weighted_mean(samples, particles.weights);
    if (!mean.ok())
    {
      return mean.status();
    }

    const auto covariance =
        core::stats::weighted_covariance(samples, particles.weights);
    if (!covariance.ok())
    {
      return covariance.status();
    }

    GaussianEstimate estimate;
    estimate.state.value = mean.value();
    estimate.state.timestamp = timestamp;
    estimate.state.frame_id = frame_id;
    estimate.covariance = covariance.value();
    estimate.particle_set = particles;
    return estimate;
  }

} // namespace kracker::filters
