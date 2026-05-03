#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "ros_tracker/core/math/random.hpp"
#include "ros_tracker/core/math/resampling.hpp"
#include "ros_tracker/core/math/statistics.hpp"
#include "ros_tracker/filters/filter_base.hpp"

namespace ros_tracker::filters
{
  class ParticleFilter final : public FilterBase
  {
  public:
    /// Constructs ParticleFilter.
    explicit ParticleFilter(
        const core::Index particle_count = 256,
        const std::uint64_t seed = 0U,
        const core::Scalar resampling_offset = 0.5);

    /// Predicts the next estimate.
    [[nodiscard]] core::Result<GaussianEstimate> predict(
        const GaussianEstimate &estimate,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const override;

    /// Corrects an estimate with a measurement.
    [[nodiscard]] core::Result<GaussianEstimate> correct(
        const GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const override;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override;

  private:
    /// Returns the prior particle set for the current estimate.
    [[nodiscard]] core::Result<ParticleSet> prior_particle_set(
        const GaussianEstimate &estimate) const;

    /// Samples particles from the prior estimate.
    [[nodiscard]] core::Result<ParticleSet> sample_prior_particles(
        const GaussianEstimate &estimate) const;

    /// Propagates each particle through the motion model.
    [[nodiscard]] core::Result<ParticleSet> propagate_particles(
        const ParticleSet &particles,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const;

    /// Computes particle weights from a measurement.
    [[nodiscard]] core::Result<ParticleSet> weight_particles(
        const ParticleSet &particles,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context) const;

    /// Resamples particles according to their weights.
    [[nodiscard]] core::Result<ParticleSet> resample_particles(
        const ParticleSet &particles) const;

    /// Collapses a particle set into a Gaussian estimate.
    [[nodiscard]] core::Result<GaussianEstimate> particle_set_to_estimate(
        const ParticleSet &particles,
        const core::Scalar timestamp,
        const std::string &frame_id) const;

    core::Index particle_count_;
    mutable core::stats::RandomEngine rng_;
    core::Scalar resampling_offset_;
  };

} // namespace ros_tracker::filters
