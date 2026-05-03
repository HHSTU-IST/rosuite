#pragma once

#include <optional>
#include <vector>

#include "ros_tracker/core/math/linear_algebra.hpp"
#include "ros_tracker/core/result.hpp"
#include "ros_tracker/core/status.hpp"
#include "ros_tracker/core/types.hpp"

namespace ros_tracker::filters {

struct ParticleSet {
  core::Matrix particles;
  std::vector<core::Scalar> weights;

  /// Returns the vector dimension.
  [[nodiscard]] core::Index dimension() const noexcept {
    return particles.rows();
  }

  /// Returns the particle count.
  [[nodiscard]] core::Index size() const noexcept {
    return particles.cols();
  }
};

struct GaussianEstimate {
  core::State state;
  core::Covariance covariance;
  std::optional<ParticleSet> particle_set;

  /// Returns the vector dimension.
  [[nodiscard]] core::Index dimension() const noexcept {
    return state.dimension();
  }
};

struct PredictedMeasurement {
  core::Measurement measurement;
  core::Covariance covariance;
  core::Matrix cross_covariance;
};

struct LeastSquaresEstimate {
  core::Vector solution;
  core::Covariance covariance;

  /// Returns the vector dimension.
  [[nodiscard]] core::Index dimension() const noexcept {
    return solution.size();
  }
};

struct SigmaPointSet {
  core::Matrix points;
  core::Vector mean_weights;
  core::Vector covariance_weights;
};

/// Validates a particle set.
[[nodiscard]] inline core::Status validate_particle_set(
    const ParticleSet& particle_set) {
  if (particle_set.particles.rows() <= 0 || particle_set.particles.cols() <= 0) {
    return core::Status::invalid_argument(
        "ParticleSet must contain at least one particle.");
  }

  if (static_cast<core::Index>(particle_set.weights.size()) !=
      particle_set.particles.cols()) {
    return core::Status::dimension_mismatch(
        "Particle weights must match the number of particles.");
  }

  return core::Status::ok_status();
}

/// Validates a Gaussian estimate.
[[nodiscard]] inline core::Status validate_estimate(
    const GaussianEstimate& estimate) {
  if (estimate.state.dimension() <= 0) {
    return core::Status::invalid_argument(
        "GaussianEstimate state vector must not be empty.");
  }

  if (estimate.covariance.rows() != estimate.dimension() ||
      estimate.covariance.cols() != estimate.dimension()) {
    return core::Status::dimension_mismatch(
        "GaussianEstimate covariance dimension must match the state dimension.");
  }

  if (estimate.particle_set.has_value()) {
    const core::Status particle_status =
        validate_particle_set(*estimate.particle_set);
    if (!particle_status.ok()) {
      return particle_status;
    }

    if (estimate.particle_set->dimension() != estimate.dimension()) {
      return core::Status::dimension_mismatch(
          "GaussianEstimate particle-set dimension must match the state dimension.");
    }
  }

  return core::validate_covariance(estimate.covariance);
}

}  // namespace ros_tracker::filters
