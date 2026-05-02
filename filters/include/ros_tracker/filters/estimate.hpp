#pragma once

#include <vector>

#include "ros_tracker/core/math/linear_algebra.hpp"
#include "ros_tracker/core/result.hpp"
#include "ros_tracker/core/status.hpp"
#include "ros_tracker/core/types.hpp"

namespace ros_tracker::filters {

struct GaussianEstimate {
  core::State state;
  core::Covariance covariance;

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

  [[nodiscard]] core::Index dimension() const noexcept {
    return solution.size();
  }
};

struct SigmaPointSet {
  core::Matrix points;
  core::Vector mean_weights;
  core::Vector covariance_weights;
};

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

  return core::validate_covariance(estimate.covariance);
}

}  // namespace ros_tracker::filters
