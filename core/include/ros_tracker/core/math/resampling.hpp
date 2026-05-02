#pragma once

#include <cstddef>
#include <vector>

#include "ros_tracker/core/math/statistics.hpp"

namespace ros_tracker::core::utils {

[[nodiscard]] inline Result<std::vector<std::size_t>> systematic_resample(
    std::vector<Scalar> weights,
    const Scalar offset = Scalar {0.5}) {
  const auto normalization = stats::normalize_weights_in_place(weights);
  if (!normalization.ok()) {
    return normalization.status();
  }

  if (offset < Scalar {0.0} || offset >= Scalar {1.0}) {
    return Status::out_of_range(
        "Systematic resampling offset must be in [0, 1).");
  }

  const std::size_t particle_count = weights.size();
  const Scalar step = Scalar {1.0} / static_cast<Scalar>(particle_count);
  Scalar threshold = offset * step;
  Scalar cumulative_weight = weights.front();
  std::size_t source_index = 0U;

  std::vector<std::size_t> indices;
  indices.reserve(particle_count);

  for (std::size_t i = 0; i < particle_count; ++i) {
    while (threshold > cumulative_weight &&
           source_index + 1U < particle_count) {
      ++source_index;
      cumulative_weight += weights[source_index];
    }

    indices.push_back(source_index);
    threshold += step;
  }

  return indices;
}

}  // namespace ros_tracker::core::utils
