#pragma once

#include <cstddef>
#include <vector>

#include "rosuite/filters/estimate.hpp"
#include "rosuite/tracking/base.hpp"

namespace rosuite::apps::offline
{

  struct TrackerFrame
  {
    core::Scalar timestamp{0.0};
    std::vector<tracking::Track> tracks;
  };

  struct TrackingMetrics
  {
    core::Scalar position_rmse{0.0};
    core::Scalar velocity_rmse{0.0};
    std::size_t truth_frames{0U};
    std::size_t estimated_frames{0U};
  };

  /// Computes tracking metrics against ground truth.
  [[nodiscard]] inline core::Result<TrackingMetrics> compute_tracking_metrics(
      const std::vector<core::State> &truth_states,
      const std::vector<filters::GaussianEstimate> &estimated_states)
  {
    if (truth_states.empty() || estimated_states.empty())
    {
      return core::Status::invalid_argument(
          "Tracking metrics require non-empty truth and estimate sequences.");
    }

    if (truth_states.size() != estimated_states.size())
    {
      return core::Status::dimension_mismatch(
          "Tracking metrics require equal-length truth and estimate sequences.");
    }

    core::Scalar position_squared_error = 0.0;
    core::Scalar velocity_squared_error = 0.0;

    for (std::size_t i = 0; i < truth_states.size(); ++i)
    {
      if (truth_states[i].dimension() != estimated_states[i].dimension())
      {
        return core::Status::dimension_mismatch(
            "Tracking metrics require matching state dimensions.");
      }

      if (truth_states[i].dimension() < 4)
      {
        return core::Status::dimension_mismatch(
            "Tracking metrics expect at least a 4D state [x, y, vx, vy].");
      }

      position_squared_error +=
          (truth_states[i].value.segment(0, 2) -
           estimated_states[i].state.value.segment(0, 2))
              .squaredNorm();
      velocity_squared_error +=
          (truth_states[i].value.segment(2, 2) -
           estimated_states[i].state.value.segment(2, 2))
              .squaredNorm();
    }

    TrackingMetrics metrics;
    metrics.position_rmse = std::sqrt(
        position_squared_error /
        static_cast<core::Scalar>(2U * truth_states.size()));
    metrics.velocity_rmse = std::sqrt(
        velocity_squared_error /
        static_cast<core::Scalar>(2U * truth_states.size()));
    metrics.truth_frames = truth_states.size();
    metrics.estimated_frames = estimated_states.size();
    return metrics;
  }

  /// Extracts the primary estimates from a track set.
  [[nodiscard]] inline std::vector<filters::GaussianEstimate>
  extract_primary_track_estimates(
      const std::vector<TrackerFrame> &tracker_frames)
  {
    std::vector<filters::GaussianEstimate> estimates;
    estimates.reserve(tracker_frames.size());

    for (const TrackerFrame &frame : tracker_frames)
    {
      if (!frame.tracks.empty())
      {
        estimates.push_back(frame.tracks.front().estimate);
      }
    }

    return estimates;
  }

} // namespace rosuite::apps::offline
