#pragma once

#include <algorithm>
#include <tuple>
#include <vector>

#include "ros_tracker/core/math/statistics.hpp"
#include "ros_tracker/tracking/base.hpp"

namespace ros_tracker::tracking {

namespace detail {

[[nodiscard]] inline core::Result<core::Scalar> association_cost(
    const Track& track,
    const models::SensorModel& sensor,
    const core::Measurement& measurement,
    const models::ModelContext& context) {
  const core::Status track_status = validate_track(track);
  if (!track_status.ok()) {
    return track_status;
  }

  const core::Status sensor_status = sensor.validate();
  if (!sensor_status.ok()) {
    return sensor_status;
  }

  const models::MeasurementRequest request {
      .state = track.estimate.state,
      .context = context,
      .sensor_id = measurement.sensor_id,
  };

  const auto predicted_measurement = sensor.measurement->measure(request);
  if (!predicted_measurement.ok()) {
    return predicted_measurement.status();
  }

  if (predicted_measurement.value().measurement.dimension() !=
      measurement.dimension()) {
    return core::Status::dimension_mismatch(
        "Association measurement dimension must match the predicted measurement dimension.");
  }

  const auto jacobian = sensor.measurement->state_jacobian(request);
  if (!jacobian.ok()) {
    if (jacobian.status().code == core::StatusCode::kUnimplemented) {
      return (measurement.value -
              predicted_measurement.value().measurement.value).squaredNorm();
    }

    return jacobian.status();
  }

  const auto measurement_noise = sensor.measurement_noise->covariance(request);
  if (!measurement_noise.ok()) {
    return measurement_noise.status();
  }

  if (measurement_noise.value().rows() != measurement.dimension() ||
      measurement_noise.value().cols() != measurement.dimension()) {
    return core::Status::dimension_mismatch(
        "Association measurement noise covariance dimension must match the measurement dimension.");
  }

  const core::Covariance innovation_covariance = core::symmetrize(
      jacobian.value() * track.estimate.covariance * jacobian.value().transpose() +
      measurement_noise.value());
  return core::stats::mahalanobis_distance_squared(
      measurement.value,
      predicted_measurement.value().measurement.value,
      innovation_covariance);
}

}  // namespace detail

class NearestNeighborAssociationStrategy final : public AssociationStrategy {
 public:
  explicit NearestNeighborAssociationStrategy(
      const core::Scalar gating_threshold = 16.0)
      : gating_threshold_(gating_threshold) {}

  [[nodiscard]] core::Result<AssociationResult> associate(
      const std::vector<Track>& tracks,
      const std::vector<core::Measurement>& measurements,
      const models::SensorModel& sensor,
      const models::ModelContext& context = {}) const override {
    if (gating_threshold_ < 0.0) {
      return core::Status::invalid_argument(
          "Nearest-neighbor gating threshold must be non-negative.");
    }

    AssociationResult result;
    result.unmatched_tracks.reserve(tracks.size());
    result.unmatched_measurements.reserve(measurements.size());
    for (std::size_t i = 0; i < tracks.size(); ++i) {
      result.unmatched_tracks.push_back(i);
    }
    for (std::size_t i = 0; i < measurements.size(); ++i) {
      result.unmatched_measurements.push_back(i);
    }

    using Candidate = std::tuple<core::Scalar, std::size_t, std::size_t>;
    std::vector<Candidate> candidates;
    candidates.reserve(tracks.size() * measurements.size());

    for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
      for (std::size_t measurement_index = 0; measurement_index < measurements.size();
           ++measurement_index) {
        const auto cost = detail::association_cost(
            tracks[track_index], sensor, measurements[measurement_index], context);
        if (!cost.ok()) {
          return cost.status();
        }

        if (cost.value() <= gating_threshold_) {
          candidates.emplace_back(cost.value(), track_index, measurement_index);
        }
      }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& lhs, const Candidate& rhs) {
                return std::get<0>(lhs) < std::get<0>(rhs);
              });

    std::vector<bool> used_tracks(tracks.size(), false);
    std::vector<bool> used_measurements(measurements.size(), false);

    for (const auto& [cost, track_index, measurement_index] : candidates) {
      if (used_tracks[track_index] || used_measurements[measurement_index]) {
        continue;
      }

      used_tracks[track_index] = true;
      used_measurements[measurement_index] = true;
      result.matches.push_back(
          Association {track_index, measurement_index, cost});
    }

    result.unmatched_tracks.clear();
    for (std::size_t i = 0; i < used_tracks.size(); ++i) {
      if (!used_tracks[i]) {
        result.unmatched_tracks.push_back(i);
      }
    }

    result.unmatched_measurements.clear();
    for (std::size_t i = 0; i < used_measurements.size(); ++i) {
      if (!used_measurements[i]) {
        result.unmatched_measurements.push_back(i);
      }
    }

    return result;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "nearest_neighbor";
  }

 private:
  core::Scalar gating_threshold_;
};

}  // namespace ros_tracker::tracking
