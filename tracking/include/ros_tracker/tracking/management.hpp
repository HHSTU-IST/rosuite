#pragma once

#include <utility>
#include <vector>

#include "ros_tracker/tracking/base.hpp"

namespace ros_tracker::tracking {

class BasicTrackManager final : public TrackManager {
 public:
  explicit BasicTrackManager(
      const core::Index state_dimension,
      core::Covariance initial_covariance,
      std::vector<core::Index> measured_state_indices = {},
      const std::size_t confirmation_hits = 2U,
      const std::size_t max_consecutive_misses = 2U)
      : state_dimension_(state_dimension),
        initial_covariance_(std::move(initial_covariance)),
        measured_state_indices_(std::move(measured_state_indices)),
        confirmation_hits_(confirmation_hits),
        max_consecutive_misses_(max_consecutive_misses) {}

  [[nodiscard]] core::Result<Track> initiate_track(
      TrackId id,
      const core::Measurement& measurement,
      const models::SensorModel& sensor,
      const models::ModelContext& /*context*/ = {}) const override {
    const core::Status sensor_status = sensor.validate();
    if (!sensor_status.ok()) {
      return sensor_status;
    }

    if (state_dimension_ <= 0) {
      return core::Status::invalid_argument(
          "BasicTrackManager requires a positive state dimension.");
    }

    if (confirmation_hits_ == 0U) {
      return core::Status::invalid_argument(
          "BasicTrackManager confirmation_hits must be at least one.");
    }

    if (initial_covariance_.rows() != state_dimension_ ||
        initial_covariance_.cols() != state_dimension_) {
      return core::Status::dimension_mismatch(
          "BasicTrackManager initial covariance dimension must match the state dimension.");
    }

    const core::Status covariance_status =
        core::validate_covariance(initial_covariance_);
    if (!covariance_status.ok()) {
      return covariance_status;
    }

    std::vector<core::Index> measured_state_indices = measured_state_indices_;
    if (measured_state_indices.empty()) {
      measured_state_indices.reserve(static_cast<std::size_t>(measurement.dimension()));
      for (core::Index i = 0; i < measurement.dimension(); ++i) {
        measured_state_indices.push_back(i);
      }
    }

    if (static_cast<core::Index>(measured_state_indices.size()) !=
        measurement.dimension()) {
      return core::Status::dimension_mismatch(
          "Measured state index count must match the measurement dimension.");
    }

    core::Vector initial_state = core::Vector::Zero(state_dimension_);
    for (core::Index i = 0; i < measurement.dimension(); ++i) {
      const core::Index state_index =
          measured_state_indices[static_cast<std::size_t>(i)];
      if (state_index < 0 || state_index >= state_dimension_) {
        return core::Status::out_of_range(
            "Measured state index is out of range for the configured state dimension.");
      }
      initial_state[state_index] = measurement.value[i];
    }

    Track track;
    track.id = id;
    track.estimate.state.value = initial_state;
    track.estimate.state.timestamp = measurement.timestamp;
    track.estimate.state.frame_id = measurement.frame_id;
    track.estimate.covariance = initial_covariance_;
    track.lifecycle = confirmation_hits_ <= 1U
                          ? TrackLifecycle::kConfirmed
                          : TrackLifecycle::kTentative;
    track.age = 1U;
    track.hit_count = 1U;
    track.source_sensor_id = measurement.sensor_id;
    return track;
  }

  [[nodiscard]] core::Result<Track> on_prediction(
      const Track& track,
      const filters::GaussianEstimate& predicted_estimate) const override {
    const core::Status estimate_status =
        filters::validate_estimate(predicted_estimate);
    if (!estimate_status.ok()) {
      return estimate_status;
    }

    Track updated = track;
    updated.estimate = predicted_estimate;
    updated.age += 1U;
    return updated;
  }

  [[nodiscard]] core::Result<Track> on_correction(
      const Track& track,
      const filters::GaussianEstimate& corrected_estimate,
      const core::Measurement& measurement) const override {
    const core::Status estimate_status =
        filters::validate_estimate(corrected_estimate);
    if (!estimate_status.ok()) {
      return estimate_status;
    }

    Track updated = track;
    updated.estimate = corrected_estimate;
    updated.hit_count += 1U;
    updated.consecutive_misses = 0U;
    updated.source_sensor_id = measurement.sensor_id;
    if (updated.hit_count >= confirmation_hits_) {
      updated.lifecycle = TrackLifecycle::kConfirmed;
    }
    return updated;
  }

  [[nodiscard]] core::Result<Track> on_missed_detection(
      const Track& track) const override {
    Track updated = track;
    updated.miss_count += 1U;
    updated.consecutive_misses += 1U;
    if (updated.consecutive_misses >= max_consecutive_misses_) {
      updated.lifecycle = TrackLifecycle::kDeleted;
    }
    return updated;
  }

  [[nodiscard]] bool should_remove(const Track& track) const noexcept override {
    return track.lifecycle == TrackLifecycle::kDeleted;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "basic_manager";
  }

 private:
  core::Index state_dimension_;
  core::Covariance initial_covariance_;
  std::vector<core::Index> measured_state_indices_;
  std::size_t confirmation_hits_;
  std::size_t max_consecutive_misses_;
};

}  // namespace ros_tracker::tracking
