#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ros_tracker/filters/filter_base.hpp"
#include "ros_tracker/core/result.hpp"
#include "ros_tracker/core/status.hpp"
#include "ros_tracker/core/types.hpp"
#include "ros_tracker/filters/estimate.hpp"
#include "ros_tracker/models/base.hpp"

namespace ros_tracker::tracking {

using TrackId = std::size_t;

enum class TrackLifecycle {
  kTentative = 0,
  kConfirmed,
  kDeleted,
};

struct TrackDependencies {
  std::shared_ptr<const filters::FilterBase> filter;
  models::DynamicSystemModel system_model;
  models::SensorModel sensor_model;

  [[nodiscard]] bool empty() const noexcept {
    return !filter &&
           !system_model.motion &&
           !system_model.process_noise &&
           !sensor_model.measurement &&
           !sensor_model.measurement_noise;
  }

  [[nodiscard]] core::Status validate() const {
    if (!filter) {
      return core::Status::invalid_argument(
          "TrackDependencies requires a filter.");
    }

    const core::Status system_status = system_model.validate();
    if (!system_status.ok()) {
      return system_status;
    }

    return sensor_model.validate();
  }
};

struct Track {
  TrackId id {0U};
  filters::GaussianEstimate estimate;
  TrackDependencies dependencies;
  TrackLifecycle lifecycle {TrackLifecycle::kTentative};
  std::size_t age {0U};
  std::size_t hit_count {0U};
  std::size_t miss_count {0U};
  std::size_t consecutive_misses {0U};
  std::string source_sensor_id;

  [[nodiscard]] core::Index dimension() const noexcept {
    return estimate.dimension();
  }
};

[[nodiscard]] inline core::Status validate_track(const Track& track) {
  if (track.age == 0U) {
    return core::Status::invalid_argument(
        "Track age must be at least one.");
  }

  if (!track.dependencies.empty()) {
    const core::Status dependency_status = track.dependencies.validate();
    if (!dependency_status.ok()) {
      return dependency_status;
    }
  }

  return filters::validate_estimate(track.estimate);
}

struct Association {
  std::size_t track_index {0U};
  std::size_t measurement_index {0U};
  core::Scalar cost {0.0};
};

struct AssociationResult {
  std::vector<Association> matches;
  std::vector<std::size_t> unmatched_tracks;
  std::vector<std::size_t> unmatched_measurements;
};

class AssociationStrategy {
 public:
  virtual ~AssociationStrategy() = default;

  [[nodiscard]] virtual core::Result<AssociationResult> associate(
      const std::vector<Track>& tracks,
      const std::vector<core::Measurement>& measurements,
      const models::SensorModel& sensor,
      const models::ModelContext& context = {}) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class TrackManager {
 public:
  virtual ~TrackManager() = default;

  [[nodiscard]] virtual core::Result<Track> initiate_track(
      TrackId id,
      const core::Measurement& measurement,
      const TrackDependencies& dependencies,
      const models::ModelContext& context = {}) const = 0;

  [[nodiscard]] virtual core::Result<Track> on_prediction(
      const Track& track,
      const filters::GaussianEstimate& predicted_estimate) const = 0;

  [[nodiscard]] virtual core::Result<Track> on_correction(
      const Track& track,
      const filters::GaussianEstimate& corrected_estimate,
      const core::Measurement& measurement) const = 0;

  [[nodiscard]] virtual core::Result<Track> on_missed_detection(
      const Track& track) const = 0;

  [[nodiscard]] virtual bool should_remove(const Track& track) const noexcept = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class TrackerBase {
 public:
  virtual ~TrackerBase() = default;

  [[nodiscard]] virtual core::Result<std::vector<Track>> step(
      const std::vector<core::Measurement>& measurements,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) = 0;

  [[nodiscard]] virtual const std::vector<Track>& tracks() const noexcept = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

}  // namespace ros_tracker::tracking
