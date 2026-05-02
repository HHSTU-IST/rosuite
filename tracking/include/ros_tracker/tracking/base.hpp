#pragma once

#include <cmath>
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

class TrackEstimatorModelHandle {
 public:
  virtual ~TrackEstimatorModelHandle() = default;

  [[nodiscard]] virtual core::Status validate() const = 0;

  [[nodiscard]] virtual core::Result<filters::GaussianEstimate> predict(
      const filters::GaussianEstimate& estimate,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) const = 0;

  [[nodiscard]] virtual core::Result<filters::GaussianEstimate> correct(
      const filters::GaussianEstimate& estimate,
      const core::Measurement& measurement,
      const models::ModelContext& context = {}) const = 0;

  [[nodiscard]] virtual const models::SensorModel& association_sensor_model()
      const noexcept = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class StaticTrackEstimatorModelHandle final : public TrackEstimatorModelHandle {
 public:
  StaticTrackEstimatorModelHandle(
      std::shared_ptr<const filters::FilterBase> filter,
      models::DynamicSystemModel system_model,
      models::SensorModel sensor_model,
      std::string name = "static_handle")
      : filter_(std::move(filter)),
        system_model_(std::move(system_model)),
        sensor_model_(std::move(sensor_model)),
        name_(std::move(name)) {}

  [[nodiscard]] core::Status validate() const override {
    if (!filter_) {
      return core::Status::invalid_argument(
          "StaticTrackEstimatorModelHandle requires a filter.");
    }

    const core::Status system_status = system_model_.validate();
    if (!system_status.ok()) {
      return system_status;
    }

    return sensor_model_.validate();
  }

  [[nodiscard]] core::Result<filters::GaussianEstimate> predict(
      const filters::GaussianEstimate& estimate,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) const override {
    const core::Status status = validate();
    if (!status.ok()) {
      return status;
    }

    return filter_->predict(estimate, system_model_, context, std::move(control));
  }

  [[nodiscard]] core::Result<filters::GaussianEstimate> correct(
      const filters::GaussianEstimate& estimate,
      const core::Measurement& measurement,
      const models::ModelContext& context = {}) const override {
    const core::Status status = validate();
    if (!status.ok()) {
      return status;
    }

    return filter_->correct(estimate, sensor_model_, measurement, context);
  }

  [[nodiscard]] const models::SensorModel& association_sensor_model()
      const noexcept override {
    return sensor_model_;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return name_;
  }

 private:
  std::shared_ptr<const filters::FilterBase> filter_;
  models::DynamicSystemModel system_model_;
  models::SensorModel sensor_model_;
  std::string name_;
};

class TrackHandleFactory {
 public:
  virtual ~TrackHandleFactory() = default;

  [[nodiscard]] virtual core::Result<std::shared_ptr<const TrackEstimatorModelHandle>>
  make_handle(
      const core::Measurement& measurement,
      const models::ModelContext& context = {}) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

struct Track {
  TrackId id {0U};
  filters::GaussianEstimate estimate;
  std::shared_ptr<const TrackEstimatorModelHandle> handle;
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

struct MeasurementBatchSummary {
  core::Scalar timestamp {0.0};
  std::string frame_id;
};

[[nodiscard]] inline core::Result<MeasurementBatchSummary>
summarize_measurement_batch(
    const std::vector<core::Measurement>& measurements) {
  MeasurementBatchSummary summary;
  if (measurements.empty()) {
    return summary;
  }

  summary.timestamp = measurements.front().timestamp;
  summary.frame_id = measurements.front().frame_id;

  for (std::size_t i = 1; i < measurements.size(); ++i) {
    if (std::abs(measurements[i].timestamp - summary.timestamp) > 1e-9) {
      return core::Status::invalid_argument(
          "All measurements in a batch must share the same timestamp.");
    }

    if (!measurements[i].frame_id.empty()) {
      if (summary.frame_id.empty()) {
        summary.frame_id = measurements[i].frame_id;
      } else if (measurements[i].frame_id != summary.frame_id) {
        return core::Status::invalid_argument(
            "All measurements in a batch must share the same frame_id.");
      }
    }
  }

  return summary;
}

[[nodiscard]] inline core::Status validate_track(const Track& track) {
  if (track.age == 0U) {
    return core::Status::invalid_argument(
        "Track age must be at least one.");
  }

  if (!track.handle) {
    return core::Status::invalid_argument(
        "Track requires a per-track estimator/model handle.");
  }

  const core::Status handle_status = track.handle->validate();
  if (!handle_status.ok()) {
    return handle_status;
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
      std::shared_ptr<const TrackEstimatorModelHandle> handle,
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
