#include "ros_tracker/tracking/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace ros_tracker::tracking {

MultiTargetTracker::MultiTargetTracker(
    std::shared_ptr<const filters::FilterBase> filter,
    models::DynamicSystemModel system_model,
    models::SensorModel sensor_model,
    std::shared_ptr<const AssociationStrategy> association_strategy,
    std::shared_ptr<const TrackManager> track_manager)
    : MultiTargetTracker(
          std::make_shared<StaticTrackEstimatorModelHandle>(
              std::move(filter),
              std::move(system_model),
              std::move(sensor_model),
              "default_track_handle"),
          std::move(association_strategy),
          std::move(track_manager)) {}

MultiTargetTracker::MultiTargetTracker(
    std::shared_ptr<const TrackEstimatorModelHandle> default_handle,
    std::shared_ptr<const AssociationStrategy> association_strategy,
    std::shared_ptr<const TrackManager> track_manager,
    std::shared_ptr<const TrackHandleFactory> handle_factory)
    : default_handle_(std::move(default_handle)),
      association_strategy_(std::move(association_strategy)),
      track_manager_(std::move(track_manager)),
      handle_factory_(std::move(handle_factory)) {}

core::Result<std::vector<Track>> MultiTargetTracker::step(
    const std::vector<core::Measurement>& measurements,
    const models::ModelContext& context,
    std::optional<core::ControlInput> control) {
  const core::Status dependency_status = validate_dependencies();
  if (!dependency_status.ok()) {
    return dependency_status;
  }

  const auto batch_summary = summarize_measurement_batch(measurements);
  if (!batch_summary.ok()) {
    return batch_summary.status();
  }

  if (!measurements.empty()) {
    if (std::abs(batch_summary.value().timestamp - context.timestamp) > 1e-9) {
      return core::Status::invalid_argument(
          "MultiTargetTracker context timestamp must match the measurement batch timestamp.");
    }

    if (!batch_summary.value().frame_id.empty() &&
        !context.frame_id.empty() &&
        batch_summary.value().frame_id != context.frame_id) {
      return core::Status::invalid_argument(
          "MultiTargetTracker context frame_id must match the measurement batch frame_id.");
    }
  }

  std::vector<Track> predicted_tracks;
  predicted_tracks.reserve(tracks_.size());
  for (const Track& track : tracks_) {
    const core::Status track_status = validate_track(track);
    if (!track_status.ok()) {
      return track_status;
    }

    const auto predicted_estimate =
        track.handle->predict(track.estimate, context, control);
    if (!predicted_estimate.ok()) {
      return predicted_estimate.status();
    }

    const auto predicted_track =
        track_manager_->on_prediction(track, predicted_estimate.value());
    if (!predicted_track.ok()) {
      return predicted_track.status();
    }

    predicted_tracks.push_back(predicted_track.value());
  }

  const auto association = association_strategy_->associate(
      predicted_tracks,
      measurements,
      default_handle_->association_sensor_model(),
      context);
  if (!association.ok()) {
    return association.status();
  }

  std::vector<Track> next_tracks;
  next_tracks.reserve(
      predicted_tracks.size() + association.value().unmatched_measurements.size());

  for (const Association& match : association.value().matches) {
    const Track& predicted_track = predicted_tracks[match.track_index];
    const core::Measurement& measurement = measurements[match.measurement_index];

    const auto corrected_estimate = predicted_track.handle->correct(
        predicted_track.estimate, measurement, context);
    if (!corrected_estimate.ok()) {
      return corrected_estimate.status();
    }

    const auto corrected_track = track_manager_->on_correction(
        predicted_track, corrected_estimate.value(), measurement);
    if (!corrected_track.ok()) {
      return corrected_track.status();
    }

    next_tracks.push_back(corrected_track.value());
  }

  for (const std::size_t unmatched_track_index :
       association.value().unmatched_tracks) {
    const auto missed_track = track_manager_->on_missed_detection(
        predicted_tracks[unmatched_track_index]);
    if (!missed_track.ok()) {
      return missed_track.status();
    }

    if (!track_manager_->should_remove(missed_track.value())) {
      next_tracks.push_back(missed_track.value());
    }
  }

  for (const std::size_t unmatched_measurement_index :
       association.value().unmatched_measurements) {
    const auto handle =
        make_handle(measurements[unmatched_measurement_index], context);
    if (!handle.ok()) {
      return handle.status();
    }

    const auto initiated_track = track_manager_->initiate_track(
        next_track_id_++,
        measurements[unmatched_measurement_index],
        handle.value(),
        context);
    if (!initiated_track.ok()) {
      return initiated_track.status();
    }

    next_tracks.push_back(initiated_track.value());
  }

  std::sort(next_tracks.begin(), next_tracks.end(),
            [](const Track& lhs, const Track& rhs) {
              return lhs.id < rhs.id;
            });
  tracks_ = std::move(next_tracks);
  return tracks_;
}

const std::vector<Track>& MultiTargetTracker::tracks() const noexcept {
  return tracks_;
}

std::string_view MultiTargetTracker::name() const noexcept {
  return "multi_target";
}

core::Status MultiTargetTracker::validate_dependencies() const {
  if (!default_handle_) {
    return core::Status::invalid_argument(
        "MultiTargetTracker requires a default per-track handle.");
  }

  const core::Status default_status = default_handle_->validate();
  if (!default_status.ok()) {
    return default_status;
  }

  if (!association_strategy_) {
    return core::Status::invalid_argument(
        "MultiTargetTracker requires an association strategy.");
  }

  if (!track_manager_) {
    return core::Status::invalid_argument(
        "MultiTargetTracker requires a track manager.");
  }

  return core::Status::ok_status();
}

core::Result<std::shared_ptr<const TrackEstimatorModelHandle>>
MultiTargetTracker::make_handle(
    const core::Measurement& measurement,
    const models::ModelContext& context) const {
  if (handle_factory_) {
    const auto handle = handle_factory_->make_handle(measurement, context);
    if (!handle.ok()) {
      return handle.status();
    }

    if (!handle.value()) {
      return core::Status::invalid_argument(
          "TrackHandleFactory returned a null per-track handle.");
    }

    const core::Status handle_status = handle.value()->validate();
    if (!handle_status.ok()) {
      return handle_status;
    }

    return handle.value();
  }

  return default_handle_;
}

}  // namespace ros_tracker::tracking
