#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "ros_tracker/tracking/base.hpp"

namespace ros_tracker::tracking {

class MultiTargetTracker final : public TrackerBase {
 public:
  MultiTargetTracker(
      std::shared_ptr<const filters::FilterBase> filter,
      models::DynamicSystemModel system_model,
      models::SensorModel sensor_model,
      std::shared_ptr<const AssociationStrategy> association_strategy,
      std::shared_ptr<const TrackManager> track_manager)
      : default_dependencies_ {
            std::move(filter),
            std::move(system_model),
            std::move(sensor_model),
        },
        association_strategy_(std::move(association_strategy)),
        track_manager_(std::move(track_manager)) {}

  MultiTargetTracker(
      TrackDependencies default_dependencies,
      std::shared_ptr<const AssociationStrategy> association_strategy,
      std::shared_ptr<const TrackManager> track_manager)
      : default_dependencies_(std::move(default_dependencies)),
        association_strategy_(std::move(association_strategy)),
        track_manager_(std::move(track_manager)) {}

  [[nodiscard]] core::Result<std::vector<Track>> step(
      const std::vector<core::Measurement>& measurements,
      const models::ModelContext& context,
      std::optional<core::ControlInput> control = std::nullopt) override {
    const core::Status dependency_status = validate_dependencies();
    if (!dependency_status.ok()) {
      return dependency_status;
    }

    std::vector<Track> predicted_tracks;
    predicted_tracks.reserve(tracks_.size());
    for (const Track& track : tracks_) {
      const auto dependencies = resolve_dependencies(track);
      if (!dependencies.ok()) {
        return dependencies.status();
      }

      const auto predicted_estimate = dependencies.value().filter->predict(
          track.estimate,
          dependencies.value().system_model,
          context,
          control);
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
        default_dependencies_.sensor_model,
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
      const auto dependencies = resolve_dependencies(predicted_track);
      if (!dependencies.ok()) {
        return dependencies.status();
      }

      const models::SensorModel& sensor_model =
          dependencies.value().sensor_model.validate().ok()
              ? dependencies.value().sensor_model
              : default_dependencies_.sensor_model;
      const auto corrected_estimate = dependencies.value().filter->correct(
          predicted_track.estimate,
          sensor_model,
          measurement,
          context);
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
      const auto initiated_track = track_manager_->initiate_track(
          next_track_id_++,
          measurements[unmatched_measurement_index],
          default_dependencies_,
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

  [[nodiscard]] const std::vector<Track>& tracks() const noexcept override {
    return tracks_;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "multi_target";
  }

 private:
  [[nodiscard]] core::Status validate_dependencies() const {
    if (!association_strategy_) {
      return core::Status::invalid_argument(
          "MultiTargetTracker requires an association strategy.");
    }

    if (!track_manager_) {
      return core::Status::invalid_argument(
          "MultiTargetTracker requires a track manager.");
    }

    return default_dependencies_.validate();
  }

  [[nodiscard]] core::Result<TrackDependencies> resolve_dependencies(
      const Track& track) const {
    if (!track.dependencies.empty()) {
      const core::Status track_status = track.dependencies.validate();
      if (!track_status.ok()) {
        return track_status;
      }

      return track.dependencies;
    }

    const core::Status default_status = default_dependencies_.validate();
    if (!default_status.ok()) {
      return default_status;
    }

    return default_dependencies_;
  }

  TrackDependencies default_dependencies_;
  std::shared_ptr<const AssociationStrategy> association_strategy_;
  std::shared_ptr<const TrackManager> track_manager_;
  std::vector<Track> tracks_;
  TrackId next_track_id_ {1U};
};

}  // namespace ros_tracker::tracking
