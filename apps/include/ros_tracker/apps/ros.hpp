#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ros_tracker/tracking/tracking.hpp"

namespace ros_tracker::apps::ros {

struct TrackerNodeParameters {
  core::Scalar dt {1.0};
  std::string frame_id {"map"};
};

struct TrackMessage {
  tracking::TrackId id {0U};
  core::Scalar timestamp {0.0};
  std::string frame_id;
  std::string sensor_id;
  std::string lifecycle;
  std::vector<core::Scalar> state;
  std::vector<core::Scalar> covariance;
};

struct TrackArrayMessage {
  core::Scalar timestamp {0.0};
  std::string frame_id;
  std::vector<TrackMessage> tracks;
};

[[nodiscard]] inline std::string lifecycle_to_string(
    const tracking::TrackLifecycle lifecycle) {
  switch (lifecycle) {
    case tracking::TrackLifecycle::kTentative:
      return "tentative";
    case tracking::TrackLifecycle::kConfirmed:
      return "confirmed";
    case tracking::TrackLifecycle::kDeleted:
      return "deleted";
  }

  return "unknown";
}

[[nodiscard]] inline TrackMessage to_track_message(
    const tracking::Track& track) {
  TrackMessage message;
  message.id = track.id;
  message.timestamp = track.estimate.state.timestamp;
  message.frame_id = track.estimate.state.frame_id;
  message.sensor_id = track.source_sensor_id;
  message.lifecycle = lifecycle_to_string(track.lifecycle);
  message.state.assign(
      track.estimate.state.value.data(),
      track.estimate.state.value.data() + track.estimate.state.value.size());
  message.covariance.reserve(static_cast<std::size_t>(
      track.estimate.covariance.rows() * track.estimate.covariance.cols()));
  for (core::Index row = 0; row < track.estimate.covariance.rows(); ++row) {
    for (core::Index col = 0; col < track.estimate.covariance.cols(); ++col) {
      message.covariance.push_back(track.estimate.covariance(row, col));
    }
  }
  return message;
}

class TrackerNodeAdapter {
 public:
  TrackerNodeAdapter(
      std::shared_ptr<tracking::TrackerBase> tracker,
      TrackerNodeParameters parameters = {})
      : tracker_(std::move(tracker)),
        parameters_(std::move(parameters)) {}

  [[nodiscard]] core::Result<TrackArrayMessage> process_measurements(
      const std::vector<core::Measurement>& measurements,
      std::optional<core::ControlInput> control = std::nullopt) {
    if (!tracker_) {
      return core::Status::invalid_argument(
          "TrackerNodeAdapter requires a tracker instance.");
    }

    core::Scalar timestamp = 0.0;
    std::string frame_id = parameters_.frame_id;
    if (!measurements.empty()) {
      timestamp = measurements.front().timestamp;
      if (!measurements.front().frame_id.empty()) {
        frame_id = measurements.front().frame_id;
      }
    }

    const auto tracks = tracker_->step(
        measurements,
        models::ModelContext {parameters_.dt, timestamp, frame_id},
        std::move(control));
    if (!tracks.ok()) {
      return tracks.status();
    }

    TrackArrayMessage message;
    message.timestamp = timestamp;
    message.frame_id = frame_id;
    message.tracks.reserve(tracks.value().size());
    for (const tracking::Track& track : tracks.value()) {
      message.tracks.push_back(to_track_message(track));
    }
    return message;
  }

  [[nodiscard]] std::string_view name() const noexcept {
    return "tracker_node_adapter";
  }

 private:
  std::shared_ptr<tracking::TrackerBase> tracker_;
  TrackerNodeParameters parameters_;
};

}  // namespace ros_tracker::apps::ros
