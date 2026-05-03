#include "ros_tracker/apps/ros.hpp"

#include <utility>

namespace ros_tracker::apps::ros {

/// Constructs TrackerNodeAdapter.
TrackerNodeAdapter::TrackerNodeAdapter(
    std::shared_ptr<tracking::TrackerBase> tracker,
    TrackerNodeParameters parameters)
    : tracker_(std::move(tracker)),
      parameters_(std::move(parameters)) {}

/// Converts a track lifecycle enum to a string label.
std::string lifecycle_to_string(const tracking::TrackLifecycle lifecycle) {
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

/// Converts an internal track into an adapter message.
TrackMessage to_track_message(const tracking::Track& track) {
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

/// Processes a measurement batch and produces track output.
core::Result<TrackArrayMessage> TrackerNodeAdapter::process_measurements(
    const std::vector<core::Measurement>& measurements,
    std::optional<core::ControlInput> control) {
  if (!tracker_) {
    return core::Status::invalid_argument(
        "TrackerNodeAdapter requires a tracker instance.");
  }

  const auto batch_summary = tracking::summarize_measurement_batch(measurements);
  if (!batch_summary.ok()) {
    return batch_summary.status();
  }

  core::Scalar timestamp = 0.0;
  std::string frame_id = parameters_.frame_id;
  if (!measurements.empty()) {
    timestamp = batch_summary.value().timestamp;
    if (!batch_summary.value().frame_id.empty()) {
      frame_id = batch_summary.value().frame_id;
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

/// Returns the component name.
std::string_view TrackerNodeAdapter::name() const noexcept {
  return "tracker_node_adapter";
}

}  // namespace ros_tracker::apps::ros
