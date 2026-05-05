#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rosuite/tracking/track_lifecycle.hpp"

namespace rosuite::apps::ros
{
  struct TrackerNodeParameters
  {
    core::Scalar dt{1.0};
    std::string frame_id{"map"};
  };

  struct TrackMessage
  {
    tracking::TrackId id{0U};
    core::Scalar timestamp{0.0};
    std::string frame_id;
    std::string sensor_id;
    std::string lifecycle;
    std::vector<core::Scalar> state;
    std::vector<core::Scalar> covariance;
  };

  struct TrackArrayMessage
  {
    core::Scalar timestamp{0.0};
    std::string frame_id;
    std::vector<TrackMessage> tracks;
  };

  /// Converts a track lifecycle enum to a string label.
  [[nodiscard]] std::string lifecycle_to_string(
      tracking::TrackLifecycle lifecycle);

  /// Converts an internal track into an adapter message.
  [[nodiscard]] TrackMessage to_track_message(const tracking::Track &track);

  class TrackerNodeAdapter
  {
  public:
    /// Constructs TrackerNodeAdapter.
    TrackerNodeAdapter(
        std::shared_ptr<tracking::TrackerBase> tracker,
        TrackerNodeParameters parameters = {});

    /// Processes a measurement batch and produces track output.
    [[nodiscard]] core::Result<TrackArrayMessage> process_measurements(
        const std::vector<core::Measurement> &measurements,
        std::optional<core::ControlInput> control = std::nullopt);

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept;

  private:
    std::shared_ptr<tracking::TrackerBase> tracker_;
    TrackerNodeParameters parameters_;
  };

} // namespace rosuite::apps::ros
