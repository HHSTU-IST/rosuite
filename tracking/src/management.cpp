#include "ros_tracker/tracking/management.hpp"

#include <utility>

namespace ros_tracker::tracking
{
  /// Constructs BasicTrackManager.
  BasicTrackManager::BasicTrackManager(
      const core::Index state_dimension,
      core::Covariance initial_covariance,
      std::vector<core::Index> measured_state_indices,
      const std::size_t confirmation_hits,
      const std::size_t max_consecutive_misses)
      : state_dimension_(state_dimension),
        initial_covariance_(std::move(initial_covariance)),
        measured_state_indices_(std::move(measured_state_indices)),
        confirmation_hits_(confirmation_hits),
        max_consecutive_misses_(max_consecutive_misses) {}

  /// Initializes a new track from a measurement.
  core::Result<Track> BasicTrackManager::initiate_track(
      const TrackId id,
      const core::Measurement &measurement,
      std::shared_ptr<const TrackEstimatorModelHandle> handle,
      const models::ModelContext & /*context*/) const
  {
    if (!handle)
    {
      return core::Status::invalid_argument(
          "BasicTrackManager requires a per-track estimator/model handle.");
    }

    const core::Status handle_status = handle->validate();
    if (!handle_status.ok())
    {
      return handle_status;
    }

    if (state_dimension_ <= 0)
    {
      return core::Status::invalid_argument(
          "BasicTrackManager requires a positive state dimension.");
    }

    if (confirmation_hits_ == 0U)
    {
      return core::Status::invalid_argument(
          "BasicTrackManager confirmation_hits must be at least one.");
    }

    if (initial_covariance_.rows() != state_dimension_ ||
        initial_covariance_.cols() != state_dimension_)
    {
      return core::Status::dimension_mismatch(
          "BasicTrackManager initial covariance dimension must match the state dimension.");
    }

    const core::Status covariance_status =
        core::validate_covariance(initial_covariance_);
    if (!covariance_status.ok())
    {
      return covariance_status;
    }

    std::vector<core::Index> measured_state_indices = measured_state_indices_;
    if (measured_state_indices.empty())
    {
      measured_state_indices.reserve(static_cast<std::size_t>(measurement.dimension()));
      for (core::Index i = 0; i < measurement.dimension(); ++i)
      {
        measured_state_indices.push_back(i);
      }
    }

    if (static_cast<core::Index>(measured_state_indices.size()) !=
        measurement.dimension())
    {
      return core::Status::dimension_mismatch(
          "Measured state index count must match the measurement dimension.");
    }

    core::Vector initial_state = core::Vector::Zero(state_dimension_);
    for (core::Index i = 0; i < measurement.dimension(); ++i)
    {
      const core::Index state_index =
          measured_state_indices[static_cast<std::size_t>(i)];
      if (state_index < 0 || state_index >= state_dimension_)
      {
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
    track.handle = std::move(handle);
    track.lifecycle = confirmation_hits_ <= 1U
                          ? TrackLifecycle::kConfirmed
                          : TrackLifecycle::kTentative;
    track.age = 1U;
    track.hit_count = 1U;
    track.source_sensor_id = measurement.sensor_id;
    return track;
  }

  /// Updates track metadata after prediction.
  core::Result<Track> BasicTrackManager::on_prediction(
      const Track &track,
      const filters::GaussianEstimate &predicted_estimate) const
  {
    const core::Status estimate_status =
        filters::validate_estimate(predicted_estimate);
    if (!estimate_status.ok())
    {
      return estimate_status;
    }

    Track updated = track;
    updated.estimate = predicted_estimate;
    updated.age += 1U;
    return updated;
  }

  /// Updates track metadata after correction.
  core::Result<Track> BasicTrackManager::on_correction(
      const Track &track,
      const filters::GaussianEstimate &corrected_estimate,
      const core::Measurement &measurement) const
  {
    const core::Status estimate_status =
        filters::validate_estimate(corrected_estimate);
    if (!estimate_status.ok())
    {
      return estimate_status;
    }

    Track updated = track;
    updated.estimate = corrected_estimate;
    updated.hit_count += 1U;
    updated.consecutive_misses = 0U;
    updated.source_sensor_id = measurement.sensor_id;
    if (updated.hit_count >= confirmation_hits_)
    {
      updated.lifecycle = TrackLifecycle::kConfirmed;
    }
    return updated;
  }

  /// Updates track metadata after a missed detection.
  core::Result<Track> BasicTrackManager::on_missed_detection(
      const Track &track) const
  {
    Track updated = track;
    updated.miss_count += 1U;
    updated.consecutive_misses += 1U;
    if (updated.consecutive_misses >= max_consecutive_misses_)
    {
      updated.lifecycle = TrackLifecycle::kDeleted;
    }
    return updated;
  }

  /// Returns whether the track should be removed.
  bool BasicTrackManager::should_remove(const Track &track) const noexcept
  {
    return track.lifecycle == TrackLifecycle::kDeleted;
  }

  /// Returns the component name.
  std::string_view BasicTrackManager::name() const noexcept
  {
    return "basic_manager";
  }

} // namespace ros_tracker::tracking
