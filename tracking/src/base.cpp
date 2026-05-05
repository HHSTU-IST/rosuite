#include "rosuite/tracking/base.hpp"

#include <cmath>
#include <utility>

namespace rosuite::tracking
{
  /// Constructs StaticTrackEstimatorModelHandle.
  StaticTrackEstimatorModelHandle::StaticTrackEstimatorModelHandle(
      std::shared_ptr<const filters::FilterBase> filter,
      models::DynamicSystemModel system_model,
      models::SensorModel sensor_model,
      std::string name)
      : filter_(std::move(filter)),
        system_model_(std::move(system_model)),
        sensor_model_(std::move(sensor_model)),
        name_(std::move(name)) {}

  /// Validates the current configuration.
  core::Status StaticTrackEstimatorModelHandle::validate() const
  {
    if (!filter_)
    {
      return core::Status::invalid_argument(
          "StaticTrackEstimatorModelHandle requires a filter.");
    }

    const core::Status system_status = system_model_.validate();
    if (!system_status.ok())
    {
      return system_status;
    }

    return sensor_model_.validate();
  }

  /// Predicts the next estimate.
  core::Result<filters::GaussianEstimate>
  StaticTrackEstimatorModelHandle::predict(
      const filters::GaussianEstimate &estimate,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control) const
  {
    const core::Status status = validate();
    if (!status.ok())
    {
      return status;
    }

    return filter_->predict(estimate, system_model_, context, std::move(control));
  }

  /// Corrects an estimate with a measurement.
  core::Result<filters::GaussianEstimate>
  StaticTrackEstimatorModelHandle::correct(
      const filters::GaussianEstimate &estimate,
      const core::Measurement &measurement,
      const models::ModelContext &context) const
  {
    const core::Status status = validate();
    if (!status.ok())
    {
      return status;
    }

    return filter_->correct(estimate, sensor_model_, measurement, context);
  }

  /// Returns the sensor model used for association.
  const models::SensorModel &
  StaticTrackEstimatorModelHandle::association_sensor_model() const noexcept
  {
    return sensor_model_;
  }

  /// Returns the component name.
  std::string_view StaticTrackEstimatorModelHandle::name() const noexcept
  {
    return name_;
  }

  /// Summarizes shared metadata from a measurement batch.
  core::Result<MeasurementBatchSummary> summarize_measurement_batch(
      const std::vector<core::Measurement> &measurements)
  {
    MeasurementBatchSummary summary;
    if (measurements.empty())
    {
      return summary;
    }

    summary.timestamp = measurements.front().timestamp;
    summary.frame_id = measurements.front().frame_id;

    for (std::size_t i = 1; i < measurements.size(); ++i)
    {
      if (std::abs(measurements[i].timestamp - summary.timestamp) > 1e-9)
      {
        return core::Status::invalid_argument(
            "All measurements in a batch must share the same timestamp.");
      }

      if (!measurements[i].frame_id.empty())
      {
        if (summary.frame_id.empty())
        {
          summary.frame_id = measurements[i].frame_id;
        }
        else if (measurements[i].frame_id != summary.frame_id)
        {
          return core::Status::invalid_argument(
              "All measurements in a batch must share the same frame_id.");
        }
      }
    }

    return summary;
  }

  /// Validates a track instance.
  core::Status validate_track(const Track &track)
  {
    if (track.age == 0U)
    {
      return core::Status::invalid_argument(
          "Track age must be at least one.");
    }

    if (!track.handle)
    {
      return core::Status::invalid_argument(
          "Track requires a per-track estimator/model handle.");
    }

    const core::Status handle_status = track.handle->validate();
    if (!handle_status.ok())
    {
      return handle_status;
    }

    return filters::validate_estimate(track.estimate);
  }

} // namespace rosuite::tracking
