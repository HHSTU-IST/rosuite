#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ros_tracker/apps/offline_examples.hpp"
#include "ros_tracker/filters/kalman_filters.hpp"
#include "ros_tracker/models/factories.hpp"
#include "ros_tracker/tracking/association.hpp"
#include "ros_tracker/tracking/management.hpp"
#include "ros_tracker/tracking/tracker.hpp"

namespace py = pybind11;

namespace
{
using namespace ros_tracker;

template <typename T>
T unwrap_result(core::Result<T> result)
{
  if (!result.ok())
  {
    throw std::runtime_error(result.status().message);
  }

  return std::move(result.value());
}

std::string lifecycle_to_string(const tracking::TrackLifecycle lifecycle)
{
  switch (lifecycle)
  {
  case tracking::TrackLifecycle::kTentative:
    return "tentative";
  case tracking::TrackLifecycle::kConfirmed:
    return "confirmed";
  case tracking::TrackLifecycle::kDeleted:
    return "deleted";
  }

  throw std::runtime_error("Unknown track lifecycle value.");
}

struct TrackSnapshot
{
  std::size_t id{0U};
  core::Vector state;
  core::Matrix covariance;
  std::string lifecycle;
  std::size_t age{0U};
  std::size_t hit_count{0U};
  std::size_t miss_count{0U};
  std::size_t consecutive_misses{0U};
  std::string source_sensor_id;
  core::Scalar timestamp{0.0};
  std::string frame_id;
};

struct TrackerConfig
{
  core::Scalar process_noise{0.01};
  core::Scalar measurement_noise{0.25};
  core::Scalar initial_covariance{4.0};
  core::Scalar gating_threshold{16.0};
  std::size_t confirmation_hits{1U};
  std::size_t max_consecutive_misses{2U};
  std::string frame_id{"map"};
  std::string sensor_id{"python_sensor"};
};

TrackSnapshot make_track_snapshot(const tracking::Track &track)
{
  TrackSnapshot snapshot;
  snapshot.id = track.id;
  snapshot.state = track.estimate.state.value;
  snapshot.covariance = track.estimate.covariance;
  snapshot.lifecycle = lifecycle_to_string(track.lifecycle);
  snapshot.age = track.age;
  snapshot.hit_count = track.hit_count;
  snapshot.miss_count = track.miss_count;
  snapshot.consecutive_misses = track.consecutive_misses;
  snapshot.source_sensor_id = track.source_sensor_id;
  snapshot.timestamp = track.estimate.state.timestamp;
  snapshot.frame_id = track.estimate.state.frame_id;
  return snapshot;
}

std::vector<TrackSnapshot> make_track_snapshots(
    const std::vector<tracking::Track> &tracks)
{
  std::vector<TrackSnapshot> snapshots;
  snapshots.reserve(tracks.size());

  for (const tracking::Track &track : tracks)
  {
    snapshots.push_back(make_track_snapshot(track));
  }

  return snapshots;
}

class PythonTracker
{
public:
  explicit PythonTracker(TrackerConfig config)
      : config_(std::move(config))
  {
    reset();
  }

  void reset()
  {
    auto filter = std::make_shared<filters::KalmanFilter>();
    auto association =
        std::make_shared<tracking::NearestNeighborAssociationStrategy>(
            config_.gating_threshold);
    auto manager = std::make_shared<tracking::BasicTrackManager>(
        4,
        config_.initial_covariance * core::Matrix::Identity(4, 4),
        std::vector<core::Index>{0, 1},
        config_.confirmation_hits,
        config_.max_consecutive_misses);

    tracker_ = std::make_unique<tracking::MultiTargetTracker>(
        filter,
        models::make_constant_velocity_system(
            config_.process_noise * core::Matrix::Identity(4, 4)),
        models::make_position_sensor(
            config_.measurement_noise * core::Matrix::Identity(2, 2)),
        association,
        manager);
  }

  std::vector<TrackSnapshot> step(
      const py::iterable &measurement_values,
      const core::Scalar timestamp,
      const core::Scalar dt,
      std::string frame_id,
      std::string sensor_id)
  {
    std::vector<core::Measurement> measurements;

    const std::string resolved_frame_id =
        frame_id.empty() ? config_.frame_id : std::move(frame_id);
    const std::string resolved_sensor_id =
        sensor_id.empty() ? config_.sensor_id : std::move(sensor_id);

    for (const py::handle value : measurement_values)
    {
      measurements.push_back(core::Measurement{
          py::cast<core::Vector>(value),
          timestamp,
          resolved_sensor_id,
          resolved_frame_id,
      });
    }

    return step_measurements_impl(
        std::move(measurements),
        timestamp,
        dt,
        resolved_frame_id,
        resolved_sensor_id);
  }

  std::vector<TrackSnapshot> step_measurements(
      std::vector<core::Measurement> measurements,
      const core::Scalar timestamp,
      const core::Scalar dt,
      std::string frame_id,
      std::string sensor_id)
  {
    const std::string resolved_frame_id =
        frame_id.empty() ? config_.frame_id : std::move(frame_id);
    const std::string resolved_sensor_id =
        sensor_id.empty() ? config_.sensor_id : std::move(sensor_id);

    for (core::Measurement &measurement : measurements)
    {
      measurement.timestamp = timestamp;
      if (measurement.frame_id.empty())
      {
        measurement.frame_id = resolved_frame_id;
      }
      if (measurement.sensor_id.empty())
      {
        measurement.sensor_id = resolved_sensor_id;
      }
    }

    return step_measurements_impl(
        std::move(measurements),
        timestamp,
        dt,
        resolved_frame_id,
        resolved_sensor_id);
  }

  std::vector<TrackSnapshot> tracks() const
  {
    return make_track_snapshots(tracker_->tracks());
  }

private:
  std::vector<TrackSnapshot> step_measurements_impl(
      std::vector<core::Measurement> measurements,
      const core::Scalar timestamp,
      const core::Scalar dt,
      const std::string &frame_id,
      const std::string & /*sensor_id*/)
  {
    const auto tracks = tracker_->step(
        measurements,
        models::ModelContext{dt, timestamp, frame_id});
    return make_track_snapshots(unwrap_result(std::move(tracks)));
  }

  TrackerConfig config_;
  std::unique_ptr<tracking::MultiTargetTracker> tracker_;
};

} // namespace

PYBIND11_MODULE(_ros_tracker, module)
{
  module.doc() = "Python bindings for ros_tracker.";

  py::class_<core::Measurement>(module, "Measurement")
      .def(
          py::init<core::Vector, core::Scalar, std::string, std::string>(),
          py::arg("value"),
          py::arg("timestamp") = 0.0,
          py::arg("sensor_id") = "",
          py::arg("frame_id") = "")
      .def_readwrite("value", &core::Measurement::value)
      .def_readwrite("timestamp", &core::Measurement::timestamp)
      .def_readwrite("sensor_id", &core::Measurement::sensor_id)
      .def_readwrite("frame_id", &core::Measurement::frame_id);

  py::class_<TrackSnapshot>(module, "Track")
      .def_readonly("id", &TrackSnapshot::id)
      .def_readonly("state", &TrackSnapshot::state)
      .def_readonly("covariance", &TrackSnapshot::covariance)
      .def_readonly("lifecycle", &TrackSnapshot::lifecycle)
      .def_readonly("age", &TrackSnapshot::age)
      .def_readonly("hit_count", &TrackSnapshot::hit_count)
      .def_readonly("miss_count", &TrackSnapshot::miss_count)
      .def_readonly(
          "consecutive_misses",
          &TrackSnapshot::consecutive_misses)
      .def_readonly("source_sensor_id", &TrackSnapshot::source_sensor_id)
      .def_readonly("timestamp", &TrackSnapshot::timestamp)
      .def_readonly("frame_id", &TrackSnapshot::frame_id);

  py::class_<PythonTracker>(module, "Tracker")
      .def(
          py::init([](core::Scalar process_noise,
                      core::Scalar measurement_noise,
                      core::Scalar initial_covariance,
                      core::Scalar gating_threshold,
                      std::size_t confirmation_hits,
                      std::size_t max_consecutive_misses,
                      std::string frame_id,
                      std::string sensor_id) {
            return PythonTracker(TrackerConfig{
                process_noise,
                measurement_noise,
                initial_covariance,
                gating_threshold,
                confirmation_hits,
                max_consecutive_misses,
                std::move(frame_id),
                std::move(sensor_id),
            });
          }),
          py::arg("process_noise") = 0.01,
          py::arg("measurement_noise") = 0.25,
          py::arg("initial_covariance") = 4.0,
          py::arg("gating_threshold") = 16.0,
          py::arg("confirmation_hits") = 1U,
          py::arg("max_consecutive_misses") = 2U,
          py::arg("frame_id") = "map",
          py::arg("sensor_id") = "python_sensor")
      .def(
          "step",
          &PythonTracker::step,
          py::arg("measurements"),
          py::arg("timestamp"),
          py::arg("dt"),
          py::arg("frame_id") = "",
          py::arg("sensor_id") = "")
      .def(
          "step_measurements",
          &PythonTracker::step_measurements,
          py::arg("measurements"),
          py::arg("timestamp"),
          py::arg("dt"),
          py::arg("frame_id") = "",
          py::arg("sensor_id") = "")
      .def("tracks", &PythonTracker::tracks)
      .def("reset", &PythonTracker::reset);

  module.def(
      "run_single_target_kalman_example",
      [](const std::uint64_t seed) {
        const auto summary =
            unwrap_result(apps::offline::run_single_target_kalman_example(seed));

        py::dict result;
        result["truth_frames"] = summary.metrics.truth_frames;
        result["estimated_frames"] = summary.metrics.estimated_frames;
        result["position_rmse"] = summary.metrics.position_rmse;
        result["velocity_rmse"] = summary.metrics.velocity_rmse;
        result["final_track_count"] =
            summary.tracker_frames.empty()
                ? std::size_t{0U}
                : summary.tracker_frames.back().tracks.size();
        return result;
      },
      py::arg("seed") = 0U);
}
