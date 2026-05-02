#pragma once

#include <memory>

#include "ros_tracker/core/status.hpp"
#include "ros_tracker/models/measurement_model.hpp"
#include "ros_tracker/models/motion_model.hpp"

namespace ros_tracker::models {

struct DynamicSystemModel {
  std::shared_ptr<MotionModel> motion;
  std::shared_ptr<ProcessNoiseModel> process_noise;

  [[nodiscard]] core::Status validate() const {
    if (!motion) {
      return core::Status::invalid_argument(
          "DynamicSystemModel requires a motion model.");
    }

    if (!process_noise) {
      return core::Status::invalid_argument(
          "DynamicSystemModel requires a process noise model.");
    }

    return core::Status::ok_status();
  }
};

struct SensorModel {
  std::shared_ptr<MeasurementModel> measurement;
  std::shared_ptr<MeasurementNoiseModel> measurement_noise;

  [[nodiscard]] core::Status validate() const {
    if (!measurement) {
      return core::Status::invalid_argument(
          "SensorModel requires a measurement model.");
    }

    if (!measurement_noise) {
      return core::Status::invalid_argument(
          "SensorModel requires a measurement noise model.");
    }

    return core::Status::ok_status();
  }
};

}  // namespace ros_tracker::models
