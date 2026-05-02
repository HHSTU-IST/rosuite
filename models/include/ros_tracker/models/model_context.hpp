#pragma once

#include <optional>
#include <string>

#include "ros_tracker/core/types.hpp"

namespace ros_tracker::models {

using core::ControlInput;
using core::Measurement;
using core::State;

struct ModelContext {
  core::Scalar dt {0.0};
  core::Scalar timestamp {0.0};
  std::string frame_id;
};

struct MotionRequest {
  State state;
  std::optional<ControlInput> control;
  ModelContext context;
};

struct MeasurementRequest {
  State state;
  ModelContext context;
  std::string sensor_id;
};

struct TransitionResult {
  State state;
};

struct MeasurementResult {
  Measurement measurement;
};

}  // namespace ros_tracker::models
