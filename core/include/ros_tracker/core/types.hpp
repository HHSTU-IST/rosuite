#pragma once

#include <string>

#include "ros_tracker/core/math/linear_algebra.hpp"

namespace ros_tracker::core {

struct State {
  Vector value;
  Scalar timestamp {0.0};
  std::string frame_id;

  [[nodiscard]] Index dimension() const noexcept {
    return value.size();
  }
};

struct Measurement {
  Vector value;
  Scalar timestamp {0.0};
  std::string sensor_id;
  std::string frame_id;

  [[nodiscard]] Index dimension() const noexcept {
    return value.size();
  }
};

struct ControlInput {
  Vector value;
  Scalar timestamp {0.0};

  [[nodiscard]] Index dimension() const noexcept {
    return value.size();
  }
};

}  // namespace ros_tracker::core
