#pragma once

#include <cmath>

#include "ros_tracker/models/motion_model.hpp"

namespace ros_tracker::models {

class CoordinatedTurnMotionModel final : public LinearizableMotionModel {
 public:
  explicit CoordinatedTurnMotionModel(const core::Scalar turn_rate_epsilon = 1e-6)
      : turn_rate_epsilon_(turn_rate_epsilon) {}

  [[nodiscard]] Result<TransitionResult> propagate(
      const MotionRequest& request) const override {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok()) {
      return dt_status;
    }

    if (request.state.dimension() != 5) {
      return Status::dimension_mismatch(
          "CoordinatedTurnMotionModel expects a 5D state [px, py, speed, heading, turn_rate].");
    }

    const Vector& x = request.state.value;
    const core::Scalar dt = request.context.dt;
    const core::Scalar px = x[0];
    const core::Scalar py = x[1];
    const core::Scalar speed = x[2];
    const core::Scalar heading = x[3];
    const core::Scalar turn_rate = x[4];

    Vector next = x;
    if (std::abs(turn_rate) < turn_rate_epsilon_) {
      next[0] = px + speed * dt * std::cos(heading);
      next[1] = py + speed * dt * std::sin(heading);
    } else {
      const core::Scalar heading_next = heading + turn_rate * dt;
      next[0] = px + speed / turn_rate *
                         (std::sin(heading_next) - std::sin(heading));
      next[1] = py + speed / turn_rate *
                         (-std::cos(heading_next) + std::cos(heading));
      next[3] = heading_next;
    }

    TransitionResult result;
    result.state = request.state;
    result.state.value = next;
    result.state.timestamp = request.context.timestamp;
    result.state.frame_id = request.context.frame_id.empty()
                                ? request.state.frame_id
                                : request.context.frame_id;
    return result;
  }

  [[nodiscard]] Result<Matrix> state_jacobian(
      const MotionRequest& request) const override {
    if (request.state.dimension() != 5) {
      return Status::dimension_mismatch(
          "CoordinatedTurnMotionModel expects a 5D state [px, py, speed, heading, turn_rate].");
    }

    const Vector& x = request.state.value;
    const core::Scalar dt = request.context.dt;
    const core::Scalar speed = x[2];
    const core::Scalar heading = x[3];
    const core::Scalar turn_rate = x[4];

    Matrix f = Matrix::Identity(5, 5);
    if (std::abs(turn_rate) < turn_rate_epsilon_) {
      f(0, 2) = dt * std::cos(heading);
      f(0, 3) = -speed * dt * std::sin(heading);
      f(1, 2) = dt * std::sin(heading);
      f(1, 3) = speed * dt * std::cos(heading);
      f(3, 4) = dt;
      return f;
    }

    const core::Scalar heading_next = heading + turn_rate * dt;
    const core::Scalar sin_heading = std::sin(heading);
    const core::Scalar cos_heading = std::cos(heading);
    const core::Scalar sin_heading_next = std::sin(heading_next);
    const core::Scalar cos_heading_next = std::cos(heading_next);

    f(0, 2) = (sin_heading_next - sin_heading) / turn_rate;
    f(0, 3) = speed / turn_rate * (cos_heading_next - cos_heading);
    f(0, 4) = speed *
              ((dt * turn_rate * cos_heading_next) -
               (sin_heading_next - sin_heading)) /
              (turn_rate * turn_rate);

    f(1, 2) = (-cos_heading_next + cos_heading) / turn_rate;
    f(1, 3) = speed / turn_rate * (sin_heading_next - sin_heading);
    f(1, 4) = speed *
              ((dt * turn_rate * sin_heading_next) -
               (-cos_heading_next + cos_heading)) /
              (turn_rate * turn_rate);

    f(3, 4) = dt;
    return f;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "coord_turn";
  }

 private:
  core::Scalar turn_rate_epsilon_;
};

}  // namespace ros_tracker::models
