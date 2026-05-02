#pragma once

#include <cmath>

#include "ros_tracker/models/motion_model.hpp"

namespace ros_tracker::models {

class SingerMotionModel final : public LinearizableMotionModel {
 public:
  explicit SingerMotionModel(const core::Scalar maneuver_decay)
      : maneuver_decay_(maneuver_decay) {}

  [[nodiscard]] Result<TransitionResult> propagate(
      const MotionRequest& request) const override {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok()) {
      return dt_status;
    }

    if (maneuver_decay_ <= 0.0) {
      return Status::invalid_argument(
          "SingerMotionModel requires a positive maneuver decay rate.");
    }

    if (request.state.dimension() != 6) {
      return Status::dimension_mismatch(
          "SingerMotionModel expects a 6D state [px, py, vx, vy, ax, ay].");
    }

    const auto jacobian = state_jacobian(request);
    if (!jacobian.ok()) {
      return jacobian.status();
    }

    TransitionResult result;
    result.state = request.state;
    result.state.timestamp = request.context.timestamp;
    result.state.frame_id = request.context.frame_id.empty()
                                ? request.state.frame_id
                                : request.context.frame_id;
    result.state.value = jacobian.value() * request.state.value;
    return result;
  }

  [[nodiscard]] Result<Matrix> state_jacobian(
      const MotionRequest& request) const override {
    if (maneuver_decay_ <= 0.0) {
      return Status::invalid_argument(
          "SingerMotionModel requires a positive maneuver decay rate.");
    }

    if (request.state.dimension() != 6) {
      return Status::dimension_mismatch(
          "SingerMotionModel expects a 6D state [px, py, vx, vy, ax, ay].");
    }

    const core::Scalar dt = request.context.dt;
    const core::Scalar alpha = maneuver_decay_;
    const core::Scalar exp_term = std::exp(-alpha * dt);
    const core::Scalar velocity_gain = (1.0 - exp_term) / alpha;
    const core::Scalar position_gain =
        (alpha * dt - 1.0 + exp_term) / (alpha * alpha);

    Matrix f = Matrix::Identity(6, 6);
    f(0, 2) = dt;
    f(1, 3) = dt;
    f(0, 4) = position_gain;
    f(1, 5) = position_gain;
    f(2, 4) = velocity_gain;
    f(3, 5) = velocity_gain;
    f(4, 4) = exp_term;
    f(5, 5) = exp_term;
    return f;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "singer";
  }

 private:
  core::Scalar maneuver_decay_;
};

}  // namespace ros_tracker::models
