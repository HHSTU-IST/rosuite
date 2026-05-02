#pragma once

#include "ros_tracker/models/motion_model.hpp"

namespace ros_tracker::models {

class ConstantVelocityMotionModel final : public MotionModel {
 public:
  [[nodiscard]] Result<TransitionResult> propagate(
      const MotionRequest& request) const override {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok()) {
      return dt_status;
    }

    if (request.state.dimension() != 4) {
      return Status::dimension_mismatch(
          "ConstantVelocityMotionModel expects a 4D state [px, py, vx, vy].");
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
    if (request.state.dimension() != 4) {
      return Status::dimension_mismatch(
          "ConstantVelocityMotionModel expects a 4D state [px, py, vx, vy].");
    }

    Matrix f = Matrix::Identity(4, 4);
    f(0, 2) = request.context.dt;
    f(1, 3) = request.context.dt;
    return f;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "const_vel";
  }
};

}  // namespace ros_tracker::models
