#pragma once

#include "ros_tracker/models/motion_model.hpp"

namespace ros_tracker::models {

class ConstantAccelerationMotionModel final : public LinearizableMotionModel {
 public:
  [[nodiscard]] Result<TransitionResult> propagate(
      const MotionRequest& request) const override {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok()) {
      return dt_status;
    }

    if (request.state.dimension() != 6) {
      return Status::dimension_mismatch(
          "ConstantAccelerationMotionModel expects a 6D state [px, py, vx, vy, ax, ay].");
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
    if (request.state.dimension() != 6) {
      return Status::dimension_mismatch(
          "ConstantAccelerationMotionModel expects a 6D state [px, py, vx, vy, ax, ay].");
    }

    const core::Scalar dt = request.context.dt;
    const core::Scalar dt2 = 0.5 * dt * dt;

    Matrix f = Matrix::Identity(6, 6);
    f(0, 2) = dt;
    f(1, 3) = dt;
    f(0, 4) = dt2;
    f(1, 5) = dt2;
    f(2, 4) = dt;
    f(3, 5) = dt;
    return f;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "const_acc";
  }
};

}  // namespace ros_tracker::models
