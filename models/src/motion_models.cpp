#include "ros_tracker/models/motion_models.hpp"

namespace ros_tracker::models
{
  Result<TransitionResult> ConstantVelocityMotionModel::propagate(
      const MotionRequest &request) const
  {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok())
    {
      return dt_status;
    }

    if (request.state.dimension() != 4)
    {
      return Status::dimension_mismatch(
          "ConstantVelocityMotionModel expects a 4D state [px, py, vx, vy].");
    }

    const auto jacobian = state_jacobian(request);
    if (!jacobian.ok())
    {
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

  Result<Matrix> ConstantVelocityMotionModel::state_jacobian(
      const MotionRequest &request) const
  {
    if (request.state.dimension() != 4)
    {
      return Status::dimension_mismatch(
          "ConstantVelocityMotionModel expects a 4D state [px, py, vx, vy].");
    }

    Matrix f = Matrix::Identity(4, 4);
    f(0, 2) = request.context.dt;
    f(1, 3) = request.context.dt;
    return f;
  }

  std::string_view ConstantVelocityMotionModel::name() const noexcept
  {
    return "const_vel";
  }

  Result<TransitionResult> ConstantAccelerationMotionModel::propagate(
      const MotionRequest &request) const
  {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok())
    {
      return dt_status;
    }

    if (request.state.dimension() != 6)
    {
      return Status::dimension_mismatch(
          "ConstantAccelerationMotionModel expects a 6D state [px, py, vx, vy, ax, ay].");
    }

    const auto jacobian = state_jacobian(request);
    if (!jacobian.ok())
    {
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

  Result<Matrix> ConstantAccelerationMotionModel::state_jacobian(
      const MotionRequest &request) const
  {
    if (request.state.dimension() != 6)
    {
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

  std::string_view ConstantAccelerationMotionModel::name() const noexcept
  {
    return "const_acc";
  }

  CoordinatedTurnMotionModel::CoordinatedTurnMotionModel(
      const core::Scalar turn_rate_epsilon)
      : turn_rate_epsilon_(turn_rate_epsilon) {}

  Result<TransitionResult> CoordinatedTurnMotionModel::propagate(
      const MotionRequest &request) const
  {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok())
    {
      return dt_status;
    }

    if (request.state.dimension() != 5)
    {
      return Status::dimension_mismatch(
          "CoordinatedTurnMotionModel expects a 5D state [px, py, speed, heading, turn_rate].");
    }

    const Vector &x = request.state.value;
    const core::Scalar dt = request.context.dt;
    const core::Scalar px = x[0];
    const core::Scalar py = x[1];
    const core::Scalar speed = x[2];
    const core::Scalar heading = x[3];
    const core::Scalar turn_rate = x[4];

    Vector next = x;
    if (std::abs(turn_rate) < turn_rate_epsilon_)
    {
      next[0] = px + speed * dt * std::cos(heading);
      next[1] = py + speed * dt * std::sin(heading);
    }
    else
    {
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

  Result<Matrix> CoordinatedTurnMotionModel::state_jacobian(
      const MotionRequest &request) const
  {
    if (request.state.dimension() != 5)
    {
      return Status::dimension_mismatch(
          "CoordinatedTurnMotionModel expects a 5D state [px, py, speed, heading, turn_rate].");
    }

    const Vector &x = request.state.value;
    const core::Scalar dt = request.context.dt;
    const core::Scalar speed = x[2];
    const core::Scalar heading = x[3];
    const core::Scalar turn_rate = x[4];

    Matrix f = Matrix::Identity(5, 5);
    if (std::abs(turn_rate) < turn_rate_epsilon_)
    {
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

  std::string_view CoordinatedTurnMotionModel::name() const noexcept
  {
    return "coord_turn";
  }

  SingerMotionModel::SingerMotionModel(const core::Scalar maneuver_decay)
      : maneuver_decay_(maneuver_decay) {}

  Result<TransitionResult> SingerMotionModel::propagate(
      const MotionRequest &request) const
  {
    const Status dt_status = validate_dt(request);
    if (!dt_status.ok())
    {
      return dt_status;
    }

    if (maneuver_decay_ <= 0.0)
    {
      return Status::invalid_argument(
          "SingerMotionModel requires a positive maneuver decay rate.");
    }

    if (request.state.dimension() != 6)
    {
      return Status::dimension_mismatch(
          "SingerMotionModel expects a 6D state [px, py, vx, vy, ax, ay].");
    }

    const auto jacobian = state_jacobian(request);
    if (!jacobian.ok())
    {
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

  Result<Matrix> SingerMotionModel::state_jacobian(
      const MotionRequest &request) const
  {
    if (maneuver_decay_ <= 0.0)
    {
      return Status::invalid_argument(
          "SingerMotionModel requires a positive maneuver decay rate.");
    }

    if (request.state.dimension() != 6)
    {
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

  std::string_view SingerMotionModel::name() const noexcept
  {
    return "singer";
  }

} // namespace ros_tracker::models
