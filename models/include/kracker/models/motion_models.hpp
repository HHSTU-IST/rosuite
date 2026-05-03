#pragma once

#include <cmath>

#include "kracker/models/base.hpp"

namespace kracker::models
{
  class ConstantVelocityMotionModel final : public MotionModel
  {
  public:
    /// Propagates the state for one time step.
    [[nodiscard]] Result<TransitionResult> propagate(
        const MotionRequest &request) const override;

    /// Computes the state Jacobian for the given request.
    [[nodiscard]] Result<Matrix> state_jacobian(
        const MotionRequest &request) const override;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override;
  };

  class ConstantAccelerationMotionModel final : public MotionModel
  {
  public:
    /// Propagates the state for one time step.
    [[nodiscard]] Result<TransitionResult> propagate(
        const MotionRequest &request) const override;

    /// Computes the state Jacobian for the given request.
    [[nodiscard]] Result<Matrix> state_jacobian(
        const MotionRequest &request) const override;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override;
  };

  class CoordinatedTurnMotionModel final : public MotionModel
  {
  public:
    /// Constructs CoordinatedTurnMotionModel.
    explicit CoordinatedTurnMotionModel(const core::Scalar turn_rate_epsilon = 1e-6);

    /// Propagates the state for one time step.
    [[nodiscard]] Result<TransitionResult> propagate(
        const MotionRequest &request) const override;

    /// Computes the state Jacobian for the given request.
    [[nodiscard]] Result<Matrix> state_jacobian(
        const MotionRequest &request) const override;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override;

  private:
    core::Scalar turn_rate_epsilon_;
  };

  class SingerMotionModel final : public MotionModel
  {
  public:
    /// Constructs SingerMotionModel.
    explicit SingerMotionModel(const core::Scalar maneuver_decay);

    /// Propagates the state for one time step.
    [[nodiscard]] Result<TransitionResult> propagate(
        const MotionRequest &request) const override;

    /// Computes the state Jacobian for the given request.
    [[nodiscard]] Result<Matrix> state_jacobian(
        const MotionRequest &request) const override;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override;

  private:
    core::Scalar maneuver_decay_;
  };

} // namespace kracker::models
