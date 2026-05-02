#pragma once

#include <optional>
#include <string_view>

#include "ros_tracker/core/result.hpp"
#include "ros_tracker/core/types.hpp"
#include "ros_tracker/models/model_context.hpp"

namespace ros_tracker::models {

using core::Covariance;
using core::Matrix;
using core::Result;
using core::Status;
using core::Vector;

class MotionModel {
 public:
  virtual ~MotionModel() = default;

  [[nodiscard]] virtual Result<TransitionResult> propagate(
      const MotionRequest& request) const = 0;

  [[nodiscard]] virtual Result<Matrix> state_jacobian(
      const MotionRequest& /*request*/) const {
    return Status::unimplemented(
        "This motion model does not provide a state Jacobian.");
  }

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

 protected:
  [[nodiscard]] static Status validate_dt(const MotionRequest& request) {
    if (request.context.dt < 0.0) {
      return Status::invalid_argument("Motion model dt must be non-negative.");
    }

    return Status::ok_status();
  }
};

class ProcessNoiseModel {
 public:
  virtual ~ProcessNoiseModel() = default;

  [[nodiscard]] virtual Result<Covariance> covariance(
      const MotionRequest& request) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

}  // namespace ros_tracker::models
