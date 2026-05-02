#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "ros_tracker/core/result.hpp"
#include "ros_tracker/core/status.hpp"
#include "ros_tracker/core/types.hpp"

namespace ros_tracker::models {

using core::ControlInput;
using core::Covariance;
using core::Matrix;
using core::Measurement;
using core::Result;
using core::State;
using core::Status;
using core::Vector;

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

class MeasurementModel {
 public:
  virtual ~MeasurementModel() = default;

  [[nodiscard]] virtual Result<MeasurementResult> measure(
      const MeasurementRequest& request) const = 0;

  [[nodiscard]] virtual Result<Matrix> state_jacobian(
      const MeasurementRequest& /*request*/) const {
    return Status::unimplemented(
        "This measurement model does not provide a state Jacobian.");
  }

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class ProcessNoiseModel {
 public:
  virtual ~ProcessNoiseModel() = default;

  [[nodiscard]] virtual Result<Covariance> covariance(
      const MotionRequest& request) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class MeasurementNoiseModel {
 public:
  virtual ~MeasurementNoiseModel() = default;

  [[nodiscard]] virtual Result<Covariance> covariance(
      const MeasurementRequest& request) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

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
