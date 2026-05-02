#pragma once

#include <string_view>

#include "ros_tracker/core/result.hpp"
#include "ros_tracker/core/types.hpp"
#include "ros_tracker/models/model_context.hpp"

namespace ros_tracker::models {

using core::Covariance;
using core::Matrix;
using core::Result;
using core::Status;

class MeasurementModel {
 public:
  virtual ~MeasurementModel() = default;

  [[nodiscard]] virtual Result<MeasurementResult> measure(
      const MeasurementRequest& request) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class LinearizableMeasurementModel : public MeasurementModel {
 public:
  [[nodiscard]] virtual Result<Matrix> state_jacobian(
      const MeasurementRequest& request) const = 0;
};

class MeasurementNoiseModel {
 public:
  virtual ~MeasurementNoiseModel() = default;

  [[nodiscard]] virtual Result<Covariance> covariance(
      const MeasurementRequest& request) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

}  // namespace ros_tracker::models
