#pragma once

#include <utility>

#include "ros_tracker/models/measurement_model.hpp"
#include "ros_tracker/models/motion_model.hpp"

namespace ros_tracker::models {

class ConstantGaussianProcessNoise final : public ProcessNoiseModel {
 public:
  explicit ConstantGaussianProcessNoise(Covariance covariance)
      : covariance_(std::move(covariance)) {}

  [[nodiscard]] Result<Covariance> covariance(
      const MotionRequest& /*request*/) const override {
    const Status status = core::validate_covariance(covariance_);
    if (!status.ok()) {
      return status;
    }

    return covariance_;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "gaussian_process_noise";
  }

 private:
  Covariance covariance_;
};

class ConstantGaussianMeasurementNoise final : public MeasurementNoiseModel {
 public:
  explicit ConstantGaussianMeasurementNoise(Covariance covariance)
      : covariance_(std::move(covariance)) {}

  [[nodiscard]] Result<Covariance> covariance(
      const MeasurementRequest& /*request*/) const override {
    const Status status = core::validate_covariance(covariance_);
    if (!status.ok()) {
      return status;
    }

    return covariance_;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "gaussian_measurement_noise";
  }

 private:
  Covariance covariance_;
};

}  // namespace ros_tracker::models
