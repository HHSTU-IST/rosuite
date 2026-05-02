#pragma once

#include "ros_tracker/models/measurement_model.hpp"

namespace ros_tracker::models {

class LinearMeasurementModel final : public LinearizableMeasurementModel {
 public:
  explicit LinearMeasurementModel(
      Matrix observation_matrix,
      Vector bias = Vector {})
      : observation_matrix_(std::move(observation_matrix)),
        bias_(std::move(bias)) {}

  [[nodiscard]] Result<MeasurementResult> measure(
      const MeasurementRequest& request) const override {
    if (request.state.dimension() != observation_matrix_.cols()) {
      return Status::dimension_mismatch(
          "LinearMeasurementModel state dimension does not match observation matrix.");
    }

    if (bias_.size() != 0 && bias_.size() != observation_matrix_.rows()) {
      return Status::dimension_mismatch(
          "LinearMeasurementModel bias dimension must match measurement dimension.");
    }

    MeasurementResult result;
    result.measurement.timestamp = request.context.timestamp;
    result.measurement.frame_id = request.context.frame_id.empty()
                                      ? request.state.frame_id
                                      : request.context.frame_id;
    result.measurement.sensor_id = request.sensor_id;
    result.measurement.value = observation_matrix_ * request.state.value;
    if (bias_.size() != 0) {
      result.measurement.value += bias_;
    }

    return result;
  }

  [[nodiscard]] Result<Matrix> state_jacobian(
      const MeasurementRequest& request) const override {
    if (request.state.dimension() != observation_matrix_.cols()) {
      return Status::dimension_mismatch(
          "LinearMeasurementModel state dimension does not match observation matrix.");
    }

    return observation_matrix_;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "linear_measurement";
  }

 private:
  Matrix observation_matrix_;
  Vector bias_;
};

}  // namespace ros_tracker::models
