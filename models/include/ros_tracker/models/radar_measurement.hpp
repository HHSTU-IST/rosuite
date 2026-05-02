#pragma once

#include <cmath>

#include "ros_tracker/models/measurement_model.hpp"

namespace ros_tracker::models {

class RadarMeasurementModel final : public MeasurementModel {
 public:
  explicit RadarMeasurementModel(const core::Scalar range_epsilon = 1e-9)
      : range_epsilon_(range_epsilon) {}

  [[nodiscard]] Result<MeasurementResult> measure(
      const MeasurementRequest& request) const override {
    if (request.state.dimension() != 4) {
      return Status::dimension_mismatch(
          "RadarMeasurementModel expects a 4D state [px, py, vx, vy].");
    }

    const Vector& x = request.state.value;
    const core::Scalar px = x[0];
    const core::Scalar py = x[1];
    const core::Scalar vx = x[2];
    const core::Scalar vy = x[3];
    const core::Scalar range_sq = px * px + py * py;
    const core::Scalar range = std::sqrt(range_sq);

    if (range < range_epsilon_) {
      return Status::out_of_range(
          "RadarMeasurementModel range is too close to zero.");
    }

    MeasurementResult result;
    result.measurement.timestamp = request.context.timestamp;
    result.measurement.frame_id = request.context.frame_id.empty()
                                      ? request.state.frame_id
                                      : request.context.frame_id;
    result.measurement.sensor_id = request.sensor_id;
    result.measurement.value = Vector::Zero(3);
    result.measurement.value[0] = range;
    result.measurement.value[1] = std::atan2(py, px);
    result.measurement.value[2] = (px * vx + py * vy) / range;
    return result;
  }

  [[nodiscard]] Result<Matrix> state_jacobian(
      const MeasurementRequest& request) const override {
    if (request.state.dimension() != 4) {
      return Status::dimension_mismatch(
          "RadarMeasurementModel expects a 4D state [px, py, vx, vy].");
    }

    const Vector& x = request.state.value;
    const core::Scalar px = x[0];
    const core::Scalar py = x[1];
    const core::Scalar vx = x[2];
    const core::Scalar vy = x[3];
    const core::Scalar range_sq = px * px + py * py;
    const core::Scalar range = std::sqrt(range_sq);

    if (range < range_epsilon_ || range_sq < range_epsilon_ * range_epsilon_) {
      return Status::out_of_range(
          "RadarMeasurementModel range is too close to zero.");
    }

    Matrix h = Matrix::Zero(3, 4);
    h(0, 0) = px / range;
    h(0, 1) = py / range;

    h(1, 0) = -py / range_sq;
    h(1, 1) = px / range_sq;

    const core::Scalar range_cubed = range_sq * range;
    const core::Scalar dot = px * vx + py * vy;
    h(2, 0) = vx / range - dot * px / range_cubed;
    h(2, 1) = vy / range - dot * py / range_cubed;
    h(2, 2) = px / range;
    h(2, 3) = py / range;
    return h;
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "radar_measurement";
  }

 private:
  core::Scalar range_epsilon_;
};

}  // namespace ros_tracker::models
