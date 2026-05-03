#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "ros_tracker/models/models.hpp"

namespace
{
  int g_failures = 0;

  /// Asserts that a condition is true.
  void expect_true(const bool condition, const std::string &message)
  {
    if (!condition)
    {
      std::cerr << "FAIL: " << message << '\n';
      ++g_failures;
    }
  }

  /// Asserts that two scalar values are close.
  void expect_near(
      const ros_tracker::core::Scalar actual,
      const ros_tracker::core::Scalar expected,
      const ros_tracker::core::Scalar tolerance,
      const std::string &message)
  {
    expect_true(std::abs(actual - expected) <= tolerance, message);
  }

  /// Approximates a measurement Jacobian with finite differences.
  ros_tracker::core::Matrix finite_difference_jacobian(
      const ros_tracker::models::MeasurementModel &model,
      const ros_tracker::models::MeasurementRequest &request,
      const ros_tracker::core::Scalar epsilon = 1e-6)
  {
    using namespace ros_tracker::core;
    using namespace ros_tracker::models;

    const auto base = model.measure(request);
    Matrix jacobian(base.value().measurement.dimension(), request.state.dimension());

    for (Index i = 0; i < request.state.dimension(); ++i)
    {
      MeasurementRequest plus = request;
      MeasurementRequest minus = request;
      plus.state.value[i] += epsilon;
      minus.state.value[i] -= epsilon;

      const auto plus_result = model.measure(plus);
      const auto minus_result = model.measure(minus);
      jacobian.col(i) = (plus_result.value().measurement.value -
                         minus_result.value().measurement.value) /
                        (2.0 * epsilon);
    }

    return jacobian;
  }

  /// Tests the motion-model implementations.
  void test_motion_models()
  {
    using namespace ros_tracker::core;
    using namespace ros_tracker::models;

    MotionRequest cv_request{
        State{(Vector(4) << 1.0, 2.0, 0.5, -1.0).finished(), 0.0, "map"},
        std::nullopt,
        ModelContext{2.0, 2.0, "map"},
    };
    ConstantVelocityMotionModel cv_model;
    const auto cv_result = cv_model.propagate(cv_request);
    expect_true(cv_result.ok(), "Constant velocity propagation should succeed.");
    expect_near(cv_result.value().state.value[0], 2.0, 1e-12,
                "Constant velocity x position should advance by vx * dt.");
    expect_near(cv_result.value().state.value[1], 0.0, 1e-12,
                "Constant velocity y position should advance by vy * dt.");

    MotionRequest ca_request{
        State{(Vector(6) << 0.0, 0.0, 1.0, 2.0, 0.5, -0.5).finished(), 0.0, "map"},
        std::nullopt,
        ModelContext{2.0, 2.0, "map"},
    };
    ConstantAccelerationMotionModel ca_model;
    const auto ca_result = ca_model.propagate(ca_request);
    expect_true(ca_result.ok(), "Constant acceleration propagation should succeed.");
    expect_near(ca_result.value().state.value[0], 3.0, 1e-12,
                "Constant acceleration x position should include acceleration.");
    expect_near(ca_result.value().state.value[3], 1.0, 1e-12,
                "Constant acceleration vy should advance by ay * dt.");

    MotionRequest ct_request{
        State{(Vector(5) << 0.0, 0.0, 10.0, 0.0, 0.0).finished(), 0.0, "map"},
        std::nullopt,
        ModelContext{0.5, 0.5, "map"},
    };
    CoordinatedTurnMotionModel ct_model;
    const auto ct_result = ct_model.propagate(ct_request);
    expect_true(ct_result.ok(), "Coordinated turn propagation should succeed.");
    expect_near(ct_result.value().state.value[0], 5.0, 1e-12,
                "Zero turn-rate coordinated turn should reduce to straight motion.");
    expect_near(ct_result.value().state.value[1], 0.0, 1e-12,
                "Zero turn-rate coordinated turn should preserve lateral position.");

    MotionRequest singer_request{
        State{(Vector(6) << 0.0, 0.0, 1.0, 0.0, 2.0, 0.0).finished(), 0.0, "map"},
        std::nullopt,
        ModelContext{1.0, 1.0, "map"},
    };
    SingerMotionModel singer_model(0.5);
    const auto singer_result = singer_model.propagate(singer_request);
    expect_true(singer_result.ok(), "Singer propagation should succeed.");
    expect_true(singer_result.value().state.value[0] > 1.0,
                "Singer position should include decaying acceleration contribution.");
    expect_true(singer_result.value().state.value[4] < 2.0,
                "Singer acceleration should decay over time.");
  }

  /// Tests the measurement-model implementations.
  void test_measurement_models()
  {
    using namespace ros_tracker::core;
    using namespace ros_tracker::models;

    MeasurementRequest linear_request{
        State{(Vector(4) << 3.0, 4.0, 1.0, 2.0).finished(), 0.0, "map"},
        ModelContext{0.0, 1.0, "map"},
        "linear_sensor",
    };

    Matrix h(2, 4);
    h << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;
    LinearMeasurementModel linear_model(h);
    const auto linear_result = linear_model.measure(linear_request);
    expect_true(linear_result.ok(), "Linear measurement should succeed.");
    expect_near(linear_result.value().measurement.value[0], 3.0, 1e-12,
                "Linear measurement should project the x position.");
    expect_near(linear_result.value().measurement.value[1], 4.0, 1e-12,
                "Linear measurement should project the y position.");

    RadarMeasurementModel radar_model;
    const auto radar_result = radar_model.measure(linear_request);
    expect_true(radar_result.ok(), "Radar measurement should succeed.");
    expect_near(radar_result.value().measurement.value[0], 5.0, 1e-12,
                "Radar range should equal sqrt(px^2 + py^2).");
    expect_near(radar_result.value().measurement.value[2], 2.2, 1e-12,
                "Radar range rate should equal radial velocity.");

    const auto analytic_jacobian = radar_model.state_jacobian(linear_request);
    expect_true(analytic_jacobian.ok(), "Radar Jacobian should succeed.");
    const Matrix numeric_jacobian =
        finite_difference_jacobian(radar_model, linear_request);
    expect_true(analytic_jacobian.value().isApprox(numeric_jacobian, 1e-5),
                "Radar analytic Jacobian should match a finite-difference approximation.");
  }

  /// Tests the noise models and model composition helpers.
  void test_noise_and_composition()
  {
    using namespace ros_tracker::core;
    using namespace ros_tracker::models;

    MotionRequest motion_request{
        State{Vector::Zero(4), 0.0, "map"},
        std::nullopt,
        ModelContext{0.1, 0.1, "map"},
    };
    MeasurementRequest measurement_request{
        State{Vector::Zero(4), 0.0, "map"},
        ModelContext{0.0, 0.1, "map"},
        "radar",
    };

    ConstantGaussianProcessNoise process_noise(Matrix::Identity(4, 4));
    ConstantGaussianMeasurementNoise measurement_noise(Matrix::Identity(2, 2));

    const auto q = process_noise.covariance(motion_request);
    const auto r = measurement_noise.covariance(measurement_request);
    expect_true(q.ok(), "Process noise covariance should validate.");
    expect_true(r.ok(), "Measurement noise covariance should validate.");

    DynamicSystemModel dynamic_system{
        std::make_shared<ConstantVelocityMotionModel>(),
        std::make_shared<ConstantGaussianProcessNoise>(Matrix::Identity(4, 4)),
    };
    SensorModel sensor{
        std::make_shared<LinearMeasurementModel>((Matrix(2, 4) << 1.0, 0.0, 0.0, 0.0,
                                                  0.0, 1.0, 0.0, 0.0)
                                                     .finished()),
        std::make_shared<ConstantGaussianMeasurementNoise>(Matrix::Identity(2, 2)),
    };

    expect_true(dynamic_system.validate().ok(),
                "DynamicSystemModel should validate when both dependencies are present.");
    expect_true(sensor.validate().ok(),
                "SensorModel should validate when both dependencies are present.");
  }

} // namespace

/// Runs the test executable.
int main()
{
  test_motion_models();
  test_measurement_models();
  test_noise_and_composition();

  if (g_failures != 0)
  {
    std::cerr << g_failures << " test(s) failed.\n";
    return EXIT_FAILURE;
  }

  std::cout << "All model tests passed.\n";
  return EXIT_SUCCESS;
}
