#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "ros_tracker/filters/filters.hpp"
#include "ros_tracker/models/models.hpp"

namespace {

int g_failures = 0;

/// Asserts that a condition is true.
void expect_true(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

/// Asserts that two scalar values are close.
void expect_near(
    const ros_tracker::core::Scalar actual,
    const ros_tracker::core::Scalar expected,
    const ros_tracker::core::Scalar tolerance,
    const std::string& message) {
  expect_true(std::abs(actual - expected) <= tolerance, message);
}

/// Builds a constant-velocity system model for tests.
ros_tracker::models::DynamicSystemModel make_constant_velocity_system() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::models;

  return {
      std::make_shared<ConstantVelocityMotionModel>(),
      std::make_shared<ConstantGaussianProcessNoise>(
          0.1 * Matrix::Identity(4, 4)),
  };
}

/// Builds a linear position sensor model.
ros_tracker::models::SensorModel make_position_sensor() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::models;

  Matrix h(2, 4);
  h << 1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0;

  return {
      std::make_shared<LinearMeasurementModel>(h),
      std::make_shared<ConstantGaussianMeasurementNoise>(
          0.25 * Matrix::Identity(2, 2)),
  };
}

/// Tests the linear Kalman filter.
void test_kalman_filter() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  KalmanFilter filter;
  GaussianEstimate estimate {
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
      Matrix::Identity(4, 4),
  };

  const DynamicSystemModel system = make_constant_velocity_system();
  const ModelContext predict_context {1.0, 1.0, "map"};
  const auto predicted = filter.predict(estimate, system, predict_context);
  expect_true(predicted.ok(), "Kalman prediction should succeed.");
  expect_near(predicted.value().state.value[0], 1.0, 1e-12,
              "Kalman prediction should advance x position.");

  const SensorModel sensor = make_position_sensor();
  Measurement measurement {
      (Vector(2) << 1.2, -0.1).finished(),
      1.0,
      "pos_sensor",
      "map",
  };
  const auto corrected = filter.correct(predicted.value(), sensor, measurement);
  expect_true(corrected.ok(), "Kalman correction should succeed.");
  expect_true(corrected.value().state.value[0] > predicted.value().state.value[0],
              "Kalman correction should move the estimate toward the measurement.");
  expect_true(corrected.value().covariance.trace() <
                  predicted.value().covariance.trace(),
              "Kalman correction should reduce covariance trace.");
}

/// Tests the constant-gain filter.
void test_constant_gain_filter() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  Matrix gain = Matrix::Zero(4, 2);
  gain(0, 0) = 1.0;
  gain(1, 1) = 1.0;
  ConstantGainFilter filter(gain);

  GaussianEstimate estimate {
      State {Vector::Zero(4), 0.0, "map"},
      Matrix::Identity(4, 4),
  };

  Measurement measurement {
      (Vector(2) << 3.0, -2.0).finished(),
      1.0,
      "pos_sensor",
      "map",
  };

  const auto corrected = filter.correct(estimate, make_position_sensor(), measurement);
  expect_true(corrected.ok(), "Constant gain correction should succeed.");
  expect_near(corrected.value().state.value[0], 3.0, 1e-12,
              "Constant gain filter should overwrite x position with unit gain.");
  expect_near(corrected.value().state.value[1], -2.0, 1e-12,
              "Constant gain filter should overwrite y position with unit gain.");
}

/// Tests the extended Kalman filter with a radar model.
void test_kalman_filter_extended_with_radar() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  KalmanFilterExtended filter;
  GaussianEstimate estimate {
      State {(Vector(4) << 2.5, 4.5, 0.8, 1.7).finished(), 0.0, "map"},
      0.5 * Matrix::Identity(4, 4),
  };

  SensorModel radar {
      std::make_shared<RadarMeasurementModel>(),
      std::make_shared<ConstantGaussianMeasurementNoise>(
          0.1 * Matrix::Identity(3, 3)),
  };

  Measurement measurement {
      (Vector(3) << 5.0, std::atan2(4.0, 3.0), 2.2).finished(),
      1.0,
      "radar",
      "map",
  };

  const auto corrected = filter.correct(estimate, radar, measurement);
  expect_true(corrected.ok(), "Extended Kalman correction should succeed.");
  expect_true(corrected.value().covariance.trace() < estimate.covariance.trace(),
              "Extended Kalman correction should reduce covariance trace.");
}

/// Tests the least-squares estimator.
void test_least_squares() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;

  LeastSquaresEstimator estimator;
  LeastSquaresProblem problem;
  problem.design_matrix = (Matrix(3, 2) << 1.0, 0.0,
                                          0.0, 1.0,
                                          1.0, 1.0).finished();
  problem.observation_vector = (Vector(3) << 1.0, 2.0, 3.0).finished();
  problem.observation_covariance = Matrix::Identity(3, 3);

  const auto solution = estimator.solve(problem);
  expect_true(solution.ok(), "Least squares solution should succeed.");
  expect_near(solution.value().solution[0], 1.0, 1e-12,
              "Least squares should recover the first state component.");
  expect_near(solution.value().solution[1], 2.0, 1e-12,
              "Least squares should recover the second state component.");
}

/// Tests sigma-point generation utilities.
void test_sigma_points() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;

  GaussianEstimate estimate {
      State {(Vector(2) << 1.0, -2.0).finished(), 0.0, "map"},
      (Matrix(2, 2) << 4.0, 1.0,
                       1.0, 2.0).finished(),
  };

  MerweSigmaPointGenerator generator(0.5, 2.0, 0.0);
  const auto sigma_points = generator.generate(estimate);
  expect_true(sigma_points.ok(), "Sigma point generation should succeed.");

  Vector mean = Vector::Zero(2);
  for (Index i = 0; i < sigma_points.value().points.cols(); ++i) {
    mean += sigma_points.value().mean_weights[i] *
            sigma_points.value().points.col(i);
  }
  expect_true(mean.isApprox(estimate.state.value, 1e-10),
              "Sigma points should reconstruct the original mean.");
}

/// Tests the Rauch-Tung-Striebel smoother.
void test_rts_smoother() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;

  GaussianEstimate filtered {
      State {(Vector(2) << 0.0, 1.0).finished(), 0.0, "map"},
      Matrix::Identity(2, 2),
  };
  GaussianEstimate predicted_next {
      State {(Vector(2) << 1.0, 1.0).finished(), 1.0, "map"},
      2.0 * Matrix::Identity(2, 2),
  };
  GaussianEstimate smoothed_next {
      State {(Vector(2) << 0.8, 0.9).finished(), 1.0, "map"},
      0.5 * Matrix::Identity(2, 2),
  };
  Matrix transition = Matrix::Identity(2, 2);

  RauchTungStriebelSmoother smoother;
  const auto smoothed =
      smoother.smooth_step(filtered, predicted_next, smoothed_next, transition);
  expect_true(smoothed.ok(), "RTS smoothing step should succeed.");
  expect_true(smoothed.value().covariance.trace() <= filtered.covariance.trace(),
              "RTS smoothing should not increase covariance trace in this setup.");
}

/// Tests the unscented and cubature Kalman filters.
void test_unscented_and_cubature_filters() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  const auto system = make_constant_velocity_system();
  const auto sensor = make_position_sensor();

  GaussianEstimate estimate {
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
      Matrix::Identity(4, 4),
  };
  const ModelContext context {1.0, 1.0, "map"};

  KalmanFilterUnscented ukf;
  const auto ukf_predicted = ukf.predict(estimate, system, context);
  expect_true(ukf_predicted.ok(), "UKF prediction should succeed.");
  expect_near(ukf_predicted.value().state.value[0], 1.0, 1e-8,
              "UKF prediction should advance x position.");

  Measurement measurement {
      (Vector(2) << 1.1, 0.1).finished(),
      1.0,
      "pos_sensor",
      "map",
  };
  const auto ukf_corrected = ukf.correct(ukf_predicted.value(), sensor, measurement);
  expect_true(ukf_corrected.ok(), "UKF correction should succeed.");
  expect_true(ukf_corrected.value().covariance.trace() <
                  ukf_predicted.value().covariance.trace(),
              "UKF correction should reduce covariance trace.");

  KalmanFilterCubature ckf;
  const auto ckf_predicted = ckf.predict(estimate, system, context);
  expect_true(ckf_predicted.ok(), "CKF prediction should succeed.");
  expect_near(ckf_predicted.value().state.value[0], 1.0, 1e-8,
              "CKF prediction should advance x position.");
  const auto ckf_corrected = ckf.correct(ckf_predicted.value(), sensor, measurement);
  expect_true(ckf_corrected.ok(), "CKF correction should succeed.");
  expect_true(ckf_corrected.value().covariance.trace() <
                  ckf_predicted.value().covariance.trace(),
              "CKF correction should reduce covariance trace.");
}

/// Tests the ensemble Kalman filter.
void test_kalman_filter_ensemble() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  KalmanFilterEnsemble enkf(128, 42U);
  GaussianEstimate estimate {
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
      0.5 * Matrix::Identity(4, 4),
  };
  const auto system = make_constant_velocity_system();
  const ModelContext context {1.0, 1.0, "map"};

  const auto predicted = enkf.predict(estimate, system, context);
  expect_true(predicted.ok(), "EnKF prediction should succeed.");
  expect_near(predicted.value().state.value[0], 1.0, 0.2,
              "EnKF prediction mean should stay close to constant-velocity motion.");

  Measurement measurement {
      (Vector(2) << 1.2, -0.1).finished(),
      1.0,
      "pos_sensor",
      "map",
  };
  const auto corrected = enkf.correct(predicted.value(), make_position_sensor(), measurement);
  expect_true(corrected.ok(), "EnKF correction should succeed.");
  expect_true(corrected.value().state.value[0] > predicted.value().state.value[0] - 1e-6,
              "EnKF correction should move the estimate toward the measurement.");
}

/// Tests the fading-memory Kalman filter.
void test_kalman_filter_fading_memory() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  KalmanFilter classical;
  KalmanFilterFadingMemory fading_memory(1.2);

  GaussianEstimate estimate {
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
      Matrix::Identity(4, 4),
  };

  const auto system = make_constant_velocity_system();
  const ModelContext context {1.0, 1.0, "map"};

  const auto baseline = classical.predict(estimate, system, context);
  expect_true(baseline.ok(), "Baseline Kalman prediction should succeed.");

  const auto predicted = fading_memory.predict(estimate, system, context);
  expect_true(predicted.ok(), "Fading-memory Kalman prediction should succeed.");
  expect_true(predicted.value().covariance.trace() > baseline.value().covariance.trace(),
              "Fading-memory Kalman prediction should inflate covariance.");

  Measurement measurement {
      (Vector(2) << 1.2, -0.05).finished(),
      1.0,
      "pos_sensor",
      "map",
  };
  const auto corrected =
      fading_memory.correct(predicted.value(), make_position_sensor(), measurement);
  expect_true(corrected.ok(), "Fading-memory Kalman correction should succeed.");
  expect_true(corrected.value().covariance.trace() <
                  predicted.value().covariance.trace(),
              "Fading-memory Kalman correction should reduce covariance trace.");
}

/// Tests the H-infinity Kalman filter.
void test_kalman_filter_h_infinity() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  KalmanFilterHInfinity hinf(25.0);
  GaussianEstimate estimate {
      State {(Vector(4) << 0.8, -0.2, 1.0, 0.0).finished(), 0.0, "map"},
      Matrix::Identity(4, 4),
  };

  Measurement measurement {
      (Vector(2) << 1.25, 0.1).finished(),
      1.0,
      "pos_sensor",
      "map",
  };

  const auto corrected =
      hinf.correct(estimate, make_position_sensor(), measurement);
  expect_true(corrected.ok(), "H-infinity Kalman correction should succeed.");
  expect_true(corrected.value().covariance.trace() < estimate.covariance.trace(),
              "H-infinity Kalman correction should reduce covariance trace.");
  expect_true(corrected.value().state.value[0] > estimate.state.value[0],
              "H-infinity Kalman correction should move the estimate toward the measurement.");
}

/// Tests the particle filter.
void test_particle_filter() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;

  ParticleFilter pf(256, 42U, 0.3);
  GaussianEstimate estimate {
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
      0.25 * Matrix::Identity(4, 4),
  };

  const auto system = make_constant_velocity_system();
  const ModelContext context {1.0, 1.0, "map"};
  const auto predicted = pf.predict(estimate, system, context);
  expect_true(predicted.ok(), "Particle filter prediction should succeed.");
  expect_near(predicted.value().state.value[0], 1.0, 0.25,
              "Particle filter prediction mean should stay close to constant-velocity motion.");
  expect_true(predicted.value().particle_set.has_value(),
              "Particle filter prediction should preserve a particle posterior.");

  Measurement measurement {
      (Vector(2) << 1.3, 0.05).finished(),
      1.0,
      "pos_sensor",
      "map",
  };
  const auto corrected =
      pf.correct(predicted.value(), make_position_sensor(), measurement);
  expect_true(corrected.ok(), "Particle filter correction should succeed.");
  expect_true(corrected.value().state.value[0] > predicted.value().state.value[0],
              "Particle filter correction should move the estimate toward the measurement.");
  expect_true(corrected.value().particle_set.has_value(),
              "Particle filter correction should keep the resampled particles.");
}

}  // namespace

/// Runs the test executable.
int main() {
  test_kalman_filter();
  test_constant_gain_filter();
  test_kalman_filter_extended_with_radar();
  test_least_squares();
  test_sigma_points();
  test_rts_smoother();
  test_unscented_and_cubature_filters();
  test_kalman_filter_ensemble();
  test_kalman_filter_fading_memory();
  test_kalman_filter_h_infinity();
  test_particle_filter();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed.\n";
    return EXIT_FAILURE;
  }

  std::cout << "All filter tests passed.\n";
  return EXIT_SUCCESS;
}
