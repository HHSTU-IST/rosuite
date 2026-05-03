#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ros_tracker/apps/apps.hpp"

namespace {

int g_failures = 0;

/// Asserts that a condition is true.
void expect_true(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

/// Tests offline scenario reproducibility.
void test_offline_scenario_reproducibility() {
  using namespace ros_tracker::apps::offline;
  using namespace ros_tracker::core;

  ConstantVelocityScenarioConfig config;
  config.initial_state =
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"};
  config.dt = 1.0;
  config.steps = 5U;
  config.process_noise_covariance = 0.01 * Matrix::Identity(4, 4);
  config.measurement_noise_covariance = 0.25 * Matrix::Identity(2, 2);
  config.sensor_id = "offline_sensor";
  config.frame_id = "map";

  const auto scenario_a = build_constant_velocity_position_scenario(config, 7U);
  const auto scenario_b = build_constant_velocity_position_scenario(config, 7U);
  expect_true(scenario_a.ok() && scenario_b.ok(),
              "Offline scenarios should build successfully.");
  expect_true(scenario_a.value().truth_states.size() == config.steps,
              "Offline scenario should generate the requested number of truth frames.");
  expect_true(
      scenario_a.value().measurement_frames[0].measurements[0].value.isApprox(
          scenario_b.value().measurement_frames[0].measurements[0].value, 1e-12),
      "Offline scenario generation should be reproducible for a fixed seed.");
}

/// Tests offline example metric generation.
void test_offline_example_metrics() {
  using namespace ros_tracker::apps::offline;

  const auto summary = run_single_target_kalman_example(42U);
  expect_true(summary.ok(), "Offline example should run successfully.");
  expect_true(summary.value().metrics.position_rmse < 1.0,
              "Offline example position RMSE should stay bounded.");
  expect_true(summary.value().metrics.velocity_rmse < 1.0,
              "Offline example velocity RMSE should stay bounded.");
  expect_true(summary.value().tracker_frames.back().tracks.size() == 1U,
              "Offline example should keep a single primary track.");
}

/// Tests the ROS adapter workflow.
void test_ros_adapter() {
  using namespace ros_tracker::apps::ros;
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::models;
  using namespace ros_tracker::tracking;

  auto tracker = std::make_shared<MultiTargetTracker>(
      std::make_shared<KalmanFilter>(),
      make_constant_velocity_system(0.05 * Matrix::Identity(4, 4)),
      make_position_sensor(0.25 * Matrix::Identity(2, 2)),
      std::make_shared<NearestNeighborAssociationStrategy>(16.0),
      std::make_shared<BasicTrackManager>(
          4,
          4.0 * Matrix::Identity(4, 4),
          std::vector<Index> {0, 1},
          1U,
          2U));

  TrackerNodeAdapter adapter(
      tracker,
      TrackerNodeParameters {1.0, "map"});

  std::vector<Measurement> measurements {
      Measurement {(Vector(2) << 0.5, -0.1).finished(), 1.0, "ros_sensor", "map"},
  };
  const auto message = adapter.process_measurements(measurements);
  expect_true(message.ok(), "ROS adapter should process measurements successfully.");
  expect_true(message.value().tracks.size() == 1U,
              "ROS adapter should publish one track for one initialized target.");
  expect_true(message.value().tracks.front().lifecycle == "confirmed",
              "ROS adapter should expose the track lifecycle as a string.");

  std::vector<Measurement> inconsistent_measurements {
      Measurement {(Vector(2) << 1.0, 0.0).finished(), 2.0, "ros_sensor", "map"},
      Measurement {(Vector(2) << 1.5, 0.2).finished(), 2.1, "ros_sensor", "map"},
  };
  const auto inconsistent_message =
      adapter.process_measurements(inconsistent_measurements);
  expect_true(!inconsistent_message.ok(),
              "ROS adapter should reject a measurement batch with inconsistent timestamps.");
}

}  // namespace

/// Runs the test executable.
int main() {
  test_offline_scenario_reproducibility();
  test_offline_example_metrics();
  test_ros_adapter();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed.\n";
    return EXIT_FAILURE;
  }

  std::cout << "All apps tests passed.\n";
  return EXIT_SUCCESS;
}
