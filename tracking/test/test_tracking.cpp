#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ros_tracker/filters/filters.hpp"
#include "ros_tracker/models/models.hpp"
#include "ros_tracker/tracking/tracking.hpp"

namespace {

int g_failures = 0;

void expect_true(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

ros_tracker::models::DynamicSystemModel make_constant_velocity_system() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::models;

  return {
      std::make_shared<ConstantVelocityMotionModel>(),
      std::make_shared<ConstantGaussianProcessNoise>(
          0.05 * Matrix::Identity(4, 4)),
  };
}

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

void test_nearest_neighbor_association() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::tracking;

  NearestNeighborAssociationStrategy strategy(9.0);

  std::vector<Track> tracks;
  tracks.push_back(Track {
      1U,
      GaussianEstimate {
          State {(Vector(4) << 0.0, 0.0, 0.0, 0.0).finished(), 0.0, "map"},
          Matrix::Identity(4, 4),
      },
      TrackLifecycle::kConfirmed,
      2U,
      2U,
      0U,
      0U,
      "",
  });
  tracks.push_back(Track {
      2U,
      GaussianEstimate {
          State {(Vector(4) << 10.0, 10.0, 0.0, 0.0).finished(), 0.0, "map"},
          Matrix::Identity(4, 4),
      },
      TrackLifecycle::kConfirmed,
      2U,
      2U,
      0U,
      0U,
      "",
  });

  std::vector<Measurement> measurements {
      Measurement {(Vector(2) << 0.1, -0.2).finished(), 1.0, "pos", "map"},
      Measurement {(Vector(2) << 9.8, 10.1).finished(), 1.0, "pos", "map"},
  };

  const auto result = strategy.associate(
      tracks,
      measurements,
      make_position_sensor(),
      ros_tracker::models::ModelContext {1.0, 1.0, "map"});
  expect_true(result.ok(), "Nearest-neighbor association should succeed.");
  expect_true(result.value().matches.size() == 2U,
              "Nearest-neighbor association should match both tracks.");
  expect_true(result.value().unmatched_tracks.empty(),
              "Nearest-neighbor association should not leave unmatched tracks.");
  expect_true(result.value().unmatched_measurements.empty(),
              "Nearest-neighbor association should not leave unmatched measurements.");
  bool found_first_pair = false;
  bool found_second_pair = false;
  for (const auto& match : result.value().matches) {
    found_first_pair |=
        (match.track_index == 0U && match.measurement_index == 0U);
    found_second_pair |=
        (match.track_index == 1U && match.measurement_index == 1U);
  }
  expect_true(found_first_pair,
              "Nearest-neighbor association should pair the first track with the first measurement.");
  expect_true(found_second_pair,
              "Nearest-neighbor association should pair the second track with the second measurement.");
}

void test_multi_target_tracker_lifecycle() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::tracking;

  auto filter = std::make_shared<KalmanFilter>();
  auto association = std::make_shared<NearestNeighborAssociationStrategy>(9.0);
  auto manager = std::make_shared<BasicTrackManager>(
      4,
      4.0 * Matrix::Identity(4, 4),
      std::vector<Index> {0, 1},
      2U,
      2U);

  MultiTargetTracker tracker(
      filter,
      make_constant_velocity_system(),
      make_position_sensor(),
      association,
      manager);

  std::vector<Measurement> frame0 {
      Measurement {(Vector(2) << 0.0, 0.0).finished(), 0.0, "pos", "map"},
      Measurement {(Vector(2) << 10.0, 10.0).finished(), 0.0, "pos", "map"},
  };
  const auto tracks0 = tracker.step(
      frame0,
      ros_tracker::models::ModelContext {0.0, 0.0, "map"});
  expect_true(tracks0.ok(), "Tracker should initialize tracks from the first frame.");
  expect_true(tracks0.value().size() == 2U,
              "Tracker should initialize two tracks from two measurements.");
  expect_true(tracks0.value()[0].lifecycle == TrackLifecycle::kTentative,
              "New tracks should start as tentative before enough hits.");

  std::vector<Measurement> frame1 {
      Measurement {(Vector(2) << 1.0, 0.1).finished(), 1.0, "pos", "map"},
      Measurement {(Vector(2) << 11.0, 10.1).finished(), 1.0, "pos", "map"},
  };
  const auto tracks1 = tracker.step(
      frame1,
      ros_tracker::models::ModelContext {1.0, 1.0, "map"});
  expect_true(tracks1.ok(), "Tracker should predict and correct the second frame.");
  expect_true(tracks1.value().size() == 2U,
              "Tracker should keep both tracks after the second frame.");
  expect_true(tracks1.value()[0].lifecycle == TrackLifecycle::kConfirmed &&
                  tracks1.value()[1].lifecycle == TrackLifecycle::kConfirmed,
              "Tracks should be confirmed after two hits.");

  const TrackId lost_track_id = tracks1.value()[1].id;

  std::vector<Measurement> frame2 {
      Measurement {(Vector(2) << 2.0, 0.0).finished(), 2.0, "pos", "map"},
  };
  const auto tracks2 = tracker.step(
      frame2,
      ros_tracker::models::ModelContext {1.0, 2.0, "map"});
  expect_true(tracks2.ok(), "Tracker should tolerate a missed detection.");
  expect_true(tracks2.value().size() == 2U,
              "Tracker should keep a track alive after one missed detection.");

  const auto tracks3 = tracker.step(
      frame2,
      ros_tracker::models::ModelContext {1.0, 3.0, "map"});
  expect_true(tracks3.ok(), "Tracker should process another frame after a miss.");
  expect_true(tracks3.value().size() == 1U,
              "Tracker should prune a track after too many consecutive misses.");
  expect_true(tracks3.value()[0].id != lost_track_id,
              "Tracker should prune the missed track instead of the active one.");
  expect_true(tracks3.value()[0].lifecycle == TrackLifecycle::kConfirmed,
              "Remaining active track should stay confirmed.");
}

}  // namespace

int main() {
  test_nearest_neighbor_association();
  test_multi_target_tracker_lifecycle();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed.\n";
    return EXIT_FAILURE;
  }

  std::cout << "All tracking tests passed.\n";
  return EXIT_SUCCESS;
}
