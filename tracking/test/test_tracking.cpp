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

ros_tracker::tracking::TrackDependencies make_default_dependencies() {
  using namespace ros_tracker::filters;
  using namespace ros_tracker::tracking;

  return {
      std::make_shared<KalmanFilter>(),
      make_constant_velocity_system(),
      make_position_sensor(),
  };
}

class SensorAwareTrackManager final : public ros_tracker::tracking::TrackManager {
 public:
  SensorAwareTrackManager(
      const ros_tracker::core::Index state_dimension,
      ros_tracker::core::Covariance initial_covariance)
      : base_manager_(
            state_dimension,
            std::move(initial_covariance),
            std::vector<ros_tracker::core::Index> {0, 1},
            1U,
            2U) {}

  [[nodiscard]] ros_tracker::core::Result<ros_tracker::tracking::Track>
  initiate_track(
      const ros_tracker::tracking::TrackId id,
      const ros_tracker::core::Measurement& measurement,
      const ros_tracker::tracking::TrackDependencies& dependencies,
      const ros_tracker::models::ModelContext& context = {}) const override {
    auto track = base_manager_.initiate_track(id, measurement, dependencies, context);
    if (!track.ok()) {
      return track.status();
    }

    if (measurement.sensor_id == "pf") {
      track.value().dependencies.filter =
          std::make_shared<ros_tracker::filters::ParticleFilter>(128, 42U, 0.3);
    }

    return track;
  }

  [[nodiscard]] ros_tracker::core::Result<ros_tracker::tracking::Track>
  on_prediction(
      const ros_tracker::tracking::Track& track,
      const ros_tracker::filters::GaussianEstimate& predicted_estimate) const override {
    return base_manager_.on_prediction(track, predicted_estimate);
  }

  [[nodiscard]] ros_tracker::core::Result<ros_tracker::tracking::Track>
  on_correction(
      const ros_tracker::tracking::Track& track,
      const ros_tracker::filters::GaussianEstimate& corrected_estimate,
      const ros_tracker::core::Measurement& measurement) const override {
    return base_manager_.on_correction(track, corrected_estimate, measurement);
  }

  [[nodiscard]] ros_tracker::core::Result<ros_tracker::tracking::Track>
  on_missed_detection(
      const ros_tracker::tracking::Track& track) const override {
    return base_manager_.on_missed_detection(track);
  }

  [[nodiscard]] bool should_remove(
      const ros_tracker::tracking::Track& track) const noexcept override {
    return base_manager_.should_remove(track);
  }

  [[nodiscard]] std::string_view name() const noexcept override {
    return "sensor_aware_manager";
  }

 private:
  ros_tracker::tracking::BasicTrackManager base_manager_;
};

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
      make_default_dependencies(),
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
      make_default_dependencies(),
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

void test_multi_target_tracker_per_track_dependencies() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::tracking;

  auto association = std::make_shared<NearestNeighborAssociationStrategy>(9.0);
  auto manager = std::make_shared<SensorAwareTrackManager>(
      4,
      4.0 * Matrix::Identity(4, 4));

  MultiTargetTracker tracker(
      make_default_dependencies(),
      association,
      manager);

  std::vector<Measurement> frame0 {
      Measurement {(Vector(2) << 0.0, 0.0).finished(), 0.0, "kalman", "map"},
      Measurement {(Vector(2) << 10.0, 10.0).finished(), 0.0, "pf", "map"},
  };
  const auto tracks0 = tracker.step(
      frame0,
      ros_tracker::models::ModelContext {0.0, 0.0, "map"});
  expect_true(tracks0.ok(), "Tracker should initialize per-track dependencies.");
  expect_true(tracks0.value().size() == 2U,
              "Tracker should initialize one track per measurement.");

  std::vector<Measurement> frame1 {
      Measurement {(Vector(2) << 1.0, 0.0).finished(), 1.0, "kalman", "map"},
      Measurement {(Vector(2) << 11.0, 10.0).finished(), 1.0, "pf", "map"},
  };
  const auto tracks1 = tracker.step(
      frame1,
      ros_tracker::models::ModelContext {1.0, 1.0, "map"});
  expect_true(tracks1.ok(), "Tracker should update tracks with their own filters.");

  bool saw_particle_track = false;
  bool saw_kalman_track = false;
  for (const Track& track : tracks1.value()) {
    if (track.source_sensor_id == "pf") {
      saw_particle_track = true;
      expect_true(track.estimate.particle_set.has_value(),
                  "Particle-filter track should keep its particle posterior.");
    }
    if (track.source_sensor_id == "kalman") {
      saw_kalman_track = true;
      expect_true(!track.estimate.particle_set.has_value(),
                  "Kalman-filter track should remain Gaussian-only.");
    }
  }
  expect_true(saw_particle_track,
              "Tracker should keep the particle-filter track in the active set.");
  expect_true(saw_kalman_track,
              "Tracker should keep the Kalman-filter track in the active set.");
}

void test_multi_model_estimator() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::tracking;

  auto shared_filter = std::make_shared<KalmanFilter>();

  std::vector<ModelBankEntry> model_bank {
      ModelBankEntry {
          "low_q",
          {
              std::make_shared<ros_tracker::models::ConstantVelocityMotionModel>(),
              std::make_shared<ros_tracker::models::ConstantGaussianProcessNoise>(
                  0.01 * Matrix::Identity(4, 4)),
          },
          shared_filter,
      },
      ModelBankEntry {
          "high_q",
          {
              std::make_shared<ros_tracker::models::ConstantVelocityMotionModel>(),
              std::make_shared<ros_tracker::models::ConstantGaussianProcessNoise>(
                  0.5 * Matrix::Identity(4, 4)),
          },
          shared_filter,
      },
  };

  Matrix transition(2, 2);
  transition << 0.95, 0.05,
                0.10, 0.90;
  InteractingMultipleModelEstimator imm(model_bank, transition);

  GaussianEstimate seed {
      State {(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
      Matrix::Identity(4, 4),
  };
  const auto initialized = imm.initialize(seed, {0.8, 0.2});
  expect_true(initialized.ok(), "IMM initialization should succeed.");
  expect_true(initialized.value().modes.size() == 2U,
              "IMM should keep one hypothesis per model.");

  Measurement measurement {
      (Vector(2) << 1.1, -0.05).finished(),
      1.0,
      "pos",
      "map",
  };
  const auto updated = imm.step(
      initialized.value(),
      make_position_sensor(),
      measurement,
      ros_tracker::models::ModelContext {1.0, 1.0, "map"});
  expect_true(updated.ok(), "IMM predict-correct step should succeed.");
  expect_true(updated.value().merged_estimate.state.value[0] > 0.5,
              "IMM merged estimate should move toward the new measurement.");

  Scalar probability_sum = 0.0;
  for (const auto& mode : updated.value().modes) {
    probability_sum += mode.probability;
  }
  expect_true(std::abs(probability_sum - 1.0) < 1e-9,
              "IMM mode probabilities should stay normalized.");
}

void test_covariance_intersection_fusion() {
  using namespace ros_tracker::core;
  using namespace ros_tracker::filters;
  using namespace ros_tracker::tracking;

  CovarianceIntersectionFuser fuser;
  GaussianEstimate estimate_a {
      State {(Vector(4) << 1.0, 0.0, 0.5, 0.0).finished(), 1.0, "map"},
      1.5 * Matrix::Identity(4, 4),
  };
  GaussianEstimate estimate_b {
      State {(Vector(4) << 1.4, 0.2, 0.4, 0.0).finished(), 1.2, "map"},
      0.8 * Matrix::Identity(4, 4),
  };

  const auto fused = fuser.fuse_pair(estimate_a, estimate_b);
  expect_true(fused.ok(), "Covariance intersection should succeed.");
  expect_true(std::abs(fused.value().state.value[0] - estimate_b.state.value[0]) <=
                  std::abs(estimate_a.state.value[0] - estimate_b.state.value[0]) + 1e-9,
              "Covariance intersection mean should stay no farther from the more certain estimate than the inputs are from each other.");
  expect_true(fused.value().covariance.trace() <= estimate_a.covariance.trace() + 1e-9,
              "Covariance intersection should not be looser than the worst input covariance.");
  expect_true(fused.value().covariance.trace() <= estimate_b.covariance.trace() + 1e-9,
              "Covariance intersection should stay conservative relative to the best input covariance.");
}

}  // namespace

int main() {
  test_nearest_neighbor_association();
  test_multi_target_tracker_lifecycle();
  test_multi_target_tracker_per_track_dependencies();
  test_multi_model_estimator();
  test_covariance_intersection_fusion();

  if (g_failures != 0) {
    std::cerr << g_failures << " test(s) failed.\n";
    return EXIT_FAILURE;
  }

  std::cout << "All tracking tests passed.\n";
  return EXIT_SUCCESS;
}
