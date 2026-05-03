#include "support.hpp"

#include <vector>

#include "ros_tracker/tracking/association_tools.hpp"

namespace
{

using ros_tracker::tracking::test_support::SensorAwareTrackHandleFactory;
using ros_tracker::tracking::test_support::TestContext;

/// Tests multi-target tracker lifecycle updates.
void test_multi_target_tracker_lifecycle(TestContext &context)
{
    using namespace ros_tracker::core;
    using namespace ros_tracker::filters;
    using namespace ros_tracker::tracking;

    auto filter = std::make_shared<KalmanFilter>();
    auto association = std::make_shared<NearestNeighborAssociationStrategy>(9.0);
    auto manager = std::make_shared<BasicTrackManager>(
        4,
        4.0 * Matrix::Identity(4, 4),
        std::vector<Index>{0, 1},
        2U,
        2U);

    MultiTargetTracker tracker(
        filter,
        ros_tracker::models::make_constant_velocity_system(0.05 * Matrix::Identity(4, 4)),
        ros_tracker::models::make_position_sensor(0.25 * Matrix::Identity(2, 2)),
        association,
        manager);

    std::vector<Measurement> frame0{
        Measurement{(Vector(2) << 0.0, 0.0).finished(), 0.0, "pos", "map"},
        Measurement{(Vector(2) << 10.0, 10.0).finished(), 0.0, "pos", "map"},
    };
    const auto tracks0 = tracker.step(
        frame0,
        ros_tracker::models::ModelContext{0.0, 0.0, "map"});
    context.expect_true(tracks0.ok(), "Tracker should initialize tracks from the first frame.");
    context.expect_true(tracks0.value().size() == 2U,
                        "Tracker should initialize two tracks from two measurements.");
    context.expect_true(tracks0.value()[0].lifecycle == TrackLifecycle::kTentative,
                        "New tracks should start as tentative before enough hits.");

    std::vector<Measurement> frame1{
        Measurement{(Vector(2) << 1.0, 0.1).finished(), 1.0, "pos", "map"},
        Measurement{(Vector(2) << 11.0, 10.1).finished(), 1.0, "pos", "map"},
    };
    const auto tracks1 = tracker.step(
        frame1,
        ros_tracker::models::ModelContext{1.0, 1.0, "map"});
    context.expect_true(tracks1.ok(), "Tracker should predict and correct the second frame.");
    context.expect_true(tracks1.value().size() == 2U,
                        "Tracker should keep both tracks after the second frame.");
    context.expect_true(tracks1.value()[0].lifecycle == TrackLifecycle::kConfirmed &&
                            tracks1.value()[1].lifecycle == TrackLifecycle::kConfirmed,
                        "Tracks should be confirmed after two hits.");

    const TrackId lost_track_id = tracks1.value()[1].id;

    std::vector<Measurement> frame2{
        Measurement{(Vector(2) << 2.0, 0.0).finished(), 2.0, "pos", "map"},
    };
    const auto tracks2 = tracker.step(
        frame2,
        ros_tracker::models::ModelContext{1.0, 2.0, "map"});
    context.expect_true(tracks2.ok(), "Tracker should tolerate a missed detection.");
    context.expect_true(tracks2.value().size() == 2U,
                        "Tracker should keep a track alive after one missed detection.");

    std::vector<Measurement> frame3{
        Measurement{(Vector(2) << 2.0, 0.0).finished(), 3.0, "pos", "map"},
    };
    const auto tracks3 = tracker.step(
        frame3,
        ros_tracker::models::ModelContext{1.0, 3.0, "map"});
    context.expect_true(tracks3.ok(), "Tracker should process another frame after a miss.");
    context.expect_true(tracks3.value().size() == 1U,
                        "Tracker should prune a track after too many consecutive misses.");
    context.expect_true(tracks3.value()[0].id != lost_track_id,
                        "Tracker should prune the missed track instead of the active one.");
    context.expect_true(tracks3.value()[0].lifecycle == TrackLifecycle::kConfirmed,
                        "Remaining active track should stay confirmed.");

    std::vector<Measurement> inconsistent_frame{
        Measurement{(Vector(2) << 4.0, 0.0).finished(), 4.0, "pos", "map"},
        Measurement{(Vector(2) << 14.0, 10.0).finished(), 4.0, "pos", "odom"},
    };
    const auto inconsistent_tracks = tracker.step(
        inconsistent_frame,
        ros_tracker::models::ModelContext{1.0, 4.0, "map"});
    context.expect_true(!inconsistent_tracks.ok(),
                        "Tracker should reject a measurement batch with inconsistent frame_id values.");
}

/// Tests per-track dependency selection.
void test_multi_target_tracker_per_track_dependencies(TestContext &context)
{
    using namespace ros_tracker::core;
    using namespace ros_tracker::tracking;

    auto association = std::make_shared<NearestNeighborAssociationStrategy>(9.0);
    auto manager = std::make_shared<BasicTrackManager>(
        4,
        4.0 * Matrix::Identity(4, 4),
        std::vector<Index>{0, 1},
        1U,
        2U);
    auto handle_factory = std::make_shared<SensorAwareTrackHandleFactory>();

    MultiTargetTracker tracker(
        ros_tracker::tracking::test_support::make_default_handle(),
        association,
        manager,
        handle_factory);

    std::vector<Measurement> frame0{
        Measurement{(Vector(2) << 0.0, 0.0).finished(), 0.0, "kalman", "map"},
        Measurement{(Vector(2) << 10.0, 10.0).finished(), 0.0, "pf", "map"},
    };
    const auto tracks0 = tracker.step(
        frame0,
        ros_tracker::models::ModelContext{0.0, 0.0, "map"});
    context.expect_true(tracks0.ok(), "Tracker should initialize per-track dependencies.");
    context.expect_true(tracks0.value().size() == 2U,
                        "Tracker should initialize one track per measurement.");

    std::vector<Measurement> frame1{
        Measurement{(Vector(2) << 1.0, 0.0).finished(), 1.0, "kalman", "map"},
        Measurement{(Vector(2) << 11.0, 10.0).finished(), 1.0, "pf", "map"},
    };
    const auto tracks1 = tracker.step(
        frame1,
        ros_tracker::models::ModelContext{1.0, 1.0, "map"});
    context.expect_true(tracks1.ok(), "Tracker should update tracks with their own filters.");

    bool saw_particle_track = false;
    bool saw_kalman_track = false;
    for (const Track &track : tracks1.value())
    {
        if (track.source_sensor_id == "pf")
        {
            saw_particle_track = true;
            context.expect_true(track.handle && track.handle->name() == "particle_cv",
                                "Particle-filter track should keep its own track handle.");
            context.expect_true(track.estimate.particle_set.has_value(),
                                "Particle-filter track should keep its particle posterior.");
        }
        if (track.source_sensor_id == "kalman")
        {
            saw_kalman_track = true;
            context.expect_true(track.handle && track.handle->name() == "kalman_cv",
                                "Kalman track should keep its own track handle.");
            context.expect_true(!track.estimate.particle_set.has_value(),
                                "Kalman-filter track should remain Gaussian-only.");
        }
    }
    context.expect_true(saw_particle_track,
                        "Tracker should keep the particle-filter track in the active set.");
    context.expect_true(saw_kalman_track,
                        "Tracker should keep the Kalman-filter track in the active set.");
}

} // namespace

int main()
{
    TestContext context;
    test_multi_target_tracker_lifecycle(context);
    test_multi_target_tracker_per_track_dependencies(context);
    return ros_tracker::tracking::test_support::finish(
        context,
        "All tracker lifecycle tests passed.");
}
