#include "ros_tracker/apps/offline_examples.hpp"

#include <memory>
#include <vector>

#include "ros_tracker/filters/kalman_filters.hpp"
#include "ros_tracker/models/factories.hpp"
#include "ros_tracker/tracking/association_tools.hpp"
#include "ros_tracker/tracking/track_lifecycle.hpp"

namespace ros_tracker::apps::offline
{

    core::Result<ExampleRunSummary>
    run_single_target_kalman_example(const std::uint64_t seed)
    {
        using namespace ros_tracker::core;
        using namespace ros_tracker::filters;
        using namespace ros_tracker::models;
        using namespace ros_tracker::tracking;

        ConstantVelocityScenarioConfig scenario_config;
        scenario_config.initial_state =
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.5).finished(), 0.0, "map"};
        scenario_config.dt = 1.0;
        scenario_config.steps = 8U;
        scenario_config.process_noise_covariance = 0.01 * Matrix::Identity(4, 4);
        scenario_config.measurement_noise_covariance = 0.25 * Matrix::Identity(2, 2);
        scenario_config.sensor_id = "offline_sensor";
        scenario_config.frame_id = "map";

        const auto scenario =
            build_constant_velocity_position_scenario(scenario_config, seed);
        if (!scenario.ok())
        {
            return scenario.status();
        }

        auto filter = std::make_shared<KalmanFilter>();
        auto association =
            std::make_shared<NearestNeighborAssociationStrategy>(16.0);
        auto manager = std::make_shared<BasicTrackManager>(
            4,
            4.0 * Matrix::Identity(4, 4),
            std::vector<Index>{0, 1},
            1U,
            2U);

        MultiTargetTracker tracker(
            filter,
            make_constant_velocity_system(scenario_config.process_noise_covariance),
            make_position_sensor(scenario_config.measurement_noise_covariance),
            association,
            manager);

        ExampleRunSummary summary;
        summary.scenario = scenario.value();
        summary.tracker_frames.reserve(summary.scenario.measurement_frames.size());

        for (const MeasurementFrame &frame : summary.scenario.measurement_frames)
        {
            const auto tracks = tracker.step(
                frame.measurements,
                ModelContext{scenario_config.dt, frame.timestamp, scenario_config.frame_id});
            if (!tracks.ok())
            {
                return tracks.status();
            }

            summary.tracker_frames.push_back(TrackerFrame{frame.timestamp, tracks.value()});
        }

        summary.estimated_states =
            extract_primary_track_estimates(summary.tracker_frames);
        if (summary.estimated_states.size() != summary.scenario.truth_states.size())
        {
            return Status::internal_error(
                "Offline example expected one primary estimate per truth frame.");
        }

        const auto metrics = compute_tracking_metrics(
            summary.scenario.truth_states,
            summary.estimated_states);
        if (!metrics.ok())
        {
            return metrics.status();
        }

        summary.metrics = metrics.value();
        return summary;
    }

} // namespace ros_tracker::apps::offline
