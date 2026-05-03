#pragma once

#include <vector>

#include "ros_tracker/apps/offline_sim.hpp"
#include "ros_tracker/apps/offline_tools.hpp"
#include "ros_tracker/filters/filter_primitives.hpp"

namespace ros_tracker::apps::offline
{

    struct ExampleRunSummary
    {
        SingleTargetScenario scenario;
        std::vector<TrackerFrame> tracker_frames;
        std::vector<filters::GaussianEstimate> estimated_states;
        TrackingMetrics metrics;
    };

    /// Runs the single-target Kalman filter example.
    [[nodiscard]] core::Result<ExampleRunSummary>
    run_single_target_kalman_example(std::uint64_t seed = 0U);

} // namespace ros_tracker::apps::offline
