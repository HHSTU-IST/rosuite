#pragma once

#include <cstdint>
#include <vector>

#include "rosuite/apps/offline_sim.hpp"
#include "rosuite/apps/offline_tools.hpp"
#include "rosuite/filters/filter_primitives.hpp"

namespace rosuite::apps::offline
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

} // namespace rosuite::apps::offline
