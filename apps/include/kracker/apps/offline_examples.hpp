#pragma once

#include <cstdint>
#include <vector>

#include "kracker/apps/offline_sim.hpp"
#include "kracker/apps/offline_tools.hpp"
#include "kracker/filters/filter_primitives.hpp"

namespace kracker::apps::offline
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

} // namespace kracker::apps::offline
