#include "support.hpp"

#include <vector>

#include "ros_tracker/tracking/association_tools.hpp"

namespace
{

using ros_tracker::tracking::test_support::TestContext;
using ros_tracker::filters::GaussianEstimate;

/// Tests nearest-neighbor association strategies.
void test_nearest_neighbor_association(TestContext &context)
{
    using namespace ros_tracker::core;
    using namespace ros_tracker::tracking;

    NearestNeighborAssociationStrategy strategy(9.0);

    std::vector<Track> tracks;
    tracks.push_back(Track{
        1U,
        GaussianEstimate{
            State{(Vector(4) << 0.0, 0.0, 0.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        },
        ros_tracker::tracking::test_support::make_default_handle(),
        TrackLifecycle::kConfirmed,
        2U,
        2U,
        0U,
        0U,
        "",
    });
    tracks.push_back(Track{
        2U,
        GaussianEstimate{
            State{(Vector(4) << 10.0, 10.0, 0.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        },
        ros_tracker::tracking::test_support::make_default_handle(),
        TrackLifecycle::kConfirmed,
        2U,
        2U,
        0U,
        0U,
        "",
    });

    std::vector<Measurement> measurements{
        Measurement{(Vector(2) << 0.1, -0.2).finished(), 1.0, "pos", "map"},
        Measurement{(Vector(2) << 9.8, 10.1).finished(), 1.0, "pos", "map"},
    };

    const auto result = strategy.associate(
        tracks,
        measurements,
        ros_tracker::models::make_position_sensor(0.25 * Matrix::Identity(2, 2)),
        ros_tracker::models::ModelContext{1.0, 1.0, "map"});
    context.expect_true(result.ok(), "Nearest-neighbor association should succeed.");
    context.expect_true(result.value().matches.size() == 2U,
                        "Nearest-neighbor association should match both tracks.");
    context.expect_true(result.value().unmatched_tracks.empty(),
                        "Nearest-neighbor association should not leave unmatched tracks.");
    context.expect_true(result.value().unmatched_measurements.empty(),
                        "Nearest-neighbor association should not leave unmatched measurements.");
    bool found_first_pair = false;
    bool found_second_pair = false;
    for (const auto &match : result.value().matches)
    {
        found_first_pair |=
            (match.track_index == 0U && match.measurement_index == 0U);
        found_second_pair |=
            (match.track_index == 1U && match.measurement_index == 1U);
    }
    context.expect_true(found_first_pair,
                        "Nearest-neighbor association should pair the first track with the first measurement.");
    context.expect_true(found_second_pair,
                        "Nearest-neighbor association should pair the second track with the second measurement.");
}

/// Tests the global assignment solver interface.
void test_global_assignment_solver_interface(TestContext &context)
{
    using namespace ros_tracker::tracking;

    AssociationProblem problem;
    problem.track_count = 2U;
    problem.measurement_count = 2U;
    problem.candidates = {
        AssociationCandidate{0U, 0U, 1.0},
        AssociationCandidate{0U, 1U, 2.0},
        AssociationCandidate{1U, 0U, 1.5},
    };

    GreedyAssociationAssignmentSolver greedy_solver;
    const auto greedy = greedy_solver.solve(problem);
    context.expect_true(greedy.ok(), "Greedy assignment solver should succeed.");
    context.expect_true(greedy.value().matches.size() == 1U,
                        "Greedy assignment should settle for one match in this ambiguous problem.");

    OptimalAssociationAssignmentSolver optimal_solver;
    const auto optimal = optimal_solver.solve(problem);
    context.expect_true(optimal.ok(), "Optimal assignment solver should succeed.");
    context.expect_true(optimal.value().matches.size() == 2U,
                        "Optimal assignment should find the globally best two-match solution.");
}

} // namespace

int main()
{
    TestContext context;
    test_nearest_neighbor_association(context);
    test_global_assignment_solver_interface(context);
    return ros_tracker::tracking::test_support::finish(
        context,
        "All tracking association tests passed.");
}
