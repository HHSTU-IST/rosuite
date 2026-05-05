#include "support.hpp"

#include <cmath>
#include <vector>

#include "rosuite/tracking/association_tools.hpp"
#include "rosuite/tracking/model_multi_tools.hpp"

namespace
{

    using rosuite::filters::GaussianEstimate;
    using rosuite::tracking::test_support::TestContext;

    /// Tests the interacting multiple-model estimator.
    void test_multi_model_estimator(TestContext &context)
    {
        using namespace rosuite::core;
        using namespace rosuite::filters;
        using namespace rosuite::tracking;

        auto shared_filter = std::make_shared<KalmanFilter>();
        const auto low_q_system =
            rosuite::models::make_constant_velocity_system(
                0.01 * Matrix::Identity(4, 4));
        const auto high_q_system =
            rosuite::models::make_constant_velocity_system(
                0.5 * Matrix::Identity(4, 4));

        std::vector<ModelBankEntry> model_bank{
            ModelBankEntry{
                "low_q",
                low_q_system,
                shared_filter,
            },
            ModelBankEntry{
                "high_q",
                high_q_system,
                shared_filter,
            },
        };

        Matrix transition(2, 2);
        transition << 0.95, 0.05,
            0.10, 0.90;
        InteractingMultipleModelEstimator imm(model_bank, transition);

        GaussianEstimate seed{
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        };
        const auto initialized = imm.initialize(seed, {0.8, 0.2});
        context.expect_true(initialized.ok(), "IMM initialization should succeed.");
        context.expect_true(initialized.value().modes.size() == 2U,
                            "IMM should keep one hypothesis per model.");

        Measurement measurement{
            (Vector(2) << 1.1, -0.05).finished(),
            1.0,
            "pos",
            "map",
        };
        const auto updated = imm.step(
            initialized.value(),
            rosuite::models::make_position_sensor(0.25 * Matrix::Identity(2, 2)),
            measurement,
            rosuite::models::ModelContext{1.0, 1.0, "map"});
        context.expect_true(updated.ok(), "IMM predict-correct step should succeed.");
        context.expect_true(updated.value().merged_estimate.state.value[0] > 0.5,
                            "IMM merged estimate should move toward the new measurement.");

        Scalar probability_sum = 0.0;
        for (const auto &mode : updated.value().modes)
        {
            probability_sum += mode.probability;
        }
        context.expect_true(std::abs(probability_sum - 1.0) < 1e-9,
                            "IMM mode probabilities should stay normalized.");
    }

    /// Tests covariance-intersection fusion.
    void test_covariance_intersection_fusion(TestContext &context)
    {
        using namespace rosuite::core;
        using namespace rosuite::tracking;

        CovarianceIntersectionFuser fuser;
        GaussianEstimate estimate_a{
            State{(Vector(4) << 1.0, 0.0, 0.5, 0.0).finished(), 1.0, "map"},
            1.5 * Matrix::Identity(4, 4),
            std::nullopt,
        };
        GaussianEstimate estimate_b{
            State{(Vector(4) << 1.4, 0.2, 0.4, 0.0).finished(), 1.2, "map"},
            0.8 * Matrix::Identity(4, 4),
            std::nullopt,
        };

        const auto fused = fuser.fuse_pair(estimate_a, estimate_b);
        context.expect_true(fused.ok(), "Covariance intersection should succeed.");
        context.expect_true(std::abs(fused.value().state.value[0] - estimate_b.state.value[0]) <=
                                std::abs(estimate_a.state.value[0] - estimate_b.state.value[0]) + 1e-9,
                            "Covariance intersection mean should stay no farther from the more certain estimate than the inputs are from each other.");
        context.expect_true(fused.value().covariance.trace() <= estimate_a.covariance.trace() + 1e-9,
                            "Covariance intersection should not be looser than the worst input covariance.");
        context.expect_true(fused.value().covariance.trace() <= estimate_b.covariance.trace() + 1e-9,
                            "Covariance intersection should stay conservative relative to the best input covariance.");
    }

} // namespace

int main()
{
    TestContext context;
    test_multi_model_estimator(context);
    test_covariance_intersection_fusion(context);
    return rosuite::tracking::test_support::finish(
        context,
        "All multi-model tracking tests passed.");
}
