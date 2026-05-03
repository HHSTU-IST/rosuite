#include "support.hpp"

#include "kracker/filters/particle_filters.hpp"
#include "kracker/models/factories.hpp"

namespace
{

    using kracker::filters::test_support::TestContext;

    /// Tests the particle filter.
    void test_particle_filter(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        ParticleFilter pf(256, 42U, 0.3);
        GaussianEstimate estimate{
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
            0.25 * Matrix::Identity(4, 4),
            std::nullopt,
        };

        const auto system =
            make_constant_velocity_system(0.1 * Matrix::Identity(4, 4));
        const ModelContext context_model{1.0, 1.0, "map"};
        const auto predicted = pf.predict(estimate, system, context_model);
        context.expect_true(predicted.ok(), "Particle filter prediction should succeed.");
        context.expect_near(predicted.value().state.value[0], 1.0, 0.25,
                            "Particle filter prediction mean should stay close to constant-velocity motion.");
        context.expect_true(predicted.value().particle_set.has_value(),
                            "Particle filter prediction should preserve a particle posterior.");

        Measurement measurement{
            (Vector(2) << 1.3, 0.05).finished(),
            1.0,
            "pos_sensor",
            "map",
        };
        const auto corrected =
            pf.correct(
                predicted.value(),
                make_position_sensor(0.25 * Matrix::Identity(2, 2)),
                measurement);
        context.expect_true(corrected.ok(), "Particle filter correction should succeed.");
        context.expect_true(corrected.value().state.value[0] > predicted.value().state.value[0],
                            "Particle filter correction should move the estimate toward the measurement.");
        context.expect_true(corrected.value().particle_set.has_value(),
                            "Particle filter correction should keep the resampled particles.");
    }

} // namespace

int main()
{
    TestContext context;
    test_particle_filter(context);
    return kracker::filters::test_support::finish(
        context,
        "All particle-filter tests passed.");
}
