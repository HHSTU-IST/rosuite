#include "support.hpp"

#include "ros_tracker/filters/filters.hpp"

namespace
{

using ros_tracker::filters::test_support::TestContext;

/// Tests the least-squares estimator.
void test_least_squares(TestContext &context)
{
    using namespace ros_tracker::core;
    using namespace ros_tracker::filters;

    LeastSquaresEstimator estimator;
    LeastSquaresProblem problem;
    problem.design_matrix = (Matrix(3, 2) << 1.0, 0.0,
                             0.0, 1.0,
                             1.0, 1.0)
                                .finished();
    problem.observation_vector = (Vector(3) << 1.0, 2.0, 3.0).finished();
    problem.observation_covariance = Matrix::Identity(3, 3);

    const auto solution = estimator.solve(problem);
    context.expect_true(solution.ok(), "Least squares solution should succeed.");
    context.expect_near(solution.value().solution[0], 1.0, 1e-12,
                        "Least squares should recover the first state component.");
    context.expect_near(solution.value().solution[1], 2.0, 1e-12,
                        "Least squares should recover the second state component.");
}

/// Tests sigma-point generation utilities.
void test_sigma_points(TestContext &context)
{
    using namespace ros_tracker::core;
    using namespace ros_tracker::filters;

    GaussianEstimate estimate{
        State{(Vector(2) << 1.0, -2.0).finished(), 0.0, "map"},
        (Matrix(2, 2) << 4.0, 1.0,
         1.0, 2.0)
            .finished(),
    };

    MerweSigmaPointGenerator generator(0.5, 2.0, 0.0);
    const auto sigma_points = generator.generate(estimate);
    context.expect_true(sigma_points.ok(), "Sigma point generation should succeed.");

    Vector mean = Vector::Zero(2);
    for (Index i = 0; i < sigma_points.value().points.cols(); ++i)
    {
        mean += sigma_points.value().mean_weights[i] *
                sigma_points.value().points.col(i);
    }
    context.expect_true(mean.isApprox(estimate.state.value, 1e-10),
                        "Sigma points should reconstruct the original mean.");
}

/// Tests the Rauch-Tung-Striebel smoother.
void test_rts_smoother(TestContext &context)
{
    using namespace ros_tracker::core;
    using namespace ros_tracker::filters;

    GaussianEstimate filtered{
        State{(Vector(2) << 0.0, 1.0).finished(), 0.0, "map"},
        Matrix::Identity(2, 2),
    };
    GaussianEstimate predicted_next{
        State{(Vector(2) << 1.0, 1.0).finished(), 1.0, "map"},
        2.0 * Matrix::Identity(2, 2),
    };
    GaussianEstimate smoothed_next{
        State{(Vector(2) << 0.8, 0.9).finished(), 1.0, "map"},
        0.5 * Matrix::Identity(2, 2),
    };
    Matrix transition = Matrix::Identity(2, 2);

    RauchTungStriebelSmoother smoother;
    const auto smoothed =
        smoother.smooth_step(filtered, predicted_next, smoothed_next, transition);
    context.expect_true(smoothed.ok(), "RTS smoothing step should succeed.");
    context.expect_true(smoothed.value().covariance.trace() <= filtered.covariance.trace(),
                        "RTS smoothing should not increase covariance trace in this setup.");
}

} // namespace

int main()
{
    TestContext context;
    test_least_squares(context);
    test_sigma_points(context);
    test_rts_smoother(context);
    return ros_tracker::filters::test_support::finish(
        context,
        "All filter estimation-utility tests passed.");
}
