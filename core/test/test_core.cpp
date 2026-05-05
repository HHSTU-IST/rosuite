#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rosuite/core/core.hpp"

namespace
{
  int g_failures = 0;

  /// Asserts that a condition is true.
  void expect_true(const bool condition, const std::string &message)
  {
    if (!condition)
    {
      std::cerr << "FAIL: " << message << '\n';
      ++g_failures;
    }
  }

  /// Asserts that two scalar values are close.
  void expect_near(
      const rosuite::core::Scalar actual,
      const rosuite::core::Scalar expected,
      const rosuite::core::Scalar tolerance,
      const std::string &message)
  {
    expect_true(std::abs(actual - expected) <= tolerance, message);
  }

  /// Tests the core type and status helpers.
  void test_types_and_status()
  {
    using namespace rosuite::core;

    State state{Vector::LinSpaced(3, 1.0, 3.0), 1.25, "map"};
    Measurement measurement{Vector::LinSpaced(2, 4.0, 5.0), 2.0, "radar", "map"};
    ControlInput control{Vector::Constant(2, 0.5), 3.0};

    expect_true(state.dimension() == 3, "State dimension should match vector size.");
    expect_true(measurement.dimension() == 2,
                "Measurement dimension should match vector size.");
    expect_true(control.dimension() == 2,
                "Control input dimension should match vector size.");

    const Status ok = Status::ok_status();
    const Status error = Status::invalid_argument("bad input");
    expect_true(ok.ok(), "OK status should report success.");
    expect_true(!error.ok(), "Error status should report failure.");

    Result<int> success(7);
    Result<int> failure(Status::out_of_range("bad range"));
    Result<int> invalid_ok(Status::ok_status());
    expect_true(success.ok() && success.value() == 7, "Result should hold a value.");
    expect_true(!failure.ok(), "Failed result should report its status.");
    expect_true(static_cast<bool>(success), "Successful Result should convert to true.");
    expect_true(success.has_value(), "Successful Result should report that it has a value.");
    expect_true(!failure.has_value(), "Failed Result should report that it has no value.");
    expect_true(!invalid_ok.ok(),
                "Constructing Result from an OK status without a value should be rejected.");
    bool threw = false;
    try
    {
      static_cast<void>(failure.value());
    }
    catch (const std::logic_error &)
    {
      threw = true;
    }
    expect_true(threw, "Accessing value() on a failed Result should throw a logic_error.");
  }

  /// Tests the linear-algebra helpers.
  void test_linear_algebra()
  {
    using namespace rosuite::core;

    const Matrix matrix = (Matrix(2, 2) << 1.0, 3.0, 5.0, 2.0).finished();
    const Matrix symmetric = symmetrize(matrix);

    expect_true(is_square(matrix), "2x2 matrix should be square.");
    expect_true(is_symmetric(symmetric), "Symmetrized matrix should be symmetric.");

    const Vector variances = (Vector(2) << 1.0, 4.0).finished();
    const auto covariance = diagonal_covariance(variances);
    expect_true(covariance.ok(), "Diagonal covariance should be created for non-negative variances.");
    expect_true(validate_covariance(covariance.value()).ok(),
                "Diagonal covariance should validate successfully.");
  }

  /// Tests the integration and quadrature helpers.
  void test_integration_and_quadrature()
  {
    using namespace rosuite::core;

    Vector x0(1);
    x0 << 1.0;

    const Vector x1 = numerics::runge_kutta_4(
        x0,
        0.0,
        0.1,
        [](const Scalar /*time*/, const Vector &state)
        { return state; });
    expect_near(x1[0], std::exp(0.1), 1e-6,
                "RK4 should accurately integrate dx/dt = x over a short step.");

    const auto trap = numerics::composite_trapezoidal(
        [](const Scalar x)
        { return x * x; }, 0.0, 1.0, 100U);
    expect_true(trap.ok(), "Trapezoidal integration should succeed.");
    expect_near(trap.value(), 1.0 / 3.0, 1e-4,
                "Trapezoidal integration should approximate integral of x^2.");

    const auto simpson = numerics::composite_simpson(
        [](const Scalar x)
        { return x * x; }, 0.0, 1.0, 100U);
    expect_true(simpson.ok(), "Simpson integration should succeed.");
    expect_near(simpson.value(), 1.0 / 3.0, 1e-10,
                "Simpson integration should accurately integrate quadratic polynomials.");
  }

  /// Tests the statistics helpers.
  void test_statistics()
  {
    using namespace rosuite::core;

    const std::vector<Vector> samples = {
        (Vector(2) << 0.0, 0.0).finished(),
        (Vector(2) << 2.0, 2.0).finished(),
    };
    const std::vector<Scalar> weights = {0.25, 0.75};

    const auto mean = stats::weighted_mean(samples, weights);
    expect_true(mean.ok(), "Weighted mean should succeed.");
    expect_near(mean.value()[0], 1.5, 1e-12, "Weighted mean x-coordinate should match expectation.");
    expect_near(mean.value()[1], 1.5, 1e-12, "Weighted mean y-coordinate should match expectation.");

    const auto covariance = stats::weighted_covariance(samples, weights);
    expect_true(covariance.ok(), "Weighted covariance should succeed.");
    expect_true(is_symmetric(covariance.value()),
                "Weighted covariance should be symmetric.");

    const Vector mean_zero = Vector::Zero(2);
    const Vector sample = (Vector(2) << 1.0, 2.0).finished();
    const Covariance identity = Matrix::Identity(2, 2);

    const auto distance = stats::mahalanobis_distance_squared(sample, mean_zero, identity);
    expect_true(distance.ok(), "Mahalanobis distance should succeed.");
    expect_near(distance.value(), 5.0, 1e-12,
                "Mahalanobis distance should match Euclidean norm under identity covariance.");

    const auto log_likelihood =
        stats::gaussian_log_likelihood(sample, mean_zero, identity);
    expect_true(log_likelihood.ok(), "Gaussian log-likelihood should succeed.");
  }

  /// Tests the random-sampling and resampling helpers.
  void test_random_and_resampling()
  {
    using namespace rosuite::core;

    stats::RandomEngine rng(42U);
    expect_near(rng.sample_normal(3.0, 0.0), 3.0, 1e-12,
                "Zero-variance normal samples should equal the mean.");

    const Vector mean = (Vector(2) << 1.0, -1.0).finished();
    const Covariance zero_covariance = Matrix::Zero(2, 2);
    const auto sample = rng.sample_multivariate_normal(mean, zero_covariance);
    expect_true(sample.ok(), "Sampling with zero covariance should succeed.");
    expect_true(sample.value().isApprox(mean, 1e-12),
                "Zero covariance multivariate normal should return the mean.");

    const auto indices = utils::systematic_resample({0.1, 0.2, 0.7}, 0.2);
    expect_true(indices.ok(), "Systematic resampling should succeed.");
    expect_true(indices->size() == 3, "Resampling should return one index per weight.");
    expect_true((*indices)[0] == 0 && (*indices)[1] == 2 && (*indices)[2] == 2,
                "Systematic resampling should be deterministic for a fixed offset.");
  }

} // namespace

/// Runs the test executable.
int main()
{
  test_types_and_status();
  test_linear_algebra();
  test_integration_and_quadrature();
  test_statistics();
  test_random_and_resampling();

  if (g_failures != 0)
  {
    std::cerr << g_failures << " test(s) failed.\n";
    return EXIT_FAILURE;
  }

  std::cout << "All core tests passed.\n";
  return EXIT_SUCCESS;
}
