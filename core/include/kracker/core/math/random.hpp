#pragma once

#include <cstdint>
#include <random>

#include <Eigen/Cholesky>

#include "kracker/core/math/linear_algebra.hpp"

namespace kracker::core::stats
{
  class RandomEngine
  {
  public:
    /// Constructs RandomEngine.
    explicit RandomEngine(const std::uint64_t seed = 0U) : engine_(seed) {}

    /// Reseeds the random engine.
    void seed(const std::uint64_t value)
    {
      engine_.seed(value);
    }

    /// Samples a value from a uniform distribution.
    [[nodiscard]] Scalar sample_uniform(
        const Scalar lower = Scalar{0.0},
        const Scalar upper = Scalar{1.0})
    {
      std::uniform_real_distribution<Scalar> distribution(lower, upper);
      return distribution(engine_);
    }

    /// Samples a value from a normal distribution.
    [[nodiscard]] Scalar sample_normal(
        const Scalar mean = Scalar{0.0},
        const Scalar standard_deviation = Scalar{1.0})
    {
      std::normal_distribution<Scalar> distribution(mean, standard_deviation);
      return distribution(engine_);
    }

    /// Samples a vector from a multivariate normal distribution.
    [[nodiscard]] Result<Vector> sample_multivariate_normal(
        const Vector &mean,
        const Covariance &covariance)
    {
      if (covariance.rows() != mean.size() || covariance.cols() != mean.size())
      {
        return Status::dimension_mismatch(
            "Mean vector and covariance matrix dimensions do not match.");
      }

      const Status status = validate_covariance(covariance);
      if (!status.ok())
      {
        return status;
      }

      if (covariance.isZero(1e-12))
      {
        return mean;
      }

      Eigen::LLT<Covariance> llt(covariance);
      if (llt.info() != Eigen::Success)
      {
        return Status::numerical_error(
            "Failed to compute Cholesky factor for covariance matrix.");
      }

      Vector standard_normal(mean.size());
      for (Index i = 0; i < mean.size(); ++i)
      {
        standard_normal[i] = sample_normal();
      }

      return Vector(mean + llt.matrixL() * standard_normal);
    }

  private:
    std::mt19937_64 engine_;
  };

} // namespace kracker::core::stats
