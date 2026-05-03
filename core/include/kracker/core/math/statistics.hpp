#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>

#include "kracker/core/math/linear_algebra.hpp"

namespace kracker::core::stats
{
  /// Normalizes a weight vector in place.
  [[nodiscard]] inline Result<Scalar> normalize_weights_in_place(
      std::vector<Scalar> &weights)
  {
    if (weights.empty())
    {
      return Status::invalid_argument("Weights must not be empty.");
    }

    Scalar sum = Scalar{0.0};
    for (const Scalar weight : weights)
    {
      if (weight < Scalar{0.0})
      {
        return Status::invalid_argument("Weights must be non-negative.");
      }
      sum += weight;
    }

    if (sum <= Scalar{0.0})
    {
      return Status::invalid_argument("Weight sum must be positive.");
    }

    for (Scalar &weight : weights)
    {
      weight /= sum;
    }

    return sum;
  }

  /// Converts log-weights into normalized linear weights.
  [[nodiscard]] inline Result<std::vector<Scalar>> normalize_log_weights(
      const std::vector<Scalar> &log_weights)
  {
    if (log_weights.empty())
    {
      return Status::invalid_argument("Log-weights must not be empty.");
    }

    const auto max_it = std::max_element(log_weights.begin(), log_weights.end());
    const Scalar max_log_weight = *max_it;

    std::vector<Scalar> weights;
    weights.reserve(log_weights.size());
    for (const Scalar log_weight : log_weights)
    {
      weights.push_back(std::exp(log_weight - max_log_weight));
    }

    const auto normalization = normalize_weights_in_place(weights);
    if (!normalization.ok())
    {
      return normalization.status();
    }

    return weights;
  }

  /// Computes the weighted mean of a sample set.
  [[nodiscard]] inline Result<Vector> weighted_mean(
      const std::vector<Vector> &samples,
      std::vector<Scalar> weights)
  {
    if (samples.empty())
    {
      return Status::invalid_argument("Samples must not be empty.");
    }

    if (samples.size() != weights.size())
    {
      return Status::dimension_mismatch(
          "Sample and weight counts must match.");
    }

    const Index dimension = samples.front().size();
    for (const Vector &sample : samples)
    {
      if (sample.size() != dimension)
      {
        return Status::dimension_mismatch(
            "All samples must have the same dimension.");
      }
    }

    const auto normalization = normalize_weights_in_place(weights);
    if (!normalization.ok())
    {
      return normalization.status();
    }

    Vector mean = Vector::Zero(dimension);
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
      mean += weights[i] * samples[i];
    }

    return mean;
  }

  /// Computes the weighted covariance of a sample set.
  [[nodiscard]] inline Result<Covariance> weighted_covariance(
      const std::vector<Vector> &samples,
      std::vector<Scalar> weights)
  {
    const auto mean_result = weighted_mean(samples, weights);
    if (!mean_result.ok())
    {
      return mean_result.status();
    }

    const auto normalization = normalize_weights_in_place(weights);
    if (!normalization.ok())
    {
      return normalization.status();
    }

    const Vector &mean = mean_result.value();
    Covariance covariance = Covariance::Zero(mean.size(), mean.size());

    for (std::size_t i = 0; i < samples.size(); ++i)
    {
      const Vector delta = samples[i] - mean;
      covariance += weights[i] * (delta * delta.transpose());
    }

    return symmetrize(covariance);
  }

  /// Computes the squared Mahalanobis distance.
  [[nodiscard]] inline Result<Scalar> mahalanobis_distance_squared(
      const Vector &sample,
      const Vector &mean,
      const Covariance &covariance)
  {
    if (sample.size() != mean.size())
    {
      return Status::dimension_mismatch(
          "Sample and mean vectors must have the same dimension.");
    }

    if (covariance.rows() != sample.size() || covariance.cols() != sample.size())
    {
      return Status::dimension_mismatch(
          "Covariance matrix dimension must match the vector dimension.");
    }

    const Status status = validate_covariance(covariance);
    if (!status.ok())
    {
      return status;
    }

    Eigen::LDLT<Covariance> ldlt(covariance);
    if (ldlt.info() != Eigen::Success)
    {
      return Status::numerical_error(
          "Failed to factorize covariance matrix for Mahalanobis distance.");
    }

    const Vector delta = sample - mean;
    return delta.dot(ldlt.solve(delta));
  }

  /// Evaluates the Gaussian log-likelihood of a sample.
  [[nodiscard]] inline Result<Scalar> gaussian_log_likelihood(
      const Vector &sample,
      const Vector &mean,
      const Covariance &covariance)
  {
    const auto mahalanobis = mahalanobis_distance_squared(sample, mean, covariance);
    if (!mahalanobis.ok())
    {
      return mahalanobis.status();
    }

    Eigen::LDLT<Covariance> ldlt(covariance);
    if (ldlt.info() != Eigen::Success)
    {
      return Status::numerical_error(
          "Failed to factorize covariance matrix for Gaussian log-likelihood.");
    }

    const auto &diagonal = ldlt.vectorD();
    if ((diagonal.array() <= Scalar{0.0}).any())
    {
      return Status::invalid_argument(
          "Gaussian log-likelihood requires a positive definite covariance.");
    }

    const Scalar log_determinant = diagonal.array().log().sum();
    const Scalar dimension = static_cast<Scalar>(sample.size());
    constexpr Scalar kPi = 3.14159265358979323846;
    const Scalar normalization =
        dimension * std::log(Scalar{2.0} * kPi) + log_determinant;

    return Scalar{-0.5} * (normalization + mahalanobis.value());
  }

} // namespace kracker::core::stats
