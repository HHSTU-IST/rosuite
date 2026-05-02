#pragma once

#include <optional>

#include <Eigen/Cholesky>

#include "ros_tracker/filters/estimate.hpp"

namespace ros_tracker::filters {

struct LeastSquaresProblem {
  core::Matrix design_matrix;
  core::Vector observation_vector;
  std::optional<core::Covariance> observation_covariance;
  std::optional<core::Vector> prior_mean;
  std::optional<core::Covariance> prior_covariance;
};

class LeastSquaresEstimator {
 public:
  [[nodiscard]] core::Result<LeastSquaresEstimate> solve(
      const LeastSquaresProblem& problem) const {
    using core::Covariance;
    using core::Matrix;
    using core::Result;
    using core::Status;
    using core::Vector;

    if (problem.design_matrix.rows() != problem.observation_vector.size()) {
      return Status::dimension_mismatch(
          "Least squares observation vector size must match the number of design rows.");
    }

    if (problem.design_matrix.cols() <= 0) {
      return Status::invalid_argument(
          "Least squares design matrix must have at least one column.");
    }

    Matrix information =
        Matrix::Zero(problem.design_matrix.cols(), problem.design_matrix.cols());
    Vector rhs = Vector::Zero(problem.design_matrix.cols());

    if (problem.observation_covariance.has_value()) {
      const Covariance& covariance = *problem.observation_covariance;
      if (covariance.rows() != problem.observation_vector.size() ||
          covariance.cols() != problem.observation_vector.size()) {
        return Status::dimension_mismatch(
            "Observation covariance dimension must match the observation vector.");
      }

      const Status covariance_status = core::validate_covariance(covariance);
      if (!covariance_status.ok()) {
        return covariance_status;
      }

      Eigen::LDLT<Covariance> ldlt(covariance);
      if (ldlt.info() != Eigen::Success) {
        return Status::numerical_error(
            "Failed to factorize observation covariance.");
      }

      const Matrix whitened_design = ldlt.solve(problem.design_matrix);
      const Vector whitened_observations = ldlt.solve(problem.observation_vector);
      information = problem.design_matrix.transpose() * whitened_design;
      rhs = problem.design_matrix.transpose() * whitened_observations;
    } else {
      information = problem.design_matrix.transpose() * problem.design_matrix;
      rhs = problem.design_matrix.transpose() * problem.observation_vector;
    }

    if (problem.prior_mean.has_value() != problem.prior_covariance.has_value()) {
      return Status::invalid_argument(
          "Least squares prior mean and prior covariance must be provided together.");
    }

    if (problem.prior_mean.has_value()) {
      const Vector& prior_mean = *problem.prior_mean;
      const Covariance& prior_covariance = *problem.prior_covariance;
      if (prior_mean.size() != problem.design_matrix.cols() ||
          prior_covariance.rows() != problem.design_matrix.cols() ||
          prior_covariance.cols() != problem.design_matrix.cols()) {
        return Status::dimension_mismatch(
            "Prior mean and covariance dimensions must match the state dimension.");
      }

      const Status covariance_status = core::validate_covariance(prior_covariance);
      if (!covariance_status.ok()) {
        return covariance_status;
      }

      Eigen::LDLT<Covariance> ldlt(prior_covariance);
      if (ldlt.info() != Eigen::Success) {
        return Status::numerical_error(
            "Failed to factorize prior covariance.");
      }

      information += ldlt.solve(
          Matrix::Identity(prior_covariance.rows(), prior_covariance.cols()));
      rhs += ldlt.solve(prior_mean);
    }

    information = core::symmetrize(information);
    Eigen::LDLT<Covariance> info_ldlt(information);
    if (info_ldlt.info() != Eigen::Success) {
      return Status::numerical_error(
          "Failed to factorize least squares information matrix.");
    }

    LeastSquaresEstimate estimate;
    estimate.solution = info_ldlt.solve(rhs);
    estimate.covariance = core::symmetrize(info_ldlt.solve(
        Matrix::Identity(information.rows(), information.cols())));
    return estimate;
  }

  [[nodiscard]] std::string_view name() const noexcept {
    return "lsq";
  }
};

}  // namespace ros_tracker::filters
