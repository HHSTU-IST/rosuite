#pragma once

#include <Eigen/Dense>

#include "kracker/core/result.hpp"

namespace kracker::core
{
  using Scalar = double;
  using Vector = Eigen::VectorXd;
  using Matrix = Eigen::MatrixXd;
  using Covariance = Eigen::MatrixXd;
  using Index = Eigen::Index;

  /// Returns whether a matrix is square.
  [[nodiscard]] inline bool is_square(const Matrix &matrix) noexcept
  {
    return matrix.rows() == matrix.cols();
  }

  /// Returns whether a matrix is symmetric.
  [[nodiscard]] inline bool is_symmetric(
      const Matrix &matrix,
      const Scalar tolerance = 1e-9) noexcept
  {
    if (!is_square(matrix))
    {
      return false;
    }

    return matrix.isApprox(matrix.transpose(), tolerance);
  }

  /// Symmetrizes a square matrix.
  [[nodiscard]] inline Matrix symmetrize(const Matrix &matrix)
  {
    return Scalar{0.5} * (matrix + matrix.transpose());
  }

  /// Validates a covariance matrix.
  [[nodiscard]] inline Status validate_covariance(
      const Covariance &covariance,
      const Scalar tolerance = 1e-9)
  {
    if (!is_square(covariance))
    {
      return Status::dimension_mismatch("Covariance matrix must be square.");
    }

    if (!is_symmetric(covariance, tolerance))
    {
      return Status::invalid_argument("Covariance matrix must be symmetric.");
    }

    Eigen::SelfAdjointEigenSolver<Covariance> solver(covariance);
    if (solver.info() != Eigen::Success)
    {
      return Status::numerical_error(
          "Failed to compute covariance eigenvalues.");
    }

    const auto min_eigenvalue = solver.eigenvalues().minCoeff();
    if (min_eigenvalue < -tolerance)
    {
      return Status::invalid_argument(
          "Covariance matrix must be positive semi-definite.");
    }

    return Status::ok_status();
  }

  /// Builds a diagonal covariance matrix from variances.
  [[nodiscard]] inline Result<Covariance> diagonal_covariance(
      const Vector &variances)
  {
    if ((variances.array() < Scalar{0.0}).any())
    {
      return Status::invalid_argument(
          "Diagonal covariance variances must be non-negative.");
    }

    return variances.asDiagonal().toDenseMatrix();
  }

} // namespace kracker::core
