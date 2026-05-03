#pragma once

#include <cmath>

#include <Eigen/Eigenvalues>

#include "ros_tracker/filters/estimate.hpp"

namespace ros_tracker::filters
{
    class MerweSigmaPointGenerator
    {
    public:
        /// Constructs MerweSigmaPointGenerator.
        explicit MerweSigmaPointGenerator(
            const core::Scalar alpha = 1e-3,
            const core::Scalar beta = 2.0,
            const core::Scalar kappa = 0.0)
            : alpha_(alpha), beta_(beta), kappa_(kappa) {}

        /// Generates sigma points.
        [[nodiscard]] core::Result<SigmaPointSet> generate(
            const GaussianEstimate &estimate) const
        {
            const core::Status status = validate_estimate(estimate);
            if (!status.ok())
            {
                return status;
            }

            if (alpha_ <= 0.0)
            {
                return core::Status::invalid_argument(
                    "Merwe sigma point alpha must be positive.");
            }

            const core::Scalar n = static_cast<core::Scalar>(estimate.dimension());
            const core::Scalar lambda = alpha_ * alpha_ * (n + kappa_) - n;
            const core::Scalar scaling = n + lambda;
            if (scaling <= 0.0)
            {
                return core::Status::invalid_argument(
                    "Merwe sigma point scaling factor must be positive.");
            }

            Eigen::SelfAdjointEigenSolver<core::Covariance> solver(estimate.covariance);
            if (solver.info() != Eigen::Success)
            {
                return core::Status::numerical_error(
                    "Failed to compute covariance eigendecomposition for sigma points.");
            }

            const auto sqrt_eigenvalues =
                solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
            const core::Matrix sqrt_covariance =
                solver.eigenvectors() * sqrt_eigenvalues.asDiagonal();
            const core::Scalar gamma = std::sqrt(scaling);

            SigmaPointSet sigma_points;
            sigma_points.points =
                core::Matrix::Zero(estimate.dimension(), 2 * estimate.dimension() + 1);
            sigma_points.mean_weights =
                core::Vector::Constant(2 * estimate.dimension() + 1, 0.5 / scaling);
            sigma_points.covariance_weights = sigma_points.mean_weights;

            sigma_points.points.col(0) = estimate.state.value;
            sigma_points.mean_weights[0] = lambda / scaling;
            sigma_points.covariance_weights[0] =
                lambda / scaling + (1.0 - alpha_ * alpha_ + beta_);

            for (core::Index i = 0; i < estimate.dimension(); ++i)
            {
                const core::Vector offset = gamma * sqrt_covariance.col(i);
                sigma_points.points.col(i + 1) = estimate.state.value + offset;
                sigma_points.points.col(i + 1 + estimate.dimension()) =
                    estimate.state.value - offset;
            }

            return sigma_points;
        }

    private:
        core::Scalar alpha_;
        core::Scalar beta_;
        core::Scalar kappa_;
    };

    class CubaturePointGenerator
    {
    public:
        /// Generates sigma points.
        [[nodiscard]] core::Result<SigmaPointSet> generate(
            const GaussianEstimate &estimate) const
        {
            const core::Status status = validate_estimate(estimate);
            if (!status.ok())
            {
                return status;
            }

            Eigen::SelfAdjointEigenSolver<core::Covariance> solver(estimate.covariance);
            if (solver.info() != Eigen::Success)
            {
                return core::Status::numerical_error(
                    "Failed to compute covariance eigendecomposition for cubature points.");
            }

            const auto sqrt_eigenvalues =
                solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
            const core::Matrix sqrt_covariance =
                solver.eigenvectors() * sqrt_eigenvalues.asDiagonal();
            const core::Scalar radius =
                std::sqrt(static_cast<core::Scalar>(estimate.dimension()));

            SigmaPointSet sigma_points;
            sigma_points.points =
                core::Matrix::Zero(estimate.dimension(), 2 * estimate.dimension());
            sigma_points.mean_weights = core::Vector::Constant(
                2 * estimate.dimension(),
                0.5 / static_cast<core::Scalar>(estimate.dimension()));
            sigma_points.covariance_weights = sigma_points.mean_weights;

            for (core::Index i = 0; i < estimate.dimension(); ++i)
            {
                const core::Vector offset = radius * sqrt_covariance.col(i);
                sigma_points.points.col(i) = estimate.state.value + offset;
                sigma_points.points.col(i + estimate.dimension()) =
                    estimate.state.value - offset;
            }

            return sigma_points;
        }
    };

} // namespace ros_tracker::filters
