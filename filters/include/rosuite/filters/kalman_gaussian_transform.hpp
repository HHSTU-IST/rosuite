#pragma once

#include <optional>

#include <Eigen/Cholesky>

#include "rosuite/filters/estimate.hpp"
#include "rosuite/models/base.hpp"

namespace rosuite::filters::detail
{
    using core::Covariance;
    using core::Matrix;
    using core::Result;
    using core::Status;
    using core::Vector;

    /// Validates a sigma-point set.
    [[nodiscard]] Status validate_sigma_point_set(
        const SigmaPointSet &sigma_points,
        const core::Index dimension);

    /// Computes a weighted mean from column samples.
    [[nodiscard]] Result<Vector> weighted_mean_from_columns(
        const Matrix &samples,
        const Vector &weights);

    /// Computes a weighted covariance from column samples.
    [[nodiscard]] Result<Covariance> weighted_covariance_from_columns(
        const Matrix &samples,
        const Vector &weights,
        const Vector &mean);

    /// Computes a weighted cross-covariance from column samples.
    [[nodiscard]] Result<Matrix> weighted_cross_covariance_from_columns(
        const Matrix &left_samples,
        const Vector &left_mean,
        const Matrix &right_samples,
        const Vector &right_mean,
        const Vector &weights);

    /// Propagates sigma points through the motion model.
    [[nodiscard]] Result<GaussianEstimate> predict_from_sigma_points(
        const GaussianEstimate &estimate,
        const SigmaPointSet &sigma_points,
        const models::DynamicSystemModel &model,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt);

    /// Corrects an estimate from transformed sigma points.
    [[nodiscard]] Result<GaussianEstimate> correct_from_sigma_points(
        const GaussianEstimate &estimate,
        const SigmaPointSet &sigma_points,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {});

    /// Builds a Gaussian estimate from an ensemble.
    [[nodiscard]] Result<GaussianEstimate> estimate_from_ensemble(
        const Matrix &ensemble,
        const core::Scalar timestamp,
        const std::string &frame_id);

} // namespace rosuite::filters::detail
