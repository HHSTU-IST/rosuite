#pragma once

#include <Eigen/Cholesky>

#include "kracker/filters/estimate.hpp"

namespace kracker::filters
{
  class RauchTungStriebelSmoother
  {
  public:
    /// Smooths step.
    [[nodiscard]] core::Result<GaussianEstimate> smooth_step(
        const GaussianEstimate &filtered,
        const GaussianEstimate &predicted_next,
        const GaussianEstimate &smoothed_next,
        const core::Matrix &transition_matrix) const
    {
      const core::Status filtered_status = validate_estimate(filtered);
      if (!filtered_status.ok())
      {
        return filtered_status;
      }

      const core::Status predicted_status = validate_estimate(predicted_next);
      if (!predicted_status.ok())
      {
        return predicted_status;
      }

      const core::Status smoothed_status = validate_estimate(smoothed_next);
      if (!smoothed_status.ok())
      {
        return smoothed_status;
      }

      if (filtered.dimension() != predicted_next.dimension() ||
          filtered.dimension() != smoothed_next.dimension())
      {
        return core::Status::dimension_mismatch(
            "RTS smoother requires matching estimate dimensions.");
      }

      if (transition_matrix.rows() != filtered.dimension() ||
          transition_matrix.cols() != filtered.dimension())
      {
        return core::Status::dimension_mismatch(
            "RTS smoother transition matrix dimension must match the state dimension.");
      }

      Eigen::LDLT<core::Covariance> ldlt(predicted_next.covariance);
      if (ldlt.info() != Eigen::Success)
      {
        return core::Status::numerical_error(
            "Failed to factorize predicted covariance for RTS smoother.");
      }

      const core::Matrix smoother_gain =
          filtered.covariance * transition_matrix.transpose() *
          ldlt.solve(core::Matrix::Identity(
              predicted_next.dimension(), predicted_next.dimension()));

      GaussianEstimate result = filtered;
      result.state.value += smoother_gain *
                            (smoothed_next.state.value - predicted_next.state.value);
      result.covariance = core::symmetrize(
          filtered.covariance +
          smoother_gain *
              (smoothed_next.covariance - predicted_next.covariance) *
              smoother_gain.transpose());
      result.state.timestamp = smoothed_next.state.timestamp;
      return result;
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept
    {
      return "rts";
    }
  };

} // namespace kracker::filters
