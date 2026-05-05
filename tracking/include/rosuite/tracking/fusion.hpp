#pragma once

#include <limits>
#include <string_view>
#include <vector>

#include <Eigen/Cholesky>

#include "rosuite/tracking/base.hpp"

namespace rosuite::tracking
{
  class CovarianceIntersectionFuser
  {
  public:
    /// Constructs CovarianceIntersectionFuser.
    explicit CovarianceIntersectionFuser(
        const core::Index omega_steps = 21)
        : omega_steps_(omega_steps) {}

    /// Fuses a pair of Gaussian estimates.
    [[nodiscard]] core::Result<filters::GaussianEstimate> fuse_pair(
        const filters::GaussianEstimate &lhs,
        const filters::GaussianEstimate &rhs) const
    {
      const core::Status lhs_status = filters::validate_estimate(lhs);
      if (!lhs_status.ok())
      {
        return lhs_status;
      }

      const core::Status rhs_status = filters::validate_estimate(rhs);
      if (!rhs_status.ok())
      {
        return rhs_status;
      }

      if (lhs.dimension() != rhs.dimension())
      {
        return core::Status::dimension_mismatch(
            "Covariance intersection requires matching estimate dimensions.");
      }

      if (omega_steps_ < 2)
      {
        return core::Status::invalid_argument(
            "Covariance intersection requires at least two omega grid steps.");
      }

      Eigen::LDLT<core::Covariance> lhs_ldlt(lhs.covariance);
      Eigen::LDLT<core::Covariance> rhs_ldlt(rhs.covariance);
      if (lhs_ldlt.info() != Eigen::Success || rhs_ldlt.info() != Eigen::Success)
      {
        return core::Status::numerical_error(
            "Failed to factorize covariance for covariance intersection.");
      }

      const core::Matrix identity =
          core::Matrix::Identity(lhs.dimension(), lhs.dimension());
      const core::Matrix lhs_information = lhs_ldlt.solve(identity);
      const core::Matrix rhs_information = rhs_ldlt.solve(identity);

      filters::GaussianEstimate best_estimate;
      core::Scalar best_score = std::numeric_limits<core::Scalar>::infinity();
      bool found_solution = false;

      for (core::Index step = 0; step < omega_steps_; ++step)
      {
        const core::Scalar omega =
            static_cast<core::Scalar>(step) /
            static_cast<core::Scalar>(omega_steps_ - 1);
        const core::Matrix information =
            omega * lhs_information + (1.0 - omega) * rhs_information;
        Eigen::LDLT<core::Covariance> info_ldlt(core::symmetrize(information));
        if (info_ldlt.info() != Eigen::Success)
        {
          continue;
        }

        filters::GaussianEstimate fused;
        fused.covariance = core::symmetrize(info_ldlt.solve(identity));
        fused.state.value = fused.covariance * (omega * lhs_information * lhs.state.value +
                                                (1.0 - omega) * rhs_information * rhs.state.value);
        fused.state.timestamp = std::max(lhs.state.timestamp, rhs.state.timestamp);
        fused.state.frame_id =
            rhs.state.timestamp >= lhs.state.timestamp
                ? rhs.state.frame_id
                : lhs.state.frame_id;

        const core::Scalar score = fused.covariance.trace();
        if (score < best_score)
        {
          best_score = score;
          best_estimate = fused;
          found_solution = true;
        }
      }

      if (!found_solution)
      {
        return core::Status::numerical_error(
            "Covariance intersection failed to find a numerically stable fusion result.");
      }

      return best_estimate;
    }

    /// Fuses multiple Gaussian estimates into one result.
    [[nodiscard]] core::Result<filters::GaussianEstimate> fuse(
        const std::vector<filters::GaussianEstimate> &estimates) const
    {
      if (estimates.empty())
      {
        return core::Status::invalid_argument(
            "Covariance intersection requires at least one estimate.");
      }

      filters::GaussianEstimate fused = estimates.front();
      for (std::size_t i = 1; i < estimates.size(); ++i)
      {
        const auto pairwise = fuse_pair(fused, estimates[i]);
        if (!pairwise.ok())
        {
          return pairwise.status();
        }
        fused = pairwise.value();
      }

      return fused;
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept
    {
      return "covariance_intersection";
    }

  private:
    core::Index omega_steps_;
  };

} // namespace rosuite::tracking
