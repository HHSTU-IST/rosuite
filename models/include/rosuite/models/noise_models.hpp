#pragma once

#include <utility>

#include "rosuite/models/base.hpp"

namespace rosuite::models
{
  class ConstantGaussianProcessNoise final : public ProcessNoiseModel
  {
  public:
    /// Constructs ConstantGaussianProcessNoise.
    explicit ConstantGaussianProcessNoise(Covariance covariance)
        : covariance_(std::move(covariance)) {}

    /// Computes the covariance for the given request.
    [[nodiscard]] Result<Covariance> covariance(
        const MotionRequest & /*request*/) const override
    {
      const Status status = core::validate_covariance(covariance_);
      if (!status.ok())
      {
        return status;
      }

      return covariance_;
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override
    {
      return "gaussian_process_noise";
    }

  private:
    Covariance covariance_;
  };

  class ConstantGaussianMeasurementNoise final : public MeasurementNoiseModel
  {
  public:
    /// Constructs ConstantGaussianMeasurementNoise.
    explicit ConstantGaussianMeasurementNoise(Covariance covariance)
        : covariance_(std::move(covariance)) {}

    /// Computes the covariance for the given request.
    [[nodiscard]] Result<Covariance> covariance(
        const MeasurementRequest & /*request*/) const override
    {
      const Status status = core::validate_covariance(covariance_);
      if (!status.ok())
      {
        return status;
      }

      return covariance_;
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override
    {
      return "gaussian_measurement_noise";
    }

  private:
    Covariance covariance_;
  };

} // namespace rosuite::models
