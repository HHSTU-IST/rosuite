#pragma once

#include <cstddef>

#include "kracker/core/math/linear_algebra.hpp"

namespace kracker::core::numerics
{
  /// Integrates a function with the composite trapezoidal rule.
  template <typename Function>
  [[nodiscard]] inline Result<Scalar> composite_trapezoidal(
      Function &&function,
      const Scalar lower,
      const Scalar upper,
      const std::size_t intervals)
  {
    if (intervals == 0U)
    {
      return Status::invalid_argument(
          "Composite trapezoidal rule requires at least one interval.");
    }

    const Scalar h = (upper - lower) / static_cast<Scalar>(intervals);
    Scalar sum = Scalar{0.5} * (function(lower) + function(upper));

    for (std::size_t i = 1; i < intervals; ++i)
    {
      sum += function(lower + static_cast<Scalar>(i) * h);
    }

    return h * sum;
  }

  /// Integrates a function with the composite Simpson rule.
  template <typename Function>
  [[nodiscard]] inline Result<Scalar> composite_simpson(
      Function &&function,
      const Scalar lower,
      const Scalar upper,
      const std::size_t intervals)
  {
    if (intervals == 0U || intervals % 2U != 0U)
    {
      return Status::invalid_argument(
          "Composite Simpson rule requires a positive even interval count.");
    }

    const Scalar h = (upper - lower) / static_cast<Scalar>(intervals);
    Scalar sum = function(lower) + function(upper);

    for (std::size_t i = 1; i < intervals; ++i)
    {
      const Scalar weight = (i % 2U == 0U) ? Scalar{2.0} : Scalar{4.0};
      sum += weight * function(lower + static_cast<Scalar>(i) * h);
    }

    return (h / Scalar{3.0}) * sum;
  }

} // namespace kracker::core::numerics
