#pragma once

#include <string>

#include "rosuite/core/math/linear_algebra.hpp"

namespace rosuite::core
{
  struct State
  {
    Vector value;
    Scalar timestamp{0.0};
    std::string frame_id;

    /// Returns the vector dimension.
    [[nodiscard]] Index dimension() const noexcept
    {
      return value.size();
    }
  };

  struct Measurement
  {
    Vector value;
    Scalar timestamp{0.0};
    std::string sensor_id;
    std::string frame_id;

    /// Returns the vector dimension.
    [[nodiscard]] Index dimension() const noexcept
    {
      return value.size();
    }
  };

  struct ControlInput
  {
    Vector value;
    Scalar timestamp{0.0};

    /// Returns the vector dimension.
    [[nodiscard]] Index dimension() const noexcept
    {
      return value.size();
    }
  };

} // namespace rosuite::core
