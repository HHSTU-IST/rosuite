#pragma once

#include <string>

#include "kracker/core/math/linear_algebra.hpp"

namespace kracker::core
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

} // namespace kracker::core
