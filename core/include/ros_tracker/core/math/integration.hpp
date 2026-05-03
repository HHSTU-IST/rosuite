#pragma once

#include "ros_tracker/core/math/linear_algebra.hpp"

namespace ros_tracker::core::numerics {

/// Integrates one step with fourth-order Runge-Kutta.
template <typename DerivativeFn>
[[nodiscard]] inline Vector runge_kutta_4(
    const Vector& state,
    const Scalar time,
    const Scalar dt,
    DerivativeFn&& derivative) {
  const Vector k1 = derivative(time, state);
  const Vector k2 = derivative(time + Scalar {0.5} * dt,
                               state + Scalar {0.5} * dt * k1);
  const Vector k3 = derivative(time + Scalar {0.5} * dt,
                               state + Scalar {0.5} * dt * k2);
  const Vector k4 = derivative(time + dt, state + dt * k3);

  return state + (dt / Scalar {6.0}) * (k1 + Scalar {2.0} * k2 +
                                        Scalar {2.0} * k3 + k4);
}

}  // namespace ros_tracker::core::numerics
