#pragma once

#include <memory>

#include "kracker/models/measurement_models.hpp"
#include "kracker/models/motion_models.hpp"
#include "kracker/models/noise_models.hpp"

namespace kracker::models
{
    /// Builds a constant-velocity system model with the provided process noise.
    [[nodiscard]] inline DynamicSystemModel make_constant_velocity_system(
        Covariance process_noise_covariance)
    {
        return {
            std::make_shared<ConstantVelocityMotionModel>(),
            std::make_shared<ConstantGaussianProcessNoise>(
                std::move(process_noise_covariance)),
        };
    }

    /// Builds a linear position sensor model with the provided measurement noise.
    [[nodiscard]] inline SensorModel make_position_sensor(
        Covariance measurement_noise_covariance)
    {
        Matrix observation_matrix(2, 4);
        observation_matrix << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0;

        return {
            std::make_shared<LinearMeasurementModel>(std::move(observation_matrix)),
            std::make_shared<ConstantGaussianMeasurementNoise>(
                std::move(measurement_noise_covariance)),
        };
    }
} // namespace kracker::models
