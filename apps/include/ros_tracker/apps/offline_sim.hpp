#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "ros_tracker/core/math/random.hpp"
#include "ros_tracker/models/models.hpp"

namespace ros_tracker::apps::offline
{

  struct MeasurementFrame
  {
    core::Scalar timestamp{0.0};
    std::vector<core::Measurement> measurements;
  };

  struct SingleTargetScenario
  {
    std::vector<core::State> truth_states;
    std::vector<MeasurementFrame> measurement_frames;
  };

  struct ConstantVelocityScenarioConfig
  {
    core::State initial_state;
    core::Scalar dt{1.0};
    std::size_t steps{1U};
    core::Covariance process_noise_covariance;
    core::Covariance measurement_noise_covariance;
    std::size_t clutter_per_frame{0U};
    core::Vector clutter_min;
    core::Vector clutter_max;
    std::string sensor_id{"offline_sensor"};
    std::string frame_id{"map"};
  };

  /// Builds a reproducible constant-velocity tracking scenario.
  [[nodiscard]] inline core::Result<SingleTargetScenario>
  build_constant_velocity_position_scenario(
      const ConstantVelocityScenarioConfig &config,
      const std::uint64_t seed = 0U)
  {
    if (config.initial_state.dimension() != 4)
    {
      return core::Status::dimension_mismatch(
          "Offline constant-velocity scenario expects a 4D state [x, y, vx, vy].");
    }

    if (config.dt <= 0.0)
    {
      return core::Status::invalid_argument(
          "Offline constant-velocity scenario requires dt > 0.");
    }

    if (config.steps == 0U)
    {
      return core::Status::invalid_argument(
          "Offline constant-velocity scenario requires at least one step.");
    }

    if (config.process_noise_covariance.rows() != 4 ||
        config.process_noise_covariance.cols() != 4)
    {
      return core::Status::dimension_mismatch(
          "Offline process-noise covariance must be 4x4.");
    }

    if (config.measurement_noise_covariance.rows() != 2 ||
        config.measurement_noise_covariance.cols() != 2)
    {
      return core::Status::dimension_mismatch(
          "Offline measurement-noise covariance must be 2x2.");
    }

    const core::Status process_status =
        core::validate_covariance(config.process_noise_covariance);
    if (!process_status.ok())
    {
      return process_status;
    }

    const core::Status measurement_status =
        core::validate_covariance(config.measurement_noise_covariance);
    if (!measurement_status.ok())
    {
      return measurement_status;
    }

    const bool use_clutter = config.clutter_per_frame > 0U;
    if (use_clutter)
    {
      if (config.clutter_min.size() != 2 || config.clutter_max.size() != 2)
      {
        return core::Status::dimension_mismatch(
            "Offline clutter bounds must be 2D when clutter generation is enabled.");
      }

      if ((config.clutter_max.array() < config.clutter_min.array()).any())
      {
        return core::Status::invalid_argument(
            "Offline clutter_max must be elementwise >= clutter_min.");
      }
    }

    core::stats::RandomEngine rng(seed);
    models::ConstantVelocityMotionModel motion_model;

    SingleTargetScenario scenario;
    scenario.truth_states.reserve(config.steps);
    scenario.measurement_frames.reserve(config.steps);

    core::State current_state = config.initial_state;
    current_state.frame_id = config.frame_id;

    for (std::size_t step = 0; step < config.steps; ++step)
    {
      current_state.timestamp =
          config.initial_state.timestamp + static_cast<core::Scalar>(step) * config.dt;
      scenario.truth_states.push_back(current_state);

      MeasurementFrame frame;
      frame.timestamp = current_state.timestamp;

      const auto measurement_noise = rng.sample_multivariate_normal(
          core::Vector::Zero(2), config.measurement_noise_covariance);
      if (!measurement_noise.ok())
      {
        return measurement_noise.status();
      }

      core::Measurement target_measurement;
      target_measurement.value = (core::Vector(2) << current_state.value[0],
                                  current_state.value[1])
                                     .finished() +
                                 measurement_noise.value();
      target_measurement.timestamp = current_state.timestamp;
      target_measurement.sensor_id = config.sensor_id;
      target_measurement.frame_id = config.frame_id;
      frame.measurements.push_back(target_measurement);

      for (std::size_t clutter_index = 0; clutter_index < config.clutter_per_frame;
           ++clutter_index)
      {
        core::Measurement clutter_measurement;
        clutter_measurement.value = core::Vector(2);
        clutter_measurement.value[0] = rng.sample_uniform(
            config.clutter_min[0], config.clutter_max[0]);
        clutter_measurement.value[1] = rng.sample_uniform(
            config.clutter_min[1], config.clutter_max[1]);
        clutter_measurement.timestamp = current_state.timestamp;
        clutter_measurement.sensor_id = config.sensor_id;
        clutter_measurement.frame_id = config.frame_id;
        frame.measurements.push_back(clutter_measurement);
      }

      scenario.measurement_frames.push_back(frame);

      if (step + 1U == config.steps)
      {
        break;
      }

      models::MotionRequest request{
          current_state,
          std::nullopt,
          models::ModelContext{
              config.dt,
              current_state.timestamp + config.dt,
              config.frame_id,
          },
      };
      const auto propagated = motion_model.propagate(request);
      if (!propagated.ok())
      {
        return propagated.status();
      }

      const auto process_noise = rng.sample_multivariate_normal(
          core::Vector::Zero(4), config.process_noise_covariance);
      if (!process_noise.ok())
      {
        return process_noise.status();
      }

      current_state = propagated.value().state;
      current_state.value += process_noise.value();
    }

    return scenario;
  }

} // namespace ros_tracker::apps::offline
