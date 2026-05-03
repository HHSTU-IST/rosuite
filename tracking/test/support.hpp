#pragma once

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "ros_tracker/filters/kalman_filters.hpp"
#include "ros_tracker/filters/particle_filters.hpp"
#include "ros_tracker/models/models.hpp"
#include "ros_tracker/tracking/track_lifecycle.hpp"

namespace ros_tracker::tracking::test_support
{

struct TestContext
{
    int failures = 0;

    /// Asserts that a condition is true.
    void expect_true(const bool condition, const std::string &message)
    {
        if (!condition)
        {
            std::cerr << "FAIL: " << message << '\n';
            ++failures;
        }
    }
};

/// Creates the default track-estimator handle.
inline std::shared_ptr<const TrackEstimatorModelHandle> make_default_handle()
{
    return std::make_shared<StaticTrackEstimatorModelHandle>(
        std::make_shared<filters::KalmanFilter>(),
        models::make_constant_velocity_system(0.05 * core::Matrix::Identity(4, 4)),
        models::make_position_sensor(0.25 * core::Matrix::Identity(2, 2)),
        "kalman_cv");
}

class SensorAwareTrackHandleFactory final : public TrackHandleFactory
{
public:
    /// Creates a track-estimator handle for a measurement.
    [[nodiscard]] core::Result<std::shared_ptr<const TrackEstimatorModelHandle>>
    make_handle(
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const override
    {
        static_cast<void>(context);

        if (measurement.sensor_id == "pf")
        {
            const std::shared_ptr<const TrackEstimatorModelHandle> handle =
                std::make_shared<StaticTrackEstimatorModelHandle>(
                    std::make_shared<filters::ParticleFilter>(128, 42U, 0.3),
                    models::make_constant_velocity_system(
                        0.05 * core::Matrix::Identity(4, 4)),
                    models::make_position_sensor(0.25 * core::Matrix::Identity(2, 2)),
                    "particle_cv");
            return handle;
        }

        return make_default_handle();
    }

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept override
    {
        return "sensor_aware_factory";
    }
};

/// Completes the test executable with a summary message.
inline int finish(TestContext &context, const char *success_message)
{
    if (context.failures != 0)
    {
        std::cerr << context.failures << " test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << success_message << '\n';
    return EXIT_SUCCESS;
}

} // namespace ros_tracker::tracking::test_support
