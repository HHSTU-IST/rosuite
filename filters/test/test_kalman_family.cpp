#include "support.hpp"

#include "kracker/filters/kalman_filters.hpp"
#include "kracker/filters/sigma_point_filters.hpp"
#include "kracker/models/factories.hpp"

namespace
{

    using kracker::filters::test_support::TestContext;

    /// Tests the linear Kalman filter.
    void test_kalman(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        KalmanFilter filter;
        GaussianEstimate estimate{
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        };

        const DynamicSystemModel system =
            make_constant_velocity_system(0.1 * Matrix::Identity(4, 4));
        const ModelContext predict_context{1.0, 1.0, "map"};
        const auto predicted = filter.predict(estimate, system, predict_context);
        context.expect_true(predicted.ok(), "Kalman prediction should succeed.");
        context.expect_near(predicted.value().state.value[0], 1.0, 1e-12,
                            "Kalman prediction should advance x position.");

        const SensorModel sensor =
            make_position_sensor(0.25 * Matrix::Identity(2, 2));
        Measurement measurement{
            (Vector(2) << 1.2, -0.1).finished(),
            1.0,
            "pos_sensor",
            "map",
        };
        const auto corrected = filter.correct(predicted.value(), sensor, measurement);
        context.expect_true(corrected.ok(), "Kalman correction should succeed.");
        context.expect_true(corrected.value().state.value[0] > predicted.value().state.value[0],
                            "Kalman correction should move the estimate toward the measurement.");
        context.expect_true(corrected.value().covariance.trace() <
                                predicted.value().covariance.trace(),
                            "Kalman correction should reduce covariance trace.");
    }

    /// Tests the constant-gain filter.
    void test_constant_gain_filter(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        Matrix gain = Matrix::Zero(4, 2);
        gain(0, 0) = 1.0;
        gain(1, 1) = 1.0;
        ConstantGainFilter filter(gain);

        GaussianEstimate estimate{
            State{Vector::Zero(4), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        };

        Measurement measurement{
            (Vector(2) << 3.0, -2.0).finished(),
            1.0,
            "pos_sensor",
            "map",
        };

        const auto corrected = filter.correct(
            estimate,
            make_position_sensor(0.25 * Matrix::Identity(2, 2)),
            measurement);
        context.expect_true(corrected.ok(), "Constant gain correction should succeed.");
        context.expect_near(corrected.value().state.value[0], 3.0, 1e-12,
                            "Constant gain filter should overwrite x position with unit gain.");
        context.expect_near(corrected.value().state.value[1], -2.0, 1e-12,
                            "Constant gain filter should overwrite y position with unit gain.");
    }

    /// Tests the extended Kalman filter with a radar model.
    void test_kalman_extended_with_radar(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        KalmanFilterExtended filter;
        GaussianEstimate estimate{
            State{(Vector(4) << 2.5, 4.5, 0.8, 1.7).finished(), 0.0, "map"},
            0.5 * Matrix::Identity(4, 4),
            std::nullopt,
        };

        SensorModel radar{
            std::make_shared<RadarMeasurementModel>(),
            std::make_shared<ConstantGaussianMeasurementNoise>(
                0.1 * Matrix::Identity(3, 3)),
        };

        Measurement measurement{
            (Vector(3) << 5.0, std::atan2(4.0, 3.0), 2.2).finished(),
            1.0,
            "radar",
            "map",
        };

        const auto corrected = filter.correct(estimate, radar, measurement);
        context.expect_true(corrected.ok(), "Extended Kalman correction should succeed.");
        context.expect_true(corrected.value().covariance.trace() < estimate.covariance.trace(),
                            "Extended Kalman correction should reduce covariance trace.");
    }

    /// Tests the unscented and cubature Kalman filters.
    void test_unscented_and_cubature_filters(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        const auto system =
            make_constant_velocity_system(0.1 * Matrix::Identity(4, 4));
        const auto sensor =
            make_position_sensor(0.25 * Matrix::Identity(2, 2));

        GaussianEstimate estimate{
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        };
        const ModelContext context_model{1.0, 1.0, "map"};

        KalmanFilterUnscented ukf;
        const auto ukf_predicted = ukf.predict(estimate, system, context_model);
        context.expect_true(ukf_predicted.ok(), "UKF prediction should succeed.");
        context.expect_near(ukf_predicted.value().state.value[0], 1.0, 1e-8,
                            "UKF prediction should advance x position.");

        Measurement measurement{
            (Vector(2) << 1.1, 0.1).finished(),
            1.0,
            "pos_sensor",
            "map",
        };
        const auto ukf_corrected = ukf.correct(ukf_predicted.value(), sensor, measurement);
        context.expect_true(ukf_corrected.ok(), "UKF correction should succeed.");
        context.expect_true(ukf_corrected.value().covariance.trace() <
                                ukf_predicted.value().covariance.trace(),
                            "UKF correction should reduce covariance trace.");

        KalmanFilterCubature ckf;
        const auto ckf_predicted = ckf.predict(estimate, system, context_model);
        context.expect_true(ckf_predicted.ok(), "CKF prediction should succeed.");
        context.expect_near(ckf_predicted.value().state.value[0], 1.0, 1e-8,
                            "CKF prediction should advance x position.");
        const auto ckf_corrected = ckf.correct(ckf_predicted.value(), sensor, measurement);
        context.expect_true(ckf_corrected.ok(), "CKF correction should succeed.");
        context.expect_true(ckf_corrected.value().covariance.trace() <
                                ckf_predicted.value().covariance.trace(),
                            "CKF correction should reduce covariance trace.");
    }

    /// Tests the ensemble Kalman filter.
    void test_kalman_ensemble(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        KalmanFilterEnsemble enkf(128, 42U);
        GaussianEstimate estimate{
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
            0.5 * Matrix::Identity(4, 4),
            std::nullopt,
        };
        const auto system =
            make_constant_velocity_system(0.1 * Matrix::Identity(4, 4));
        const ModelContext context_model{1.0, 1.0, "map"};

        const auto predicted = enkf.predict(estimate, system, context_model);
        context.expect_true(predicted.ok(), "EnKF prediction should succeed.");
        context.expect_near(predicted.value().state.value[0], 1.0, 0.2,
                            "EnKF prediction mean should stay close to constant-velocity motion.");

        Measurement measurement{
            (Vector(2) << 1.2, -0.1).finished(),
            1.0,
            "pos_sensor",
            "map",
        };
        const auto corrected = enkf.correct(
            predicted.value(),
            make_position_sensor(0.25 * Matrix::Identity(2, 2)),
            measurement);
        context.expect_true(corrected.ok(), "EnKF correction should succeed.");
        context.expect_true(corrected.value().state.value[0] >
                                predicted.value().state.value[0] - 1e-6,
                            "EnKF correction should move the estimate toward the measurement.");
    }

    /// Tests the fading-memory Kalman filter.
    void test_kalman_fading_memory(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        KalmanFilter classical;
        KalmanFilterFadingMemory fading_memory(1.2);

        GaussianEstimate estimate{
            State{(Vector(4) << 0.0, 0.0, 1.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        };

        const auto system =
            make_constant_velocity_system(0.1 * Matrix::Identity(4, 4));
        const ModelContext context_model{1.0, 1.0, "map"};

        const auto baseline = classical.predict(estimate, system, context_model);
        context.expect_true(baseline.ok(), "Baseline Kalman prediction should succeed.");

        const auto predicted = fading_memory.predict(estimate, system, context_model);
        context.expect_true(predicted.ok(), "Fading-memory Kalman prediction should succeed.");
        context.expect_true(predicted.value().covariance.trace() > baseline.value().covariance.trace(),
                            "Fading-memory Kalman prediction should inflate covariance.");

        Measurement measurement{
            (Vector(2) << 1.2, -0.05).finished(),
            1.0,
            "pos_sensor",
            "map",
        };
        const auto corrected =
            fading_memory.correct(
                predicted.value(),
                make_position_sensor(0.25 * Matrix::Identity(2, 2)),
                measurement);
        context.expect_true(corrected.ok(), "Fading-memory Kalman correction should succeed.");
        context.expect_true(corrected.value().covariance.trace() <
                                predicted.value().covariance.trace(),
                            "Fading-memory Kalman correction should reduce covariance trace.");
    }

    /// Tests the H-infinity Kalman filter.
    void test_kalman_h_infinity(TestContext &context)
    {
        using namespace kracker::core;
        using namespace kracker::filters;
        using namespace kracker::models;

        KalmanFilterHInfinity hinf(25.0);
        GaussianEstimate estimate{
            State{(Vector(4) << 0.8, -0.2, 1.0, 0.0).finished(), 0.0, "map"},
            Matrix::Identity(4, 4),
            std::nullopt,
        };

        Measurement measurement{
            (Vector(2) << 1.25, 0.1).finished(),
            1.0,
            "pos_sensor",
            "map",
        };

        const auto corrected =
            hinf.correct(
                estimate,
                make_position_sensor(0.25 * Matrix::Identity(2, 2)),
                measurement);
        context.expect_true(corrected.ok(), "H-infinity Kalman correction should succeed.");
        context.expect_true(corrected.value().covariance.trace() < estimate.covariance.trace(),
                            "H-infinity Kalman correction should reduce covariance trace.");
        context.expect_true(corrected.value().state.value[0] > estimate.state.value[0],
                            "H-infinity Kalman correction should move the estimate toward the measurement.");
    }

} // namespace

int main()
{
    TestContext context;
    test_kalman(context);
    test_constant_gain_filter(context);
    test_kalman_extended_with_radar(context);
    test_unscented_and_cubature_filters(context);
    test_kalman_ensemble(context);
    test_kalman_fading_memory(context);
    test_kalman_h_infinity(context);
    return kracker::filters::test_support::finish(
        context,
        "All Kalman-family filter tests passed.");
}
