#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "rosuite/core/result.hpp"
#include "rosuite/core/status.hpp"
#include "rosuite/core/types.hpp"
#include "rosuite/filters/estimate.hpp"
#include "rosuite/filters/filter_base.hpp"
#include "rosuite/models/base.hpp"

namespace rosuite::tracking
{
    using TrackId = std::size_t;

    enum class TrackLifecycle
    {
        kTentative = 0,
        kConfirmed,
        kDeleted,
    };

    class TrackEstimatorModelHandle
    {
    public:
        /// Destroys TrackEstimatorModelHandle.
        virtual ~TrackEstimatorModelHandle() = default;

        /// Validates the current configuration.
        [[nodiscard]] virtual core::Status validate() const = 0;

        /// Predicts the next estimate.
        [[nodiscard]] virtual core::Result<filters::GaussianEstimate> predict(
            const filters::GaussianEstimate &estimate,
            const models::ModelContext &context,
            std::optional<core::ControlInput> control = std::nullopt) const = 0;

        /// Corrects an estimate with a measurement.
        [[nodiscard]] virtual core::Result<filters::GaussianEstimate> correct(
            const filters::GaussianEstimate &estimate,
            const core::Measurement &measurement,
            const models::ModelContext &context = {}) const = 0;

        /// Returns the sensor model used for association.
        [[nodiscard]] virtual const models::SensorModel &association_sensor_model()
            const noexcept = 0;

        /// Returns the component name.
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    };

    class StaticTrackEstimatorModelHandle final : public TrackEstimatorModelHandle
    {
    public:
        /// Constructs StaticTrackEstimatorModelHandle.
        StaticTrackEstimatorModelHandle(
            std::shared_ptr<const filters::FilterBase> filter,
            models::DynamicSystemModel system_model,
            models::SensorModel sensor_model,
            std::string name = "static_handle");

        /// Validates the current configuration.
        [[nodiscard]] core::Status validate() const override;

        /// Predicts the next estimate.
        [[nodiscard]] core::Result<filters::GaussianEstimate> predict(
            const filters::GaussianEstimate &estimate,
            const models::ModelContext &context,
            std::optional<core::ControlInput> control = std::nullopt) const override;

        /// Corrects an estimate with a measurement.
        [[nodiscard]] core::Result<filters::GaussianEstimate> correct(
            const filters::GaussianEstimate &estimate,
            const core::Measurement &measurement,
            const models::ModelContext &context = {}) const override;

        /// Returns the sensor model used for association.
        [[nodiscard]] const models::SensorModel &association_sensor_model()
            const noexcept override;

        /// Returns the component name.
        [[nodiscard]] std::string_view name() const noexcept override;

    private:
        std::shared_ptr<const filters::FilterBase> filter_;
        models::DynamicSystemModel system_model_;
        models::SensorModel sensor_model_;
        std::string name_;
    };

    class TrackHandleFactory
    {
    public:
        /// Destroys TrackHandleFactory.
        virtual ~TrackHandleFactory() = default;

        /// Creates a track-estimator handle for a measurement.
        [[nodiscard]] virtual core::Result<std::shared_ptr<const TrackEstimatorModelHandle>>
        make_handle(
            const core::Measurement &measurement,
            const models::ModelContext &context = {}) const = 0;

        /// Returns the component name.
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    };

    struct Track
    {
        TrackId id{0U};
        filters::GaussianEstimate estimate;
        std::shared_ptr<const TrackEstimatorModelHandle> handle;
        TrackLifecycle lifecycle{TrackLifecycle::kTentative};
        std::size_t age{0U};
        std::size_t hit_count{0U};
        std::size_t miss_count{0U};
        std::size_t consecutive_misses{0U};
        std::string source_sensor_id;

        /// Returns the vector dimension.
        [[nodiscard]] core::Index dimension() const noexcept
        {
            return estimate.dimension();
        }
    };

    struct MeasurementBatchSummary
    {
        core::Scalar timestamp{0.0};
        std::string frame_id;
    };

    /// Summarizes shared metadata from a measurement batch.
    [[nodiscard]] core::Result<MeasurementBatchSummary> summarize_measurement_batch(
        const std::vector<core::Measurement> &measurements);

    /// Validates a track instance.
    [[nodiscard]] core::Status validate_track(const Track &track);

    struct Association
    {
        std::size_t track_index{0U};
        std::size_t measurement_index{0U};
        core::Scalar cost{0.0};
    };

    struct AssociationResult
    {
        std::vector<Association> matches;
        std::vector<std::size_t> unmatched_tracks;
        std::vector<std::size_t> unmatched_measurements;
    };

    class AssociationStrategy
    {
    public:
        /// Destroys AssociationStrategy.
        virtual ~AssociationStrategy() = default;

        /// Associates measurements with tracks.
        [[nodiscard]] virtual core::Result<AssociationResult> associate(
            const std::vector<Track> &tracks,
            const std::vector<core::Measurement> &measurements,
            const models::SensorModel &sensor,
            const models::ModelContext &context = {}) const = 0;

        /// Returns the component name.
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    };

    class TrackManager
    {
    public:
        /// Destroys TrackManager.
        virtual ~TrackManager() = default;

        /// Initializes a new track from a measurement.
        [[nodiscard]] virtual core::Result<Track> initiate_track(
            TrackId id,
            const core::Measurement &measurement,
            std::shared_ptr<const TrackEstimatorModelHandle> handle,
            const models::ModelContext &context = {}) const = 0;

        /// Updates track metadata after prediction.
        [[nodiscard]] virtual core::Result<Track> on_prediction(
            const Track &track,
            const filters::GaussianEstimate &predicted_estimate) const = 0;

        /// Updates track metadata after correction.
        [[nodiscard]] virtual core::Result<Track> on_correction(
            const Track &track,
            const filters::GaussianEstimate &corrected_estimate,
            const core::Measurement &measurement) const = 0;

        /// Updates track metadata after a missed detection.
        [[nodiscard]] virtual core::Result<Track> on_missed_detection(
            const Track &track) const = 0;

        /// Returns whether the track should be removed.
        [[nodiscard]] virtual bool should_remove(const Track &track) const noexcept = 0;

        /// Returns the component name.
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    };

    class TrackerBase
    {
    public:
        /// Destroys TrackerBase.
        virtual ~TrackerBase() = default;

        /// Advances the tracker by one step.
        [[nodiscard]] virtual core::Result<std::vector<Track>> step(
            const std::vector<core::Measurement> &measurements,
            const models::ModelContext &context,
            std::optional<core::ControlInput> control = std::nullopt) = 0;

        /// Returns the current track set.
        [[nodiscard]] virtual const std::vector<Track> &tracks() const noexcept = 0;

        /// Returns the component name.
        [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    };

} // namespace rosuite::tracking
