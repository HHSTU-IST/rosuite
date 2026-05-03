#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "kracker/tracking/base.hpp"

namespace kracker::tracking
{
    class MultiTargetTracker final : public TrackerBase
    {
    public:
        /// Constructs MultiTargetTracker.
        MultiTargetTracker(
            std::shared_ptr<const filters::FilterBase> filter,
            models::DynamicSystemModel system_model,
            models::SensorModel sensor_model,
            std::shared_ptr<const AssociationStrategy> association_strategy,
            std::shared_ptr<const TrackManager> track_manager);

        /// Constructs MultiTargetTracker.
        MultiTargetTracker(
            std::shared_ptr<const TrackEstimatorModelHandle> default_handle,
            std::shared_ptr<const AssociationStrategy> association_strategy,
            std::shared_ptr<const TrackManager> track_manager,
            std::shared_ptr<const TrackHandleFactory> handle_factory = nullptr);

        /// Advances the tracker by one step.
        [[nodiscard]] core::Result<std::vector<Track>> step(
            const std::vector<core::Measurement> &measurements,
            const models::ModelContext &context,
            std::optional<core::ControlInput> control = std::nullopt) override;

        /// Returns the current track set.
        [[nodiscard]] const std::vector<Track> &tracks() const noexcept override;

        /// Returns the component name.
        [[nodiscard]] std::string_view name() const noexcept override;

    private:
        /// Validates dependencies.
        [[nodiscard]] core::Status validate_dependencies() const;

        /// Creates a track-estimator handle for a measurement.
        [[nodiscard]] core::Result<std::shared_ptr<const TrackEstimatorModelHandle>>
        make_handle(
            const core::Measurement &measurement,
            const models::ModelContext &context) const;

        std::shared_ptr<const TrackEstimatorModelHandle> default_handle_;
        std::shared_ptr<const AssociationStrategy> association_strategy_;
        std::shared_ptr<const TrackManager> track_manager_;
        std::shared_ptr<const TrackHandleFactory> handle_factory_;
        std::vector<Track> tracks_;
        TrackId next_track_id_{1U};
    };

} // namespace kracker::tracking
