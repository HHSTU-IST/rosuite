#pragma once

#include <utility>
#include <vector>

#include "rosuite/tracking/base.hpp"

namespace rosuite::tracking
{
    class BasicTrackManager final : public TrackManager
    {
    public:
        /// Constructs BasicTrackManager.
        explicit BasicTrackManager(
            core::Index state_dimension,
            core::Covariance initial_covariance,
            std::vector<core::Index> measured_state_indices = {},
            std::size_t confirmation_hits = 2U,
            std::size_t max_consecutive_misses = 2U);

        /// Initializes a new track from a measurement.
        [[nodiscard]] core::Result<Track> initiate_track(
            TrackId id,
            const core::Measurement &measurement,
            std::shared_ptr<const TrackEstimatorModelHandle> handle,
            const models::ModelContext &context = {}) const override;

        /// Updates track metadata after prediction.
        [[nodiscard]] core::Result<Track> on_prediction(
            const Track &track,
            const filters::GaussianEstimate &predicted_estimate) const override;

        /// Updates track metadata after correction.
        [[nodiscard]] core::Result<Track> on_correction(
            const Track &track,
            const filters::GaussianEstimate &corrected_estimate,
            const core::Measurement &measurement) const override;

        /// Updates track metadata after a missed detection.
        [[nodiscard]] core::Result<Track> on_missed_detection(
            const Track &track) const override;

        /// Returns whether the track should be removed.
        [[nodiscard]] bool should_remove(const Track &track) const noexcept override;

        /// Returns the component name.
        [[nodiscard]] std::string_view name() const noexcept override;

    private:
        core::Index state_dimension_;
        core::Covariance initial_covariance_;
        std::vector<core::Index> measured_state_indices_;
        std::size_t confirmation_hits_;
        std::size_t max_consecutive_misses_;
    };

} // namespace rosuite::tracking
