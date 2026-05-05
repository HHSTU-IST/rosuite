#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rosuite/core/math/statistics.hpp"
#include "rosuite/filters/filter_base.hpp"
#include "rosuite/tracking/base.hpp"

namespace rosuite::tracking
{
  struct ModelBankEntry
  {
    std::string name;
    models::DynamicSystemModel system_model;
    std::shared_ptr<const filters::FilterBase> filter;
  };

  struct ModeEstimate
  {
    std::string name;
    filters::GaussianEstimate estimate;
    core::Scalar probability{0.0};
  };

  struct MultiModelEstimate
  {
    std::vector<ModeEstimate> modes;
    filters::GaussianEstimate merged_estimate;
  };

  namespace detail
  {

    /// Merges per-mode estimates into a single estimate.
    [[nodiscard]] core::Result<filters::GaussianEstimate> merge_mode_estimates(
        const std::vector<ModeEstimate> &modes);

    /// Evaluates the likelihood of a model-conditioned measurement update.
    [[nodiscard]] core::Result<core::Scalar> model_log_likelihood(
        const filters::GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context);

  } // namespace detail

  class InteractingMultipleModelEstimator
  {
  public:
    /// Constructs InteractingMultipleModelEstimator.
    InteractingMultipleModelEstimator(
        std::vector<ModelBankEntry> model_bank,
        core::Matrix transition_probabilities);

    /// Initializes a multi-model estimate.
    [[nodiscard]] core::Result<MultiModelEstimate> initialize(
        const filters::GaussianEstimate &estimate,
        std::vector<core::Scalar> probabilities = {}) const;

    /// Predicts the next estimate.
    [[nodiscard]] core::Result<MultiModelEstimate> predict(
        const MultiModelEstimate &estimate,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const;

    /// Corrects an estimate with a measurement.
    [[nodiscard]] core::Result<MultiModelEstimate> correct(
        const MultiModelEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context = {}) const;

    /// Advances the tracker by one step.
    [[nodiscard]] core::Result<MultiModelEstimate> step(
        const MultiModelEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context,
        std::optional<core::ControlInput> control = std::nullopt) const;

    /// Returns the component name.
    [[nodiscard]] std::string_view name() const noexcept;

  private:
    /// Validates the configured model bank.
    [[nodiscard]] core::Status validate_model_bank() const;

    /// Validates a multi-model estimate.
    [[nodiscard]] core::Status validate_multi_model_estimate(
        const MultiModelEstimate &estimate) const;

    std::vector<ModelBankEntry> model_bank_;
    core::Matrix transition_probabilities_;
  };

} // namespace rosuite::tracking
