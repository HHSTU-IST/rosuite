#include "ros_tracker/tracking/multi_model.hpp"

namespace ros_tracker::tracking
{
  namespace detail
  {

    core::Result<filters::GaussianEstimate> merge_mode_estimates(
        const std::vector<ModeEstimate> &modes)
    {
      if (modes.empty())
      {
        return core::Status::invalid_argument(
            "Multi-model merge requires at least one mode estimate.");
      }

      std::vector<core::Scalar> probabilities;
      probabilities.reserve(modes.size());
      for (const ModeEstimate &mode : modes)
      {
        const core::Status estimate_status = filters::validate_estimate(mode.estimate);
        if (!estimate_status.ok())
        {
          return estimate_status;
        }
        probabilities.push_back(mode.probability);
      }

      const auto normalized = core::stats::normalize_weights_in_place(probabilities);
      if (!normalized.ok())
      {
        return normalized.status();
      }

      const core::Index dimension = modes.front().estimate.dimension();
      for (const ModeEstimate &mode : modes)
      {
        if (mode.estimate.dimension() != dimension)
        {
          return core::Status::dimension_mismatch(
              "All mode estimates must share the same state dimension.");
        }
      }

      core::Vector mean = core::Vector::Zero(dimension);
      for (std::size_t i = 0; i < modes.size(); ++i)
      {
        mean += probabilities[i] * modes[i].estimate.state.value;
      }

      core::Covariance covariance = core::Covariance::Zero(dimension, dimension);
      for (std::size_t i = 0; i < modes.size(); ++i)
      {
        const core::Vector delta = modes[i].estimate.state.value - mean;
        covariance += probabilities[i] *
                      (modes[i].estimate.covariance + delta * delta.transpose());
      }

      filters::GaussianEstimate merged;
      merged.state.value = mean;
      merged.state.timestamp = modes.front().estimate.state.timestamp;
      merged.state.frame_id = modes.front().estimate.state.frame_id;
      merged.covariance = core::symmetrize(covariance);
      return merged;
    }

    core::Result<core::Scalar> model_log_likelihood(
        const filters::GaussianEstimate &estimate,
        const models::SensorModel &sensor,
        const core::Measurement &measurement,
        const models::ModelContext &context)
    {
      const core::Status estimate_status = filters::validate_estimate(estimate);
      if (!estimate_status.ok())
      {
        return estimate_status;
      }

      const core::Status sensor_status = sensor.validate();
      if (!sensor_status.ok())
      {
        return sensor_status;
      }

      const models::MeasurementRequest request{
          estimate.state,
          context,
          measurement.sensor_id,
      };

      const auto predicted_measurement = sensor.measurement->measure(request);
      if (!predicted_measurement.ok())
      {
        return predicted_measurement.status();
      }

      if (predicted_measurement.value().measurement.dimension() !=
          measurement.dimension())
      {
        return core::Status::dimension_mismatch(
            "Measurement dimension must match the predicted measurement dimension.");
      }

      const auto jacobian = sensor.measurement->state_jacobian(request);
      const auto measurement_noise = sensor.measurement_noise->covariance(request);
      if (!measurement_noise.ok())
      {
        return measurement_noise.status();
      }

      if (measurement_noise.value().rows() != measurement.dimension() ||
          measurement_noise.value().cols() != measurement.dimension())
      {
        return core::Status::dimension_mismatch(
            "Measurement noise covariance dimension must match the measurement dimension.");
      }

      core::Covariance innovation_covariance = measurement_noise.value();
      if (jacobian.ok())
      {
        innovation_covariance = core::symmetrize(
            jacobian.value() * estimate.covariance * jacobian.value().transpose() +
            measurement_noise.value());
      }
      else if (jacobian.status().code != core::StatusCode::kUnimplemented)
      {
        return jacobian.status();
      }

      return core::stats::gaussian_log_likelihood(
          measurement.value,
          predicted_measurement.value().measurement.value,
          innovation_covariance);
    }

  } // namespace detail

  InteractingMultipleModelEstimator::InteractingMultipleModelEstimator(
      std::vector<ModelBankEntry> model_bank,
      core::Matrix transition_probabilities)
      : model_bank_(std::move(model_bank)),
        transition_probabilities_(std::move(transition_probabilities)) {}

  core::Result<MultiModelEstimate> InteractingMultipleModelEstimator::initialize(
      const filters::GaussianEstimate &estimate,
      std::vector<core::Scalar> probabilities) const
  {
    const core::Status bank_status = validate_model_bank();
    if (!bank_status.ok())
    {
      return bank_status;
    }

    const core::Status estimate_status = filters::validate_estimate(estimate);
    if (!estimate_status.ok())
    {
      return estimate_status;
    }

    if (probabilities.empty())
    {
      probabilities.assign(
          model_bank_.size(),
          1.0 / static_cast<core::Scalar>(model_bank_.size()));
    }

    if (probabilities.size() != model_bank_.size())
    {
      return core::Status::dimension_mismatch(
          "Initial IMM probabilities must match the model bank size.");
    }

    const auto normalized = core::stats::normalize_weights_in_place(probabilities);
    if (!normalized.ok())
    {
      return normalized.status();
    }

    MultiModelEstimate initialized;
    initialized.modes.reserve(model_bank_.size());
    for (std::size_t i = 0; i < model_bank_.size(); ++i)
    {
      initialized.modes.push_back(
          ModeEstimate{model_bank_[i].name, estimate, probabilities[i]});
    }
    initialized.merged_estimate = estimate;
    return initialized;
  }

  core::Result<MultiModelEstimate> InteractingMultipleModelEstimator::predict(
      const MultiModelEstimate &estimate,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control) const
  {
    const core::Status validation_status = validate_multi_model_estimate(estimate);
    if (!validation_status.ok())
    {
      return validation_status;
    }

    const std::size_t mode_count = model_bank_.size();
    std::vector<core::Scalar> prior_probabilities(mode_count, 0.0);
    for (std::size_t i = 0; i < mode_count; ++i)
    {
      prior_probabilities[i] = estimate.modes[i].probability;
    }

    std::vector<core::Scalar> mixed_probabilities(mode_count, 0.0);
    for (std::size_t target_mode = 0; target_mode < mode_count; ++target_mode)
    {
      for (std::size_t source_mode = 0; source_mode < mode_count; ++source_mode)
      {
        mixed_probabilities[target_mode] +=
            prior_probabilities[source_mode] *
            transition_probabilities_(
                static_cast<core::Index>(source_mode),
                static_cast<core::Index>(target_mode));
      }
    }

    std::vector<ModeEstimate> predicted_modes;
    predicted_modes.reserve(mode_count);
    for (std::size_t target_mode = 0; target_mode < mode_count; ++target_mode)
    {
      const core::Scalar normalization = mixed_probabilities[target_mode];
      if (normalization <= 0.0)
      {
        return core::Status::numerical_error(
            "IMM mixed mode probability must stay positive.");
      }

      std::vector<ModeEstimate> contributors;
      contributors.reserve(mode_count);
      for (std::size_t source_mode = 0; source_mode < mode_count; ++source_mode)
      {
        const core::Scalar numerator =
            prior_probabilities[source_mode] *
            transition_probabilities_(
                static_cast<core::Index>(source_mode),
                static_cast<core::Index>(target_mode));
        contributors.push_back(ModeEstimate{
            estimate.modes[source_mode].name,
            estimate.modes[source_mode].estimate,
            numerator / normalization,
        });
      }

      const auto mixed_estimate = detail::merge_mode_estimates(contributors);
      if (!mixed_estimate.ok())
      {
        return mixed_estimate.status();
      }

      const auto predicted_estimate = model_bank_[target_mode].filter->predict(
          mixed_estimate.value(),
          model_bank_[target_mode].system_model,
          context,
          control);
      if (!predicted_estimate.ok())
      {
        return predicted_estimate.status();
      }

      predicted_modes.push_back(ModeEstimate{
          model_bank_[target_mode].name,
          predicted_estimate.value(),
          mixed_probabilities[target_mode],
      });
    }

    const auto merged = detail::merge_mode_estimates(predicted_modes);
    if (!merged.ok())
    {
      return merged.status();
    }

    return MultiModelEstimate{predicted_modes, merged.value()};
  }

  core::Result<MultiModelEstimate> InteractingMultipleModelEstimator::correct(
      const MultiModelEstimate &estimate,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context) const
  {
    const core::Status validation_status = validate_multi_model_estimate(estimate);
    if (!validation_status.ok())
    {
      return validation_status;
    }

    std::vector<ModeEstimate> corrected_modes;
    corrected_modes.reserve(model_bank_.size());
    std::vector<core::Scalar> log_weights;
    log_weights.reserve(model_bank_.size());

    for (std::size_t mode_index = 0; mode_index < model_bank_.size(); ++mode_index)
    {
      const auto corrected_estimate = model_bank_[mode_index].filter->correct(
          estimate.modes[mode_index].estimate,
          sensor,
          measurement,
          context);
      if (!corrected_estimate.ok())
      {
        return corrected_estimate.status();
      }

      const auto log_likelihood = detail::model_log_likelihood(
          estimate.modes[mode_index].estimate,
          sensor,
          measurement,
          context);
      if (!log_likelihood.ok())
      {
        return log_likelihood.status();
      }

      const core::Scalar safe_probability =
          std::max(estimate.modes[mode_index].probability,
                   std::numeric_limits<core::Scalar>::min());
      log_weights.push_back(log_likelihood.value() + std::log(safe_probability));
      corrected_modes.push_back(ModeEstimate{
          model_bank_[mode_index].name,
          corrected_estimate.value(),
          0.0,
      });
    }

    const auto posterior_probabilities =
        core::stats::normalize_log_weights(log_weights);
    if (!posterior_probabilities.ok())
    {
      return posterior_probabilities.status();
    }

    for (std::size_t i = 0; i < corrected_modes.size(); ++i)
    {
      corrected_modes[i].probability = posterior_probabilities.value()[i];
    }

    const auto merged = detail::merge_mode_estimates(corrected_modes);
    if (!merged.ok())
    {
      return merged.status();
    }

    return MultiModelEstimate{corrected_modes, merged.value()};
  }

  core::Result<MultiModelEstimate> InteractingMultipleModelEstimator::step(
      const MultiModelEstimate &estimate,
      const models::SensorModel &sensor,
      const core::Measurement &measurement,
      const models::ModelContext &context,
      std::optional<core::ControlInput> control) const
  {
    const auto predicted = predict(estimate, context, std::move(control));
    if (!predicted.ok())
    {
      return predicted.status();
    }

    return correct(predicted.value(), sensor, measurement, context);
  }

  std::string_view InteractingMultipleModelEstimator::name() const noexcept
  {
    return "imm";
  }

  core::Status InteractingMultipleModelEstimator::validate_model_bank() const
  {
    if (model_bank_.empty())
    {
      return core::Status::invalid_argument(
          "InteractingMultipleModelEstimator requires at least one model.");
    }

    if (transition_probabilities_.rows() !=
            static_cast<core::Index>(model_bank_.size()) ||
        transition_probabilities_.cols() !=
            static_cast<core::Index>(model_bank_.size()))
    {
      return core::Status::dimension_mismatch(
          "IMM transition matrix must be square with one row/column per model.");
    }

    for (std::size_t i = 0; i < model_bank_.size(); ++i)
    {
      if (!model_bank_[i].filter)
      {
        return core::Status::invalid_argument(
            "IMM model bank entries require a filter.");
      }

      const core::Status system_status = model_bank_[i].system_model.validate();
      if (!system_status.ok())
      {
        return system_status;
      }

      const auto row =
          transition_probabilities_.row(static_cast<core::Index>(i));
      if ((row.array() < 0.0).any())
      {
        return core::Status::invalid_argument(
            "IMM transition probabilities must be non-negative.");
      }

      if (std::abs(row.sum() - 1.0) > 1e-9)
      {
        return core::Status::invalid_argument(
            "Each IMM transition-probability row must sum to one.");
      }
    }

    return core::Status::ok_status();
  }

  core::Status InteractingMultipleModelEstimator::validate_multi_model_estimate(
      const MultiModelEstimate &estimate) const
  {
    const core::Status bank_status = validate_model_bank();
    if (!bank_status.ok())
    {
      return bank_status;
    }

    if (estimate.modes.size() != model_bank_.size())
    {
      return core::Status::dimension_mismatch(
          "IMM estimate must contain one mode state per model-bank entry.");
    }

    std::vector<core::Scalar> probabilities;
    probabilities.reserve(estimate.modes.size());
    for (const ModeEstimate &mode : estimate.modes)
    {
      const core::Status estimate_status = filters::validate_estimate(mode.estimate);
      if (!estimate_status.ok())
      {
        return estimate_status;
      }
      probabilities.push_back(mode.probability);
    }

    const auto normalized = core::stats::normalize_weights_in_place(probabilities);
    if (!normalized.ok())
    {
      return normalized.status();
    }

    return filters::validate_estimate(estimate.merged_estimate);
  }

} // namespace ros_tracker::tracking
