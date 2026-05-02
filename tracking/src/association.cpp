#include "ros_tracker/tracking/association.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>

#include "ros_tracker/core/math/statistics.hpp"

namespace ros_tracker::tracking {
namespace {

core::Status validate_association_problem(const AssociationProblem& problem) {
  for (const AssociationCandidate& candidate : problem.candidates) {
    if (candidate.track_index >= problem.track_count) {
      return core::Status::out_of_range(
          "Association candidate track index is out of range.");
    }

    if (candidate.measurement_index >= problem.measurement_count) {
      return core::Status::out_of_range(
          "Association candidate measurement index is out of range.");
    }

    if (candidate.cost < 0.0 || !std::isfinite(candidate.cost)) {
      return core::Status::invalid_argument(
          "Association candidate cost must be finite and non-negative.");
    }
  }

  return core::Status::ok_status();
}

AssociationResult finalize_association_result(
    const AssociationProblem& problem,
    std::vector<Association> matches) {
  std::vector<bool> used_tracks(problem.track_count, false);
  std::vector<bool> used_measurements(problem.measurement_count, false);
  for (const Association& match : matches) {
    used_tracks[match.track_index] = true;
    used_measurements[match.measurement_index] = true;
  }

  AssociationResult result;
  result.matches = std::move(matches);
  result.unmatched_tracks.reserve(problem.track_count);
  result.unmatched_measurements.reserve(problem.measurement_count);

  for (std::size_t i = 0; i < used_tracks.size(); ++i) {
    if (!used_tracks[i]) {
      result.unmatched_tracks.push_back(i);
    }
  }

  for (std::size_t i = 0; i < used_measurements.size(); ++i) {
    if (!used_measurements[i]) {
      result.unmatched_measurements.push_back(i);
    }
  }

  return result;
}

core::Result<core::Scalar> association_cost(
    const Track& track,
    const models::SensorModel& default_sensor,
    const core::Measurement& measurement,
    const models::ModelContext& context) {
  const core::Status track_status = validate_track(track);
  if (!track_status.ok()) {
    return track_status;
  }

  const models::SensorModel& sensor =
      track.handle ? track.handle->association_sensor_model() : default_sensor;
  const core::Status sensor_status = sensor.validate();
  if (!sensor_status.ok()) {
    return sensor_status;
  }

  const models::MeasurementRequest request {
      track.estimate.state,
      context,
      measurement.sensor_id,
  };

  const auto predicted_measurement = sensor.measurement->measure(request);
  if (!predicted_measurement.ok()) {
    return predicted_measurement.status();
  }

  if (predicted_measurement.value().measurement.dimension() !=
      measurement.dimension()) {
    return core::Status::dimension_mismatch(
        "Association measurement dimension must match the predicted measurement dimension.");
  }

  const auto jacobian = sensor.measurement->state_jacobian(request);
  if (!jacobian.ok()) {
    if (jacobian.status().code == core::StatusCode::kUnimplemented) {
      return (measurement.value -
              predicted_measurement.value().measurement.value).squaredNorm();
    }

    return jacobian.status();
  }

  const auto measurement_noise = sensor.measurement_noise->covariance(request);
  if (!measurement_noise.ok()) {
    return measurement_noise.status();
  }

  if (measurement_noise.value().rows() != measurement.dimension() ||
      measurement_noise.value().cols() != measurement.dimension()) {
    return core::Status::dimension_mismatch(
        "Association measurement noise covariance dimension must match the measurement dimension.");
  }

  const core::Covariance innovation_covariance = core::symmetrize(
      jacobian.value() * track.estimate.covariance *
          jacobian.value().transpose() +
      measurement_noise.value());
  return core::stats::mahalanobis_distance_squared(
      measurement.value,
      predicted_measurement.value().measurement.value,
      innovation_covariance);
}

core::Result<AssociationProblem> build_association_problem(
    const std::vector<Track>& tracks,
    const std::vector<core::Measurement>& measurements,
    const models::SensorModel& sensor,
    const models::ModelContext& context,
    const core::Scalar gating_threshold) {
  if (gating_threshold < 0.0) {
    return core::Status::invalid_argument(
        "Association gating threshold must be non-negative.");
  }

  AssociationProblem problem;
  problem.track_count = tracks.size();
  problem.measurement_count = measurements.size();
  problem.candidates.reserve(tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0;
         measurement_index < measurements.size();
         ++measurement_index) {
      const auto cost = association_cost(
          tracks[track_index], sensor, measurements[measurement_index], context);
      if (!cost.ok()) {
        return cost.status();
      }

      if (cost.value() <= gating_threshold) {
        problem.candidates.push_back(
            AssociationCandidate {
                track_index,
                measurement_index,
                cost.value(),
            });
      }
    }
  }

  return problem;
}

}  // namespace

core::Result<AssociationResult> GreedyAssociationAssignmentSolver::solve(
    const AssociationProblem& problem) const {
  const core::Status validation = validate_association_problem(problem);
  if (!validation.ok()) {
    return validation;
  }

  std::vector<AssociationCandidate> candidates = problem.candidates;
  std::sort(candidates.begin(), candidates.end(),
            [](const AssociationCandidate& lhs,
               const AssociationCandidate& rhs) {
              return lhs.cost < rhs.cost;
            });

  std::vector<bool> used_tracks(problem.track_count, false);
  std::vector<bool> used_measurements(problem.measurement_count, false);
  std::vector<Association> matches;
  matches.reserve(std::min(problem.track_count, problem.measurement_count));

  for (const AssociationCandidate& candidate : candidates) {
    if (used_tracks[candidate.track_index] ||
        used_measurements[candidate.measurement_index]) {
      continue;
    }

    used_tracks[candidate.track_index] = true;
    used_measurements[candidate.measurement_index] = true;
    matches.push_back(
        Association {
            candidate.track_index,
            candidate.measurement_index,
            candidate.cost,
        });
  }

  return finalize_association_result(problem, std::move(matches));
}

std::string_view GreedyAssociationAssignmentSolver::name() const noexcept {
  return "greedy_assignment";
}

OptimalAssociationAssignmentSolver::OptimalAssociationAssignmentSolver(
    const std::size_t max_track_count)
    : max_track_count_(max_track_count) {}

core::Result<AssociationResult> OptimalAssociationAssignmentSolver::solve(
    const AssociationProblem& problem) const {
  const core::Status validation = validate_association_problem(problem);
  if (!validation.ok()) {
    return validation;
  }

  if (problem.track_count > max_track_count_) {
    return core::Status::unimplemented(
        "OptimalAssociationAssignmentSolver problem size exceeds the configured track-count limit.");
  }

  std::vector<std::vector<AssociationCandidate>> candidates_by_track(
      problem.track_count);
  for (const AssociationCandidate& candidate : problem.candidates) {
    candidates_by_track[candidate.track_index].push_back(candidate);
  }
  for (auto& candidates : candidates_by_track) {
    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate& lhs,
                 const AssociationCandidate& rhs) {
                return lhs.cost < rhs.cost;
              });
  }

  std::vector<bool> used_measurements(problem.measurement_count, false);
  std::vector<Association> current_matches;
  std::vector<Association> best_matches;
  std::size_t best_match_count = 0U;
  core::Scalar best_cost = std::numeric_limits<core::Scalar>::infinity();

  const std::function<void(std::size_t, std::size_t, core::Scalar)> search =
      [&](const std::size_t track_index,
          const std::size_t current_match_count,
          const core::Scalar current_cost) {
        if (track_index == problem.track_count) {
          if (current_match_count > best_match_count ||
              (current_match_count == best_match_count &&
               current_cost < best_cost)) {
            best_match_count = current_match_count;
            best_cost = current_cost;
            best_matches = current_matches;
          }
          return;
        }

        const std::size_t remaining_tracks = problem.track_count - track_index;
        if (current_match_count + remaining_tracks < best_match_count) {
          return;
        }

        if (current_match_count + remaining_tracks == best_match_count &&
            current_cost >= best_cost) {
          return;
        }

        search(track_index + 1U, current_match_count, current_cost);

        for (const AssociationCandidate& candidate :
             candidates_by_track[track_index]) {
          if (used_measurements[candidate.measurement_index]) {
            continue;
          }

          used_measurements[candidate.measurement_index] = true;
          current_matches.push_back(
              Association {
                  candidate.track_index,
                  candidate.measurement_index,
                  candidate.cost,
              });
          search(track_index + 1U,
                 current_match_count + 1U,
                 current_cost + candidate.cost);
          current_matches.pop_back();
          used_measurements[candidate.measurement_index] = false;
        }
      };

  search(0U, 0U, 0.0);
  return finalize_association_result(problem, std::move(best_matches));
}

std::string_view OptimalAssociationAssignmentSolver::name() const noexcept {
  return "optimal_assignment";
}

NearestNeighborAssociationStrategy::NearestNeighborAssociationStrategy(
    const core::Scalar gating_threshold)
    : gating_threshold_(gating_threshold),
      assignment_solver_(std::make_shared<GreedyAssociationAssignmentSolver>()) {}

core::Result<AssociationResult> NearestNeighborAssociationStrategy::associate(
    const std::vector<Track>& tracks,
    const std::vector<core::Measurement>& measurements,
    const models::SensorModel& sensor,
    const models::ModelContext& context) const {
  const auto problem = build_association_problem(
      tracks, measurements, sensor, context, gating_threshold_);
  if (!problem.ok()) {
    return problem.status();
  }

  return assignment_solver_->solve(problem.value());
}

std::string_view NearestNeighborAssociationStrategy::name() const noexcept {
  return "nearest_neighbor";
}

GlobalNearestNeighborAssociationStrategy::GlobalNearestNeighborAssociationStrategy(
    const core::Scalar gating_threshold,
    std::shared_ptr<const AssociationAssignmentSolver> assignment_solver)
    : gating_threshold_(gating_threshold),
      assignment_solver_(std::move(assignment_solver)) {
  if (!assignment_solver_) {
    assignment_solver_ = std::make_shared<OptimalAssociationAssignmentSolver>();
  }
}

core::Result<AssociationResult>
GlobalNearestNeighborAssociationStrategy::associate(
    const std::vector<Track>& tracks,
    const std::vector<core::Measurement>& measurements,
    const models::SensorModel& sensor,
    const models::ModelContext& context) const {
  if (!assignment_solver_) {
    return core::Status::invalid_argument(
        "GlobalNearestNeighborAssociationStrategy requires an assignment solver.");
  }

  const auto problem = build_association_problem(
      tracks, measurements, sensor, context, gating_threshold_);
  if (!problem.ok()) {
    return problem.status();
  }

  return assignment_solver_->solve(problem.value());
}

std::string_view GlobalNearestNeighborAssociationStrategy::name() const noexcept {
  return "global_nearest_neighbor";
}

}  // namespace ros_tracker::tracking
