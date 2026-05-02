#pragma once

#include <memory>
#include <vector>

#include "ros_tracker/tracking/base.hpp"

namespace ros_tracker::tracking {

struct AssociationCandidate {
  std::size_t track_index {0U};
  std::size_t measurement_index {0U};
  core::Scalar cost {0.0};
};

struct AssociationProblem {
  std::size_t track_count {0U};
  std::size_t measurement_count {0U};
  std::vector<AssociationCandidate> candidates;
};

class AssociationAssignmentSolver {
 public:
  virtual ~AssociationAssignmentSolver() = default;

  [[nodiscard]] virtual core::Result<AssociationResult> solve(
      const AssociationProblem& problem) const = 0;

  [[nodiscard]] virtual std::string_view name() const noexcept = 0;
};

class GreedyAssociationAssignmentSolver final : public AssociationAssignmentSolver {
 public:
  [[nodiscard]] core::Result<AssociationResult> solve(
      const AssociationProblem& problem) const override;

  [[nodiscard]] std::string_view name() const noexcept override;
};

class OptimalAssociationAssignmentSolver final : public AssociationAssignmentSolver {
 public:
  explicit OptimalAssociationAssignmentSolver(std::size_t max_track_count = 8U);

  [[nodiscard]] core::Result<AssociationResult> solve(
      const AssociationProblem& problem) const override;

  [[nodiscard]] std::string_view name() const noexcept override;

 private:
  std::size_t max_track_count_;
};

class NearestNeighborAssociationStrategy final : public AssociationStrategy {
 public:
  explicit NearestNeighborAssociationStrategy(core::Scalar gating_threshold = 16.0);

  [[nodiscard]] core::Result<AssociationResult> associate(
      const std::vector<Track>& tracks,
      const std::vector<core::Measurement>& measurements,
      const models::SensorModel& sensor,
      const models::ModelContext& context = {}) const override;

  [[nodiscard]] std::string_view name() const noexcept override;

 private:
  core::Scalar gating_threshold_;
  std::shared_ptr<const AssociationAssignmentSolver> assignment_solver_;
};

class GlobalNearestNeighborAssociationStrategy final : public AssociationStrategy {
 public:
  explicit GlobalNearestNeighborAssociationStrategy(
      core::Scalar gating_threshold = 16.0,
      std::shared_ptr<const AssociationAssignmentSolver> assignment_solver =
          std::shared_ptr<const AssociationAssignmentSolver> {});

  [[nodiscard]] core::Result<AssociationResult> associate(
      const std::vector<Track>& tracks,
      const std::vector<core::Measurement>& measurements,
      const models::SensorModel& sensor,
      const models::ModelContext& context = {}) const override;

  [[nodiscard]] std::string_view name() const noexcept override;

 private:
  core::Scalar gating_threshold_;
  std::shared_ptr<const AssociationAssignmentSolver> assignment_solver_;
};

}  // namespace ros_tracker::tracking
