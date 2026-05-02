# ROS Tracker

ROS Tracker is a ROS2 object tracking project written in C++17.

The original module list is useful as an algorithm checklist, but it is not yet a structure that scales well for implementation, testing, or ROS2 integration. This document proposes a more maintainable project layout based on layered responsibilities and multiple packages.

## Design Goals

- Separate core algorithms from ROS2 runtime code
- Keep mathematical utilities reusable and easy to test
- Distinguish models, estimation, and tracking logic
- Treat simulation and plotting as support layers, not core layers
- Make it easy to add new filters, motion models, and tracking pipelines

## Dependencies

- Core algorithms
  - C++ 17+
  - Eigen 5.0.0+
- Build and tests
  - CMake 3.20+
  - CTest

## Module TODOs

See [docs/architecture.md](docs/architecture.md) for details.

- [x] `core`
  - Shared math, numerics, random/statistical helpers, reusable types, and status/result handling.
  - [x] `math`
    - Linear algebra helpers, covariance utilities, and deterministic numeric building blocks.
  - [x] `numerics`
    - Time integration and quadrature utilities used by dynamic models and estimators.
  - [x] `stats`
    - Random sampling, likelihood evaluation, and weighted statistics for Bayesian estimators.
  - [x] `types`
    - Shared `State`, `Measurement`, `ControlInput`, `Status`, and `Result` abstractions.
  - [x] `test`
    - Deterministic unit coverage for reusable math, numerics, and statistics helpers.
- [x] `models`
  - Deterministic motion and measurement models plus separate process and measurement noise abstractions.
  - [x] `base`
    - Stable request/context objects and abstract interfaces for motion, measurement, and noise models.
  - [x] `motion_models`
    - Constant-velocity, constant-acceleration, coordinated-turn, and Singer dynamics.
  - [x] `measurement_models`
    - Linear observation and radar-style measurement mappings over shared state types.
  - [x] `noise_models`
    - Process and measurement covariance providers kept separate from deterministic equations.
  - [x] `test`
    - Propagation, observation, covariance, and Jacobian validation for reusable model components.
- [x] `filters`
  - Kalman-family estimators, particle filtering, sigma-point utilities, least-squares tools, and smoothers over abstract models.
  - [x] `kalman`
    - Linear and nonlinear Kalman-family filters including EKF, fading-memory, H-infinity, UKF, CKF, and EnKF.
  - [x] `particle`
    - Bootstrap particle filtering with reusable resampling support from `core`.
  - [x] `sigma_points`
    - Merwe and cubature point generation plus shared Gaussian-transform helpers.
  - [x] `least_squares`
    - Batch least-squares estimation utilities for linearized estimation problems.
  - [x] `smoothers`
    - Rauch-Tung-Striebel smoothing utilities over Gaussian state estimates.
  - [x] `test`
    - Convergence and covariance checks for the implemented estimation families.
- [x] `tracking`
  - Track-level orchestration on top of `filters` and `models`.
  - [x] `base`
    - Shared `Track`, lifecycle state, association result, and tracker-facing abstract interfaces.
  - [x] `association`
    - Gated nearest-neighbor data association for single-sensor multi-target workflows.
  - [x] `management`
    - Track initiation, confirmation, missed-detection handling, and deletion policies.
  - [x] `tracker`
    - `MultiTargetTracker` orchestration that chains prediction, association, correction, spawning, and pruning.
  - [x] `multi_model`
    - IMM-style multi-model estimation over a bank of filters and dynamic-system hypotheses with probabilistic mode mixing.
  - [x] `fusion`
    - Covariance-intersection-based Gaussian estimate fusion for conservative track-to-track combination.
  - [x] `test`
    - Association, lifecycle, multi-model, and fusion regression tests for the current orchestration layer.
- [x] `apps`
  - Offline simulation/tools/examples and ROS2-facing boundary wrappers that stay outside the algorithmic core.
  - [x] `offline/sim`
    - Scenario generation, measurement synthesis, optional clutter, and reproducible regression cases for constant-velocity position-tracking workflows.
  - [x] `offline/tools`
    - Tracking metrics, estimate extraction, and lightweight benchmarking helpers over offline runs.
  - [x] `offline/examples`
    - Minimal end-to-end Kalman/tracker example wiring together simulation, tracking, and metrics.
  - [x] `ros`
    - Dependency-light tracker-node adapter, parameter bundle, and track-message conversion layer that can be bound to ROS2 transport later.
