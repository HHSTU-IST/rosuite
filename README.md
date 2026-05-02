# ROS Tracker

ROS Tracker is a ROS 2 object tracking project written in C++20.

The original module list is useful as an algorithm checklist, but it is not yet a structure that scales well for implementation, testing, or ROS 2 integration. This document proposes a more maintainable project layout based on layered responsibilities and multiple packages.

## Design Goals

- Separate core algorithms from ROS 2 runtime code
- Keep mathematical utilities reusable and easy to test
- Distinguish models, estimation, and tracking logic
- Treat simulation and plotting as support layers, not core layers
- Make it easy to add new filters, motion models, and tracking pipelines

## Dependencies

- Core algorithms
  - C++ 20+
  - Eigen 5.0.0+
- Build and tests
  - CMake 3.20+
  - CTest

## Module TODOs

See [docs/architecture.md](docs/architecture.md) for details.

- [x] `core`
  - [x] Define shared foundational types such as `State`, `Measurement`, and common error/status handling
  - [x] Add reusable math helpers and linear algebra wrappers
  - [x] Add numerical utilities such as ODE solvers and quadrature helpers
  - [x] Add random, Bayesian, and statistical utilities
  - [x] Add resampling helpers that can be reused by particle-based estimators
  - [x] Add deterministic unit tests for math, numerics, and stats modules
- [x] `models`
  - [x] Consolidate the package into `base`, `motion_models`, `measurement_models`, and `noise_models`
  - [x] Define stable request/context objects instead of growing positional function signatures
  - [x] Define `MotionModel` and `MeasurementModel` as pure deterministic interfaces
  - [x] Define `ProcessNoiseModel` and `MeasurementNoiseModel` as separate abstractions
  - [x] Add optional Jacobian support for EKF-style consumers without burdening all models
  - [x] Implement motion models: `const_vel`, `const_acc`, `coord_turn`, `singer`
  - [x] Implement sensor/measurement models for radar and other common observation pipelines
  - [x] Add composition objects that bundle deterministic models with their noise models
  - [x] Add model validation tests for state transition, measurement mapping, and Jacobian correctness
- [x] `filters`
  - [x] Define `FilterBase` and estimator-facing interfaces against abstract models
  - [x] Implement linear estimators: `const_gain`, `kalman`, `lsq`
  - [x] Implement `kalman_ekf`
  - [x] Implement nonlinear Kalman-family estimators: `kalman_ukf`, `kalman_ckf`, `kalman_enkf`
  - [x] Implement remaining nonlinear Kalman-family estimators: `kalman_fm`, `kalman_hinf`
  - [x] Implement particle filtering and connect it to reusable resampling utilities
  - [x] Implement `sigma_points` utilities and `smoothers`
- [ ] `tracking`
  - [ ] Define `Track`, `TrackerBase`, `AssociationStrategy`, and `TrackManager`
  - [ ] Add `association/`, `multi_model/`, `fusion/`, and `management/` under `tracking/`
  - [ ] Move `pda` into `tracking/association/`
  - [ ] Move `imm` into `tracking/multi_model/`
  - [ ] Add convergence, consistency, covariance, association, and lifecycle tests
- [ ] `apps`
  - [ ] Add `offline/sim/` for scenario generation, sensor simulation, and reproducible regression cases
  - [ ] Add `offline/examples/` for minimal demos and benchmark runners
  - [ ] Add `offline/tools/` for plotting and benchmarking helpers
  - [ ] Add `ros/` for node wrappers, launch files, parameters, and RViz integration
  - [ ] Keep all runtime glue and support tooling out of `core`, `models`, `filters`, and `tracking`
  - [ ] Add reproducibility tests for offline scenarios and node-level integration tests for ROS
