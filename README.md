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

- C++ core
  - C++ 20+
  - Eigen 5.0.0+
- Python wrapper
  - Python 3.12+
  - PyBind11 3.0.0+

## Module TODOs

Architecture reference: `tracker_core -> tracker_models -> tracker_estimation -> tracker_tracking -> tracker_ros/tracker_examples`, with `tracker_sim` as a support layer and `tools` outside the core dependency graph. See [docs/architecture.md](docs/architecture.md) for details.

- [ ] `tracker_core`
  - [ ] Define shared foundational types such as `State`, `Measurement`, and common error/status handling
  - [ ] Add reusable math helpers and linear algebra wrappers
  - [ ] Add numerical utilities such as ODE solvers and quadrature helpers
  - [ ] Add random, Bayesian, and statistical utilities
  - [ ] Add resampling helpers that can be reused by particle-based estimators
  - [ ] Add deterministic unit tests for math, numerics, and stats modules
- [ ] `tracker_models`
  - [ ] Define stable `MotionModel` and `MeasurementModel` interfaces
  - [ ] Implement motion models: `const_vel`, `const_acc`, `coord_turn`, `singer`
  - [ ] Implement sensor/measurement models for radar and other common observation pipelines
  - [ ] Implement process noise and measurement noise abstractions
  - [ ] Add model validation tests for state transition and measurement mapping
- [ ] `tracker_estimation`
  - [ ] Define `FilterBase` and estimator-facing interfaces against abstract models
  - [ ] Implement linear estimators: `const_gain`, `kalman`, `lsq`
  - [ ] Implement nonlinear Kalman-family estimators: `kalman_ekf`, `kalman_ukf`, `kalman_ckf`, `kalman_enkf`, `kalman_fm`, `kalman_hinf`
  - [ ] Implement particle filtering and connect it to reusable resampling utilities
  - [ ] Implement `sigma_points` utilities
  - [ ] Implement `smoothers`
  - [ ] Add convergence, consistency, and covariance regression tests
- [ ] `tracker_tracking`
  - [ ] Define `Track`, `TrackerBase`, `AssociationStrategy`, and `TrackManager`
  - [ ] Move `pda` under `tracking/association/`
  - [ ] Move `imm` under `tracking/multi_model/`
  - [ ] Add multi-sensor fusion under `tracking/fusion/`
  - [ ] Add track initialization, scoring, maintenance, and deletion logic
  - [ ] Add tests for association behavior and track lifecycle management
- [ ] `tracker_sim`
  - [ ] Add scenario definitions for single-target, multi-target, and cluttered scenes
  - [ ] Add trajectory generation and maneuver injection utilities
  - [ ] Add target simulators such as `robot` and maneuvering targets
  - [ ] Add sensor simulators such as `radar`
  - [ ] Add noisy measurement and clutter generation pipelines
  - [ ] Add repeatable benchmark/regression scenarios
  - [ ] Add reproducibility tests for scenario generation
- [ ] `tracker_examples`
  - [ ] Add a minimal single-target offline demo
  - [ ] Add a radar tracking demo
  - [ ] Add an IMM comparison demo after multi-model support is ready
  - [ ] Add example configs and scripts that validate API design before ROS integration grows
- [ ] `tracker_ros`
  - [ ] Keep this package limited to ROS 2 integration boundaries
  - [ ] Add node wrappers, subscriptions, and publications
  - [ ] Add launch files and parameter/config support
  - [ ] Add RViz integration
  - [ ] Add node-level integration tests
- [ ] `tools`
  - [ ] Move plotting utilities to `tools/plot/` or `tracker_examples/scripts/`
  - [ ] Add benchmarking helpers under `tools/benchmark/`
  - [ ] Keep tooling dependencies out of the core package graph
- [ ] Migration of the current backlog
  - [ ] Move `solvers`, `ghq`, `stats`, and shared Bayesian helpers into `tracker_core`
  - [ ] Move `const_acc`, `const_vel`, `coord_turn`, `singer`, and `noise` into `tracker_models`
  - [ ] Move `const_gain`, `kalman*`, `particle`, `lsq`, `sigma_points`, and `smoothers` into `tracker_estimation`
  - [ ] Move `fusion` into `tracker_tracking/fusion/`
  - [ ] Move `imm` into `tracker_tracking/multi_model/`
  - [ ] Move `pda` into `tracker_tracking/association/`
  - [ ] Move `datagen`, `trajectory`, `maneuver`, `robot`, and `radar` into `tracker_sim`
  - [ ] Move plotting-related modules out of the core packages
- [ ] First implementation milestone
  - [ ] Build a complete offline single-target pipeline
  - [ ] Use a constant velocity motion model
  - [ ] Use a linear measurement model
  - [ ] Implement a basic linear Kalman filter
  - [ ] Generate synthetic trajectories and noisy measurements
  - [ ] Validate the pipeline with an example executable and plotting support
