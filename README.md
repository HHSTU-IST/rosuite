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

## Modules

See [docs/architecture.md](docs/architecture.md) for details.

- `core`
  - Shared math, numerics, random/statistical helpers, reusable types, and status/result handling.
- `models`
  - Deterministic motion and measurement models plus separate process and measurement noise abstractions.
- `filters`
  - Kalman-family estimators, particle filtering, sigma-point utilities, least-squares tools, and smoothers over abstract models.
- `tracking`
  - Track-level orchestration on top of `filters` and `models`.
  - Current implementation provides `Track`, `AssociationStrategy`, `TrackManager`, and `MultiTargetTracker`.
  - The default pipeline uses gated nearest-neighbor association plus lifecycle-based initiation, confirmation, miss handling, and pruning for single-sensor multi-target workflows.
- `apps` (planned)
  - Offline simulation/examples/tools and ROS runtime wrappers that stay outside the algorithmic core.
