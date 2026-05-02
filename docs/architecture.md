# Architecture Notes

This document explains the reasoning behind the recommended structure for ROS Tracker and clarifies how the major modules should evolve over time.

## 1. Architectural Layers

The repository should be organized as a set of packages with clear responsibilities:

```text
core -> models -> estimation -> tracking -> ros/examples
```

Simulation is a support layer that consumes the lower layers:

```text
sim -> core, models, estimation, tracking
```

Tooling such as plotting and benchmarking should stay outside the core dependency graph.

## 2. Responsibility of Each Package

### `tracker_core`

Contains small, reusable building blocks:

- linear algebra wrappers or helpers
- numerical integration
- random utilities
- statistical utilities
- shared types and error handling

This package should be generic and stable.

### `tracker_models`

Contains physical and observation models:

- motion models
- sensor measurement models
- process noise and measurement noise

This layer describes the world and the sensors rather than the estimation method.

### `tracker_estimation`

Contains algorithms that estimate latent state from measurements:

- Kalman family filters
- particle filter
- least-squares estimators
- smoothers
- sigma-point implementations

This layer should work with abstract models instead of hard-coding ROS 2 details.

### `tracker_tracking`

Contains orchestration and track-oriented logic:

- data association
- multi-model switching
- track initialization and deletion
- track scoring and maintenance
- multi-sensor fusion

This layer combines model and estimation outputs into a tracking system.

### `tracker_sim`

Contains synthetic scenario generation:

- trajectory generation
- maneuver injection
- sensor simulation
- clutter and noise generation
- repeatable benchmark scenarios

This package is critical for algorithm validation and regression testing.

### `tracker_ros`

Contains ROS 2 integration only:

- node wrappers
- subscriptions and publications
- launch files
- parameters
- RViz integration

Keep ROS concerns at the boundary.

### `tracker_examples`

Contains sample applications and demos:

- minimal single-target demo
- radar tracking demo
- IMM comparison demo
- benchmark runners

These examples help validate API design before ROS integration becomes heavy.

## 3. Why `IMM` and `PDA` Should Move

The original structure places `imm` and `pda` under `models`, but they are not world models:

- `IMM` is a model management strategy
- `PDA` is a measurement association strategy

Both act above the level of a single motion or sensor model, so they belong in `tracking`.

Recommended locations:

- `IMM` -> `tracker_tracking/multi_model/`
- `PDA` -> `tracker_tracking/association/`

## 4. Why Plotting Should Not Be a Core Module

Plotting is useful for research, evaluation, and demos, but it is not part of the algorithmic kernel. If plotting sits inside the core project structure, it often introduces avoidable dependencies and blurs the boundary between library code and experimentation tools.

Recommended locations:

- `tools/plot/`
- `tracker_examples/scripts/`

## 5. Suggested Early Interfaces

Try to stabilize these concepts early:

```cpp
struct State;
struct Measurement;

class MotionModel;
class MeasurementModel;
class FilterBase;
class TrackerBase;
class AssociationStrategy;
class TrackManager;
```

The exact API can remain lightweight at first, but naming the boundaries early helps avoid later rewrites.

## 6. Testing Strategy

Each package should have its own tests:

- `tracker_core`: deterministic math and numerical tests
- `tracker_models`: state transition and measurement mapping tests
- `tracker_estimation`: filter convergence and covariance consistency tests
- `tracker_tracking`: association and track lifecycle tests
- `tracker_sim`: scenario reproducibility tests
- `tracker_ros`: node-level integration tests

This structure makes it possible to test most of the system without launching ROS.

## 7. Recommended First Milestone

A good first milestone is a complete single-target offline pipeline:

1. constant velocity model
2. linear measurement model
3. linear Kalman filter
4. synthetic trajectory generator
5. noisy measurement generator
6. example executable with result plotting

Only after this is stable should the repository expand into IMM, PDA, and full ROS 2 runtime integration.

## 8. Naming Advice

Prefer package and directory names that reflect responsibility rather than algorithm families mixed together.

Good:

- `estimation`
- `tracking`
- `association`
- `multi_model`
- `measurement`

Less clear:

- `filters` when it also contains statistics, solvers, and fusion
- `models` when it also contains tracking strategies

## 9. Migration Path From the Current README

The current README is still valuable as an algorithm backlog. A practical migration path is:

1. keep the algorithm checklist
2. remap each item into the new layered package structure
3. implement packages in dependency order
4. add examples before adding ROS nodes
5. add plotting and benchmarking as support tools

This keeps the roadmap intact while giving the codebase a cleaner long-term shape.
