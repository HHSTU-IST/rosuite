# Architecture Notes

This document explains the reasoning behind the recommended structure for ROS Tracker and clarifies how the major modules should evolve over time.

## 1. Architectural Layers

The conceptual architecture should keep a clear responsibility flow:

```text
core -> models -> estimation -> tracking -> ros/examples
```

Simulation is a support layer that consumes the lower layers:

```text
sim -> core, models, estimation, tracking
```

Tooling such as plotting and benchmarking should stay outside the core dependency graph.

To improve maintainability, the engineering modules should stay close to the conceptual layers while keeping top-level responsibilities explicit:

```text
core/
models/
filters/
tracking/
apps/
  offline/
    sim/
    examples/
    tools/
  ros/
```

This keeps the architecture clean without over-fragmenting the repository too early:

- keep `core` and `models` separate because they are foundational and stable
- map `estimation` responsibilities into a top-level `filters` module
- keep `tracking` as its own top-level module because its orchestration concerns differ from filtering
- merge `sim`, `examples`, and support tooling into `apps/offline`
- keep ROS integration under `apps/ros` as the only runtime boundary package
- split modules into standalone packages only after ownership, build times, or dependency isolation clearly require it

## 2. Responsibility of Each Engineering Module

### 2.1. `core`

Contains small, reusable building blocks:

- linear algebra wrappers or helpers
- numerical integration
- random utilities
- statistical utilities
- shared types and error handling

This package should be generic and stable.

### 2.2. `models`

Contains physical and observation models:

- motion models
- sensor measurement models
- process noise and measurement noise

This layer describes the world and the sensors rather than the estimation method.

Recommended redesign goals:

- keep model evaluation independent from estimation policy
- keep interfaces pure and side-effect free
- avoid large positional argument lists by using request/context objects
- separate deterministic model equations from noise definitions
- make Jacobians and other derivatives optional extensions rather than mandatory APIs

Recommended internal structure:

```text
models/
  common/
    state.hpp
    measurement.hpp
    control.hpp
    model_context.hpp
    parameter_block.hpp
    dimensions.hpp
  motion/
    motion_model.hpp
    const_vel.hpp
    const_acc.hpp
    coord_turn.hpp
    singer.hpp
  measurement/
    measurement_model.hpp
    linear_measurement.hpp
    radar_measurement.hpp
  noise/
    process_noise_model.hpp
    measurement_noise_model.hpp
    gaussian_noise.hpp
  composition/
    dynamic_system_model.hpp
    sensor_model.hpp
```

The key design choice is to split "what the world does" from "how uncertainty is injected":

- motion models compute deterministic state transitions
- measurement models compute deterministic observation mappings
- noise models provide process or measurement covariance
- composition types bundle them together for consumers that want a full system description

Recommended request/response style:

```cpp
struct ModelContext {
  double dt {};
  double timestamp {};
};

struct MotionRequest {
  State x;
  std::optional<ControlInput> u;
  ModelContext ctx;
};

struct MeasurementRequest {
  State x;
  ModelContext ctx;
};

struct TransitionResult {
  State x_next;
};

struct MeasurementResult {
  Measurement z_pred;
};
```

This keeps function signatures stable as the project grows. When a model later needs control input, sensor pose, frame metadata, or cached parameters, those can be added to the request or context object without rewriting every call site.

Recommended base interfaces:

```cpp
class MotionModel {
 public:
  virtual ~MotionModel() = default;
  virtual TransitionResult propagate(const MotionRequest& req) const = 0;
  virtual std::string_view name() const noexcept = 0;
};

class MeasurementModel {
 public:
  virtual ~MeasurementModel() = default;
  virtual MeasurementResult measure(const MeasurementRequest& req) const = 0;
  virtual std::string_view name() const noexcept = 0;
};

class ProcessNoiseModel {
 public:
  virtual ~ProcessNoiseModel() = default;
  virtual Covariance covariance(const MotionRequest& req) const = 0;
};

class MeasurementNoiseModel {
 public:
  virtual ~MeasurementNoiseModel() = default;
  virtual Covariance covariance(const MeasurementRequest& req) const = 0;
};
```

Recommended optional extensions for algorithms that need more structure:

```cpp
struct Linearization {
  Matrix F;
  Matrix H;
  std::optional<Matrix> G;
  std::optional<Matrix> V;
};

class LinearizableMotionModel : public MotionModel {
 public:
  virtual Matrix state_jacobian(const MotionRequest& req) const = 0;
};

class LinearizableMeasurementModel : public MeasurementModel {
 public:
  virtual Matrix state_jacobian(const MeasurementRequest& req) const = 0;
};
```

This avoids forcing every model to implement Jacobians. A plain particle filter can consume `MotionModel` directly, while EKF-style code can depend on `LinearizableMotionModel` or `LinearizableMeasurementModel`.

Recommended composition objects:

```cpp
struct DynamicSystemModel {
  std::shared_ptr<MotionModel> motion;
  std::shared_ptr<ProcessNoiseModel> process_noise;
};

struct SensorModel {
  std::shared_ptr<MeasurementModel> measurement;
  std::shared_ptr<MeasurementNoiseModel> measurement_noise;
};
```

This gives estimation code a stable unit of dependency. It also avoids giant "model" classes that mix propagation, observation, covariance, sampling, and configuration parsing into one type.

Function-level maintainability rules:

- A model function should do one thing only: propagate, observe, linearize, or provide covariance.
- Model functions should not allocate ROS messages, read configuration files, or own estimator state.
- Model functions should return value objects instead of mutating output parameters where practical.
- Dimensions and units should be explicit in shared types rather than encoded in comments.
- Parameter parsing and factory logic should live at the boundary layer, not inside model equations.

Recommended extension points:

- add new motion models by implementing `MotionModel`
- add EKF-ready models by additionally implementing the linearization interface
- add alternative noise assumptions without changing motion or measurement code
- swap `SensorModel` instances per sensor without changing tracker logic
- compose IMM candidates in `tracking` by reusing `DynamicSystemModel` instances rather than embedding IMM inside `models`

Suggested tests for the redesigned module:

- propagation invariants for each motion model
- expected observation mapping for each measurement model
- covariance shape and symmetry for each noise model
- Jacobian finite-difference checks where linearization is provided
- compatibility tests showing the same model works with multiple estimators

### 2.3. `filters`

Contains the state-estimation algorithms above `models`:

- Kalman family filters
- particle filter
- least-squares estimators
- smoothers
- sigma-point implementations

This module should depend on abstract motion and measurement models rather than ROS-facing types.

### 2.4. `tracking`

Contains the track-oriented orchestration logic above `filters` and `models`:

- data association
- multi-model switching
- track initialization and deletion
- track scoring and maintenance
- multi-sensor fusion
- measurement association strategy, e.g., PDA
- model management strategy, e.g., IMM

This split keeps the main algorithmic boundary explicit: `filters` estimate latent state, while `tracking` composes estimators into full tracking behavior.

### 2.5. `apps`

Contains all boundary-facing and validation-facing code. This module should be split by runtime purpose rather than by small technical categories:

- `apps/offline/sim/`: trajectory generation, maneuver injection, sensor simulation, clutter, repeatable scenarios
- `apps/offline/examples/`: minimal demos, radar demos, IMM comparisons, benchmark runners
- `apps/offline/tools/`: plotting and benchmarking helpers
- `apps/ros/`: node wrappers, subscriptions, publications, launch files, parameters, RViz integration

This keeps validation and runtime glue out of the algorithmic modules while avoiding separate top-level packages for every support concern.

## 3. Suggested Early Interfaces

Try to stabilize these concepts early:

```cpp
struct State;
struct Measurement;
struct ControlInput;

class MotionModel;
class MeasurementModel;
class ProcessNoiseModel;
class MeasurementNoiseModel;
class FilterBase;
class TrackerBase;
class AssociationStrategy;
class TrackManager;
```

The exact API can remain lightweight at first, but naming the boundaries early helps avoid later rewrites.

## 4. Testing Strategy

Each engineering module should own its tests, with subdirectories matching the conceptual layers where needed:

- `core`: deterministic math and numerical tests
- `models`: state transition and measurement mapping tests
- `filters`: filter convergence and covariance consistency tests
- `tracking`: association and track lifecycle tests
- `apps/offline/sim`: scenario reproducibility tests
- `apps/ros`: node-level integration tests

This structure keeps most of the project testable without launching ROS and avoids scattering small test packages across too many top-level modules.

## 5. Recommended First Milestone

A good first milestone is a complete single-target offline pipeline:

1. constant velocity model
2. linear measurement model
3. linear Kalman filter
4. synthetic trajectory generator
5. noisy measurement generator
6. example executable with result plotting

In terms of engineering modules, this first milestone only needs:

- `core`
- `models`
- `filters`
- `apps/offline`

Only after this is stable should the repository expand into `tracking` features such as IMM/PDA and into `apps/ros`.
