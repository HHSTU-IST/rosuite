# Kracker

`kracker` is a ROS2 object tracking module written in C++17.

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

## Build Shape

- `core` remains header-first for lightweight numeric reuse.
- `models`, `filters`, `tracking`, and `apps` now build compiled libraries, so heavier orchestration and estimator implementations no longer live entirely in public headers.
- `core::Result<T>` rejects `ok`-without-value construction and throws `std::logic_error` on invalid `value()` access to make status-handling bugs fail fast during development.

## Module Status

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
  - [x] `estimator_least_squares`
    - Batch least-squares estimation utilities for linearized estimation problems.
  - [x] `smoothers`
    - Rauch-Tung-Striebel smoothing utilities over Gaussian state estimates.
  - [x] `test`
    - Convergence and covariance checks for the implemented estimation families.
- [x] `tracking`
  - Track-level orchestration on top of `filters` and `models`.
  - [x] `base`
    - Shared `Track`, lifecycle state, per-track estimator/model handle abstractions, and tracker-facing abstract interfaces.
  - [x] `association`
    - Gated nearest-neighbor candidate generation plus pluggable greedy/global assignment solver interfaces for multi-target workflows.
  - [x] `management`
    - Track initiation, confirmation, missed-detection handling, and deletion policies.
  - [x] `tracker`
    - `MultiTargetTracker` orchestration that chains prediction, association, correction, spawning, pruning, and measurement-batch consistency validation.
  - [x] `model_multi`
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
    - Dependency-light tracker-node adapter, parameter bundle, track-message conversion, and measurement-batch validation layer that can be bound to ROS2 transport later.

## How To Use The Modules

The modules are intended to be used in the same dependency order as the architecture:

```text
core -> models -> filters -> tracking -> apps
```

If you only want one include per layer, start from these umbrella headers:

- `#include "kracker/core/core.hpp"`
- `#include "kracker/models/models.hpp"`
- `#include "kracker/filters/filters.hpp"`
- `#include "kracker/tracking/tracking.hpp"`
- `#include "kracker/apps/apps.hpp"`

If you want lighter-weight entry points, prefer the second-level umbrellas:

- `filters`
  - `#include "kracker/filters/filter_primitives.hpp"`
  - `#include "kracker/filters/kalman_filters.hpp"`
  - `#include "kracker/filters/sigma_point_filters.hpp"`
  - `#include "kracker/filters/particle_filters.hpp"`
  - `#include "kracker/filters/estimation_tools.hpp"`
- `tracking`
  - `#include "kracker/tracking/track_lifecycle.hpp"`
  - `#include "kracker/tracking/association_tools.hpp"`
  - `#include "kracker/tracking/model_multi_tools.hpp"`

### 1. `core`: shared math, types, status, and result helpers

Use `core` whenever you need the common numeric types (`Vector`, `Matrix`, `State`, `Measurement`) or reusable math/statistics helpers.

```cpp
#include "kracker/core/core.hpp"

using namespace kracker::core;

State state {
    (Vector(4) << 0.0, 0.0, 1.0, 0.5).finished(),
    0.0,
    "map",
};

Measurement measurement {
    (Vector(2) << 0.2, -0.1).finished(),
    0.0,
    "lidar",
    "map",
};

const auto covariance =
    diagonal_covariance((Vector(2) << 0.25, 0.25).finished());
if (!covariance.ok()) {
  return covariance.status();
}
```

Typical entry points:

- `core/types.hpp`: `State`, `Measurement`, `ControlInput`
- `core/status.hpp` and `core/result.hpp`: status-returning APIs
- `core/math/*`: linear algebra, integration, random sampling, resampling, statistics

### 2. `models`: compose motion, measurement, and noise models

Use `models` to describe system dynamics and sensor behavior independently of any filter.

```cpp
#include "kracker/models/models.hpp"

using namespace kracker::core;
using namespace kracker::models;

Matrix h(2, 4);
h << 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 0.0, 0.0;

DynamicSystemModel system {
    std::make_shared<ConstantVelocityMotionModel>(),
    std::make_shared<ConstantGaussianProcessNoise>(
        0.01 * Matrix::Identity(4, 4)),
};

SensorModel sensor {
    std::make_shared<LinearMeasurementModel>(h),
    std::make_shared<ConstantGaussianMeasurementNoise>(
        0.25 * Matrix::Identity(2, 2)),
};

const auto predicted_state = system.motion->propagate(MotionRequest {
    State {(Vector(4) << 0.0, 0.0, 1.0, 0.5).finished(), 0.0, "map"},
    std::nullopt,
    ModelContext {1.0, 1.0, "map"},
});
```

Use `DynamicSystemModel` when a filter needs state transition plus process noise, and `SensorModel` when a filter or tracker needs observation plus measurement noise.

### 3. `filters`: run a single-estimate prediction/correction loop

Use `filters` when you already have a system model, a sensor model, and an estimate, and want to perform Bayesian state estimation. The example below reuses `system` and `sensor` from the `models` section.

```cpp
#include "kracker/filters/filters.hpp"
#include "kracker/models/models.hpp"

using namespace kracker::core;
using namespace kracker::filters;
using namespace kracker::models;

KalmanFilter filter;
GaussianEstimate estimate {
    State {(Vector(4) << 0.0, 0.0, 1.0, 0.5).finished(), 0.0, "map"},
    Matrix::Identity(4, 4),
};

const auto predicted = filter.predict(
    estimate,
    system,
    ModelContext {1.0, 1.0, "map"});
if (!predicted.ok()) {
  return predicted.status();
}

Measurement measurement {
    (Vector(2) << 1.1, 0.4).finished(),
    1.0,
    "camera",
    "map",
};

const auto corrected = filter.correct(
    predicted.value(),
    sensor,
    measurement);
if (!corrected.ok()) {
  return corrected.status();
}
```

Choose the estimator according to the model assumptions:

- `KalmanFilter`: linear Gaussian systems
- `KalmanFilterExtended`: nonlinear models with Jacobians
- `KalmanFilterUnscented` / `KalmanFilterCubature`: nonlinear models without explicit Jacobians
- `KalmanFilterEnsemble`: ensemble approximation
- `ParticleFilter`: non-Gaussian or strongly nonlinear cases
- `LeastSquaresEstimator` and `RtsSmoother`: batch estimation and smoothing utilities

### 4. `tracking`: build a complete multi-target tracker

Use `tracking` once you want track lifecycle management, association, track spawning, and pruning on top of filters and models. The example below reuses `system` and `sensor` from the `models` section.

```cpp
#include "kracker/tracking/tracking.hpp"
#include "kracker/filters/filters.hpp"
#include "kracker/models/models.hpp"

using namespace kracker::core;
using namespace kracker::filters;
using namespace kracker::models;
using namespace kracker::tracking;

auto filter = std::make_shared<KalmanFilter>();
auto association =
    std::make_shared<NearestNeighborAssociationStrategy>(16.0);
auto manager = std::make_shared<BasicTrackManager>(
    4,
    4.0 * Matrix::Identity(4, 4),
    std::vector<Index> {0, 1},
    1U,
    2U);

MultiTargetTracker tracker(
    filter,
    system,
    sensor,
    association,
    manager);

std::vector<Measurement> measurements {
    {(Vector(2) << 0.1, -0.1).finished(), 1.0, "camera", "map"},
    {(Vector(2) << 10.0, 5.0).finished(), 1.0, "camera", "map"},
};

const auto tracks = tracker.step(
    measurements,
    ModelContext {1.0, 1.0, "map"});
if (!tracks.ok()) {
  return tracks.status();
}
```

Typical customization points:

- `AssociationStrategy`: choose gated nearest-neighbor or global assignment
- `TrackManager`: control initialization, confirmation, missed detections, and deletion
- `TrackEstimatorModelHandle` / `TrackHandleFactory`: allow per-track filter/model choices
- `InteractingMultipleModelEstimator`: combine multiple motion hypotheses for one track
- `CovarianceIntersectionFuser`: conservatively fuse multiple Gaussian estimates

### 5. `apps`: reuse the provided offline example and ROS-facing adapter

Use `apps` when you want end-to-end wiring rather than building every layer manually.

Offline example:

```cpp
#include "kracker/apps/apps.hpp"

const auto summary =
    kracker::apps::offline::run_single_target_kalman_example(42U);
if (!summary.ok()) {
  return 1;
}

const auto& metrics = summary.value().metrics;
```

ROS-facing adapter:

```cpp
#include "kracker/apps/apps.hpp"

auto tracker = std::make_shared<kracker::tracking::MultiTargetTracker>(
    filter, system, sensor, association, manager);

kracker::apps::ros::TrackerNodeParameters params;
params.dt = 1.0;
params.frame_id = "map";

kracker::apps::ros::TrackerNodeAdapter adapter(tracker, params);

const auto message = adapter.process_measurements(measurements);
if (!message.ok()) {
  return message.status();
}
```

Use `apps::offline` for reproducible experiments, regression cases, and quick demos. Use `apps::ros` as the boundary layer that converts tracker output into transport-friendly message structs.

## Python Bindings With uv

The repository also ships a Python packaging entrypoint managed by `uv`. The Python wheel is built from the same C++ sources through `scikit-build-core` and `pybind11`, so the Python API stays on top of the existing `apps` and `tracking` libraries instead of reimplementing tracker logic in Python.

### Install The Python Package

From the repository root:

```bash
uv sync
```

This creates the project virtual environment and builds the extension module when needed.

### Call The Tracker From Python

Use the high-level `Tracker` facade when you want external Python code to drive the tracker directly:

```python
import Kracker

tracker = Kracker.Tracker(
    process_noise=0.01,
    measurement_noise=0.25,
    gating_threshold=16.0,
    frame_id="map",
    sensor_id="camera",
)

tracks = tracker.step(
    measurements=[
        [0.1, -0.1],
        [10.0, 5.0],
    ],
    timestamp=1.0,
    dt=1.0,
)

for track in tracks:
    print(track.id, track.state, track.lifecycle)
```

If you already have fully populated measurement objects, use `step_measurements(...)` instead:

```python
import Kracker

measurements = [
    Kracker.Measurement([0.1, -0.1], timestamp=1.0, sensor_id="camera", frame_id="map"),
]

tracker = Kracker.Tracker()
tracks = tracker.step_measurements(measurements, timestamp=1.0, dt=1.0)
```

### Run The Bundled Example

```bash
uv run python -c "import Kracker; print(Kracker.run_single_target_kalman_example(42))"
```

The initial Python surface is intentionally small:

- `Tracker.step(...)`: convenient entry point for batches of position measurements
- `Tracker.step_measurements(...)`: advanced entry point when metadata is already prepared
- `Tracker.tracks()`: inspect the current in-memory track set
- `run_single_target_kalman_example(...)`: smoke-test the packaged binding against the offline example
