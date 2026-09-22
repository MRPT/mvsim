# MVSim — AI Agent Context

> **Maintenance rule:** Any change to the project (new class, new module, new sensor, refactor, dependency change, etc.) must be reflected in this file to keep it accurate for future AI tools.

---

## Project overview

**MultiVehicle Simulator (MVSim)** is a lightweight, realistic 2.5D dynamics simulator for mobile robots and multi-agent research. It runs standalone, as a ROS 2 node, or embedded in C++/Python applications. Worlds are fully configured via XML files. Current version: 1.4.0.

- License: BSD 3-Clause
- Language: C++17 (minimum CMake 3.9)
- Repo: https://github.com/MRPT/mvsim

---

## Repository layout

```
mvsim/
├── modules/               # Core C++ library
│   ├── simulator/         # Main simulation engine (libmvsim)
│   ├── comms/             # ZMQ/Protobuf pub-sub communications layer
│   ├── msgs/              # Protobuf message definitions (.proto files)
│   └── raytracer/         # Exact analytic ray casting (no Box2D/GUI/ZMQ)
├── mvsim_node_src/        # ROS 2 node wrapper
├── mvsim-cli/             # Command-line tool (mvsim launch/topic/node/server)
├── mvsim-pid-tuner/       # GUI tool for tuning PID controllers
├── mvsim-dataset-gen/     # Offline ray-traced LiDAR/IMU/odometry dataset generator
├── mvsim_tutorial/        # Demo world XML files + launch files + RViz configs
├── definitions/           # Reusable vehicle and sensor XML definitions
├── examples_cpp/          # C++ subscriber and service-caller examples
├── examples_python/       # Python API examples
├── docs/                  # Sphinx + RST documentation
├── cmake/                 # CMake helpers and version template
├── .clang-format          # Code style (Google base, Allman braces, 4-space indent, tabs)
└── .clang-tidy            # Linter config (bugprone-*, selected cppcoreguidelines-*)
```

---

## Module: `modules/simulator` — `libmvsim`

The simulation engine. Headers live in `modules/simulator/include/mvsim/`.

### Key classes

| Class | Header | Role |
|---|---|---|
| `World` | `World.h` | Central simulation container. Owns all vehicles, blocks, world elements, Box2D physics world, GUI window, ZMQ comms client. Split across `World.cpp`, `World_gui.cpp`, `World_load_xml.cpp`, `World_services.cpp`, `World_simul.cpp`, `World_walls.cpp`. |
| `Simulable` | `Simulable.h` | Base interface for anything that steps through simulation time. Provides `simul_pre_timestep()`, `simul_post_timestep()`, pose access with shared mutex. |
| `VisualObject` | `VisualObject.h` | Base interface for anything renderable in the OpenGL GUI. |
| `VehicleBase` | `VehicleBase.h` | Abstract vehicle. Inherits `VisualObject` and `Simulable`. Holds wheels, sensors, friction model, controller, CSV logger. Created via `ClassFactory`. |
| `Block` | `Block.h` | Static or dynamic rigid-body obstacle/object in the world. |
| `HumanActor` | `HumanActor.h` | Pedestrian/human agent with motion model. |
| `Wheel` | `Wheel.h` | Per-wheel physical state (torque, slip, contact). |
| `SensorBase` | `Sensors/SensorBase.h` | Base class for all sensors. |
| `WorldElementBase` | `WorldElements/WorldElementBase.h` | Base class for ground plane, grids, elevation maps, etc. |
| `ControllerBase` | `ControllerBase.h` | Abstract controller interface. |
| `FrictionBase` | `FrictionModels/FrictionBase.h` | Abstract friction model. |
| `ClassFactory` | `ClassFactory.h` | Template factory that maps XML type strings to C++ constructors. |
| `TParameterDefinitions` | `TParameterDefinitions.h` | Declares XML-readable parameters with `DECLARE_PARAMETER_*` macros. |
| `Shape2p5` | `Shape2p5.h` | 2.5D shape (2D footprint + height interval). |
| `RemoteResourcesManager` | `RemoteResourcesManager.h` | Downloads and caches remote assets (maps, meshes). |
| `CsvLogger` | `CsvLogger.h` | Per-vehicle time-series logging to CSV. |
| `PID_Controller` | `PID_Controller.h` | Generic discrete PID used by controllers. |
| `CollisionShapeCache` | `CollisionShapeCache.h` | Caches Box2D collision shapes for mesh reuse. |
| `PoseTrajectoryFollower` | `PoseTrajectoryFollower.h` | Standalone (no World/Box2D dependency) pure-pursuit tracker for the "exactly reproducible trajectories" feature: evaluates a time-parameterized `(t,x,y)` polyline and computes the `(vx,omega)` twist to follow it. Used by the `trajectory` controller class (differential and Ackermann). |

### Vehicle dynamics (`src/VehicleDynamics/`)

| File | Vehicle type |
|---|---|
| `VehicleDifferential.cpp` | Differential drive (TurtleBot, Jackal style) |
| `VehicleAckermann.cpp` | Ackermann (car-like) with kinematic/dynamic control |
| `VehicleAckermann_Drivetrain.cpp` | Ackermann + realistic mechanical differentials (open/Torsen, 2WD/4WD) |

Each vehicle type has companion `*_Controller*.cpp` files for its controllers (Raw torque, Twist PID, Ideal twist, Front-steer PID, and — for `differential`/`ackermann` only — `trajectory`, the exactly-reproducible-trajectory controller built on `PoseTrajectoryFollower`).

### Sensors (`src/Sensors/`)

| File | Sensor |
|---|---|
| `LaserScanner.cpp` | 2D LiDAR (generic + RPLidar A2) |
| `Lidar3D.cpp` | 3D LiDAR (Velodyne VLP-16, Ouster OS1, Hesai Helios-32) |
| `CameraSensor.cpp` | RGB pinhole camera |
| `DepthCameraSensor.cpp` | RGBD depth + color camera |
| `IMU.cpp` / `ImuNoiseModel.cpp` | IMU with Forster 2016 noise model |
| `GNSS.cpp` | GPS/GNSS with configurable noise |

Cameras (`CameraSensor`, `DepthCameraSensor`) open a GUI preview subwindow showing their live image(s). The common `SensorBase` XML tag `<preview_win_visible>` (default `true`) controls whether that subwindow starts opened or minimized, without affecting the simulated sensor data itself.

### World elements (`src/WorldElements/`)

`OccupancyGridMap`, `ElevationMap`, `HorizontalPlane`, `VerticalPlane`, `GroundGrid`, `PointCloud`, `SkyBox`, `PropertyRegion` (friction zones).

### Friction models (`src/FrictionModels/`)

Default Coulomb, Ward-Iagnemma (off-road), Ellipse (slip angle + slip ratio).

---

## Module: `modules/comms` — ZMQ/Protobuf communications

Provides a language-agnostic pub-sub and service-call layer using ZeroMQ + Protobuf.

Key headers in `modules/comms/include/mvsim/Comms/`:
- `Server.h` — runs inside `World`, accepts registrations and forwards messages.
- `Client.h` — used by external processes (Python, C++ tools, ROS node) to connect.
- `common.h`, `ports.h` — shared constants and port numbers.

---

## Module: `modules/msgs` — Protobuf message definitions

`.proto` files under `modules/msgs/proto/`. Key messages:

- `RegisterNodeRequest/Answer` — node registration handshake
- `AdvertiseTopicRequest` / `SubscribeRequest` — pub-sub setup
- `CallService` / `GenericAnswer` — RPC
- `ObservationLidar2D` / `GenericObservation` — sensor data
- `Pose` / `TimeStampedPose` — pose data
- `SrvGetPose` / `SrvSetPose` / `SrvSetControllerTwist` / `SrvShutdown` — services

---

## Module: `modules/raytracer` — `mvsim::rt`

Exact analytic ray casting, no Box2D/GUI/ZMQ dependency (only `mrpt-math`, `mrpt-poses`), used by `mvsim-dataset-gen/`:

- `Primitive.h` — 6 primitives as a `std::variant`: `Plane` (finite rectangle), `Prism` (2D polygon extruded along world Z, possibly non-convex), `Cylinder` (exact quadric), `Sphere`, `Triangle`, `HeightField` (regular grid; `z(row,col)` with row along X, col along Y, mirroring `ElevationMap::meshCacheZ_`'s own indexing).
- `BVH.h` / `RayScene.h` — median-split BVH over primitive AABBs; `RayScene::castRay()` is the single entry point.
- `Lidar3DModel.h` — ring/column ray-generation math for a rotating 3D LiDAR, mirroring `Lidar3D`'s own conventions (ascending ring order, `vertical_ray_angles` re-sorted the same way) so the same sensor XML describes both the interactive simulator and this ray tracer.

---

## ROS 2 node (`mvsim_node_src/`)

- `mvsim_node_main.cpp` — entry point
- `mvsim_node.cpp` — `MvSimNode` class wrapping `World`, publishing sensor observations as ROS topics, subscribing to `cmd_vel`, advertising TF transforms and ROS 2 parameters.
- `mvsim_node_src/include/` — node header

**Simulation time:** the node is the ROS time source. It publishes `/clock` and stamps every header with *simulation* time (`World::get_simul_timestamp()`); sensor messages use the observation's own `obs.timestamp` so stamps are immune to publisher-thread latency. `myNow()`/`myNowSec()` return sim time (wall-clock fallback before the first step). Downstream nodes should set `use_sim_time:=true`; the node itself runs with `use_sim_time:=false` (only warns if set true). The `disable_sim_time_clock` parameter (default `false`) opts out of all of the above: no `/clock` publication, and every header stamp (via `myNow()`/`myObsStamp()`) uses wall-clock time instead, matching pre-simulation-clock behavior.

---

## CLI tool (`mvsim-cli/`)

`mvsim` binary with subcommands:
- `launch <world.xml>` — start simulator with GUI
- `server` — start headless server
- `topic list/echo/pub` — inspect/inject ZMQ topics
- `node list` — list connected nodes

---

## Dataset generator (`mvsim-dataset-gen/`)

`mvsim-dataset-gen` binary (CLI11): offline, ray-traced (no OpenGL) simulated dataset generator. Given a world XML restricted to analytic geometry and a prescribed ground-truth trajectory (`.tum` SE(3), or 2D `(t,x,y)` waypoints with terrain-following; no controller, no physics stepping — the trajectory *is* the pose), it writes an MRPT `.rawlog` with 3D LiDAR (global or rolling shutter), IMU, and wheel-odometry observations, plus companion `.gt.tum` ground-truth files. See `docs/mvsim-dataset-gen.rst` and `~/plans/mvsim-lidar-simulator.md` for the full design.

- `SceneBuilder.h/.cpp` — converts a headless-loaded `World` into a `mvsim::rt::RayScene`.
- `TrajectorySource.h/.cpp` — `.tum` and 2D-waypoints-with-terrain-following loading via `CPose3DInterpolator`.
- `LidarSimulator.h/.cpp`, `ImuSimulator.h/.cpp`, `OdometrySimulator.h/.cpp` — per-sensor observation generators. `LidarSimulator` ray-casts each sweep's columns in parallel with TBB (`tbb::parallel_for`); each column gets its own RNG stream, seeded up front from the caller's `std::mt19937`, so a given `--seed` produces a byte-identical `.rawlog` regardless of thread scheduling.
- `main.cpp` — CLI11 wiring and a min-heap scheduler merging all sensor streams into strict chronological order; prints a live progress bar with ETA (`mrpt::system::progress()` / `formatTimeInterval()`) while the merge loop runs.

---

## World XML format

Worlds are XML files. Root element: `<mvsim_world>`. Key child elements:

- `<element class="...">` — world elements (occupancy grids, elevation maps, planes)
- `<vehicle class="..." name="...">` — vehicle instances; XML parsed by `VehicleBase::factory()`
- `<block class="..." name="...">` — rigid blocks
- `<actor class="..." name="...">` — actors (e.g., humans)
- `<include>` — file inclusion for reuse
- `<variable>` / `<for>` — flow-control / parameterization
- `<if>` / `<verbatim>` — conditional blocks

Sensor and vehicle definitions can be split into reusable files under `definitions/`.

`<include>` resolution (`xml_to_str_solving_includes()` in `xml_utils.cpp`) is recursive at any nesting depth: nodes with no nested `<include>`/`<if>` are serialized verbatim (fast path), but a subtree containing one anywhere underneath (e.g. inside `<vehicle:class><dynamics><controller>`) is rebuilt node-by-node so the nested tag gets resolved too. This is what lets the `trajectory` controller pull in a waypoint list via a nested `<include>`.

---

## Reusable XML definitions (`definitions/`)

Ready-to-include vehicle and sensor snippets:

- Vehicles: `jackal.vehicle.xml`, `turtlebot3_burger.vehicle.xml`, `ackermann.vehicle.xml`, `small_robot.vehicle.xml`, `pickup.vehicle.xml`, `scania_truck.vehicle.xml`, `agricobiot2.vehicle.xml`, `jackal-ellipse.vehicle.xml`
- Sensors: `velodyne-vlp16.sensor.xml`, `ouster-os1.sensor.xml`, `helios-32-FOV-{26,31,70}.sensor.xml`, `rplidar-a2.sensor.xml`, `camera.sensor.xml`, `rgbd_camera.sensor.xml`, `imu.sensor.xml`, `gnss.sensor.xml`
- Trajectories (`trajectories/` subdir, included inside a `<controller class="trajectory">` node): `square_loop.trajectory.xml`, `figure_eight.trajectory.xml`, `point_to_point.trajectory.xml`

---

## Demo worlds (`mvsim_tutorial/`)

`demo_warehouse.world.xml`, `demo_2robots.world.xml`, `demo_greenhouse.world.xml`, `demo_elevation_map.world.xml`, `demo_road_circuit1.world.xml`, `demo_multistorey.world.xml`, `demo_logistics_center.world.xml`, `demo_articulated_vehicle.world.xml`, `demo_friction_zones.world.xml`, `demo_camera.world.xml`, `demo_depth_camera.world.xml`, `demo_jackal.world.xml`, `demo_many_robots.world.xml`, `demo_indoor_outdoor.world.xml`, `demo_outdoor.world.xml`, `demo_walls.world.xml`, `demo_turtlebot_world.world.xml`, `mvsim_slam.world.xml`, `demo_trajectory.world.xml`, `demo_trajectory_ackermann.world.xml`.

**Exactly reproducible trajectories** (`trajectory` controller class, `PoseTrajectoryFollower`): drives a `differential`/`ackermann` vehicle along a closed-form, time-parameterized `(t,x,y)` polyline given directly in `<waypoint>` XML tags, using a pure-pursuit strategy (speed from waypoint distance/time, heading from a lookahead point, with `max_angular_speed` slowing `vx` down — not just capping `omega` — to round sharp corners realistically). Supports `loop="true"` (repeats forever) and `loop="false"` (runs once and stops). `demo_trajectory.world.xml`/`demo_trajectory_ackermann.world.xml` select between the 3 predefined `definitions/trajectories/*.trajectory.xml` presets via a top-level `TRAJECTORY` `<variable>` and `<include>`; both carry a 3D LiDAR + GNSS sensor. Tested in `tests/test_pose_trajectory_follower.cpp` (pure algorithm, no World) and `tests/test_trajectory_controller.cpp` (full World + Box2D). The path polyline can also be drawn in the 3D GUI (a `mrpt::viz::CSetOfLines` at a configurable `viz_height`, default 0.5m) via `ControllerBaseInterface::getTrajectoryPlotPoints()`, toggled by the "View trajectories" checkbox / `<gui><show_trajectories>` option; both demo worlds enable it by default.

---

## Python API (`examples_python/`)

Uses ZMQ/Protobuf `Client`. Examples: `subscriber-example.py`, `mvsim-teleop.py`, `simple-obstacle-avoidance.py`, `move-object-example.py`, `call-shutdown.py`, `plot-log-files-4-wheels.py`.

---

## Key external dependencies

| Library | Role |
|---|---|
| **MRPT** (>= 3.0) | Math, poses, observations, GUI. 3D scene graph lives in `mrpt/viz` (`mrpt::viz::Scene`, `CSetOfObjects`, ...); `mrpt/opengl` is only used for offscreen FBO rendering (`CFBORender`) in sensors. |
| **Box2D** | 2D rigid-body physics engine |
| **ZeroMQ** (optional) | Pub-sub communications |
| **Protobuf** (optional) | Message serialization |
| **pybind11** (optional) | Python bindings |
| **ROS 2** (optional) | ROS node wrapper |
| **rapidxml** | XML parsing (header-only, bundled) |
| **CLI11** | Command-line parsing in `mvsim-cli/` and `mvsim-dataset-gen/` |
| **TBB** (oneTBB) | Parallel `for` loops (e.g. per-column LiDAR ray casting in `mvsim-dataset-gen/LidarSimulator.cpp`) |

---

## Build system

- CMake 3.9+, C++17.
- Detects ROS 2 via `$ROS_VERSION` env var; builds the ROS node and uses `ament_cmake` when present.
- Standalone build: `cmake .. && make`.
- ROS 2 build: `colcon build`.
- Version is read from `package.xml` and injected into `modules/simulator/include/mvsim/mvsim_version.h` at configure time.
- `cmake/mvsim_cmake_functions.cmake` contains helpers used across targets.
- Optional features guarded by `MVSIM_HAS_ZMQ` and `MVSIM_HAS_PROTOBUF` compile-time defines.

---

## Code style

- **clang-format**: Google base style, Allman brace wrapping, 4-space indent width, **tabs for indentation**, column limit 100, `SortIncludes: true`. Run via `formatter.sh` or `.circleci/clang_format_codebase.sh`.
- **clang-tidy**: `bugprone-*`, selected `cppcoreguidelines-*`, `readability-braces-around-statements`, `readability-else-after-return`. Configured in `.clang-tidy`.
- **Always use braces** for all control-flow statements (`if`, `else`, `for`, `while`, `do`), even single-statement bodies. No brace-less single-line statements. This is enforced by `readability-braces-around-statements` in `.clang-tidy`.
- CI enforces clang-format via the `check-clang-format` GitHub Actions workflow.
- No trailing comments alignment (disabled in `.clang-format`).
- All new C++ code must pass both clang-format and clang-tidy without warnings.

---

## Namespace and conventions

- All simulator code lives in the `mvsim` namespace.
- Protobuf messages are in the `mvsim_msgs` namespace.
- Smart pointer alias `Ptr = std::shared_ptr<T>` defined in every major class.
- Factory-registered classes use `MVSIM_REGISTER_*` macros.
- XML-readable parameters declared with `DECLARE_PARAMETER_*` macros from `TParameterDefinitions.h`.
- File naming: `CamelCase.h` / `CamelCase.cpp` for classes; `lowercase-hyphenated.cpp` for CLI tool files.
- Header guard style: `#pragma once`.
