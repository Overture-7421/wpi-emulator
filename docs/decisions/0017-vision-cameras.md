# ADR 0017 - Vision cameras: simulated Limelight / PhotonVision coprocessors

- Status: accepted
- Scope: `sdk/config/camera.h`, `sdk/vision/*`, `RobotConfig::cameras`, `IGameModule::aprilTagLayout()`, `SensorFrame::vision`,
  the `VisionSetup` message, `bridges/vision` (+ its ntcore binding), core `VisionSystem`, editor, run view

## Context

Robot code does not read a camera; it reads a **coprocessor** over NetworkTables: a Limelight publishes the `limelight[-name]`
table (read through `LimelightHelpers`), PhotonVision publishes `/photonvision/<camera>/rawBytes` plus a few state topics (read
through PhotonLib's `PhotonCamera`). The emulator's central promise is that the robot project stays **unmodified**, so vision has
to look to the robot like the real coprocessors do: same topics, same encodings, timestamps in the robot's own time base, and a
realistic delay between capture and publication. Photo-realistic rendering is not required; pose estimators only need to know
which AprilTags were seen, where, and when.

## Decision

* **Fake the coprocessors inside the robot process.** The emulator decides what each simulated camera sees; the HALSim extension,
  which already lives in the robot process, publishes the same NetworkTables data into the robot's **default NT instance**
  by linking **ntcore** directly. The robot then finds its cameras through its own `NetworkTableInstance.getDefault()` exactly as
  it would on a real robot. Nothing in the project changes.
* **The visibility model is deliberately simple** and runs on the sim thread (`VisionSystem` in core): a tag is seen when its centre
  is inside the camera's horizontal and vertical field of view, within `maxRangeM`, and its face points at the camera
  (angle to the tag normal <= `maxIncidenceDeg`). A camera captures at its `fps` and its result is published `latencyMs` later
  in **sim time**; optional Gaussian noise (`positionNoiseM`, `yawNoiseDeg`, from `SimContext::rng()`) perturbs the estimated robot
  pose. Output is deterministic for a seed. <!-- VERIFY: noise distribution (Gaussian) and the exact capture rule in core VisionSystem --> No occlusion, lens distortion, exposure, motion blur or object detection.
* **Cameras are first-class robot parts**: `RobotConfig::cameras` (`CameraConfig`, `sdk/config/camera.h`), owned by core (not
  plugin-owned, so there is no kind catalog; `CameraKind` is `Limelight` or `PhotonVision`). Like zones ([ADR 0016](0016-zones.md))
  a camera hangs on the chassis or a mechanism link through `MechanismAttachment` plus a `local` pose, so a camera on a turret
  moves with the turret. `id` shares the namespace of mechanism and zone ids; `name` (the table / `PhotonCamera` name) must be unique
  per kind. Validation codes are the stable `issue_code::kCamera*` keys.
* **Seams** (same pattern as device bridges and plugins):
  * `sdk::IVisionBridge` (`sdk/vision/vision_bridge.h`: `start`, `configure(VisionSetup)`, `poll`, `apply(frames, simTimeUs)`,
    `reset`, `stop`) and `VisionBridgeRegistry`. Vision bridges run in the robot process on the extension's clock thread, depend
    only on the SDK, and are aggregated by `wpiemu_bridges_builtin` through `registerBuiltinVisionBridges(VisionBridgeRegistry&)`
    (`sdk/devices/builtin_bridges.h`); the extension knows no concrete vision module.
  * `sdk::VisionSetup` (the camera list, the robot project's PhotonLib version, the layout name) is a **new IPC message** sent
    after `HelloAck` and again for every new epoch; the frames due each tick travel in **`SensorFrame::vision`**
    (`std::vector<CameraFrame>`, `sdk/vision/camera_frame.h`).
  * The season's AprilTag field comes from the **game plugin**: `IGameModule::aprilTagLayout()` (default empty = cameras never see
    a tag). `game_2026` embeds the layout `2026-rebuilt-welded` (WPILib `apriltag` 2026.2.1). Core and the bridges never contain
    season data.
* **Two vendor encoders, one ntcore binding.** `bridges/vision` holds pure encoders (CameraFrame -> Limelight topics, CameraFrame ->
  PhotonVision `PhotonPipelineResult` bytes) behind a small `INtPublisher` seam, so they are unit-testable with a fake; the real
  binding links ntcore (`wpiemu_wpilib_ntcore`, discovered next to HAL / wpiutil by `halsim_ext/cmake/WpiEmuWpilib.cmake`) and publishes
  with explicit timestamps taken from sim time.
* **Frames.** Camera frame: +X forward (optical axis), +Y left, +Z up. A tag's +X points **out of its face** (WPILib
  `AprilTagFieldLayout`). Everything reported to robot code (`robotToCamera`, `robotPoseField`) uses the **WPILib robot frame**
  (origin on the floor below the hull centre, yaw = chassis yaw), while the persisted `local` pose is relative to the attachment link
  frame (the chassis frame, whose origin is the centre of mass, [ADR 0003](0003-box3d-chassis-physics.md)). The core converts.
* **Time base.** The extension publishes with `HAL_GetFPGATime()` (the robot's sim-paced FPGA time) as the explicit NetworkTables
  timestamp, so `Timer.getFPGATimestamp() - latency` and PhotonLib's `getTimestampSeconds()` line up with the robot code's clock.
    <!-- VERIFY: HAL_GetFPGATime is what the ntcore binding uses for the explicit timestamp (bridge stream) -->
    <!-- VERIFY: the time-base statement against the e2e latency assertion -->
* **No migration** ([ADR 0013](0013-pre-release-single-sdk-version.md)): `cameras` is read as empty when absent, `schemaVersion`
  stays 1 and the SDK / protocol versions are unchanged.

## Alternatives rejected

* **An external NetworkTables 4 client in the app** (publish to the robot's NT server from the Qt process). It needs a second
  NT stack in the app, publishes in wall time instead of the lock-stepped sim time (the robot's `Timer` and `nt::Now()` would
  disagree with the timestamps), and cannot be paused or reset together with the simulation. Publishing from inside the robot
  process uses the clock thread and the robot's own time base.
* **PhotonVision's `PhotonCameraSim` / `VisionSystemSim`.** They are the right answer for projects that opted in to simulation
  support, but they must be wired into the robot code (`simulationPeriodic`), which violates "unmodified project" and does not help
  Limelights.
* **Rendering real images** (and running a pipeline on them). Far more work and a GPU dependency for information the robot only
  consumes as a handful of numbers.

## Consequences

* A project that already uses `LimelightHelpers` or PhotonLib works with only a camera entry whose name and mount match the code.
  The user has to set the mount (robot-to-camera transform) in the editor to what the code uses; the emulator does not read it from
  the robot program.
* **Limitations (documented in `docs/vision.md` and the user guide):** no occlusion (a tag behind a hub or another robot is still
  seen), no lens distortion, exposure or lighting, no object detection / neural pipelines / colour pipelines; one detection pipeline
  per camera is modelled and the `pipeline`/driver-mode/LED topics are only mirrored. Pose ambiguity is always 0. MegaTag-style
  robot poses are computed from ground truth plus the configured noise, not from a solve.
* **PhotonLib's `rawBytes` is a packed struct whose layout depends on the PhotonLib version.** Only pinned versions are supported
  (`v2026.3.4` first); the launcher passes the project's `photonlib` vendordep version in `VisionSetup`, an unknown or unsupported
  version logs a warning and falls back to the default layout.
* The extension now links ntcore. If loading it by name were to create a second copy next to the one the JVM / robot already
  loaded, the binding resolves the symbols from the already-loaded module instead (same approach as the vendor symbol resolver).
* Vision adds one more thing that must be reset with the simulation: queued frames and published results are dropped on every new
  connection or epoch (`IVisionBridge::reset`) so a stale result is never seen after a restart.
* Cameras are strictly a robot-process / NetworkTables feature; other consumers of camera images (e.g. an `HttpCamera` dashboard
  stream) are out of scope.
