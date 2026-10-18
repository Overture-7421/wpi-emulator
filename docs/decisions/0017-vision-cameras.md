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
  through the **ntcore** the robot process already loaded. The robot then finds its cameras through its own
  `NetworkTableInstance.getDefault()` exactly as it would on a real robot. Nothing in the project changes.
* **The visibility model is deliberately simple** and runs on the sim thread (`VisionSystem` in core): a tag is seen when its centre
  is inside the camera's horizontal and vertical field of view, within `maxRangeM`, and its face points at the camera
  (angle to the tag normal <= `maxIncidenceDeg`). A camera captures at its `fps` (on step boundaries, starting at the first tick)
  and its result is published `latencyMs` later in **sim time**; optional Gaussian noise (`positionNoiseM`, `yawNoiseDeg`, from
  `SimContext::rng()`) perturbs the estimated robot pose. Output is deterministic for a seed. No occlusion, lens distortion,
  exposure, motion blur or object detection.
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
    right after `HelloAck` and again before the first frame of every epoch (idempotent); the frames due each tick travel in
    **`SensorFrame::vision`** (`std::vector<CameraFrame>`, `sdk/vision/camera_frame.h`).
  * The season's AprilTag field comes from the **game plugin**: `IGameModule::aprilTagLayout()` (default empty = cameras never see
    a tag). `game_2026` embeds the layout `2026-rebuilt-welded` (WPILib `apriltag` 2026.2.1). Core and the bridges never contain
    season data.
* **Two vendor encoders, one ntcore binding.** `bridges/vision` holds the Limelight and PhotonVision bridges (CameraFrame -> topics,
  `PhotonPipelineResult` bytes) behind a small `INtBackend` seam, so they are unit-testable with a fake backend. The real binding
  (`wpiemu_bridges_vision_nt`) has **no link-time dependency on ntcore**: it resolves the C API from the module the robot process
  has already loaded (a Java robot loads ntcore through JNI, possibly after the extension), and stays unavailable until then. It
  publishes with explicit timestamps taken from sim time.
* **Frames.** Camera frame: +X forward (optical axis), +Y left, +Z up. A tag's +X points **out of its face** (WPILib
  `AprilTagFieldLayout`). Everything reported to robot code (`robotToCamera`, `robotPoseField`) uses the **WPILib robot frame**
  (origin on the floor below the hull centre, rotated by the chassis yaw only, so pitch and roll never shift it), while the
  persisted `local` pose is relative to the attachment link frame (the chassis frame, whose origin is the centre of mass,
  [ADR 0003](0003-box3d-chassis-physics.md)). The core converts; the editor shows the resulting robot-to-camera transform.
* **Time base.** The extension publishes with `HAL_GetFPGATime()` (the robot's sim-paced FPGA time, which equals `nt::Now()` in a
  simulated robot) as the explicit NetworkTables timestamp, so `Timer.getFPGATimestamp() - latency` and PhotonLib's
  `getTimestampSeconds()` line up with the robot code's clock; the end-to-end test asserts the latency this way.
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
* **PhotonLib's `rawBytes` is a packed struct whose type string embeds a hash of its definition, and PhotonLib throws on a mismatch.**
  Only versions whose hash matches the encoder are supported (v2025.3.2 and v2026.1.1 to v2026.7 were verified, default v2026.3.4);
  the launcher passes the project's `photonlib` vendordep version in `VisionSetup`, and any other version is refused with one logged
  error (no PhotonVision camera is published).
* The extension depends on ntcore only at run time (see above); a build without the ntcore artifacts still works, with inert cameras.
  Verified on Windows with ntcore 2026.2.1 (Java and C++ robots); the Linux/macOS binding is untested.
* Vision adds one more thing that must be reset with the simulation: queued frames are dropped in core and the published results are
  reset on every new connection or epoch (`IVisionBridge::reset`) so a stale result is never seen after a restart.
* Cameras are strictly a robot-process / NetworkTables feature; other consumers of camera images (e.g. an `HttpCamera` dashboard
  stream) are out of scope.
