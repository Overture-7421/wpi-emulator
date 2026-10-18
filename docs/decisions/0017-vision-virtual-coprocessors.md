# ADR 0017 - Vision: virtual coprocessors published in-process

- Status: accepted
- Scope: `sdk/vision/*`, `RobotConfig::vision`, vision fields of `HelloAck` / `SensorFrame` / `ActuatorFrame`, `core/vision`,
  `bridges/vision`, the HALSim extension's bridge fan-out, `IGameModule::contributeFiducialLayouts`, editor, run view

## Context

Robot programs get AprilTag data from a coprocessor (PhotonVision) or a smart camera (Limelight) over NetworkTables. The emulator had
no vision support, and the robot project must stay **unmodified**, so the usual route (robot-side `VisionSystemSim` /
`PhotonCameraSim`) is not available. The emulator is the clock master and runs the robot in lock-step
([ADR 0001](0001-halsim-extension-injection.md)): anything the robot observes has to land on a specific tick and honour pause, step and
time scale, and runs must stay reproducible.

## Decision

* **Virtual coprocessors.** The emulator computes vendor-neutral camera results (`VisionFrame`) from the robot pose, the field's AprilTag
  layout and the camera mounts in `RobotConfig::vision`. A bridge inside the robot process (`bridges/vision`, loaded by `halsim_wpiemu`
  through the existing aggregation seam, [ADR 0004](0004-modular-boundaries.md)) turns them into the real PhotonVision and Limelight
  NetworkTables entries. The robot reads them with its own, unchanged PhotonLib / `LimelightHelpers` code, in Java or C++.
* **In-process publishing, not an NT client.** The bridge binds at run time to the `ntcore` module the robot process already loaded
  (`GetModuleHandle` / `dlopen(RTLD_NOLOAD)`, as the HAL and CTRE bindings do) and publishes through its C API. It stays inert with a
  warning when the module or a symbol is missing. In WPILib 2026.2.1 simulation `wpi::Now()` is the HAL FPGA clock
  (`wpi::SetNowImpl(GetFPGATime)`), so values stamped with `nt::Now()` carry robot time and need no translation. Publishing happens on the
  clock thread between steps, so frames appear at an exact, reproducible simulated time. An emulator-side NT4 client was rejected: its timing
  follows the wall clock and breaks pause / step / time scale.
* **Tick-aligned transport.** Frames ride `SensorFrame::vision`, robot-observed camera state returns in `ActuatorFrame::vision`, and the
  setup (cameras, intrinsics, PhotonVision message UUID) rides `HelloAck::vision`. Protocol version stays 1 ([ADR 0013](0013-pre-release-single-sdk-version.md)).
* **Division of labour.** The engine (`core/vision`) is a deterministic function of truth, config, sim clock and `SimContext::rng()` and never
  reads robot-written state. Vendor "firmware" behaviour lives in the bridge: topic layout, request handling (pipeline, driver mode, throttle,
  LED, ...), Limelight MegaTag1/2 from neutral observations plus the robot-supplied orientation, assumed-vs-physical camera pose.
* **PhotonVision profiles are keyed by message UUID.** A PhotonVision message UUID is the MD5 of its schema string, and a mismatch makes the
  robot throw. The bridge ships one profile (`4b2ff16a...`, verified identical in PhotonLib v2025.3.2, v2026.1.1 and v2026.3.4) and refuses
  unknown UUIDs instead of guessing. The robot never crashes because of the emulator.
* **Ideal by default.** Cameras produce exact geometry unless realism (corner noise, latency jitter, dropouts) is enabled per camera. Frame
  rate and latency are device behaviour and always apply; capture and latency are quantized to ticks.
* **Cameras are not devices.** They are name-keyed, structured and attachable (like zones, [ADR 0016](0016-zones.md)), so they do not use
  `DeviceKey`, slots or the hub. Layouts come from game plugins (`contributeFiducialLayouts`) or a WPILib layout JSON file.
* **No migration** (ADR 0013): `vision` is read as empty when absent, `schemaVersion` stays 1.

## Consequences

An unmodified robot sees PhotonVision and Limelight data without any simulation code, and misconfigured camera transforms in robot code
show up as they would on hardware. Vision needs a robot placed on the field, so the robot start pose and a reposition command are part of
this work. The bridge depends on ntcore's C API (pinned to the staged 2026.2.1 headers) and on PhotonLib's wire format; both are covered by
golden tests and a version check against the project's `photonlib` vendordep. A robot that already simulates its own PhotonVision
(`PhotonCameraSim`) would publish to the same topics, so the bridge detects an existing publisher and leaves that camera alone.
Video streams, camera POV rendering and neural / colored-target pipelines are out of scope for v1.
