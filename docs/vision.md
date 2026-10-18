# Vision cameras (`bridges/vision`, core `VisionSystem`)

Simulated **Limelight** and **PhotonVision** coprocessors for AprilTag pose estimation. The emulator decides which tags each
camera sees; the HALSim extension, inside the robot process, publishes the same NetworkTables data a real coprocessor would, into
the robot's default NT instance, with simulated latency. The robot project is not modified. Rationale and rejected options:
[ADR 0017](decisions/0017-vision-cameras.md). User-level steps: [USER_GUIDE.md](USER_GUIDE.md) section 3 (*Cameras*).

Everything vision-specific is optional: with no entry in `RobotConfig::cameras` no topic is created or published (the emulator
still sends an empty `VisionSetup`); in a season with no tag layout the cameras exist but always report "no target".

```
sdk/include/wpiemu/sdk/
  config/camera.h              CameraKind, CameraConfig (persisted in RobotConfig::cameras)
  vision/apriltag.h            AprilTag, AprilTagLayout
  vision/camera_frame.h        TagDetection, CameraFrame, CameraSetup, VisionSetup
  vision/vision_bridge.h       IVisionBridge, VisionBridgeRegistry
  devices/builtin_bridges.h    registerBuiltinVisionBridges(VisionBridgeRegistry&)  (aggregation seam)
  ipc/messages.h               SensorFrame::vision, the VisionSetup message
  telemetry/snapshot.h         CameraState, ISimSnapshot::cameras()
core/runtime/                  VisionSystem (runtime/vision_system.*, owned by SimulationSession), cameras codec + validation
plugins/game_2026/             AprilTag layout 2026-rebuilt-welded (src/april_tags.cpp, IGameModule::aprilTagLayout)
bridges/vision/                wpiemu_bridges_vision: Limelight + PhotonVision bridges and encoders, the INtBackend seam
                               (nt_backend.h), a fake backend for tests, registerVisionBridges
  .../nt_binding.h             wpiemu_bridges_vision_nt: the real ntcore INtBackend (only built when ntcore was found)
bridges/builtin_bridges.cpp    registerBuiltinVisionBridges -> bridges/vision (extension-side aggregation seam)
halsim_ext/                    BridgeSet vision fan-out (pollVision / applyVision / configureVision), SimSession wiring
tests/runtime, tests/bridges_vision, halsim_ext/tests, tests/ipc, tests/app_editor, tests/app_runview   (see Tests)
samples/vision_robot/{java,cpp}  sample robots using both vendors
```

## 1. Architecture

```
 Emulator process                                                  Robot process (halsim_wpiemu)
 +------------------------------------------+                     +-------------------------------------------+
 | SIM THREAD                               |   VisionSetup       | IPC thread -> BridgeSet (clock thread)    |
 |  SimulationSession::tick                 |  ----------------->  |                                           |
 |   ... physics, game state ...            |   (after HelloAck,   |  IVisionBridge "vision.limelight"         |
 |   VisionSystem::step(simTime)            |    every new epoch)  |     poll(t)  <- robot-written settings    |
 |    per camera due this tick:             |                      |     apply(frames, t) -> NT topics         |
 |     pose = chassis * link * local        |   SensorFrame        |  IVisionBridge "vision.photonvision"      |
 |     tags in FOV / range / facing         |  ----------------->  |     poll(t)  <- *Request topics           |
 |     -> CameraFrame (latency queue)       |   .vision[]          |     apply(frames, t) -> rawBytes, state   |
 |   frames with publishTime <= now         |                      |                                           |
 |     go into SensorFrame::vision          |                      |  INtBackend -> ntcore (default NT inst)   |
 |  snapshot: CameraState (run view)        |                      |  then HALSIM_StepTiming(stepUs)           |
 +------------------------------------------+                      +----------------------+--------------------+
                                                                                          |
                                                                       robot code: LimelightHelpers / PhotonCamera
```

Ownership follows the rest of the project (ARCHITECTURE section 1): the SDK holds the data types and the `IVisionBridge`
seam; core (Qt-free, season-free) computes visibility; the season's tag field comes from the game plugin; the bridges run in
the robot process and depend only on the SDK (ntcore stays behind `INtBackend`); the app only shows the result.

### Data flow per tick

1. Right after `HelloAck`, and again before the first frame of every epoch (Reset, Begin, reconnect), the emulator sends a
   **`VisionSetup`** (`CameraSetup` per configured camera, the robot project's PhotonLib version from its `photonlib` vendordep, the
   layout name). It is sent even when there are no cameras (an empty list) and is idempotent. `BridgeSet::configureVision`
   remembers it and calls `IVisionBridge::configure` on every started bridge; each bridge builds the cameras of its kind.
2. Sim thread, tick N, after the **physics step and the game step** (ARCHITECTURE 4, steps 4-5): `VisionSystem` resolves each
   camera's world pose from the final chassis pose, the live link pose of its attachment and `local`. Cameras whose capture time
   has come (period `round(1e6/fps)` us) compute a `CameraFrame` and queue it until `simTime >= capture + latencyMs`.
3. `SimulationSession::tick` moves every frame **due now** (`publishTimeUs <= simTime`) into `SensorFrame::vision`; the snapshot
   carries `CameraState` (world pose, `tagsVisible`, `visibleTagIds` of the last captured frame) for the run view.
4. Extension, on receipt, on the clock thread: `applyInputs`, then `pollVision(simTime)` (mirrors robot-written pipeline / driver
   mode / LED topics and advances heartbeats), then `applyVision(frames, simTime)` (publishes the due results), all **right
   before `HALSIM_StepTiming`**, so the robot loop that runs next sees every result published up to this sim time.
5. `HALSIM_StepTiming(stepUs)` runs the robot code, which reads the NT topics through `LimelightHelpers` / PhotonLib.

## 2. The camera model

`CameraConfig` (`sdk/config/camera.h`, persisted under `cameras` in `robot.json`, see [runtime.md](runtime.md)):

| Field | Default | Meaning |
|---|---|---|
| `id`, `label` | | unique id (also distinct from mechanism and zone ids), display name |
| `name` | empty | The name robot code uses. Limelight: the table is `limelight` when empty, else `limelight-<name>`. PhotonVision: the `PhotonCamera` name (`/photonvision/<name>`). Unique per kind |
| `kind` | `Limelight` | `Limelight` or `PhotonVision` (`cameraKindKey`: `"limelight"`, `"photonvision"`) |
| `attachment`, `local` | chassis | `MechanismAttachment` (chassis or mechanism + link) and the camera pose relative to that link frame |
| `hFovDeg`, `vFovDeg` | 75, 50 | field of view; both in (0, 180) |
| `maxRangeM` | 6 | farther tags are not seen |
| `maxIncidenceDeg` | 75 | tags seen more obliquely than this (angle between the tag normal and the direction to the camera) are not seen |
| `fps` | 30 | capture rate |
| `latencyMs` | 40 | capture-to-publish delay |
| `widthPx`, `heightPx` | 1280 x 800 | pixel space used for tag corners and area |
| `positionNoiseM`, `yawNoiseDeg` | 0, 0 | 1-sigma noise on the estimated robot position (x, y) and yaw |

### Visibility rules

A tag is **seen** by a camera at capture time when all of these hold (evaluated in the camera frame, for the tag **centre**):

1. the tag centre is in front of the camera (`x > 0`) and inside its rectangular field of view (|bearing| <= `hFovDeg`/2,
   |elevation| <= `vFovDeg`/2); only the centre is tested, a tag half outside the image still counts, and there is no minimum range;
2. the distance from the camera to the tag centre is <= `maxRangeM`;
3. the tag faces the camera: the camera is on the front side of the tag (+X side) and the angle between the tag's outward
   normal and the vector from the tag to the camera is <= `maxIncidenceDeg`.

There is **no occlusion** (a tag behind the hub or another robot is still seen), no distortion, no exposure and no minimum pixel
size. A frame with no visible tag is still produced and published (that is how a real pipeline reports "no target").
Tag corners are projected with a pinhole model from the field of view and `widthPx` x `heightPx`; the area is the quadrilateral's
share of the image in percent.

### Rate and latency

* The first capture is at the first tick of an epoch (sim time 0); the camera then captures on the grid `k * round(1e6/fps)` us
  from there. The emulator steps in fixed ticks (default 5 ms), so each capture happens on the first tick boundary at or after its
  grid point (without drift); `CameraFrame::captureTimeUs` is that tick's sim time. The sequence number counts frames captured by
  that camera since the epoch started.
* The result is published `latencyMs` after capture: `publishTimeUs == captureTimeUs + latency` exactly, and it is delivered to the
  extension on the first tick at or after that time (so up to one step later). Robot code sees it in the first loop after that, so a
  20 ms periodic loop can observe a result up to one period later again.
* The data carries the **capture** time (Limelight: `now - (tl + cl)` is the capture time; PhotonVision: the result's capture
  timestamp), so `addVisionMeasurement` receives the timestamp of the moment the robot was where the camera saw it.

### Noise

`positionNoiseM` / `yawNoiseDeg` perturb the robot pose a camera reports (`CameraFrame::robotPoseField`) with zero-mean Gaussian
noise (x and y independently, plus a yaw rotation), drawn once per frame that sees at least one tag; detections of single tags
(`cameraToTag`, bearings, corners) are noise-free. Noise comes from `SimContext::rng()` (seeded by `RobotConfig::sim.seed`), so a
run is reproducible.

### Robot pose (multi-tag)

When at least one tag is seen, `CameraFrame::hasRobotPose` is true and `robotPoseField` is the **ground-truth robot pose in the
field frame** (plus noise), i.e. what a perfect solver would return from the visible tags. `avgTagDistM`, `avgTagAreaPercent` and
`tagSpanM` (largest distance between two seen tags, 0 for one) are filled for the Limelight `botpose` metadata. There is no solver:
the coprocessor-side poses (Limelight `botpose*`, the PhotonVision multi-tag result, which exists when two or more tags are seen)
come from ground truth, while PhotonLib's own estimators on the robot (`PhotonPoseEstimator` single-tag strategies) run on the
emulated detections like on a real robot. Pose ambiguity is always 0.

## 3. Frames and conventions

| Frame | Definition |
|---|---|
| Field | right-handed Z-up NWU, +X toward the opposing wall, origin at the field corner of the layout (WPILib blue-origin) |
| Chassis (SDK) | +X forward, +Y left, +Z up, **origin at the centre of mass** ([ADR 0003](decisions/0003-box3d-chassis-physics.md)); the frame `CameraConfig::local` is relative to when attached to the chassis |
| **WPILib robot** | +X forward, +Y left, +Z up, **origin on the floor (z = 0) below the hull (chassis box) centre, rotated by the chassis yaw only**. What robot code, `LimelightHelpers.setCameraPose_RobotSpace` and PhotonLib's `robotToCamera` use |
| Camera | +X forward (optical axis), +Y left, +Z up |
| Tag | centre of the tag; **+X points out of its face** (`AprilTagFieldLayout` convention) |

`CameraFrame::robotToCamera` and `robotPoseField` are in the WPILib robot frame, so robot code can compare them with its own
`Transform3d`. The robot frame ignores the chassis' pitch and roll (a tilted chassis never shifts the reported robot position),
while the camera pose is the real one, so `robotToCamera` follows the tilt like a camera on a tilting robot. `local` of a chassis
camera is relative to the COM-origin chassis frame, so at rest on a flat floor
`robotToCamera = local + (comOffsetM.x, comOffsetM.y, comHeightM)` (translation; rotation unchanged: the hull centre is `comOffsetM`
away from the COM in x/y). For a camera on a mechanism the link pose is composed first (`linkPoseInChassis(attachment) * local`),
so a turret camera's `robotToCamera` changes as the turret turns.

**Getting the robot-to-camera transform from the editor.** Select the camera: the inspector's *Placement* card shows its pose
relative to the attachment link, followed by a read-only **Robot-to-camera (WPILib robot frame)** line (x, y, z and roll / pitch /
yaw; its tooltip has metres and radians to paste into robot code). For a mechanism-mounted camera it is the rest pose. When your
code already has a `Transform3d` (for example `new Transform3d(0.3, 0.0, 0.5, new Rotation3d(0, Math.toRadians(-20), 0))`),
enter the camera pose so that this line shows the same numbers: for a chassis camera subtract the chassis centre of mass
(Chassis inspector: *Centre of mass* offset x/y from the hull centre and height above the floor) from the translation; rotation is
entered as is (the editor's roll / pitch / yaw are the same right-handed angles as WPILib's `Rotation3d`: a camera pitched 20 degrees up
has pitch = -20 degrees).

Tags and units: layouts are metres, Z-up, face-outward `+X`; pixel corners are `x,y` with the origin at the image top-left,
x to the right, y down. `bearing`: `yawDeg` positive to the **right** of the optical axis and `pitchDeg` positive **up**
(Limelight `tx` / `ty` convention, also PhotonVision's `yaw` / `pitch`: robot code negates yaw to get a CCW angle).

## 4. NetworkTables output

The extension creates the topics in the robot's default NT instance when ntcore becomes available (see section 7; until then the
bridges skip quietly and retry every tick) and updates them from `poll` / `apply` on the clock thread. Every value is stamped with
`INtBackend::nowUs()` = `HAL_GetFPGATime()`, i.e. the robot's `Timer.getFPGATimestamp()` (equal to `nt::Now()` in a simulated
robot); the capture time is always `now - latency`. `<-` marks what robot code (or a dashboard) writes and the bridge mirrors.

### Limelight (`vision.limelight`, table `limelight` / `limelight-<name>`)

Layouts follow the Limelight 2026 firmware and `LimelightHelpers` v1.14 (the version vendored into the sample). Topics are
published on the first available tick as a "no target" state and then once per received frame.

| Topic | Type | Content |
|---|---|---|
| `tv` | double | 1 if at least one tag was seen, else 0 |
| `tid` | double | id of the primary tag (the one with the largest area); -1 when none |
| `tx`, `ty` | double | bearing (+ right) / elevation (+ up) of the primary tag in degrees; `txnc`, `tync` carry the same values (no principal-point offset) |
| `ta` | double | area of the primary tag in % of the image |
| `tl`, `cl` | double | pipeline / capture latency, ms: each is half of `latencyMs`, so `tl + cl` = `latencyMs` |
| `botpose`, `botpose_wpiblue`, `botpose_wpired` | double[] | robot pose (field centre / blue corner / red corner origin): x, y, z, roll, pitch, yaw (m / deg), latency ms, tag count, tag span m, average distance m, average area %, then 7 values per tag (`rawfiducials` layout); **empty** when no tag is seen |
| `botpose_orb`, `botpose_orb_wpiblue`, `botpose_orb_wpired` | double[] | the same arrays for MegaTag 2 (the simulated pose is ground truth plus noise, so the robot-written orientation has no effect) |
| `rawfiducials` | double[] | 7 values per tag, largest first: id, txnc, tync, ta, distance to camera, distance to robot, ambiguity |
| `stddevs` | double[] | 12 values (MegaTag 1 then MegaTag 2: x y z roll pitch yaw) from the camera's `positionNoiseM` / `yawNoiseDeg`; zeros when no tag |
| `t2d`, `tcornxy` | double[] | the 17-value target summary and the primary tag's corners (empty when none) |
| `camerapose_robotspace`, `targetpose_cameraspace`, `targetpose_robotspace`, `camerapose_targetspace`, `botpose_targetspace` | double[6] | geometry of the camera / primary tag (zeros when no tag) |
| `json` | string | a flat `LimelightResults` object as parsed by `LimelightHelpers.getLatestResults`: `pID`, `tl`, `cl`, timestamps, `v`, `tx`, `ty`, `ta`, the `botpose*` arrays, `t6c_rs`, and `Fiducial[]` (`fID`, `fam`, `t6c_ts`, `t6r_*`, `t6t_*`, `ta`, `tx`, `ty`, `txp`, `typ`); the other result arrays are empty |
| `hb` | double | heartbeat, incremented every 100 ms of sim time |
| `getpipe`, `getpipetype` | double, string | the current pipeline index (mirrors the robot-written `pipeline`) and `"pipe_fiducial"`; every pipeline is an AprilTag pipeline |
| `pipeline` <- | double | written by the robot; mirrored into `getpipe` |

Not published or not modelled: `ledMode` and `robot_orientation_set` are accepted but not modelled (LEDs do not influence tag
detection, and the MegaTag 2 pose is ground truth anyway); `camMode`, `crop`, neural / detector / colour results (`rawdetections`,
`tclass`, ...) and the other settings topics are not simulated.

### PhotonVision (`vision.photonvision`, tables `/photonvision` and `/photonvision/<name>`)

| Topic | Type | Content |
|---|---|---|
| `<name>/rawBytes` | raw | the packed-struct `PhotonPipelineResult` PhotonLib deserialises: metadata (sequence id, capture and publish timestamps, `timeSinceLastPong = 0`), the targets (largest first: fiducial id, yaw, pitch, area, skew 0, corners, `bestCameraToTarget` (= alt), ambiguity) and, when two or more tags are seen, the multi-tag result (field-to-camera pose and the ids used). The topic type string is `photonstruct:PhotonPipelineResult:<hash>` and the topic carries the `message_uuid` property PhotonLib's version check reads |
| `version` (root) | string | the project's PhotonLib version string |
| `<name>/heartbeat` | int | incremented every 100 ms of sim time (PhotonLib's `isConnected`) |
| `<name>/pipelineIndexState`, `driverMode`, `fpsLimit`, `fps` | int / bool / int / double | current state; `fps` is the configured frame rate |
| `<name>/pipelineIndexRequest`, `driverModeRequest`, `fpsLimitRequest` <- ; root `ledModeRequest` <- | int / bool | written by PhotonLib; mirrored into the state topics (`ledModeState` at the root). Driver mode publishes empty results |
| `<name>/cameraIntrinsics`, `cameraDistortion` | double[] | 3x3 row-major pinhole intrinsics from `hFovDeg` / `vFovDeg` / resolution; 8 zero distortion coefficients |
| `<name>/hasTarget`, `latencyMillis`, `targetYaw`, `targetPitch`, `targetArea`, `targetSkew`, `targetPixelsX`, `targetPixelsY` | bool / double | the legacy scalar topics for the best target |

Not published: PhotonLib's `.schema` topics, `inputSaveImgCmd` / `outputSaveImgCmd`, object-detection targets, the time-sync
server (PhotonLib's `TimeSync` is never answered; `timeSinceLastPong` is 0 and the latency is carried by the capture / publish
timestamps).

**Supported PhotonLib versions.** The result's type string embeds a hash of the message definition and PhotonLib throws when it
differs from its own, so the bridge serves only versions whose definition hash matches the encoder: **v2025.3.2, v2026.1.1 to
v2026.3.4 and v2026.7 were verified** (the default for an unknown project version is v2026.3.4). `VisionSetup::photonVersion` comes
from the project's `photonlib` vendordep (`ProjectInspector`); for any other version the bridge logs **one error** to the console
and publishes **no PhotonVision camera** (Limelight cameras are unaffected).

## 5. Reconnect, reset and epochs

* The vision state is part of the simulation epoch. The emulator sends `VisionSetup` right after `HelloAck` and again before the
  first frame of every epoch (idempotent: `configure` rebuilds the camera list). When a connection starts, on an applied
  **Reset** / **Begin**, and when a `SensorFrame` adopts a **newer epoch**, `BridgeSet::resetInputHistory` also calls
  `IVisionBridge::reset()` on the clock thread (retries of the same epoch do not): each camera republishes a "no target" state
  (Limelight: `tv` 0, empty pose arrays; PhotonVision: an empty result, heartbeat 0), so robot code never sees a result from
  before the reset.
* Core clears its latency queue on `SimulationSession::reset()` and restarts the camera sequence numbers and capture phases; the
  first tick of the new epoch captures immediately for every camera (frames still in the latency queue are dropped).
* `configure` replaces the camera list but does not unpublish the topics of cameras that were removed; `stop()` (extension
  shutdown) unpublishes every topic the bridge created. After the emulator goes away the topics keep their last value, but the
  heartbeats stop advancing (they only advance with sim time), so `PhotonCamera.isConnected()` turns false and robot code can see
  the staleness; a reconnect restarts everything as above.
* The camera list is fixed when the simulation session is built. When the robot model (including the cameras) was edited since
  the robot was launched, `RealRuntimeHost` relaunches the robot with the new configuration the next time the simulation starts
  (entering the Run tab), like for any other model change.

## 6. Threading

* **Sim thread (emulator)**: owns `VisionSystem` and its queue. It uses only sim time and `SimContext::rng()`; no wall clock,
  no I/O, no ntcore.
* **Extension**: `configure`, `poll`, `apply`, `reset` and `stop` run **only on the clock thread, while the robot is quiescent**
  (between `HALSIM_StepTiming` calls, exactly like `IDeviceBridge::applyInputs`). Never call them from HAL callbacks, from the IPC
  thread or from the watchdog thread, and never block there. ntcore publication is a non-blocking local-queue operation.
* A vision bridge never throws across the SDK interface (`start` returns `Status`; later failures are logged, the affected
  camera is skipped).
* The run view reads `ISimSnapshot::cameras()` (immutable) on the UI thread; the app never talks to ntcore.

## 7. ntcore

The ntcore binding (`wpiemu_bridges_vision_nt`, `nt_binding.h`) has **no link-time dependency on ntcore**. It attaches at run time to
the `ntcore` module the robot process has already loaded (`GetModuleHandle` on Windows, `dlopen(RTLD_NOLOAD)` on Linux/macOS)
and resolves the C API (`ntcore_c.h`) from it, the same approach as the vendor bridges for CTRE. A Java robot loads ntcore through
JNI from an extracted temp directory, possibly after the extension was loaded, so a load-time import would either fail or bind a
private second copy. `INtBackend::available()` is **false until the module is loaded** (and the robot program started): the bridges
publish nothing and retry on every tick, and a robot program that never loads ntcore is unaffected. Timestamps use
`HAL_GetFPGATime()` (resolved from the already loaded wpiHal; `NT_Now()` when there is no HAL, as in unit tests), which is
what `Timer.getFPGATimestamp()` reports and what ntcore itself uses as `nt::Now()` in a simulated robot, so they share one clock and
follow pause and time scale.

CMake: `halsim_ext/cmake/WpiEmuWpilib.cmake` locates the `ntcore-cpp` and `wpinet-cpp` artifacts next to HAL / wpiutil (Maven cache,
extracted root or `-DWPIEMU_WPILIB_DOWNLOAD=ON`) and, when found, creates the imported target `wpiemu_wpilib_ntcore` and sets
`WPIEMU_NTCORE_FOUND`. Only its headers are used to build `wpiemu_bridges_vision_nt` (the DLL is for the smoke test, which loads
it in-process; the robot supplies its own). `wpiemu_bridges_builtin` links the binding and defines `WPIEMU_BUILTIN_VISION_NT` only when it
exists. **Without ntcore** at build time the vision bridges are still registered by `bridges/builtin_bridges.cpp` but with a backend
that is never available, so cameras are inert and nothing else changes; the encoders and their fake-backend tests build and run.
The imported target is owned by `wpilib` in `tools/check_boundaries.py`, like `wpiemu_wpilib_hal`.

Only the robot's **default** NT instance is used (`NT_GetDefaultInstance`), so the robot's `NetworkTableInstance.getDefault()`
sees the topics without any configuration; dashboards (Shuffleboard, AdvantageScope) see them through the robot's NT server as
usual. Verified on Windows with ntcore 2026.2.1 and both a Java and a C++ robot; the Linux/macOS path (`dlopen` by soname) is untested.

## 8. Extension recipe: another vision vendor

Example: a Limelight-like or Photon-like coprocessor from another vendor (a Luxonis / PhotonVision fork, Quest, ...).

1. Add the source files to `bridges/vision/` (or a new `bridges/<name>/` module linking `wpiemu_sdk` only, following the
   device-bridge recipe in [CONTRIBUTING.md](CONTRIBUTING.md)). Keep the encoder a **pure function** from `CameraFrame` (+ its
   `CameraConfig`) to named values / bytes, written through the `INtBackend` seam (`bridges/vision/nt_backend.h`) so it is testable
   with the fake backend, without ntcore.
2. If the vendor needs its own `CameraKind` value, add it to `sdk/config/camera.h` (`CameraKind`, `cameraKindKey`), the codec
   (`"kind"`), the validator and the editor menu (**Add camera**). Pre-release rules apply: change the SDK freely, no migration.
3. Implement `sdk::IVisionBridge`: `name()` unique (`"vision.<vendor>"`); `configure` creates topics only for cameras of your kind
   (ignore the others), idempotently; `apply` publishes frames of your cameras whose `publishTimeUs <= simTimeUs`; `reset` drops
   published state; `stop` is idempotent.
4. Expose `registerVisionBridges(sdk::VisionBridgeRegistry&)` from the module and add one call in
   `bridges/builtin_bridges.cpp` (`registerBuiltinVisionBridges`). The extension and `BridgeSet` do not change.
5. Tests (`tests/bridges_vision/`): golden vectors for every encoded layout from a real device capture or the vendor's documentation,
   a configure / apply / reset / stop sequence on the fake backend, a frame of another kind being ignored, and a unit test that
   a result is not published before its `publishTimeUs`. Run `python tools\check_boundaries.py` (and `--self-test`).
6. Document the topic table in this file.

## 9. Tests

| Where | What |
|---|---|
| `tests/sdk` | the vision headers are included stand-alone (generated self-contained-include check) |
| `tests/runtime/test_vision_system.cpp` | `VisionSystem`: visibility (FOV / range / facing), fps and latency queue, determinism with noise, mechanism-mounted camera, reset |
| `tests/runtime/test_config_codec.cpp`, `test_config_pipeline.cpp`, `test_session*.cpp` | `cameras` JSON round-trip and validation codes, `VisionSetup` / `SensorFrame::vision` in the session and runner |
| `tests/game_2026/test_april_tags.cpp` | the 2026 layout: 32 tags, ids, poses against `2026-rebuilt-welded.json`, face orientation, quaternion order |
| `tests/ipc/test_binary_codec.cpp` | codec round-trip of `SensorFrame::vision` and the `VisionSetup` message |
| `tests/bridges_vision/` | Limelight and PhotonVision bridges and the PhotonPipelineResult encoding against golden vectors, registration, on a fake `INtBackend` |
| `bridges/vision/tests/ntcore_smoke_tests.cpp` (`bridges_vision.ntcore_smoke`) | the ntcore binding against the real ntcore in-process (only when ntcore was found) |
| `halsim_ext/tests/test_vision_fanout.cpp` | `BridgeSet` fan-out: configure on setup, poll / apply per frame, reset on epoch change |
| `tests/app_editor/test_cameras.cc`, `test_camera_frustum.cc` | camera config ops, undo, tree, inspector, viewport |
| `tests/app_runview/test_cameras.cpp` | `CameraRig` (frustum, highlight from the snapshot), AprilTag markers |
| `app/host/tests/e2e_vision_test.cpp` (`e2e.vision_sample`) | opt-in end-to-end on the vision sample robots (below) |

**End-to-end** (opt-in, `WPIEMU_E2E=1`, target `wpiemu_e2e_vision_tests`, CTest `e2e.vision_sample`, label `e2e`; see
[BUILDING.md](BUILDING.md)): it copies `samples\vision_robot\<java|cpp>`, runs it through Gradle with the real extension and a real
session whose `RobotConfig` has a Limelight (`front`, forward) and a PhotonVision camera (`photon-rear`, backward) matching the sample's
`robotToCamera` constants, drives the autonomous routine and compares the robot's own `VISION ...` console lines with the
emulator's ground truth. It asserts that both cameras are connected, tags are seen by the robot code exactly when the emulator says a
camera saw them, the Limelight's `tl + cl` equals the configured latency, the age of the newest PhotonVision result is within the
latency plus one frame spacing and one robot loop, and that the poses at rest equal the ground truth (Limelight within 2 cm,
PhotonVision within 4 cm, because PhotonLib solves from the true, slightly pitched camera pose). The Java robot must also drive
forward. Both samples pass.

## 10. Limitations

* AprilTags only. No occlusion, lens distortion, exposure, lighting, motion blur or colour / neural / object detection pipelines.
* The coprocessor-side robot pose is ground truth plus the configured noise, not a perspective solve; pose ambiguity is 0.
* The robot-to-camera transform is configured in the editor; it is not read from the robot program, so keep it equal to what
  your code uses.
* PhotonVision works only for PhotonLib versions whose result definition hash is known (v2025.3.2, v2026.1.1 to v2026.3.4, v2026.7);
  Limelight arrays follow the pinned `LimelightHelpers` v1.14 layout.
* Camera settings the robot writes are mirrored (pipeline index, driver mode, fps limit) or accepted but not modelled
  (Limelight `ledMode` and `robot_orientation_set`, Photon LED mode, crop...); none of them changes what is detected. PhotonLib's `.schema`
  topics are not published.
* One season layout per run (`IGameModule::aprilTagLayout()`); a project whose season has no game plugin has no tags and cameras see nothing.
* PhotonLib for C++ reports results about one NetworkTables cycle older than PhotonLib for Java (the end-to-end test allows more slack
  for C++).
* The C++ sample's Phoenix 6 drive did not move the robot in the emulator (Phoenix 6 simulation from C++ is unverified, ADR 0002 R8),
  so the C++ sample was verified **stationary**; vision itself works for C++ robots (both cameras, tags, latency and poses were checked).
* Linux and macOS: the ntcore binding (`dlopen` by soname) and the rest of the vision path were not run there.
