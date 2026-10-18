# Vision cameras (`bridges/vision`, core `VisionSystem`)

Simulated **Limelight** and **PhotonVision** coprocessors for AprilTag pose estimation. The emulator decides which tags each
camera sees; the HALSim extension, inside the robot process, publishes the same NetworkTables data a real coprocessor would, into
the robot's default NT instance, with simulated latency. The robot project is not modified. Rationale and rejected options:
[ADR 0017](decisions/0017-vision-cameras.md). User-level steps: [USER_GUIDE.md](USER_GUIDE.md) section 3 (*Cameras*).

Everything vision-specific is optional: with no entry in `RobotConfig::cameras` (or a season with no tag layout) nothing is
created, published or sent.

<!-- VERIFY: file/target names below are the planned layout; compare with bridges/ and core/runtime after integration -->
```
sdk/include/wpiemu/sdk/
  config/camera.h              CameraKind, CameraConfig (persisted in RobotConfig::cameras)
  vision/apriltag.h            AprilTag, AprilTagLayout
  vision/camera_frame.h        TagDetection, CameraFrame, CameraSetup, VisionSetup
  vision/vision_bridge.h       IVisionBridge, VisionBridgeRegistry
  devices/builtin_bridges.h    registerBuiltinVisionBridges(VisionBridgeRegistry&)  (aggregation seam)
  ipc/messages.h               SensorFrame::vision, the VisionSetup message
  telemetry/snapshot.h         CameraState, ISimSnapshot::cameras()
core/runtime/                  VisionSystem (owned by SimulationSession), cameras codec + validation
plugins/game_2026/             AprilTag layout 2026-rebuilt-welded (IGameModule::aprilTagLayout)
bridges/vision/                Limelight + PhotonVision encoders, INtPublisher seam, registerVisionBridges
bridges/vision_nt/             the real ntcore binding of INtPublisher (needs ntcore; see Building)
bridges/builtin_bridges.cpp    registerBuiltinVisionBridges -> bridges/vision (extension-side aggregation seam)
tests/runtime, tests/bridges_vision, tests/ipc, tests/app_editor, tests/app_runview   (see Tests)
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
 |     go into SensorFrame::vision          |                      |  INtPublisher -> ntcore (default NT inst) |
 |  snapshot: CameraState (run view)        |                      |  then HALSIM_StepTiming(stepUs)           |
 +------------------------------------------+                      +----------------------+--------------------+
                                                                                          |
                                                                       robot code: LimelightHelpers / PhotonCamera
```

Ownership follows the rest of the project (ARCHITECTURE section 1): the SDK holds the data types and the `IVisionBridge`
seam; core (Qt-free, season-free) computes visibility; the season's tag field comes from the game plugin; the bridges run in
the robot process and depend only on the SDK (ntcore stays behind `INtPublisher`); the app only shows the result.

### Data flow per tick

1. At connect, after `HelloAck`, the emulator sends a **`VisionSetup`** (`CameraSetup` per configured camera, the robot's
   PhotonLib version, the layout name). The extension hands it to every `IVisionBridge::configure`, which creates the topics of the
   cameras of its kind. It is sent again for every new `runEpoch` (Reset, reconnect).
2. Sim thread, tick N, after the **physics step and the game step** (ARCHITECTURE 4, steps 4-5): `VisionSystem` resolves each
   camera's world pose from the final chassis pose, the live link pose of its attachment and `local`. Cameras whose capture time
   has come (period `1/fps`) compute a `CameraFrame` and queue it until `simTime >= capture + latencyMs`.
3. `buildSensorFrame` moves every frame **due now** (`publishTimeUs <= simTime`) into `SensorFrame::vision`; the snapshot carries
   `CameraState` (world pose, `tagsVisible`, `visibleTagIds` of the last captured frame) for the run view.
4. Extension, on receipt: `BridgeSet` routes `SensorFrame::vision` to the vision bridges' `apply(frames, simTimeUs)`
   **before `HALSIM_StepTiming`**, so the robot loop that runs next sees results published up to this sim time; then `poll`
   mirrors what the robot wrote (pipeline, LED, driver mode) and heartbeats advance.
5. `HALSIM_StepTiming(stepUs)` runs the robot code, which reads the NT topics through `LimelightHelpers` / PhotonLib.

<!-- VERIFY: ordering of apply vs poll vs StepTiming in BridgeSet (bridge stream) -->

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

A tag is **seen** by a camera at capture time when all of these hold (angles and distances are evaluated in the camera frame):

1. its centre is inside the camera's rectangular field of view (|bearing| <= `hFovDeg`/2, |elevation| <= `vFovDeg`/2) and in front of the camera;
2. the distance from the camera to the tag centre is <= `maxRangeM`;
3. the tag faces the camera: the angle between the tag's outward normal (+X of the tag) and the vector from the tag to the
   camera is <= `maxIncidenceDeg`, and the camera is on the front side of the tag.

<!-- VERIFY: whether the field-of-view test uses the tag centre only or all four corners; whether a minimum range exists -->

There is **no occlusion** (a tag behind the hub or another robot is still seen), no distortion, no exposure and no minimum pixel
size. A frame with no visible tag is still produced and published (that is how a real pipeline reports "no target").

### Rate and latency

* A camera captures once per `1/fps`. The emulator steps in fixed ticks (default 5 ms), so the capture happens on the first
  tick boundary at or after the ideal capture time; `CameraFrame::captureTimeUs` records the real sim time. The sequence number
  counts frames captured by that camera since the session started (it restarts at every epoch).
* The result is published `latencyMs` after capture, quantised to the tick the same way: `publishTimeUs >= captureTimeUs + latency`.
  Robot code sees it on the first loop after that tick, so a 20 ms periodic loop observes the result up to one period later.
* The data carries the **capture** time (Limelight: `now - (tl + cl)` is the capture time; PhotonVision: the result's capture
  timestamp), so `addVisionMeasurement` receives the timestamp of the moment the robot was where the camera saw it.

### Noise

`positionNoiseM` / `yawNoiseDeg` perturb the robot pose a camera reports (`CameraFrame::robotPoseField`); detections of single
tags (`cameraToTag`, bearings, corners) are noise-free. Noise comes from `SimContext::rng()` (seeded by `RobotConfig::sim.seed`),
so a run is reproducible. <!-- VERIFY: noise is Gaussian, drawn once per frame; confirm with core VisionSystem -->

### Robot pose (multi-tag)

When at least one tag is seen, `CameraFrame::hasRobotPose` is true and `robotPoseField` is the **ground-truth robot pose in the
field frame** (plus noise), i.e. what a perfect solver would return from the visible tags. `avgTagDistM`, `avgTagAreaPercent` and
`tagSpanM` (largest distance between two seen tags, 0 for one) are filled for the Limelight `botpose` metadata. There is no solver:
Photon's multi-tag/`estimatedPose` on the robot side runs on the emulated detections like on a real robot, while the *coprocessor-side*
pose (Limelight `botpose*`, Photon multi-tag result) comes from ground truth. <!-- VERIFY: Photon multitag result is filled -->
Pose ambiguity is always 0.

## 3. Frames and conventions

| Frame | Definition |
|---|---|
| Field | right-handed Z-up NWU, +X toward the opposing wall, origin at the field corner of the layout (WPILib blue-origin) |
| Chassis (SDK) | +X forward, +Y left, +Z up, **origin at the centre of mass** ([ADR 0003](decisions/0003-box3d-chassis-physics.md)); the frame `CameraConfig::local` is relative to when attached to the chassis |
| **WPILib robot** | +X forward, +Y left, +Z up, **origin on the floor below the hull (chassis box) centre**, yaw = chassis yaw. What robot code, `LimelightHelpers.setCameraPose_RobotSpace` and PhotonLib's `robotToCamera` use |
| Camera | +X forward (optical axis), +Y left, +Z up |
| Tag | centre of the tag; **+X points out of its face** (`AprilTagFieldLayout` convention) |

`CameraFrame::robotToCamera` and `robotPoseField` are always in the WPILib robot frame, so robot code can compare them with its own
`Transform3d`. The editor stores `local` relative to the attachment link, so the two differ by the centre of mass for a
chassis-mounted camera: `robotToCamera = (comOffsetXY, comHeightM) + local` (translation; rotation is unchanged). For a camera on
a mechanism the editor's link pose is composed first (`linkPoseInChassis(attachment) * local`) and the same COM shift applies
(the pose of the camera at capture time, so a turret camera's `robotToCamera` changes as it turns).

**Getting the robot-to-camera transform from the editor.** Select the camera: the *Placement* card shows its pose relative to the
attachment link (translation in metres or inches, rotation as yaw/pitch/roll). For a chassis camera add the chassis COM
offset (Chassis inspector: *Centre of mass*, x/y from the hull centre, z = COM height above the floor) to the translation to get
the WPILib `Transform3d` translation your robot code uses; rotation is the same. Conversely, when the code already says
`new Transform3d(0.3, 0.0, 0.5, new Rotation3d(0, Math.toRadians(-20), 0))`, subtract the COM and enter the result as the camera pose
(a camera pitched 20 degrees up has pitch = -20 degrees in WPILib's right-handed rotation convention).
<!-- VERIFY: inspector placement card labels / whether the editor offers a "robot frame" readout; angle sign convention shown in the UI -->

Tags and units: layouts are metres, Z-up, face-outward `+X`; pixel corners are `x,y` with the origin at the image top-left,
x to the right, y down. `bearing`: `yawDeg` positive to the **right** of the optical axis and `pitchDeg` positive **up**
(Limelight `tx` / `ty` convention, also PhotonVision's `yaw` / `pitch`: robot code negates yaw to get a CCW angle).

## 4. NetworkTables output

The extension creates the topics in the robot's default NT instance when it receives `VisionSetup` and updates them from
`apply` on the clock thread with **explicit timestamps from sim time** (`HAL_GetFPGATime`). Direction `->` is what the simulation
publishes, `<-` what robot code (or a dashboard) writes and the bridge mirrors.

### Limelight (`vision.limelight`, table `limelight` / `limelight-<name>`)

Layouts follow `LimelightHelpers` v1.14 <!-- VERIFY: exact helper version and array layouts, see bridges/vision sources -->.

| Topic | Type | Content |
|---|---|---|
| `tv` | double | 1 if at least one tag was seen, else 0 |
| `tid` | double | id of the primary (largest / nearest) tag; -1 when none |
| `tx`, `ty` | double | bearing / elevation of the primary tag in degrees (`txnc`, `tync`: the same relative to the principal point) |
| `ta` | double | area of the primary tag in % of the image |
| `tl`, `cl` | double | pipeline and capture latency, ms (`tl + cl` = `latencyMs`) <!-- VERIFY: split between tl and cl --> |
| `botpose`, `botpose_wpiblue`, `botpose_wpired` | double[] | robot pose in the respective field frame: x, y, z, roll, pitch, yaw (m / deg), total latency ms, tag count, tag span m, average distance m, average area %, then per-tag fiducial blocks |
| `botpose_orb_wpiblue`, `botpose_orb_wpired` | double[] | same layout, MegaTag 2 (ground truth plus noise; robot orientation set by the robot is ignored) |
| `rawfiducials` | double[] | 7 values per tag: id, txnc, tync, ta, distance to camera, distance to robot, ambiguity |
| `stddevs` | double[] | MegaTag standard deviations (fixed values derived from the noise settings) <!-- VERIFY --> |
| `json` | string | minimal results JSON: `Results.Fiducial[]` (`fID`, `tx`, `ty`, `ta`, `t6c_ts`, ...), `tl`, `cl`, `botpose*` <!-- VERIFY: exact fields --> |
| `hb` | double | heartbeat, incremented every poll |
| `pipeline` / `ledMode` <- | double | written by the robot; mirrored into `getpipe` / reported state, no other effect |

Other Limelight topics (neural/detector results `rawdetections`, `tclass`, colour-target data, `crop`, `camMode`, `t6c_*`) are not
published. <!-- VERIFY: list of unsupported topics -->

### PhotonVision (`vision.photonvision`, table `/photonvision/<name>`)

| Topic | Type | Content |
|---|---|---|
| `rawBytes` | raw | the packed-struct `PhotonPipelineResult` PhotonLib deserialises (metadata: capture / publish timestamps, sequence id, `timeSinceLastPong = 0`; the targets with fiducial id, yaw, pitch, area, skew, corners, `bestCameraToTarget`, ambiguity; the multi-tag result when two or more are seen). Layout and struct type string follow the **PhotonLib version** reported by `VisionSetup` (pinned, below) |
| `version` | string | the PhotonLib version string PhotonLib compares against its own |
| `heartbeat` | int | incremented every poll (PhotonLib's `isConnected`) |
| `pipelineIndexState`, `driverMode`, `ledModeState` | int / bool | the current state, mirroring the robot's `*Request` topics |
| `pipelineIndexRequest`, `driverModeRequest`, `ledModeRequest` <- | int / bool | written by PhotonLib; mirrored into the state topics |
| `cameraIntrinsics`, `cameraDistortion` | double[] | pinhole intrinsics derived from `hFovDeg`/`widthPx`/`heightPx`; zero distortion |

<!-- VERIFY: exact topic list and the struct type strings in bridges/vision sources -->
Not simulated: `inputSaveImgCmd` / `outputSaveImgCmd`, object-detection targets, the time-sync server (PhotonLib's `TimeSync` is
never answered; `timeSinceLastPong` is faked as 0 and the latency is carried by capture / publish timestamps).

**Supported PhotonLib versions.** The packed-struct layout is version specific, so the encoder supports a pinned list
(`v2026.3.4` first, see `bridges/vision`). The launcher reads the project's `photonlib` vendordep version (`ProjectInspector`,
[runtime.md](runtime.md)) and passes it as `VisionSetup::photonVersion`; an empty or unsupported version logs a warning and uses
the default layout, in which case `PhotonCamera` may report the camera as disconnected or fail to decode results.
<!-- VERIFY: supported version list, behaviour on unsupported version -->

## 5. Reconnect, reset and epochs

* The vision state is part of the simulation epoch. On a **new connection** (including the extension's automatic reconnect) the
  emulator resends `VisionSetup`; on **Reset**, **Begin** and when a `SensorFrame` adopts a **newer epoch**, `BridgeSet` calls
  `IVisionBridge::reset()` on the clock thread: queued frames are dropped and the published results are cleared so robot code
  never sees a result from before the reset. Retries (same epoch) do not reset.
* Core clears its latency queue and restarts camera sequence numbers and capture phases on `SimulationSession::reset()` (tick 0 of
  the new epoch captures immediately for every camera). <!-- VERIFY: first capture at tick 0 -->
* Topics are (re)created by `configure`, which is idempotent: cameras that were removed from the config stop being updated, and
  their topics are unpublished. After the emulator goes away, the topics keep their last value until the next connection (a real
  coprocessor does the same when its stream stalls); robot code observing `heartbeat` / timestamps sees staleness.
  <!-- VERIFY: removed-camera topics are unpublished, and topics keep their last value after a disconnect -->
* Editing `cameras` while a robot is running takes effect for the next robot run / `VisionSetup`; the editor's running instance is
  restarted like any other robot-model change. <!-- VERIFY: whether camera edits are live-applied -->

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

The real publisher links **ntcore** (headers + `ntcore` desktop library from the same WPILib Maven cache as HAL and wpiutil),
discovered by `halsim_ext/cmake/WpiEmuWpilib.cmake` as the imported target `wpiemu_wpilib_ntcore` (owner `wpilib` in
`tools/check_boundaries.py`, like `wpiemu_wpilib_hal`); CMake sets `WPIEMU_NTCORE_FOUND`. When ntcore is not found, the vision
bridges are still registered by `bridges/builtin_bridges.cpp` but `start` fails with a clear message (`Status`) and cameras are inert;
the encoders and their tests (fake publisher) build and run without ntcore. <!-- VERIFY: behaviour when WPIEMU_NTCORE_FOUND is off -->

Only the robot's **default** NT instance is used (`nt::GetDefaultInstance`), so the robot's `NetworkTableInstance.getDefault()`
sees the topics without any configuration; dashboards (Shuffleboard, AdvantageScope) see them through the robot's NT server as
usual. Explicit timestamps are taken from sim time (`HAL_GetFPGATime`), not `nt::Now()`, so they follow pause and time scale.
If linking ntcore by name were to load a second copy next to the one the robot already uses (JNI for Java robots), the binding
resolves the symbols from the already-loaded module instead, like the vendor `symbol_resolver`. <!-- VERIFY: outcome of the Java loading-order check in the sample -->

## 8. Extension recipe: another vision vendor

Example: a Limelight-like or Photon-like coprocessor from another vendor (a Luxonis / PhotonVision fork, Quest, ...).

1. Add the source files to `bridges/vision/` (or a new `bridges/<name>/` module linking `wpiemu_sdk` only, following the
   device-bridge recipe in [CONTRIBUTING.md](CONTRIBUTING.md)). Keep the encoder a **pure function** from `CameraFrame` (+ its
   `CameraConfig`) to named values / bytes, written through the module's `INtPublisher`-style seam so it is testable without ntcore.
2. If the vendor needs its own `CameraKind` value, add it to `sdk/config/camera.h` (`CameraKind`, `cameraKindKey`), the codec
   (`"kind"`), the validator and the editor menu (**Add camera**). Pre-release rules apply: change the SDK freely, no migration.
3. Implement `sdk::IVisionBridge`: `name()` unique (`"vision.<vendor>"`); `configure` creates topics only for cameras of your kind
   (ignore the others), idempotently; `apply` publishes frames of your cameras whose `publishTimeUs <= simTimeUs`; `reset` drops
   published state; `stop` is idempotent.
4. Expose `registerVisionBridges(sdk::VisionBridgeRegistry&)` from the module and add one call in
   `bridges/builtin_bridges.cpp` (`registerBuiltinVisionBridges`). The extension and `BridgeSet` do not change.
5. Tests (`tests/bridges_vision/`): golden vectors for every encoded layout from a real device capture or the vendor's documentation,
   a configure / apply / reset / stop sequence on a fake publisher, a frame of another kind being ignored, and a unit test that
   a result is not published before its `publishTimeUs`. Run `python tools\check_boundaries.py` (and `--self-test`).
6. Document the topic table in this file.

## 9. Tests

<!-- VERIFY: test file names below are the planned ones; check them against tests/ after integration -->
| Where | What |
|---|---|
| `tests/sdk` | the vision headers are included stand-alone (generated self-contained-include check) |
| `tests/runtime` (`test_vision*.cpp`, camera cases in the codec / validator tests) | visibility rules, FOV / range / facing, fps and latency queue, determinism with noise, mechanism-mounted camera, `cameras` JSON round-trip and validation codes |
| `tests/plugins`, `tests/game_2026` | the 2026 tag layout: tag count, ids, face orientation, field size |
| `tests/ipc` | codec round-trip of `SensorFrame::vision` and the `VisionSetup` message |
| `tests/bridges_vision` | Limelight and PhotonVision encoders against golden vectors, `IVisionBridge` configure / apply / reset behaviour on a fake `INtPublisher` |
| `halsim_ext/tests` | `BridgeSet` fan-out (setup, frames, reset on connect / Reset / epoch change) |
| `tests/app_editor/test_cameras.cc` | camera config ops, undo, tree, inspector, viewport |
| `tests/app_runview` | `CameraRig` (frustum, highlight from the snapshot) |
| `app/host/tests/e2e_test.cpp` | opt-in end-to-end on the vision sample robots (below) |

End-to-end (opt-in, `WPIEMU_E2E=1`, see [BUILDING.md](BUILDING.md)): it runs `samples\vision_robot\java` (and `...\cpp`) through
Gradle with the real extension and asserts that both cameras report tags, the estimated pose matches the ground truth, and
`now - resultTimestamp` matches the configured latency. <!-- VERIFY: e2e test case name and assertions -->

## 10. Limitations

* AprilTags only. No occlusion, lens distortion, exposure, lighting, motion blur or colour / neural / object detection pipelines.
* The coprocessor-side robot pose is ground truth plus the configured noise, not a perspective solve; pose ambiguity is 0.
* The robot-to-camera transform is configured in the editor; it is not read from the robot program, so keep it equal to what
  your code uses.
* The Photon encoder supports a pinned list of PhotonLib versions; Limelight arrays follow the pinned `LimelightHelpers` layout.
* Camera settings the robot writes (pipeline, LED, driver mode, crop, ...) are mirrored but do not change what is detected.
* One season layout per run (`IGameModule::aprilTagLayout()`); a project whose season has no game plugin has no tags and cameras see nothing.
