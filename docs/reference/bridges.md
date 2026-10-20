# Bridges

Read this when extending a device adapter or camera encoder, or diagnosing robot-facing feedback.
Prerequisites: [Architecture](../ARCHITECTURE.md) for module boundaries and
[Extension and IPC](extension-ipc.md) for clock-thread, epoch, and recovery rules.
All add-a-bridge/vendor recipes live in [Extending](../extending.md); camera setup is in the
[User guide](../USER_GUIDE.md).

## Registration and device contract

The extension instantiates SDK `DeviceBridgeRegistry` and `VisionBridgeRegistry` factories through
[the built-in aggregator](../../bridges/builtin_bridges.cpp). Individual bridge modules do not
depend on each other, and the extension does not include their concrete interfaces.
Device adapters implement [IDeviceBridge](../../sdk/include/wpiemu/sdk/devices/device_bridge.h);
camera coprocessors implement [IVisionBridge](../../sdk/include/wpiemu/sdk/vision/vision_bridge.h),
not HAL/CAN devices. ntcore and vendor APIs stay behind bridge-local seams, outside SDK headers.

A device announcement supplies capabilities and ordered signal specifications; frame signal indices
address that list. Announcements are idempotent by key, including updates as late values appear.
`collectOutputs` appends the full robot-output snapshot; the extension performs delta compression.
`applyInputs` receives only owned devices, and `applyLoopback` handles unmapped devices locally.
Directions are from the simulator: Input is written to the robot, Output is read from it.
[Canonical signals](../../sdk/include/wpiemu/sdk/devices/signals.h) own names and units, not this page.

## HAL families

[ChannelFamilyBridge](../../sdk/include/wpiemu/sdk/devices/channel_family_bridge.h) discovers indexed
HAL families by initialized-state polling and retires them when freed. The
[HAL API seam](../../bridges/hal/include/wpiemu/bridges/hal/hal_api.h) permits fake backends;
the real [binding](../../bridges/hal/src/hal_sim_binding.cpp) obtains counts from HAL and bounds
indices before calling HALSIM getters/setters. The
[registration](../../bridges/hal/src/registration.cpp) is the authority on shipped families.

- PWM speed is an already normalized robot command. DIO direction is chosen at discovery; the
  bridge never writes a pin the robot configured as output.
- Encoder rotations/rps need an injected counts-per-revolution assumption; HAL does not supply
  mechanism gearing. The [sensor adapters](../../bridges/hal/src/sensor_bridges.cpp) honor robot
  encoder reversal and give newly initialized zero-frequency duty-cycle inputs a connected default.
- A solenoid channel is latched as discovered after first ON until its module is freed, unless
  [options](../../bridges/hal/include/wpiemu/bridges/hal/hal_bridges.h) request all channels.
- [HAL CAN](../../bridges/hal/src/can_bridge.cpp) records sends and announces observations later;
  it never answers CAN traffic or emulates a controller. Its callback has no bus identity; the
  descriptor uses `rio`. Seeing an unknown vendor's CAN frames is not a working vendor bridge.

## Declared address versus physical identity

[Device identity](../../sdk/include/wpiemu/sdk/devices/device_identity.h) distinguishes a declared
`DeviceKey` (family/type/id/bus/channel) from `PhysicalDeviceId` (the same address without bus).
Bus labels in robot configuration do not prove that the vendor simulator can distinguish devices.
The shipped [CTRE rules](../../bridges/vendor/src/ctre_rules.cpp) all use `busIsStatic=false`:
SimDevice names carry no bus, so descriptors retain `kBusUnknown`. Never infer `rio` or CANivore
from a static rule. Runtime observations may supply a label through `IBridgeHost::observeBus`;
the optional hook-provider interface exists, but no inline-hook provider ships.

Several declared bus addresses can alias one physical simulator device. One mapping is a warning;
independent mappings through distinct aliases are a validation error. If such a configuration
reaches the [hub](../../core/runtime/src/runtime/device_hub.cpp), conflicting devices are treated
as unmapped: slots read neutral data and ignore writes, with local loopback and an error warning.
Mappings are retained so correcting the conflict can restore operation.

## Vendor rules and firmware feedback

The shipped [vendor registration](../../bridges/vendor/src/vendor_bridges.cpp) creates Phoenix 6
and Phoenix 5 adapters over HAL SimDevices, not a general CAN protocol emulator.
[SimDeviceBridge](../../sdk/include/wpiemu/sdk/devices/sim_device_bridge.h) full-matches device-name
rules, groups children, and maps raw values with `canonical = raw * scale + offset` (inverse on write).
An unmatched device is ignored. For explicit binding tables, unmatched values are ignored too;
only a deliberately empty table auto-exposes native value names and HAL directions.
That fallback does not infer canonical signals or capabilities. Phoenix 5 includes native sensor
exposure and a discover-only Pigeon IMU; it does not inherit Phoenix 6's input guard/config readback.

For Phoenix motors, the emulator drives rotor feedback while firmware applies ratios and offsets.
The plant consumes firmware-resolved `motorVoltage`; the hub falls back to duty × battery only
when voltage is absent. Phoenix `torqueCurrentReported` is telemetry, **not** the canonical
torque-current request: feeding it back as a request double-models the controller.
Robot controller inversion is already reflected in outputs. Model-to-device mounting reversal is
the separate per-mapping contract in [Mechanisms](mechanisms.md), not another vendor-side inversion.

Mapped CANcoder feedback is a calibrated model-frame position. With observed finite magnet offset
`o` and known direction `s` (`-1` clockwise-positive, `+1` otherwise), the
[vendor adapter](../../bridges/vendor/src/vendor_sim_bridge.cpp) writes:

```text
rawPosition = s * (modelPosition - o)
rawVelocity = s * modelVelocity
```

Firmware then reports `s * rawPosition + o`. Use actual readback, including its quantization;
do not copy assumed robot constants. Until calibration is usable, feedback remains raw passthrough;
disabled/incomplete/unsupported readback is diagnosed. Model-side sensor offsets are separate; see
[Runtime](runtime.md). Config snapshots are read after discovery from quiescent output collection,
with bounded attempts once the native API is available, not continuously after robot reconfiguration.
`poll()` never enters CTRE native config code.
Firmware limit-polarity configuration is not automatically applied by the bridge/hub.

The vendor wrapper discovers values added after a device's creation, appends stable signal indices,
and re-announces. Its guard batches integrated Phoenix raw inputs, unwraps yaw continuously, skips
unchanged writes, and checks readback when committing inputs. Equivalent yaw modulo a turn is not
a disturbance; encoder positions are not compared modulo a turn. Robot `setYaw`/`setPosition`
offsets stay firmware-owned. Persistent competing writes are diagnosed; disabling writes for a
device also suppresses its loopback. The optional mirror-reassert helper is not wired into the
shipped extension; do not assume it runs before each robot loop.
See the [guard](../../bridges/vendor/src/guarded_backend.cpp) for these policies.

While devices are live, vendor pacing advice requests at most real time and marks pauses unsafe.
It is advisory: the shipped extension does not apply it.
Without a HAL backend the vendor bridges register but discover nothing and warn at startup.

## Vision frame and time contract

The [camera frame](../../sdk/include/wpiemu/sdk/vision/camera_frame.h) is vendor-neutral.
`cameraIndex` addresses `VisionSetup::cameras`; each encoder ignores other kinds/unknown indices.
[VisionSystem](../../core/runtime/src/runtime/vision_system.cpp) captures on the simulation tick
grid, assigns a per-camera sequence, queues configured latency, and emits only due frames.
`IVisionBridge::apply` expects that due-only batch: the shipped encoders do not independently reject
future `publishTimeUs`. Reset clears queued captures and restarts capture sequences.

The producer uses blue-origin NWU field geometry and a WPILib robot frame on the floor below the
hull center, with yaw only, not the chassis COM frame. `robotToCamera` follows the actual attachment
and tilt relative to that robot frame. Tag +X is face-outward; bearing is positive right, elevation
positive up, and image pixels run right/down from the top-left.
Visibility tests tag centers for FOV, range, and facing; there is no occlusion or optical distortion.
Detections have zero ambiguity; coprocessor robot poses are ground truth plus configured noise,
not a perspective solve. Robot-side estimators can still solve from the encoded detections.

Publication timestamps use the robot clock, **not** resettable epoch sim time. The bridges sample
backend `nowUs()` before HAL stepping; capture time is that publication time minus
`max(0, frame.publishTimeUs - frame.captureTimeUs)`. Reset does not rewind HAL's FPGA clock.
Heartbeats advance with simulated time but are not new captures or proof of raw-result compatibility.

## ntcore binding and lifecycle

The [real backend](../../bridges/vision/src/ntcore_backend.cpp) resolves the C API from the robot's
already-loaded ntcore and uses its default NT instance. It has no link-time ntcore dependency.
Production attaches only; the load-if-absent option is for tools/tests, not a private robot runtime.
Absent modules are retried lazily because loading can follow extension initialization; missing
required exports instead diagnose and disable that binding. Without the optional build-time
binding, registered vision bridges stay unavailable and publish nothing.

`nowUs()` prefers `HAL_GetFPGATime`, falling back to `NT_Now` without a usable HAL clock.
The publisher initializes C API options with `structSize`, `sendAll`, and `keepDuplicates`:
unchanged ideal pose arrays must still arrive as new timestamped samples. Zero/nonpositive publish
timestamps become `1`, because ntcore reserves zero for "now"; this is not a general monotonic clamp.
Reads copy ntcore-owned values before `NT_DisposeValue`; unassigned values become no request, not
a default setting. Subscriber-only topics therefore must not be treated as robot writes.

Configure rebuilds camera state but does not sweep removed cameras' publishers. Reset republishes
empty/no-target results immediately if initialized and available, otherwise on the next available
tick. Stop unpublishes current camera-prefix topics; Photon root state is not swept by that loop.

## Limelight interoperability

The [encoder](../../bridges/vision/src/limelight_bridge.cpp) publishes under `/limelight` or
`/limelight-<name>`. The `botpose*` arrays contain six pose values (meters, roll/pitch/yaw degrees), then
latency ms, tag count, span m, mean distance m, mean area %, then seven values per fiducial:
`id, txnc, tync, areaPercent, distanceToCameraM, distanceToRobotM, ambiguity`.
Largest area is primary; no-target pose arrays are empty, not a zero pose.
`tl` and `cl` each carry half the frame latency; their sum reconstructs capture age.
MegaTag 2 arrays carry the same simulated estimate, not an orientation-conditioned solve.

Blue arrays preserve blue-origin poses; red uses the opposite corner with a half-turn, and bare
`botpose` uses field-center origin. Conversion dimensions come from
[VisionBridgeOptions](../../bridges/vision/include/wpiemu/bridges/vision/vision_bridges.h), not the
layout name. Geometry arrays use [pose6](../../bridges/vision/src/vision_math.h) directly in NWU;
there is no extra optical-axis remapping. `camerapose_robotspace` comes from model frames, not a
robot-written transform request. Pipeline index is mirrored; LED/orientation/crop settings do not
change detections. The encoder is AprilTag-only, not a complete Limelight firmware implementation.

## PhotonVision interoperability

[Photon encoding](../../bridges/vision/src/photon_encoding.cpp) defines the exact packed schema.
`/photonvision/<name>/rawBytes` uses type
`photonstruct:PhotonPipelineResult:4b2ff16a964b5e2bf04be0c1454d91c4`;
its `message_uuid` property is the JSON **string** `"4b2ff16a964b5e2bf04be0c1454d91c4"`.
Both must match PhotonLib, not merely the root version text or a connected heartbeat.
An initial empty raw result establishes that type/property before real detections.

Metadata is four little-endian `i64`s: sequence, capture microseconds, publication microseconds,
and time-since-last-pong (zero). Transforms encode translation then quaternion `w,x,y,z`, unlike
SDK quaternion member order. Lists have a one-byte count capped at 127; optional multi-tag data
uses a presence byte and signed 16-bit fiducial IDs. The
[packet contract](../../bridges/vision/include/wpiemu/bridges/vision/photon_encoding.h) owns the rest.
Results copy capture sequences, not heartbeat counts. Multi-tag results require at least two tags
and a robot pose; their transform is field-to-camera and their used IDs are sorted.

The [bridge](../../bridges/vision/src/photon_bridge.cpp) checks the explicit PhotonLib allowlist in
the encoder. An empty project version uses its built-in default; an unrecognized nonempty version
logs an error and suppresses Photon cameras, without disabling Limelight. Do not infer support for
every intervening release from similar version numbers. Driver-mode requests produce empty results;
pipeline/fps-limit/LED requests mirror state but do not change the capture model. Intrinsics are a
row-major 3×3 pinhole matrix with eight zero distortion coefficients. No time-sync server, `.schema`
publication, image stream, or object-detection pipeline is supplied.

[Golden-vector checks](../../tests/bridges_vision/test_photon_encoding.cpp) and fake publishers
cannot establish runtime ABI compatibility alone; the
[real ntcore checks](../../bridges/vision/tests/ntcore_smoke_tests.cpp) and
[robot integration](../../app/host/tests/e2e_vision_test.cpp) exercise that boundary.
[Building](../BUILDING.md) owns test commands, including the Java and C++ sample prerequisites.

Next: [Game plugins](game-plugin.md).
