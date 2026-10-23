# Add a vision coprocessor adapter

Core decides what a camera sees and when its result is due. A vision bridge only converts and
publishes that due result in the robot-facing vendor protocol. Keeping those owners separate
prevents timestamp, visibility and scheduling rules from leaking into encoders.

See the [vision bridge contract](../reference/bridges.md) for publication semantics.

## Start from a tested camera path

Read [`IVisionBridge`](../../sdk/include/wpiemu/sdk/vision/vision_bridge.h), the
[`INtBackend`](../../bridges/vision/include/wpiemu/bridges/vision/nt_backend.h) publication seam,
and the existing [vision module](../../bridges/vision/src).
Limelight demonstrates topic arrays/JSON; PhotonVision separates its binary encoder in
[`photon_encoding.cpp`](../../bridges/vision/src/photon_encoding.cpp).

This exact test excerpt from
[`test_limelight_bridge.cpp`](../../tests/bridges_vision/test_limelight_bridge.cpp), inside
`vision limelight: no tag publishes tv 0 and empty arrays`, exercises an empty result:

```cpp
Rig rig;
configureLimelight(rig);
rig.apply({makeFrame(0, {})}, 1'040'000);
CHECK(rig.nt->lastDouble(kLL + "/tv") == 0.0);
CHECK(rig.nt->lastDouble(kLL + "/tid") == -1.0);
```

Use its `vision_test` rig/helpers and `kLL` topic prefix. Expected result: publishing a no-tag frame
actively clears the valid-target flag and target ID; it must not leave the previous target visible.
Adapt this pattern alongside a one-tag and multi-tag golden payload when adding an encoder.

## Implement the lifecycle in order

1. `start`: initialize or attach the backend and return checked status before robot user code.
   Keep real ntcore details in the binding, not the pure encoder.
2. `configure`: create/update only cameras of the supported `CameraKind`, idempotently.
   Use [`CameraSet`](../../sdk/include/wpiemu/sdk/vision/camera_bridge.h) to keep per-camera state.
3. `poll`: observe robot-written settings and update heartbeats on the extension clock thread.
   Do not perform this work in HAL callbacks.
4. `apply`: publish the producer's **due-only** frames. Do not recapture or advance physics here.
   Foreign camera kinds must be ignored.
5. `reset`: clear stale published results and cached state at connection/epoch boundaries.
6. `stop`: release topics/backend idempotently and without exceptions.

Simulation capture time, scheduled publish time and robot/NT time are not interchangeable.
Follow the [timestamp contract](../reference/bridges.md) when calculating latency or packet times;
do not stamp an old capture as a new measurement. Test producer scheduling separately from topic
encoding: no frame should leave core before `publishTimeUs`.

## Register and build

1. Add implementation files to the explicit `wpiemu_bridges_vision` source list in
   [`bridges/vision/CMakeLists.txt`](../../bridges/vision/CMakeLists.txt).
2. Register a unique bridge factory with `VisionBridgeRegistry::add` in
   [`registration.cpp`](../../bridges/vision/src/registration.cpp).
   Keep null-backend and duplicate-registration failures visible.
3. Preserve the entry-point split: `registerVisionBridges` accepts fake/inert backends;
   `registerNtVisionBridges` supplies the real binding. The
   [aggregator](../../bridges/builtin_bridges.cpp) calls it through
   `registerBuiltinVisionBridges`, not an extension-to-vendor dependency.
4. If a new camera kind is actually required, update
   [`camera.h`](../../sdk/include/wpiemu/sdk/config/camera.h), codec, validator and editor camera
   menu together. A new encoder does not justify compatibility migrations.
5. For a separate bridge module, follow [module wiring](devices.md#wire-a-new-module-only-when-necessary),
   including boundary ownership and explicit test library links.

Without ntcore, the built-ins can register inert bridges. A successful fake registration is
not proof that a real robot received NetworkTables values.

## Verify publication and scheduling

```powershell
ctest --preset windows-ninja-release -R "^(vision |vision:|session: cameras|runner: VisionSetup|codec:.*[Vv]ision|codec:.*camera)"
```

This runs the excerpt, golden encodings, empty/multiple targets, coordinate conversions, latency,
foreign-kind filtering, lifecycle reset and core scheduling cases. Add exact payload assertions
against the target vendor's supported wire format, not just “some topic exists.”
The [fake-backend rig](../../tests/bridges_vision/vision_test_rig.h) keeps this work independent of NT.

For end-to-end checks use the existing
[Java/C++ vision robots](../../samples/vision_robot/README.md) and the opt-in commands in
[Building](../BUILDING.md#opt-in-robot-project-tests), running each language separately.
A skipped test proves nothing about publication; neither does an offscreen UI test.
If poses disagree, check field dimensions/alliance frames, camera attachment and timestamp mapping
before adjusting the encoder. Vendor topic/packet details stay in the bridge reference.
