# Diagnose a run

Read this when a robot project fails to launch, a device behaves unexpectedly, or a run stops
progressing. Prerequisites: [the user guide](../USER_GUIDE.md), [Architecture](../ARCHITECTURE.md),
and [Building](../BUILDING.md) for environment and test setup.

This is a hands-on diagnosis sequence. Change one thing at a time, record what you observed, and
follow the first boundary that fails. Use the [switches reference](../reference/switches.md) for
the exact command-line switches and diagnostic environment variables; do not copy undocumented
settings from old reports. Never include credentials or private robot code in a report.

## Before reproducing

Record the emulator version, operating system, robot language, WPILib season, launch mode, whether
the build found real HAL and ntcore, and the exact first error. Confirm that the robot project
builds with its normal WPILib toolchain. Do not edit its source or build files to make the emulator
load.

For a controlled test:

1. Open the intended project and confirm its project folder and saved setup.
2. Start discovery first. Check whether the extension attaches and announces devices.
3. Start a run only after discovery is understood. Observe the first error or missing transition;
   later symptoms often follow from that boundary.
4. Repeat with a minimal robot project or the closest focused fake-based test. Use a real-HAL
   project when the failure depends on WPILib, vendor firmware, HAL timing, or robot execution.

## Follow the failing boundary

| What you observe | Check first | Next source of truth |
|---|---|---|
| Gradle or robot process fails before connecting | Selected project, wrapper, chosen JDK, extension path, launch output | [Runtime and launcher](../reference/runtime.md#launcher-boundary); [launch diagnosis](../reference/app.md) |
| Process starts, but no extension connects | Real-HAL build, extension path and endpoint, extension startup log | [Extension and IPC](../reference/extension-ipc.md#injection-and-initialization) |
| Extension connects, but devices are absent | HAL binding, SimDevice names, built-in bridge registration and discovery | [Bridges](../reference/bridges.md); [device guide](devices.md) |
| Device is discovered but cannot be mapped | Slot capabilities, descriptor identity and mapping owner | [Bridges](../reference/bridges.md); [device hub](../../core/runtime/src/runtime/device_hub.cpp) |
| Motor moves incorrectly or encoder/odometry disagrees | Command and feedback units, gearing, mapping reversal, firmware-owned offsets | [Mechanisms](../reference/mechanisms.md); [mechanism guide](mechanisms.md) |
| Camera topics exist but targets are stale | Capture schedule, epoch reset, result kind/name and NT timestamp | [Vision](vision.md); [vision bridge contract](../reference/bridges.md) |
| Setup is rejected or does not round-trip | Decode error path, plugin selection, defaults, validation issue key | [Runtime configuration](../reference/runtime.md); [pipeline tests](../../tests/runtime/test_config_pipeline.cpp) |
| Run stalls, repeats old motion, or reset is ignored | Accepted session/epoch/tick, command acknowledgement and watchdog state | [Extension and IPC](../reference/extension-ipc.md); [runner tests](../../tests/runtime/test_session_runner.cpp) |
| Run is healthy but the viewport is blank | Qt platform/graphics setup and Quick 3D scene rendering | [Application](../reference/app.md); [visual demos](../BUILDING.md#what-a-passing-test-proves) |

Use switches only to isolate a boundary. For example, compare the documented Gradle and direct
launch modes if process setup is suspect. Temporarily disabling watchdog protection can help attach
a debugger, but removes independent robot recovery; restore it immediately after the experiment.
Pause is an intentional suspension, not a watchdog failure. Exact names, defaults and safety
effects are in the [switches reference](../reference/switches.md).

## Trace one device or one tick

For a device issue, follow one signal end to end:

1. Robot-facing HAL/vendor signal and its units.
2. Extension announcement and descriptor capabilities.
3. Emulator mapping and any one-time reversal or normalization.
4. Mechanism command, plant update, and returned feedback.

For a run-control issue, compare command ID, accepted `sessionId` and `runEpoch`, tick number,
acknowledgement, and watchdog state. The emulator permits one outstanding tick. Reset opens a new
simulation epoch; it does not restart the robot process or reset robot-owned odometry.

For example, a reversed motor slot must reverse its command and rotor feedback exactly once. Start
with the existing regression `hub: a reversed motor slot flips its command and its rotor feedback`
in [`test_device_hub.cpp`](../../tests/runtime/test_device_hub.cpp) before changing a bridge sign.
If it passes but a vendor device fails, narrow the adapter fixture rather than altering global
mapping behavior.

Preserve watchdog recovery while debugging. The extension must resume HAL time or terminate by
policy even when an emulator or bridge is stuck; HAL callbacks must not block or perform socket I/O.
The [extension/IPC reference](../reference/extension-ipc.md) owns the lifecycle and recovery rules.

## Choose evidence that matches the claim

- A fake-backend test proves translation against its fixture, not real wrapper discovery.
- A config round-trip proves serialization, not valid physics or editor undo behavior.
- A skipped opt-in integration test did not execute robot code. Run Java and C++ vision projects
  separately when changing their paths; [Building](../BUILDING.md#opt-in-robot-project-tests)
  owns prerequisites and commands.
- Offscreen Qt tests do not prove Quick 3D rendering. Run a visual demo interactively for placement,
  lighting, picking or scene-conversion changes.
- Windows results do not establish Linux/macOS behavior.

Use shared [SDK fakes](../../sdk/include/wpiemu/sdk/testing), [runtime fixtures](../../tests/runtime),
[vendor fixtures](../../tests/bridges_vendor/fakes.h) and
[vision fixtures](../../tests/bridges_vision/vision_test_rig.h). Use the local `waitUntil` helper
for asynchronous assertions, not sleep loops. Keep deterministic simulation tests on a fixed step
and context seed.

After fixing the owning layer, run its smallest regression first, then the applicable
[boundary, documentation and version checks](../BUILDING.md#run-tests-and-repository-checks). Report the command,
observed result, and any prerequisite you could not verify. Do not turn a symptom workaround into a
new cross-module dependency.

Next: [choose the right extension seam](../extending.md).
