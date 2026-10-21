# Diagnose and maintain the emulator

Read this when a project fails to launch, devices behave unexpectedly, or a run stops progressing.
Prerequisites: [the user workflow](../USER_GUIDE.md), [Architecture](../ARCHITECTURE.md), and
[Building](../BUILDING.md) for environment setup and test execution.

Start with the first failing boundary, not the last visible symptom. Keep the project/config and
reproduction steps; record whether you are using a real HAL/ntcore build, a fake backend, or demo mode.
Never include credentials or private robot code in a diagnostic report.

## Locate the owner from the symptom

| Symptom | Inspect first | Contract / focused source |
|---|---|---|
| Gradle fails before a connection | JDK/toolchain, wrapper output, extension path and selected project | [Launcher and runtime](../reference/runtime.md), [launcher sources](../../core/runtime/src/launcher) |
| Robot launches but no devices appear | Real HAL binding, SimDevice names, bridge registration/discovery | [Bridges](../reference/bridges.md), [adapter guide](devices.md) |
| Discovered device cannot map | Slot capability and descriptor/alias identity | [Device hub](../../core/runtime/src/runtime/device_hub.cpp) |
| Motor spins but encoder/odometry disagrees | Shaft/output units, gearing, mapping reversal, firmware-owned offsets | [Mechanisms](../reference/mechanisms.md), [motors](motors.md) |
| Camera topics exist but targets stay stale | Due frames, epoch reset, kind/name, capture versus NT time | [Vision guide](vision.md) |
| Config rejected or silently defaulted | Decode, plugin selection, defaults, validation issue path/key | [Runtime config](../reference/runtime.md), [pipeline tests](../../tests/runtime/test_config_pipeline.cpp) |
| Run stalls or repeats old motion | Accepted session/epoch/tick and acknowledgements | [Extension/IPC](../reference/extension-ipc.md), [runner tests](../../tests/runtime/test_session_runner.cpp) |
| Viewport is blank but tests pass | Real Qt platform/graphics setup and Quick 3D scene | [App](../reference/app.md), [visual demos](../BUILDING.md#tests-and-checks) |

## Reproduce one boundary at a time

1. Confirm the robot project builds with its normal WPILib toolchain. Do not “fix” launch by editing
   the user's project to load the emulator differently.
2. Check the configured build actually found HAL. A stub extension is not a partial device
   simulator. Vision additionally needs ntcore; inert registration can still succeed without it.
3. Trace one device: native backend name, announced descriptor/capabilities, mapping, actuator
   sample, plant state, feedback signal. Match units at every transition.
4. Trace one run control: command ID, accepted session/epoch, acknowledgement and tick number.
   A reset is a new simulation epoch, not a robot process restart or an odometry reset.
5. Reduce to a focused fake-based test for translation/orchestration, then use a real backend when
   the issue depends on firmware, geometry, HAL timing or robot execution.

For example, a reversed motor slot must reverse command and rotor feedback exactly once.
Use the existing regression `hub: a reversed motor slot flips its command and its rotor feedback`
in [`test_device_hub.cpp`](../../tests/runtime/test_device_hub.cpp) before changing a bridge sign.
If that passes but a specific vendor fails, narrow the adapter fixture rather than altering global
mapping behavior.

## Preserve lock-step recovery while debugging

The emulator is clock master and allows one outstanding tick. Do not add retries that apply an old
actuator frame to the next epoch, remove reset acknowledgements, or put blocking diagnostics inside
HAL callbacks.

When stalled, compare runner and extension state. The extension's independent watchdog must recover
HAL time even if a bridge/clock operation is stuck; a log lock must not disable that recovery.
Pause is intentional suspension, not a timeout. The exact wire/lifecycle rules stay in
[extension/IPC](../reference/extension-ipc.md).

## Select evidence that matches the claim

Run this concrete diagnosis selection from the repository root:

```powershell
ctest --preset windows-ninja-release -R "^hub: a reversed motor slot|^runner:.*(Reset|watchdog|disconnect)|^session: reset after"
```

Expected result: hub reversal is symmetric, reset rejects late frames, watchdog expiry stalls safely,
reconnect opens a fresh session, and neutral reset matches a fresh simulation tick.
This is an existing test selection, not a substitute for adding your reproducer.

Use shared fakes instead of creating another subtly different protocol harness:

- [SDK testing helpers](../../sdk/include/wpiemu/sdk/testing) provide clocks, physics, hubs, mechanisms
  and protocol peers.
- [`tests/runtime`](../../tests/runtime) owns runtime fixtures and runner helpers;
  [`tests/support`](../../tests/support) owns platform-aware test helpers.
- [Vendor fixtures](../../tests/bridges_vendor/fakes.h) and
  [vision fixtures](../../tests/bridges_vision/vision_test_rig.h) preserve backend semantics.

For asynchronous assertions use the local `waitUntil` helper, not sleep loops. Keep deterministic
simulations on a fixed step and context seed. See [Contributing](../CONTRIBUTING.md#tests).

## Know what a passing check does not prove

- A fake-backend test proves translation against the fixture, not real wrapper discovery.
- A config round-trip proves serialization, not valid physics or correct editor undo.
- A skipped opt-in integration test proves no robot execution. Run Java and C++ vision projects
  separately when changing their path; [Building](../BUILDING.md#opt-in-robot-project-tests) owns
  prerequisites and commands.
- Offscreen Qt checks exercise models/widgets but not Quick 3D rendering. Run a demo interactively
  for placement, lighting, picking or scene conversion changes.
- Windows results do not certify Linux/macOS execution.

After fixing the owning layer, run the smallest regression selection first, then the applicable
boundary/docs/version checks in [Building](../BUILDING.md#tests-and-checks).
Keep a concise report of command, observed result and any unverified prerequisite.
Do not turn a symptom workaround into a new cross-module dependency.

Next: [choose the right extension seam](../extending.md).
