# HALSim extension (`halsim_wpiemu`)

The extension is the part of the emulator that lives **inside the robot process**. It is loaded by WPILib's own
`HALSIM_EXTENSIONS` mechanism ([ADR 0001](decisions/0001-halsim-extension-injection.md)), connects back to the emulator
over the local IPC transport, announces every simulated device, and (in lock-step mode) is the bridge between the
emulator's simulation clock and the robot's HAL clock. The robot project is never edited.

```
halsim_ext/
  include/wpiemu/halsim_ext/   HAL-free classes (the logic): config, IpcLink, DeviceTable, BridgeSet, SimSession, ExtensionRuntime
  src/extension.cpp            HALSIM_InitExtension + composition root (needs the WPILib HAL)
  src/hal/hal_sim_platform.cpp the real HALSIM_* binding: clock, driver station, battery
  src/extension_stub.cpp       built instead of extension.cpp when the HAL is not available (fails loudly)
  cmake/WpiEmuWpilib.cmake     locates / stages the WPILib HAL + wpiutil
  tests/                       doctest unit tests (fakes), tests/hal (real sim HAL), tests/manual (end-to-end script)
bridges/builtin_bridges.cpp    the bridge aggregation seam (target wpiemu_bridges_builtin)
```

Dependencies (ADR 0004): `sdk`, `ipc`, and the HAL headers. The extension never includes a `bridges/<x>` header: it
calls `sdk::registerBuiltinBridges(DeviceBridgeRegistry&)` (declared in `sdk/devices/builtin_bridges.h`), defined by the
aggregator target `wpiemu_bridges_builtin` that links `bridges/hal` (+ its HALSIM binding when the WPILib HAL was found)
and `bridges/vendor`. Adding a bridge module = link it in `bridges/CMakeLists.txt` + one call in
`bridges/builtin_bridges.cpp`.

## Entry point and environment

`int HALSIM_InitExtension(void)` (exported, `extern "C"`) runs on the robot's main thread inside `HAL_Initialize`.

| Return | Meaning |
|---|---|
| `0` | loaded. Also returned when the extension is **inert** (`WPIEMU_IPC_ENDPOINT` unset or `WPIEMU_DISABLE=1`): the robot runs normally, nothing is paused. |
| `1` | non-fatal failure (no emulator listening, handshake refused/timed out, exception). The HAL unloads the library; HAL timing was not touched. |
| `-1` | only the *stub* build (no WPILib HAL at build time): aborts HAL init with a message. |

Environment (all optional except the endpoint; the emulator's launcher sets them):

| Variable | Default | Meaning |
|---|---|---|
| `WPIEMU_IPC_ENDPOINT` (alias `WPIEMU_ENDPOINT`) | – | Address the emulator listens on (`\\.\pipe\wpiemu-…` / unix socket path), from `ipc::ConnectionEndpoint::environmentEntry()`. |
| `WPIEMU_IPC_CONNECT_TIMEOUT_MS` (alias `WPIEMU_CONNECT_TIMEOUT_MS`) | 10000 | How long `connect()` retries. |
| `WPIEMU_HANDSHAKE_TIMEOUT_MS` | 5000 | Wait for `HelloAck`. |
| `WPIEMU_IPC_WATCHDOG_MS` (alias `WPIEMU_WATCHDOG_MS`) | 0 | `Hello::proposedWatchdogMs` (0 = no preference). |
| `WPIEMU_WATCHDOG` | `on` | `off`: the extension never unblocks the robot by itself (debugging with breakpoints). |
| `WPIEMU_CLOCK` | `master` | `realtime`: never pause HAL time, whatever `HelloAck` says (observer only). |
| `WPIEMU_RECONNECT` | `1` | Re-connect (every second) after the emulator went away, unless the policy is `Terminate`. |
| `WPIEMU_LIVE_DISCOVERY` | `0` | `1`: poll the bridges while the robot constructor still runs (see *Threading*). |
| `WPIEMU_DISABLE_BRIDGES` | – | Comma-separated bridge-name prefixes to leave out (`ctre`, `hal.can`, …). |
| `WPIEMU_LOG_LEVEL` | `info` | `trace|debug|info|warn|error`; log lines go to stderr **and** to the emulator as `Log`. |
| `WPIEMU_PROGRAM_NAME` | exe name | Reported in `Hello::programName`. |
| `WPIEMU_DISABLE` | – | `1`: inert. |

## Session protocol (ARCHITECTURE section 4)

1. **Handshake (synchronous, in `HALSIM_InitExtension`).** Connect, send `Hello` (pid, Java/C++, WPILib version, HAL port
   counts, proposed watchdog, `supportsArmed=true`), wait for `HelloAck` (`accepted`, `sessionId`, `runEpoch`, clock mode, `stepUs`,
   watchdog timeout/policy). A refused/missing ack leaves the robot untouched.
2. **Pause + bridges.** In lock-step (`ClockMode::Lockstep`, not `discoveryOnly`) HAL time is paused **before** any robot code
   exists, then every bridge from `registerBuiltinBridges` is started (their `initialNotify` callbacks run here).
3. **Clock thread** (the only thread that touches bridges and `HALSIM_StepTiming`): waits for
   `HALSIM_GetProgramStarted()` (polled, so the wait is interruptible; equivalent to `HALSIM_WaitForProgramStart`),
   then until a robot notifier is armed (TimedRobot arms it *after* it observed program start; stepping earlier would
   shift the loop phase), polls the bridges **once** (every device the constructor created is announced), reports the
   cameras the robot uses (`VisionReport`, see *Vision*) and sends `Log{source="extension.lifecycle", message="program_started"}` (`sdk/ipc/lifecycle.h`).
   **A `SensorFrame` that arrives earlier is held, not stepped.** The emulator must wait for that Log before it
   starts counting the watchdog / advancing physics.
4. **Per tick** (`SensorFrame` N): apply the latest `DsState` (HALSIM_SetDriverStation*, joysticks, game message,
   `NotifyNewData`), write the battery (`HALSIM_SetRoboRioVInVoltage/Current`), route `SignalSample`s to the owning
   bridge (`applyInputs`), hand the camera frames to the bridges (`applyVision`, see *Vision*),
   `HALSIM_StepTiming(stepUs)` (blocks until the whole robot loop body finished), poll the
   bridges (new devices are announced **before** the ack), `collectOutputs`, drop devices of the `LoopbackSet`,
   `applyLoopback` for those, delta-compress against the previous frame, ask the bridges for the camera state
   (`collectVision`), send `ActuatorFrame{ackTick=N, result}`.
   Exactly one tick is outstanding; stale epochs / foreign sessions are dropped, duplicate ticks answer `ProtocolError`.
   The first frame of an epoch (and every `fullFrameInterval`=400th) is `full`.
5. **Commands.** `Pause`/`Resume` are tracked (the watchdog is disarmed while paused), `Step` is only accepted while paused,
   `SetTimeScale` is accepted in lock-step (pacing is the emulator's), `Reset` adopts the new epoch and forces a full
   frame (and re-engages lock-step after a stall), `Stop` is acked, HAL time is resumed, `Goodbye` is sent and the
   runtime ends (no reconnect). `BeginLockstep` starts an Armed (`discoveryOnly`) session in a new epoch;
   `EndLockstep` returns it to free-running Armed mode without relaunching the robot.
   Identity validation and command retry lookup precede state-dependent checks. Nonzero command IDs
   increase across epochs within a connection; the last **64 distinct results**, including rejections,
   are replayed exactly, without reapplying the command. Older/evicted IDs are rejected, never silently
   reapplied. Retry does not refresh retention order. New connections start fresh history; `Stop` ends
   the connection, so its retry guarantee ends when the transport closes.

The optional `Hello::supportsArmed` boolean is absent => **false**; protocol version and free-form extension
version do not imply support. Production advertises true. The runner refuses a discovery-only handshake without
explicit support and never sends Begin/End to such peers. Normal lock-step remains accepted (minimum wire version 1).

`BridgeSet` calls the defaulted SDK `IDeviceBridge::resetInputHistory()` on the clock thread before
the first feedback of a new connection/reconnect, on applied Reset/Begin, and when a SensorFrame
adopts a newer epoch. Retries do not reset history. Bridges discard integration/angle-unwrapping
history only: discovered device IDs and robot-requested firmware offsets/configuration survive.
This generic seam does not depend on concrete vendor interfaces.

**The emulator MUST forward `Pause` and `Resume` as `SimControl` messages.** Between frames there is no heartbeat: a
silent emulator and a paused emulator are indistinguishable unless the extension was told about the pause.

### Failure handling (the extension unblocks the robot itself)

| Event | Extension behaviour |
|---|---|
| no message for the negotiated watchdog while lock-step is running and not paused | `HALSIM_ResumeTiming()` (policy `ResumeRealtime`) or terminate (`Terminate`); later frames are answered `Stopped`. |
| a step does not return within the watchdog | `ActuatorFrame{RobotStalled}` for that tick is sent from the watchdog thread, then the same unblock. A late-returning step sends nothing. Lock-step resumes after a `Reset`. |
| transport lost / emulator `Goodbye` | same unblock; then reconnects (re-handshake, re-announces all live devices with the same `DeviceId`s, new `sessionId`/epoch, lock-step re-engaged) unless the policy is `Terminate`. |
| `stop()` (HAL shutdown hook) | close the bridge set (no bridge call starts any more), resume timing, `Goodbye`, join (bounded wait; a thread stuck in a step or in a bridge call is detached), then stop the bridges. |

Recovery acts **before diagnostics**, without acquiring the synchronous logger/console mutex or any
bridge lock. Watchdog diagnostics are best-effort remote-only queue entries; they intentionally bypass
stderr, whose pipe may be blocked. Disconnect/Goodbye also recover without synchronous logging.
`Terminate` ends the process with `_Exit(0)` (or Windows `TerminateProcess`), without flushing stdio
or running static destructors: either can block, and a JVM may be running. Bounded `stop()` also
avoids logging before detaching an unresponsive clock thread; its shared state remains alive.

### Threading

* **Robot threads**: only HAL callbacks inside the bridges; they set flags and never block or do I/O.
* **Clock thread**: all bridge calls and `StepTiming`; one per session.
* **IPC link**: a reader and a writer thread own the socket; everything else only enqueues.
* **Watchdog thread** (lock-step only): never touches a bridge.

Bridges are polled only when the robot is quiescent: between steps, and once at program start. They are **not** polled
while the robot constructor runs (`WPIEMU_LIVE_DISCOVERY=0`) because the first end-to-end run showed that polling the
Phoenix 6 bridge concurrently with robot device construction crashes the JVM (access violation in the CTRE native sim).
In free-running modes (`Realtime`, `discoveryOnly`) the robot runs concurrently with the polls by definition; the vendor
bridge has to tolerate that (open item, see the stream report).

`IBridgeHost::programStarted()` is the HAL's program-start flag (`HALSIM_GetProgramStarted`, an atomic read, any thread):
false while the robot constructor runs (with `WPIEMU_LIVE_DISCOVERY=1` the bridges are polled during it), true from the
poll at program start on. A bridge that creates robot-visible state of its own (the vision bridge's NetworkTables
topics) waits for it, so that everything the constructor created is already visible to it.

## Vision

Cameras are not devices: they are keyed by name (`VisionFrame::cameraId` / `ntName` / `vendor`), have no `DeviceKey`
and never go through the device table. The extension only moves camera data between the wire and the bridges through
the defaulted SDK hooks `IBridgeHost::visionSetup()` / `visionSetupRevision()` / `programStarted()` and
`IDeviceBridge::applyVision()` / `collectVision()`; the vendor protocols live in the vision bridge (it plugs in through
`registerBuiltinBridges` like every other bridge). Without that bridge the hooks do nothing.

**Per tick** (`SensorFrame` N), on the clock thread with the robot quiescent:

```
applyInputs -> applyVision -> HALSIM_StepTiming -> poll -> collectOutputs -> collectVision -> ActuatorFrame
```

* **Acceptance.** `SensorFrame::vision` is applied only for a frame the lock-step follower accepted (the same frames
  that are stepped). Dropped frames (stale epoch, foreign session), duplicates (`ProtocolError`), and every frame
  answered without a step (`RobotStalled`, `Stopped`, a second frame while one is held before program start) never
  reach a bridge. A frame held until the program starts keeps its vision and is applied when it is stepped. Free-running
  sessions apply it too (there is just no step).
* **`applyVision(frames)`** gets all frames of the tick in one call, in wire order, and every active bridge sees all of
  them (no routing: each bridge keeps the cameras it owns and ignores the rest). It runs *before* the step, so the HAL
  clock reads T(N-1), the sim time after the previous tick, while it runs: the bridge stamps what it publishes with the
  current NetworkTables time (the HAL clock) and its capture time is that minus `VisionFrame::latencyUs`, which is
  already a whole number of steps. No camera frames, no call: ticks without vision are not timed, counted or allocated for.
* **`collectVision(out, full)`** runs after the step and `collectOutputs`: every active bridge **appends** (never clears)
  what the robot did with its cameras (`CameraRobotState`): every camera when `full`, otherwise only the entries that
  changed since its previous call. The result rides `ActuatorFrame::vision`. It is asked on every completed tick, also
  while no camera is configured, because the vision bridge reports the cameras the robot uses that nobody configured
  (one virtual call per bridge, no allocation, nothing to do by default). `full` is true on the first completed tick of a
  connection, after an applied `Reset` / `BeginLockstep`, when a frame adopts a newer epoch, and on every full signal
  frame (`fullFrameInterval`). The wire format caps the list at 256 states (`ipc::kMaxVisionCameras`): the surplus is
  cut and one warning is logged, so that an oversized list cannot drop the connection.

**Outside the tick path (`VisionReport`).** Where no `ActuatorFrame` answers a tick, the camera state travels in a
`proto::VisionReport` (message type 12, no ack, no identity, not part of the tick window): in an Armed
(`discoveryOnly`) session, before `BeginLockstep` and after `EndLockstep`, in free-running phases, in a stalled or lost
session, and for projects without configured cameras. The extension calls `collectVision(out, full)` once right after the
bridge poll at program start (before the `program_started` log, so the emulator learns the cameras the robot uses at the
moment it learns its devices) and after every later idle poll, and posts ONE report only when `out` is not empty: `full`
on the first call of a connection (a reconnect starts again), deltas afterwards. Nothing is asked or reported before the
program started, not even by the live-discovery polls of the constructor phase, and never from a HAL callback or the
watchdog thread. Lock-step ticks keep using `ActuatorFrame::vision` and send no report. Both paths ask the same bridge,
which tracks what changed since its previous call, so a change is reported once, by whichever path comes first; the
first `ActuatorFrame` after `BeginLockstep`, a `Reset`, a new epoch and a reconnect lists every camera again. In
free-running phases the robot runs concurrently with these calls, which the bridge has to tolerate as it does for
`poll()`.

**Setup and reconnect.** `HelloAck::vision` (empty when absent) is what `IBridgeHost::visionSetup()` returns for that
connection. `visionSetupRevision()` is incremented every time a `HelloAck` is adopted, also for an identical setup: on the
first connection before the bridges start, after a reconnect before the new session makes its first bridge call (the
`resetInputHistory()` of the connection) and therefore before its first `SensorFrame`. A bridge that cached the setup
compares the revision from `poll()` and rebuilds its cameras when it changed (existing topics are reused, cameras
that disappeared go quiet). The reference from `visionSetup()` is valid until the revision changes; the setup is only
replaced between sessions, never while a bridge call runs.

**Rules for a bridge.** Everything above runs on the clock thread: no locking against the robot loop is needed, but
never block, never do socket I/O and never throw (exceptions are not caught around bridge calls, like for every other
hook). HAL callbacks only record events. Keep `applyVision` short: it is not part of the step the watchdog times, so a
call slower than the negotiated watchdog is indistinguishable from a silent emulator and unfreezes the robot. The
watchdog thread never calls a bridge and its `RobotStalled` ack carries no vision. A bridge that failed to start never
sees vision. Create robot-visible state (NetworkTables topics) only once `programStarted()` is true.

**Shutdown.** The HAL shutdown hook closes the bridge set first thing: from then on no `poll`, `applyInputs`,
`applyVision`, `collectOutputs`, `collectVision` or `applyLoopback` starts, even on a clock thread that was stuck inside a
bridge call and is detached when the bounded join times out. The bridges are `stop()`ped after the clock thread
ended. The vision bridge must not touch ntcore once the clock thread has stopped (not even from `stop()`): the
library may be torn down right after the hook returns.

**Stats.** `ExtensionRuntime::stats()` counts `visionFramesApplied` (frames handed to the bridges),
`visionApplyCalls` (ticks with camera frames) and, with the monotonic clock, `visionApplyTotalUs` / `visionApplyMaxUs`
(time spent in the `applyVision` fan-outs, i.e. the publish cost). All zero while no `SensorFrame` carries a camera
frame.

## Building

`cmake/WpiEmuWpilib.cmake` finds the WPILib HAL and wpiutil (headers + desktop shared libs, version
`WPIEMU_WPILIB_VERSION`, default 2026.2.1):

* `-DWPIEMU_WPILIB_ROOT=<dir>`: either a **Maven cache** (`…/wpilib/2026/maven`; also searched by default under
  `%PUBLIC%`, `$HOME`) – it is staged into `<build>/_wpilib/<version>` – or an already **extracted** root
  (`include/hal/HAL.h`, `include/wpi/string.h`, `<platform>/shared/wpiHal.*`). The staged root is re-exported as
  `WPIEMU_WPILIB_ROOT` for `bridges/hal` and `bridges/vendor`.
* `-DWPIEMU_WPILIB_DOWNLOAD=ON`: fetch the four zips from frcmaven (CI on a clean machine).
* Not found: configure still succeeds; `halsim_wpiemu` becomes a stub whose `HALSIM_InitExtension` returns `-1` with a clear
  message; the HAL-free `wpiemu_halsim_core` and its tests still build and run.

The extension links `wpiHal` only (not wpiutil at run time). Windows: `/MD`, 64-bit. Linux/macOS: the extension carries a
normal `DT_NEEDED`/install-name dependency on `libwpiHal` so the loader binds the copy the robot process already loaded
(unverified on Linux/macOS; ADR 0001 lists the `RTLD_LOCAL` risk).

## Tests

| Where | What | Runs |
|---|---|---|
| `halsim_ext/tests/*.cpp` → `wpiemu_halsim_ext_tests` | 100+ doctest cases with fakes: config, device table, delta filter, IPC link, handshake, lock-step protocol, watchdog / stall / disconnect, reconnect, vision dispatch (call order, acceptance, setup revision, stats, shutdown) and the synthetic camera scenario of the fake emulator | ctest `halsim_ext.*`, no WPILib needed |
| `halsim_ext/tests/hal/` → `wpiemu_halsim_ext_hal_tests` | real sim HAL in-process: exact `StepTiming`, 25 loops per 500 ms, DS/battery/joystick, device I/O, camera hooks at the HAL times T(N-1) / T(N), unfreeze on disconnect; the real `halsim_wpiemu` library loaded through `HAL_LoadOneExtension` over a real named pipe with the binary codec and the real built-in bridges | ctest `halsim_ext.hal_integration` (when WPILib was found) |
| `halsim_ext/tests/manual/run_robot_project.ps1` | a real Gradle robot project (`gradlew -I wpiemu-init.gradle simulateJava`) against `wpiemu_halsim_fake_emulator`; `-Vision` adds the synthetic cameras | manual only |

The tests live under `halsim_ext/tests/` (not `tests/`) because `tests/CMakeLists.txt` globs every `*.cpp` into the one
`wpiemu_tests` executable, which cannot link the extension library or WPILib.

Manual run (the project is copied to `%TEMP%`, never modified):
```powershell
cmake --build build\windows-ninja --config Release --target halsim_wpiemu wpiemu_halsim_fake_emulator
.\halsim_ext\tests\manual\run_robot_project.ps1 -Project C:\path\to\robot -BuildDir build\windows-ninja -Config Release
```
Add `-Vision` (or pass `--vision` to `wpiemu_halsim_fake_emulator`) to give the fake emulator two synthetic cameras, a
PhotonVision `photon-front` and a Limelight `limelight-rear`, that see 2026 welded tags from a robot standing at (2, 4):
their setup rides the `HelloAck`, their frames the `SensorFrame`s, and every camera state coming back is printed as a
`CAMERA` line (`halsim_ext/tests/manual/README.md`).

Reference result (FRC-Shelby-2026OS, Java, 28 CAN devices): 1000 ticks of 5 ms = exactly 5.000 s of sim time,
250 robot loops, 29 devices announced, ~1.8 s of wall time.
