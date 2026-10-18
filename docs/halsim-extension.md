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

Dependencies (ADR 0004): `sdk`, `ipc`, and the HAL headers (the vision binding also uses ntcore headers, see *Vision cameras*). The extension never includes a `bridges/<x>` header: it
calls `sdk::registerBuiltinBridges(DeviceBridgeRegistry&)` and `sdk::registerBuiltinVisionBridges(VisionBridgeRegistry&)`
(declared in `sdk/devices/builtin_bridges.h`), defined by the
aggregator target `wpiemu_bridges_builtin` that links `bridges/hal` (+ its HALSIM binding when the WPILib HAL was found),
`bridges/vendor` and `bridges/vision`. Adding a bridge module = link it in `bridges/CMakeLists.txt` + one call in
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
   The emulator then sends a **`VisionSetup`** message (the camera list, empty when there are no cameras, the robot project's PhotonLib
   version, the layout name) right after the `HelloAck`, and again before the first frame of every epoch (Reset, Begin, reconnect); it is
   idempotent. It is delivered to the vision bridges (`configure`) on the clock thread, see *Vision cameras*.
2. **Pause + bridges.** In lock-step (`ClockMode::Lockstep`, not `discoveryOnly`) HAL time is paused **before** any robot code
   exists, then every bridge from `registerBuiltinBridges` is started (their `initialNotify` callbacks run here).
3. **Clock thread** (the only thread that touches bridges and `HALSIM_StepTiming`): waits for
   `HALSIM_GetProgramStarted()` (polled, so the wait is interruptible; equivalent to `HALSIM_WaitForProgramStart`),
   then until a robot notifier is armed (TimedRobot arms it *after* it observed program start; stepping earlier would
   shift the loop phase), polls the bridges **once** (every device the constructor created is announced) and sends
   `Log{source="extension.lifecycle", message="program_started"}` (`sdk/ipc/lifecycle.h`).
   **A `SensorFrame` that arrives earlier is held, not stepped.** The emulator must wait for that Log before it
   starts counting the watchdog / advancing physics.
4. **Per tick** (`SensorFrame` N): apply the latest `DsState` (HALSIM_SetDriverStation*, joysticks, game message,
   `NotifyNewData`), write the battery (`HALSIM_SetRoboRioVInVoltage/Current`), route `SignalSample`s to the owning
   bridge (`applyInputs`), publish the due camera frames (`SensorFrame::vision` -> vision bridges' `apply`, see below),
   `HALSIM_StepTiming(stepUs)` (blocks until the whole robot loop body finished), poll the
   bridges (new devices are announced **before** the ack), `collectOutputs`, drop devices of the `LoopbackSet`,
   `applyLoopback` for those, delta-compress against the previous frame, send `ActuatorFrame{ackTick=N, result}`.
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

### Vision cameras (fan-out to `IVisionBridge`)

`BridgeSet` owns, besides the device bridges, the vision bridges created by `registerBuiltinVisionBridges`
([vision.md](vision.md), [ADR 0017](decisions/0017-vision-cameras.md)). Everything runs on the **clock thread** while the robot is
quiescent, in the same place as `applyInputs`; HAL callbacks never reach a vision bridge.

| Trigger | Call on every `IVisionBridge` |
|---|---|
| bridges started (before robot code runs, as for device bridges) | `start(logger)` (a failure is logged and the bridge is skipped); the last `VisionSetup` seen, if any, is applied with `configure` |
| `VisionSetup` received (after `HelloAck`, and before the first frame of each epoch) | `configure(setup)` (idempotent; builds the cameras of the bridge's `CameraKind`) |
| every `SensorFrame` (also with an empty `vision` list), after `applyInputs` and right before `HALSIM_StepTiming` | `poll(simTimeUs)` (mirrors the robot-written pipeline / driver-mode topics, advances heartbeats), then `apply(frame.vision, simTimeUs)` (publishes the frames due) |
| connection / reconnect, applied Reset / Begin, newer epoch adopted | `reset()` (each camera republishes a "no target" state; same triggers as `resetInputHistory`; retries do not reset) |
| shutdown | `stop()` (unpublishes the bridge's topics) |

**ntcore dependency.** The real backend has no link-time dependency on ntcore: it attaches to the `ntcore` module the robot process
has already loaded (`wpiemu_bridges_vision_nt`; `INtBackend::available()` is false until then, so the bridges quietly wait) and
publishes into the robot's **default NT instance** with explicit timestamps from `HAL_GetFPGATime()` (which equals `nt::Now()` in
the simulated robot), so results line up with `Timer.getFPGATimestamp()`. The ntcore headers are needed at build time
(`wpiemu_wpilib_ntcore`, found by `cmake/WpiEmuWpilib.cmake` from the same Maven cache as the HAL, `WPIEMU_NTCORE_FOUND`); without
them the vision bridges are registered with a backend that never becomes available and cameras stay inert.
Details, the Limelight / PhotonVision topics and the reset semantics: [vision.md](vision.md).

### Failure handling (the extension unblocks the robot itself)

| Event | Extension behaviour |
|---|---|
| no message for the negotiated watchdog while lock-step is running and not paused | `HALSIM_ResumeTiming()` (policy `ResumeRealtime`) or terminate (`Terminate`); later frames are answered `Stopped`. |
| a step does not return within the watchdog | `ActuatorFrame{RobotStalled}` for that tick is sent from the watchdog thread, then the same unblock. A late-returning step sends nothing. Lock-step resumes after a `Reset`. |
| transport lost / emulator `Goodbye` | same unblock; then reconnects (re-handshake, re-announces all live devices with the same `DeviceId`s, new `sessionId`/epoch, lock-step re-engaged) unless the policy is `Terminate`. |
| `stop()` (HAL shutdown hook) | resume timing, `Goodbye`, join (bounded wait; a thread stuck in a step is detached). |

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

## Building

`cmake/WpiEmuWpilib.cmake` finds the WPILib HAL, wpiutil and **ntcore** (headers + desktop shared libs, version
`WPIEMU_WPILIB_VERSION`, default 2026.2.1):

* `-DWPIEMU_WPILIB_ROOT=<dir>`: either a **Maven cache** (`…/wpilib/2026/maven`; also searched by default under
  `%PUBLIC%`, `$HOME`) – it is staged into `<build>/_wpilib/<version>` – or an already **extracted** root
  (`include/hal/HAL.h`, `include/wpi/string.h`, `<platform>/shared/wpiHal.*`). The staged root is re-exported as
  `WPIEMU_WPILIB_ROOT` for `bridges/hal` and `bridges/vendor`.
* `-DWPIEMU_WPILIB_DOWNLOAD=ON`: fetch the zips (HAL, wpiutil, ntcore: headers + desktop libraries) from frcmaven (CI on a clean machine).
* Not found: configure still succeeds; `halsim_wpiemu` becomes a stub whose `HALSIM_InitExtension` returns `-1` with a clear
  message; the HAL-free `wpiemu_halsim_core` and its tests still build and run.

The extension links `wpiHal` only (not wpiutil at run time) and does **not** link ntcore either: the vision binding only needs its headers
at build time (imported target `wpiemu_wpilib_ntcore`, `WPIEMU_NTCORE_FOUND`) and attaches to the ntcore the robot loaded; when the
artifacts are missing the vision bridges are inert, nothing else changes. Windows: `/MD`, 64-bit. Linux/macOS: the extension carries a
normal `DT_NEEDED`/install-name dependency on `libwpiHal` so the loader binds the copy the robot process already loaded
(unverified on Linux/macOS; ADR 0001 lists the `RTLD_LOCAL` risk).

## Tests

| Where | What | Runs |
|---|---|---|
| `halsim_ext/tests/*.cpp` → `wpiemu_halsim_ext_tests` | 60+ doctest cases with fakes: config, device table, delta filter, IPC link, handshake, lock-step protocol, watchdog / stall / disconnect, reconnect, vision fan-out (`test_vision_fanout.cpp`) | ctest `halsim_ext.*`, no WPILib needed |
| `halsim_ext/tests/hal/` → `wpiemu_halsim_ext_hal_tests` | real sim HAL in-process: exact `StepTiming`, 25 loops per 500 ms, DS/battery/joystick, device I/O, unfreeze on disconnect; the real `halsim_wpiemu` library loaded through `HAL_LoadOneExtension` over a real named pipe with the binary codec and the real built-in bridges | ctest `halsim_ext.hal_integration` (when WPILib was found) |
| `halsim_ext/tests/manual/run_robot_project.ps1` | a real Gradle robot project (`gradlew -I wpiemu-init.gradle simulateJava`) against `wpiemu_halsim_fake_emulator` | manual only |

The tests live under `halsim_ext/tests/` (not `tests/`) because `tests/CMakeLists.txt` globs every `*.cpp` into the one
`wpiemu_tests` executable, which cannot link the extension library or WPILib.

Manual run (the project is copied to `%TEMP%`, never modified):
```powershell
cmake --build build\windows-ninja --config Release --target halsim_wpiemu wpiemu_halsim_fake_emulator
.\halsim_ext\tests\manual\run_robot_project.ps1 -Project C:\path\to\robot -BuildDir build\windows-ninja -Config Release
```
Reference result (FRC-Shelby-2026OS, Java, 28 CAN devices): 1000 ticks of 5 ms = exactly 5.000 s of sim time,
250 robot loops, 29 devices announced, ~1.8 s of wall time.
