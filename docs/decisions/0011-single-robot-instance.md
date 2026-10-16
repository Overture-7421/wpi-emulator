# ADR 0011: Single long-lived robot instance (SDK 1.3, additive)

Amends [0001](0001-halsim-extension-injection.md) and [0006](0006-sdk-1-1-amendments.md).

## Context
Device discovery and the simulation each launched their own Gradle/robot process, and `discoveryOnly` was fixed at the
handshake. Users had to launch twice, and a stopped simulation killed the robot.

## Decision
* The app launches the robot once, when the Editor/Run tab is entered. The session starts **Armed**: connected,
  announces and loopback handled, no ticks, HAL time free-running. Devices are detected from this same instance.
* `SimCommand::BeginLockstep = 6` (acked via `SimControlAck`) switches an Armed session to lock-step at sim t = 0
  (new epoch, HAL time paused by the extension). `SimCommand::EndLockstep = 7` returns to Armed. The robot process is
  untouched; **Restart** relaunches it.
* `kSdkVersionMinor = 3`. Older extensions never receive the new commands (the runner only sends them to capable peers).
* Unmapped devices stay loopback; there is no auto-binding.
* The watchdog runs whenever the session is lock-step capable, so a vanished host still unfreezes HAL time.

## Driver Station
Modes are **Auto / Teleop / Practice / Test**. Practice is DS-only (never a wire value): Enable runs auto, a disabled gap,
then teleop on the match clock; Disable or E-stop ends it. The old "sequence" option and *Start match* button are gone.
