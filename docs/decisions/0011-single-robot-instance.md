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
* SDK 1.3 introduced these commands. Capability negotiation is made explicit by the SDK 1.4 amendment below.
* Unmapped devices stay loopback; there is no auto-binding.
* The watchdog runs whenever the session is lock-step capable, so a vanished host still unfreezes HAL time.

## Driver Station
Modes are **Auto / Teleop / Practice / Test**. Practice is DS-only (never a wire value): Enable runs auto, a disabled gap,
then teleop on the match clock; Disable or E-stop ends it. The old "sequence" option and *Start match* button are gone.

## SDK 1.4 / protocol v3 amendment
`Hello::supportsArmed` is appended after `proposedWatchdogMs`, defaulting to false. The extension
explicitly advertises it; a protocol-v2 peer is not assumed to support the SDK-1.3 commands.
The runner sends Begin/End only when the bit is true and rejects an Armed/discovery-only
handshake without it. Normal legacy lock-step remains supported. `kProtocolVersion = 3`,
`kMinCompatibleProtocolVersion = 1`, and `kSdkVersionMinor = 4`.

`IDeviceBridge::resetInputHistory()` is an appended non-pure virtual with a default no-op.
The clock-thread bridge fan-out calls it between steps on connection/reconnect, Reset,
BeginLockstep, and adoption of a newer sensor epoch. It resets transform integration and
unwrap history, not firmware/user offsets or discovered-device identity. This is additive
source evolution; statically built implementations use the same toolchain as the host.
