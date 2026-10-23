# Extension and IPC

**Read this when:** changing robot-process integration, the message transport, or lock-step
lifecycle behavior.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for ownership and tick order;
[Building](../BUILDING.md) for HAL dependencies and test commands.

The desktop app is the simulation clock master. A HALSim extension in the robot process
connects to the app over local IPC and applies emulator feedback to HAL. This page owns the
wire and lifecycle contract. Follow [Simulation strategies](../guides/simulation-strategies.md)
for transport and message recipes; robot-project setup is in the [User guide](../USER_GUIDE.md).
For background, see [WPILib simulation](../concepts/wpilib-simulation.md) and
[NetworkTables and cameras](../concepts/networktables-and-cameras.md).

## Injection and initialization

The emulator opens its listener before launching robot code. The launcher supplies the
endpoint and HALSim extension path; the Gradle mode injects the extension without editing
the robot project. `HALSIM_InitExtension` connects and completes the `Hello`/`HelloAck`
handshake before lock-step starts.

In lock-step, the extension pauses HAL timing before starting bridges. It waits for the
robot program constructor to finish, allows its notifier scheduler to settle, polls the
devices created by the constructor, then sends the `program_started` lifecycle signal.
The emulator must wait for this signal before sending the first simulation tick or starting
the tick watchdog. A handshake alone does not mean the robot is ready. Constructor-time
bridge polling is off by default; `WPIEMU_LIVE_DISCOVERY` opts in.

The production extension advertises `supportsArmed`. Armed mode lets the robot run in real
time for device discovery; Begin enters lock-step and End returns to free-running Armed mode
without restarting the robot. Peers without Armed support must not receive Begin or End.
`WPIEMU_CLOCK=realtime` opts out of following the emulator clock.

The real extension can run inert when disabled or no endpoint is configured; that is not the
same as the no-HAL stub, which cannot synchronize a robot. A HAL-enabled library is required
for robot-process simulation. Dependency setup belongs to [Building](../BUILDING.md).

## Messages versus stream framing

The SDK separates message encoding from transport framing. A codec encodes one message;
the transport sends and receives complete payloads. The local named-pipe or Unix-socket
transport frames a byte stream as:

```text
stream frame  = u32 little-endian payload length | codec payload
codec payload = u8 wire format (= 1) | u16 little-endian message type | message body
```

The length excludes its own four bytes. Do not add that prefix in a message codec. The
shipped codec is `wpiemu-binary-1`; wire-format and handshake protocol versions are separate.
Integers are little-endian, floating values use IEEE-754 bit patterns, booleans are one byte,
and variable-length counts use bounded LEB128. Device descriptors, signal specifications
and joystick records have `u16` byte-length prefixes. Field order is defined by the codec,
not by C++ struct layout. PhotonVision packets use different layouts even where scalar
helpers are shared.

The default payload limit is 8 MiB and the string limit is 1 MiB; lists also have per-type
limits. A malformed message is a protocol error; an unknown wire format is a version
mismatch; an unknown message type can be skipped. This is not a promise of general
multi-version SDK compatibility.

A receive timeout preserves partial input for the next receive; zero timeout polls. Closing
wakes blocked operations, and complete buffered frames are returned before disconnect.
A failed or timed-out stream write closes the connection: a partial frame cannot safely be
retried. Send timeout is separate from the negotiated robot watchdog. A server accept timeout
is not a disconnect; keep the listener and accept again.

## Handshake and identity

`Hello` reports the robot identity, language, HAL port counts, watchdog preference and Armed
capability. `HelloAck` accepts or refuses the peer and assigns `sessionId`, `runEpoch`, step
size, clock mode and watchdog policy. The emulator assigns a new session id on reconnect and
owns the increasing epoch within a session. Tick numbers can restart, so a tick number alone
does not identify a frame.

Frames, driver-station state and controls carry session and epoch identity. Foreign sessions
and stale epochs are discarded. The extension may adopt a newer sensor-frame epoch; a newer
actuator-frame epoch at the emulator is a protocol error. Normal Reset, Begin and End controls
announce the new epoch before its first frame. Zero identity is tolerated by the shared
classifier as unspecified, but production traffic must carry the negotiated identity to
reject stale messages.

After `HelloAck`, `VisionSetup` is sent before the first sensor frame of each epoch, including
when the camera list is empty. Camera and encoder details belong to [Bridges](bridges.md).

The emulator sends each tick's sensor and control state. The extension applies pending
driver-station state and device/battery inputs, polls and applies vision, advances HAL timing
by the negotiated step, then polls devices and gathers outputs. It announces devices before
sending samples that refer to them. Outputs are delta-filtered, with a full baseline at the
start and after applied Reset or Begin. The returned `ActuatorFrame` acknowledges the tick
and carries the robot's outputs. See [Architecture](../ARCHITECTURE.md) for the complete
simulation tick order.

## Tick readiness and commands

Only one sensor tick may be outstanding. The emulator sends no next tick until the extension
returns an `ActuatorFrame` acknowledging the current tick. An unsolicited, duplicate, or
wrong-tick acknowledgement is a protocol error. Reset opens a new tick window, so a late
answer from the old epoch cannot complete the first tick after reset.

Control commands use increasing, nonzero `commandId`s within a connection, including across
epochs. Reset, Stop, BeginLockstep and EndLockstep require acknowledgements. The extension
retains results for the latest 64 distinct ids: retrying a retained id replays its result
without reapplying the command; an older id outside that window is rejected. Foreign-session
controls are ignored. Stop is honored for its session regardless of epoch; ordinary stale
controls are rejected.

Pause and Resume must be forwarded. Otherwise silence cannot be distinguished from a lost
emulator. Step is accepted only while paused. BeginLockstep starts simulation on the existing
Armed robot. EndLockstep returns it to free-running discovery. Stop acknowledges, resumes HAL
timing, sends Goodbye and ends the connection; reconnect is a new handshake, not a continuation
of the old session.

## Thread and failure boundaries

Bridge polling, feedback application, and HAL stepping run on the extension clock thread.
HAL callbacks on robot threads only record events; they must never block, step HAL time, or do
IPC. Separate reader and writer workers move messages through bounded queues. Start bridges
before the clock thread; stop them after the clock thread joins. The watchdog does not call
bridges.

The negotiated watchdog uses the larger peer preference, defaults to 2 seconds if neither
peer proposes one, and is clamped to 100 ms–60 s. It covers a stuck robot step and loss of
emulator traffic while running. An acknowledged Pause suppresses the idle-traffic timeout,
not a stuck-step timeout. `WPIEMU_WATCHDOG=off` disables timeout recovery for debugging.

On a timeout or disconnect, the extension recovers without an emulator reply: by default it
resumes HAL real-time timing, or it terminates the robot when the negotiated policy is
Terminate. The emulator also times out an outstanding tick and stops sending. A late robot
step must not send a second completion. Recovery must not wait on synchronous console logging;
watchdog diagnostics are best-effort queued messages, not writes to potentially blocked
stderr.

An applied Reset can re-engage lock-step after a stall. Reconnect performs a new handshake,
re-announces live devices with their retained ids, and adopts the new session and epoch.
Reset, Begin, reconnect and adoption of a newer sensor epoch clear bridge input and vision
history without erasing discovery or firmware/user offsets. Shutdown resumes HAL timing even
if the peer is gone. Under Terminate policy the extension does not attempt reconnect.

Behavioral tests live in [IPC](../../tests/ipc/) and
[HALSim extension tests](../../halsim_ext/tests/); [Building](../BUILDING.md) documents how
to run them.

**Next:** [Bridges](bridges.md).
