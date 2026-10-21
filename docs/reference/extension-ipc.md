# Extension and IPC

Read this when changing robot-process integration, a transport, or lock-step lifecycle behavior.
Prerequisites: [Architecture](../ARCHITECTURE.md) for process ownership and the global tick pipeline,
and [Building](../BUILDING.md) for HAL dependencies and test commands. Extension recipes belong in
[Extending](../extending.md); robot-project setup belongs in the [User guide](../USER_GUIDE.md).

## Injection and initialization

The emulator listens before launching the robot; the extension connects back as the client.
[RobotSession](../../app/host/robot_session.cpp) supplies the endpoint and extension path to the
[launcher](../../core/runtime/src/launcher/robot_launcher.cpp). Its Gradle mode runs the project's
simulation task with a generated init script, without editing robot sources or build scripts.
The [script](../../core/runtime/src/launcher/init_script.cpp) disables GradleRIO's default HAL
extensions and sets `HALSIM_EXTENSIONS` to `WPIEMU_EXT`, the emulator extension's library path.
The alternate direct-launch mode obtains Gradle's simulation descriptor before launching its target.

WPILib calls the exported `HALSIM_InitExtension` during HAL initialization. The
[entry point](../../halsim_ext/src/extension.cpp) composes SDK registries, IPC, and the built-in bridge
aggregator, not core or individual bridge modules. See Architecture for the dependency rule.
Connection and handshake complete before lock-step pauses HAL timing and starts bridges.

| Entry-point result | Meaning |
|---|---|
| `0` | Loaded; also returned when inert because the endpoint is absent or `WPIEMU_DISABLE=1`. |
| `1` | Attachment or initialization failed; the failure is reported to stderr. |
| `-1` | The no-HAL stub reports that this library was built without WPILib HAL support. |

The launcher passes `WPIEMU_IPC_ENDPOINT`. The names the emulator and the extension share live in the
[SDK environment header](../../sdk/include/wpiemu/sdk/core/environment.h); the
[configuration contract](../../halsim_ext/include/wpiemu/halsim_ext/config.h) owns the extension-only names,
aliases, defaults, and debugging switches; [parsing](../../halsim_ext/src/config.cpp) warns and keeps
defaults for malformed settings. `WPIEMU_CLOCK=realtime` prevents HAL pausing even if the peer asks
for lock-step. An inert real extension is different from the
[no-HAL stub](../../halsim_ext/src/extension_stub.cpp): the latter cannot synchronize a robot.
Dependency discovery and rebuilding a HAL-enabled library are covered in Building.

## Messages versus stream framing

The [SDK transport contract](../../sdk/include/wpiemu/sdk/ipc/transport.h) separates two layers:

- `IMessageCodec` encodes one message into one payload and decodes that payload.
- `ITransport` sends and receives whole payloads; it owns message boundaries, not the codec.
- The shipped local transport uses Windows named pipes or Unix-domain sockets through
  [local IPC](../../ipc/include/wpiemu/ipc/local_ipc.h). Its
  [FramedTransport](../../ipc/src/framed_transport.cpp) adapts a byte stream with this layout:

```text
stream frame  = u32 little-endian payload length | codec payload
codec payload = u8 wireFormat (= 1) | u16 little-endian messageType | body
```

The length excludes its own four bytes. Do not put that prefix into an `IMessageCodec`.
The shipped [binary codec](../../ipc/src/binary_codec.cpp) is `wpiemu-binary-1`; wire format and
handshake protocol are separate values, both currently `1`. Integers are little-endian, floating
values use IEEE-754 bit patterns, and booleans occupy one byte. Lengths/counts use bounded LEB128;
device descriptors, signal specifications, and joystick records have a `u16` byte-length prefix.
Signal samples encode `(u32 device, u16 signal index, f64 value)`, not C++ object memory.
One `fields` function per struct in the codec drives both encoding and decoding, so it, rather than
struct layout, owns field order and optional-tail handling. The scalar encodings are shared with the
PhotonVision packets, whose layout is a different protocol ([byte_order.h](../../sdk/include/wpiemu/sdk/core/byte_order.h)).

Default payload limit is 8 MiB, string limit 1 MiB; counts are also checked against remaining bytes
and per-type caps, and the encoder refuses a list above its cap rather than send a frame the peer rejects. A malformed payload returns `ProtocolError`, another wire-format byte returns
`VersionMismatch`, and an unknown message type returns `Unsupported`. Only the last is the
unknown-type skip case handled by `receiveSkippingUnknown` and the extension's receive loop.
These decoder behaviors do not establish a multi-version SDK compatibility policy.

A receive timeout preserves partial input for the next receive; zero timeout polls.
Closing wakes blocked operations, and complete buffered frames precede a disconnect result.
A failed stream write, including a timeout, closes the connection: a partial frame is unsafe to retry.
The [transport options](../../ipc/include/wpiemu/ipc/framed_transport.h) set a separate send timeout;
it is not the negotiated robot watchdog. The SDK permits one sender and one receiver concurrently,
with `close()` callable from another thread. Server accept timeouts are not disconnects: retain the
listener and accept again for reconnect.

## Handshake and identity

The [session messages](../../sdk/include/wpiemu/sdk/ipc/session_messages.h) define `Hello` and `HelloAck`.
The extension reports robot identity, language, HAL port counts, watchdog preference, and
`supportsArmed`. The emulator accepts/refuses it and assigns `sessionId`, `runEpoch`, `stepUs`,
clock mode, watchdog policy/timeout, and whether this is a discovery-only Armed session.
The production extension advertises Armed support explicitly; neither protocol number nor a
free-form extension version implies it. The
[runner](../../core/runtime/src/runtime/session_runner.cpp) refuses an Armed handshake without that
capability and never sends Begin/End to peers that omit it.

Identity rules are shared by the SDK's
[lock-step state machines](../../sdk/include/wpiemu/sdk/ipc/lockstep.h):

- `sessionId` identifies an accepted connection; reconnect assigns a new one.
- The emulator owns monotonically increasing `runEpoch`; Reset and reconnect begin a new epoch.
  Tick numbers may restart, so a tick alone is not an identity.
- Sensor/actuator frames, driver-station state, and controls carry identity. Stale epochs and foreign
  sessions are discarded. A newer sensor epoch is adopted by the extension; a newer actuator epoch
  is a protocol error at the emulator.
- Zero identity fields mean unspecified and are accepted by `classifyFrame`; they do not provide
  stale-frame isolation. Production traffic must carry the negotiated identity.

`VisionSetup`, including an empty camera list, follows `HelloAck` and precedes the first sensor
frame of each epoch. It carries the cameras, the photonlib version, the layout name and the field size of the
game's AprilTag layout (two trailing doubles; absent means unknown). Its camera indices and encoder-specific
contract are in [Bridges](bridges.md).

## Tick readiness and commands

There is exactly **one outstanding sensor tick**. A completed `ActuatorFrame` must acknowledge that
tick before the next is sent. Wrong/unsolicited acknowledgements and duplicate/out-of-order sensor
ticks are protocol errors; the latter are not stepped. Reset opens a new window, so a late answer
from an abandoned epoch cannot satisfy its first tick.

The [extension session](../../halsim_ext/src/sim_session.cpp) waits for HAL program start, then gives
the notifier scheduler a bounded settling interval. It polls constructor-created devices before
sending `Log{source="extension.lifecycle", message="program_started"}`.
An early sensor frame is held, not stepped. The production runner waits for that readiness signal
before advancing physics or starting a tick deadline; a completed handshake is not program readiness.
Constructor-time bridge polling is off by default; `WPIEMU_LIVE_DISCOVERY` is an explicit opt-in.

At the extension boundary, a tick applies pending driver-station state, battery and device inputs,
polls/applies vision, steps HAL by `stepUs`, then polls discovery and collects outputs.
Announcements precede samples that reference them. A `DeviceAnnounce` is the SDK's
[DeviceRecord](../../sdk/include/wpiemu/sdk/devices/device_record.h) (id, descriptor, signal specifications,
observed bus) plus a `retired` flag; the hub's `DiscoveredDevice` extends the same record with mapping state. Unmapped devices in `LoopbackSet` are handled
locally and omitted from actuator samples; loopback may be a no-op for a device with nothing to echo.
Outputs are delta-filtered, with a full baseline initially and after applied Reset/Begin.

Control senders use increasing nonzero `commandId`s across epochs within a connection; Reset, Stop,
BeginLockstep, and EndLockstep require acknowledgements. The follower retains the last 64 distinct
results, including rejections. Retrying an ID replays its result without reapplying the command or
refreshing retention; evicted/older IDs are rejected. Foreign-session controls are dropped first.
Stop is honored regardless of epoch within its session; stale ordinary controls are rejected.
Reset/Begin/End announce a newer epoch before its first frame.

Pause and Resume **must be forwarded** as controls: silence is otherwise indistinguishable from a
lost emulator. Step is accepted only while paused. Begin switches Armed to lock-step; End resumes
free-running Armed discovery without relaunching the robot. Stop acknowledges, resumes HAL timing,
sends Goodbye, and ends the connection rather than reconnecting.

## Thread and failure boundaries

Bridge tick operations and HAL stepping belong to the clock thread. HAL callbacks on robot threads
only record events; they must not step time or perform IPC I/O. The
[IPC link](../../halsim_ext/src/ipc_link.cpp) has separate reader/writer threads and message queues.
Bridge startup happens during initialization, before the clock thread starts; normal shutdown stops
bridges after joining it. Free-running modes permit concurrent robot execution, unlike lock-step's
between-step quiescence. The watchdog never calls a bridge.

Watchdog negotiation takes the larger preference, defaults to 2 s when neither specifies one, and
clamps to 100 ms through 60 s. The timeout covers an outstanding robot step and loss of emulator
traffic while running. Announced Pause suppresses the idle timeout, not a stuck-step timeout.
`WPIEMU_WATCHDOG=off` removes timeout protection for debugging.

On timeout or detected disconnect, the extension independently resumes HAL timing
(`ResumeRealtime`, default) or terminates the robot (`Terminate`); it never needs an emulator reply.
A stuck-step timeout queues `RobotStalled`; a late-returning step must not send another completion.
Recovery runs before diagnostics and avoids synchronous logging/console locks. Watchdog diagnostics
are best-effort queued remote logs, not writes to possibly blocked stderr.
The emulator also times out its window and stops sending rather than waiting indefinitely.

Applied Reset can re-engage lock-step after a stall. Reconnect performs another handshake,
re-announces live devices with their retained IDs, and adopts the new session/epoch. Connection,
Reset, Begin, and adoption of a newer sensor epoch clear bridge input history and vision state while
preserving discovery and firmware/user offsets; command retries do not clear them again.
The [runtime](../../halsim_ext/src/extension_runtime.cpp) retries disconnected sessions when enabled,
except under Terminate policy. Shutdown always resumes timing; its bounded join may detach a stuck
clock thread while retaining shared state. A Terminate policy uses non-flushing process termination,
not destructors that could block.

Behavioral evidence includes [transport tests](../../tests/ipc/) and
[extension lifecycle tests](../../halsim_ext/tests/); Building owns how to run the available suites.

Next: [Bridges](bridges.md).
