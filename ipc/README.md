# ipc/ - transports, wire codec, handshake

Target `wpiemu_ipc` (static, position independent; depends on `wpiemu_sdk` only). Implements the
interfaces declared in `sdk/ipc/transport.h` for the protocol of `sdk/ipc/messages.h` (version 1).
Rationale for the transport choice: [ADR 0007](../docs/decisions/0007-ipc-transport.md).

| Header (`wpiemu/ipc/`) | What |
|---|---|
| `binary_codec.h` | `BinaryMessageCodec : IMessageCodec`, `receiveSkippingUnknown()` |
| `framing.h`, `framed_transport.h`, `byte_stream.h` | length-prefixed framing, `FramedTransport : ITransport` over any `IByteStream` |
| `local_ipc.h` | `makeLocalServer(endpoint)` / `makeLocalClient()` - named pipe (Windows) / unix socket (Linux, macOS) |
| `in_process.h`, `memory_stream.h` | `InProcessNetwork`: server/client pairs through memory (tests, embedding) |
| `endpoint.h` | `ConnectionEndpoint`, env var names (`WPIEMU_IPC_ENDPOINT`, ...) |
| `handshake.h` | `performServerHandshake` / `performClientHandshake` (Hello / HelloAck, watchdog negotiation) |

## Typical use

```cpp
// emulator (server)
auto endpoint = ipc::ConnectionEndpoint::generate();           // unique per session
auto server   = std::move(ipc::makeLocalServer(endpoint)).value();
server->listen();                                               // BEFORE launching the robot
launcher.setEnvironment(endpoint.environmentEntry());           // WPIEMU_IPC_ENDPOINT=<address>
auto transport = server->accept(30s);                           // again after every disconnect
proto::MessageChannel ch(std::move(transport).value(), std::make_shared<ipc::BinaryMessageCodec>());
auto hs = ipc::performServerHandshake(ch, {.sessionId = nextSessionId++, .runEpoch = ++epoch}, 5s);

// extension (client, inside the robot process)
auto ep     = ipc::ConnectionEndpoint::fromEnvironment();       // WPIEMU_IPC_ENDPOINT
auto client = ipc::makeLocalClient();
auto t      = client->connect(ep.value().address(), ipc::readEnvironmentMillis(ipc::kConnectTimeoutEnvVar, ipc::kDefaultConnectTimeout));
proto::MessageChannel ch(std::move(t).value(), std::make_shared<ipc::BinaryMessageCodec>());
auto ack = ipc::performClientHandshake(ch, hello, 5s);          // then LockstepFollower::begin(ack.sessionId, ack.runEpoch)
```

Receive loops should use `ipc::receiveSkippingUnknown(channel, msg, timeout)` so that a newer peer's
extra message types are skipped instead of ending the session.

## Transport behaviour (all transports, tested identically)

* `ITransport::receive(frame, timeout)` blocks up to `timeout` (`0` = poll) -> `true` frame / `false` timeout /
  `Disconnected`. A timed-out receive never loses bytes. `close()` from any thread wakes a blocked
  `receive()`/`send()`/`accept()`. Frames queued before a peer's `close()` are delivered first; the
  `Disconnected` error is sticky.
* `send()` has a send timeout (`FramedTransportOptions::sendTimeout`, 5 s): a peer that stopped reading
  yields `Timeout` and the connection is closed (a half-written frame would corrupt the stream).
* Frame limit `kDefaultMaxFrameBytes` (8 MiB) on both directions; an oversized header from the peer is a
  `ProtocolError` and ends the connection.
* `ITransportServer::accept()` returns `nullptr` on timeout, `Cancelled` after `close()`. Keep the server
  listening and call `accept()` again after a disconnect: that is the reconnect path. Windows keeps a
  pipe instance pre-created so the endpoint is never unowned between connections.
* `ITransportClient::connect()` retries until the server appears (robot may start before the emulator
  listens) up to the timeout; `Timeout` otherwise.
* Reconnection semantics (ARCHITECTURE 4.1) live in `LockstepWindow`/`LockstepFollower`: after any
  reconnect the emulator picks a new `sessionId`, bumps `runEpoch` and calls `window.begin()`; the
  extension adopts both from `HelloAck`. See `tests/ipc/test_lockstep_integration.cpp` for every scenario
  (stale epoch, reset with a tick outstanding, peer vanishing, robot stall/watchdog, Stop).

## Wire format (`wpiemu-binary-1`, little-endian)

```
frame   := u8 wireFormat(=1) | u16 messageType | body          (frame length comes from the transport)
varint  := LEB128, <= 10 bytes (lengths / counts only)
string  := varint length | bytes (<= 1 MiB)
vector  := varint count | elements      (count validated against the bytes left and a per-type cap)
record  := u16 length | fields          (DeviceDescriptor, SignalSpec, JoystickState: may grow)
sample  := u32 device | u16 signal | f64 value                 (14 bytes)
```

Bodies list the fields of the sdk struct in declaration order (see `src/binary_codec.cpp`); doubles are
IEEE-754 bit patterns, bools one byte, enums one byte.

* **Evolution**: appended fields are an *optional tail*: a decoder defaults tail fields that are absent
  (older peer) and ignores bytes it does not know (newer peer); records are length-prefixed so they can
  grow as well. A `Hello`/`HelloAck`/`SensorFrame`/`ActuatorFrame`/`DsState`/`SimControl`/`DeviceAnnounce`
  without its optional tail decodes with the defaults (tested).
* **Errors**: `ProtocolError` malformed/truncated/out-of-range (closed enums `ClockMode`, `RobotMode`,
  `AllianceStation`, `LogLevel`, `SignalDirection`, `SignalType` are validated; `SimCommand`, `TickResult`,
  `ControlResult`, `WatchdogPolicy`, `RobotLanguage` pass through for the consumer's `default:` path);
  `VersionMismatch` unknown wire-format byte; `Unsupported` well-formed frame of an unknown message type
  (skippable). Decoding never throws, never allocates more than the caps allow, never reads out of bounds
  (fuzz-style tests: every truncation, bit flips, random garbage; ASan/UBSan/TSan clean on Linux).

## Environment variables

| Name | Meaning |
|---|---|
| `WPIEMU_IPC_ENDPOINT` | endpoint address the extension connects to (set by the emulator's launcher) |
| `WPIEMU_IPC_CONNECT_TIMEOUT_MS` | optional: how long the extension keeps trying to connect (default 10000) |
| `WPIEMU_IPC_WATCHDOG_MS` | optional: extension's `Hello::proposedWatchdogMs` |

## Measured round trip (SensorFrame -> ActuatorFrame through the real codec + transport, 1 sample)

Windows 11, MSVC Release, named pipe: median ~12 us, p99 ~40 us. Debug: median ~30-50 us.
Linux (WSL2, g++ -O1), unix socket: median ~55 us, p99 ~120 us. Target was well under 500 us. Reproduce:
run `wpiemu_tests -tc="lockstep: round-trip latency per tick"` (prints a `[ipc latency]` line).
