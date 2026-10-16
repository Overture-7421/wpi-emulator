# ADR 0007 - IPC transport: named pipe / unix domain socket, hand-rolled binary codec

Status: **Accepted** (implemented in `ipc/`, tested on Windows/MSVC and Linux/g++; macOS shares the POSIX code).

## Context
The emulator (Qt app) and the HALSim extension (inside the robot process) exchange one tick-sized request/response
per 5 ms step (ADR 0001, ARCHITECTURE section 4). Requirements: sub-0.5 ms round trip, never reachable from other users or
the network, no ports/firewall prompts, deterministic framing, robust against a crashing or hostile peer, no heavy
third-party dependency inside the robot process (the extension is a shared library loaded into a JVM or a native robot).

## Decision
* **Transport**: Windows **named pipe** (overlapped I/O, byte mode), Linux/macOS **unix domain stream socket**.
  TCP loopback was rejected: it needs a port (collisions, firewall prompts on Windows, other local users can connect
  to it without an ACL), adds nothing the pipe/socket lacks, and is slower to wake. Shared memory was rejected as
  unnecessary at 200 Hz (measured ~12 us per round trip on a pipe).
* **Security**: random per-session endpoint (`\\.\pipe\wpiemu-<16 hex>` / `<runtime-dir>/wpiemu-<16 hex>/ipc.sock`);
  pipe: first instance created with `FILE_FLAG_FIRST_PIPE_INSTANCE` (no squatting), `PIPE_REJECT_REMOTE_CLIENTS`,
  DACL = current user only, client opens with `SECURITY_IDENTIFICATION`; socket: private `0700` directory, socket
  `0600`, `FD_CLOEXEC` everywhere (the robot is a child process and must not inherit our handles).
* **Endpoint hand-off**: the emulator generates the endpoint (`ConnectionEndpoint::generate()`), listens, then launches
  the robot with `WPIEMU_IPC_ENDPOINT=<address>`. (`sdk/ipc/transport.h` mentions `WPIEMU_ENDPOINT` in a comment; the
  constant `ipc::kEndpointEnvVar` = `WPIEMU_IPC_ENDPOINT` is authoritative.)
* **Layering**: `IMessageCodec` (`BinaryMessageCodec`) -> frame -> `FramedTransport` (u32-LE length prefix, 8 MiB cap)
  -> `IByteStream` (pipe / socket / memory). Everything above the byte stream is platform independent and tested with
  chunked in-memory streams; the in-process network reuses it.
* **Codec**: hand-rolled compact little-endian binary (no flatbuffers/protobuf dependency in the robot process); frame =
  wire-format byte + message type + fields. Appended fields are an optional tail, nested records are length-prefixed,
  unknown message types decode to `ErrorCode::Unsupported` and are skipped by `receiveSkippingUnknown()`; decoding is
  bounds-checked with allocation caps. Layout documented in `ipc/README.md`.
* **Lifecycle**: blocking `receive(timeout)`, `close()` from any thread wakes blocked calls, queued data is delivered
  before a disconnect is reported, send timeout closes the connection. Reconnects use the same server (`accept()` again,
  new `sessionId`, bumped `runEpoch`); Windows keeps a spare pipe instance so the name is never unowned.

## Consequences
* Zero third-party runtime dependencies in `wpiemu_ipc`; only `advapi32` on Windows.
* A Windows pipe name exists while any instance (including an accepted connection) is alive, so re-listening on the same
  endpoint requires the old connection to be gone; the emulator generates a fresh endpoint per session anyway.
* The wire format is ours to maintain; the compatibility rules are enforced by tests (v1 frames, newer-peer trailing
  bytes, grown records, unknown types).
