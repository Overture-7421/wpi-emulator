# Match network

**Read this when:** changing the practice-match protocol, its encryption, the relay or a game
client, or when deciding what a hosted match does and does not protect.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for ownership;
[Runtime](runtime.md#remote-robots-practice-matches) for how a remote robot is simulated;
[Application](app.md#hosting-a-practice-match) for the host UI. The reasons for the design are in
the [decision index](../decisions/README.md); the Azure side is in the
[relay guide](../guides/relay-azure.md).

A practice match lets other people drive their own robots on the host's field. The host desktop
app runs the match as a game server. The message structs, limits and enums are plain SDK
headers in [`sdk/include/wpiemu/sdk/match`](../../sdk/include/wpiemu/sdk/match); this page explains how they fit
together and what each limit is for. When a number below disagrees with a header, the header wins.

## Who owns what

```text
Host app (desktop)                                          Game client (thin)
robot code -> HALSim extension -> SessionRunner             gamepad -> wheel setpoints + zone flags
SimulationSession: host robot + remote robots (one world)   prediction world, interpolation, rendering
MatchHost (core/match) <-- SecureTransport --> ITransport <-- SecureTransport --> client session
   |  direct: ENet server          or          relay: RelayHostTransport  -->  wpiemu_relay  <--  RelayClientTransport
```

| Party | Owns | Does not own |
|---|---|---|
| Host app (simulation thread) | The one physics world, the host's real WPILib robot, every remote robot's chassis and controller, game state, sim clock, tick order, roster and epoch | |
| Client | Its own input, a disposable prediction world that authoritative state overwrites, interpolation buffers, rendering | Other robots, game pieces, scores, match time |
| Relay | The room table: room id, host authentication value, peer list, limits | Any simulation or readable game data |

The host is authoritative and trusted. The world advances only while the host's robot runs in
lock-step, so a client sees `MatchState::Lobby` until the host starts, `Paused` while the host
pauses and `HostStalled` when the robot watchdog fires. While clients are connected the host app
holds the time scale at 1x and disables Step (`HostConstraints`).

`MatchHost` ([`core/match`](../../core/match/include/wpiemu/core/match/match_host.h)) is Qt-free and
transport-agnostic. Connection reader threads decode frames into its inbox. The simulation thread
calls `onPoll` and `onTickEnd` (the runner's tick observer) and does everything else: joins,
inputs, snapshots. Sending is a bounded per-client `PeerOutbox` with its own thread: a slow client
loses its oldest snapshots (4 queued), a client whose reliable queue overflows (32) or whose send
blocks for 3 s is dropped, and the simulation never waits for either.

## Layers

| Layer | Where | What it adds |
|---|---|---|
| Message codec | [`ipc/`](../../ipc/src/binary_codec.cpp), `wpiemu-binary-1` | The same `BinaryCodec` that encodes the HALSim messages; match and relay messages are further alternatives of `proto::Message` (values 13 to 29) |
| Delivery class | [`delivery.h`](../../sdk/include/wpiemu/sdk/ipc/delivery.h) | `Reliable` or `Unreliable` per message type (`deliveryOf`) |
| Encryption | [`secure_transport.h`](../../net/include/wpiemu/net/secure_transport.h) | End-to-end host to client records (below) |
| Transport | [`enet_transport.h`](../../net/include/wpiemu/net/enet_transport.h), [`relay_transport.h`](../../net/include/wpiemu/net/relay_transport.h) | ENet6 over UDP, direct or through the relay |

`MatchHost` and the client session see only `sdk::proto::ITransport`, so direct and relay
connections run the same game code. ENet (the `SirLynix/enet6` fork, pinned in
[`WpiEmuDependencies.cmake`](../../cmake/WpiEmuDependencies.cmake)) is private to `net/`; the
fork is used for dual-stack IPv6, and encryption does not depend on it.

ENet channels (the relay's numbering, used for direct connect too): `0` control, `1` game reliable,
`2` game unreliable (sequenced). A direct ENet peer's reliable frames may be 128 KiB and its
unreliable frames 1400 bytes; an unreliable frame above its limit is refused, never silently sent
reliably. ENet's range-coder compression is on (both ends must agree).

### Endpoints

`NetEndpoint::parse` ([`endpoint.h`](../../net/include/wpiemu/net/endpoint.h)) reads
`enet://host:port` (direct; `[::1]` for IPv6 literals) and `relay://host:port/ROOMID`. Parsing never
resolves a name or touches the network. The two apps accept what a person types:
`NetEndpoint::parseRelayAddress` (`host:port` or `relay://host:port`, with the room id taken from the
invite) and `NetEndpoint::parseDirectAddress` (`host`, `host:port` or `enet://host:port`; the port
defaults to `kDefaultDirectPort`, 47300).

## Invite and encryption

### Invite format

The host draws a random 100-bit secret from the OS (`BCryptGenRandom`, `getentropy` or
`getrandom`; there is no weak fallback) once per hosting session and shows it as
`XXXXX-XXXXX-XXXXX-XXXXX`: 20 Crockford base32 characters (`0123456789ABCDEFGHJKMNPQRSTVWXYZ`).
[`parseInvite`](../../net/include/wpiemu/net/invite.h) accepts any case, dashes and spaces anywhere,
and reads `O` as `0` and `I` or `L` as `1`. Three independent values derive from the secret with
keyed BLAKE2b-256, each with its own fixed domain string as the key:

| Value | Derivation | Who sees it |
|---|---|---|
| `roomId` | First 8 base32 characters of the hash keyed `wpiemu-room` | The relay (the public lookup name) |
| `hostAuth` | The 32-byte hash keyed `wpiemu-host` | The relay (proves a host re-registering is the same host) |
| `channelKey` | The 32-byte hash keyed `wpiemu-chan` | Nobody: it keys the handshake |

Knowing `roomId` or `hostAuth` reveals nothing usable about the secret or the other values. An
invite whose `roomId` would equal the reserved health-ping room is redrawn. The secret is shown only
in the host UI (with a Copy button) and is never logged or persisted; the secret type wipes its
memory on destruction. Direct connect uses the same invite plus `host:port`. A low-entropy typed
password is deliberately not supported: a captured handshake would allow offline guessing.

### Handshake

[`SecureTransport`](../../net/include/wpiemu/net/secure_transport.h) wraps each per-peer transport
(an ENet peer or a relay virtual peer), above ENet and above the relay's peer-id envelope. The
handshake is blocking, bounded by one deadline and run by the factories (`connectAsClient`,
`acceptAsHost`), before any application frame is delivered. All integers are little endian.

| Step | Frame | Size |
|---|---|---|
| 1. client to host | `"WPSC"`, version byte `1`, ephemeral X25519 public key | 37 bytes |
| 2. host to client | `"WPSC"`, version byte `1`, ephemeral public key, confirm record | 77 bytes |
| 3. client to host | confirm record | 40 bytes |

Both sides compute `DH = X25519(own ephemeral secret, peer ephemeral public)` and then
`BLAKE2b-512(key = channelKey, message = DH | ephemeralClient | ephemeralHost | roomId)`. The first 32
bytes key client-to-host traffic and the last 32 host-to-client. A confirm record is a normal record
with counter 0 holding `wpiemu-confirm-h` (host) or `wpiemu-confirm-c` (client). A wrong invite, bad
confirmation, wrong magic or version, an all-zero DH result or a frame of another size closes the
inner transport before any game message is parsed. Ephemeral keys give forward secrecy.

### Records

Each application frame is one record on the inner transport:

```text
u64 counter | ciphertext (same length as the plaintext) | 16-byte Poly1305 tag
```

That is 24 bytes of overhead. The cipher is XChaCha20-Poly1305 (Monocypher 4.0.3, a private static
library in `net/`). The 24-byte nonce is the direction byte, 15 zero bytes and the counter. The
associated data is the delivery class and the record kind (application or confirm). Counters are
per direction, start at 1, and the receiver keeps a 2048-entry sliding replay window, because ENet
can reorder unreliable frames against reliable ones. A short frame, bad tag or replayed or too old
counter is dropped and counted in `SecureTransportStats`; it is never thrown and not a disconnect
by itself. A send counter that would wrap closes the transport. A reconnect runs a fresh handshake.

## Messages

All messages are alternatives of `proto::Message`. Numbers 13 to 21 travel inside the encrypted
transport; 22 to 29 are the plaintext relay control messages and travel only on relay channel 0.
Receivers reject a malformed field, an over-long list or string, or unsorted ids (the decoder's
caps are the limits below).

| # | Message | Direction | Delivery | Purpose |
|---|---|---|---|---|
| 13 | `Join` | client to host | reliable | Protocol version, display name, plugin id, WPILib year, resume token, robot spec JSON |
| 14 | `JoinAccept` | host to client | reliable | Participant id, session id, epoch, station, step, rates, start pose, resume token, match state, tick, roster |
| 15 | `JoinReject` | host to client | reliable | Reason and detail; the host then closes |
| 16 | `Roster` | host to all | reliable | Participants with station, ping, loss and `RobotAppearance` |
| 17 | `Leave` | client to host | reliable | The client is leaving |
| 18 | `MatchInputPacket` | client to host | unreliable | The last 3 `MatchInput` records |
| 19 | `MatchSnapshot` | host to client | unreliable | World state built for one client |
| 20 | `MatchSnapshotAck` | client to host | unreliable | The newest snapshot the client applied |
| 21 | `MatchControl` | both | reliable | Reset, Pause, Resume, Kick, ServerClosing, PingRequest, PingReply |
| 22 | `RegisterRoom` | host to relay | reliable | Room id and `hostAuth` |
| 23 | `RoomRegistered` | relay to host | reliable | `resumed`, `maxPeers` |
| 24 | `RoomRejected` | relay to host | reliable | `RoomTaken`, `BadAuth`, `RelayFull`, `BadVersion` |
| 25 | `JoinRoom` | client to relay | reliable | Room id |
| 26 | `JoinResult` | relay to client | reliable | `ok`, failure reason, assigned peer id |
| 27 | `PeerJoined` | relay to host | reliable | A client entered the room |
| 28 | `PeerLeft` | relay to host | reliable | A client left, with a reason |
| 29 | `CloseRoom` | host to relay | reliable | Free the room at once |

Protocol version is `kMatchProtocolVersion` (1); the host accepts the range
`kMinCompatibleMatchProtocolVersion` to `kMatchProtocolVersion`. This is not a promise of
multi-version compatibility (see [Contributing](../CONTRIBUTING.md#sdk-and-configuration-changes)).
The first twelve alternatives are the HALSim extension protocol; the robot process and the local
`SessionRunner` ignore the match alternatives (`isMatchMessage`).

### Run identity

Match frames reuse `proto::classifyFrame` unchanged. `sessionId` is assigned by the host to each
accepted connection (`JoinAccept`) and `runEpoch` is the host's epoch, bumped by every Reset and
resume. A receiver drops frames of an older epoch or another session; a frame of an epoch the host
has not announced is a protocol violation. Frames before `JoinAccept` carry the unspecified value 0.
Pings are answered whatever the epoch so a client can re-sync after a Reset.

### Joining

The host handles one `Join` per poll (decoding and validating JSON must not hitch the simulation).
It checks, in order: the room is not closing, the protocol version, the game plugin id
(`WrongPlugin`) and WPILib year (`WrongSeason`) equal the host's, a resume token (a known one
resumes, a kicked one is `Kicked`), the player cap (`RoomFull`), then the robot description.

The description is JSON of a `RobotConfig` subset (at most 64 KiB). The host decodes it with the
normal `RobotConfigCodec` and `ConfigPipeline` (decode, default, validate), ignores slot and device
issues (a remote robot has no devices), then applies the fairness table
[`kRemoteSpecLimits`](../../sdk/include/wpiemu/sdk/physics/remote_spec_limits.h): mass, footprint,
wheel and motor constants, current limits, tire coefficients and zone envelopes are **clamped** into
plausible FRC ranges and the host logs what it changed. A wheel count outside 2 to 8 or a drivetrain
kind the host's registry does not know is **refused** (`BadSpec`), because the client's wheel
setpoints follow its own wheel list. Because behaviour follows the spec, these ranges are also the
fairness limit; a stronger-than-real robot inside them is allowed.

The host assigns the alliance with fewer robots first, then the lowest free station, and answers
`JoinAccept` with a fresh resume token. A client that drops keeps its participant id, station and
robot for 30 s (`resumeWindow`); a `Join` with its token resumes it with a new session id and a
new token. A connection silent for 5 s is dropped, one that sends no `Join` within 2 s is closed,
and 3 malformed frames or violations disconnect it. Kicking a participant removes its robot and
refuses its resume token for the rest of the run; `JoinRejectReason::Banned`, `BadInvite` and
`BadResumeToken` exist in the enum but the host does not send them today (a wrong invite fails in
the secure handshake, before `Join`).

### Inputs

A `MatchInput` carries commanded **wheel setpoints**, not measured state: per wheel a target wheel
speed (`f32` rad/s) and a steer angle (`i16`, a fraction of a full turn), the zone-flag bitmask,
optional zone pose overrides, a strictly increasing `inputSeq` (from 1; compare with `seqNewer`)
and the host `targetTick` it should apply on. About 60 inputs per second are sent, each packet
repeating the last 3 (oldest first), so a lost datagram costs nothing. The host:

- rejects a wheel count that differs from the accepted spec, sequence 0 and a target more than 1 s
  ahead (counted in the per-robot input stats), and treats a non-finite wheel speed as 0 and clamps
  it to the wheel's limit (`sanitizedWheelSpeed`);
- masks `zoneFlags` with the accepted zone count;
- rate-limits per client with a token bucket at 120 inputs per second (`kMaxInputRateHz`), burst 12;
  pings are limited to 10 per second, burst 5;
- applies the newest queued input whose target tick has come and supersedes older ones, so arrival
  order and wall time cannot change a run; and
- zeroes the setpoints (the robot brakes) after 250 ms of simulated time without an input, and
  ignores inputs while the host Driver Station is disabled, e-stopped or the match is paused.

`inputLeadTicks` in each snapshot tells the client how early or late its newest input arrived, so
it can adapt the lead. Pings (`MatchControl` PingRequest and PingReply) measure the round trip and
seed the client's clock sync with the host's tick and sim time.

### Snapshots

A `MatchSnapshot` is built for **one** client: header (`snapshotSeq` per client from 1, host tick,
sim time), the receiving client's `ackInputSeq` and `ackTick`, `MatchInfo` (state, enabled, mode,
e-stop, match time, phase key), one `RobotWireState` per participant including the host, entity
deltas and game-wide HUD values. A receiver drops a snapshot whose `snapshotSeq` is not newer than
the last applied. Robot poses are quantized as in
[`quantize.h`](../../sdk/include/wpiemu/sdk/match/quantize.h): position in 1 mm `i32`, yaw as an `i16`
angle, world-frame velocity in mm/s and mrad/s as `i16` (saturating, never wrapping), per-wheel steer
and spin angle as `i16`. HUD values are `(keyId, float)` pairs; the key ids come from a table both
sides derive from the game plugin's HUD descriptors
([`hud_keys.h`](../../core/match/include/wpiemu/core/match/hud_keys.h)), so the wire carries no strings.

**Rates.** The simulation runs at 200 Hz. Snapshots go out at 60 Hz on a LAN and 30 Hz on a WAN,
chosen per client from the transport's measured round trip and loss with hysteresis: a client moves
to 30 Hz after two consecutive samples (taken every 500 ms) at or above 60 ms or 30 per mille loss,
and back after six at or below 40 ms and 10 per mille. While the world is not ticking (lobby, paused,
stalled) snapshots go out every 100 ms and the roster every second.

**Size budget.** The host aims each snapshot at `kSnapshotBudgetBytes` = **1200 bytes** of payload.
The reason: a snapshot is an unreliable datagram, and ENet fragments a larger one into pieces of which
losing any drops the whole frame. With the 24-byte record overhead and, through the relay, the 2-byte
envelope, 1200 bytes stays below the relay's 1400-byte datagram cap
(`kRelayMaxDatagram`) and inside one ordinary packet. It is a budget, not a wire limit: the core part
(header, `MatchInfo`, **all** robots, game HUD) is always sent, and only the entity deltas are trimmed
to fit, best change first
([`snapshot_budget.h`](../../core/match/include/wpiemu/core/match/snapshot_budget.h)). If the core
alone is over budget the per-robot HUD lists and then the game HUD are dropped; robots and
`MatchInfo` never are. Changes that did not fit wait for a later snapshot.

### Entity contract

Numerous things (fuel) are entities with stable `u32` ids that survive the match, in at most 8
groups. Each upsert is an `EntityState`: position in centimetres (`i16`), velocity in cm/s and a
`ballistic` flag for free flight. Both sides extrapolate an entity from its last update with the one
integer-only model [`predictEntity`](../../sdk/include/wpiemu/sdk/match/entity_prediction.h): constant
velocity in x and y, a parabola under 981 cm/s² while ballistic, landing at the floor, and holding
still beyond a 2 s horizon. The host sends an upsert only when the client's prediction would be wrong
by more than a threshold (5 cm moving, 1 cm at rest; 15 cm while its velocity is unsettled), when the
client holds nothing for it, or as a keyframe (1 s moving, 4 s at rest). Due changes are ranked by
error and age, nearer to a robot first
([`entity_delta.h`](../../core/match/include/wpiemu/core/match/entity_delta.h)).

Deltas are relative to a baseline: the newest `snapshotSeq` the client acknowledged
(`MatchSnapshotAck`), or `kNoBaseline` for full state. Upserts and removals are strictly ascending
by id (at most 1024 each per group per snapshot). An update sent less than 150 ms ago whose
acknowledgement is pending counts as in flight and is not repeated; the client keeps it ahead of the
older version its baseline holds. The host repeats the removal of every entity it ever upserted and no
longer has until a snapshot that carries it is acknowledged, whatever was lost or reordered. The exact
client bookkeeping (mirror and display maps per snapshot) is specified in
[`snapshot_messages.h`](../../sdk/include/wpiemu/sdk/match/snapshot_messages.h).

### Zone poses

A zone is a client-controlled volume or muzzle in the robot's chassis frame (origin at the centre of
mass, +X forward, +Y left, +Z up). A remote robot simulates no mechanisms, so a client may **move** its
zones. Each input lists, for any subset of its zones, a pose (position in 1 mm `i16`, orientation as
three quantized Euler angles; 13 bytes) and the list is self-contained: a zone without an entry sits
at its spec pose. The host clamps each pose to the frame box grown by 0.75 m in x and y and to 0 to
1.5 m above the floor, ignores non-finite values, and exposes the effective pose to the game plugin
on `RobotView::zonePoses`.

Everyone sees moved zones: each snapshot lists, per robot, the zones whose effective pose differs from
their rest pose (compared after quantization), at most 4 per robot, active zones first and then lowest
indices, as overrides over the rest poses the receiver already has from the roster. The host robot's
own zones are included. It is the latest state, not a delta.

### Roster appearance

A client must draw robots whose specs only the host accepted, so each `RosterEntry` carries a bounded,
host-sanitized `RobotAppearance`
([`robot_appearance.h`](../../sdk/include/wpiemu/sdk/match/robot_appearance.h)): frame box and centre,
at most 8 visual primitives, at most 8 wheels (kind, steered, hub position, radius, width) and at most 16
zones (rest pose, kind key, size, mechanism flag). It is built from the accepted, clamped spec, never
from the client's JSON, and sent reliably in `JoinAccept` and every `Roster`. List order is the spec
order: wheel `i` is what `MatchInput::wheels[i]` commands and `RobotWireState::wheels[i]` reports, and
zone `i` is flag bit `i` and zone pose index `i`. Zone rest poses use `sdk::remoteZoneSpecPose`, the
one rule the host also uses.

### Client behaviour

`MatchClient` ([`match_client.h`](../../core/match/include/wpiemu/core/match/match_client.h)) is the
Qt-free client session. It has no threads of its own: the app (or a test) calls `update()` at
120 Hz or more and binds its UI to the immutable `MatchClientView` it publishes
([`client_view.h`](../../core/match/include/wpiemu/core/match/client_view.h)). Time is an injected
`IWallClock`.

- **Join.** `connect(transport, robot)` encodes the robot with `RobotConfigCodec` and runs the
  host's own intake (`acceptRemoteSpec`: decode, defaults, validation, fairness clamps) before it
  sends anything. A description the host would refuse ends as `Disconnected` with `BadSpec` and no
  Join on the wire; the fields the host will clamp are listed in `view.specClamps`. `JoinAccept`
  carries the station, the run identity, the start pose, a resume token and the roster; `JoinReject`
  ends as `JoinRejected` with the host's `JoinRejectReason` and detail. No answer within 5 s is
  `JoinTimeout`.
- **State.** `ConnectionState` is `Connecting`, `Lobby`, `Live`, `Paused`, `HostStalled` or
  `Disconnected`; a disconnect carries a `DisconnectReason` (`Left`, `JoinRejected`, `JoinTimeout`,
  `BadSpec`, `Kicked`, `ServerClosing`, `TransportLost`, `HostSilent`, `ProtocolError`). A host that
  sends nothing for 3 s is `HostSilent`; `Running` snapshots whose sim time stops advancing for
  0.75 s show `HostStalled`.
- **Clock and input.** The client pings the host (every 120 ms at first, then every 500 ms) and fits
  the host's sim time against its own clock. Each input carries the wheel setpoints, the zone flags
  and any commanded zone poses, stamped with the host tick it should apply on. It is sent at 60 Hz
  (sooner when the command changes) with the last three repeated, and the lead (3 to 120 ticks)
  follows the latency and jitter the host reports.
- **Own robot.** Predicted in a local `PredictionWorld` (the field and its own chassis, with the same
  wheel controller as the host) that runs ahead of the host by the input lead, rewound to the
  authoritative state at `ackTick` on every snapshot and replayed, with the visible error decaying
  over about 100 ms and snapping above about 1 m or 45 degrees. The other robots are kinematic
  boxes in that world, so the own robot pushes against them.
- **Others and fuel.** Remote robots are interpolated in the past by an adaptive delay
  (`clamp(2 x snapshot interval + 3 x measured jitter, 2 x interval, 250 ms)`, so roughly 35 to
  40 ms on a LAN and 60 to 80 ms on a typical WAN) and extrapolated for at most 250 ms
  (`kMaxExtrapolationUs`) before they hold. Entities go through the client entity memory and the
  shared `predictEntity` model.
- **Diagnostics.** `view.diagnostics` has the round trip (smoothed and minimum), jitter, snapshot
  loss and age, the interpolation delay, clock drift, the prediction error of the own robot (latest
  and 95th percentile of the last 128 snapshots), how far the prediction runs ahead, the input lead
  and counters for reconciliations, snaps, late inputs and entity baseline misses.
- **Reconnect.** When the link breaks or the host goes silent and the client holds a resume token,
  `services.reconnect` (the app dials the same endpoint again, handshake included) is retried with a
  backoff of 250 ms doubling to 4 s, at most 8 attempts, each rejoining with the token. After that
  the status is `Disconnected` and `resume(transport)` rejoins by hand. The host keeps the robot for
  its resume window (30 s); later it is a fresh join.

`wpiemu_client` ([`app/client`](../../app/client)) is the app around that session. It adds no
network logic of its own; the pieces are:

| Piece | File | What it does |
|---|---|---|
| Join form | [`join_form.h`](../../app/client/join_form.h) | Validates the invite (any case, dashes optional), an optional direct address (`host`, `host:port`, `enet://host:port`; port 47300 when omitted) and the relay choice into a `JoinPlan`, with a stable key for every wrong field. Production is the default relay; Development and a custom `host:port` are under Advanced. A direct address overrides the relay. |
| Robot file | [`robot_file.h`](../../app/client/robot_file.h) | Reads any `robot.json`, runs the host's own intake on it and reports errors and the fields the host will clamp. |
| Secure link | [`secure_link.h`](../../app/client/secure_link.h) | Opens `EnetClientTransport` or `RelayClientTransport` (the room id comes from the invite) and wraps it in the `SecureTransport` client handshake keyed by the invite. The relay and the path see only opaque bytes. |
| Session | [`client_session.h`](../../app/client/client_session.h) | Runs `MatchClient` on a worker thread (so dialling, the handshake and reconnects never block the window), owns the resume and reconnect hooks and publishes `SessionState`. |
| State machine | [`client_state.h`](../../app/client/client_state.h) | Turns the link phase and `ConnectionState` into the screen (join form until the host accepts, then the drive screen), the banner and the buttons. |
| Input | [`input_mapping.h`](../../app/client/input_mapping.h) | Joystick to chassis speeds (deadband, scale, optional field-relative), then `MatchClient::driveRobotRelative` or `driveFieldRelative` applies the shared kinematics. Button `i` holds zone `i`; the d-pad, right stick and triggers move the selected zone inside the host's envelope. |
| Scene feed | [`scene_feed.h`](../../app/client/scene_feed.h) | Adapts the `MatchClientView` to the run view's `ISimSnapshot`, `HostScene` and roster, so the client draws with the same rigs and HUD code as the desktop app. |
| Settings | [`client_settings.h`](../../app/client/client_settings.h) | Name, direct address, relay choice, last robot file, drive tuning and the stats overlay, through the app's key-value port. The invite is never stored. |

The wire carries no look for entity groups, so every group is drawn as the same small yellow
sphere (`kEntityRadiusM`, `kEntityColor`). Text HUD values other than the match phase are not on
the wire either (see the snapshot fields above), so a scoreboard's status lines stay empty. The
user-facing join steps are in the [user guide](../USER_GUIDE.md#join-a-practice-match). Tests:
`tests/app_client` has the model tests, the window tests under the offscreen platform and an
in-process end to end over a real `MatchHosting` with encryption on, including a dropped link that
resumes. `wpiemu_client --demo [--scenario NAME] [--screenshot FILE]` runs the window against a
scripted match (nothing leaves the process).

## Limits and budgets

| Limit | Value | Where |
|---|---|---|
| Robots per match, host included | 6 (`kMaxRobotsPerMatch`); the host UI's player count is clamped to it | `match_protocol.h` |
| Input redundancy | 3 inputs per packet | `kInputRedundancy` |
| Input rate | 60 Hz default, host limit 120 Hz | `kDefaultInputRateHz`, `kMaxInputRateHz` |
| Snapshot rate | 60 Hz LAN, 30 Hz WAN | `kSnapshotRateLanHz`, `kSnapshotRateWanHz` |
| Snapshot payload budget | 1200 bytes | `kSnapshotBudgetBytes` |
| Encryption overhead | 24 bytes per record | `SecureTransport::kRecordOverhead` |
| Wheels, zones per robot | 8, 16 | `kMaxWheelsPerRobot`, `kMaxZonesPerRobot` |
| Appearance primitives; moved zones per snapshot robot | 8; 4 | `kMaxAppearancePrimitives`, `kMaxSnapshotZonePosesPerRobot` |
| Names and plugin ids; free-text details | 64; 256 bytes | `kMaxNameBytes`, `kMaxDetailBytes` |
| Robot spec JSON | 64 KiB | `kMaxRobotSpecBytes` |
| HUD values per robot; game HUD | 8; 16 | `kMaxHudValuesPerRobot`, `kMaxGameHudValues` |
| Entity groups; entries per group per snapshot | 8; 1024 | `kMaxEntityGroups`, `kMaxEntityDeltaEntries` |
| Input timeout (brake) | 250 ms of simulated time | `kInputTimeoutUs` |
| Max extrapolation; max interpolation delay | 250 ms; 250 ms | `kMaxExtrapolationUs`, `kMaxInterpolationDelayUs` |
| Resume token | 16 bytes, valid 30 s after a drop | `kResumeTokenBytes`, `MatchHostConfig::resumeWindow` |
| Connections the host table holds (joined or not) | 9 by default (robots per match plus 3) | `MatchHostConfig::maxConnections` |

## Relay protocol

The relay ([`relay/`](../../relay/include/wpiemu/relay/relay_server.h)) lets a host and its clients
meet without port forwarding. ENet cannot sit behind a transparent UDP forwarder, so the relay speaks
ENet too: the host and every client connect **outbound** to it and the relay forwards between them.
It never parses game frames, so match protocol changes need no relay redeploy. The constants are in
[`relay_messages.h`](../../sdk/include/wpiemu/sdk/match/relay_messages.h).

**Channels.** `0` control (reliable): only the plaintext messages 22 to 29. `1` game reliable and
`2` game unreliable-sequenced: opaque encrypted frames, forwarded on the delivery class they arrived
on. No game frame ever travels on channel 0.

**Envelope.** On the game channels a client sends its frames unprefixed. The relay prepends the
sender's peer id (`u16` little endian) to each client frame before handing it to the host. The host
prefixes each frame with the target peer id, which the relay strips; `0xFFFF` (`kRelayBroadcastPeer`)
asks the relay to fan one frame out to every client. The host is peer 0 and clients get ids from 1.
`RelayHostTransport` and `RelayClientTransport` add and remove the envelope, so the game layer sees
ordinary per-peer transports. The host transport does not use broadcast today, because each peer's
`SecureTransport` encrypts with its own key.

**Flow.**

1. The host derives `roomId` and `hostAuth` from the invite and sends `RegisterRoom(version, roomId,
   hostAuth)`. The relay answers `RoomRegistered(resumed, maxPeers)` or `RoomRejected`:
   `RoomTaken` (another live host owns the id; the host draws a new invite), `BadAuth` (the room
   exists, its host is away and the `hostAuth` differs), `RelayFull` or `BadVersion`.
2. A client sends `JoinRoom(version, roomId)` and gets `JoinResult(ok, reason, peerId)`. Failures:
   `NoSuchRoom`, `RoomFull`, `BadVersion`, `LockedOut` (too many failed joins from its address) and
   `HostAbsent` (the room exists but its host is not connected).
3. The relay sends the host `PeerJoined(peerId)` and later `PeerLeft(peerId, reason)` with
   `Disconnected`, `TimedOut`, `RateLimited` or `Oversized`. The host transport turns each into a
   virtual peer; the encrypted handshake then runs over it.
4. A clean host shutdown sends `CloseRoom` on the connection that registered, which frees the room
   and disconnects its clients at once. From any other connection it is a control violation.

**Reconnect.** A host that loses the relay re-registers the same room with exponential backoff
(250 ms doubling to 4 s) for up to twice the room TTL. The relay keeps a room without its host for
30 s (`kRelayRoomTtlWithoutHostMs`); clients stay connected but their frames are dropped. Registering
again with the same `hostAuth` resumes the room, and the relay announces the clients still there with
`PeerJoined`; each gets a **new** virtual peer and the encryption handshake runs again. The same
`hostAuth` from a new connection while the old one still looks live takes the room over. A different
`hostAuth` is `RoomTaken` while the host is live and `BadAuth` while it is away.

**Reserved room.** `00000000` is the health-ping room: it never exists, `RegisterRoom` for it is
refused and invites never derive it. A `JoinRoom` for it is answered `NoSuchRoom` and does not count
toward the lockout, so liveness probes can run as often as they like.

**Relay limits** (defaults; the container sets them with `WPIEMU_RELAY_*`, see the
[relay guide](../guides/relay-azure.md#configure-and-operate)):

| Limit | Default |
|---|---|
| Clients per room | 6 (`kRelayMaxPeersPerRoom`; the host app still enforces its own player count) |
| Rooms | 100 (configurable up to 500) |
| Datagram (unreliable game frame as a client sends it; a host frame also counts its 2-byte envelope) | 1400 bytes |
| Reliable packet (a `Join` with a large spec) | 128 KiB |
| Per-client token bucket | 64 KiB/s with 2 s of burst; the host's is 6 times larger; each room is capped at twice the clients' total |
| Abuse disconnect | 512 rate or size drops within 1 s |
| Failed joins or registrations from one source | 10 per 60 s, then 300 s of lockout (loopback exempt) |
| Unregistered connection | closed after 10 s or 3 refused attempts |
| Room without a host | 30 s |

The relay runs a single-threaded event loop, logs room events with the room id and peer address and
periodic counters but never frame contents, and holds no secrets. Its liveness check, `wpiemu_relay --selfcheck`, joins the ping room and expects a
refusal.

## Trust and threat model

**Protected.** The relay and anyone on the network path cannot read or forge game data: it is
authenticated and encrypted end to end between host and client with keys only the invite holders can
derive (confidentiality, integrity, replay resistance, forward secrecy). A wrong invite fails the
handshake before any game message is parsed, so a stranger cannot inject input or snapshots. Hostile
input is bounded everywhere: the codec's list and string caps, a clamped robot spec, a 64 KiB spec
limit, per-client rate limits, strike counts and bounded outbound queues.

**Visible to the relay and the path.** IP addresses, packet sizes and timing, the room id and
`hostAuth`, and the plaintext control messages (which carry no game data). They can also drop or delay
packets.

**Not protected.**

- **The host is trusted.** The host simulates everything: a malicious or buggy host can cheat, crash
  the match, and sees each client's robot description, inputs and connection address. Clients cannot
  verify the simulation.
- **The invite is the only credential.** Anyone who holds it is a participant: they can join, see
  every robot, snapshot and roster entry, and there is no identity or PKI. If it leaks, stop hosting
  and start again for a new invite. Kicking removes a robot and refuses that participant's resume
  token for the run, but a client with the invite can join again as a new participant.
- **A stranger can occupy relay slots.** Joining a room needs only its 40-bit `roomId`, not the
  secret, and a virtual peer the host closes after a failed handshake is not removed from the relay
  until that client leaves, so someone who learns or guesses a room id can fill its 6 client slots
  and make joins fail. The per-source lockout slows guessing; it does not stop it.
- **A deleted relay's name can be claimed.** The relay's DNS label uses the `Unsecure` reuse policy so
  the name stays predictable, which means anyone can register the same label after the relay is
  deleted. A client that dials the old name would reach a stranger's relay, which sees only the room id
  and ciphertext (it cannot read or forge game data) but can refuse or stall the match. Retire a relay
  by changing the name the apps use, not by deleting it quietly.
- **Availability.** Relay or network failure ends the match; one relay instance has no redundancy and
  an update restarts it. The host can be stalled by its own robot.
- **Blocked UDP.** Both direct connect and the relay need outbound UDP. A school or corporate network
  that blocks it cannot join, and version 1 has no TCP fallback. The relay's UDP port is a deployment
  parameter, and port 443 is often open on networks that block other UDP ports.
- **Fairness is a rules decision.** The clamps keep specs plausible, not equal.
- **Remote robots are simplified.** No battery sag or brownout, no mechanisms and no mechanism mass
  shifts; behaviour follows the client's spec within the clamps.

## Tests

The protocol and its hostile-input tests live next to their owners: codec tests in `tests/ipc`,
match host and entity tests in `tests/match`, transport, handshake and relay-transport tests in
`net/tests`, and relay rooms, server, probe and stack tests in `relay/tests`. The secure transport
is covered by known-answer, tamper, replay, reorder, wrong-secret and truncated-handshake cases, and
a relay test double asserts that no plaintext marker crosses the relay. See
[Building](../BUILDING.md#run-tests-and-repository-checks).

Next: [deploy and operate the relay](../guides/relay-azure.md).
