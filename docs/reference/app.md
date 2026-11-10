# Application reference

**Read this when:** changing how the app composes the editor and run view, connects them to a
runtime host, presents snapshots, or offers updates.
**Prerequisites:** [Architecture](../ARCHITECTURE.md) for module and thread ownership;
[Contributing](../CONTRIBUTING.md) for UI conventions.

This page covers app-level contracts, not widget or service implementation details. Use the
[User guide](../USER_GUIDE.md) for controls and settings, [Building](../BUILDING.md) for
build and test commands, and [Extending](../extending.md) for feature recipes.
Configuration and robot placement belong to [Runtime](runtime.md); process communication
belongs to [Extension and IPC](extension-ipc.md).
For the app's Qt/UI ownership model, see [Qt in this project](../concepts/qt-in-this-project.md).

## Composition and runtime boundary

[`AppServices`](../../app/common/app_services.h) is the composition root. It creates the
shared registries and project services, then supplies one runtime host to the editor and run
view. Built-in games are reached through the plugin aggregator, not direct season dependencies.

The editor and run view share one [`IRuntimeHost`](../../app/contract/runtime_host.h).
That host exposes project presentation data, runtime status, snapshots and commands without
letting either page access mutable simulation state. Project selection is a separate
`IProjectBinding` responsibility owned by the shell and composed by `AppServices`.

The real host delegates process and simulation orchestration to Qt-free `RobotSession`;
Qt event-loop adaptation remains in the app. `--demo` selects a scripted host and does not
exercise robot launch or IPC. Keep these host boundaries intact when adding UI behavior.

Setup checks the extension selected for the live project season, listing available seasons
when it is missing (`no_extension_for_season`). A refused year handshake reports
`season_mismatch`. Native loader or initialization failures for `halsim_wpiemu` report
`extension_load_failed`, retaining the loader error in the console. These stable status keys
are translated into setup guidance by the run view; extension selection belongs to
[Runtime](runtime.md#launcher-boundary).

## Hosting a practice match

`host::Options::hosting` (mode `Off`, `Direct` or `Relay`, port, relay address, player count) makes
`RobotSession` compose a [`MatchHosting`](../../app/host/match_hosting.h) next to the robot link;
with mode `Off` nothing is created and the session is exactly as before. The robot-process IPC path is
not touched. `MatchHosting` draws the invite secret (the only place it is shown is
`HostingStatus::invite`, never logged or persisted), opens the listener (ENet, or a registered relay
room), wraps **every** accepted peer in `SecureTransport` on a short-lived handshake thread before
`MatchHost` sees it, and runs as the runner's tick observer on the sim thread. While clients are
connected the UI's `RobotSession::controller()` holds the time scale at 1x and drops Step
(`ConstrainedController`; the scale the user asked for returns afterwards). The options are read when
the robot is launched, so changing them applies at the next launch.

The Qt side is [`HostingService`](../../app/common/hosting_service.h) (`AppServices::hosting()`): it
owns the persisted options and hands the UI an immutable `HostingStatus` snapshot (phase
Off, Starting, Lobby, Live, Paused, HostStalled or Error, invite, endpoint, roster with ping and loss,
constraint flags), and relays kicks. `RealRuntimeHost` publishes the running session's status to it.
Shutdown closes the room (the relay frees it with `CloseRoom`), tells clients `ServerClosing` and
removes their robots from the session. The relay and the match protocol are described in
[Match network](match-network.md); the simulation side is in
[Runtime](runtime.md#remote-robots-practice-matches).

### The Practice match panel

The shell docks [`HostPanel`](../../app/shell/host_panel.h) on the right (View > Practice match; the scripted `--demo`
host cannot host, so it does not offer it). All logic is in
[`HostPanelModel`](../../app/shell/host_panel_model.h), which reads the immutable `HostingStatus` and edits options only
through `HostingService::setOptions`:

- **Settings**: mode (Off, Direct, Relay), UDP port (Direct), player count, and under *Advanced* the relay choice
  (Production by default, Development, or a custom `host:port`) and the Direct listen address. The panel keeps what it does
  not edit (the phase key). An edit that would make `validateHosting` fail is **not saved**: the controls keep it and a
  banner says why, so a typo can never stop the robot from launching.
- **Launch hint**: settings are read when the robot launches. Saved but not running shows "starts the next time the robot
  program is launched"; a running session whose options differ (`restartRequired()`) says to stop and run again.
- **Phase banner**: Starting, Lobby, Live, Paused, Host stalled and Error, with the host's `detail` line when it has one.
- **Invite**: the only place the secret is shown, with a Copy button (it is never stored).
- **Roster**: the host first, then each player with alliance colour and station, ping, loss (a dropped player shows
  Disconnected) and a Kick button (`HostingService::kick`, never for the host).

The relay addresses are presets built by [`relay_endpoints.h`](../../app/common/relay_endpoints.h) from the Bicep naming
(`wpiemu-relay[-dev]<suffix>.southcentralus.azurecontainer.io:7777`). The DNS suffix of a deployment is a setting, in this
order: the `WPIEMU_RELAY_DNS_SUFFIX` environment variable (the one `infra/relay/*.bicepparam` reads), the stored
`hosting/relayDnsSuffix` preference, then the value a release build bakes in with the CMake definition
`WPIEMU_RELAY_DNS_SUFFIX`. Without any, the presets are empty and the panel asks for a custom address.

The game's match-phase key (`HostingOptions::phaseHudKey`) is the text value its scoreboard shows in the centre
(`phaseHudKeyOf(HostScene::hud)`); `RealRuntimeHost` stores it with the options when a project opens and at each launch.

### Remote robots in the run view

`IRuntimeHost::matchRoster()` (with `matchRosterChanged`) hands the run view the remote participants of a hosted match:
`contract/match_roster.h`, SDK types only (id, name, station, `RobotAppearance`). `RealRuntimeHost` derives it from the
hosting status; the run view never sees the host or core. Where each robot is arrives in the snapshot the run view already
consumes (`ISimSnapshot::remoteRobots()`, one `RobotSample` per remote participant, matched by participant id); there is no
second snapshot path.

[`RemoteRigSet`](../../app/runview/remote_rigs.h) builds one `RobotRig` and one `ZoneRig` per participant whose appearance
is known (`robotConfigOfAppearance`: frame box and primitives, wheels, zones at their rest poses), so a remote robot is
drawn by the code that draws the local one. Wheels and zones are paired with the sample **by position**, not by name (the
client's spec names are its own). A zone moves to the pose its `ZoneState::poseInChassis` reports and lights while active;
zone styling comes from the game's zone kinds, and a zone of an unknown kind is not drawn. `RemoteRobotModel` exposes the
rigs to QML (`scene.remoteRobots`): rows are added and removed as the roster changes, a robot the latest snapshot does not
describe is hidden, each row carries nested part models, an alliance-coloured plate on the frame and a label position. QML
places each robot in the field frame inside `ZUpWorld` (the one frame conversion) and draws name labels as projected
overlays. `wpiemu_runview_demo --scenario match` shows three scripted remote robots. Offscreen tests cover the models and
that the QML loads; they do not validate Quick 3D rendering.

## Editor and run-view contracts

The editor works through `IEditorBackend` and an undoable document; it does not access core,
IPC or concrete game plugins. Edits are submitted to the backend, and saving is a separate
request. New edit surfaces must use the document's undo/redo path rather than mutating
runtime state or writing files directly. Before changing project identity or closing, the app
flushes pending edits to their original project; a failed save blocks the transition.
Metadata-driven property behavior and its recipe are in
[Editor properties](../guides/editor-properties.md).

The run view consumes `IRuntimeHost` and immutable scene descriptions. A latest snapshot can
be absent before the first tick, and the UI may skip intermediate snapshots. Repaint rate is
not simulation time. Host calls and delivered signals are UI-thread operations; hosts marshal
worker-thread results before delivering them to the views.

The Run-view camera is presentation state: Driver POV reads the latest snapshot's chassis pose
and smooths its gaze using elapsed UI-frame time, including while simulation is paused. Camera
presets never write simulation inputs or change the Driver Station alliance or station. The
viewport's Driver POV is distinct from simulated robot cameras, which publish vision results.

Launching robot code and starting simulation are separate operations. A live robot can stay
connected for device discovery; beginning and ending simulation enter and leave lock-step
without restarting the robot. Resetting simulation does not rerun robot initialization.
See [Runtime](runtime.md) for reset, configuration and start-placement behavior, and
[Extension and IPC](extension-ipc.md) for lock-step readiness and message ordering.

## Updates

Installed copies check for a newer stable release at startup when update checks are enabled
in Settings. The user can also request **Help > Check for updates**; only this manual check
shows a result message for success, unsupported installs or failure. Demo mode and development
builds do not offer updates. A portable install opens the release page instead of downloading
an installer.

For a supported installer, the app offers a download only when the release has an asset and
checksum for the current platform. It verifies the downloaded file against that checksum
before making installation available; a failed download or verification is discarded and can
be retried. The user can cancel a download or skip that release.

Installing a verified update asks the app to close where installation replaces the running
app. If a simulation is running, the user must confirm; unsaved edits are flushed before
closing, and a refused close leaves the update ready to install later. On macOS, installation
opens the verified disk image while the app stays open. The Help menu mirrors the button's
primary update action; the button's context menu offers release notes, skip and, during a
download, cancel. On macOS the Help menu is the primary action when the native menu bar has
no corner button.

**Next:** [Runtime and configuration](runtime.md).
